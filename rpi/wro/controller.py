# -*- coding: utf-8 -*-
"""
Estrategia y control del vehiculo.

Arquitectura del control
------------------------
El rumbo lo lleva el GIROSCOPIO y la posicion lateral la lleva la VISION:

    rumbo_deseado = rumbo_objetivo(multiplo de 90) + correccion_lateral
    direccion     = k_heading * (rumbo_deseado - rumbo_actual)

Ventajas frente a un PID puro sobre la distancia al muro:
  * si la vision falla un fotograma, el robot sigue recto en vez de dar un
    volantazo;
  * las curvas son exactas (se suma 90 grados al rumbo objetivo) y no dependen
    de cuanto tiempo este el volante girado;
  * la correccion lateral se satura, asi que un error grande de vision no
    puede tumbar la trayectoria.

Ademas el angulo del muro medido por vision corrige lentamente la deriva del
giroscopio (fusion complementaria), que es lo que hace que la tercera vuelta
salga igual que la primera.

Convenios: direccion (+) = IZQUIERDA, rumbo (+) = IZQUIERDA,
           sentido +1 = ANTIHORARIO (giros a izquierda, muro interior a la
           izquierda), sentido -1 = HORARIO.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from .perception import Scene, WallFit

CCW = +1     # antihorario: giros a la izquierda, muro interior a la izquierda
CW = -1      # horario:     giros a la derecha,  muro interior a la derecha

ST_IDLE = "IDLE"
ST_RUN = "SIGUE_MURO"
ST_TURN = "GIRO"
ST_FINISH = "FINAL"
ST_DONE = "PARADO"
ST_RECOVER = "RECUPERANDO"


def wrap180(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


@dataclass
class Filtered:
    """Suavizado exponencial con memoria de validez."""
    value: float = 0.0
    valid: bool = False

    def push(self, v: Optional[float], alpha: float):
        if v is None:
            return
        if not self.valid:
            self.value, self.valid = float(v), True
        else:
            self.value += (float(v) - self.value) * alpha

    def clear(self):
        self.valid = False


class Controller:
    def __init__(self, cfg, clock=None):
        self.cfg = cfg
        # Reloj inyectable: permite ejecutar el mismo control en el simulador
        # con tiempo virtual (ver rpi/tools/simulador.py).
        self.now = clock if clock is not None else time.time
        self.reset()

    # ------------------------------------------------------------------ ciclo
    def reset(self):
        self.state = ST_IDLE
        self.direction = 0                 # 0 = aun desconocido
        self.dir_votes = 0                 # >0 antihorario, <0 horario
        self.dir_source = "-"
        self.target_yaw = 0.0
        self.yaw_bias = 0.0                # correccion vision -> giroscopio
        self.corners = 0
        self.laps = 0
        self.t_start = 0.0
        self.t_end = 0.0
        self.t_state = 0.0
        self.t_last_corner = -99.0
        self.steer = 0.0
        self.speed = 0.0
        self.note = ""
        self.d_inner = Filtered()
        self.d_outer = Filtered()
        self.a_inner = Filtered()
        self.d_left = Filtered()
        self.d_right = Filtered()
        self.front = Filtered()
        self.inner_end: Optional[float] = None
        self.corner_armed = False   # el muro interior se acaba delante
        self.finishing = False      # el giro en curso es el ultimo
        self.corridor: Optional[float] = None
        self.lat_err = 0.0
        self.head_corr = 0.0
        self.head_err = 0.0
        self.target_lat = 0.0
        self._stuck_t = 0.0
        self._stuck_ref = {}
        self._recover_until = 0.0
        self._line_seq0 = None
        self.pillar = None

    def start(self, yaw_now: float, line_seq: int):
        self.reset()
        self.state = ST_RUN
        self.t_start = self.now()
        self.t_end = 0.0
        self.t_state = self.t_start
        self.target_yaw = 0.0
        self._line_seq0 = line_seq
        if self.cfg.direction_source == "cw":
            self._set_direction(CW, "forzado")
        elif self.cfg.direction_source == "ccw":
            self._set_direction(CCW, "forzado")

    def stop(self):
        # Se congela el cronometro: si no, sigue corriendo despues de acabar la
        # ronda y en el panel aparecen tiempos de varios minutos que confunden.
        self.t_end = self.now()
        self.state = ST_DONE
        self.steer = 0.0
        self.speed = 0.0

    def _goto(self, st: str):
        self.state = st
        self.t_state = self.now()

    def _set_direction(self, d: int, src: str):
        if self.direction == 0:
            self.direction = d
            self.dir_source = src

    # ------------------------------------------------------------- deteccion
    def _update_direction(self, sc: Scene, tel):
        if self.direction != 0:
            return
        cfg = self.cfg

        # (a) Sensor de color: la primera linea define el sentido.
        #     Azul primero  -> antihorario ; Naranja primero -> horario.
        if bool(cfg.use_color_lines) and self._line_seq0 is not None:
            if tel.seq > self._line_seq0 and tel.last_event in (1, 2):
                self._set_direction(CCW if tel.last_event == 2 else CW, "color")
                return

        # (b) Vision: el muro que TERMINA en una esquina convexa cercana es el
        #     interior, porque el exterior sigue de largo hasta la esquina
        #     lejana (que ademas es concava y no produce salto).
        lim = float(cfg.wall_end_max_mm)
        le = sc.left.end_mm if (sc.left and sc.left.end_mm is not None) else None
        re = sc.right.end_mm if (sc.right and sc.right.end_mm is not None) else None
        vote = 0
        if le is not None and le < lim and (re is None or re > le + 300.0):
            vote = +1
        elif re is not None and re < lim and (le is None or le > re + 300.0):
            vote = -1
        self.dir_votes += vote
        if abs(self.dir_votes) >= 6:
            self._set_direction(CCW if self.dir_votes > 0 else CW, "vision")

    # ---------------------------------------------------------------- lectura
    def _ingest(self, sc: Scene):
        a = float(self.cfg.smooth_alpha)
        self.d_left.push(sc.left.dist_mm if sc.left else None, a)
        self.d_right.push(sc.right.dist_mm if sc.right else None, a)
        if sc.left is None:
            self.d_left.clear()
        if sc.right is None:
            self.d_right.clear()
        self.front.push(sc.front_mm, a)

        inner = outer = None
        if self.direction == CCW:
            inner, outer = sc.left, sc.right
        elif self.direction == CW:
            inner, outer = sc.right, sc.left

        if inner is not None:
            self.d_inner.push(inner.dist_mm, a)
            self.a_inner.push(inner.angle_deg, a)
            self.inner_end = inner.end_mm
        else:
            self.d_inner.clear()
            self.a_inner.clear()
            self.inner_end = None
        if outer is not None:
            self.d_outer.push(outer.dist_mm, a)
        else:
            self.d_outer.clear()

        self.corner_armed = (self.inner_end is not None and
                             self.inner_end < float(self.cfg.corner_arm_mm))
        self.corridor = sc.corridor_mm
        self._inner_fit = inner
        self._outer_fit = outer

    # ------------------------------------------------------------- objetivos
    def _target_distance(self) -> float:
        """Distancia deseada al muro interior segun el modo y el ancho medido."""
        cfg = self.cfg
        t = float(cfg.target_inner_mm)
        w = self.corridor
        if cfg.wall_mode == "center":
            return (w * 0.5) if w else t
        if cfg.wall_mode == "adaptive" and w:
            # Pasillo estrecho (600 mm): ir centrado. Ancho (1000 mm): por dentro.
            return min(t, max(w * 0.5, float(cfg.outer_min_mm)))
        if w:
            # Nunca pedir una trayectoria que acerque demasiado al exterior.
            return min(t, max(w - float(cfg.outer_min_mm), 150.0))
        return t

    def _lateral_correction(self) -> float:
        """Correccion de rumbo (grados) pedida por el control lateral."""
        cfg = self.cfg
        d = self.direction
        corr = 0.0
        self.lat_err = 0.0
        self.target_lat = self._target_distance()

        if d != 0 and self.d_inner.valid:
            err = self.d_inner.value - self.target_lat        # + = demasiado lejos
            err = max(-float(cfg.lat_err_max_mm), min(float(cfg.lat_err_max_mm), err))
            self.lat_err = err
            corr = float(cfg.k_lateral) * d * err
        elif self.d_left.valid and self.d_right.valid:
            err = 0.5 * (self.d_left.value - self.d_right.value)
            err = max(-float(cfg.lat_err_max_mm), min(float(cfg.lat_err_max_mm), err))
            self.lat_err = err
            corr = float(cfg.k_lateral) * err
        elif d != 0 and self.d_outer.valid:
            # Sabemos el sentido pero el muro interior queda FUERA del campo de
            # vision (pasa al arrancar lejos del interior: para ver un muro a
            # Y mm hace falta estar a X > Y/tan(fov/2) de el). Nos acercamos
            # despacio al lado interior hasta que aparezca, pero solo si hay
            # sitio de sobra respecto al muro exterior.
            if self.d_outer.value > self.target_lat + 220.0:
                corr = d * float(cfg.seek_inner_deg)
                self.lat_err = 0.0

        # Empujon de seguridad para no rozar el muro exterior (regla 9.18).
        if d != 0 and self.d_outer.valid:
            margin = float(cfg.outer_min_mm) - self.d_outer.value
            if margin > 0:
                corr += float(cfg.k_lateral) * d * margin * 1.4

        # Curva "armada": el muro interior se acaba justo delante. Se atenua la
        # correccion lateral para no seguir tirando hacia un muro que termina
        # (y cuyo ajuste, ademas, se apoya en cada vez menos puntos).
        if self.corner_armed:
            corr *= 0.25

        lim = float(cfg.lat_head_max_deg)
        return max(-lim, min(lim, corr))

    def _obstacle_correction(self, pillar, base_corr: float) -> float:
        """Mezcla la correccion de muro con la de esquiva de pilar."""
        cfg = self.cfg
        if pillar is None:
            return base_corr
        off = float(cfg.pillar_pass_offset_mm)
        # Rojo -> pasar por su derecha (trayectoria a Y menor que el pilar)
        # Verde -> pasar por su izquierda (trayectoria a Y mayor)
        want_y = pillar.y_mm - off if pillar.color == "red" else pillar.y_mm + off
        x = max(220.0, pillar.x_mm)
        ang = math.degrees(math.atan2(want_y, x))

        react = float(cfg.pillar_react_mm)
        w = (react - pillar.x_mm) / max(1.0, react - 250.0)
        w = max(0.0, min(1.0, w))
        lim = float(cfg.lat_head_max_deg) * 1.5
        ang = max(-lim, min(lim, ang))
        return (1.0 - w) * base_corr + w * ang

    # ------------------------------------------------------------- deteccion
    def _corner_ahead(self) -> bool:
        """
        Decide si toca empezar la curva.

        Hay dos disparos y los dos son geometricos:

        (1) FRENTE (disparo principal). El muro que tenemos delante SERA, tras
            el giro, el muro EXTERIOR del siguiente pasillo. Un giro de 90
            grados con radio R adelanta al robot exactamente R, asi que si se
            empieza cuando el frente esta a (R + holgura) se termina a esa
            holgura del nuevo muro exterior. Dos propiedades muy utiles:
              - no depende del ancho del siguiente carril (600 o 1000 mm),
              - no depende de por donde del carril veniamos: cada curva
                RECOLOCA lateralmente al robot.
            Con R ~ 300 mm y holgura ~ 380 mm salen unos 680 mm.

        (2) FRENTE CRITICO. Red de seguridad si (1) no llego a saltar.

        Ojo con el "fin del muro interior": NO sirve como disparo. La geometria
        dice que para bordear la esquina hay que empezar el giro cuando la
        esquina esta a (R - distancia_objetivo) por delante, que con R ~ 300 y
        objetivo 340 es NEGATIVO: hay que pasarla antes de girar. Usarlo como
        disparo hace que el robot corte contra el muro interior. Aqui se usa
        solo para (a) deducir el sentido de marcha y (b) "armar" la curva, que
        congela la correccion lateral para no meterse contra un muro que se
        acaba.
        """
        cfg = self.cfg
        if (self.now() - self.t_last_corner) < float(cfg.corner_cooldown_s):
            return False
        front = self.front.value if self.front.valid else 9999.0

        if front < float(cfg.turn_hard_front_mm):
            self.note = "giro: frente critico (%.0f mm)" % front
            return True
        if front < float(cfg.turn_trigger_front_mm):
            self.note = "giro: frente a %.0f mm" % front
            return True
        return False

    def _check_stuck(self, dt: float, yaw: float) -> bool:
        """
        Detecta que el robot no avanza. Sin encoders hay que deducirlo de lo
        que si medimos, y ninguna senal sirve por si sola:

          * la distancia al FRENTE se satura en recta (no hay muro dentro del
            alcance), asi que "no cambia" aunque el robot avance a tope;
          * el RUMBO no cambia en recta aunque se avance;
          * las distancias LATERALES no cambian si el seguimiento es perfecto;
          * el FIN DEL MURO INTERIOR si baja de forma continua al avanzar por
            un tramo recto: es el mejor sustituto de odometria que tenemos.

        Se declara atasco solo si NINGUNA de las senales disponibles se mueve.
        Si no hay ninguna senal utilizable, no se declara nada (a favor de
        seguir conduciendo).
        """
        cfg = self.cfg
        if self.speed < float(cfg.min_speed) * 0.5:
            self._stuck_t = 0.0
            self._stuck_ref = {}
            return False

        thr = float(cfg.stuck_front_delta_mm)
        feats = {}
        if self.front.valid and self.front.value < float(cfg.roi_x_max_mm) * 0.95:
            feats["front"] = (self.front.value, thr)
        if self.inner_end is not None:
            feats["end"] = (self.inner_end, thr)
        if self.d_inner.valid:
            feats["din"] = (self.d_inner.value, thr)
        if self.d_outer.valid:
            feats["dout"] = (self.d_outer.value, thr)
        feats["yaw"] = (yaw, 3.0)

        if not isinstance(self._stuck_ref, dict) or not self._stuck_ref:
            self._stuck_ref = {k: v[0] for k, v in feats.items()}
            self._stuck_t = 0.0
            return False

        moved = False
        for k, (v, tol) in feats.items():
            ref = self._stuck_ref.get(k)
            if ref is None or abs(v - ref) > tol:
                moved = True
                break
        if moved or len(feats) <= 1:
            self._stuck_ref = {k: v[0] for k, v in feats.items()}
            self._stuck_t = 0.0
            return False

        self._stuck_t += dt
        return self._stuck_t > float(cfg.stuck_time_s)

    # ------------------------------------------------------------------ yaw
    def _yaw(self, tel) -> float:
        y = -tel.yaw if bool(self.cfg.yaw_invert) else tel.yaw
        return y + self.yaw_bias

    def _fuse_yaw(self, yaw_raw: float, dt: float):
        """Corrige la deriva del giroscopio con el angulo del muro."""
        g = float(self.cfg.yaw_vision_gain)
        if g <= 0 or self.state != ST_RUN or self.direction == 0:
            return
        fit = self._inner_fit
        if fit is None or fit.quality < 0.45 or abs(fit.angle_deg) > 25.0:
            return
        # El muro es paralelo al pasillo: rumbo implicado por la vision.
        yaw_vision = self.target_yaw - fit.angle_deg
        err = wrap180(yaw_vision - yaw_raw)
        step = g * err * dt
        step = max(-3.0 * dt, min(3.0 * dt, step))       # como mucho 3 deg/s
        self.yaw_bias = max(-30.0, min(30.0, self.yaw_bias + step))

    # =====================================================================
    #  Lazo principal
    # =====================================================================
    def update(self, sc: Scene, tel, dt: float, pillar=None):
        cfg = self.cfg
        now = self.now()
        self.pillar = pillar

        if self.state in (ST_IDLE, ST_DONE):
            self.steer, self.speed = 0.0, 0.0
            return self.steer, self.speed

        # Limite de tiempo de ronda (3 minutos de reglamento)
        if now - self.t_start > float(cfg.round_time_limit_s):
            self.note = "fin por tiempo de ronda"
            self.stop()
            return 0.0, 0.0

        self._update_direction(sc, tel)
        self._ingest(sc)

        yaw_raw = -tel.yaw if bool(cfg.yaw_invert) else tel.yaw
        self._fuse_yaw(yaw_raw, dt)
        yaw = yaw_raw + self.yaw_bias

        front_min = sc.front_min_mm if sc.ok else 9999.0

        # ---------------- transiciones de estado ------------------------
        if self.state in (ST_RUN, ST_FINISH):
            if front_min < float(cfg.emergency_front_mm):
                self.note = "emergencia: obstaculo a %.0f mm" % front_min
                self._recover_until = now + float(cfg.reverse_time_s)
                self._goto(ST_RECOVER)
            elif self._check_stuck(dt, yaw):
                self.note = "atascado: retrocediendo"
                self._recover_until = now + float(cfg.reverse_time_s)
                self._goto(ST_RECOVER)

        if self.state == ST_RUN and self.direction != 0 and self._corner_ahead():
            self.corners += 1
            self.laps = self.corners // 4
            self.t_last_corner = now
            self.target_yaw += self.direction * float(cfg.turn_angle_deg)
            self.finishing = self.laps >= int(cfg.laps_target)
            self._goto(ST_TURN)
            if self.finishing:
                self.note = "ultima esquina: al salir busco la seccion de meta"

        elif self.state == ST_TURN:
            self.head_err = wrap180(self.target_yaw - yaw)
            dt_state = now - self.t_state
            done = (abs(self.head_err) < float(cfg.turn_exit_tol_deg)
                    and dt_state > float(cfg.turn_min_time_s))
            if done or dt_state > float(cfg.turn_max_time_s):
                # El enfriamiento se cuenta desde que ACABA la curva, no desde
                # que empieza: si no, nada mas salir de una curva la siguiente
                # lectura de frente puede disparar otra inmediatamente.
                self.t_last_corner = now
                if self.finishing:
                    self._goto(ST_FINISH)
                    self.note = "3 vueltas hechas: avanzando a la seccion de meta"
                else:
                    self._goto(ST_RUN)
                    self.note = "giro completado (%d esquinas, %d vueltas)" % (
                        self.corners, self.laps)

        elif self.state == ST_FINISH:
            dt_state = now - self.t_state
            stop_by_wall = (self.inner_end is not None
                            and self.inner_end < float(cfg.finish_end_mm))
            if stop_by_wall or dt_state > float(cfg.finish_max_time_s):
                self.note = "parada en la seccion de meta"
                self.stop()
                return 0.0, 0.0

        elif self.state == ST_RECOVER:
            if now >= self._recover_until:
                self._stuck_t = 0.0
                self._goto(ST_RUN)
                self.note = "recuperado"

        # ---------------- ley de control --------------------------------
        if self.state == ST_RECOVER:
            # Marcha atras girando al reves para separarse del obstaculo.
            self.steer = -max(-60.0, min(60.0, self.steer))
            self.speed = -float(cfg.reverse_speed)
            return self.steer, self.speed

        corr = 0.0
        if self.state in (ST_RUN, ST_FINISH):
            corr = self._lateral_correction()
            if bool(cfg.obstacles_enabled):
                corr = self._obstacle_correction(pillar, corr)
        self.head_corr = corr

        desired = self.target_yaw + corr
        self.head_err = wrap180(desired - yaw)

        steer = float(cfg.k_heading) * self.head_err
        lim = float(cfg.steer_limit)
        steer = max(-lim, min(lim, steer))

        # Rampa de direccion
        max_step = float(cfg.steer_slew) * dt
        d = steer - self.steer
        self.steer += max(-max_step, min(max_step, d))

        # ---------------- velocidad -------------------------------------
        base = float(cfg.base_speed)
        if self.state == ST_TURN:
            base = float(cfg.turn_speed)
        elif self.state == ST_FINISH:
            base = max(float(cfg.min_speed), float(cfg.turn_speed))

        f = self.front.value if self.front.valid else 9999.0
        slow = float(cfg.slow_front_mm)
        if f < slow:
            k = max(0.0, min(1.0, (f - float(cfg.turn_hard_front_mm)) /
                             max(1.0, slow - float(cfg.turn_hard_front_mm))))
            base = float(cfg.turn_speed) + (base - float(cfg.turn_speed)) * k

        base *= (1.0 - float(cfg.speed_steer_gain) * abs(self.steer) / 100.0)
        self.speed = max(float(cfg.min_speed), base)
        return self.steer, self.speed

    # ------------------------------------------------------------- telemetria
    def snapshot(self) -> dict:
        return {
            "state": self.state,
            "direction": self.direction,
            "direction_txt": {0: "?", CCW: "antihorario", CW: "horario"}[self.direction],
            "dir_source": self.dir_source,
            "corners": self.corners,
            "laps": self.laps,
            "target_yaw": round(self.target_yaw, 1),
            "yaw_bias": round(self.yaw_bias, 2),
            "head_err": round(self.head_err, 2),
            "head_corr": round(self.head_corr, 2),
            "lat_err": round(self.lat_err, 1),
            "target_lat": round(self.target_lat, 1),
            "d_inner": round(self.d_inner.value, 1) if self.d_inner.valid else None,
            "d_outer": round(self.d_outer.value, 1) if self.d_outer.valid else None,
            "d_left": round(self.d_left.value, 1) if self.d_left.valid else None,
            "d_right": round(self.d_right.value, 1) if self.d_right.valid else None,
            "front": round(self.front.value, 1) if self.front.valid else None,
            "inner_end": round(self.inner_end, 1) if self.inner_end is not None else None,
            "corner_armed": self.corner_armed,
            "corridor": round(self.corridor, 1) if self.corridor else None,
            "steer": round(self.steer, 1),
            "speed": round(self.speed, 1),
            "note": self.note,
            "elapsed": round((self.t_end or self.now()) - self.t_start, 1)
                       if self.t_start else 0.0,
        }
