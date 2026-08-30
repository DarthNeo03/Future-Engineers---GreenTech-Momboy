"""
navegacion.py — Decidir velocidad y direccion a partir del perfil del muro.

Maquina de estados:

    RECTO -> PRE_GIRO -> GIRO -> RECTO        y ESCAPE por encima de todo
      |________________________________|

  * RECTO      la estrategia manda ('centrado' o 'pared'), el giroscopio
               corrige el rumbo (acotado: ayuda, no manda) y el esquive de
               pilares aporta su sesgo si esta activo.
  * PRE_GIRO   la esquina ya se disparo pero: (1) se frena ANTES de doblar,
               (2) se espera retardo_giro_ms para que las ruedas TRASERAS
               pasen el canto del muro interno, y (3) si hay sitio, se abre
               hacia el lado contrario (giro abierto, como un camion en un
               cruce de 90) para no cortar la esquina con la cola.
  * GIRO       con giroscopio: rumbo objetivo +-90 y a clavarlo. Sin el:
               direccion fija hasta que el pasillo abre.
  * ESCAPE     el pasillo se cerro de verdad: marcha atras COMPROMETIDA
               (minimo + extra segun el deficit). Ir y venir cada 500 ms
               frente a un muro es exactamente como se choca; el compromiso
               es la cura. La direccion va HACIA el muro para que el morro
               se separe, como al salir de un estacionamiento.

Sentido de la ronda: geometria pura. HORARIO = el centro de la pista queda a
la DERECHA del carro = muro interno a la derecha = las esquinas doblan a la
derecha. ANTIHORARIO, todo al reves. Si el sentido aun no se conoce, la
esquina se decide por el lado que tenga mas espacio (como el programa viejo).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from .muro import PerfilMuro, DetectorEsquinaInterna

RECTO = "recto"
PRE_GIRO = "pre_giro"
GIRO = "giro"
ESCAPE = "escape"


@dataclass
class Decision:
    vel: int = 0                 # % de vmax, con signo
    direccion: int = 0           # % -100 izquierda .. +100 derecha
    estado: str = RECTO
    motivo: str = ""
    metricas: Dict[str, Any] = field(default_factory=dict)


class _PD:
    def __init__(self):
        self.prev: Optional[float] = None
        self.t_prev = 0.0

    def paso(self, err: float, kp: float, kd: float, ahora: float) -> float:
        d = 0.0
        if self.prev is not None:
            dt = max(1e-3, ahora - self.t_prev)
            d = (err - self.prev) / dt
        self.prev = err
        self.t_prev = ahora
        return kp * err + kd * d * 0.1

    def reiniciar(self):
        self.prev = None


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _norm_ang(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Navegador:
    def __init__(self, cfg_nav: Dict[str, Any], cfg_lim: Dict[str, Any],
                 cfg_esc: Dict[str, Any],
                 al_completar_giro: Optional[Callable[[int], None]] = None):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.esc = cfg_esc
        self.al_completar_giro = al_completar_giro or (lambda lado: None)

        self.pd = _PD()
        self.pd_pared = _PD()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.lado_giro = 0
        self.rumbo_objetivo: Optional[float] = None
        self.ultimo = Decision()

        self._t_fin_escape = 0.0
        self._escape_intentos = 0
        self._pasillo_prev: Optional[float] = None
        self._t_pasillo = 0.0
        self._vel_cierre = 0.0        # mm/s, positivo = el muro se acerca

    # ------------------------------------------------------------------
    def reiniciar(self):
        self.pd.reiniciar()
        self.pd_pared.reiniciar()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.rumbo_objetivo = None
        self._escape_intentos = 0
        self._pasillo_prev = None

    def _cambiar(self, estado: str):
        if estado != self.estado:
            self.estado = estado
            self.t_estado = time.time()

    # ------------------------------------------------------------------
    def paso(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
             linea_reciente: bool = False,
             bias_obstaculo: Tuple[float, float] = (0.0, 0.0)) -> Decision:
        """sentido: +1 horario, -1 antihorario, 0 desconocido.
        bias_obstaculo: (direccion_deseada_pct, peso 0..1) del esquive."""
        ahora = time.time()
        cfg, lim, esc = self.cfg, self.lim, self.esc
        vel_crucero = float(lim.get("vel_crucero", 55))
        vel_giro = float(lim.get("vel_giro", 38))
        dir_max = float(lim.get("dir_max", 100))
        pasillo = p.pasillo_mm

        usar_yaw = bool(cfg.get("usar_yaw", True)) and yaw is not None
        if usar_yaw and self.rumbo_objetivo is None:
            self.rumbo_objetivo = yaw

        # --- velocidad de cierre del pasillo (para el freno por tiempo) ---
        if self._pasillo_prev is not None:
            dt = max(1e-3, ahora - self._t_pasillo)
            inst = (self._pasillo_prev - pasillo) / dt
            self._vel_cierre = 0.7 * self._vel_cierre + 0.3 * inst
        self._pasillo_prev = pasillo
        self._t_pasillo = ahora

        aviso_interna = self.detector_interna.paso(p, cfg)

        # =================== ESCAPE (prioridad maxima) ====================
        parar_bajo = float(cfg.get("parar_bajo_mm", 300.0))
        if self.estado != ESCAPE and pasillo < parar_bajo:
            self._cambiar(ESCAPE)
            deficit = parar_bajo - pasillo
            comp = float(esc.get("escape_min_ms", 750)) + \
                float(esc.get("escape_k_ms_por_mm", 3.0)) * deficit
            self._t_fin_escape = ahora + comp / 1000.0
            self._escape_intentos += 1

        if self.estado == ESCAPE:
            if ahora >= self._t_fin_escape:
                if pasillo > float(cfg.get("girar_bajo_mm", 650.0)) * 0.8:
                    self._escape_intentos = 0
                    self._cambiar(RECTO)
                    self.pd.reiniciar()
                    if usar_yaw:
                        self.rumbo_objetivo = yaw   # el rumbo viejo ya no vale
                elif self._escape_intentos > int(esc.get("escape_max_intentos", 4)):
                    # la reversa no gana espacio (algo detras): giro adelante
                    self._escape_intentos = 0
                    self.lado_giro = 1 if p.der > p.izq else -1
                    self._cambiar(GIRO)
                    if usar_yaw:
                        self.rumbo_objetivo = _norm_ang(
                            yaw + self.lado_giro * float(cfg.get("giro_grados", 90.0)))
                else:
                    deficit = max(0.0, parar_bajo - pasillo)
                    comp = float(esc.get("escape_min_ms", 750)) + \
                        float(esc.get("escape_k_ms_por_mm", 3.0)) * deficit
                    self._t_fin_escape = ahora + comp / 1000.0
                    self._escape_intentos += 1
            if self.estado == ESCAPE:
                lado_muro = -1 if p.izq < p.der else 1   # donde esta el muro
                return self._salida(
                    -float(lim.get("vel_reversa", 35)),
                    lado_muro * float(esc.get("escape_dir", 80.0)),
                    p, yaw, sentido,
                    f"escape #{self._escape_intentos} pasillo={pasillo:.0f}mm")

        # =================== PRE_GIRO =====================================
        if self.estado == PRE_GIRO:
            if (ahora - self.t_estado) * 1000 >= float(cfg.get("retardo_giro_ms", 220)):
                self._cambiar(GIRO)
                if usar_yaw:
                    base = self.rumbo_objetivo if self.rumbo_objetivo is not None else yaw
                    self.rumbo_objetivo = _norm_ang(
                        base + self.lado_giro * float(cfg.get("giro_grados", 90.0)))
            else:
                # giro abierto: contra-direccion si el lado contrario tiene sitio
                apertura = float(cfg.get("apertura_pct", 25.0))
                libre_contrario = (p.izq if self.lado_giro > 0 else p.der) * p.alcance_mm
                d = 0.0
                if apertura > 0 and libre_contrario > float(
                        cfg.get("apertura_min_libre_mm", 400.0)):
                    d = -self.lado_giro * apertura
                return self._salida(vel_giro, d, p, yaw, sentido,
                                    f"pre-giro {'der' if self.lado_giro > 0 else 'izq'}")

        # =================== GIRO =========================================
        if self.estado == GIRO:
            venc = (ahora - self.t_estado) * 1000 > float(cfg.get("giro_max_ms", 3000))
            if usar_yaw and self.rumbo_objetivo is not None:
                err = _norm_ang(self.rumbo_objetivo - yaw)
                if abs(err) < float(cfg.get("giro_tolerancia_deg", 8.0)) or venc:
                    if venc:
                        self.rumbo_objetivo = yaw
                    self._terminar_giro()
                else:
                    d = _lim(err * float(cfg.get("yaw_kp", 1.6)) * 3.0,
                             -dir_max, dir_max)
                    return self._salida(vel_giro, d, p, yaw, sentido,
                                        f"giro yaw err={err:+.0f}")
            else:
                if pasillo > float(cfg.get("salir_giro_mm", 950.0)) or venc:
                    self._terminar_giro()
                else:
                    d = self.lado_giro * float(cfg.get("dir_giro", 85.0))
                    return self._salida(vel_giro, d, p, yaw, sentido,
                                        f"giro vision pasillo={pasillo:.0f}")

        # =================== RECTO ========================================
        recto_estable = (ahora - self.t_estado) * 1000 >= float(
            cfg.get("min_recto_ms", 700))

        disparo = ""
        if recto_estable:
            if pasillo < float(cfg.get("girar_bajo_mm", 650.0)):
                disparo = f"pasillo {pasillo:.0f}mm"
            elif aviso_interna is not None:
                lado_aviso = 1 if aviso_interna == "der" else -1
                # si conocemos el sentido, solo cuenta si desaparecio el lado
                # INTERNO (el externo casi nunca desaparece; si lo hace, es ruido)
                if sentido == 0 or lado_aviso == sentido:
                    disparo = f"muro interno ({aviso_interna}) desaparecio"
                    self.lado_giro = lado_aviso
            elif linea_reciente and bool(cfg.get("giro_por_linea", True)) and \
                    pasillo < float(cfg.get("girar_bajo_mm", 650.0)) * 1.6:
                disparo = "linea de esquina + pasillo cerrando"

        if disparo:
            if not self.lado_giro or "pasillo" in disparo or "linea" in disparo:
                self.lado_giro = sentido if sentido != 0 else (
                    1 if p.der > p.izq else -1)
            self._cambiar(PRE_GIRO)
            return self._salida(vel_giro, 0.0, p, yaw, sentido,
                                f"esquina: {disparo}")

        # --- estrategia ----------------------------------------------------
        if str(cfg.get("estrategia", "centrado")).startswith("par"):
            direccion, motivo = self._dir_pared(p, sentido, ahora)
        else:
            direccion, motivo = self._dir_centrado(p, ahora)

        # --- esquive de pilares (sesgo ponderado) --------------------------
        bias_dir, peso = bias_obstaculo
        if peso > 0.0:
            direccion = (1.0 - peso) * direccion + peso * bias_dir
            motivo += f" esq({bias_dir:+.0f}x{peso:.2f})"

        # --- rumbo por giroscopio -----------------------------------------
        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            corr = _lim(err * float(cfg.get("yaw_kp", 1.6)),
                        -float(cfg.get("yaw_max", 45.0)),
                        float(cfg.get("yaw_max", 45.0)))
            direccion += corr
            motivo += f" yaw{err:+.0f}"

        # --- velocidad -----------------------------------------------------
        frenar = float(cfg.get("frenar_bajo_mm", 1000.0))
        if pasillo >= frenar:
            vel = vel_crucero
        else:
            t = (pasillo - parar_bajo) / max(1.0, frenar - parar_bajo)
            vel = vel_giro + (vel_crucero - vel_giro) * _lim(t, 0.0, 1.0)
            motivo += " frenando"

        # freno por tiempo-hasta-el-muro: la inercia no espera a la distancia
        ttc_min = float(cfg.get("ttc_min_s", 0.7))
        if self._vel_cierre > 60.0:
            ttc = pasillo / self._vel_cierre
            if ttc < ttc_min:
                vel = min(vel, vel_giro * _lim(ttc / ttc_min, 0.35, 1.0))
                motivo += f" ttc={ttc:.1f}s"

        # girar fuerte y correr a la vez es como se sale de la pista
        vel *= 1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max))

        return self._salida(vel, direccion, p, yaw, sentido, motivo)

    # ------------------------------------------------------------------
    def _terminar_giro(self):
        lado = self.lado_giro
        self._cambiar(RECTO)
        self.pd.reiniciar()
        try:
            self.al_completar_giro(lado)
        except Exception:
            pass

    def _dir_centrado(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        err = p.der - p.izq          # + = mas espacio a la derecha
        d = self.pd.paso(err, float(self.cfg.get("kp", 95.0)),
                         float(self.cfg.get("kd", 22.0)), ahora)
        return d, f"centrado {err:+.2f}"

    def _dir_pared(self, p: PerfilMuro, sentido: int,
                   ahora: float) -> Tuple[float, str]:
        """Sigue el muro INTERNO (horario: derecha; antihorario: izquierda)
        a distancia fija. Si el sentido no se conoce aun, cae al centrado."""
        if sentido == 0:
            return self._dir_centrado(p, ahora)
        lado_int = sentido               # +1 = interno a la derecha
        d_actual = (p.der if lado_int > 0 else p.izq) * p.alcance_mm
        objetivo = float(self.cfg.get("pared_objetivo_mm", 320.0))
        err = d_actual - objetivo        # >0 = estoy lejos del muro interno
        salida = self.pd_pared.paso(err, float(self.cfg.get("kp_pared", 0.22)),
                                    float(self.cfg.get("kd_pared", 0.05)), ahora)
        # interno a la derecha y lejos -> acercarse girando a la derecha
        return lado_int * salida, f"pared int d={d_actual:.0f} err={err:+.0f}"

    # ------------------------------------------------------------------
    def _salida(self, vel: float, direccion: float, p: PerfilMuro,
                yaw: Optional[float], sentido: int, motivo: str) -> Decision:
        dir_max = float(self.lim.get("dir_max", 100))
        d = Decision(
            vel=int(round(_lim(vel, -100, 100))),
            direccion=int(round(_lim(direccion, -dir_max, dir_max))),
            estado=self.estado,
            motivo=motivo,
            metricas=self._metricas(p, yaw, sentido),
        )
        self.ultimo = d
        return d

    def _metricas(self, p: PerfilMuro, yaw: Optional[float],
                  sentido: int) -> Dict[str, Any]:
        m = {
            "izq": round(p.izq, 2),
            "der": round(p.der, 2),
            "pasillo_mm": round(p.pasillo_mm, 0),
            "cob_izq": round(p.cobertura_izq, 2),
            "cob_der": round(p.cobertura_der, 2),
            "cierre_mms": round(self._vel_cierre, 0),
            "sentido": sentido,
        }
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["rumbo_obj"] = round(self.rumbo_objetivo, 1)
        return m
