"""
navegacion.py — No chocar con los muros.

============================================================================
LAS OPCIONES QUE HAY, Y POR QUE ESTAN IMPLEMENTADAS ESTAS DOS
============================================================================

Con una sola camara mirando al frente y muros negros sobre piso blanco, las
maneras razonables de resolverlo son:

1. AREA DE NEGRO EN DOS VENTANAS (izquierda/derecha).
   Contar pixeles negros en dos rectangulos y girar hacia el que tenga menos.
   Es un sensor diferencial virtual, cuesta cuatro lineas y funciona... hasta
   que el muro del fondo (lejos, inofensivo) pesa lo mismo que el de al lado
   (cerca, peligroso). Descartada como principal: no distingue distancia.

2. PERFIL DE CONTACTO MURO-PISO, COLUMNA POR COLUMNA.  <-- IMPLEMENTADA
   Para cada columna de la imagen se busca el pixel negro MAS BAJO: ese es el
   punto donde el muro toca el suelo en esa direccion. Cuanto mas abajo, mas
   cerca. Sale un perfil de distancia de ancho completo, como un LIDAR pobre.
   Con eso se compara el espacio libre a izquierda y derecha y se gira hacia el
   lado despejado con un PD. Es lo que pediste (medir en la franja util, no un
   solo pixel) generalizado a todas las columnas.
   Ventaja: no necesita saber en que sentido se da la vuelta ni calibrar
   geometria. Tolera una calibracion de color mediocre.

3. SEGUIR UNA PARED A DISTANCIA FIJA.  <-- IMPLEMENTADA
   Del mismo perfil se toma solo una banda lateral y se mantiene a una
   distancia objetivo con un PD. Da vueltas muy limpias y repetibles, pero hay
   que decirle que pared seguir y se pierde si esa pared desaparece (puerta de
   salida, hueco). Por eso convive con la 2 y se elige desde la interfaz.

4. VISTA DE PAJARO (homografia) + PURE PURSUIT.
   Rectificar el piso a vista cenital, marcar el corredor libre y perseguir un
   punto objetivo. Es lo "correcto" y lo que hacen los equipos fuertes, pero
   exige calibrar la homografia con un patron y la camara fija. Queda como
   siguiente paso: el perfil de la opcion 2 ya es la entrada natural.

5. GIROSCOPIO COMO RUMBO.  <-- IMPLEMENTADA COMO CAPA ENCIMA
   La pista es un cuadrado: los giros son de 90 grados exactos. Integrar el
   yaw del MPU6050 y mantener el rumbo hace que en recta el carro no serpentee
   y que las esquinas salgan clavadas. La camara decide CUANDO girar, el
   giroscopio decide CUANTO. Si no hay MPU6050, todo funciona igual sin el.

6. ULTRASONIDOS LATERALES.
   Distancia directa y barata, pero un muro de 100 mm visto en angulo rebota el
   eco hacia otro lado y devuelve "sin obstaculo" justo cuando vas a chocar.
   Util solo como red de seguridad de ultimo metro, no como sensor principal.

La capa de SEGURIDAD es independiente de la estrategia: si el pasillo por
donde pasan las ruedas se cierra por debajo de un umbral, se frena; si se
cierra del todo, se para. Eso corre siempre, elijas lo que elijas.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

# Estados de la maquina
RECTO = "recto"
GIRO = "giro"
BLOQUEADO = "bloqueado"


# ---------------------------------------------------------------------------
@dataclass
class PerfilMuro:
    """El 'LIDAR pobre': una distancia libre por columna de la imagen."""
    libre: np.ndarray            # (W,) normalizado 0..1; 1 = despejado
    y_contacto: np.ndarray       # (W,) fila donde el muro toca el piso (0 = sin muro)
    alto: int
    ancho: int
    izq: float = 0.0             # media de la banda izquierda
    der: float = 0.0             # media de la banda derecha
    pasillo: float = 0.0         # percentil bajo entre las lineas de las ruedas
    pasillo_medio: float = 0.0
    min_global: float = 0.0
    hay_muro: bool = False


def _media_movil(v: np.ndarray, k: int) -> np.ndarray:
    if k < 3:
        return v
    if k % 2 == 0:
        k += 1
    pad = k // 2
    ext = np.pad(v, pad, mode="edge")
    nucleo = np.ones(k, dtype=np.float32) / k
    return np.convolve(ext, nucleo, mode="valid")


def perfil_desde_mascara(mascara: np.ndarray, cfg: Dict[str, Any]) -> PerfilMuro:
    """Mascara binaria del muro -> perfil de espacio libre.

    El truco del pixel mas bajo por columna es un argmax sobre la mascara dada
    la vuelta: una sola pasada vectorizada, ~0.3 ms en 640x480.
    """
    H, W = mascara.shape[:2]
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.0))))
    y_fin = max(1, min(H, y_fin))

    m = mascara[:y_fin] > 0
    cuenta = m.sum(axis=0)
    # fila del ultimo True de cada columna
    idx = (y_fin - 1) - np.argmax(m[::-1], axis=0)
    sin_muro = cuenta < int(cfg.get("px_min_columna", 1))
    y_cont = np.where(sin_muro, 0, idx).astype(np.int32)

    libre = np.where(sin_muro, 1.0, (H - y_cont) / float(H)).astype(np.float32)
    libre = _media_movil(libre, int(cfg.get("suavizado", 0)))
    libre = np.clip(libre, 0.0, 1.0)

    p = PerfilMuro(libre=libre, y_contacto=y_cont, alto=H, ancho=W)

    banda = float(cfg.get("banda_lateral", 0.28))
    n_lat = max(1, int(W * banda))
    p.izq = float(libre[:n_lat].mean())
    p.der = float(libre[-n_lat:].mean())

    a = max(0, min(W - 1, int(W * float(cfg.get("ruedas_izq", 0.32)))))
    b = max(a + 1, min(W, int(W * float(cfg.get("ruedas_der", 0.68)))))
    corredor = libre[a:b]
    # percentil bajo, no el minimo: una columna con ruido no frena el carro
    p.pasillo = float(np.percentile(corredor, 15))
    p.pasillo_medio = float(corredor.mean())
    p.min_global = float(libre.min())
    p.hay_muro = bool((~sin_muro).any())
    return p


# ---------------------------------------------------------------------------
@dataclass
class Decision:
    vel: int = 0                 # % de vmax, con signo
    direccion: int = 0           # % -100 izquierda .. +100 derecha
    estado: str = RECTO
    motivo: str = ""
    metricas: Dict[str, float] = field(default_factory=dict)


class _PD:
    """PD con derivada sobre el tiempo real (los frames no llegan parejos)."""

    def __init__(self):
        self.prev: Optional[float] = None
        self.t_prev: float = 0.0

    def paso(self, err: float, kp: float, kd: float, ahora: float) -> float:
        d = 0.0
        if self.prev is not None:
            dt = max(1e-3, ahora - self.t_prev)
            d = (err - self.prev) / dt
        self.prev = err
        self.t_prev = ahora
        return kp * err + kd * d * 0.1     # el 0.1 deja kd en el mismo orden que kp

    def reiniciar(self):
        self.prev = None


# ---------------------------------------------------------------------------
class Navegador:
    """Junta perfil + estrategia + maquina de estados + seguridad.

    La configuracion se lee en cada llamada, asi que mover un slider en el panel
    o en la web cambia el comportamiento en el mismo frame.
    """

    def __init__(self, cfg_nav: Dict[str, Any], cfg_lim: Dict[str, Any]):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.pd_centrado = _PD()
        self.pd_pared = _PD()
        self.estado = RECTO
        self.t_estado = time.time()
        self.lado_giro = 1               # +1 derecha, -1 izquierda
        self.rumbo_objetivo: Optional[float] = None
        self.ultimo: Decision = Decision()

    # -- utilidades -------------------------------------------------------
    def reiniciar(self):
        self.pd_centrado.reiniciar()
        self.pd_pared.reiniciar()
        self.estado = RECTO
        self.t_estado = time.time()
        self.rumbo_objetivo = None

    def _cambiar(self, estado: str):
        if estado != self.estado:
            self.estado = estado
            self.t_estado = time.time()

    # -- estrategias ------------------------------------------------------
    def _dir_centrado(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        # + = mas espacio a la derecha -> girar a la derecha
        err = p.der - p.izq
        d = self.pd_centrado.paso(err, float(self.cfg.get("kp", 95.0)),
                                  float(self.cfg.get("kd", 22.0)), ahora)
        return d, f"centrado err={err:+.3f}"

    def _dir_pared(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        lado = str(self.cfg.get("lado_pared", "izq")).lower().startswith("i")
        d_actual = p.izq if lado else p.der
        objetivo = float(self.cfg.get("pared_objetivo", 0.45))
        err = d_actual - objetivo          # >0 = estoy lejos de esa pared
        salida = self.pd_pared.paso(err, float(self.cfg.get("kp_pared", 130.0)),
                                    float(self.cfg.get("kd_pared", 28.0)), ahora)
        # Si sigo la pared izquierda y estoy lejos, me acerco girando a la izquierda
        signo = -1.0 if lado else 1.0
        return signo * salida, f"pared {'izq' if lado else 'der'} d={d_actual:.3f} err={err:+.3f}"

    # -- ciclo principal --------------------------------------------------
    def paso(self, perfil: PerfilMuro, yaw: Optional[float] = None) -> Decision:
        ahora = time.time()
        cfg = self.cfg
        vel_crucero = float(self.lim.get("vel_crucero", 55))
        vel_giro = float(self.lim.get("vel_giro", 38))
        dir_max = float(self.lim.get("dir_max", 100))

        libre = perfil.pasillo
        usar_yaw = bool(cfg.get("usar_yaw", True)) and yaw is not None
        if usar_yaw and self.rumbo_objetivo is None:
            self.rumbo_objetivo = yaw

        motivo = ""
        direccion = 0.0
        vel = 0.0

        # ---------- BLOQUEADO: el pasillo se cerro ------------------------
        if libre < float(cfg.get("parar_bajo", 0.24)):
            self._cambiar(BLOQUEADO)
        elif self.estado == BLOQUEADO and libre > float(cfg.get("girar_bajo", 0.40)):
            self._cambiar(RECTO)

        if self.estado == BLOQUEADO:
            # Girar a fondo hacia el lado despejado con el carro parado no sirve
            # (direccion Ackermann), asi que se retrocede despacio girando al
            # reves para reencuadrar. El ESP32 obliga a la pausa de inversion.
            lado = 1.0 if perfil.der > perfil.izq else -1.0
            d = Decision(vel=int(-vel_giro * 0.6), direccion=int(-lado * dir_max * 0.8),
                         estado=BLOQUEADO, motivo=f"pasillo {libre:.2f} cerrado, retrocedo")
            d.metricas = self._metricas(perfil, yaw)
            self.ultimo = d
            return d

        # ---------- GIRO: esquina ----------------------------------------
        if self.estado == GIRO:
            venc = (ahora - self.t_estado) * 1000 > float(cfg.get("giro_max_ms", 3000))
            if usar_yaw and self.rumbo_objetivo is not None:
                err_yaw = _dif_angulo(self.rumbo_objetivo, yaw)
                if abs(err_yaw) < float(cfg.get("giro_tolerancia", 8.0)) or venc:
                    self._cambiar(RECTO)
                    self.rumbo_objetivo = yaw if venc else self.rumbo_objetivo
                else:
                    direccion = _lim(err_yaw * float(cfg.get("yaw_kp", 1.6)) * 3.0,
                                     -dir_max, dir_max)
                    vel = vel_giro
                    motivo = f"giro por yaw err={err_yaw:+.1f}"
                    return self._salida(vel, direccion, perfil, yaw, motivo)
            else:
                if libre > float(cfg.get("salir_giro_sobre", 0.55)) or venc:
                    self._cambiar(RECTO)
                    self.pd_centrado.reiniciar()
                else:
                    direccion = self.lado_giro * float(cfg.get("dir_giro", 90.0))
                    vel = vel_giro
                    motivo = f"giro por vision libre={libre:.2f}"
                    return self._salida(vel, direccion, perfil, yaw, motivo)

        # ---------- RECTO -------------------------------------------------
        # Tiempo minimo en recta antes de admitir otra esquina: si no, al salir
        # de un giro el pasillo sigue medio cerrado y el carro encadena giros
        # dando vueltas sobre si mismo (y con yaw, sumando 90 grados cada vez).
        recto_estable = (ahora - self.t_estado) * 1000 >= float(
            cfg.get("min_recto_ms", 600))
        if libre < float(cfg.get("girar_bajo", 0.40)) and recto_estable:
            self.lado_giro = 1 if perfil.der > perfil.izq else -1
            self._cambiar(GIRO)
            if usar_yaw:
                paso = float(cfg.get("giro_grados", 90.0)) * self.lado_giro
                self.rumbo_objetivo = _norm_angulo((self.rumbo_objetivo or yaw) + paso)
            direccion = self.lado_giro * float(cfg.get("dir_giro", 90.0))
            return self._salida(vel_giro, direccion, perfil, yaw,
                                f"entro en giro hacia {'der' if self.lado_giro > 0 else 'izq'}")

        estrategia = str(cfg.get("estrategia", "centrado")).lower()
        if estrategia.startswith("par"):
            direccion, motivo = self._dir_pared(perfil, ahora)
        else:
            direccion, motivo = self._dir_centrado(perfil, ahora)

        # Correccion por giroscopio: mantiene la recta pegada al rumbo.
        # Solo corrige, nunca manda: se suma acotada para no pelear con la vision.
        if usar_yaw and self.rumbo_objetivo is not None:
            err_yaw = _dif_angulo(self.rumbo_objetivo, yaw)
            correccion = _lim(err_yaw * float(cfg.get("yaw_kp", 1.6)),
                              -float(cfg.get("yaw_max", 45.0)),
                              float(cfg.get("yaw_max", 45.0)))
            direccion += correccion
            motivo += f" yaw{err_yaw:+.1f}"

        # ---------- velocidad segun lo despejado que este ------------------
        frenar = float(cfg.get("frenar_bajo", 0.50))
        parar = float(cfg.get("parar_bajo", 0.24))
        if libre >= frenar:
            vel = vel_crucero
        else:
            t = (libre - parar) / max(1e-3, frenar - parar)
            vel = vel_giro + (vel_crucero - vel_giro) * _lim(t, 0.0, 1.0)
            motivo += " (frenando)"

        # Girar fuerte y correr a la vez es como se sale de la pista.
        vel *= 1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max))

        return self._salida(vel, direccion, perfil, yaw, motivo)

    # ---------------------------------------------------------------------
    def _salida(self, vel: float, direccion: float, perfil: PerfilMuro,
                yaw: Optional[float], motivo: str) -> Decision:
        dir_max = float(self.lim.get("dir_max", 100))
        d = Decision(
            vel=int(round(_lim(vel, -100, 100))),
            direccion=int(round(_lim(direccion, -dir_max, dir_max))),
            estado=self.estado,
            motivo=motivo,
            metricas=self._metricas(perfil, yaw),
        )
        self.ultimo = d
        return d

    def _metricas(self, perfil: PerfilMuro, yaw: Optional[float]) -> Dict[str, float]:
        m = {
            "izq": round(perfil.izq, 3),
            "der": round(perfil.der, 3),
            "pasillo": round(perfil.pasillo, 3),
            "min": round(perfil.min_global, 3),
        }
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["yaw_obj"] = round(self.rumbo_objetivo, 1)
        return m


# ---------------------------------------------------------------------------
def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _norm_angulo(a: float) -> float:
    """Deja el angulo en (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def _dif_angulo(objetivo: float, actual: float) -> float:
    return _norm_angulo(objetivo - actual)


# ---------------------------------------------------------------------------
def dibujar_navegacion(frame: np.ndarray, perfil: PerfilMuro, d: Decision,
                       cfg: Dict[str, Any]) -> np.ndarray:
    """Superpone lo que el carro esta 'viendo': el perfil del muro, las bandas,
    por donde pasan las ruedas y la decision. Es lo que se ve en carrito.local."""
    H, W = frame.shape[:2]

    # --- perfil de contacto: la linea que recorre la base del muro ---------
    pts = []
    for x in range(0, W, 4):
        y = int(perfil.y_contacto[min(x, len(perfil.y_contacto) - 1)])
        if y > 0:
            pts.append((x, y))
    for i in range(1, len(pts)):
        if pts[i][0] - pts[i - 1][0] <= 8:
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 255), 2)

    # --- lineas por donde pasan las ruedas (la camara no las ve) ----------
    xi = int(W * float(cfg.get("ruedas_izq", 0.32)))
    xd = int(W * float(cfg.get("ruedas_der", 0.68)))
    for x in (xi, xd):
        cv2.line(frame, (x, int(H * 0.45)), (x, H), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (xi, H - 2), (xd, H - 2), (255, 255, 255), 1)

    # --- franja del chasis ignorada ---------------------------------------
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.0))))
    if y_fin < H:
        cv2.line(frame, (0, y_fin), (W, y_fin), (120, 120, 120), 1)

    # --- barras de espacio libre ------------------------------------------
    def barra(x0, x1, valor, etiqueta, color):
        alto = int(44 * _lim(valor, 0, 1))
        cv2.rectangle(frame, (x0, H - 12 - alto), (x1, H - 12), color, -1)
        cv2.putText(frame, f"{etiqueta}{valor:.2f}", (x0 - 2, H - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    banda = float(cfg.get("banda_lateral", 0.28))
    n_lat = max(1, int(W * banda))
    barra(4, 4 + 26, perfil.izq, "I", (60, 220, 60))
    barra(W - 30, W - 4, perfil.der, "D", (60, 220, 60))
    barra(W // 2 - 13, W // 2 + 13, perfil.pasillo, "C", (60, 200, 255))
    cv2.line(frame, (n_lat, int(H * 0.5)), (n_lat, H), (80, 80, 80), 1)
    cv2.line(frame, (W - n_lat, int(H * 0.5)), (W - n_lat, H), (80, 80, 80), 1)

    # --- volante y velocidad ----------------------------------------------
    cx, cy = W // 2, 26
    largo = int(W * 0.22)
    x2 = int(cx + largo * (d.direccion / 100.0))
    cv2.line(frame, (cx, cy), (x2, cy), (0, 165, 255), 4)
    cv2.circle(frame, (cx, cy), 4, (255, 255, 255), -1)

    color_estado = {RECTO: (0, 255, 0), GIRO: (0, 200, 255),
                    BLOQUEADO: (0, 0, 255)}.get(d.estado, (255, 255, 255))
    cv2.putText(frame, f"{d.estado.upper()}  vel={d.vel:+d}%  dir={d.direccion:+d}%",
                (8, H - 94), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_estado, 1, cv2.LINE_AA)
    if d.motivo:
        cv2.putText(frame, d.motivo[:58], (8, H - 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    return frame
