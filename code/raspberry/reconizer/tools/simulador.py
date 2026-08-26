#!/usr/bin/env python3
"""
simulador.py — Da vueltas a la pista entera sin carro, sin camara y sin ESP32.

    python3 tools/simulador.py                      # una vuelta tipica
    python3 tools/simulador.py --todas               # las 16 combinaciones
    python3 tools/simulador.py --sentido cw --anchos 600 1000 600 1000
    python3 tools/simulador.py --sin-yaw             # como si fallara la IMU
    python3 tools/simulador.py --ver               # VERLO en una ventana
    python3 tools/simulador.py --video vuelta.gif  # grabarlo
    python3 tools/simulador.py --senales 2 --ver   # con pilares
    python3 tools/simulador.py --traza salida.png

POR QUE HACE FALTA
------------------
Las pruebas de `selftest_robot.py` validan piezas sueltas: que el salto de
rango se detecta, que el PD tiene el signo bueno, que el giro cierra a 90
grados. Ninguna de ellas puede detectar que el conjunto se sale de la pista en
la tercera esquina, porque para eso hace falta CERRAR EL LAZO: la decision del
navegador tiene que mover el carro, y el carro moverse tiene que cambiar lo que
la camara ve en el frame siguiente.

Aqui esta ese lazo. La pista es la de verdad (reglamento 2026: cuadrado
exterior de 3000x3000 fijo, rectangulo interior con lados de 1000, 1400 o
1800 mm segun el sorteo), el carro es un modelo de bicicleta con direccion
Ackermann, y la "camara" es un trazador de rayos sobre esas paredes con el
mismo campo de vision que la de verdad.

LO QUE MIDE
-----------
  * si toca el muro EXTERIOR, que la regla 9.18 prohibe expresamente;
  * si toca el interior (permitido si no lo mueve, pero mala señal);
  * cuantas vueltas completa y en cuanto tiempo;
  * la distancia minima a cada muro durante toda la vuelta.

NO SUSTITUYE A LA PISTA. El simulador no tiene reflejos, ni exposicion
automatica, ni holgura en la direccion, ni deriva de giroscopio de verdad.
Sirve para lo que sirve: cazar errores de logica y de signo antes de gastar
bateria, y comparar ajustes de parametros de forma repetible.
"""

from __future__ import annotations

import argparse
import itertools
import math
import pathlib
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import geometria as geo, navegacion as nav, obstaculos as obs  # noqa: E402
from src import robot_config  # noqa: E402

LADO_PISTA = 3000.0          # cara interna del muro exterior (regla 13.1)
ANCHOS_VALIDOS = (600.0, 1000.0)
PILAR_MM = 50.0              # regla 13.19: pilares de 50 x 50 x 100 mm


# ---------------------------------------------------------------------------
@dataclass
class Pista:
    """Geometria de la pista, con el sorteo de anchos ya aplicado.

    `anchos` son los cuatro corredores en orden SUR, ESTE, NORTE, OESTE.
    Cada uno vale 1000 o 600 mm, y de ahi sale un rectangulo interior cuyos
    lados miden 1000, 1400 o 1800 mm, que es exactamente el juego de piezas
    que describe el Apendice B del reglamento.
    """
    anchos: Tuple[float, float, float, float] = (1000.0, 1000.0, 1000.0, 1000.0)
    # Señales de trafico: (x, y, color). Pilares de 50x50x100 (regla 13.19).
    senales: Tuple[Tuple[float, float, str], ...] = ()

    def __post_init__(self):
        s, e, n, o = self.anchos
        self.ix0, self.ix1 = o, LADO_PISTA - e
        self.iy0, self.iy1 = s, LADO_PISTA - n

    @property
    def segmentos(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        L = LADO_PISTA
        ext = [((0, 0), (L, 0)), ((L, 0), (L, L)), ((L, L), (0, L)), ((0, L), (0, 0))]
        a, b, c, d = ((self.ix0, self.iy0), (self.ix1, self.iy0),
                      (self.ix1, self.iy1), (self.ix0, self.iy1))
        segs = ext + [(a, b), (b, c), (c, d), (d, a)]
        return segs

    @property
    def segmentos_pilar(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """Caras de los pilares. Van APARTE de los muros a proposito.

        En el carro de verdad el perfil del muro sale de la mascara del color
        NEGRO, y un pilar rojo o verde no esta en esa mascara. Lo que si hace
        un pilar es TAPAR el muro que tiene detras: esas columnas se quedan sin
        contacto y pasan a invalidas. O sea que un pilar no es un obstaculo
        para el escaneo de muros, es un AGUJERO en el.

        Modelarlo al reves -como si fuera un trozo de muro- haria la prueba
        mas facil de lo que es la realidad y escondería el hueco de verdad: la
        capa de seguridad, que solo mira muros, es CIEGA a los pilares.
        """
        segs = []
        for (px, py, _c) in self.senales:
            h = PILAR_MM / 2
            e0 = (px - h, py - h); e1 = (px + h, py - h)
            e2 = (px + h, py + h); e3 = (px - h, py + h)
            segs += [(e0, e1), (e1, e2), (e2, e3), (e3, e0)]
        return segs

    def dist_exterior(self, p: Tuple[float, float]) -> float:
        x, y = p
        return min(x, LADO_PISTA - x, y, LADO_PISTA - y)

    def dist_interior(self, p: Tuple[float, float]) -> float:
        """Distancia al rectangulo interior. Negativa si esta dentro de el."""
        x, y = p
        dx = max(self.ix0 - x, x - self.ix1)
        dy = max(self.iy0 - y, y - self.iy1)
        if dx > 0 and dy > 0:
            return math.hypot(dx, dy)
        return max(dx, dy)

    def centro_corredor(self, sector: int, f: float = 0.5) -> Tuple[float, float]:
        """Punto del corredor de un sector (0=sur, 1=este, 2=norte, 3=oeste).

        `f` recorre la SECCION RECTA, que es el tramo que abarca el rectangulo
        interior. Con f=0.2 el carro sale por el principio de la recta, que es
        lo tipico y ademas deja sitio de reaccion antes del primer pilar.
        """
        s, e, n, o = self.anchos
        L = LADO_PISTA
        if sector == 0:
            return (self.ix0 + (self.ix1 - self.ix0) * f, s / 2)
        if sector == 1:
            return (L - e / 2, self.iy0 + (self.iy1 - self.iy0) * f)
        if sector == 2:
            return (self.ix1 - (self.ix1 - self.ix0) * f, L - n / 2)
        return (o / 2, self.iy1 - (self.iy1 - self.iy0) * f)


# ---------------------------------------------------------------------------
@dataclass
class Carro:
    """Modelo de bicicleta con direccion Ackermann.

    El reglamento prohibe la traccion diferencial (regla 11.3/11.5), asi que
    este es el modelo correcto: no puede girar sobre si mismo, y esa es
    justamente la razon de que el estado BLOQUEADO retroceda en vez de pivotar.
    """
    x: float
    y: float
    theta: float                     # rad, 0 = hacia +x
    batalla_mm: float = 200.0        # distancia entre ejes
    largo_mm: float = 300.0
    ancho_mm: float = 200.0
    rueda_max_deg: float = 30.0      # tope real de la rueda, no del servo
    rueda_grados_s: float = 200.0    # limite de barrido de la direccion
    mm_por_seg_a_100: float = 900.0
    delta: float = 0.0               # angulo de rueda actual, rad

    @property
    def radio_giro_mm(self) -> float:
        return self.batalla_mm / math.tan(math.radians(self.rueda_max_deg))

    def avanzar(self, vel_pct: float, dir_pct: float, dt: float) -> None:
        # OJO CON EL SIGNO: en todo el proyecto direccion positiva = DERECHA
        # (ver protocolo.py). En coordenadas de campo con Y hacia arriba, girar
        # a la derecha hace DISMINUIR theta, de ahi el menos.
        objetivo = -math.radians(self.rueda_max_deg) * max(-1.0, min(1.0, dir_pct / 100.0))
        paso = math.radians(self.rueda_grados_s) * dt
        self.delta += max(-paso, min(paso, objetivo - self.delta))

        v = self.mm_por_seg_a_100 * (vel_pct / 100.0)
        self.theta += (v / self.batalla_mm) * math.tan(self.delta) * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

    def esquinas(self) -> List[Tuple[float, float]]:
        c, s = math.cos(self.theta), math.sin(self.theta)
        hl, hw = self.largo_mm / 2, self.ancho_mm / 2
        return [(self.x + c * dx - s * dy, self.y + s * dx + c * dy)
                for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]

    @property
    def yaw_deg(self) -> float:
        """Yaw en convenio de BRUJULA: positivo al girar a la DERECHA.

        Es el convenio que espera el navegador (`rumbo_objetivo = base + 90 *
        lado_interno`, con lado +1 = derecha). En coordenadas de campo con Y
        hacia arriba, girar a la derecha hace DISMINUIR theta, de ahi el signo.
        Si el MPU6050 del carro real entrega el signo contrario, se corrige con
        `imu.invertir_yaw` en robot.json en vez de tocar el navegador.
        """
        return -math.degrees(self.theta)


# ---------------------------------------------------------------------------
def escanear_pista(pista: Pista, carro: Carro, ancho_px: int = 640,
                   hfov: float = 100.0, ruido_mm: float = 0.0,
                   rng: Optional[np.random.Generator] = None) -> geo.Escaneo:
    """La 'camara': un trazador de rayos sobre las paredes de la pista.

    Vectorizado sobre los 640 rayos a la vez: el bucle solo recorre los 8
    segmentos de la pista. En bucle puro de Python son 5120 intersecciones por
    frame y el barrido de las 16 combinaciones no terminaba nunca.
    """
    med = math.radians(hfov) / 2.0
    b = np.linspace(-med, med, ancho_px)
    ang = carro.theta - b                       # bearing + = a la derecha
    dx, dy = np.cos(ang), np.sin(ang)
    ox, oy = carro.x, carro.y

    def _cortar(segmentos):
        m = np.full(ancho_px, np.inf)
        for (x1, y1), (x2, y2) in segmentos:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            with np.errstate(divide="ignore", invalid="ignore"):
                t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
                u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            ok = (np.abs(den) > 1e-9) & (t > 1e-6) & (u >= -1e-9) & (u <= 1 + 1e-9)
            m = np.where(ok & (t < m), t, m)
        return m

    mejor = _cortar(pista.segmentos)
    # Un pilar delante del muro TAPA su base: esa columna se queda sin
    # contacto valido, no genera uno falso.
    pilares = pista.segmentos_pilar
    if pilares:
        tp = _cortar(pilares)
        mejor = np.where(tp < mejor, np.inf, mejor)

    val = np.isfinite(mejor) & (mejor <= geo.Z_MAX_MM) & (mejor >= geo.Z_MIN_MM)
    r = mejor.copy()
    if ruido_mm and rng is not None:
        r = r + rng.normal(0, ruido_mm, ancho_px)
    X = np.where(val, r * np.sin(b), np.nan)
    Z = np.where(val, r * np.cos(b), np.nan)
    return geo.Escaneo(x=X, z=Z, valido=val,
                       y_contacto=np.zeros(ancho_px, np.int32),
                       ancho=ancho_px, alto=480)



# ---------------------------------------------------------------------------
# Vista: ver la vuelta en vez de leerla
# ---------------------------------------------------------------------------
COL_FONDO = (24, 26, 32)
COL_MURO = (210, 210, 210)
COL_CARRO = (80, 200, 255)
COL_TRAZA = (90, 200, 120)
COL_RAYO = (70, 90, 70)
COL_HIT = (0, 220, 220)
COL_ESQ = (0, 140, 255)
COL_TXT = (225, 230, 235)


def _cv2():
    import cv2
    return cv2


def hay_ventanas() -> bool:
    """True si este OpenCV puede abrir ventanas.

    El paquete `opencv-python-headless` no trae ni GUI ni codecs de video, y
    falla tarde y feo: `cv2.imshow` revienta con "The function is not
    implemented. Rebuild the library with Windows, GTK+ 2.x or Cocoa support",
    que suena a que hay que recompilar OpenCV cuando en realidad solo hay que
    instalar el paquete correcto. `requirements.txt` pide `opencv-python`, que
    es el bueno; el headless solo tiene sentido en un servidor sin pantalla.
    """
    import cv2
    if not hasattr(cv2, "imshow"):
        return False
    try:
        cv2.namedWindow("__probe__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__probe__")
        return True
    except Exception:
        return False


class Vista:
    """Dibuja la pista, el carro y lo que el carro cree que esta viendo.

    Lo util no es ver el coche dar vueltas -eso ya lo dice el resumen de
    texto- sino ver LOS DOS MUNDOS A LA VEZ: la pista de verdad y el escaneo
    que el navegador esta usando para decidir. Cuando algo va mal, casi
    siempre es que esos dos no coinciden.
    """

    def __init__(self, pista: "Pista", escala: float = 0.19, borde: int = 45,
                 panel: int = 430):
        self.pista = pista
        self.escala = escala
        self.borde = borde
        self.lado = int(LADO_PISTA * escala)
        self.mapa_ancho = self.lado + 2 * borde
        self.alto = self.lado + 2 * borde
        self.panel = panel
        self.ancho = self.mapa_ancho + panel
        # Escala del panel del robot. La limita el ANCHO: hay que ver +-1300 mm
        # a los lados, que es corredor y medio, sin que los puntos se salgan.
        self.p_lateral = 1300.0
        self.p_esc = (panel / 2 - 12) / self.p_lateral
        self.p_alcance = min(2500.0, (self.alto - 110) / self.p_esc)

    def pt(self, p):
        return (int(p[0] * self.escala) + self.borde,
                self.lado - int(p[1] * self.escala) + self.borde)

    def fondo(self, tocados=()):
        cv2 = _cv2()
        img = np.full((self.alto, self.mapa_ancho, 3), COL_FONDO, np.uint8)
        # El tapete: fuera del muro exterior no hay pista.
        cv2.rectangle(img, self.pt((0, 0)), self.pt((LADO_PISTA, LADO_PISTA)),
                      (42, 45, 52), -1)
        for (a, b) in self.pista.segmentos:
            cv2.line(img, self.pt(a), self.pt(b), COL_MURO, 3)
        for k, (px, py, color) in enumerate(self.pista.senales):
            c = (0, 0, 235) if color == obs.ROJO else (60, 210, 60)
            h = PILAR_MM / 2
            cv2.rectangle(img, self.pt((px - h, py - h)), self.pt((px + h, py + h)), c, -1)
            if k in tocados:
                # Un pilar tocado se marca y se queda marcado: si no, el carro
                # le pasa por encima en dos cuadros y no te enteras.
                cv2.circle(img, self.pt((px, py)), 11, (255, 255, 255), 2)
                cv2.drawMarker(img, self.pt((px, py)), (255, 255, 255),
                               cv2.MARKER_TILTED_CROSS, 16, 2)
        return img

    def frame(self, carro: "Carro", nav_, e, traza, inf, t):
        cv2 = _cv2()
        img = self.fondo(getattr(inf, 'senales_tocadas', ()))

        # --- lo que ve la camara, devuelto al mundo ----------------------
        ct, st = math.cos(carro.theta), math.sin(carro.theta)
        idx = np.flatnonzero(e.valido)
        for i in idx[::6]:
            zx, zz = float(e.x[i]), float(e.z[i])
            wx = carro.x + zz * ct + zx * st
            wy = carro.y + zz * st - zx * ct
            cv2.line(img, self.pt((carro.x, carro.y)), self.pt((wx, wy)), COL_RAYO, 1)
        for i in idx[::3]:
            zx, zz = float(e.x[i]), float(e.z[i])
            wx = carro.x + zz * ct + zx * st
            wy = carro.y + zz * st - zx * ct
            cv2.circle(img, self.pt((wx, wy)), 1, COL_HIT, -1)

        # --- la esquina interna que el navegador esta siguiendo ----------
        if nav_.esq_z is not None:
            wx = carro.x + nav_.esq_z * ct + nav_.esq_x * st
            wy = carro.y + nav_.esq_z * st - nav_.esq_x * ct
            cv2.drawMarker(img, self.pt((wx, wy)), COL_ESQ,
                           cv2.MARKER_TILTED_CROSS, 14, 2)
            cv2.circle(img, self.pt((wx, wy)), 5,
                       COL_ESQ if nav_.esq_medida else (0, 90, 160), 1)

        # --- por donde ha pasado -----------------------------------------
        if len(traza) > 1:
            cv2.polylines(img, [np.array([self.pt(q) for q in traza], np.int32)],
                          False, COL_TRAZA, 1, cv2.LINE_AA)

        # --- el carro -----------------------------------------------------
        esq = np.array([self.pt(q) for q in carro.esquinas()], np.int32)
        chocando = getattr(inf, "chocando", False)
        cv2.fillPoly(img, [esq], (30, 30, 110) if chocando else (40, 90, 115))
        cv2.polylines(img, [esq], True,
                      (60, 60, 255) if chocando else COL_CARRO, 2, cv2.LINE_AA)
        morro = (carro.x + ct * carro.largo_mm * 0.75,
                 carro.y + st * carro.largo_mm * 0.75)
        cv2.arrowedLine(img, self.pt((carro.x, carro.y)), self.pt(morro),
                        COL_CARRO, 2, tipLength=0.35)

        # --- telemetria ----------------------------------------------------
        d = nav_.ultimo
        lado = {-1: "izq", 0: "?", 1: "der"}[int(nav_.lado_interno)]
        m = d.metricas
        filas = [
            (f"{d.estado.upper():<11s} vuelta {nav_.vueltas}/3   giro {nav_.giros:2d}", COL_TXT),
            (f"t {t:5.1f}s   vel {d.vel:+4d}%   dir {d.direccion:+4d}%", COL_TXT),
            (f"interno {lado}   ref {nav_.ref_lateral:<4s} ancho~{nav_.ancho_corredor_mm:4.0f}", (170, 190, 210)),
            (f"izq {m.get('izq_mm', -1):4.0f}  frente {m.get('frente_mm', -1):5.0f}  der {m.get('der_mm', -1):4.0f}", (170, 190, 210)),
            (d.motivo[:52], (140, 160, 180)),
        ]
        for k, (txt, col) in enumerate(filas):
            cv2.putText(img, txt, (10, 20 + k * 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, col, 1, cv2.LINE_AA)

        avisos = []
        if inf.toco_exterior:
            avisos.append("TOCO EL MURO EXTERIOR (regla 9.18)")
        if inf.toco_senal:
            avisos.append(f"TOCO {len(getattr(inf, 'senales_tocadas', ()))} PILAR(ES)")
        for k, a in enumerate(avisos):
            cv2.putText(img, a, (10, 100 + k * 18), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (60, 60, 255), 2, cv2.LINE_AA)

        estado_col = (90, 220, 120) if not inf.toco_exterior else (60, 60, 240)
        cv2.putText(img, f"min ext {inf.min_exterior:4.0f} mm   min int {inf.min_interior:4.0f} mm",
                    (10, self.alto - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    estado_col, 1, cv2.LINE_AA)

        return np.hstack([img, self.panel_robot(nav_, e)])

    def panel_robot(self, nav_, e):
        """Lo que el carro cree que esta viendo, en SU marco.

        Este panel es la mitad util de la vista. El mapa de la izquierda es la
        verdad -que en la pista de verdad no existe- y esto es la unica
        realidad que el navegador maneja: un escaneo en milimetros. Cuando algo
        va mal, casi siempre es que los dos no coinciden, y verlos uno al lado
        del otro dice al instante si el fallo es de percepcion o de decision.
        """
        cv2 = _cv2()
        img = np.full((self.alto, self.panel, 3), (18, 20, 25), np.uint8)
        cx = self.panel // 2
        cy = self.alto - 60

        def pr(x_mm, z_mm):
            return (int(cx + x_mm * self.p_esc), int(cy - z_mm * self.p_esc))

        # rejilla cada 500 mm
        for r in range(500, int(self.p_alcance) + 1, 500):
            cv2.circle(img, (cx, cy), int(r * self.p_esc), (38, 42, 50), 1)
            cv2.putText(img, f"{r}", (cx + 4, cy - int(r * self.p_esc) + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (70, 78, 90), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy), pr(0, self.p_alcance), (38, 42, 50), 1)

        # pasillo de las ruedas
        semi = 110.0
        cv2.line(img, pr(-semi, 0), pr(-semi, self.p_alcance), (55, 60, 70), 1)
        cv2.line(img, pr(+semi, 0), pr(+semi, self.p_alcance), (55, 60, 70), 1)

        # puntos del escaneo
        idx = np.flatnonzero(e.valido)
        for i in idx:
            cv2.circle(img, pr(float(e.x[i]), float(e.z[i])), 1, COL_HIT, -1)

        # rectas ajustadas a los muros
        for lado, col in ((-1, (120, 255, 120)), (+1, (255, 170, 90))):
            r = e.recta(lado, 120.0, 900.0)
            if r is None:
                continue
            dist, ang, _n = r
            m = math.tan(math.radians(ang))
            # La recta es x = m*z + c, con |c|/sqrt(1+m^2) = dist. Un muro a la
            # IZQUIERDA tiene c negativo, o sea c = lado * dist * sqrt(1+m^2):
            # con el signo cambiado, el muro izquierdo se dibujaba a la derecha.
            x0 = lado * dist * math.sqrt(1 + m * m)
            cv2.line(img, pr(x0, 0), pr(x0 + m * 900.0, 900.0), col, 1, cv2.LINE_AA)

        # esquina interna seguida
        if nav_.esq_z is not None:
            cv2.drawMarker(img, pr(nav_.esq_x, nav_.esq_z), COL_ESQ,
                           cv2.MARKER_TILTED_CROSS, 13, 2)
            cv2.putText(img, "esquina" + ("" if nav_.esq_medida else " (estima)"),
                        (pr(nav_.esq_x, nav_.esq_z)[0] - 24,
                         pr(nav_.esq_x, nav_.esq_z)[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, COL_ESQ, 1, cv2.LINE_AA)

        # el carro
        cv2.rectangle(img, pr(-100, -150), pr(100, 150), (40, 90, 115), -1)
        cv2.rectangle(img, pr(-100, -150), pr(100, 150), COL_CARRO, 1)
        d = nav_.ultimo
        x2 = cx + int(d.direccion / 100.0 * 46)
        cv2.line(img, (cx, 30), (x2, 30), (0, 165, 255), 3)
        cv2.circle(img, (cx, 30), 3, (255, 255, 255), -1)

        cv2.putText(img, "LO QUE VE EL CARRO", (10, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 165, 185), 1, cv2.LINE_AA)
        cv2.putText(img, f"cobertura {d.metricas.get('cobertura', 0):.2f}",
                    (10, self.alto - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (150, 165, 185), 1, cv2.LINE_AA)
        return img


# ---------------------------------------------------------------------------
@dataclass
class Informe:
    ok: bool = False
    motivo: str = ""
    vueltas_reales: float = 0.0
    vueltas_contadas: int = 0
    giros: int = 0
    lado_interno: int = 0
    segundos: float = 0.0
    min_exterior: float = 1e9
    min_interior: float = 1e9
    toco_exterior: bool = False
    toco_interior: bool = False
    toco_senal: bool = False
    min_senal: float = 1e9
    senales_tocadas: set = field(default_factory=set)
    chocando: bool = False
    senales_bien: int = 0
    senales_mal: int = 0
    traza: List[Tuple[float, float]] = field(default_factory=list)

    def linea(self) -> str:
        estado = "OK  " if self.ok else "FALLA"
        sen = ""
        if self.senales_bien or self.senales_mal or self.toco_senal:
            sen = (f"  señales {self.senales_bien}ok/{self.senales_mal}mal"
                   + (f" holgura {self.min_senal:4.0f}mm" if self.min_senal < 1e8 else "")
                   + ("  TOCO PILAR" if self.toco_senal else ""))
        return (f"{estado} vueltas {self.vueltas_reales:4.2f} (contadas "
                f"{self.vueltas_contadas}) giros {self.giros:2d}  "
                f"t={self.segundos:5.1f}s  min ext {self.min_exterior:5.0f} mm  "
                f"min int {self.min_interior:5.0f} mm{sen}  {self.motivo}")


def simular(pista: Pista, sentido: str = "ccw", sector_inicio: int = 0,
            cfg: Optional[Dict[str, Any]] = None, usar_yaw: bool = True,
            hfov: float = 100.0, dt: float = 1.0 / 30.0, t_max: float = 180.0,
            ruido_mm: float = 0.0, semilla: int = 0,
            deriva_yaw_grados_s: float = 0.0,
            ver: bool = False, video: Optional[str] = None,
            fps_vista: int = 30) -> Informe:
    """Una ronda completa. `sentido` es 'ccw' (antihorario) o 'cw' (horario)."""
    cfg = cfg or robot_config.cargar()
    nav_cfg = dict(cfg["navegacion"])
    lim = dict(cfg["limites"])
    rng = np.random.default_rng(semilla)

    navegador = nav.Navegador(nav_cfg, lim)
    detector = obs.DetectorSenales(dict(cfg["obstaculos"]))

    # Colocacion inicial: en el centro del corredor de la seccion de salida y
    # mirando en el sentido de la ronda, que es lo que exige la regla 9.8.
    cx, cy = pista.centro_corredor(sector_inicio,
                                   0.15 if sentido == "ccw" else 0.85)
    rumbo = [0.0, 90.0, 180.0, 270.0][sector_inicio]
    if sentido == "cw":
        rumbo = [180.0, 270.0, 0.0, 90.0][sector_inicio]
    carro = Carro(x=cx, y=cy, theta=math.radians(rumbo),
                  mm_por_seg_a_100=float(nav_cfg.get("mm_por_seg_a_100", 900.0)))

    inf = Informe()
    if ver and not hay_ventanas():
        print("[simulador] este OpenCV no tiene interfaz grafica: viene del "
              "paquete opencv-python-HEADLESS, que no trae ni ventanas ni "
              "codecs de video.\n"
              "            pip uninstall opencv-python-headless && "
              "pip install opencv-python\n"
              "            (es lo que ya pide requirements.txt)\n"
              "            Mientras tanto sigo sin ventana; usa --video "
              "salida.gif para verlo.")
        ver = False

    vista = Vista(pista) if (ver or video) else None
    escritor = None
    gif_frames: List[Any] = []
    gif = bool(video) and str(video).lower().endswith(".gif")
    if video and vista is not None and not gif:
        import cv2
        escritor = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"),
                                   fps_vista, (vista.ancho, vista.alto))
        if not escritor.isOpened():
            # Muchos OpenCV de pip vienen sin codecs de video. En vez de
            # escribir un archivo vacio en silencio, se avisa y se pasa a GIF,
            # que solo necesita Pillow.
            print("[simulador] este OpenCV no tiene codecs de video; "
                  "grabo un GIF en su lugar (usa --video salida.gif para "
                  "evitar el aviso)")
            escritor.release()
            escritor = None
            gif = True
            video = str(pathlib.Path(video).with_suffix(".gif"))
    # Por que lado quedo cada pilar cuando el carro lo rebaso.
    lado_previo: Dict[int, float] = {}
    juzgadas: set = set()
    ang_prev = math.atan2(carro.y - LADO_PISTA / 2, carro.x - LADO_PISTA / 2)
    acum = 0.0
    t = 0.0
    deriva = 0.0

    while t < t_max:
        e = escanear_pista(pista, carro, hfov=hfov, ruido_mm=ruido_mm,
                           rng=rng if ruido_mm else None)
        yaw = None
        if usar_yaw:
            deriva += deriva_yaw_grados_s * dt
            yaw = nav._norm_angulo(carro.yaw_deg + deriva)

        # Reloj SIMULADO: sin esto el navegador leeria el reloj de pared y en
        # un bucle sin espera creeria que entre frames pasan 1.5 ms.
        # --- señales de trafico ------------------------------------------
        # Se saltan la vision por color (que se prueba aparte en [5c]) y se
        # construyen desde la posicion real, filtradas por el mismo campo de
        # vision y rango que usaria la camara. Lo que se valida aqui es la
        # GEOMETRIA del esquive y su convivencia con la logica de esquina.
        objetivo_lat = None
        objetivo_z = None
        motivo_sen = ""
        if pista.senales:
            vistas = []
            for (px, py, color) in pista.senales:
                dxw, dyw = px - carro.x, py - carro.y
                z = dxw * math.cos(carro.theta) + dyw * math.sin(carro.theta)
                xl = dxw * math.sin(carro.theta) - dyw * math.cos(carro.theta)
                if z <= 0 or abs(math.degrees(math.atan2(xl, z))) > hfov / 2:
                    continue
                vistas.append(obs.Senal(color=color, x=xl, z=z,
                                        ancho_px=20, alto_px=40, det=None))
            vistas.sort(key=lambda s: s.z)
            activa = detector.elegir(vistas)
            if navegador.estado != nav.GIRO:
                objetivo_lat = detector.objetivo(activa)
                if activa is not None:
                    objetivo_z = activa.z
                    motivo_sen = f"{activa.color} a {activa.z:.0f} mm"

        d = navegador.paso(e, yaw, objetivo_lateral=objetivo_lat,
                           motivo_extra=motivo_sen, ahora=t,
                           objetivo_z=objetivo_z)
        carro.avanzar(d.vel, d.direccion, dt)
        t += dt
        inf.traza.append((carro.x, carro.y))

        if vista is not None:
            img = vista.frame(carro, navegador, e, inf.traza, inf, t)
            if escritor is not None:
                escritor.write(img)
            if gif and len(gif_frames) < 500:
                # Un GIF de 60 s a 30 fps son 1800 cuadros y cientos de MB. Se
                # toma uno de cada cuatro, a media escala y con paleta de 64
                # colores: el resultado baja de 20 MB a 3 y se sigue viendo
                # todo lo que importa.
                if len(inf.traza) % 4 == 0:
                    import cv2
                    from PIL import Image
                    ch = cv2.resize(img, None, fx=0.5, fy=0.5,
                                    interpolation=cv2.INTER_AREA)
                    gif_frames.append(Image.fromarray(
                        cv2.cvtColor(ch, cv2.COLOR_BGR2RGB)
                    ).quantize(colors=64, method=Image.Quantize.FASTOCTREE))
            if ver:
                import cv2
                cv2.imshow("Simulador WRO  (espacio = pausa, q = salir)", img)
                tecla = cv2.waitKey(max(1, int(1000 / fps_vista))) & 0xFF
                if tecla == ord("q"):
                    inf.motivo = "interrumpido a mano"
                    break
                if tecla == ord(" "):
                    while (cv2.waitKey(50) & 0xFF) != ord(" "):
                        pass

        # --- choques -----------------------------------------------------
        inf.chocando = False
        for p in carro.esquinas():
            de = pista.dist_exterior(p)
            di = pista.dist_interior(p)
            inf.min_exterior = min(inf.min_exterior, de)
            inf.min_interior = min(inf.min_interior, di)
            if de <= 0:
                inf.toco_exterior = True
            if de <= 5 or di <= 5:
                inf.chocando = True
            if di <= 0:
                inf.toco_interior = True

        if inf.toco_exterior:
            inf.motivo = "TOCO EL MURO EXTERIOR (regla 9.18)"
            break

        # --- se paso cada señal por el lado correcto? ----------------------
        for k, (px, py, color) in enumerate(pista.senales):
            dxw, dyw = px - carro.x, py - carro.y
            z = dxw * math.cos(carro.theta) + dyw * math.sin(carro.theta)
            xl = dxw * math.sin(carro.theta) - dyw * math.cos(carro.theta)
            # Distancia del pilar al RECTANGULO del carro, no a su centro:
            # con el centro, un pilar rozando el morro no contaba y uno que
            # pasaba limpio por el costado si.
            u = max(-carro.largo_mm / 2, min(carro.largo_mm / 2, z))
            v = max(-carro.ancho_mm / 2, min(carro.ancho_mm / 2, xl))
            holgura = math.hypot(z - u, xl - v) - PILAR_MM / 2
            inf.min_senal = min(inf.min_senal, holgura)
            if holgura <= 0:
                inf.toco_senal = True
                inf.senales_tocadas.add(k)
            prev = lado_previo.get(k)
            # El pilar pasa de delante a detras: ese es el momento de juzgar.
            if prev is not None and prev > 0 >= z and k not in juzgadas:
                juzgadas.add(k)
                # ROJO -> el carro pasa por la DERECHA del pilar, o sea que el
                # pilar le queda a la IZQUIERDA (x negativa).
                bien = (xl < 0) if color == obs.ROJO else (xl > 0)
                if bien:
                    inf.senales_bien += 1
                else:
                    inf.senales_mal += 1
            lado_previo[k] = z

        # --- vueltas reales ----------------------------------------------
        a = math.atan2(carro.y - LADO_PISTA / 2, carro.x - LADO_PISTA / 2)
        acum += nav._norm_angulo(math.degrees(a - ang_prev))
        ang_prev = a
        inf.vueltas_reales = abs(acum) / 360.0

        if navegador.terminado:
            inf.motivo = "paro solo tras tres vueltas"
            break
        if inf.vueltas_reales >= 3.25:
            inf.motivo = "dio mas de 3 vueltas sin pararse"
            break

    if escritor is not None:
        # Un par de segundos de foto final para que se vea como acaba.
        img = vista.frame(carro, navegador, e, inf.traza, inf, t)
        for _ in range(fps_vista * 2):
            escritor.write(img)
        escritor.release()
    if gif and gif_frames:
        for _ in range(12):
            gif_frames.append(gif_frames[-1])
        gif_frames[0].save(video, save_all=True, append_images=gif_frames[1:],
                           duration=int(4000 / fps_vista), loop=0, optimize=True)
        print(f"[simulador] {video}  ({len(gif_frames)} cuadros)")
    if ver:
        import cv2
        cv2.destroyAllWindows()

    inf.segundos = t
    inf.vueltas_contadas = navegador.vueltas
    inf.giros = navegador.giros
    inf.lado_interno = navegador.lado_interno
    if not inf.motivo:
        inf.motivo = f"se acabo el tiempo en estado {navegador.estado}"
    inf.ok = ((not inf.toco_exterior) and inf.vueltas_reales >= 2.9
              and not inf.toco_senal and inf.senales_mal == 0)
    return inf


# ---------------------------------------------------------------------------
def dibujar(pista: Pista, informes: Sequence[Informe], ruta: str,
            escala: float = 0.18) -> None:
    import cv2
    L = int(LADO_PISTA * escala)
    borde = 30
    img = np.full((L + 2 * borde, L + 2 * borde, 3), 30, np.uint8)

    def pt(p):
        return (int(p[0] * escala) + borde, L - int(p[1] * escala) + borde)

    for (a, b) in pista.segmentos:
        cv2.line(img, pt(a), pt(b), (200, 200, 200), 2)
    colores = [(80, 220, 80), (80, 180, 255), (255, 180, 80), (200, 120, 255)]
    for k, inf in enumerate(informes):
        c = colores[k % len(colores)] if inf.ok else (60, 60, 235)
        pts = np.array([pt(p) for p in inf.traza], np.int32)
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, c, 1, cv2.LINE_AA)
    cv2.imwrite(ruta, img)


# ---------------------------------------------------------------------------
def senales_por_recta(pista: "Pista", n: int, semilla: int = 0
                      ) -> Tuple[Tuple[float, float, str], ...]:
    """Coloca n pilares por recta, alternando color y lado del carril.

    No reproduce el sorteo exacto del reglamento (asientos en T y en X); pone
    los pilares en el centro del corredor de cada recta, repartidos a lo largo,
    que es suficiente para validar que el esquive convive con la logica de
    esquina sin sacar al carro de la pista.
    """
    if n <= 0:
        return ()
    rng = np.random.default_rng(semilla)
    s_, e_, n_, o_ = pista.anchos
    # Los pilares van en la SECCION RECTA, que es justo el tramo que abarca el
    # rectangulo interior: entre las dos esquinas. Repartirlos sobre el lado
    # completo de 3000 mm los dejaba encima de las esquinas, donde el
    # reglamento no pone asientos de señal y donde el carro esta girando.
    salida = []
    for sector in range(4):
        for k in range(n):
            f = (k + 1) / (n + 1)
            color = obs.ROJO if rng.random() < 0.5 else obs.VERDE
            if sector == 0:      # sur: x recorre el rectangulo interior
                salida.append((pista.ix0 + (pista.ix1 - pista.ix0) * f, s_ / 2, color))
            elif sector == 1:    # este
                salida.append((LADO_PISTA - e_ / 2,
                               pista.iy0 + (pista.iy1 - pista.iy0) * f, color))
            elif sector == 2:    # norte
                salida.append((pista.ix1 - (pista.ix1 - pista.ix0) * f,
                               LADO_PISTA - n_ / 2, color))
            else:                # oeste
                salida.append((o_ / 2,
                               pista.iy1 - (pista.iy1 - pista.iy0) * f, color))
    return tuple(salida)


def main() -> int:
    ap = argparse.ArgumentParser(description="Vueltas a la pista sin carro")
    ap.add_argument("--sentido", choices=("ccw", "cw"), default="ccw")
    ap.add_argument("--anchos", type=float, nargs=4, default=[1000, 1000, 1000, 1000],
                    help="corredores sur este norte oeste (600 o 1000)")
    ap.add_argument("--sector", type=int, default=0, choices=(0, 1, 2, 3))
    ap.add_argument("--hfov", type=float, default=100.0)
    ap.add_argument("--sin-yaw", action="store_true", help="como si fallara la IMU")
    ap.add_argument("--deriva", type=float, default=0.0,
                    help="deriva del giroscopio en grados/s")
    ap.add_argument("--ruido", type=float, default=0.0, help="ruido del escaneo en mm")
    ap.add_argument("--todas", action="store_true",
                    help="barre sentidos, sectores y combinaciones de ancho")
    ap.add_argument("--traza", default=None, help="PNG con las trayectorias")
    ap.add_argument("--ver", action="store_true",
                    help="ventana en vivo: la pista, el carro y lo que el carro ve")
    ap.add_argument("--video", default=None,
                    help="graba la vuelta (.mp4 si hay codecs, .gif siempre)")
    ap.add_argument("--fps", type=int, default=30, help="velocidad de la vista")
    ap.add_argument("--senales", type=int, default=0,
                    help="pilares por recta para el reto de obstaculos (0 = ninguno)")
    args = ap.parse_args()

    cfg = robot_config.cargar()
    casos: List[Tuple[str, Pista, str, int]] = []
    if args.todas:
        for anchos in itertools.product(ANCHOS_VALIDOS, repeat=4):
            for sentido in ("ccw", "cw"):
                p = Pista(anchos)
                if args.senales:
                    p = Pista(anchos, senales_por_recta(p, args.senales))
                casos.append((f"{sentido} {'/'.join(str(int(a)) for a in anchos)}",
                              p, sentido, 0))
    else:
        p = Pista(tuple(args.anchos))
        if args.senales:
            p = Pista(tuple(args.anchos), senales_por_recta(p, args.senales))
        casos.append((f"{args.sentido} {'/'.join(str(int(a)) for a in args.anchos)}",
                      p, args.sentido, args.sector))

    informes = []
    fallos = 0
    for nombre, pista, sentido, sector in casos:
        inf = simular(pista, sentido=sentido, sector_inicio=sector, cfg=cfg,
                      usar_yaw=not args.sin_yaw, hfov=args.hfov,
                      ruido_mm=args.ruido, deriva_yaw_grados_s=args.deriva,
                      ver=args.ver and not args.todas,
                      video=args.video if not args.todas else None,
                      fps_vista=args.fps)
        informes.append(inf)
        if not inf.ok:
            fallos += 1
        print(f"{nombre:>28} | {inf.linea()}")

    print(f"\n{len(casos) - fallos}/{len(casos)} vueltas completas sin tocar el exterior")
    if args.traza:
        dibujar(casos[0][1], informes, args.traza)
        print(f"traza en {args.traza}")
    return 1 if fallos else 0


if __name__ == "__main__":
    raise SystemExit(main())
