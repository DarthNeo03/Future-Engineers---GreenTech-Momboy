"""
geometria.py — De pixeles a milimetros sobre el plano del suelo.

UNA CAMARA COMO SENSOR DE DISTANCIA. No hay ultrasonicos ni LIDAR: la camara
esta a ALTURA fija sobre el suelo e INCLINADA hacia abajo un angulo fijo. Con
esas dos constantes y la focal en pixeles, cada fila de la imagen corresponde
a UNA distancia sobre el piso y cada columna a un desplazamiento lateral
proporcional. No hace falta homografia con patron de ajedrez: basta un clic
sobre un objeto a distancia conocida para resolver la focal.

    angulo bajo el horizonte de la fila v:  theta = inclinacion + atan((v-cy)/fy)
    distancia sobre el suelo:               Y = altura / tan(theta)
    rango inclinado hasta ese punto:        R = altura / sin(theta)
    desplazamiento lateral:                 X = (u-cx)/fx * R

CONSECUENCIA QUE ARREGLA EL FALLO CLASICO ("el carro ve obstaculos en el
publico"): la fila del horizonte es FIJA y calculable. Todo lo que este por
encima de ella no puede ser pista. Los muros miden 100 mm y la camara va a
125: hasta el borde superior del muro queda siempre por debajo del horizonte.
Las sillas, las mesas y las camisetas rojas del equipo de al lado se descartan
por GEOMETRIA, no por color, que es mucho mas robusto.

SEGUNDA MEDIDA, INDEPENDIENTE: la ALTURA APARENTE. Las señales de transito
miden 100 mm exactos (reglamento 13.1). Si un pilar se ve completo, su altura
en pixeles da la distancia sin usar el suelo para nada. Tener dos medidas que
se pueden contrastar es lo que permite descartar un reflejo o una mancha: si
la distancia por base y la distancia por altura no se parecen, no es un pilar.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Tuple

import numpy as np

DIST_MAX_MM = 6000.0        # mas alla de esto se reporta "infinito util"
ALTO_PILAR_MM = 100.0       # reglamento 13.1: 50 x 50 x 100 mm


class Geometria:
    """Todas las conversiones salen de cfg (dict vivo de pista.json):

        alto_cam_mm       altura del centro optico sobre el suelo
        inclinacion_deg   inclinacion hacia abajo desde la horizontal
        fy_px, fx_px      focales en pixeles, referidas a 640x480
        ancho_carro_mm    ancho total con ruedas
        margen_ruedas_mm  margen extra por lado para el corredor
    """

    def __init__(self, cfg: Dict[str, Any], ancho_img: int = 640,
                 alto_img: int = 480) -> None:
        self.cfg = cfg
        self.W = int(ancho_img)
        self.H = int(alto_img)

    def redimensionar(self, ancho_img: int, alto_img: int) -> None:
        self.W, self.H = int(ancho_img), int(alto_img)

    # ---------------------------------------------------------- parametros
    @property
    def _h(self) -> float:
        return float(self.cfg.get("alto_cam_mm", 125.0))

    @property
    def _tilt(self) -> float:
        return math.radians(float(self.cfg.get("inclinacion_deg", 7.5)))

    @property
    def _fy(self) -> float:
        # Las focales se guardan referidas a 640x480 y escalan con el tamaño
        # real del frame, para poder bajar de resolucion sin recalibrar.
        return float(self.cfg.get("fy_px", 460.0)) * (self.H / 480.0)

    @property
    def _fx(self) -> float:
        return float(self.cfg.get("fx_px", 460.0)) * (self.W / 640.0)

    @property
    def _cx(self) -> float:
        return self.W / 2.0

    @property
    def _cy(self) -> float:
        return self.H / 2.0

    # ----------------------------------------------- filas <-> distancias
    def fila_horizonte(self) -> int:
        """Fila donde esta el horizonte. Por encima: NO es pista, nunca."""
        return int(round(self._cy - self._fy * math.tan(self._tilt)))

    def fila_a_distancia(self, v) -> np.ndarray:
        """Fila(s) -> distancia sobre el suelo en mm desde la camara.
        Acepta escalar o array. En el horizonte o encima -> DIST_MAX_MM."""
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        with np.errstate(divide="ignore", invalid="ignore"):
            d = self._h / np.tan(theta)
        d = np.where((theta <= 1e-4) | ~np.isfinite(d), DIST_MAX_MM, d)
        return np.clip(d, 0.0, DIST_MAX_MM)

    def distancia_a_fila(self, d_mm: float) -> int:
        d_mm = max(1.0, float(d_mm))
        theta = math.atan2(self._h, d_mm)
        return int(round(self._cy + self._fy * math.tan(theta - self._tilt)))

    def lateral_mm(self, u, v) -> np.ndarray:
        """Columna(s)+fila(s) -> desplazamiento lateral en mm (+ a la derecha)."""
        u = np.asarray(u, dtype=np.float32)
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        theta = np.maximum(theta, 1e-3)
        rango = self._h / np.sin(theta)
        return (u - self._cx) / self._fx * rango

    def punto_suelo(self, u: float, v: float) -> Tuple[float, float]:
        """Pixel -> (lateral_mm, adelante_mm) sobre el suelo."""
        return float(self.lateral_mm(u, v)), float(self.fila_a_distancia(v))

    def suelo_a_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """Inverso de punto_suelo, para dibujar sobre el frame."""
        v = self.distancia_a_fila(y_mm)
        theta = math.atan2(self._h, max(1.0, y_mm))
        rango = self._h / math.sin(theta)
        u = int(round(self._cx + x_mm * self._fx / rango))
        return u, v

    # ------------------------------------------- segunda medida: la altura
    def distancia_por_altura(self, alto_px: float,
                             alto_real_mm: float = ALTO_PILAR_MM) -> float:
        """Distancia deducida del tamaño aparente. Independiente del suelo, y
        por eso util justo cuando la base del pilar queda tapada o cortada por
        el borde inferior del frame."""
        if alto_px <= 1.0:
            return DIST_MAX_MM
        return min(DIST_MAX_MM, alto_real_mm * self._fy / float(alto_px))

    def coherente(self, dist_base_mm: float, dist_alto_mm: float,
                  tolerancia: float = 0.45) -> bool:
        """Las dos medidas deben parecerse. Si no, el contorno no es un pilar
        de pie sobre el tapete: es un reflejo, una sombra o algo del publico."""
        if dist_base_mm >= DIST_MAX_MM or dist_alto_mm >= DIST_MAX_MM:
            return False
        ref = max(1.0, min(dist_base_mm, dist_alto_mm))
        return abs(dist_base_mm - dist_alto_mm) / ref <= tolerancia

    # -------------------------------------------------- corredor del carro
    def semiancho_mm(self) -> float:
        return (float(self.cfg.get("ancho_carro_mm", 200.0)) / 2.0 +
                float(self.cfg.get("margen_ruedas_mm", 30.0)))

    def corredor_en_fila(self, v: float) -> Tuple[int, int]:
        """Columnas (izq, der) por donde pasara el carro a la distancia de esa
        fila. La camara no ve sus propias ruedas: esto las proyecta."""
        theta = max(self._tilt + math.atan((v - self._cy) / self._fy), 1e-3)
        rango = self._h / math.sin(theta)
        dx = self.semiancho_mm() * self._fx / rango
        return int(round(self._cx - dx)), int(round(self._cx + dx))

    def poligono_corredor(self, y_cerca_mm: float = 120.0,
                          y_lejos_mm: float = 2500.0,
                          pasos: int = 12) -> np.ndarray:
        """Pasillo que barrera el carro si sigue recto, en pixeles. Sirve para
        dibujarlo en el panel y para decidir si un pilar esta o no en la
        trayectoria actual."""
        ys = np.linspace(y_cerca_mm, y_lejos_mm, pasos)
        izq, der = [], []
        for y in ys:
            v = self.distancia_a_fila(y)
            a, b = self.corredor_en_fila(v)
            izq.append((a, v))
            der.append((b, v))
        return np.array(izq + der[::-1], dtype=np.int32)

    # ---------------------------------------------------------- calibracion
    def calibrar_fy(self, v_clic: float, distancia_mm: float) -> float:
        """Se pone un objeto a distancia conocida y se hace clic donde TOCA EL
        SUELO. Devuelve el fy resuelto (referido a 480 de alto)."""
        d = max(50.0, float(distancia_mm))
        av = math.atan2(self._h, d) - self._tilt
        if abs(math.tan(av)) < 1e-6 or abs(v_clic - self._cy) < 1.0:
            raise ValueError("el clic quedo demasiado cerca del horizonte")
        fy = (float(v_clic) - self._cy) / math.tan(av)
        if not (50 < fy < 5000):
            raise ValueError(f"fy={fy:.0f} fuera de rango: revisa altura o inclinacion")
        return fy * (480.0 / self.H)

    def estado(self) -> Dict[str, Any]:
        return {
            "horizonte": self.fila_horizonte(),
            "fy_px": round(float(self.cfg.get("fy_px", 460.0)), 1),
            "fx_px": round(float(self.cfg.get("fx_px", 460.0)), 1),
            "alto_cam_mm": self._h,
            "inclinacion_deg": float(self.cfg.get("inclinacion_deg", 7.5)),
            "dist_fila_abajo_mm": round(float(self.fila_a_distancia(self.H - 1))),
        }
