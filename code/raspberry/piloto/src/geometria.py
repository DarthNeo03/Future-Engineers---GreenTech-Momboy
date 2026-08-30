"""
geometria.py — De pixeles a milimetros sobre el plano del suelo.

La camara esta a ALTURA fija (125 mm) e INCLINADA hacia abajo un angulo fijo
(7.5 grados). Con eso, cada fila de la imagen corresponde a UNA distancia sobre
el suelo, y cada columna a un desplazamiento lateral proporcional. No hace
falta homografia con patron de ajedrez: basta la altura, la inclinacion y la
focal en pixeles, que se calibra con UN clic sobre un objeto a distancia
conocida (pestaña Calibracion de la web).

Modelo (pinhole, sin distorsion de lente):

    angulo bajo el horizonte de la fila v:   theta = inclinacion + atan((v-cy)/fy)
    distancia sobre el suelo:                Y = altura / tan(theta)
    rango inclinado hasta ese punto:         R = altura / sin(theta)
    desplazamiento lateral:                  X = (u-cx)/fx * R

Consecuencia importante que arregla el error del programa viejo ("detecta
objetos por ARRIBA de las paredes"): la fila del horizonte es fija. Todo lo
que este por encima de ella no puede ser pista (los muros miden 100 mm y la
camara esta a 125: hasta el borde superior del muro queda siempre bajo el
horizonte). Sillas, mesas y publico quedan cortados por geometria, no por
color.

NOTA sobre lentes anchas: si la camara tiene mucha distorsion de barril, el
modelo pinhole se desvia en los bordes. Para navegar (comparar izquierda vs
derecha, medir el pasillo) es suficiente; para medir con precision en las
esquinas de la imagen, calibrar fx con un objeto lateral y dejar margen.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import numpy as np

DIST_MAX_MM = 6000.0     # mas alla de esto se reporta "infinito util"


class Geometria:
    """Todas las conversiones dependen de cfg (dict vivo de params.json):

        alto_cam_mm      altura del centro optico sobre el suelo (125)
        inclinacion_deg  inclinacion hacia abajo desde la horizontal (7.5)
        fy_px, fx_px     focales en pixeles (se calibran con un clic)
        ancho_carro_mm   ancho total del carro con ruedas (200)
        margen_ruedas_mm margen extra por lado para el corredor
        adelanto_cam_mm  distancia del eje delantero a la camara (para que
                         "distancia al muro" sea desde el morro, no la lente)
    """

    def __init__(self, cfg: Dict[str, Any], ancho_img: int = 640, alto_img: int = 480):
        self.cfg = cfg
        self.W = int(ancho_img)
        self.H = int(alto_img)

    def redimensionar(self, ancho_img: int, alto_img: int) -> None:
        self.W = int(ancho_img)
        self.H = int(alto_img)

    # -- parametros ------------------------------------------------------
    @property
    def _h(self) -> float:
        return float(self.cfg.get("alto_cam_mm", 125.0))

    @property
    def _tilt(self) -> float:
        return math.radians(float(self.cfg.get("inclinacion_deg", 7.5)))

    @property
    def _fy(self) -> float:
        # escala con la resolucion: fy se guarda referido a 480 de alto
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

    # -- filas <-> distancias --------------------------------------------
    def fila_horizonte(self) -> int:
        """Fila de la imagen donde esta el horizonte. Por encima: NO es pista."""
        return int(round(self._cy - self._fy * math.tan(self._tilt)))

    def fila_a_distancia(self, v) -> np.ndarray:
        """Fila(s) de imagen -> distancia sobre el suelo en mm (desde la camara).
        Acepta escalar o array. Filas en el horizonte o encima -> DIST_MAX_MM."""
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        with np.errstate(divide="ignore", invalid="ignore"):
            d = self._h / np.tan(theta)
        d = np.where((theta <= 1e-4) | ~np.isfinite(d), DIST_MAX_MM, d)
        return np.clip(d, 0.0, DIST_MAX_MM)

    def distancia_a_fila(self, d_mm: float) -> int:
        """Distancia sobre el suelo (mm) -> fila de imagen."""
        d_mm = max(1.0, float(d_mm))
        theta = math.atan2(self._h, d_mm)
        av = theta - self._tilt
        return int(round(self._cy + self._fy * math.tan(av)))

    def lateral_mm(self, u, v) -> np.ndarray:
        """Columna(s)+fila(s) -> desplazamiento lateral en mm (+derecha)."""
        u = np.asarray(u, dtype=np.float32)
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        theta = np.maximum(theta, 1e-3)
        rango = self._h / np.sin(theta)
        return (u - self._cx) / self._fx * rango

    def punto_suelo(self, u: float, v: float) -> Tuple[float, float]:
        """Pixel -> (x_lateral_mm, y_adelante_mm) sobre el suelo."""
        y = float(self.fila_a_distancia(v))
        x = float(self.lateral_mm(u, v))
        return x, y

    def suelo_a_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """(lateral, adelante) en mm -> pixel. Inverso de punto_suelo."""
        v = self.distancia_a_fila(y_mm)
        theta = math.atan2(self._h, max(1.0, y_mm))
        rango = self._h / math.sin(theta)
        u = int(round(self._cx + x_mm * self._fx / rango))
        return u, v

    # -- corredor de las ruedas ------------------------------------------
    def corredor_en_fila(self, v: float) -> Tuple[int, int]:
        """Columnas (izq, der) por donde pasa el carro a la distancia de esa
        fila. La camara no ve las ruedas: esto las proyecta."""
        semi = (float(self.cfg.get("ancho_carro_mm", 200.0)) / 2.0 +
                float(self.cfg.get("margen_ruedas_mm", 30.0)))
        theta = self._tilt + math.atan((v - self._cy) / self._fy)
        theta = max(theta, 1e-3)
        rango = self._h / math.sin(theta)
        dx = semi * self._fx / rango
        return int(round(self._cx - dx)), int(round(self._cx + dx))

    def poligono_corredor(self, y_cerca_mm: float = 120.0,
                          y_lejos_mm: float = 2500.0, pasos: int = 12) -> np.ndarray:
        """Poligono (en pixeles) del pasillo que va a barrer el carro si sigue
        recto. Para dibujarlo y para recortar objetivos de esquive."""
        ys = np.linspace(y_cerca_mm, y_lejos_mm, pasos)
        izq, der = [], []
        for y in ys:
            v = self.distancia_a_fila(y)
            a, b = self.corredor_en_fila(v)
            izq.append((a, v))
            der.append((b, v))
        return np.array(izq + der[::-1], dtype=np.int32)

    # -- calibracion ------------------------------------------------------
    def calibrar_fy(self, v_clic: float, distancia_mm: float) -> float:
        """El usuario pone un objeto a distancia_mm del morro de la camara y
        hace clic en el punto donde TOCA EL SUELO. Devuelve el fy resuelto
        (referido a 480 de alto); el llamador decide si guardarlo."""
        d = max(50.0, float(distancia_mm))
        theta = math.atan2(self._h, d)
        av = theta - self._tilt
        if abs(math.tan(av)) < 1e-6 or abs(v_clic - self._cy) < 1.0:
            raise ValueError("clic demasiado cerca del horizonte para resolver fy")
        fy = (float(v_clic) - self._cy) / math.tan(av)
        if fy <= 50 or fy > 5000:
            raise ValueError(f"fy={fy:.0f} fuera de rango: revisa distancia/altura/inclinacion")
        return fy * (480.0 / self.H)

    def calibrar_fx(self, u_clic: float, v_clic: float, lateral_mm: float) -> float:
        """Objeto a un desplazamiento lateral conocido (mm, + a la derecha):
        clic en su base. Devuelve el fx resuelto (referido a 640 de ancho)."""
        if abs(lateral_mm) < 20:
            raise ValueError("usa un objeto claramente a un lado (>= 5 cm)")
        theta = self._tilt + math.atan((float(v_clic) - self._cy) / self._fy)
        if theta <= 1e-3:
            raise ValueError("el clic quedo en o sobre el horizonte")
        rango = self._h / math.sin(theta)
        fx = (float(u_clic) - self._cx) * rango / float(lateral_mm)
        if fx <= 50 or fx > 5000:
            raise ValueError(f"fx={fx:.0f} fuera de rango")
        return fx * (640.0 / self.W)

    # -- info -------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "horizonte": self.fila_horizonte(),
            "fy_px": round(float(self.cfg.get("fy_px", 460.0)), 1),
            "fx_px": round(float(self.cfg.get("fx_px", 460.0)), 1),
            "alto_cam_mm": self._h,
            "inclinacion_deg": float(self.cfg.get("inclinacion_deg", 7.5)),
            "dist_centro_mm": round(float(self.fila_a_distancia(self._cy)), 0),
            "dist_abajo_mm": round(float(self.fila_a_distancia(self.H - 1)), 0),
        }
