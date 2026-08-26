"""
obstaculos.py — Señales de trafico verdes y rojas (Obstacle Challenge).

============================================================================
LA REGLA, QUE ES FACIL DE ENTENDER AL REVES
============================================================================
Reglamento 2026, seccion 5 y regla 9.19:

    "The traffic sign to keep to the RIGHT side of the lane is a RED pillar."
    "The traffic sign to keep to the LEFT side of the lane is a GREEN pillar."

O sea, el color dice por que lado del CARRIL debe ir el carro, no por que
lado queda el pilar. Traducido a lo que hay que programar:

    ROJO   -> el carro pasa por la DERECHA del pilar (el pilar le queda a la
              izquierda).
    VERDE  -> el carro pasa por la IZQUIERDA del pilar (el pilar le queda a
              la derecha).

Es el error clasico de esta categoria y cuesta la ronda entera: si te lo
inviertes, la regla 9.24.5 termina la ronda en cuanto cruzas la linea de esa
señal por el lado equivocado.

============================================================================
DISTANCIA: EL PILAR SE MIDE POR LA BASE
============================================================================
Un pilar mide 50x50x100 mm. Su borde INFERIOR es donde toca el suelo, asi que
proyectando `base_y` con la misma homografia del muro sale su posicion real
en milimetros. El centro del bbox NO sirve: cambia de altura con la distancia
y con lo que se vea del pilar.

Se usa el centro horizontal del bbox y su borde inferior: (cx, base_y).

============================================================================
QUE NO HACE ESTE MODULO
============================================================================
No decide velocidad ni gestiona esquinas. Solo dice "el carro deberia estar a
X mm lateralmente". La logica de giro de 90 grados del modo abierto sigue
siendo la misma y manda por encima de esto: en cuanto el navegador entra en
GIRO, el objetivo lateral se ignora. Un pilar no cambia como se toma una
esquina.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from . import geometria as geo
from .vision import Deteccion

ROJO = "rojo"
VERDE = "verde"

# Por que lado del PILAR pasa el carro. +1 = el carro va por la derecha.
LADO_PASO = {ROJO: +1, VERDE: -1}


@dataclass
class Senal:
    color: str
    x: float             # posicion lateral del pilar en mm, + = derecha
    z: float             # distancia hacia adelante en mm
    ancho_px: int
    alto_px: int
    det: Deteccion

    @property
    def lado_paso(self) -> int:
        """+1 = el carro debe pasar por la derecha del pilar."""
        return LADO_PASO.get(self.color, +1)

    def objetivo_lateral(self, margen_mm: float) -> float:
        """Donde tiene que estar el carro, en mm. + = a la derecha."""
        return self.x + self.lado_paso * margen_mm


class DetectorSenales:
    """Convierte detecciones de color en señales con posicion metrica."""

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.activa: Optional[Senal] = None
        self.ultimo_color: str = ""

    # ---------------------------------------------------------------
    def desde_detecciones(self, dets: Dict[str, List[Deteccion]], suelo: geo.Suelo,
                          ancho: int, alto: int) -> List[Senal]:
        cfg = self.cfg
        z_min = float(cfg.get("senal_z_min_mm", 120.0))
        z_max = float(cfg.get("senal_z_max_mm", 1600.0))
        alto_min = int(cfg.get("senal_alto_min_px", 10))
        aspecto_min = float(cfg.get("senal_aspecto_min", 1.1))

        salida: List[Senal] = []
        for color in (ROJO, VERDE):
            for d in dets.get(color, []):
                if d.h < alto_min:
                    continue
                # Un pilar es 50x50x100: siempre mas alto que ancho. Filtra
                # camisetas, cintas del suelo y reflejos alargados.
                if d.w > 0 and (d.h / float(d.w)) < aspecto_min:
                    continue
                X, Z = suelo.proyectar(np.array([float(d.cx)]),
                                       np.array([float(d.base_y)]), ancho, alto)
                x, z = float(X[0]), float(Z[0])
                if not (math.isfinite(x) and math.isfinite(z)):
                    continue
                if not (z_min <= z <= z_max):
                    continue
                salida.append(Senal(color=color, x=x, z=z, ancho_px=d.w,
                                    alto_px=d.h, det=d))
        salida.sort(key=lambda s: s.z)
        return salida

    # ---------------------------------------------------------------
    def elegir(self, senales: Sequence[Senal]) -> Optional[Senal]:
        """La señal activa es la mas cercana que aun esta por delante.

        Se mantiene la eleccion mientras siga viendose para no oscilar entre
        dos pilares que estan casi a la misma distancia: cambiar de objetivo a
        media maniobra es como se acaba tocando uno.
        """
        cfg = self.cfg
        z_soltar = float(cfg.get("senal_z_soltar_mm", 220.0))
        # ---------------------------------------------------------------
        # QUE SEÑALES HAY QUE ATENDER
        #
        # La pregunta NO es "¿me estorba?" sino "¿estoy ya del lado bueno?".
        # La regla 9.19 no pide esquivar el pilar, pide pasarlo por un lado
        # concreto. Un pilar rojo 350 mm a la DERECHA no estorba lo mas minimo
        # y aun asi hay que actuar, porque tal como vamos lo pasariamos por su
        # izquierda y eso termina la ronda (regla 9.24.5). Y al reves: uno rojo
        # a la izquierda ya esta resuelto aunque este cerca.
        #
        #   ROJO  -> el carro tiene que quedar a la DERECHA: hace falta
        #            x_pilar <= -margen  (el pilar a nuestra izquierda)
        #   VERDE -> al contrario.
        #
        # Aparte, un pilar a mas de `senal_x_max_mm` de lado esta en otro
        # corredor, visto de reojo por encima de la esquina: ese se ignora.
        # ---------------------------------------------------------------
        x_max = float(cfg.get("senal_x_max_mm", 650.0))
        margen = float(cfg.get("senal_margen_mm", 240.0))
        senales = [s for s in senales
                   if abs(s.x) <= x_max and (s.lado_paso * (-s.x)) < margen]
        if not senales:
            self.activa = None
            return None

        if self.activa is not None:
            cerca = [s for s in senales
                     if s.color == self.activa.color
                     and abs(s.x - self.activa.x) < float(cfg.get("senal_salto_mm", 260.0))
                     and abs(s.z - self.activa.z) < float(cfg.get("senal_salto_mm", 260.0))]
            if cerca:
                s = cerca[0]
                if s.z <= z_soltar:      # ya la tenemos encima: soltarla
                    self.activa = None
                    self.ultimo_color = s.color
                    return None
                self.activa = s
                return s

        s = senales[0]
        if s.z <= z_soltar:
            self.activa = None
            return None
        self.activa = s
        self.ultimo_color = s.color
        return s

    # ---------------------------------------------------------------
    def objetivo(self, senal: Optional[Senal]) -> Optional[float]:
        """Objetivo lateral en mm, o None si no hay nada que esquivar."""
        if senal is None:
            return None
        cfg = self.cfg
        # Semiancho del carro (100) + semiancho del pilar (25) + holgura.
        margen = float(cfg.get("senal_margen_mm", 190.0))
        obj = senal.objetivo_lateral(margen)
        # No salirse del corredor por esquivar: se recorta a un maximo.
        tope = float(cfg.get("senal_desvio_max_mm", 380.0))
        return float(max(-tope, min(tope, obj)))


# ---------------------------------------------------------------------------
def dibujar_senales(frame: np.ndarray, senales: Sequence[Senal],
                    activa: Optional[Senal]) -> np.ndarray:
    for s in senales:
        c = (0, 0, 255) if s.color == ROJO else (0, 210, 0)
        d = s.det
        es_activa = activa is not None and d is activa.det
        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), c, 3 if es_activa else 1)
        flecha = "->" if s.lado_paso > 0 else "<-"
        cv2.putText(frame, f"{flecha}{s.z:.0f}", (d.x, max(12, d.y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)
    if activa is not None:
        cv2.putText(frame,
                    f"señal {activa.color} a {activa.z:.0f} mm: paso por "
                    f"{'DERECHA' if activa.lado_paso > 0 else 'IZQUIERDA'}",
                    (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 255) if activa.color == ROJO else (0, 210, 0), 1, cv2.LINE_AA)
    return frame
