"""
vision.py — Deteccion de objetos por color para el WRO Future Engineers.

Que mejora respecto al enfoque anterior ("cada mancha es un nucleo y las que se
tocan son un mismo objeto"):

1. Una sola conversion a HSV por frame, compartida por todos los colores.
2. Mascara multi-rango: el rojo se arma con dos intervalos porque el tono se
   envuelve en H=0/179. Se hace con cv2.inRange + cv2.bitwise_or.
3. Morfologia en dos pasos: OPEN mata las motas de 1-3 px; CLOSE tapa el hueco
   que deja el brillo especular en el centro del pilar.
4. FUSION POR HUECO (unir_huecos): en vez de exigir que las manchas se toquen,
   se dilata una COPIA de la mascara y se etiqueta sobre esa copia; las manchas
   separadas por <= 2*unir_huecos px caen en la misma etiqueta. La geometria
   (area, bbox, centro) se mide de vuelta sobre la mascara ORIGINAL, asi que el
   objeto no se infla. Esto arregla el pilar partido en dos por un reflejo.
5. Filtros por objeto: area, llenado (area/bbox), relacion de aspecto, ancho y
   alto minimos, y una ROI vertical para ignorar la parte alta de la imagen
   (publico, luces, cosas fuera de la pista).
6. Todo el recorrido pesado se hace con connectedComponentsWithStats y se
   trabaja recortando por el bounding box de cada etiqueta, no sobre la imagen
   completa. En 640x480 con 3 colores esto va sobrado en una Pi 5.

Uso:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    det = DetectorColor("rojo", params)
    objetos, mascara = det.detectar(hsv)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------------
@dataclass
class Deteccion:
    color: str
    x: int
    y: int
    w: int
    h: int
    area: int            # pixeles reales encendidos (no el area del bbox)
    llenado: float       # area / (w*h)  -> 1.0 = rectangulo perfecto
    aspecto: float       # h / w
    cx: float = 0.0      # centroide real de la mancha
    cy: float = 0.0
    contorno: Optional[np.ndarray] = None

    @property
    def base_y(self) -> int:
        """Y del borde inferior. Proxy de distancia: mas abajo = mas cerca."""
        return self.y + self.h

    @property
    def centro(self) -> Tuple[int, int]:
        return int(round(self.cx)), int(round(self.cy))

    @property
    def bbox(self) -> Tuple[int, int, int, int]:
        return self.x, self.y, self.w, self.h

    def desviacion(self, ancho_img: int) -> float:
        """-1.0 = pegado a la izquierda, 0 = centrado, +1.0 = a la derecha."""
        return (self.cx - ancho_img / 2.0) / (ancho_img / 2.0)

    def __repr__(self) -> str:
        return (f"<{self.color} bbox=({self.x},{self.y},{self.w},{self.h}) "
                f"area={self.area} llenado={self.llenado:.2f} asp={self.aspecto:.2f}>")


# --------------------------------------------------------------------------
@lru_cache(maxsize=64)
def kernel_elipse(n: int) -> np.ndarray:
    n = max(1, int(n))
    if n % 2 == 0:
        n += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (n, n))


def mascara_hsv(hsv: np.ndarray, rangos: Sequence) -> np.ndarray:
    """OR de todos los intervalos HSV del color. Sin asignaciones extra."""
    salida: Optional[np.ndarray] = None
    for bajo, alto in rangos:
        m = cv2.inRange(hsv,
                        np.array(bajo, dtype=np.uint8),
                        np.array(alto, dtype=np.uint8))
        salida = m if salida is None else cv2.bitwise_or(salida, m, dst=salida)
    if salida is None:
        salida = np.zeros(hsv.shape[:2], dtype=np.uint8)
    return salida


# --------------------------------------------------------------------------
class DetectorColor:
    """Detector de un color. Los parametros vienen de color_config."""

    def __init__(self, nombre: str, params: Dict[str, Any]):
        self.nombre = nombre
        self.params: Dict[str, Any] = dict(params)

    def actualizar(self, params: Dict[str, Any]) -> None:
        self.params = dict(params)

    # ---- pipeline -------------------------------------------------------
    def construir_mascara(self, hsv: np.ndarray) -> np.ndarray:
        """HSV (ya recortado a la ROI) -> mascara binaria limpia."""
        p = self.params
        d = int(p.get("desenfoque", 0) or 0)
        if d >= 3:
            hsv = cv2.medianBlur(hsv, d if d % 2 else d + 1)

        m = mascara_hsv(hsv, p.get("rangos", []))

        ka = int(p.get("abrir", 0) or 0)
        if ka >= 3:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel_elipse(ka))
        kc = int(p.get("cerrar", 0) or 0)
        if kc >= 3:
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel_elipse(kc))
        return m

    def detectar(self,
                 hsv: np.ndarray,
                 con_contorno: bool = False) -> Tuple[List[Deteccion], np.ndarray]:
        """Devuelve (detecciones ordenadas por area desc, mascara tamaño completo)."""
        p = self.params
        alto_img, ancho_img = hsv.shape[:2]

        # --- ROI vertical: solo procesamos la franja util -----------------
        y0 = int(max(0.0, min(0.99, float(p.get("roi_arriba", 0.0)))) * alto_img)
        y1 = int(max(0.01, min(1.0, float(p.get("roi_abajo", 1.0)))) * alto_img)
        y1 = max(y0 + 1, min(alto_img, y1))

        sub_hsv = hsv[y0:y1]
        sub_mask = self.construir_mascara(sub_hsv)

        # Mascara a tamaño completo para poder mostrarla / combinarla
        mascara = np.zeros((alto_img, ancho_img), dtype=np.uint8)
        mascara[y0:y1] = sub_mask

        # --- fusion por hueco ---------------------------------------------
        g = int(p.get("unir_huecos", 0) or 0)
        if g > 0:
            conectada = cv2.dilate(sub_mask, kernel_elipse(2 * g + 1))
        else:
            conectada = sub_mask

        n, etiquetas, stats, _ = cv2.connectedComponentsWithStats(conectada, 8, cv2.CV_32S)

        area_min = int(p.get("area_min", 0))
        area_max = int(p.get("area_max", 10 ** 9))
        llenado_min = float(p.get("llenado_min", 0.0))
        usar_asp = bool(p.get("usar_aspecto", True))
        asp_min = float(p.get("aspecto_min", 0.0))
        asp_max = float(p.get("aspecto_max", 1e9))
        ancho_min = int(p.get("ancho_min", 0))
        alto_min = int(p.get("alto_min", 0))

        detecciones: List[Deteccion] = []
        for i in range(1, n):
            bx, by, bw, bh, area_dilatada = stats[i]
            # Descarte barato antes de tocar pixeles: la mancha dilatada nunca
            # puede ser menor que la real.
            if area_dilatada < area_min:
                continue

            # Trabajamos solo dentro del bbox de la etiqueta.
            rec_lab = etiquetas[by:by + bh, bx:bx + bw]
            rec_msk = sub_mask[by:by + bh, bx:bx + bw]
            propio = (rec_lab == i) & (rec_msk > 0)

            area = int(np.count_nonzero(propio))
            if area < area_min or area > area_max:
                continue

            ys, xs = np.nonzero(propio)
            if ys.size == 0:
                continue
            rx0, rx1 = int(xs.min()), int(xs.max())
            ry0, ry1 = int(ys.min()), int(ys.max())
            w = rx1 - rx0 + 1
            h = ry1 - ry0 + 1
            if w < ancho_min or h < alto_min:
                continue

            llenado = area / float(w * h)
            if llenado < llenado_min:
                continue

            aspecto = h / float(w)
            if usar_asp and not (asp_min <= aspecto <= asp_max):
                continue

            det = Deteccion(
                color=self.nombre,
                x=bx + rx0,
                y=by + ry0 + y0,          # +y0: volvemos a coordenadas del frame
                w=w, h=h,
                area=area,
                llenado=llenado,
                aspecto=aspecto,
                cx=float(bx + xs.mean()),
                cy=float(by + ys.mean() + y0),
            )
            if con_contorno:
                mm = np.zeros((bh + 2, bw + 2), dtype=np.uint8)
                mm[1:-1, 1:-1] = propio.astype(np.uint8) * 255
                cnts, _ = cv2.findContours(mm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    c = c + np.array([[bx - 1, by - 1 + y0]], dtype=c.dtype)
                    det.contorno = c
            detecciones.append(det)

        detecciones.sort(key=lambda d: d.area, reverse=True)
        return detecciones[:int(p.get("max_objetos", 4))], mascara


# --------------------------------------------------------------------------
class Vision:
    """Agrupa varios DetectorColor y hace UNA sola conversion a HSV."""

    def __init__(self, colores: Dict[str, Dict[str, Any]]):
        self.detectores: Dict[str, DetectorColor] = {
            nombre: DetectorColor(nombre, params) for nombre, params in colores.items()
        }

    def actualizar(self, colores: Dict[str, Dict[str, Any]]) -> None:
        for nombre, params in colores.items():
            if nombre in self.detectores:
                self.detectores[nombre].actualizar(params)
            else:
                self.detectores[nombre] = DetectorColor(nombre, params)
        for sobra in set(self.detectores) - set(colores):
            del self.detectores[sobra]

    def procesar(self,
                 frame: np.ndarray,
                 solo: Optional[Sequence[str]] = None,
                 con_contorno: bool = False
                 ) -> Tuple[Dict[str, List[Deteccion]], Dict[str, np.ndarray]]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dets: Dict[str, List[Deteccion]] = {}
        masks: Dict[str, np.ndarray] = {}
        for nombre, det in self.detectores.items():
            if solo is not None and nombre not in solo:
                continue
            d, m = det.detectar(hsv, con_contorno=con_contorno)
            dets[nombre] = d
            masks[nombre] = m
        return dets, masks


# --------------------------------------------------------------------------
# Dibujo
# --------------------------------------------------------------------------
def dibujar_detecciones(frame: np.ndarray,
                        detecciones: Sequence[Deteccion],
                        color_bgr: Sequence[int] = (255, 255, 255),
                        etiqueta: bool = True,
                        contorno: bool = False) -> np.ndarray:
    c = tuple(int(v) for v in color_bgr)
    for i, d in enumerate(detecciones):
        grosor = 2 if i == 0 else 1
        cv2.rectangle(frame, (d.x, d.y), (d.x + d.w, d.y + d.h), c, grosor)
        cx, cy = d.centro
        cv2.circle(frame, (cx, cy), 4, c, -1)
        # Marca del borde inferior: es lo que usaremos como distancia relativa.
        cv2.line(frame, (d.x, d.base_y), (d.x + d.w, d.base_y), c, 1)
        if etiqueta:
            txt = f"{d.color} a{d.area} l{d.llenado:.2f}"
            ty = d.y - 6 if d.y > 18 else d.y + d.h + 14
            cv2.putText(frame, txt, (d.x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, c, 1, cv2.LINE_AA)
        if contorno and d.contorno is not None:
            cv2.drawContours(frame, [d.contorno], -1, c, 1)
    return frame


def dibujar_roi(frame: np.ndarray, roi_arriba: float, roi_abajo: float,
                color_bgr: Sequence[int] = (0, 255, 255)) -> np.ndarray:
    h, w = frame.shape[:2]
    y0 = int(roi_arriba * h)
    y1 = int(roi_abajo * h)
    c = tuple(int(v) for v in color_bgr)
    cv2.line(frame, (0, y0), (w, y0), c, 1)
    cv2.line(frame, (0, y1 - 1), (w, y1 - 1), c, 1)
    return frame


def nucleo_de_parche(parche_hsv: np.ndarray,
                     ancla: Optional[Sequence[int]] = None,
                     tol_h: int = 12,
                     tol_s: int = 70,
                     tol_v: int = 70) -> np.ndarray:
    """Limpia el parche que se toma al hacer clic.

    Al pinchar sobre un pilar es muy facil que el cuadradito de NxN pise el
    borde, una sombra o el brillo blanco. Si esos pixeles entran en el calculo,
    el rango se abre tanto que la mascara termina cogiendo el piso blanco.

    Aqui manda el pixel exacto donde se hizo clic (el "ancla"): del parche solo
    sobreviven los pixeles parecidos a el. Si el ancla es acromatica (negro,
    gris, blanco) se ignora el tono, que en ese caso es ruido puro.
    """
    pix = np.asarray(parche_hsv).reshape(-1, 3).astype(np.int16)
    if pix.shape[0] == 0:
        return pix.astype(np.uint8)
    if ancla is None:
        ancla = np.median(pix, axis=0)
    a = np.asarray(ancla, dtype=np.int16).reshape(3)

    ok = (np.abs(pix[:, 1] - a[1]) <= tol_s) & (np.abs(pix[:, 2] - a[2]) <= tol_v)
    if a[1] > 60:                                   # solo si el ancla tiene color
        dh = np.abs(((pix[:, 0] - a[0] + 90) % 180) - 90)
        ok &= dh <= tol_h
    if int(np.count_nonzero(ok)) < 3:
        return a.astype(np.uint8).reshape(1, 3)
    return pix[ok].astype(np.uint8)


def rangos_desde_pixeles(pix_hsv: np.ndarray,
                         margen_h: int = 8,
                         margen_s: int = 45,
                         margen_v: int = 50,
                         percentil: float = 90.0) -> List[List[List[int]]]:
    """Calcula rangos HSV a partir de una muestra de pixeles (click en la imagen).

    pix_hsv: array (N,3) uint8 en orden H,S,V.

    Detalles que importan:
    * El tono es circular. Se usa media circular y distancia angular, asi que
      una muestra de rojo repartida entre H=178 y H=3 da un rango correcto y,
      si hace falta, DOS intervalos ([0..x] y [y..179]).
    * Si la muestra esta poco saturada (negro, blanco, gris) el tono es puro
      ruido: en ese caso se abre el tono a 0..179 y mandan S y V. Por eso se
      puede calibrar el negro de las paredes con el mismo click.
    """
    pix = np.asarray(pix_hsv).reshape(-1, 3).astype(np.float32)
    if pix.shape[0] == 0:
        return [[[0, 0, 0], [179, 255, 255]]]

    h, s, v = pix[:, 0], pix[:, 1], pix[:, 2]

    s_lo = int(max(0, np.percentile(s, 100 - percentil) - margen_s))
    s_hi = int(min(255, np.percentile(s, percentil) + margen_s))
    v_lo = int(max(0, np.percentile(v, 100 - percentil) - margen_v))
    v_hi = int(min(255, np.percentile(v, percentil) + margen_v))

    acromatico = float(np.percentile(s, percentil)) <= 60.0
    if acromatico:
        return [[[0, s_lo, v_lo], [179, s_hi, v_hi]]]

    ang = h * (np.pi / 90.0)                      # 0..179  ->  0..2pi
    mx, my = float(np.cos(ang).mean()), float(np.sin(ang).mean())
    centro = (np.degrees(np.arctan2(my, mx)) % 360.0) / 2.0   # 0..180
    dist = np.abs(((h - centro + 90.0) % 180.0) - 90.0)       # distancia angular
    dh = float(np.percentile(dist, percentil)) + float(margen_h)

    if 2 * dh >= 179:
        return [[[0, s_lo, v_lo], [179, s_hi, v_hi]]]

    lo, hi = centro - dh, centro + dh
    if lo < 0:
        return [[[0, s_lo, v_lo], [int(round(hi)), s_hi, v_hi]],
                [[int(round(lo + 180)), s_lo, v_lo], [179, s_hi, v_hi]]]
    if hi > 179:
        return [[[int(round(lo)), s_lo, v_lo], [179, s_hi, v_hi]],
                [[0, s_lo, v_lo], [int(round(hi - 180)), s_hi, v_hi]]]
    return [[[int(round(lo)), s_lo, v_lo], [int(round(hi)), s_hi, v_hi]]]


def combinar_mascaras(masks: Dict[str, np.ndarray]) -> Optional[np.ndarray]:
    salida = None
    for m in masks.values():
        salida = m.copy() if salida is None else cv2.bitwise_or(salida, m, dst=salida)
    return salida
