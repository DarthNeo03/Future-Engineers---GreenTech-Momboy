# -*- coding: utf-8 -*-
"""
Deteccion de las senales de trafico (pilares 50x50x100 mm) y de los
delimitadores magenta del cajon de estacionamiento.

Regla 9.19:
    pilar ROJO  -> se rebasa por su DERECHA  (el robot pasa a la derecha)
    pilar VERDE -> se rebasa por su IZQUIERDA

Como los pilares apoyan en el tapete, el punto util es el CENTRO DE SU BASE:
esta sobre el plano del suelo, asi que la misma proyeccion inversa que usamos
para los muros da su posicion (X, Y) en mm sin ambiguedad de escala.
Ademas se comprueba la distancia deducida de la altura aparente (100 mm reales)
como verificacion cruzada: si ambas estimaciones discrepan mucho, la deteccion
se marca con baja confianza.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np


@dataclass
class Pillar:
    color: str          # 'red' | 'green' | 'magenta'
    x_mm: float
    y_mm: float
    dist_mm: float
    area: int
    box: tuple          # (x, y, w, h) en coordenadas de la imagen completa
    conf: float


def _masks(hsv, cfg):
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    red = (((h >= int(cfg.red_h1_lo)) & (h <= int(cfg.red_h1_hi))) |
           ((h >= int(cfg.red_h2_lo)) & (h <= int(cfg.red_h2_hi)))) & \
          (s >= int(cfg.red_s_min)) & (v >= int(cfg.red_v_min))

    grn = (h >= int(cfg.green_h_lo)) & (h <= int(cfg.green_h_hi)) & \
          (s >= int(cfg.green_s_min)) & (v >= int(cfg.green_v_min))

    mag = (h >= int(cfg.magenta_h_lo)) & (h <= int(cfg.magenta_h_hi)) & \
          (s >= int(cfg.magenta_s_min)) & (v >= int(cfg.red_v_min))

    return (red.astype(np.uint8) * 255,
            grn.astype(np.uint8) * 255,
            mag.astype(np.uint8) * 255)


def _blobs(mask, color, ground, cfg, roi_top) -> List[Pillar]:
    out: List[Pillar] = []
    ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ker, iterations=2)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = int(cfg.pillar_min_area)
    min_fill = float(cfg.pillar_min_fill)

    for c in cnts:
        area = int(cv2.contourArea(c))
        if area < min_area:
            continue
        x, y, w, hh = cv2.boundingRect(c)
        if w < 3 or hh < 4:
            continue
        if area / float(w * hh) < min_fill:
            continue
        ar = hh / float(w)
        if color != "magenta" and not (0.7 <= ar <= 6.0):
            continue
        if color == "magenta" and ar > 1.6:
            continue

        ub = x + w * 0.5
        vb = y + hh + roi_top                       # base, en la imagen completa
        X, Y, ok = ground.image_to_ground(np.array([ub]), np.array([vb]))
        if not bool(ok[0]) or not np.isfinite(X[0]) or X[0] <= 0:
            continue
        X, Y = float(X[0]), float(Y[0])
        # El borde inferior visible es la CARA FRONTAL del pilar, no su centro:
        # hay que sumar media profundidad (25 mm en un pilar de 50x50). Sin esta
        # correccion todas las distancias salen ~25 mm cortas de forma
        # sistematica, y la maniobra de esquiva se adelanta.
        if color != "magenta":
            X += 25.0
        else:
            X += 10.0                               # delimitador de 20 mm de canto

        # Verificacion por altura aparente (el pilar mide 100 mm).
        conf = 1.0
        if color != "magenta" and hh > 3:
            d_from_h = ground.fy * 100.0 / float(hh)
            rel = abs(d_from_h - X) / max(200.0, X)
            conf = float(np.clip(1.35 - rel, 0.15, 1.0))

        out.append(Pillar(color=color, x_mm=X, y_mm=Y,
                          dist_mm=float(np.hypot(X, Y)), area=area,
                          box=(x, y + roi_top, w, hh), conf=conf))
    return out


def detect(frame, ground, cfg, roi_top: int, roi_bottom: int) -> List[Pillar]:
    if frame is None or not bool(cfg.obstacles_enabled):
        return []
    roi = frame[roi_top:roi_bottom]
    if roi.size == 0:
        return []
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red, grn, mag = _masks(hsv, cfg)

    res: List[Pillar] = []
    res += _blobs(red, "red", ground, cfg, roi_top)
    res += _blobs(grn, "green", ground, cfg, roi_top)
    res += _blobs(mag, "magenta", ground, cfg, roi_top)
    res.sort(key=lambda p: p.x_mm)
    return res


def choose_target(pillars: List[Pillar], cfg, corridor_half_mm: float = 700.0):
    """
    Selecciona el pilar relevante: el mas cercano por delante, dentro del
    pasillo y dentro del alcance configurado.
    """
    best = None
    for p in pillars:
        if p.color == "magenta":
            continue
        if p.x_mm <= 60.0 or p.x_mm > float(cfg.pillar_max_range_mm):
            continue
        if abs(p.y_mm) > corridor_half_mm:
            continue
        if p.conf < 0.3:
            continue
        if best is None or p.x_mm < best.x_mm:
            best = p
    return best
