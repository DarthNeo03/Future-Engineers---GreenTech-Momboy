# -*- coding: utf-8 -*-
"""
Vistas de depuracion: superposicion sobre la camara, mascara y vista de pajaro.

La vista de pajaro es la herramienta de calibracion mas util del proyecto:
si la inclinacion y el campo de vision estan bien, un muro recto se dibuja
RECTO y a la distancia que marca la cinta metrica. Si sale curvado, sobra o
falta inclinacion; si sale recto pero a la distancia equivocada, hay que tocar
la altura o el campo de vision.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX

COL_BG = (24, 24, 28)
COL_GRID = (60, 60, 70)
COL_BOUND = (60, 220, 255)
COL_LEFT = (255, 170, 60)
COL_RIGHT = (120, 255, 120)
COL_FRONT = (200, 120, 255)
COL_OTHER = (130, 130, 140)
COL_TXT = (235, 235, 235)
COL_WARN = (60, 90, 250)
COL_OK = (120, 230, 120)


def _txt(img, s, x, y, col=COL_TXT, sc=0.42, th=1):
    cv2.putText(img, s, (x, y), FONT, sc, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), FONT, sc, col, th, cv2.LINE_AA)


# ===========================================================================
#  Superposicion sobre la imagen de camara
# ===========================================================================
def draw_overlay(frame, sc, ground, cfg, ctrl_snap, pillars=None, extra=None):
    img = frame.copy()
    h, w = img.shape[:2]

    # --- limites de la region de interes ---
    hor = int(round(ground.horizon_row()))
    if 0 <= hor < h:
        cv2.line(img, (0, hor), (w, hor), (90, 90, 120), 1)
        _txt(img, "horizonte", 4, max(10, hor - 4), (150, 150, 190), 0.38)
    cv2.line(img, (0, sc.roi_top), (w, sc.roi_top), (70, 200, 200), 1)
    cv2.line(img, (0, sc.roi_bottom - 1), (w, sc.roi_bottom - 1), (70, 200, 200), 1)
    _txt(img, "ROI  %d mm" % int(cfg.roi_x_max_mm), 4, sc.roi_top + 12,
         (70, 200, 200), 0.38)

    # --- retícula del suelo proyectada ---
    for x_mm in (300, 600, 900, 1200, 1800):
        if x_mm > float(cfg.roi_x_max_mm):
            continue
        ys = np.linspace(-700, 700, 40)
        xs = np.full_like(ys, float(x_mm))
        u, v, ok = ground.ground_to_image(xs, ys)
        pts = np.stack([u, v], 1)[ok].astype(np.int32)
        pts = pts[(pts[:, 1] > sc.roi_top) & (pts[:, 1] < h)]
        if len(pts) > 1:
            cv2.polylines(img, [pts], False, (55, 55, 65), 1, cv2.LINE_AA)
            _txt(img, "%dmm" % x_mm, int(pts[len(pts) // 2, 0]) + 3,
                 int(pts[len(pts) // 2, 1]) - 3, (90, 90, 105), 0.34)

    # --- contorno del suelo libre ---
    if len(sc.boundary_uv) > 1:
        p = sc.boundary_uv.astype(np.int32)
        cv2.polylines(img, [p], False, COL_BOUND, 2, cv2.LINE_AA)

    # --- tramos clasificados ---
    for s in sc.segments:
        if s.i1 >= len(sc.boundary_uv):
            continue
        pu = sc.boundary_uv[s.i0:s.i1 + 1].astype(np.int32)
        col = {"left": COL_LEFT, "right": COL_RIGHT,
               "front": COL_FRONT}.get(s.side, COL_OTHER)
        if len(pu) > 1:
            cv2.polylines(img, [pu], False, col, 3, cv2.LINE_AA)

    # --- pilares ---
    for p in (pillars or []):
        x, y, bw, bh = p.box
        col = {"red": (60, 60, 240), "green": (80, 220, 80),
               "magenta": (220, 80, 220)}[p.color]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
        _txt(img, "%s %.0fmm" % (p.color[0].upper(), p.x_mm), x, max(12, y - 4),
             col, 0.4)

    # --- panel de texto ---
    cs = ctrl_snap or {}
    y0 = 16
    _txt(img, "%s | %s | vuelta %s/%s | esq %s" % (
        cs.get("state", "-"), cs.get("direction_txt", "?"),
        cs.get("laps", 0), int(cfg.laps_target), cs.get("corners", 0)),
        6, y0, COL_OK if cs.get("state") != "PARADO" else COL_WARN, 0.5)
    y0 += 16
    _txt(img, "int %s  ext %s  frente %s  fin %s  ancho %s" % (
        _f(cs.get("d_inner")), _f(cs.get("d_outer")), _f(cs.get("front")),
        _f(cs.get("inner_end")), _f(cs.get("corridor"))), 6, y0)
    y0 += 14
    _txt(img, "obj %s  errLat %s  corrRumbo %s  errRumbo %s" % (
        _f(cs.get("target_lat")), _f(cs.get("lat_err")),
        _f(cs.get("head_corr"), "%.1f"), _f(cs.get("head_err"), "%.1f")), 6, y0)
    y0 += 14
    _txt(img, "dir %s%%  vel %s%%  pts %d  umbral %d" % (
        _f(cs.get("steer"), "%.0f"), _f(cs.get("speed"), "%.0f"),
        sc.n_points, sc.thresh_used), 6, y0)
    if extra:
        y0 += 14
        _txt(img, extra, 6, y0, (140, 200, 255), 0.4)
    if cs.get("note"):
        _txt(img, cs["note"], 6, h - 8, (120, 200, 255), 0.42)
    return img


def _f(v, fmt="%.0f"):
    return "-" if v is None else (fmt % v)


# ===========================================================================
#  Mascara
# ===========================================================================
def draw_mask(frame, sc):
    h, w = frame.shape[:2]
    img = np.zeros((h, w, 3), np.uint8)
    if sc.mask is not None:
        m = cv2.cvtColor(sc.mask, cv2.COLOR_GRAY2BGR)
        img[sc.roi_top:sc.roi_top + m.shape[0]] = m
    img[:sc.roi_top] = (18, 18, 40)
    if len(sc.boundary_uv) > 1:
        cv2.polylines(img, [sc.boundary_uv.astype(np.int32)], False,
                      (60, 220, 255), 2, cv2.LINE_AA)
    _txt(img, "MASCARA - el tapete debe salir NEGRO y los muros BLANCOS",
         6, 16, (200, 200, 200), 0.42)
    _txt(img, "azul oscuro arriba = zona recortada por geometria",
         6, 32, (150, 150, 200), 0.38)
    return img


# ===========================================================================
#  Vista de pajaro
# ===========================================================================
def draw_bev(sc, cfg, ctrl_snap, pillars=None, size=(420, 470)):
    W, H = size
    img = np.full((H, W, 3), COL_BG, np.uint8)

    x_max = float(cfg.roi_x_max_mm)
    y_half = max(800.0, x_max * 0.55)
    sx = (H - 30) / x_max          # px por mm en X (hacia arriba)
    sy = W / (2.0 * y_half)        # px por mm en Y (izquierda -> menor u)
    ox, oy = W // 2, H - 22        # origen del robot

    def to_px(X, Y):
        return int(ox - Y * sy), int(oy - X * sx)

    # --- retícula ---
    for x in range(0, int(x_max) + 1, 250):
        _, py = to_px(x, 0)
        cv2.line(img, (0, py), (W, py), COL_GRID, 1)
        _txt(img, "%d" % x, 3, py - 3, (110, 110, 125), 0.33)
    for y in range(-int(y_half // 250) * 250, int(y_half) + 1, 250):
        px, _ = to_px(0, y)
        cv2.line(img, (px, 0), (px, H), COL_GRID, 1)

    # --- robot ---
    rw, rl = 200.0, 200.0
    p1 = to_px(-rl * 0.15, rw / 2)
    p2 = to_px(rl * 0.85, -rw / 2)
    cv2.rectangle(img, p1, p2, (200, 200, 60), 1)
    cv2.arrowedLine(img, to_px(0, 0), to_px(190, 0), (200, 200, 60), 1,
                    tipLength=0.25)

    # --- contorno ---
    for X, Y in sc.boundary_xy:
        cv2.circle(img, to_px(X, Y), 1, COL_BOUND, -1)

    # --- tramos ---
    for s in sc.segments:
        col = {"left": COL_LEFT, "right": COL_RIGHT,
               "front": COL_FRONT}.get(s.side, COL_OTHER)
        a = to_px(s.pts[0, 0], s.pts[0, 1])
        b = to_px(s.pts[-1, 0], s.pts[-1, 1])
        cv2.line(img, a, b, col, 2, cv2.LINE_AA)

    # --- rectas ajustadas ---
    for fit, col, name in ((sc.left, COL_LEFT, "IZQ"), (sc.right, COL_RIGHT, "DER")):
        if fit is None:
            continue
        x0, x1 = 0.0, min(x_max, max(600.0, fit.x_max))
        a = to_px(x0, fit.slope * x0 + fit.offset_mm)
        b = to_px(x1, fit.slope * x1 + fit.offset_mm)
        cv2.line(img, a, b, col, 1, cv2.LINE_AA)
        px, py = to_px(float(cfg.wall_eval_x_mm),
                       fit.slope * float(cfg.wall_eval_x_mm) + fit.offset_mm)
        cv2.circle(img, (px, py), 4, col, -1)
        _txt(img, "%s %.0f (%.0f')" % (name, fit.dist_mm, fit.angle_deg),
             px - 30, py - 7, col, 0.36)
        if fit.end_mm is not None:
            e = to_px(fit.end_mm, fit.slope * fit.end_mm + fit.offset_mm)
            cv2.drawMarker(img, e, (60, 90, 250), cv2.MARKER_TILTED_CROSS, 11, 2)
            _txt(img, "fin %.0f" % fit.end_mm, e[0] + 6, e[1], (90, 130, 255), 0.36)

    # --- frente ---
    if sc.front_mm < x_max:
        _, py = to_px(sc.front_mm, 0)
        band = int(float(cfg.front_band_mm) * sy)
        cv2.line(img, (ox - band, py), (ox + band, py), COL_FRONT, 2)
        _txt(img, "frente %.0f" % sc.front_mm, ox + band + 4, py + 4, COL_FRONT, 0.36)

    # --- objetivo lateral ---
    cs = ctrl_snap or {}
    d = cs.get("direction", 0)
    tl = cs.get("target_lat")
    if d and tl:
        fit = sc.left if d > 0 else sc.right
        if fit is not None:
            xs = np.linspace(0, min(x_max, 1200), 24)
            ys = fit.slope * xs + fit.offset_mm - d * tl
            pts = np.array([to_px(x, y) for x, y in zip(xs, ys)], np.int32)
            cv2.polylines(img, [pts], False, (80, 255, 255), 1, cv2.LINE_AA)

    # --- pilares ---
    for p in (pillars or []):
        col = {"red": (60, 60, 240), "green": (80, 220, 80),
               "magenta": (220, 80, 220)}[p.color]
        px, py = to_px(p.x_mm, p.y_mm)
        cv2.circle(img, (px, py), 6, col, -1)
        _txt(img, "%.0f" % p.x_mm, px + 8, py + 3, col, 0.34)

    _txt(img, "VISTA DE PAJARO  (mm)  X arriba / Y izquierda", 6, 14,
         (190, 190, 200), 0.4)
    return img
