#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Prueba de la cadena de percepcion con escenas SINTETICAS de geometria conocida.

Construye vistas de camara de muros colocados a distancias exactas y comprueba
que perception.analyze() las mide bien. Sirve para:

  * verificar que un cambio en la deteccion de muros no rompe nada,
  * ver cuanto se degrada la medida al cambiar campo de vision, inclinacion,
    altura o distorsion, sin tener que montar la pista.

    python3 tools/test_percepcion.py
    python3 tools/test_percepcion.py --set cam_hfov_deg=70 --set lens_k1=-0.2

Deja las imagenes de depuracion en tools/test_out/.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from wro.geometry import Ground                      # noqa: E402
from wro.params import Config                        # noqa: E402
from wro import overlay, perception as perc          # noqa: E402

OUT = os.path.join(HERE, "test_out")


def strip(p0, p1, h=100.0, n=140):
    """Muro vertical de altura h a partir de su linea de base."""
    t = np.linspace(0, 1, n)[:, None]
    a = np.array([p0[0], p0[1], 0.0])
    b = np.array([p1[0], p1[1], 0.0])
    base = a + (b - a) * t
    top = base.copy()
    top[:, 2] = h
    return base, top


def lateral(y, x0, x1):
    return strip((x0, y), (x1, y))


def frontal(x, y0, y1):
    return strip((x, y0), (x, y1))


def rotate(walls, deg):
    """Gira la escena: equivale a girar el robot -deg dentro del pasillo."""
    c, s = math.cos(math.radians(-deg)), math.sin(math.radians(-deg))
    out = []
    for base, top in walls:
        nb, nt = base.copy(), top.copy()
        for src, dst in ((base, nb), (top, nt)):
            dst[:, 0] = src[:, 0] * c - src[:, 1] * s
            dst[:, 1] = src[:, 0] * s + src[:, 1] * c
        out.append((nb, nt))
    return out


def _project(g, P):
    C = np.array([g.off_x, g.off_y, g.height_mm])
    V = P - C
    fwd = V @ g.f
    ok = fwd > 120          # plano cercano: mas cerca la proyeccion explota
    fwd = np.where(ok, fwd, 1.0)
    xu, yu = (V @ g.r) / fwd, (V @ g.d) / fwd
    # Proyeccion PINHOLE, sin distorsion: la distorsion se aplica despues sobre
    # la imagen completa (ver apply_distortion). Distorsionar cada vertice no
    # sirve, porque el modelo de Brown deja de ser monotono fuera del encuadre
    # y la geometria que cae fuera vuelve doblada dentro de la imagen.
    return np.stack([g.cx + xu * g.fx, g.cy + yu * g.fy], 1), ok


def apply_distortion(img, g):
    """Deforma la imagen pinhole como lo haria un objetivo real (barril)."""
    if g.k1 == 0.0 and g.k2 == 0.0:
        return img
    H, W = img.shape[:2]
    u, v = np.meshgrid(np.arange(W, dtype=np.float64),
                       np.arange(H, dtype=np.float64))
    xu, yu = g._undistort_norm((u - g.cx) / g.fx, (v - g.cy) / g.fy)
    mx = (g.cx + xu * g.fx).astype(np.float32)
    my = (g.cy + yu * g.fy).astype(np.float32)
    return cv2.remap(img, mx, my, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE)


def render(g, walls, W, H):
    """
    Dibuja los muros como una tira de cuadrilateros pequenos (no como un unico
    poligono base+techo, que se auto-intersecta) y luego aplica la distorsion
    sobre la imagen ya formada.
    """
    img = np.full((H, W, 3), 235, np.uint8)
    cv2.rectangle(img, (0, 0), (W, int(max(0, g.horizon_row()))), (70, 60, 55), -1)
    for base, top in walls:
        pb, okb = _project(g, base)
        pt, okt = _project(g, top)
        ok = okb & okt
        for i in range(len(base) - 1):
            if not (ok[i] and ok[i + 1]):
                continue
            quad = np.array([pb[i], pb[i + 1], pt[i + 1], pt[i]])
            if np.abs(quad).max() > 4000:      # trozo degenerado, no dibujar
                continue
            cv2.fillPoly(img, [quad.astype(np.int32)], (28, 26, 24))
    return cv2.GaussianBlur(apply_distortion(img, g), (3, 3), 0)


CASES = [
    ("pasillo_1000", [frontal(1300, -650, 350), lateral(350, -300, 1300),
                      lateral(-650, -300, 1300)], 0.0,
     {"izq": (350, 30), "der": (650, 45), "ancho": (1000, 50),
      "frente": (1300, 70), "fin_izq": (None, 0), "fin_der": (None, 0)}),

    ("esquina", [frontal(1500, -650, 350), lateral(350, -300, 800),
                 lateral(-650, -300, 1500)], 0.0,
     {"izq": (350, 30), "der": (650, 45), "fin_izq": (800, 70),
      "fin_der": (None, 0)}),

    ("pasillo_600", [frontal(1600, -300, 300), lateral(300, -300, 1600),
                     lateral(-300, -300, 1600)], 0.0,
     {"izq": (300, 30), "der": (300, 30), "ancho": (600, 45)}),

    ("girado_12", [frontal(1300, -650, 350), lateral(350, -300, 1300),
                   lateral(-650, -300, 1300)], 12.0,
     {"izq": (300, 45), "ang_izq": (-12, 3.5)}),

    # Girado a la DERECHA dentro del pasillo. Los dos muros se miden a la
    # distancia de evaluacion (260 mm por delante), asi que el muro izquierdo
    # se aleja: 350 + 260*tan(8) = 386 mm.
    ("girado_m8", [frontal(1300, -650, 350), lateral(350, -300, 1300),
                   lateral(-650, -300, 1300)], -8.0,
     {"ang_der": (8, 3.5), "ang_izq": (8, 3.5), "izq": (386, 45)}),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", action="append", default=[], metavar="clave=valor")
    args = ap.parse_args()

    os.makedirs(OUT, exist_ok=True)
    cfg = Config(os.path.join(OUT, "test_config.json"))
    cfg.reset()
    cfg.set_many({"cam_pitch_deg": 18.0, "cam_hfov_deg": 90.0,
                  "cam_height_mm": 125.0, "cam_offset_x_mm": 60.0})
    for s in args.set:
        k, _, v = s.partition("=")
        cfg.set_many({k.strip(): v.strip()})

    W, H = int(cfg.cam_width), int(cfg.cam_height)
    g = Ground(cfg, W, H)
    print("camara: pitch=%.1f  fov=%.1f  altura=%.0f  horizonte=fila %.0f  "
          "recorte=fila %d"
          % (cfg.cam_pitch_deg, cfg.cam_hfov_deg, cfg.cam_height_mm,
             g.horizon_row(), g.roi_top_row(float(cfg.roi_x_max_mm))))

    total = fails = 0
    for name, walls, ang, expect in CASES:
        img = render(g, rotate(walls, ang) if ang else walls, W, H)
        sc = perc.analyze(img, g, cfg, want_mask=True)
        got = {
            "izq": sc.left.dist_mm if sc.left else None,
            "der": sc.right.dist_mm if sc.right else None,
            "ang_izq": sc.left.angle_deg if sc.left else None,
            "ang_der": sc.right.angle_deg if sc.right else None,
            "fin_izq": sc.left.end_mm if sc.left else None,
            "fin_der": sc.right.end_mm if sc.right else None,
            "frente": sc.front_mm,
            "ancho": sc.corridor_mm,
        }
        print("\n== %s ==  puntos=%d  tramos=%d" % (name, sc.n_points,
                                                   len(sc.segments)))
        for k, (want, tol) in expect.items():
            v = got.get(k)
            total += 1
            if want is None:
                ok = v is None
                shown, exp = ("-" if v is None else "%.0f" % v), "ninguno"
            else:
                ok = v is not None and abs(v - want) <= tol
                shown, exp = ("-" if v is None else "%.1f" % v), "%.0f +-%.0f" % (want, tol)
            fails += 0 if ok else 1
            print("   %-8s medido %-8s esperado %-12s %s"
                  % (k, shown, exp, "OK" if ok else "<-- FALLO"))
        cv2.imwrite(os.path.join(OUT, "%s_camara.png" % name),
                    overlay.draw_overlay(img, sc, g, cfg, {}, None, name))
        cv2.imwrite(os.path.join(OUT, "%s_pajaro.png" % name),
                    overlay.draw_bev(sc, cfg, {"direction": 1, "target_lat": 340}))

    print("\n%d/%d comprobaciones correctas. Imagenes en %s"
          % (total - fails, total, OUT))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
