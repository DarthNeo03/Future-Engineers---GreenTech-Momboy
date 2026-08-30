#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprueba que la CALIBRACION ASISTIDA del panel converge.

Genera escenas sinteticas con una camara de geometria conocida (inclinacion 18
grados, campo de vision 90), arranca desde valores mal puestos a proposito y
aplica el mismo procedimiento que hacen los botones del panel:

    2. "Calibrar inclinacion"  con una distancia real a un muro de frente
    3. "Calibrar FOV"          con el ancho real de un pasillo

Los dos parametros estan acoplados, asi que se repite el par. En 2 vueltas
queda dentro de +-0.2 grados, muy por debajo de lo que el control necesita
(inclinacion +-3, campo de vision +-6).

    python3 tools/test_calibracion.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from wro.geometry import Ground, solve_pitch_from_distance   # noqa: E402
from wro.params import Config                                # noqa: E402
from wro import perception as perc                           # noqa: E402
import simulador as S                                        # noqa: E402

OUT = os.path.join(HERE, "sim_out")
REAL_PITCH, REAL_HFOV, REAL_H = 18.0, 90.0, 125.0
TRUE_CORRIDOR = 1000.0
TRUE_FRONT = 400.0
os.makedirs(OUT, exist_ok=True)


def make_cfg(pitch, hfov):
    c = Config(os.path.join(OUT, "cal_cfg.json")); c.reset()
    c.set_many({"cam_pitch_deg": pitch, "cam_hfov_deg": hfov,
                "cam_height_mm": REAL_H, "cam_offset_x_mm": 60.0})
    return c

def render_scene(cfg_real, pose, inner):
    W, H = int(cfg_real.cam_width), int(cfg_real.cam_height)
    return S.render(Ground(cfg_real, W, H), pose, inner, W, H)

class Tmp: pass
def ground_for(cfg, over, w, h):
    t = Tmp()
    for k in ("cam_height_mm","cam_pitch_deg","cam_roll_deg","cam_hfov_deg",
              "cam_cx_off","cam_cy_off","lens_k1","lens_k2",
              "cam_offset_x_mm","cam_offset_y_mm"):
        setattr(t, k, cfg.get(k))
    for k, v in over.items(): setattr(t, k, v)
    return Ground(t, w, h)

def cal_pitch(cfg, frame, true_x):
    h, w = frame.shape[:2]
    g = Ground(cfg, w, h)
    sc = perc.analyze(frame, g, cfg)
    if len(sc.boundary_uv) < 5: return None
    sel = np.abs(sc.boundary_uv[:,0] - g.cx) < max(12.0, w*0.06)
    if sel.sum() < 3: return None
    v_row = float(np.median(sc.boundary_uv[sel,1]))
    return solve_pitch_from_distance(cfg, w, h, v_row, true_x)

def cal_hfov(cfg, frame, true_w):
    h, w = frame.shape[:2]
    def corr(f):
        return perc.analyze(frame, ground_for(cfg, {"cam_hfov_deg": f}, w, h), cfg).corridor_mm
    best, be = None, 1e9
    for i in range(0, 111):
        f = 40.0 + i; c = corr(f)
        if c is None: continue
        e = abs(c - true_w)
        if e < be: best, be = f, e
    if best is None: return None
    for i in range(-10, 11):
        f = best + i*0.1
        if not (40 <= f <= 150): continue
        c = corr(f)
        if c is not None and abs(c - true_w) < be: best, be = f, abs(c - true_w)
    return best

# Escena real (verdad): la camara ES pitch 18 / hfov 90
cfg_real = make_cfg(REAL_PITCH, REAL_HFOV)
inner = S.inner_rect(1000, 1000, 1000, 1000)

# (2) inclinacion: robot mirando de frente a un muro a 400 mm de su origen.
#     Lo colocamos en el pasillo de abajo mirando al muro exterior derecho.
frame_pitch = render_scene(cfg_real, (3000 - TRUE_FRONT, 500, 0.0), inner)
# (3) FOV: robot dentro del pasillo, mirando a lo largo, ve los dos muros.
frame_fov = render_scene(cfg_real, (1400, 650, 0.0), inner)

for start_pitch, start_hfov in [(12.0, 75.0), (24.0, 110.0), (15.0, 78.0)]:
    cfg = make_cfg(start_pitch, start_hfov)
    print("\ninicio  pitch=%.1f  hfov=%.1f   (real 18.0 / 90.0)" % (start_pitch, start_hfov))
    for it in range(1, 4):
        p = cal_pitch(cfg, frame_pitch, TRUE_FRONT)
        if p is not None: cfg.set_many({"cam_pitch_deg": round(p, 2)})
        f = cal_hfov(cfg, frame_fov, TRUE_CORRIDOR)
        if f is not None: cfg.set_many({"cam_hfov_deg": round(f, 1)})
        # comprobacion independiente
        g = Ground(cfg, 640, 480)
        sc = perc.analyze(frame_fov, g, cfg)
        print("  vuelta %d -> pitch=%6.2f  hfov=%6.1f | pasillo medido=%s mm"
              % (it, cfg.cam_pitch_deg, cfg.cam_hfov_deg,
                 "%.0f" % sc.corridor_mm if sc.corridor_mm else "-"))
