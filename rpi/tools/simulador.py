#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulador de lazo cerrado del Reto Abierto (no necesita robot ni camara).

Reproduce la pista oficial (3000x3000 mm, muros de 100 mm, muros interiores
con carriles de 600 o 1000 mm por tramo, y las 8 lineas naranja/azul del
tapete), renderiza lo que veria la camara desde 125 mm de altura, y realimenta:

    render -> perception.analyze -> Controller.update -> modelo de bicicleta

Sirve para probar cambios de estrategia y de calibracion antes de tocar el
robot: disparo de las curvas, conteo de vueltas, deteccion del sentido de
marcha y parada en la seccion de meta.

Uso:
    python3 tools/simulador.py                 # los 6 casos por defecto
    python3 tools/simulador.py --caso ccw_1000 --video
    python3 tools/simulador.py --set base_speed=60 --set target_inner_mm=300

Salidas (en tools/sim_out/):
    traj_<caso>.png   trayectoria vista desde arriba
    sim_<caso>.mp4    video con la superposicion de depuracion (con --video)
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

from wro.controller import Controller, ST_DONE            # noqa: E402
from wro.geometry import Ground                            # noqa: E402
from wro.params import Config                              # noqa: E402
from wro import overlay, perception as perc                # noqa: E402

FIELD = 3000.0
OUT = os.path.join(HERE, "sim_out")

# Modelo del vehiculo (ajusta a tu robot para que el simulador sea util)
MAX_SPEED_MM_S = 750.0        # velocidad al 100 %
MAX_STEER_DEG = 26.0          # angulo de rueda al 100 %
WHEELBASE_MM = 150.0
ROBOT_HALF = 100.0            # semiancho/semilargo del robot (20x20 cm)


class FakeTel:
    """Telemetria falsa del ESP32."""
    def __init__(self):
        self.yaw = 0.0; self.gz = 0.0; self.accel_mag = 1.0
        self.seq = 0; self.last_event = 0; self.line = 0
        self.n_orange = 0; self.n_blue = 0
        self.button = 0; self.connected = True; self.armed = 1
        self.watchdog = 0; self.age = 0.0


# ---------------------------------------------------------------- geometria
def inner_rect(lane_bottom, lane_right, lane_top, lane_left):
    """Rectangulo de muros interiores dados los anchos de carril de cada lado."""
    return (lane_left, lane_bottom, FIELD - lane_right, FIELD - lane_top)


def edges(inner):
    x0, y0, x1, y1 = inner
    outer = [((0, 0), (FIELD, 0)), ((FIELD, 0), (FIELD, FIELD)),
             ((FIELD, FIELD), (0, FIELD)), ((0, FIELD), (0, 0))]
    inn = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
           ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return outer + inn


def corner_lines():
    """
    Las 8 lineas del tapete, medidas sobre el plano oficial.
    Esquina superior izquierda: ambas salen del vertice interior (1000, 2000);
    la AZUL va al muro de arriba y la NARANJA al muro de la izquierda.
    Las otras tres esquinas son la misma figura girada -90 grados.
    """
    base = [("blue", (1000.0, 2000.0), (427.0, 3000.0)),
            ("orange", (1000.0, 2000.0), (0.0, 2570.0))]
    out = []
    for k in range(4):
        for col, a, b in base:
            p, q = a, b
            for _ in range(k):
                p = (p[1], FIELD - p[0])
                q = (q[1], FIELD - q[0])
            out.append((col, p, q))
    return out


def seg_cross(p, q, a, b):
    def cr(o, s, t):
        return (s[0]-o[0])*(t[1]-o[1]) - (s[1]-o[1])*(t[0]-o[0])
    d1, d2 = cr(a, b, p), cr(a, b, q)
    d3, d4 = cr(p, q, a), cr(p, q, b)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def dist_to_walls(pose, inner):
    rx, ry, _ = pose
    best = 1e9
    for (ax, ay), (bx, by) in edges(inner):
        px, py = rx - ax, ry - ay
        ex, ey = bx - ax, by - ay
        L2 = ex*ex + ey*ey
        u = max(0.0, min(1.0, (px*ex + py*ey) / L2))
        best = min(best, math.hypot(px - u*ex, py - u*ey))
    return best


# ------------------------------------------------------------------ render
def render(g, pose, inner, W, H):
    """Dibuja la vista de la camara con algoritmo del pintor (lejos primero)."""
    rx, ry, th = pose
    c, s = math.cos(th), math.sin(th)
    img = np.full((H, W, 3), 232, np.uint8)
    cv2.rectangle(img, (0, 0), (W, int(max(0, g.horizon_row()))), (65, 58, 52), -1)

    quads = []
    for (ax, ay), (bx, by) in edges(inner):
        n = max(2, int(math.hypot(bx-ax, by-ay) / 40))
        t = np.linspace(0, 1, n + 1)
        px, py = ax + (bx-ax)*t, ay + (by-ay)*t
        dx, dy = px - rx, py - ry
        Xr, Yr = dx*c + dy*s, -dx*s + dy*c
        for i in range(n):
            if Xr[i] < 40 and Xr[i+1] < 40:
                continue
            d = 0.5*(math.hypot(Xr[i], Yr[i]) + math.hypot(Xr[i+1], Yr[i+1]))
            quads.append((d, Xr[i], Yr[i], Xr[i+1], Yr[i+1]))

    C = np.array([g.off_x, g.off_y, g.height_mm])
    quads.sort(key=lambda z: -z[0])
    for _, X0, Y0, X1, Y1 in quads:
        P = np.array([[X0, Y0, 0], [X1, Y1, 0], [X1, Y1, 100], [X0, Y0, 100]], float)
        V = P - C
        fwd = V @ g.f
        if (fwd <= 20).any():
            continue
        u = g.cx + (V @ g.r) / fwd * g.fx
        v = g.cy + (V @ g.d) / fwd * g.fy
        poly = np.clip(np.stack([u, v], 1), -9000, 9000).astype(np.int32)
        cv2.fillPoly(img, [poly], (26, 24, 22))
    return cv2.GaussianBlur(img, (3, 3), 0)


def plot(name, trace, inner):
    S = 480
    img = np.full((S, S, 3), 18, np.uint8)

    def px(p):
        return (int(p[0]/FIELD*S), int(S-1 - p[1]/FIELD*S))

    for col, a, b in corner_lines():
        cv2.line(img, px(a), px(b),
                 (200, 120, 40) if col == "blue" else (40, 120, 230), 2)
    for (ax, ay), (bx, by) in edges(inner):
        cv2.line(img, px((ax, ay)), px((bx, by)), (120, 120, 135), 2)
    for i in range(1, len(trace)):
        cv2.line(img, px(trace[i-1]), px(trace[i]), (90, 235, 120), 1)
    if trace:
        cv2.circle(img, px(trace[0]), 5, (255, 255, 255), -1)
        cv2.circle(img, px(trace[-1]), 5, (60, 90, 250), -1)
    cv2.putText(img, name, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (200, 200, 210), 1, cv2.LINE_AA)
    os.makedirs(OUT, exist_ok=True)
    cv2.imwrite(os.path.join(OUT, "traj_%s.png" % name), img)


# -------------------------------------------------------------------- caso
def run(name, inner, start, heading_deg, overrides=None, video=False,
        tmax=180.0, verbose=False, stop_on_crash=True, miscal=None):
    cfg = Config(os.path.join(OUT, "sim_config.json"))
    cfg.reset()
    cfg.set_many({"cam_pitch_deg": 18.0, "cam_hfov_deg": 90.0,
                  "cam_height_mm": 125.0, "cam_offset_x_mm": 60.0,
                  "log_enabled": False})
    if overrides:
        cfg.set_many(overrides)

    W, H = int(cfg.cam_width), int(cfg.cam_height)
    g = Ground(cfg, W, H)               # geometria que USA el robot
    # Geometria REAL de la camara: con --miscal se hace distinta de la que cree
    # el robot, para medir cuanto error de calibracion aguanta el sistema.
    if miscal:
        cfg_real = Config(os.path.join(OUT, "sim_real.json"))
        cfg_real.reset(); cfg_real.set_many(cfg.snapshot()); cfg_real.set_many(miscal)
        g_real = Ground(cfg_real, W, H)
    else:
        g_real = g

    clock = {"t": 0.0}
    ctrl = Controller(cfg, clock=lambda: clock["t"])
    tel = FakeTel()

    pose = [float(start[0]), float(start[1]), math.radians(heading_deg)]
    tel.yaw = heading_deg
    ctrl.start(tel.yaw, tel.seq)
    ctrl.target_yaw = heading_deg           # el rumbo se mide desde el arranque

    lines = corner_lines()
    dt = 1.0 / 30.0
    last_line_t = -9.0
    trace, crash_at = [], None
    vw = None
    prev_state = ctrl.state

    while clock["t"] < tmax and ctrl.state != ST_DONE:
        img = render(g_real, pose, inner, W, H)
        sc = perc.analyze(img, g, cfg)
        steer, speed = ctrl.update(sc, tel, dt)

        if video:
            if vw is None:
                os.makedirs(OUT, exist_ok=True)
                vw = cv2.VideoWriter(os.path.join(OUT, "sim_%s.mp4" % name),
                                     cv2.VideoWriter_fourcc(*"mp4v"), 30, (W, H))
            vw.write(overlay.draw_overlay(
                img, sc, g, cfg, ctrl.snapshot(), None,
                "t=%.1f  pos=(%.0f, %.0f)" % (clock["t"], pose[0], pose[1])))

        # ---- modelo de bicicleta ----
        v = speed / 100.0 * MAX_SPEED_MM_S
        delta = math.radians(steer / 100.0 * MAX_STEER_DEG)
        prev_xy = (pose[0], pose[1])
        pose[2] += v / WHEELBASE_MM * math.tan(delta) * dt
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        tel.yaw = math.degrees(pose[2])
        tel.gz = math.degrees(v / WHEELBASE_MM * math.tan(delta))

        # ---- sensor de color del tapete ----
        if clock["t"] - last_line_t > 0.25:
            for col, a, b in lines:
                if seg_cross(prev_xy, (pose[0], pose[1]), a, b):
                    last_line_t = clock["t"]
                    tel.seq += 1
                    tel.last_event = 1 if col == "orange" else 2
                    if col == "orange":
                        tel.n_orange += 1
                    else:
                        tel.n_blue += 1
                    break

        if verbose and ctrl.state != prev_state:
            print("    t=%5.1f  %-11s -> %-11s pos=(%4.0f,%4.0f) yaw=%6.1f "
                  "int=%s ext=%s frente=%s fin=%s | %s"
                  % (clock["t"], prev_state, ctrl.state, pose[0], pose[1],
                     math.degrees(pose[2]),
                     _f(ctrl.d_inner), _f(ctrl.d_outer), _f(ctrl.front),
                     "%4.0f" % ctrl.inner_end if ctrl.inner_end is not None else "   -",
                     ctrl.note))
            prev_state = ctrl.state

        if dist_to_walls(pose, inner) < ROBOT_HALF + 5 and crash_at is None:
            crash_at = (pose[0], pose[1], clock["t"])
            if stop_on_crash:
                break

        trace.append((pose[0], pose[1]))
        clock["t"] += dt

    if vw:
        vw.release()
    plot(name, trace, inner)

    s = ctrl.snapshot()
    ok = (ctrl.state == ST_DONE and s["laps"] >= int(cfg.laps_target)
          and crash_at is None)
    print("%-11s %-6s esq=%-2d vueltas=%d/%d  sentido=%-11s(%-6s) t=%5.1fs  %s"
          % (name, "OK" if ok else "FALLO", s["corners"], s["laps"],
             int(cfg.laps_target), s["direction_txt"], s["dir_source"],
             clock["t"],
             "choque en (%.0f, %.0f) a los %.1f s" % crash_at if crash_at
             else ("parada: %s" % s["note"] if ctrl.state == ST_DONE
                   else "sin terminar (%s)" % ctrl.state)))
    return ok


def _f(filt):
    return "%4.0f" % filt.value if filt.valid else "   -"


# ------------------------------------------------------------------- casos
# Zonas de arranque validas: dentro de un tramo RECTO (x entre 1000 y 2000 en
# el carril de abajo). Rumbo +x abajo = antihorario; rumbo -x abajo = horario.
CASOS = {
    #  nombre        carriles (abajo, der, arriba, izq)   arranque      rumbo
    "ccw_1000":  (inner_rect(1000, 1000, 1000, 1000), (1400, 350), 0.0),
    "cw_1000":   (inner_rect(1000, 1000, 1000, 1000), (1600, 350), 180.0),
    "ccw_mix":   (inner_rect(600, 1000, 600, 1000),   (1400, 300), 0.0),
    "cw_mix":    (inner_rect(600, 1000, 600, 1000),   (1600, 300), 180.0),
    "ccw_mix2":  (inner_rect(1000, 600, 1000, 600),   (1400, 500), 0.0),
    "ccw_borde": (inner_rect(1000, 1000, 1000, 1000), (1900, 180), 0.0),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--caso", default=None, help="nombre de un caso concreto")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--tmax", type=float, default=180.0)
    ap.add_argument("--set", action="append", default=[],
                    metavar="clave=valor", help="sobreescribe un parametro")
    ap.add_argument("--miscal", action="append", default=[], metavar="clave=valor",
                    help="descalibra la camara REAL respecto a la que cree el "
                         "robot (p.ej. --miscal cam_pitch_deg=20)")
    args = ap.parse_args()

    ov = {}
    for s in args.set:
        k, _, v = s.partition("=")
        ov[k.strip()] = v.strip()

    mis = {}
    for x in args.miscal:
        k, _, v = x.partition("=")
        mis[k.strip()] = v.strip()
    casos = {args.caso: CASOS[args.caso]} if args.caso else CASOS
    os.makedirs(OUT, exist_ok=True)
    print("caso        result esquinas/vueltas   sentido            tiempo")
    print("-" * 96)
    good = 0
    for name, (inner, start, hd) in casos.items():
        good += run(name, inner, start, hd, ov, args.video, args.tmax,
                    args.verbose, miscal=mis or None)
    print("-" * 96)
    print("%d/%d casos correctos. Salidas en %s" % (good, len(casos), OUT))


if __name__ == "__main__":
    main()
