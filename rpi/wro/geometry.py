# -*- coding: utf-8 -*-
"""
Proyeccion inversa de perspectiva (IPM): pixel <-> punto del suelo.

Idea clave del proyecto
-----------------------
La camara esta a 125 mm y los muros miden 100 mm. Como la camara esta MAS ALTA
que los muros, la arista superior de cualquier muro cae siempre por DEBAJO del
horizonte de la imagen. Por tanto:

    todo lo que aparece por encima del horizonte es fondo (sala, mesas, gente)
    y se puede recortar sin perder ni un pixel util.

Ademas, fijando un rango maximo (por ejemplo 2.2 m) obtenemos una fila concreta
de la imagen por encima de la cual no hay nada que analizar. Ese recorte es lo
que evita que el contorno del suelo "se escape" hacia el fondo, que es el fallo
tipico cuando se busca el muro sin modelo geometrico.

Convenio de ejes
----------------
    Mundo:   X hacia adelante, Y hacia la IZQUIERDA, Z hacia arriba.
             Origen en el suelo, bajo el punto de referencia del robot.
    Imagen:  u a la derecha, v hacia abajo.
"""

from __future__ import annotations

import math
import numpy as np


class Ground:
    """Modelo geometrico camara-suelo. Se reconstruye cuando cambia la config."""

    def __init__(self, cfg, width: int, height: int):
        self.w = int(width)
        self.h = int(height)
        self.height_mm = float(cfg.cam_height_mm)
        self.pitch = math.radians(float(cfg.cam_pitch_deg))
        self.roll = math.radians(float(cfg.cam_roll_deg))
        self.k1 = float(cfg.lens_k1)
        self.k2 = float(cfg.lens_k2)
        self.off_x = float(cfg.cam_offset_x_mm)
        self.off_y = float(cfg.cam_offset_y_mm)

        hfov = math.radians(float(cfg.cam_hfov_deg))
        self.fx = (self.w * 0.5) / math.tan(hfov * 0.5)
        self.fy = self.fx                              # pixeles cuadrados
        self.cx = self.w * 0.5 + float(cfg.cam_cx_off)
        self.cy = self.h * 0.5 + float(cfg.cam_cy_off)

        # Base de la camara expresada en coordenadas del mundo.
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        f1 = np.array([cp, 0.0, -sp])          # eje optico (adelante-abajo)
        r1 = np.array([0.0, -1.0, 0.0])        # derecha de la imagen
        d1 = np.array([-sp, 0.0, -cp])         # abajo de la imagen
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        self.f = f1
        self.r = r1 * cr + d1 * sr
        self.d = d1 * cr - r1 * sr

    # ------------------------------------------------------------------ util
    def _undistort_norm(self, xd, yd):
        """Invierte la distorsion radial (Brown-Conrady, solo k1/k2)."""
        if self.k1 == 0.0 and self.k2 == 0.0:
            return xd, yd
        x, y = np.array(xd, dtype=np.float64), np.array(yd, dtype=np.float64)
        for _ in range(6):
            r2 = x * x + y * y
            f = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
            f = np.where(np.abs(f) < 1e-6, 1e-6, f)
            x = xd / f
            y = yd / f
        return x, y

    def _distort_norm(self, xu, yu):
        if self.k1 == 0.0 and self.k2 == 0.0:
            return xu, yu
        r2 = xu * xu + yu * yu
        f = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
        return xu * f, yu * f

    # ------------------------------------------------------- pixel -> suelo
    def image_to_ground(self, u, v):
        """Devuelve (X, Y, valid) en mm para arrays de pixeles."""
        u = np.asarray(u, dtype=np.float64)
        v = np.asarray(v, dtype=np.float64)
        a = (u - self.cx) / self.fx
        b = (v - self.cy) / self.fy
        a, b = self._undistort_norm(a, b)

        dx = a * self.r[0] + b * self.d[0] + self.f[0]
        dy = a * self.r[1] + b * self.d[1] + self.f[1]
        dz = a * self.r[2] + b * self.d[2] + self.f[2]

        # El rayo debe apuntar hacia abajo para cortar el suelo.
        valid = dz < -1e-6
        dz_safe = np.where(valid, dz, -1.0)
        t = -self.height_mm / dz_safe

        X = self.off_x + t * dx
        Y = self.off_y + t * dy
        return X, Y, valid

    # ------------------------------------------------------- suelo -> pixel
    def ground_to_image(self, X, Y):
        """Proyecta puntos del suelo (mm) a pixeles. Devuelve (u, v, valid)."""
        X = np.asarray(X, dtype=np.float64) - self.off_x
        Y = np.asarray(Y, dtype=np.float64) - self.off_y
        Z = -self.height_mm                       # el suelo visto desde la camara
        vx, vy, vz = X, Y, np.full_like(X, Z, dtype=np.float64)

        fwd = vx * self.f[0] + vy * self.f[1] + vz * self.f[2]
        rgt = vx * self.r[0] + vy * self.r[1] + vz * self.r[2]
        dwn = vx * self.d[0] + vy * self.d[1] + vz * self.d[2]

        valid = fwd > 1e-6
        fwd_safe = np.where(valid, fwd, 1.0)
        xu, yu = rgt / fwd_safe, dwn / fwd_safe
        xd, yd = self._distort_norm(xu, yu)
        u = self.cx + xd * self.fx
        v = self.cy + yd * self.fy
        return u, v, valid

    # ---------------------------------------------------------------- filas
    def horizon_row(self) -> float:
        """Fila de la imagen donde el suelo esta a distancia infinita."""
        # dz = 0  ->  b = -f_z / d_z   (con a = 0)
        if abs(self.d[2]) < 1e-9:
            return -1e9
        b = -self.f[2] / self.d[2]
        return self.cy + b * self.fy

    def row_for_x(self, x_mm: float) -> float:
        """Fila (v) correspondiente a un punto del suelo a X mm, en Y = 0."""
        u, v, ok = self.ground_to_image(np.array([x_mm]), np.array([0.0]))
        return float(v[0]) if bool(ok[0]) else -1e9

    def roi_top_row(self, x_max_mm: float, margin_px: int = 2) -> int:
        """Primera fila util: la que corresponde al rango maximo (nunca por
        encima del horizonte)."""
        v_far = self.row_for_x(x_max_mm)
        v_hor = self.horizon_row()
        top = max(v_far, v_hor + 1.0)
        top = max(0.0, min(self.h - 2.0, top - margin_px))
        return int(top)

    def describe(self) -> dict:
        return {
            "fx": round(self.fx, 2), "fy": round(self.fy, 2),
            "cx": round(self.cx, 2), "cy": round(self.cy, 2),
            "horizon_row": round(self.horizon_row(), 1),
            "pitch_deg": round(math.degrees(self.pitch), 2),
            "height_mm": self.height_mm,
        }


def solve_pitch_from_distance(cfg, width, height, v_row: float,
                              true_x_mm: float) -> float:
    """
    Calibracion de la inclinacion con una sola medida.

    El usuario coloca el robot mirando de frente a un muro, mide con cinta la
    distancia real desde el origen del robot a la base del muro (true_x_mm) y
    el programa toma la fila `v_row` donde el algoritmo ve esa base en el
    centro de la imagen. Se busca el pitch que hace coincidir ambos.

    Se resuelve por biseccion porque X(pitch) es monotona en el rango util.
    """
    lo, hi = math.radians(-10.0), math.radians(59.0)

    class _P:                       # copia ligera de la config con pitch variable
        pass

    def x_for(pitch_deg):
        p = _P()
        for k in ("cam_height_mm", "cam_roll_deg", "lens_k1", "lens_k2",
                  "cam_offset_x_mm", "cam_offset_y_mm", "cam_hfov_deg",
                  "cam_cx_off", "cam_cy_off"):
            setattr(p, k, getattr(cfg, k))
        p.cam_pitch_deg = pitch_deg
        g = Ground(p, width, height)
        X, Y, ok = g.image_to_ground(np.array([g.cx]), np.array([v_row]))
        if not bool(ok[0]) or not np.isfinite(X[0]) or X[0] <= 0:
            return 1e9
        return float(X[0])

    # X decrece al aumentar el pitch (mirar mas abajo = ver mas cerca).
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if x_for(math.degrees(mid)) > true_x_mm:
            lo = mid
        else:
            hi = mid
    return math.degrees(0.5 * (lo + hi))
