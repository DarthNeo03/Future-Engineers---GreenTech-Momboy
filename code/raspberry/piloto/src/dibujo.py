"""
dibujo.py — Superponer lo que el carro esta "pensando" sobre el video.

Todo lo que se ve en la web sale de aqui: el horizonte geometrico, la linea
de contacto muro-piso, las rectas ajustadas y sus esquinas, el corredor real
de las ruedas, las lineas del piso con su distancia, los pilares y el HUD.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro
from .navegacion import Decision, RECTO, PRE_GIRO, GIRO, GIRO_2T, ESCAPE

COLOR_ESTADO = {
    RECTO: (80, 220, 80),
    PRE_GIRO: (0, 200, 255),
    GIRO: (0, 165, 255),
    GIRO_2T: (0, 140, 255),
    ESCAPE: (0, 0, 255),
    "manual": (255, 200, 0),
    "parado": (160, 160, 160),
    "meta": (255, 80, 200),
}


def anotar(frame: np.ndarray,
           perfil: Optional[PerfilMuro],
           d: Decision,
           geo: Geometria,
           dets: Dict[str, List[vision.Deteccion]],
           colores_dibujo: Dict[str, Any],
           carrera: Dict[str, Any],
           obst_info: Dict[str, Any],
           hud: Dict[str, Any]) -> np.ndarray:
    H, W = frame.shape[:2]

    # --- horizonte geometrico: arriba de esto NO es pista ------------------
    y_h = geo.fila_horizonte()
    if 0 <= y_h < H:
        cv2.line(frame, (0, y_h), (W, y_h), (90, 90, 90), 1, cv2.LINE_AA)
        cv2.putText(frame, "horizonte", (W - 78, max(12, y_h - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)

    if perfil is not None:
        # --- linea de contacto muro-piso -----------------------------------
        pts = []
        for x in range(0, W, 3):
            if perfil.valido[x]:
                pts.append((x, int(perfil.y_contacto[x])))
            else:
                pts.append(None)
        prev = None
        for pt in pts:
            if pt is not None and prev is not None and abs(pt[0] - prev[0]) <= 6:
                cv2.line(frame, prev, pt, (0, 255, 255), 2)
            prev = pt

        # --- rectas ajustadas, con la clase que se les ha reconocido -------
        # verde = pared lateral (la de tu carril), rojo = pared de frente (el
        # fondo de la curva), gris = tramo sin orientacion clara.
        COLOR_CLASE = {"lateral_izq": (80, 230, 80), "lateral_der": (80, 230, 80),
                       "frontal": (60, 60, 235), "otro": (130, 130, 130)}
        for s in perfil.segmentos:
            c0 = min(max(s.col0, 0), W - 1)
            c1 = min(max(s.col1, 0), W - 1)
            p0 = (c0, int(perfil.y_contacto[c0]) or y_h)
            p1 = (c1, int(perfil.y_contacto[c1]) or y_h)
            col = COLOR_CLASE.get(s.clase, (60, 220, 60))
            grosor = 2 if s.clase != "otro" else 1
            cv2.line(frame, p0, p1, col, grosor, cv2.LINE_AA)
            if s.clase in ("lateral_izq", "lateral_der", "frontal"):
                etq = {"lateral_izq": "lat izq", "lateral_der": "lat der",
                       "frontal": "FRENTE"}[s.clase]
                cv2.putText(frame, etq, ((p0[0] + p1[0]) // 2 - 16,
                                         (p0[1] + p1[1]) // 2 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1, cv2.LINE_AA)
        morro = float(geo.cfg.get("morro_mm", 60.0))
        for e in perfil.esquinas:
            try:
                u, v = geo.suelo_a_pixel(e.x, e.y + morro)
            except Exception:
                continue
            if 0 <= u < W and 0 <= v < H:
                col = (255, 255, 0) if e.tipo == "saliente" else (255, 0, 255)
                cv2.circle(frame, (u, v), 6, col, 2)
                cv2.putText(frame, e.tipo[:3], (u + 8, v),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv2.LINE_AA)

        # --- barras de espacio libre ---------------------------------------
        def barra(x0, x1, valor, etiqueta, color):
            alto = int(40 * max(0.0, min(1.0, valor)))
            cv2.rectangle(frame, (x0, H - 10 - alto), (x1, H - 10), color, -1)
            cv2.putText(frame, etiqueta, (x0, H - 54),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)

        barra(4, 26, perfil.izq, f"I{perfil.izq:.2f}", (60, 220, 60))
        barra(W - 26, W - 4, perfil.der, f"D{perfil.der:.2f}", (60, 220, 60))
        barra(W // 2 - 12, W // 2 + 12, perfil.pasillo,
              f"{perfil.pasillo_mm:.0f}mm", (60, 200, 255))

    # --- corredor real de las ruedas ---------------------------------------
    try:
        poly = geo.poligono_corredor()
        cv2.polylines(frame, [poly], True, (230, 230, 230), 1, cv2.LINE_AA)
    except Exception:
        pass

    # --- detecciones (lineas del piso y pilares) ---------------------------
    morro = float(geo.cfg.get("morro_mm", 60.0))
    for color, lista in dets.items():
        cfg_c = colores_dibujo.get(color, {})
        bgr = tuple(int(v) for v in cfg_c.get("color_dibujo", [255, 255, 255]))
        for i, det in enumerate(lista):
            grosor = 2 if i == 0 else 1
            cv2.rectangle(frame, (det.x, det.y),
                          (det.x + det.w, det.y + det.h), bgr, grosor)
            dist = float(geo.fila_a_distancia(det.base_y)) - morro
            if dist < 5000:
                cv2.putText(frame, f"{dist / 10:.0f}cm",
                            (det.x, det.base_y + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, bgr, 1, cv2.LINE_AA)

    # --- objetivo del esquive ---------------------------------------------
    if obst_info:
        try:
            u, v = geo.suelo_a_pixel(float(obst_info["objetivo_mm"]),
                                     float(obst_info["dist_mm"]) + morro)
            cv2.drawMarker(frame, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
        except Exception:
            pass

    # --- volante -----------------------------------------------------------
    cx, cy = W // 2, 22
    x2 = int(cx + (W * 0.22) * (d.direccion / 100.0))
    cv2.line(frame, (cx, cy), (x2, cy), (0, 165, 255), 4)
    cv2.circle(frame, (cx, cy), 4, (255, 255, 255), -1)

    # --- HUD ----------------------------------------------------------------
    col_est = COLOR_ESTADO.get(d.estado, (255, 255, 255))
    zona = carrera.get("lineas", {}).get("zona", "recta")
    lineas_hud = [
        (f"{hud.get('armado_txt', '')} | {hud.get('modo', '')} | "
         f"{hud.get('fps', 0):.1f} fps", (200, 200, 200)),
        (hud.get("enlace_txt", ""), (0, 255, 0) if hud.get("enlace_ok") else (0, 0, 255)),
        (f"sentido {carrera.get('sentido', '?')} | esq {carrera.get('esquinas', 0)}"
         f"/{carrera.get('meta_esquinas', 12)} | vuelta "
         f"{carrera.get('vueltas', 0)} | {carrera.get('estado', '')}",
         (255, 220, 100)),
    ]
    if zona == "esquina":
        # marco naranja: mientras esto se ve, la vision NO decide el rumbo
        cv2.rectangle(frame, (2, 2), (W - 3, H - 3), (0, 140, 255), 3)
        cv2.putText(frame, "EN ESQUINA (giro comprometido)", (8, H - 92),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 255), 1, cv2.LINE_AA)
    y = 14
    for txt, col in lineas_hud:
        cv2.putText(frame, txt, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    col, 1, cv2.LINE_AA)
        y += 15

    if perfil is not None and (perfil.interna_mm is not None or
                               perfil.frontal_mm is not None):
        def _mm(v):
            return "-" if v is None else f"{v:.0f}"
        txt = (f"int {_mm(perfil.interna_mm)} | ext {_mm(perfil.externa_mm)}"
               f" | frente {_mm(perfil.frontal_mm)}")
        if perfil.error_rumbo is not None:
            txt += f" | desvio {perfil.error_rumbo:+.0f}"
        cv2.putText(frame, txt, (6, H - 108), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (150, 230, 150), 1, cv2.LINE_AA)

    if hud.get("yaw") is not None:
        cv2.putText(frame, f"yaw {hud['yaw']:+6.1f}", (W - 105, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 200, 0), 1, cv2.LINE_AA)
    clase = hud.get("tcs_clase", "-")
    if clase != "-":
        col = (0, 140, 255) if clase == "naranja" else (255, 120, 0)
        cv2.rectangle(frame, (W - 105, 20), (W - 55, 34), col, -1)
        cv2.putText(frame, clase, (W - 102, 31), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (0, 0, 0), 1, cv2.LINE_AA)

    cv2.putText(frame,
                f"{d.estado.upper()} vel={d.vel:+d}% dir={d.direccion:+d}%",
                (6, H - 76), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col_est, 1, cv2.LINE_AA)
    if d.motivo:
        cv2.putText(frame, d.motivo[:64], (6, H - 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1, cv2.LINE_AA)
    return frame


def vista_piso(masks: Dict[str, np.ndarray],
               perfil: Optional[PerfilMuro]) -> Optional[np.ndarray]:
    """Vista de depuracion del detector de muros: que se considero piso
    (gris claro), que no (oscuro) y la linea de contacto encontrada."""
    blanco = masks.get("blanco")
    if blanco is None:
        return None
    piso = blanco.copy()
    for extra in ("naranja", "azul"):
        m = masks.get(extra)
        if m is not None:
            piso = cv2.bitwise_or(piso, m)
    img = cv2.cvtColor((piso // 2 + 80).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    img[piso == 0] //= 3
    if perfil is not None:
        for x in range(0, img.shape[1], 2):
            if perfil.valido[x]:
                y = int(perfil.y_contacto[x])
                if 0 <= y < img.shape[0]:
                    cv2.circle(img, (x, y), 1, (0, 255, 255), -1)
        cv2.line(img, (0, perfil.y_horizonte),
                 (img.shape[1], perfil.y_horizonte), (90, 90, 200), 1)
    return img
