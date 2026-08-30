# -*- coding: utf-8 -*-
"""
Percepcion de muros a partir de la imagen.

Por que este enfoque
--------------------
El error tipico es intentar clasificar "muro interior" vs "muro exterior"
directamente en la imagen: ambos son negros, se tocan en las esquinas y el
fondo de la sala tambien es oscuro. Aqui NO se clasifica en la imagen. Se hace:

  1. Recorte geometrico: se descarta todo lo que este por encima de la fila
     correspondiente al rango maximo (siempre por debajo del horizonte). Como
     la camara (125 mm) esta por encima de los muros (100 mm), el fondo de la
     sala desaparece por completo.
  2. Mascara acromatica oscura: muro = poco brillo Y poca saturacion. Las
     lineas naranja/azul y los pilares rojo/verde quedan fuera por saturados.
  3. Contorno de suelo libre: para cada columna se busca, subiendo desde abajo,
     la primera racha de N pixeles oscuros. Ese punto es la BASE del muro, que
     esta sobre el plano del suelo, asi que la proyeccion inversa (IPM) es
     exacta ahi.
  4. El contorno se lleva a coordenadas del suelo y se parte en tramos rectos.
     Cada tramo es una cara de muro. Se clasifica por su ORIENTACION y su
     POSICION, no por su color:
        - tramo alineado con el eje del robot  -> muro lateral (izq. o der.)
        - tramo transversal                    -> muro frontal
  5. El final del muro interior se detecta como un SALTO CONVEXO del rango
     (el contorno pasa de golpe a un punto mucho mas lejano). Eso es la esquina
     y permite anticipar el giro en lugar de reaccionar cuando ya es tarde.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import cv2
import numpy as np


# ===========================================================================
#  Estructuras
# ===========================================================================
@dataclass
class Segment:
    i0: int
    i1: int
    pts: np.ndarray                 # (N,2) XY en mm
    mid: np.ndarray                 # centro
    dirv: np.ndarray                # vector director unitario
    angle_deg: float                # -90..90 respecto al eje X del robot
    length_mm: float
    kind: str                       # 'side' | 'front'
    side: str                       # 'left' | 'right' | 'front'
    x_min: float
    x_max: float
    y_mean: float


@dataclass
class WallFit:
    dist_mm: float                  # distancia lateral (positiva)
    angle_deg: float                # + = el muro se abre hacia la izquierda
    slope: float                    # dY/dX
    offset_mm: float                # Y del muro en X = 0
    n: int
    quality: float                  # 0..1
    x_min: float
    x_max: float
    end_mm: Optional[float] = None  # X del final convexo del muro (esquina)


@dataclass
class Scene:
    ok: bool = False
    roi_top: int = 0
    roi_bottom: int = 0
    boundary_uv: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    boundary_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    segments: List[Segment] = field(default_factory=list)
    left: Optional[WallFit] = None
    right: Optional[WallFit] = None
    front_mm: float = 9999.0
    front_min_mm: float = 9999.0
    corridor_mm: Optional[float] = None
    mask: Optional[np.ndarray] = None
    thresh_used: int = 0
    n_points: int = 0


# ===========================================================================
#  Utilidades numericas
# ===========================================================================
def _median1d(a: np.ndarray, k: int) -> np.ndarray:
    """Mediana movil sobre un vector que puede contener NaN."""
    if k <= 1 or a.size < k:
        return a
    pad = k // 2
    ap = np.pad(a, pad, mode="edge")
    win = np.lib.stride_tricks.sliding_window_view(ap, k)
    with np.errstate(invalid="ignore"):
        allnan = np.all(np.isnan(win), axis=1)
        out = np.empty_like(a)
        out[:] = np.nan
        good = ~allnan
        if np.any(good):
            out[good] = np.nanmedian(win[good], axis=1)
    # No inventamos puntos donde no habia deteccion
    out[np.isnan(a)] = np.nan
    return out


def _fit_tls(pts: np.ndarray):
    """Ajuste ortogonal (PCA). Devuelve (centro, direccion, normal, residuos)."""
    m = pts.mean(axis=0)
    q = pts - m
    # SVD de 2 columnas: barato incluso con cientos de puntos
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    d = vt[0]
    if d[0] < 0:                      # direccion siempre hacia +X
        d = -d
    n = np.array([-d[1], d[0]])
    resid = q @ n
    return m, d, n, resid


def _range_scale(x_mm: np.ndarray, ref: float, cap: float = 30.0) -> np.ndarray:
    """
    Cuanta tolerancia merece un punto segun lo lejos que este.

    La resolucion en profundidad se degrada con el CUADRADO de la distancia:
    con la camara a 125 mm, un pixel vale 4 mm a 600 mm, 20 mm a 1200 y 73 mm a
    2200. Por eso una tolerancia FIJA de corte es absurda: a 600 mm son 11
    pixeles de margen, pero a 1600 mm es 1,2 pixeles, asi que cualquier pixel de
    ruido parte el muro en trozos. Cada trozo corto tiene una orientacion sin
    sentido y acaba clasificado al azar como lateral o frontal: eso es el
    parpadeo de colores con el robot parado.
    """
    r = (np.asarray(x_mm, dtype=np.float64) / max(1.0, ref)) ** 2
    return np.clip(r, 1.0, cap)


def _chord_dev(pts: np.ndarray) -> float:
    """Maxima separacion de los puntos respecto a la cuerda extremo-extremo."""
    d = pts[-1] - pts[0]
    L = float(np.hypot(d[0], d[1]))
    if L < 1e-6:
        return 0.0
    nrm = np.array([-d[1], d[0]]) / L
    return float(np.max(np.abs((pts - pts[0]) @ nrm)))


def _norm_angle(deg: float) -> float:
    while deg > 90.0:
        deg -= 180.0
    while deg <= -90.0:
        deg += 180.0
    return deg


# ===========================================================================
#  1) Mascara de muro
# ===========================================================================
def build_mask(frame_roi: np.ndarray, cfg):
    hsv = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2HSV)
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    thr = int(cfg.wall_v_max)
    if bool(cfg.wall_auto_thresh):
        # Otsu sobre el canal V limitado a la region de interes.
        t, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Otsu separa tapete/muro; nos quedamos algo por debajo del corte.
        thr = int(max(20, min(200, t * 0.75)))

    mask = ((v < thr) & (s < int(cfg.wall_s_max))).astype(np.uint8) * 255

    k = int(cfg.morph_open)
    if k >= 2:
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ker)
    return mask, thr


# ===========================================================================
#  2) Contorno del suelo libre
# ===========================================================================
def floor_boundary(mask: np.ndarray, cfg):
    """
    Para cada columna (submuestreada) devuelve la fila mas baja que inicia una
    racha vertical de `wall_min_run_px` pixeles de muro. Es la base del muro.
    """
    h, w = mask.shape[:2]
    run = max(1, int(cfg.wall_min_run_px))
    if run > 1:
        ker = np.ones((run, 1), np.uint8)
        # anchor=(0, run-1): la salida en la fila r es el AND de las filas
        # r-run+1 .. r, es decir "hay muro en r y en las run-1 filas de arriba".
        er = cv2.erode(mask, ker, anchor=(0, run - 1),
                       borderType=cv2.BORDER_CONSTANT, borderValue=0)
    else:
        er = mask

    step = max(1, int(cfg.col_step))
    cols = np.arange(0, w, step, dtype=np.int32)
    sub = er[:, cols] > 0

    flip = sub[::-1, :]
    idx = np.argmax(flip, axis=0)
    has = flip[idx, np.arange(cols.size)]
    rows = (h - 1 - idx).astype(np.float64)
    rows[~has] = np.nan

    rows = _median1d(rows, int(cfg.boundary_median) | 1)
    return cols, rows


# ===========================================================================
#  3) Segmentacion del contorno en tramos rectos
# ===========================================================================
def _split_merge(P: np.ndarray, idx: np.ndarray, tol_pt: np.ndarray,
                 min_pts: int) -> List[np.ndarray]:
    """
    Parte un tramo del contorno donde deja de ser recto (Douglas-Peucker).

    El corte se busca por distancia a la CUERDA que une los dos extremos, no al
    ajuste global del tramo: en una esquina en "L" el punto que mas se separa de
    la recta ajustada cae en los EXTREMOS, no en el vertice, asi que usando el
    ajuste global la esquina no se parte nunca y los dos muros salen fundidos en
    un solo tramo con una orientacion sin sentido. Con la cuerda, el maximo cae
    justo en el vertice, que es donde hay que cortar.
    """
    out: List[np.ndarray] = []
    stack = [(0, len(idx) - 1)]
    guard = 0
    while stack and guard < 600:
        guard += 1
        a, b = stack.pop()
        n = b - a + 1
        if n < min_pts:
            continue
        pts = P[idx[a:b + 1]]
        d = pts[-1] - pts[0]
        L = float(np.hypot(d[0], d[1]))
        if L < 1e-6:
            out.append(idx[a:b + 1])
            continue
        nrm = np.array([-d[1], d[0]]) / L
        dist = np.abs((pts - pts[0]) @ nrm)
        k = int(np.argmax(dist))
        # La tolerancia es la del punto donde se pretende cortar: lejos hay que
        # ser mucho mas permisivo o se trocea el muro por puro ruido.
        if dist[k] > tol_pt[idx[a + k]] and 0 < k < n - 1:
            stack.append((a, a + k))
            stack.append((a + k, b))
        else:
            out.append(idx[a:b + 1])
    return out


def _merge_runs(P: np.ndarray, runs: List[np.ndarray],
                tol_pt: np.ndarray) -> List[np.ndarray]:
    """
    Vuelve a unir tramos contiguos que juntos siguen siendo rectos.

    El algoritmo se llama "split-and-merge" y aqui solo estaba implementada la
    mitad de partir. Sin la fusion, la decision de cortar es de todo o nada
    justo en el umbral, asi que un tramo recto se parte o no se parte segun el
    ruido de ese fotograma, y el resultado baila. Fusionar despues estabiliza
    muchisimo la salida.
    """
    out = list(runs)
    changed = True
    while changed and len(out) > 1:
        changed = False
        res: List[np.ndarray] = []
        i = 0
        while i < len(out):
            if i + 1 < len(out) and out[i][-1] + 1 == out[i + 1][0]:
                cand = np.concatenate([out[i], out[i + 1]])
                if _chord_dev(P[cand]) <= float(np.median(tol_pt[cand])):
                    res.append(cand)
                    i += 2
                    changed = True
                    continue
            res.append(out[i])
            i += 1
        out = res
    return out


def segment_boundary(P: np.ndarray, cfg) -> List[Segment]:
    """Parte el contorno ordenado en tramos rectos (cortes por hueco + PCA)."""
    n = len(P)
    if n < int(cfg.seg_min_points):
        return []

    # Tolerancias por punto: crecen con el cuadrado de la distancia, igual que
    # lo hace la incertidumbre de la medida.
    scale = _range_scale(P[:, 0], float(cfg.seg_range_ref_mm))
    tol_pt = float(cfg.seg_split_tol_mm) * scale
    gap_pt = float(cfg.seg_gap_mm) * scale

    d = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    gap_lim = 0.5 * (gap_pt[:-1] + gap_pt[1:])
    cuts = np.where(d > gap_lim)[0] + 1
    runs = np.split(np.arange(n), cuts) if cuts.size else [np.arange(n)]

    minp = int(cfg.seg_min_points)
    side_max = float(cfg.side_max_angle_deg)
    band = float(cfg.side_angle_band_deg)

    pieces: List[np.ndarray] = []
    for r in runs:
        if len(r) < minp:
            continue
        pieces.extend(_split_merge(P, r, tol_pt, minp))
    pieces = _merge_runs(P, pieces, tol_pt)

    segs: List[Segment] = []
    for idx in pieces:
        if len(idx) < minp:
            continue
        pts = P[idx]
        mid, dirv, _, _ = _fit_tls(pts)
        ang = _norm_angle(math.degrees(math.atan2(dirv[1], dirv[0])))
        length = float(np.hypot(*(pts[-1] - pts[0])))
        ymean = float(np.mean(pts[:, 1]))

        # Banda muerta alrededor del umbral de angulo: un tramo que cae
        # justo encima cambiaria de lateral a frontal cada fotograma. En
        # esta pista los muros laterales estan cerca de 0 grados y los
        # frontales cerca de 90, asi que descartar la franja intermedia no
        # cuesta nada y quita el parpadeo.
        if abs(ang) <= side_max - band:
            kind = "side"
        elif abs(ang) >= side_max + band:
            kind = "front"
        else:
            kind = "other"

        # Para llamarlo lateral hay que estar claramente a un lado y tener
        # longitud suficiente: si no, el signo de Y (y por tanto izquierda
        # o derecha) lo decide el ruido.
        if kind == "side" and (abs(ymean) < float(cfg.side_min_y_mm)
                               or length < float(cfg.side_min_len_mm)):
            kind = "other"

        if kind == "side":
            side = "left" if ymean > 0 else "right"
        elif kind == "front":
            side = "front"
        else:
            side = "other"
        segs.append(Segment(
            i0=int(idx[0]), i1=int(idx[-1]), pts=pts, mid=mid, dirv=dirv,
            angle_deg=ang, length_mm=length, kind=kind, side=side,
            x_min=float(pts[:, 0].min()), x_max=float(pts[:, 0].max()),
            y_mean=ymean))
    return segs


# ===========================================================================
#  4) Ajuste de los muros laterales
# ===========================================================================
def _convex_end(P: np.ndarray, seg: Segment, gap_mm: float):
    """
    Comprueba si el tramo termina en una ESQUINA CONVEXA: el punto del contorno
    inmediatamente posterior al extremo lejano del tramo esta mucho mas lejos.
    Ese es exactamente el final del muro interior en una curva.
    """
    far_is_i1 = P[seg.i1, 0] >= P[seg.i0, 0]
    end_i = seg.i1 if far_is_i1 else seg.i0
    nb_i = end_i + 1 if far_is_i1 else end_i - 1
    if nb_i < 0 or nb_i >= len(P):
        return None                       # el muro sale del encuadre: no es final
    r_end = float(np.hypot(P[end_i, 0], P[end_i, 1]))
    r_nb = float(np.hypot(P[nb_i, 0], P[nb_i, 1]))
    if (r_nb - r_end) < gap_mm * 0.55:
        return None                       # continua: esquina concava o muro seguido
    return float(P[end_i, 0])


def _ransac_line(pts: np.ndarray, tol: float, iters: int = 40):
    """RANSAC sencillo para Y = a*X + b. Devuelve (a, b, inliers)."""
    n = len(pts)
    if n < 2:
        return None
    if n <= 4:
        a, b = np.polyfit(pts[:, 0], pts[:, 1], 1)
        return float(a), float(b), np.ones(n, bool)

    rng = np.random.default_rng(12345)
    best_cnt, best = -1, None
    xs, ys = pts[:, 0], pts[:, 1]
    for _ in range(iters):
        i, j = rng.integers(0, n, 2)
        if i == j or abs(xs[i] - xs[j]) < 1.0:
            continue
        a = (ys[j] - ys[i]) / (xs[j] - xs[i])
        b = ys[i] - a * xs[i]
        res = np.abs(ys - (a * xs + b))
        inl = res < tol
        c = int(inl.sum())
        if c > best_cnt:
            best_cnt, best = c, inl
    if best is None or best_cnt < 2:
        a, b = np.polyfit(xs, ys, 1)
        return float(a), float(b), np.ones(n, bool)
    a, b = np.polyfit(xs[best], ys[best], 1)
    return float(a), float(b), best


def fit_side_wall(P: np.ndarray, segs: List[Segment], want: str, cfg):
    """Elige el mejor tramo del lado pedido y ajusta la recta del muro."""
    cands = [s for s in segs
             if s.kind == "side" and s.side == want
             and abs(s.y_mean) < float(cfg.wall_max_y_mm)]
    if not cands:
        return None

    # Preferimos el muro CERCANO y LARGO: es el que gobierna la conduccion.
    def score(s: Segment) -> float:
        return len(s.pts) / (1.0 + abs(s.y_mean) / 450.0)

    seg = max(cands, key=score)

    x_lo, x_hi = float(cfg.fit_x_lo_mm), float(cfg.fit_x_hi_mm)
    m = (seg.pts[:, 0] >= x_lo) & (seg.pts[:, 0] <= x_hi)
    pts = seg.pts[m] if m.sum() >= int(cfg.seg_min_points) else seg.pts
    if len(pts) < 2:
        return None

    r = _ransac_line(pts, float(cfg.ransac_tol_mm))
    if r is None:
        return None
    a, b, inl = r

    eval_x = float(cfg.wall_eval_x_mm)
    y_eval = a * eval_x + b
    # El muro debe seguir en el lado correcto tras el ajuste
    if (want == "left" and y_eval <= 20.0) or (want == "right" and y_eval >= -20.0):
        return None

    span = float(pts[:, 0].max() - pts[:, 0].min())
    quality = float(np.clip(inl.sum() / max(4.0, len(pts)), 0, 1)) * \
              float(np.clip(span / 400.0, 0.15, 1.0))

    return WallFit(
        dist_mm=abs(y_eval),
        angle_deg=math.degrees(math.atan(a)),
        slope=a, offset_mm=b,
        n=int(inl.sum()), quality=quality,
        x_min=float(seg.x_min), x_max=float(seg.x_max),
        end_mm=_convex_end(P, seg, float(cfg.seg_gap_mm)),
    )


# ===========================================================================
#  Analisis completo de un fotograma
# ===========================================================================
def analyze(frame: np.ndarray, ground, cfg, want_mask: bool = False) -> Scene:
    sc = Scene()
    if frame is None:
        return sc
    h, w = frame.shape[:2]

    top = ground.roi_top_row(float(cfg.roi_x_max_mm))
    bottom = h - int(cfg.roi_bottom_crop_px)
    if bottom - top < 20:
        top = max(0, bottom - 20)
    sc.roi_top, sc.roi_bottom = int(top), int(bottom)

    roi = frame[top:bottom]
    mask, thr = build_mask(roi, cfg)
    sc.thresh_used = thr
    if want_mask:
        sc.mask = mask

    cols, rows = floor_boundary(mask, cfg)
    good = ~np.isnan(rows)
    if good.sum() < 6:
        return sc

    u = cols[good].astype(np.float64)
    v = rows[good] + top
    X, Y, ok = ground.image_to_ground(u, v)

    keep = ok & np.isfinite(X) & np.isfinite(Y)
    keep &= (X >= float(cfg.roi_x_min_mm)) & (X <= float(cfg.roi_x_max_mm))
    keep &= np.abs(Y) <= float(cfg.wall_max_y_mm) * 1.6
    if keep.sum() < 6:
        return sc

    sc.boundary_uv = np.stack([u[keep], v[keep]], axis=1)
    P = np.stack([X[keep], Y[keep]], axis=1)
    sc.boundary_xy = P
    sc.n_points = len(P)

    sc.segments = segment_boundary(P, cfg)
    sc.left = fit_side_wall(P, sc.segments, "left", cfg)
    sc.right = fit_side_wall(P, sc.segments, "right", cfg)

    # ---- distancia al frente -------------------------------------------
    # Solo cuentan los puntos que pertenecen a un tramo TRANSVERSAL. Si no se
    # filtra por tipo de tramo, al pasar pegado y algo inclinado respecto a un
    # muro lateral, la banda frontal lo roza en diagonal y devuelve una
    # distancia corta falsa: el robot cree tener un muro delante y gira donde
    # no toca. Este filtro es el que lo evita.
    band = float(cfg.front_band_mm)
    x_max = float(cfg.roi_x_max_mm)
    sc.front_mm = x_max

    front_idx = np.zeros(len(P), bool)
    for s in sc.segments:
        if s.kind == "front":
            front_idx[s.i0:s.i1 + 1] = True
    fm = front_idx & (np.abs(P[:, 1]) < band)
    if fm.sum() >= 3:
        xs = np.sort(P[fm, 0])
        k = max(0, int(len(xs) * 0.20))
        sc.front_mm = float(xs[k])

    # Para la frenada de emergencia si usamos el minimo crudo, pero en una
    # banda tan ancha como el robot: cualquier cosa realmente de frente entra.
    nb = min(band, 110.0)
    em = np.abs(P[:, 1]) < nb
    sc.front_min_mm = float(P[em, 0].min()) if em.sum() >= 3 else x_max

    if sc.left and sc.right:
        sc.corridor_mm = sc.left.dist_mm + sc.right.dist_mm

    sc.ok = True
    return sc
