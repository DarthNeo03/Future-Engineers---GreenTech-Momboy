# -*- coding: utf-8 -*-
"""
Mide la INESTABILIDAD de la clasificacion con el robot PARADO.

Se renderiza una escena fija y se le anade el ruido que mete una camara real
(ruido del sensor + artefactos de compresion JPEG). El robot no se mueve, asi
que cualquier cambio en las etiquetas de los tramos es inestabilidad del
algoritmo, no del mundo.

Metrica: porcentaje de columnas del contorno que cambian de etiqueta
(izquierda / derecha / frontal / sin clasificar) de un fotograma al siguiente.
"""
import os, sys, numpy as np, cv2
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
from wro.params import Config
from wro.geometry import Ground
from wro import perception as perc
import simulador as S

OUT = os.path.join(HERE, "sim_out")


def noisy(img, sigma=9.0, jpeg=40, rng=None):
    n = rng.normal(0, sigma, img.shape)
    x = np.clip(img.astype(np.float32) + n, 0, 255).astype(np.uint8)
    ok, buf = cv2.imencode(".jpg", x, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR) if ok else x


def labels(sc):
    """columna del contorno -> etiqueta"""
    lab = {}
    for s in sc.segments:
        for i in range(s.i0, s.i1 + 1):
            lab[i] = s.side
    return lab


def measure(cfg, pose, frames=60, seed=7):
    W, H = int(cfg.cam_width), int(cfg.cam_height)
    g = Ground(cfg, W, H)
    base = S.render(g, pose, S.inner_rect(1000, 1000, 1000, 1000), W, H)
    rng = np.random.default_rng(seed)

    prev, flicker, nseg, fronts, izq, der = None, [], [], [], [], []
    for _ in range(frames):
        sc = perc.analyze(noisy(base, rng=rng), g, cfg)
        lab = labels(sc)
        if prev is not None:
            keys = set(lab) | set(prev)
            if keys:
                ch = sum(1 for k in keys if lab.get(k) != prev.get(k))
                flicker.append(100.0 * ch / len(keys))
        prev = lab
        nseg.append(len(sc.segments))
        fronts.append(sc.front_mm)
        izq.append(sc.left.dist_mm if sc.left else np.nan)
        der.append(sc.right.dist_mm if sc.right else np.nan)
    return (np.mean(flicker), np.mean(nseg), np.std(nseg),
            np.nanstd(fronts), np.nanstd(izq), np.nanstd(der),
            100.0 * np.mean(np.isnan(izq)), 100.0 * np.mean(np.isnan(der)))


def report(nombre, cfg, pose):
    f, ns, nsd, sf, si, sd, pi, pd = measure(cfg, pose)
    print("%-26s | %5.1f %% | %4.1f +-%.1f | %6.1f | %5.1f | %5.1f | %3.0f%% %3.0f%%"
          % (nombre, f, ns, nsd, sf, si, sd, pi, pd))


cfg = Config(os.path.join(OUT, "jit.json")); cfg.reset()
cfg.set_many({"cam_pitch_deg": 20.6, "cam_hfov_deg": 80.0, "cam_height_mm": 125.0,
              "cam_offset_x_mm": 172.0, "cam_roll_deg": 1.7, "lens_k1": 0.32})
POSE = (1400, 660, 0.0)     # parado en el carril, con el muro del fondo a 1600

print("                           | parpadeo | tramos      | sigma  | sigma | sigma | perdidas")
print("caso                       |    %     | media  desv | frente | izq   | der   | izq  der")
print("-" * 100)
report("k1=0.32, roi 2200", cfg, POSE)
cfg.set_many({"lens_k1": 0.0})
report("k1=0 (roi 2200)", cfg, POSE)
cfg.set_many({"roi_x_max_mm": 1700.0}); report("k1=0, roi 1700", cfg, POSE)
cfg.set_many({"roi_x_max_mm": 1200.0}); report("k1=0, roi 1200", cfg, POSE)

print()
print("Parpadeo alto o numero de tramos inestable = la clasificacion baila.")
print("Sube seg_range_ref_mm o baja roi_x_max_mm si ves cifras altas.")
