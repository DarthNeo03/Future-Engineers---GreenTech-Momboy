#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analiza una captura de fotogramas REALES del robot.

Las escenas sinteticas del simulador son demasiado limpias: el tapete es
uniforme y el muro perfectamente negro, asi que no reproducen los problemas de
verdad (sombras, vineteado, compresion, muros mal iluminados). Esta herramienta
pasa la MISMA cadena de percepcion sobre fotogramas grabados en la pista y
mide lo que el simulador no puede.

Como obtener una captura (con el robot encendido y el panel abierto):

    curl -X POST http://192.168.4.1:8000/api/command \\
         -H "Content-Type: application/json" \\
         -d '{"cmd":"capture","n":40,"nombre":"pasillo_1000"}'

Se guarda en rpi/capturas/<fecha>_<nombre>/ con los PNG sin comprimir, la
configuracion del momento y la telemetria.

Cada captura va a su PROPIA carpeta (lleva fecha y hora en el nombre), asi que
las capturas se acumulan y nunca se pisan entre si. Ocupan unos 250 KB por
fotograma: 10 MB una tanda de 40.

Uso:
    python3 tools/analizar_captura.py capturas/20260830_101500_pasillo_1000
    python3 tools/analizar_captura.py capturas/*            # todas, con comparativa
    python3 tools/analizar_captura.py capturas/* --resumen  # solo la tabla
    python3 tools/analizar_captura.py <carpeta> --set lens_k1=0 --set roi_x_max_mm=1700
    python3 tools/analizar_captura.py <carpeta> --barrer cam_pitch_deg=14:26:1

Con varias carpetas imprime al final una comparativa de estabilidad, que es lo
que sirve para ver en que sitio de la pista se porta peor la calibracion.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

np.seterr(all="ignore")
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from wro.geometry import Ground                       # noqa: E402
from wro.params import Config                          # noqa: E402
from wro import overlay, perception as perc            # noqa: E402


# --------------------------------------------------------------------- carga
def cargar(folder):
    with open(os.path.join(folder, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    imgs = []
    for f in meta["fotogramas"]:
        p = os.path.join(folder, f["file"])
        im = cv2.imread(p)
        if im is not None:
            imgs.append((im, f["stamp"]))
    return meta, imgs


def config_de(meta, overrides, folder):
    cfg = Config(os.path.join(folder, "_cfg_analisis.json"))
    cfg.reset()
    cfg.set_many(meta["config"])
    if overrides:
        cfg.set_many(overrides)
    return cfg


# ------------------------------------------------------------------ informes
def informe_camara(meta, imgs):
    c = meta.get("camara", {})
    print("\n== camara ==")
    print("  formato negociado : %s" % c.get("negociado"))
    print("  fps medidos       : %s   (tope por exposicion: %s)"
          % (c.get("fps"), c.get("tope_fps")))
    print("  exposicion        : %s ms" % c.get("exposicion_ms"))
    print("  tiron maximo      : %s ms" % c.get("tiron_ms"))
    print("  controles via     : %s" % c.get("controles"))

    st = [s for _, s in imgs]
    if len(st) > 2:
        d = np.diff(st) * 1000.0
        print("  intervalos entre fotogramas: media %.1f ms, maximo %.1f ms"
              % (d.mean(), d.max()))
        if d.max() > 2.5 * d.mean():
            print("  AVISO: hay tirones (el maximo mas que duplica la media)")
    if c.get("tope_fps") and c["tope_fps"] < 25:
        print("  AVISO: la exposicion limita los fps. Baja cam_exposure a ~300")
    if c.get("negociado", {}).get("fourcc") not in ("MJPG", None):
        print("  AVISO: el driver NO dio MJPG (%s)"
              % c.get("negociado", {}).get("fourcc"))


def informe_mascara(cfg, ground, imgs):
    print("\n== mascara (separacion muro / tapete) ==")
    thr = int(cfg.wall_v_max)
    top = ground.roi_top_row(float(cfg.roi_x_max_mm))
    bot = imgs[0][0].shape[0] - int(cfg.roi_bottom_crop_px)
    darks, lights, fracs, bajos = [], [], [], []
    for im, _ in imgs:
        roi = im[top:bot]
        v = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[:, :, 2]
        m = v < thr
        fracs.append(float(m.mean()))
        if m.any():
            darks.append(float(v[m].mean()))
        if (~m).any():
            lights.append(float(v[~m].mean()))
        # Franja mas baja del ROI: ahi solo deberia haber tapete. Si sale como
        # muro, el umbral esta demasiado alto o falta luz en esa zona.
        bajos.append(float(m[int(m.shape[0] * 0.8):].mean()))

    print("  umbral en uso     : %d" % thr)
    print("  brillo del muro   : %s"
          % ("%.0f" % np.mean(darks) if darks else "no se ve muro"))
    print("  brillo del tapete : %s"
          % ("%.0f" % np.mean(lights) if lights else "-"))
    sep = (np.mean(lights) - np.mean(darks)) if (darks and lights) else 0.0
    print("  separacion        : %.0f  %s" % (sep,
          "(bien)" if sep > 80 else "(JUSTA, sube la luz o la exposicion)"))
    print("  muro ocupa        : %.1f %% del ROI" % (100 * np.mean(fracs)))
    print("  franja inferior   : %.1f %% marcada como muro" % (100 * np.mean(bajos)))
    if np.mean(bajos) > 0.03:
        print("  AVISO: el tapete de delante se esta tomando por muro. Baja "
              "wall_v_max o sube la exposicion.")


def informe_percepcion(cfg, ground, imgs, folder, guardar=True,
                       silencio=False):
    if not silencio:
        print("\n== percepcion sobre fotogramas reales ==")
    izq, der, fre, anc, nseg, npts = [], [], [], [], [], []
    prev, flick = None, []
    for i, (im, _) in enumerate(imgs):
        sc = perc.analyze(im, ground, cfg)
        npts.append(sc.n_points)
        nseg.append(len(sc.segments))
        fre.append(sc.front_mm)
        izq.append(sc.left.dist_mm if sc.left else np.nan)
        der.append(sc.right.dist_mm if sc.right else np.nan)
        anc.append(sc.corridor_mm if sc.corridor_mm else np.nan)

        lab = {}
        for s in sc.segments:
            for k in range(s.i0, s.i1 + 1):
                lab[k] = s.side
        if prev is not None:
            keys = set(lab) | set(prev)
            if keys:
                flick.append(100.0 * sum(1 for k in keys
                                         if lab.get(k) != prev.get(k)) / len(keys))
        prev = lab

        if guardar and i < 3:
            out = os.path.join(folder, "analisis_%02d.png" % i)
            cv2.imwrite(out, overlay.draw_overlay(im, sc, ground, cfg, {}, None,
                                                  "analisis"))

    def linea(nombre, v, unidad="mm"):
        if silencio:
            return
        v = np.asarray(v, dtype=float)
        ok = ~np.isnan(v)
        if not ok.any():
            print("  %-18s sin medida en ningun fotograma" % nombre)
            return
        print("  %-18s media %7.1f %s   desviacion %6.1f   perdido %3.0f %%"
              % (nombre, np.nanmean(v), unidad, np.nanstd(v),
                 100.0 * (~ok).mean()))

    linea("muro izquierdo", izq)
    linea("muro derecho", der)
    linea("frente", fre)
    linea("ancho de pasillo", anc)
    if not silencio:
        print("  %-18s media %7.1f      desviacion %6.1f"
              % ("puntos contorno", np.mean(npts), np.std(npts)))
    if not silencio:
        print("  %-18s media %7.1f      desviacion %6.1f"
              % ("tramos", np.mean(nseg), np.std(nseg)))
    if not silencio:
        print("  %-18s %6.2f %% de columnas cambian de etiqueta por fotograma"
              % ("parpadeo", np.mean(flick) if flick else 0.0))

    if np.std(nseg) > 0.8 and not silencio:
        print("  AVISO: el numero de tramos baila. Sube seg_range_ref_mm o baja "
              "roi_x_max_mm.")
    a = np.nanmean(anc)
    if not np.isnan(a) and not (500 < a < 1150) and not silencio:
        print("  AVISO: el ancho de pasillo (%.0f mm) no es un valor legal "
              "(600 o 1000 +-100). Revisa cam_hfov_deg." % a)
    return {"izq": np.nanstd(izq), "der": np.nanstd(der),
            "nseg": np.std(nseg), "flick": np.mean(flick) if flick else 0.0}


def barrido(meta, imgs, folder, clave, lo, hi, paso, base_over):
    print("\n== barrido de %s ==" % clave)
    print("  valor  | izq medio | der medio | ancho | tramos desv | parpadeo")
    print("  " + "-" * 68)
    v = lo
    while v <= hi + 1e-9:
        over = dict(base_over); over[clave] = v
        cfg = config_de(meta, over, folder)
        g = Ground(cfg, imgs[0][0].shape[1], imgs[0][0].shape[0])
        izq, der, anc, nseg = [], [], [], []
        for im, _ in imgs[:20]:
            sc = perc.analyze(im, g, cfg)
            izq.append(sc.left.dist_mm if sc.left else np.nan)
            der.append(sc.right.dist_mm if sc.right else np.nan)
            anc.append(sc.corridor_mm if sc.corridor_mm else np.nan)
            nseg.append(len(sc.segments))
        print("  %6.2f | %9s | %9s | %5s | %11.2f | %s"
              % (v,
                 "%.0f" % np.nanmean(izq) if not np.isnan(np.nanmean(izq)) else "-",
                 "%.0f" % np.nanmean(der) if not np.isnan(np.nanmean(der)) else "-",
                 "%.0f" % np.nanmean(anc) if not np.isnan(np.nanmean(anc)) else "-",
                 np.std(nseg), ""))
        v += paso


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpeta", nargs="+",
                    help="una o varias carpetas de captura (admite comodines)")
    ap.add_argument("--set", action="append", default=[], metavar="clave=valor")
    ap.add_argument("--barrer", default=None,
                    metavar="clave=desde:hasta:paso",
                    help="prueba un rango de valores y compara")
    ap.add_argument("--resumen", action="store_true",
                    help="solo la tabla comparativa, sin el detalle de cada una")
    args = ap.parse_args()

    over = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        over[k.strip()] = v.strip()

    carpetas = []
    for pat in args.carpeta:
        if any(c in pat for c in "*?["):
            carpetas.extend(sorted(glob.glob(pat)))
        else:
            carpetas.append(pat)
    carpetas = [c for c in carpetas if os.path.isdir(c)]
    if not carpetas:
        print("no encontre ninguna carpeta de captura")
        return 1

    resumen = []
    for folder in carpetas:
        if not os.path.isfile(os.path.join(folder, "meta.json")):
            print("%s: no parece una captura (falta meta.json)" % folder)
            continue
        meta, imgs = cargar(folder)
        if not imgs:
            print("%s: sin fotogramas legibles" % folder)
            continue

        cfg = config_de(meta, over, folder)
        g = Ground(cfg, imgs[0][0].shape[1], imgs[0][0].shape[0])

        if args.resumen:
            r = informe_percepcion(cfg, g, imgs, folder, guardar=False,
                                   silencio=True)
        else:
            print("\n" + "=" * 78)
            print("captura '%s'  (%s)  -  %d fotogramas de %dx%d"
                  % (meta.get("nombre"), meta.get("fecha"), len(imgs),
                     imgs[0][0].shape[1], imgs[0][0].shape[0]))
            if over:
                print("con cambios:", over)
            print("\n== geometria ==")
            print("  " + ", ".join("%s=%s" % kv for kv in g.describe().items()))
            print("  primera fila util: %d   (horizonte %d)"
                  % (g.roi_top_row(float(cfg.roi_x_max_mm)), g.horizon_row()))
            informe_camara(meta, imgs)
            informe_mascara(cfg, g, imgs)
            r = informe_percepcion(cfg, g, imgs, folder)
            print("\n  superposiciones en %s (analisis_00..02.png)" % folder)

        r["nombre"] = meta.get("nombre") or os.path.basename(folder)
        resumen.append(r)

        if args.barrer:
            k, _, rng = args.barrer.partition("=")
            lo, hi, paso = (float(x) for x in rng.split(":"))
            barrido(meta, imgs, folder, k.strip(), lo, hi, paso, over)

    if len(resumen) > 1:
        print("\n" + "=" * 78)
        print("COMPARATIVA")
        print("  %-22s | izq desv | der desv | tramos desv | parpadeo" % "captura")
        print("  " + "-" * 70)
        for r in resumen:
            print("  %-22s | %8s | %8s | %11.2f | %6.2f %%"
                  % (r["nombre"][:22],
                     "-" if np.isnan(r["izq"]) else "%.1f" % r["izq"],
                     "-" if np.isnan(r["der"]) else "%.1f" % r["der"],
                     r["nseg"], r["flick"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
