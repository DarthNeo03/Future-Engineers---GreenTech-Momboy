#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reconstruye una ronda grabada: video anotado + linea de tiempo de decisiones.

Mientras las capturas PNG sirven para medir PRECISION en un punto fijo, esto
sirve para entender la DINAMICA de una vuelta: donde disparo cada curva, donde
perdio el muro interior, y que estaba midiendo justo antes de irse.

Para grabar: activa `record_run` en el panel y arma una ronda. Se guarda en
rpi/grabaciones/<fecha>_<modo>/ con los fotogramas en JPEG y un frames.csv que
lleva, para cada uno, el estado del control en ese instante.

Uso:
    python3 tools/reproducir_ronda.py grabaciones/20260830_101500_open
    python3 tools/reproducir_ronda.py <carpeta> --set turn_trigger_front_mm=760
    python3 tools/reproducir_ronda.py <carpeta> --sin-video

AVISO IMPORTANTE sobre --set: la percepcion se vuelve a calcular con los
parametros nuevos, pero la TRAYECTORIA es la que fue. Sirve para responder
"¿que habria medido con este ajuste?", no "¿que habria hecho el robot?": para
eso esta el simulador, porque el control es en lazo cerrado y cualquier cambio
habria movido el robot a otro sitio.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import cv2
import numpy as np

np.seterr(all="ignore")

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from wro.geometry import Ground                       # noqa: E402
from wro.params import Config                          # noqa: E402
from wro import overlay, perception as perc            # noqa: E402


# Parametros que SI cambian lo que se mide al reproducir. El resto los usa
# unicamente el controlador, que aqui no se vuelve a ejecutar.
PERCEPCION = {
    "cam_height_mm", "cam_pitch_deg", "cam_roll_deg", "cam_hfov_deg",
    "cam_cx_off", "cam_cy_off", "lens_k1", "lens_k2",
    "cam_offset_x_mm", "cam_offset_y_mm",
    "wall_v_max", "wall_s_max", "wall_auto_thresh", "wall_min_run_px",
    "roi_bottom_crop_px", "roi_x_min_mm", "roi_x_max_mm", "col_step",
    "boundary_median", "morph_open",
    "seg_split_tol_mm", "seg_gap_mm", "seg_min_points", "seg_range_ref_mm",
    "side_max_angle_deg", "side_angle_band_deg", "side_min_y_mm",
    "side_min_len_mm", "front_band_mm", "wall_max_y_mm", "wall_eval_x_mm",
    "fit_x_lo_mm", "fit_x_hi_mm", "ransac_tol_mm", "wall_end_max_mm",
}


def cargar(folder):
    with open(os.path.join(folder, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    filas = []
    with open(os.path.join(folder, "frames.csv"), encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            filas.append(r)
    return meta, filas


def num(r, k):
    v = r.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def linea_de_tiempo(filas):
    """Momentos en que cambia el estado o la nota del controlador."""
    print("\n== linea de tiempo ==")
    print("  tiempo | estado      | esq | vta | frente | int  | motivo")
    print("  " + "-" * 76)
    prev_estado, prev_nota = None, None
    for r in filas:
        est, nota = r["estado"], r["nota"]
        if est == prev_estado and nota == prev_nota:
            continue
        prev_estado, prev_nota = est, nota
        print("  %6.1f | %-11s | %3s | %3s | %6s | %4s | %s"
              % (num(r, "t") or 0.0, est, r["esquinas"], r["vueltas"],
                 r["frente"] or "-", r["d_int"] or "-", nota[:34]))


def resumen(filas):
    print("\n== resumen de la ronda ==")
    if not filas:
        print("  sin datos")
        return
    t = num(filas[-1], "t") or 0.0
    esq = filas[-1]["esquinas"]
    vta = filas[-1]["vueltas"]
    print("  duracion %.1f s, %s esquinas, %s vueltas, final en %s"
          % (t, esq, vta, filas[-1]["estado"]))

    for campo, nombre in (("d_int", "muro interior"), ("d_ext", "muro exterior"),
                          ("frente", "frente"), ("ancho", "ancho de pasillo")):
        v = np.array([num(r, campo) for r in filas], dtype=float)
        ok = ~np.isnan(v)
        if ok.sum() < 2:
            print("  %-16s sin medidas" % nombre)
            continue
        print("  %-16s media %6.0f mm   desviacion %5.0f   sin medida %3.0f %%"
              % (nombre, np.nanmean(v), np.nanstd(v), 100.0 * (~ok).mean()))

    e = np.array([num(r, "err_rumbo") or 0.0 for r in filas])
    seguim = np.array([r["estado"] == "SIGUE_MURO" for r in filas])
    if seguim.any():
        print("  error de rumbo en recta: media %.1f, maximo %.1f grados"
              % (np.abs(e[seguim]).mean(), np.abs(e[seguim]).max()))
    for est in ("RECUPERANDO", "GIRO"):
        n = sum(1 for r in filas if r["estado"] == est)
        if n:
            print("  %-16s %d fotogramas (%.1f s)"
                  % (est.lower(), n, n * (t / max(1, len(filas)))))


def comparar_recalculado(folder, filas, cfg, ground):
    """
    Vuelve a pasar la percepcion sobre todos los fotogramas y compara con lo
    que se midio de verdad. Sin esto, --set solo cambiaria el video y las
    cifras del resumen seguirian siendo las grabadas, que es enganoso.
    """
    print("\n== grabado contra recalculado ==")
    nuevo = {"d_int": [], "d_ext": [], "frente": [], "ancho": []}
    for r in filas:
        im = cv2.imread(os.path.join(folder, r["file"]))
        if im is None:
            continue
        sc = perc.analyze(im, ground, cfg)
        nuevo["d_int"].append(sc.left.dist_mm if sc.left else np.nan)
        nuevo["d_ext"].append(sc.right.dist_mm if sc.right else np.nan)
        nuevo["frente"].append(sc.front_mm)
        nuevo["ancho"].append(sc.corridor_mm if sc.corridor_mm else np.nan)

    # Ojo: izquierda/derecha del recalculo no se remapean a interior/exterior
    # (haria falta el sentido de marcha de cada instante), asi que se comparan
    # por lado. Para el Reto Abierto el sentido es constante en toda la ronda.
    print("  %-10s | %-28s | %-28s" % ("magnitud", "grabado", "recalculado"))
    print("  " + "-" * 72)
    for campo, nombre in (("d_int", "lado izq."), ("d_ext", "lado der."),
                          ("frente", "frente"), ("ancho", "ancho")):
        v0 = np.array([num(r, campo) for r in filas], dtype=float)
        v1 = np.array(nuevo[campo], dtype=float)

        def desc(v):
            ok = ~np.isnan(v)
            if ok.sum() < 2:
                return "sin medidas"
            return ("media %6.0f  desv %5.0f  perdido %3.0f %%"
                    % (np.nanmean(v), np.nanstd(v), 100.0 * (~ok).mean()))

        print("  %-10s | %-28s | %-28s" % (nombre, desc(v0), desc(v1)))


def video(folder, meta, filas, cfg, ground, salida, recalcular):
    im0 = cv2.imread(os.path.join(folder, filas[0]["file"]))
    if im0 is None:
        print("no se pueden leer los fotogramas")
        return
    h, w = im0.shape[:2]
    fps = float(meta.get("config", {}).get("record_fps", 10)) or 10.0
    vw = cv2.VideoWriter(salida, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    for r in filas:
        im = cv2.imread(os.path.join(folder, r["file"]))
        if im is None:
            continue
        sc = perc.analyze(im, ground, cfg)
        # Se dibuja con el estado que el robot TENIA en ese instante, no con uno
        # recalculado: asi el video muestra lo que de verdad decidio.
        snap = {"state": r["estado"],
                "direction_txt": {"1": "antihorario", "-1": "horario"}.get(
                    r["sentido"], "?"),
                "laps": r["vueltas"], "corners": r["esquinas"],
                "d_inner": num(r, "d_int"), "d_outer": num(r, "d_ext"),
                "front": num(r, "frente"), "inner_end": num(r, "fin_int"),
                "corridor": num(r, "ancho"), "steer": num(r, "dir"),
                "speed": num(r, "vel"), "head_err": num(r, "err_rumbo"),
                "head_corr": num(r, "corr_rumbo"), "note": r["nota"]}
        if recalcular:
            snap["note"] = "RECALCULADO: " + r["nota"]
        extra = "t=%ss  %s" % (r["t"], os.path.basename(folder))
        vw.write(overlay.draw_overlay(im, sc, ground, cfg, snap, None, extra))
    vw.release()
    print("\n  video: %s" % salida)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("carpeta")
    ap.add_argument("--set", action="append", default=[], metavar="clave=valor")
    ap.add_argument("--sin-video", action="store_true")
    args = ap.parse_args()

    folder = args.carpeta
    if not os.path.isfile(os.path.join(folder, "frames.csv")):
        print("no parece una grabacion (falta frames.csv):", folder)
        return 1
    meta, filas = cargar(folder)
    if not filas:
        print("la grabacion esta vacia")
        return 1

    over = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        over[k.strip()] = v.strip()

    cfg = Config(os.path.join(folder, "_cfg_replay.json"))
    cfg.reset()
    cfg.set_many(meta["config"])
    if over:
        cfg.set_many(over)

    im0 = cv2.imread(os.path.join(folder, filas[0]["file"]))
    ground = Ground(cfg, im0.shape[1], im0.shape[0])

    print("ronda '%s'  (%s)  -  %d fotogramas"
          % (meta.get("modo"), meta.get("fecha"), len(filas)))
    if over:
        print("percepcion recalculada con:", over)
        print("(la trayectoria es la que fue; esto responde que habria MEDIDO,")
        print(" no que habria HECHO. Para eso esta el simulador.)")
        # Los parametros que solo usa el control no cambian NADA aqui, porque
        # esta herramienta no vuelve a ejecutar el controlador: reproduce el
        # que ya se ejecuto. Decirlo evita perder el tiempo probandolos.
        inutiles = [k for k in over if k not in PERCEPCION]
        if inutiles:
            print("\n  AVISO: %s no afecta%s a la reproduccion."
                  % (", ".join(inutiles), "" if len(inutiles) > 1 else ""))
            print("  Solo los usa el CONTROL, y aqui el control no se vuelve a")
            print("  ejecutar. Para probar cambios de control usa:")
            print("      python3 tools/simulador.py --set %s"
                  % " --set ".join("%s=%s" % (k, over[k]) for k in inutiles))

    resumen(filas)
    if over and any(k in PERCEPCION for k in over):
        comparar_recalculado(folder, filas, cfg, ground)
    linea_de_tiempo(filas)
    if not args.sin_video:
        sufijo = "_recalculado" if over else ""
        video(folder, meta, filas, cfg, ground,
              os.path.join(folder, "ronda%s.mp4" % sufijo), bool(over))
    return 0


if __name__ == "__main__":
    sys.exit(main())
