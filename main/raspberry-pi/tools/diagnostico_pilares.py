#!/usr/bin/env python3
"""
diagnostico_pilares.py — Por que el carro NO ve ese pilar que esta ahi delante.

    python3 tools/diagnostico_pilares.py                       # camara en vivo
    python3 tools/diagnostico_pilares.py --imagen capturas/x.png
    python3 tools/diagnostico_pilares.py --reto obstaculos --imagen x.png

El video de la web dibuja un recuadro por cada mancha que el detector de color
encontro. Si el pilar se ve perfectamente y NO tiene recuadro, el problema esta
antes del esquive: es la mascara de color o uno de los filtros de forma. Pero
el video no dice CUAL, y ahi se pierden las tardes.

Esto abre la caja. Para cada color imprime:

  * cuanto del cuadro coge la mascara (si el piso sobreexpuesto entra en la
    mascara verde, sus manchas son enormes y se llevan las plazas de
    max_objetos antes que el pilar);
  * las manchas ACEPTADAS con su tamaño real en mm y a que distancia;
  * las manchas RECHAZADAS con el filtro exacto que las mato y por cuanto;
  * que hace despues el esquive con las aceptadas (veto del magenta, prueba de
    tamaño, lado de paso).

Y una comprobacion de geometria que merece la pena mirar la primera vez:

  Un pilar del reglamento mide 50 mm de ancho. Si el diagnostico dice que el
  pilar que tienes delante mide 17 mm, fx esta 3 veces largo — es lo que pasa
  al capturar a 1920 de ancho con un fx calibrado a 640, porque fx se escala
  con el ancho. Con fx mal, 'verificar_tamano' en estricto borra TODOS los
  pilares, y el punto de paso en modo 'punto' apunta a donde no es.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc          # noqa: E402
from src import params as params_mod        # noqa: E402
from src import vision                      # noqa: E402
from src.geometria import Geometria         # noqa: E402
from src.obstaculos import Esquivador       # noqa: E402

COLORES_PILAR = ("rojo", "verde", "magenta")


def leer_frame(args) -> np.ndarray:
    if args.imagen:
        img = cv2.imread(args.imagen)
        if img is None:
            raise SystemExit(f"no se pudo leer {args.imagen}")
        return img
    cam = params_mod.obtener(params_mod.cargar(
        params_mod.ruta_de_reto(args.reto)))["valores"]["camara"]
    cap = cv2.VideoCapture(int(cam.get("indice", 0)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cam.get("ancho", 640)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cam.get("alto", 480)))
    for _ in range(8):                       # que la exposicion se asiente
        ok, img = cap.read()
    cap.release()
    if not ok or img is None:
        raise SystemExit("la camara no dio imagen")
    return img


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--reto", default="obstaculos",
                    choices=("open", "obstaculos", "estacionar"))
    ap.add_argument("--imagen", default=None)
    ap.add_argument("--guardar", default=None,
                    help="PNG donde dejar el frame con las manchas marcadas")
    args = ap.parse_args()

    datos_p = params_mod.cargar(params_mod.ruta_de_reto(args.reto))
    valores = params_mod.obtener(datos_p)["valores"]
    perfil_c = cc.obtener(cc.cargar(cc.ruta_de_reto(args.reto),
                                    crear_si_falta=False))

    frame = leer_frame(args)
    H, W = frame.shape[:2]
    geo = Geometria(valores["geometria"], W, H)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    print(f"\nimagen {W}x{H}   perfil params '{datos_p.get('activo')}'   "
          f"colores '{perfil_c['nombre']}'")

    # ---- exposicion y balance -------------------------------------------
    v_med = float(np.mean(hsv[:, :, 2]))
    s_med = float(np.mean(hsv[:, :, 1]))
    quemado = float(np.mean(hsv[:, :, 2] > 250)) * 100.0
    print(f"luz: V media {v_med:.0f}/255   S media {s_med:.0f}/255   "
          f"quemado {quemado:.1f}% de pixeles")
    auto_exp = float(valores["camara"].get("exposicion", -1.0)) < 0
    auto_wb = float(valores["camara"].get("balance_blancos", -1.0)) < 0
    if auto_exp or auto_wb:
        cuales = " y ".join([t for t, c in (("exposicion", auto_exp),
                                            ("balance de blancos", auto_wb)) if c])
        print(f"  !! {cuales} en AUTO. El tapete es blanco y brillante: la")
        print("     camara se reajusta sola al girar hacia una pared clara, el")
        print("     HSV del pilar se mueve con ella y la mascara se cae unos")
        print("     frames si y otros no. Es la causa numero uno de 'a veces no")
        print("     ve los cubos'. Congelalos ANTES de calibrar colores.")
    if quemado > 5.0:
        print("  !! LA IMAGEN ESTA QUEMADA. Con exposicion y balance de")
        print("     blancos en AUTO el HSV cambia solo al girar hacia una pared")
        print("     clara, y la mascara del pilar se cae unos frames si y otros")
        print("     no. Congelalos (camara.exposicion / balance_blancos) ANTES")
        print("     de calibrar colores, o la calibracion no vale para nada.")

    # ---- geometria -------------------------------------------------------
    print(f"\ngeometria: fy={geo._fy:.0f}px fx={geo._fx:.0f}px "
          f"(guardados {valores['geometria']['fy_px']:.0f}/"
          f"{valores['geometria']['fx_px']:.0f} referidos a 640x480)")
    if abs(geo._fx - geo._fy) / max(1.0, geo._fy) > 0.25:
        print(f"  !! fx y fy se llevan un {abs(geo._fx-geo._fy)/geo._fy*100:.0f}%.")
        print("     En una camara normal (pixeles cuadrados) son casi iguales.")
        print(f"     Como fx se escala con el ancho, capturar a {W} con un fx")
        print("     calibrado a 640 lo multiplica por "
              f"{W/640.0:.1f}. Comprueba abajo cuanto mide un pilar: tiene que")
        print("     dar ~50 mm de ancho y ~100 mm de alto.")

    # ---- deteccion por color --------------------------------------------
    esq_cfg = dict(valores["obstaculos"])
    esq_cfg["activo"] = True
    esq = Esquivador(esq_cfg)
    dets: Dict[str, List[vision.Deteccion]] = {}

    for color in COLORES_PILAR:
        par = perfil_c["colores"].get(color)
        if par is None:
            print(f"\n[{color}] no esta en el perfil de colores")
            continue
        det = vision.DetectorColor(color, par)
        descartes: List[Dict[str, Any]] = []
        objetos, mascara = det.detectar(hsv, descartes=descartes)
        dets[color] = objetos
        cobertura = float(np.count_nonzero(mascara)) / (W * H) * 100.0

        print(f"\n[{color}] mascara {cobertura:.1f}% del cuadro   "
              f"rangos {par['rangos']}")
        if cobertura > 12.0:
            print("  !! la mascara coge medio cuadro: el rango esta abierto de")
            print("     mas. Con unir_huecos, esa mancha gigante se PEGA al")
            print("     pilar y los dos se caen juntos por area_max.")
        print(f"  aceptadas: {len(objetos)}   rechazadas: {len(descartes)}"
              f"   (max_objetos={par.get('max_objetos')})")

        for o in objetos:
            dist_cam = float(geo.fila_a_distancia(o.base_y))
            lat = float(geo.lateral_mm(o.cx, o.base_y))
            izq = float(geo.lateral_mm(float(o.x), float(o.base_y)))
            der = float(geo.lateral_mm(float(o.x + o.w), float(o.base_y)))
            alto_esp = geo.alto_esperado_px(max(1.0, dist_cam), 100.0)
            print(f"   OK  bbox=({o.x},{o.y},{o.w},{o.h}) area={o.area} "
                  f"lleno={o.llenado:.2f} asp={o.aspecto:.2f}")
            print(f"       a {dist_cam:.0f}mm, lateral {lat:+.0f}mm | "
                  f"mide {abs(der-izq):.0f}mm de ancho "
                  f"(un pilar mide 50) | alto {o.h}px, "
                  f"un pilar de 100mm ahi mediria {alto_esp}px")
        for d in descartes[:8]:
            extra = {k: v for k, v in d.items()
                     if k not in ("filtro", "x", "y", "w", "h")}
            print(f"   NO  {d['filtro']:<18} bbox=({d['x']},{d['y']},"
                  f"{d['w']},{d['h']}) {extra}")
        if len(descartes) > 8:
            print(f"   ... y {len(descartes)-8} rechazadas mas")

    # ---- que hace el esquive con todo eso --------------------------------
    print("\n=== decision del esquive ===")
    for _ in range(4):            # varios frames: la votacion necesita votos
        direccion, peso = esq.paso(dets, None, geo, None, False, 0, 400.0)
    info = esq.info
    if not info.get("color"):
        print("  NO hay ningun obstaculo en juego.")
        motivos = ("tras_linea", "fuera_tamano", "veto_magenta",
                   "otro_carril", "fuera_alcance")
        for k in motivos:
            if info.get(k):
                print(f"    descartados por {k}: {info[k]}")
        if info.get("descarte"):
            print(f"    motivo del primero: {info['descarte']}")
        if not any(info.get(k) for k in motivos):
            print("    ninguna mancha llego siquiera al esquive: el problema")
            print("    esta en la mascara de color o en los filtros de forma.")
    else:
        print(f"  {info['color']} a {info['dist_mm']}mm, lateral "
              f"{info['lat_mm']:+}mm -> pasa por su {info['lado'].upper()}"
              f"{' [FIJO]' if info.get('fijo') else ''}")
        print(f"  senal de transito: {info.get('senal')}   "
              f"ancho medido {info.get('ancho_mm')}mm   "
              f"direccion {direccion:+.0f}% peso {peso:.2f}")
        for k in ("tras_linea", "fuera_tamano", "veto_magenta",
                  "otro_carril", "fuera_alcance"):
            if info.get(k):
                print(f"    (ademas se descartaron {info[k]} por {k})")

    if args.guardar:
        marcado = frame.copy()
        for color, lista in dets.items():
            bgr = tuple(int(v) for v in
                        perfil_c["colores"][color].get("color_dibujo",
                                                       [255, 255, 255]))
            for o in lista:
                cv2.rectangle(marcado, (o.x, o.y), (o.x + o.w, o.y + o.h), bgr, 2)
        cv2.imwrite(args.guardar, marcado)
        print(f"\nguardado {args.guardar}")
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
