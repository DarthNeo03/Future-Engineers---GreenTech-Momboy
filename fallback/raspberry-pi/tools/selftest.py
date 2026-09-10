#!/usr/bin/env python3
"""
selftest.py — Pruebas sin camara ni pantalla.  python tools/selftest.py

Genera imagenes sinteticas (pilares rojos/verdes sobre fondo blanco, con
reflejos y ruido) y comprueba que el detector y el archivo de perfiles se
comportan. Correlo en Windows y en la Pi despues de instalar: si pasa todo,
el problema que veas luego es de camara/iluminacion, no de codigo.
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc, vision  # noqa: E402

_fallos = []
_ok = 0


def check(cond, nombre, detalle=""):
    global _ok
    if cond:
        _ok += 1
        print(f"  ok   {nombre}")
    else:
        _fallos.append(nombre)
        print(f"  FALLA {nombre}  {detalle}")


# ---------------------------------------------------------------------------
def lienzo(w=640, h=480, ruido=6):
    img = np.full((h, w, 3), 235, np.uint8)            # blanco de la pista
    r = np.random.default_rng(7).normal(0, ruido, img.shape)
    return np.clip(img.astype(np.int16) + r, 0, 255).astype(np.uint8)


def pilar(img, x, y, w, h, bgr, brillo=False):
    cv2.rectangle(img, (x, y), (x + w, y + h), bgr, -1)
    if brillo:
        # banda blanca horizontal: parte el pilar en dos manchas en la mascara
        cv2.rectangle(img, (x, y + h // 2 - 3), (x + w, y + h // 2 + 3),
                      (250, 250, 250), -1)
    return img


# ---------------------------------------------------------------------------
def test_perfiles():
    print("\n[1] Perfiles / archivo JSON")
    with tempfile.TemporaryDirectory() as d:
        ruta = Path(d) / "colors.json"
        datos = cc.cargar(ruta)
        check(ruta.exists(), "crea el archivo si no existe")
        check(len(datos["perfiles"]) == 1, "arranca con 1 perfil")
        check(set(datos["perfiles"][0]["colores"]) >= {"rojo", "verde", "negro"},
              "trae rojo, verde y negro")
        check(len(datos["perfiles"][0]["colores"]["rojo"]["rangos"]) == 2,
              "el rojo trae 2 rangos (envuelve en H=0)")

        for i in range(7):
            colores = cc.colores_por_defecto()
            colores["rojo"]["area_min"] = 100 + i
            cc.guardar_perfil(datos, f"p{i}", colores)
        cc.guardar(datos, ruta)
        recargado = cc.cargar(ruta)
        check(len(recargado["perfiles"]) == cc.MAX_PERFILES,
              f"nunca guarda mas de {cc.MAX_PERFILES}")
        check(cc.listar(recargado) == ["p6", "p5", "p4", "p3", "p2"],
              "el mas nuevo queda de primero", cc.listar(recargado))
        check(recargado["activo"] == "p6", "el guardado pasa a ser el activo")
        check(cc.obtener(recargado, "p4")["colores"]["rojo"]["area_min"] == 104,
              "los valores sobreviven al round-trip")

        cc.guardar_perfil(recargado, "p6", cc.colores_por_defecto())
        check(len(recargado["perfiles"]) == cc.MAX_PERFILES and
              cc.listar(recargado).count("p6") == 1,
              "regrabar el mismo nombre no gasta cupo")

        # perfil elegido por indice y por nombre
        check(cc.obtener(recargado, 0)["nombre"] == cc.listar(recargado)[0],
              "obtener por indice")

        # archivo corrupto -> se respalda y se regenera
        ruta.write_text("{esto no es json", encoding="utf-8")
        d2 = cc.cargar(ruta)
        check(len(d2["perfiles"]) == 1, "se recupera de un JSON corrupto")
        check(ruta.with_suffix(".json.bak").exists(), "deja respaldo .bak")

        # normalizacion: valores fuera de rango se corrigen
        malo = cc.normalizar_color({"rangos": [[[300, -5, 10], [10, 999, 20]]],
                                    "area_min": 5000, "area_max": 10})
        check(malo["rangos"][0][0][0] <= 179 and malo["rangos"][0][1][1] <= 255,
              "recorta HSV a sus limites")
        check(malo["rangos"][0][0][0] <= malo["rangos"][0][1][0],
              "ordena min<=max en el tono")
        check(malo["area_max"] > malo["area_min"], "corrige area_max <= area_min")


def test_deteccion_basica():
    print("\n[2] Deteccion basica de pilares")
    img = lienzo()
    pilar(img, 100, 200, 40, 90, (40, 40, 210))     # rojo BGR
    pilar(img, 400, 220, 38, 80, (40, 190, 60))     # verde BGR
    colores = cc.colores_por_defecto()
    v = vision.Vision(colores)
    dets, masks = v.procesar(img)

    check(len(dets["rojo"]) == 1, "encuentra 1 pilar rojo", f"{dets['rojo']}")
    check(len(dets["verde"]) == 1, "encuentra 1 pilar verde", f"{dets['verde']}")
    if dets["rojo"]:
        d = dets["rojo"][0]
        check(abs(d.x - 100) <= 4 and abs(d.w - 41) <= 6,
              "bbox del rojo aproximado", f"x={d.x} w={d.w}")
        check(d.llenado > 0.85, "llenado alto en un rectangulo solido",
              f"{d.llenado:.2f}")
        check(1.5 < d.aspecto < 3.0, "aspecto vertical del pilar",
              f"{d.aspecto:.2f}")
        check(abs(d.base_y - 291) <= 5, "base_y sirve como distancia",
              f"{d.base_y}")
        check(d.desviacion(img.shape[1]) < 0, "desviacion negativa = a la izquierda")
    check(len(dets["verde"]) and dets["verde"][0].x > 380, "el verde va a la derecha")


def test_fusion_por_hueco():
    print("\n[3] Fusion de fragmentos (el arreglo del pilar partido por brillo)")
    img = lienzo()
    pilar(img, 300, 150, 44, 100, (40, 40, 210), brillo=True)
    colores = cc.colores_por_defecto()

    sin_union = {k: dict(v) for k, v in colores.items()}
    sin_union["rojo"]["unir_huecos"] = 0
    sin_union["rojo"]["cerrar"] = 0
    sin_union["rojo"]["llenado_min"] = 0.0
    d_sin, _ = vision.Vision(sin_union).procesar(img)
    check(len(d_sin["rojo"]) >= 2, "sin fusion el brillo lo parte en 2+",
          f"{len(d_sin['rojo'])}")

    con_union = {k: dict(v) for k, v in colores.items()}
    con_union["rojo"]["unir_huecos"] = 6
    con_union["rojo"]["cerrar"] = 0
    d_con, _ = vision.Vision(con_union).procesar(img)
    check(len(d_con["rojo"]) == 1, "con unir_huecos vuelve a ser 1 objeto",
          f"{len(d_con['rojo'])}")
    if d_con["rojo"]:
        d = d_con["rojo"][0]
        check(abs(d.h - 101) <= 6, "la altura fusionada es la real, no la dilatada",
              f"h={d.h}")
        check(abs(d.w - 45) <= 6, "el ancho NO se infla por la dilatacion",
              f"w={d.w}")
        check(0.80 < d.llenado < 1.001,
              "el hueco del brillo baja el llenado pero no lo hunde",
              f"{d.llenado:.2f}")


def test_filtros():
    print("\n[4] Filtros (ruido, area, llenado, aspecto)")
    img = lienzo()
    pilar(img, 250, 180, 40, 90, (40, 40, 210))
    rng = np.random.default_rng(3)
    for _ in range(120):                      # motas rojas sueltas
        x, y = rng.integers(0, 620), rng.integers(160, 460)
        cv2.circle(img, (int(x), int(y)), int(rng.integers(1, 3)), (40, 40, 210), -1)

    colores = cc.colores_por_defecto()
    dets, _ = vision.Vision(colores).procesar(img)
    check(len(dets["rojo"]) == 1, "el ruido no genera detecciones extra",
          f"{len(dets['rojo'])}")

    duro = {k: dict(v) for k, v in colores.items()}
    duro["rojo"]["area_min"] = 50000
    check(len(vision.Vision(duro).procesar(img)[0]["rojo"]) == 0,
          "area_min alto descarta todo")

    ancho = {k: dict(v) for k, v in colores.items()}
    ancho["rojo"]["aspecto_min"] = 4.0     # exige mucho mas alto que ancho
    ancho["rojo"]["aspecto_max"] = 9.0
    check(len(vision.Vision(ancho).procesar(img)[0]["rojo"]) == 0,
          "el filtro de aspecto descarta un pilar que no cumple")

    # una mancha en forma de L tiene llenado bajo
    img2 = lienzo()
    cv2.rectangle(img2, (100, 100), (120, 200), (40, 40, 210), -1)
    cv2.rectangle(img2, (100, 180), (200, 200), (40, 40, 210), -1)
    laxo = {k: dict(v) for k, v in colores.items()}
    laxo["rojo"]["llenado_min"] = 0.0
    laxo["rojo"]["usar_aspecto"] = False
    d_laxo, _ = vision.Vision(laxo).procesar(img2)
    check(d_laxo["rojo"] and d_laxo["rojo"][0].llenado < 0.5,
          "una L da llenado bajo",
          f"{d_laxo['rojo'][0].llenado:.2f}" if d_laxo["rojo"] else "sin objeto")
    estricto = {k: dict(v) for k, v in colores.items()}
    estricto["rojo"]["llenado_min"] = 0.6
    estricto["rojo"]["usar_aspecto"] = False
    check(len(vision.Vision(estricto).procesar(img2)[0]["rojo"]) == 0,
          "llenado_min descarta la L")


def test_roi():
    print("\n[5] ROI vertical")
    img = lienzo()
    pilar(img, 200, 20, 40, 60, (40, 40, 210))     # arriba (fuera de pista)
    pilar(img, 400, 300, 40, 90, (40, 40, 210))    # abajo (en pista)
    colores = cc.colores_por_defecto()

    todo = {k: dict(v) for k, v in colores.items()}
    todo["rojo"]["roi_arriba"] = 0.0
    check(len(vision.Vision(todo).procesar(img)[0]["rojo"]) == 2,
          "sin ROI ve los dos")

    recorte = {k: dict(v) for k, v in colores.items()}
    recorte["rojo"]["roi_arriba"] = 0.35
    d, m = vision.Vision(recorte).procesar(img)
    check(len(d["rojo"]) == 1, "con ROI solo ve el de abajo", f"{len(d['rojo'])}")
    check(d["rojo"] and d["rojo"][0].y > 250,
          "las coordenadas siguen siendo del frame completo, no del recorte",
          f"y={d['rojo'][0].y}" if d["rojo"] else "")
    check(int(m["rojo"][0:100].sum()) == 0, "la mascara sale limpia sobre la ROI")


def test_rojo_envuelve():
    print("\n[6] El rojo que envuelve en H=0/179")
    img = lienzo()
    # dos rojos: uno tirando a naranja (H~4) y otro tirando a rosa (H~176)
    pilar(img, 120, 250, 40, 90, (30, 45, 205))
    pilar(img, 420, 250, 40, 90, (70, 30, 200))
    colores = cc.colores_por_defecto()
    dets, _ = vision.Vision(colores).procesar(img)
    check(len(dets["rojo"]) == 2, "los dos tonos de rojo caen en la misma clase",
          f"{len(dets['rojo'])}")


def test_toma_por_clic():
    print("\n[7] Calculo de rangos a partir de un clic")
    hsv_rojo = np.zeros((9, 9, 3), np.uint8)
    hsv_rojo[..., 0] = 178            # justo antes de envolver
    hsv_rojo[..., 1] = 200
    hsv_rojo[..., 2] = 180
    hsv_rojo[0:4, :, 0] = 3           # mitad de la muestra al otro lado del 0
    rangos = vision.rangos_desde_pixeles(hsv_rojo.reshape(-1, 3), margen_h=6)
    check(len(rangos) == 2, "detecta el envolvimiento y devuelve 2 rangos",
          f"{rangos}")
    prueba = np.zeros((1, 2, 3), np.uint8)
    prueba[0, 0] = (178, 200, 180)
    prueba[0, 1] = (3, 200, 180)
    m = vision.mascara_hsv(prueba, rangos)
    check(int(m.sum()) == 2 * 255, "los dos tonos entran en la mascara resultante")

    gris = np.zeros((9, 9, 3), np.uint8)
    gris[..., 0] = np.random.default_rng(1).integers(0, 180, (9, 9))  # tono basura
    gris[..., 1] = 12
    gris[..., 2] = 30
    r2 = vision.rangos_desde_pixeles(gris.reshape(-1, 3))
    check(len(r2) == 1 and r2[0][0][0] == 0 and r2[0][1][0] == 179,
          "en un color acromatico (negro) abre el tono a 0..179", f"{r2}")
    check(r2[0][1][2] < 130, "y deja el valor V bajo para las paredes", f"{r2}")

    # --- el clic que pisa el borde del objeto -----------------------------
    # Parche 9x9: mitad pilar rojo, mitad piso blanco. Sin limpiar, el rango
    # engorda tanto que la mascara se come el piso.
    parche = np.zeros((9, 9, 3), np.uint8)
    parche[..., :] = (2, 210, 200)          # rojo
    parche[5:, :, :] = (0, 4, 235)          # piso blanco (S casi 0)
    ancla = (2, 210, 200)

    sucio = vision.rangos_desde_pixeles(parche.reshape(-1, 3))
    piso = np.array([[[0, 4, 235]]], np.uint8)
    check(int(vision.mascara_hsv(piso, sucio).sum()) > 0,
          "sin limpiar, el parche mixto SI se come el piso (el bug)")

    limpio = vision.nucleo_de_parche(parche, ancla)
    check(limpio.shape[0] == 45, "nucleo_de_parche se queda con los 45 px rojos",
          limpio.shape)
    rangos_l = vision.rangos_desde_pixeles(limpio)
    check(int(vision.mascara_hsv(piso, rangos_l).sum()) == 0,
          "limpiando, el piso queda fuera del rango", rangos_l)
    pilar_px = np.array([[[2, 210, 200], [4, 190, 215]]], np.uint8)
    check(int(vision.mascara_hsv(pilar_px, rangos_l).sum()) == 2 * 255,
          "y el rojo del pilar sigue dentro", rangos_l)

    negro = np.zeros((5, 5, 3), np.uint8)
    negro[..., 0] = np.random.default_rng(2).integers(0, 180, (5, 5))
    negro[..., 1] = 15
    negro[..., 2] = 28
    check(vision.nucleo_de_parche(negro, (90, 15, 28)).shape[0] == 25,
          "con un ancla acromatica no descarta por tono (sirve para el negro)")


def test_negro_pared():
    print("\n[8] Pared negra")
    img = lienzo()
    cv2.rectangle(img, (0, 120), (639, 190), (18, 18, 18), -1)   # franja de pared
    colores = cc.colores_por_defecto()
    dets, _ = vision.Vision(colores).procesar(img)
    check(len(dets["negro"]) >= 1, "detecta la franja de pared",
          f"{len(dets['negro'])}")
    if dets["negro"]:
        d = dets["negro"][0]
        check(d.w > 500, "la pared sale ancha", f"w={d.w}")
        check(d.aspecto < 0.5, "y baja (aspecto<1); por eso lleva usar_aspecto=False")


def test_rendimiento():
    print("\n[9] Rendimiento (3 colores, 640x480)")
    import time
    img = lienzo()
    pilar(img, 100, 200, 40, 90, (40, 40, 210))
    pilar(img, 400, 220, 38, 80, (40, 190, 60))
    cv2.rectangle(img, (0, 120), (639, 160), (18, 18, 18), -1)
    v = vision.Vision(cc.colores_por_defecto())
    v.procesar(img)
    t0 = time.perf_counter()
    N = 60
    for _ in range(N):
        v.procesar(img)
    ms = (time.perf_counter() - t0) / N * 1000
    print(f"       {ms:.1f} ms/frame  (~{1000 / ms:.0f} FPS teoricos en esta maquina)")
    check(ms < 60, "menos de 60 ms por frame", f"{ms:.1f} ms")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print(f"OpenCV {cv2.__version__} · NumPy {np.__version__} · Python {sys.version.split()[0]}")
    for f in (test_perfiles, test_deteccion_basica, test_fusion_por_hueco,
              test_filtros, test_roi, test_rojo_envuelve, test_toma_por_clic,
              test_negro_pared, test_rendimiento):
        f()
    print(f"\n{_ok} pruebas ok, {len(_fallos)} fallos")
    if _fallos:
        for f in _fallos:
            print("  -", f)
        sys.exit(1)
    print("TODO BIEN")
