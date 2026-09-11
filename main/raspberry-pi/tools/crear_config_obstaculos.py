#!/usr/bin/env python3
"""
crear_config_obstaculos.py — Siembra config/obstaculos/ a partir de la
configuracion del Open Challenge.

    python3 tools/crear_config_obstaculos.py            # crear si no existe
    python3 tools/crear_config_obstaculos.py --forzar   # rehacer desde cero
    python3 tools/crear_config_obstaculos.py --desde FuncionalV3

POR QUE UNA CARPETA APARTE
Los perfiles ya venian etiquetados por categoria dentro del mismo archivo, y
eso basta para calibrar en el taller. En competencia no: basta un guardado
distraido desde la web con el reto equivocado seleccionado para llevarse por
delante la calibracion del Open Challenge, que es la que ya funciona, y no hay
tiempo de rehacerla. Con archivos separados (main.py --reto obstaculos) tocar
uno no puede estropear el otro.

QUE SE COPIA Y QUE SE CAMBIA
Todo lo de CONDUCIR se copia tal cual del perfil de open que se indique: si el
carro ya rueda bien, tiene que rodar igual. Solo se tocan dos cosas:

  * el grupo 'obstaculos', que en los perfiles de open estaba apagado y con
    valores de haber movido sliders sin efecto (activar a 4 m, semi_pilar de
    55 mm para un pilar de 50x50...);

  * los rangos HSV de rojo y magenta, que es el arreglo que de verdad decide
    el lado de paso. El rojo del reglamento es RGB(238,39,55) -> tono 178 en
    OpenCV, y el magenta RGB(255,0,255) -> tono 150. La calibracion de open
    abria el rojo desde 150, o sea que se tragaba el magenta ENTERO: cada
    delimitador del cajon se veia como un pilar rojo y el carro trataba de
    pasarlo por su derecha. Aqui quedan separados con una banda muerta entre
    162 y 168 para que ninguno pise al otro.

El V minimo del rojo tambien sube (venia en 6): con ese valor cualquier pixel
casi negro de tono rojizo -la sombra del zocalo, el canto de un muro- contaba
como pilar. Un pilar iluminado nunca tiene V de 6.

Y se congela el BALANCE DE BLANCOS. Con el en automatico la camara se reajusta
sola al girar hacia una pared clara, el tono de los pilares se mueve con ella y
la mascara se cae unos frames si y otros no: es la causa numero uno de "a veces
no ve los cubos". La exposicion hay que fijarla a mano mirando el video, porque
el valor util depende de la camara y del sistema.

Nada de esto sustituye a calibrar con la camara en la pista: es un punto de
partida coherente con el reglamento. Calibra desde la web arrancando con
--reto obstaculos, y lo que guardes ira a esta carpeta.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc          # noqa: E402
from src import params as params_mod        # noqa: E402

# --- rangos HSV del reglamento 2026, en escala OpenCV (H 0-179) ------------
# rojo    RGB(238,39,55)  -> H 178   (se envuelve por 0, de ahi los dos tramos)
# verde   RGB( 68,214,44) -> H  56
# magenta RGB(255,  0,255)-> H 150
ROJO = [[[168, 90, 60], [179, 255, 255]],
        [[0, 90, 60], [10, 255, 255]]]
MAGENTA = [[[138, 90, 70], [162, 255, 255]]]

# Valores de arranque del esquive. Los de open no valen: estaban apagados.
OBSTACULOS = {
    "activo": True,
    "activar_desde_mm": 1500.0,   # la seccion recta mide 1 m: ver mas lejos
    "mandar_desde_mm": 1100.0,    # y mandar del todo desde poco despues: el
    #                               cambio de carril necesita ~1 m de recta
    "margen_mm": 70.0,
    "margen_pared_mm": 30.0,      # rozar la pared no penaliza; mover el pilar si
    "semi_pilar_mm": 25.0,        # el pilar mide 50x50 (regla 13.1)
    # --- COMO se aparta: arco por geometria (ver obstaculos.py) ------------
    "modo": "arco",
    "k_arco": 1.0,
    "mirada_mm": 600.0,
    "mirada_min_mm": 350.0,
    "dir_max_pct": 85.0,          # el arco pide a tope solo si hace falta
    "peso_max": 1.0,              # el centrado se calla mientras se esquiva
    "vel_esquive": 40,            # ver un pilar es frenar
    "desvio_max_esquive_deg": 35.0,
    "pilar_parar_mm": 180.0,      # la red: pilar en el corredor y encima -> reversa
    "pilar_salir_mm": 500.0,
    "pilar_escape_dir_pct": 45.0,
    "magenta_semi_mm": 110.0,     # el delimitador mide 200: medio muro
    # --- el paso: al costado y saliendo -----------------------------------
    "no_volver_al_costado": True,
    "costado_desde_mm": 120.0,
    "costado_giro_max_pct": 25.0,
    "recuperar_ms": 1000,
    # --- un pilar no es una pared -----------------------------------------
    "pilares_no_son_muro": True,
    "sombra_pilar_px": 8,
    "pasillo_min_maniobra_mm": 80.0,
    # (los del modo 'punto', por si se vuelve a el)
    "k_dir": 1.4,
    # identificacion (lo que arregla el lado)
    "ignorar_magenta": True,
    "solape_magenta": 0.30,
    "esquivar_magenta": True,
    # 'suave', no 'estricto': la banda de ancho en mm depende de fx, y fx se
    # escala con el ancho de captura. Hasta comprobar con
    # tools/diagnostico_pilares.py que un pilar mide ~50 mm, 'estricto' puede
    # borrarlos todos y dejar al carro ciego sin decir por que.
    "verificar_tamano": "suave",
    "pilar_ancho_min_mm": 25.0,
    "pilar_ancho_max_mm": 120.0,
    "pilar_alto_mm": 100.0,
    "pilar_alto_tol": 0.6,
    "lat_max_mm": 900.0,
    "votos_color": 3,
    "fijar_lado": True,
    "emparejar_mm": 260.0,
    "pista_ms": 700,
    # seguir el pilar cuando la mascara parpadea (el arreglo de "lo esquiva y ya")
    "seguir_a_ciegas": True,
    "ciego_max_ms": 1500,         # con giroscopio la estima aguanta: el pilar
    #                               sale por el canto a 30 cm y hay que seguirlo
    "ciego_vistas_min": 2,
    "peso_ciego": 0.9,
}

# El radio de giro es lo que convierte la curvatura del arco en % de volante.
# 550 es un valor tipico; MIDELO (ver params.py, geometria.radio_giro_mm).
#
# fx_auto: la focal horizontal se toma de la vertical (pixeles cuadrados). Es
# lo que impide que los milimetros LATERALES se descalibren solos cuando la
# camara entrega una resolucion distinta de la que se le pide — y esta la
# entrega: se le piden 1920x480 y da 1280x720, con lo que fx efectivo sale un
# 33 % por encima de fy, las paredes parecen mas cerca de lo que estan y el
# carro no se atreve a pasar aunque tenga sitio de sobra.
GEOMETRIA = {"radio_giro_mm": 550.0, "fx_auto": True}

# La rampa de frenado del perfil de open esta INVERTIDA: vel_giro (78) por
# encima de vel_crucero (74) hace que el carro acelere segun se acerca al
# muro, porque la velocidad interpola de crucero (lejos) a vel_giro (cerca).
# En el reto de obstaculos, ademas, interesa llegar despacio a todo.
LIMITES = {"vel_giro": 55}

# El tono es lo unico que separa al rojo del magenta y al verde del piso, y el
# balance de blancos AUTOMATICO lo mueve solo: la camara se reajusta al girar
# hacia una pared clara y el HSV que calibraste deja de valer. Se congela en un
# valor de pabellon para que la calibracion de color signifique algo. La
# EXPOSICION no se toca aqui a proposito: el valor util depende de la camara y
# del sistema (en Linux/V4L2 son positivos, en Windows/DSHOW negativos), asi
# que clavarlo a ciegas puede dejar la imagen negra. Ajustala tu con el slider
# mirando el video, hasta que el tapete deje de estar quemado.
#
# Y se captura a 640x480, que es la resolucion a la que estan referidas las
# focales: ahi fx y fy salen iguales, que es lo que tiene que pasar en una
# camara de pixeles cuadrados. De paso son tres veces menos pixeles que
# 1280x720, o sea mas FPS en la Pi, que en este reto es tiempo de reaccion.
CAMARA = {"balance_blancos": 4500.0, "ancho": 640, "alto": 480}


def sembrar_params(desde: str, forzar: bool) -> Path:
    destino = params_mod.ruta_de_reto("obstaculos")
    if destino.exists() and not forzar:
        print(f"[params] {destino} ya existe (usa --forzar para rehacerlo)")
        return destino

    datos_open = params_mod.cargar(params_mod.ruta_de_reto("open"))
    base = params_mod.obtener(datos_open, desde or None)
    print(f"[params] copiando la conduccion del perfil open '{base['nombre']}'")

    valores = copy.deepcopy(base["valores"])
    valores["obstaculos"].update(OBSTACULOS)
    valores["geometria"].update(GEOMETRIA)
    valores["camara"].update(CAMARA)
    valores["limites"].update(LIMITES)

    datos = {"version": 1, "activo": "obstaculos_base", "perfiles": []}
    params_mod.guardar_perfil(datos, "obstaculos_base", valores, "obstaculos")
    params_mod.guardar_archivo(datos, destino)
    print(f"[params] escrito {destino}")
    return destino


def sembrar_colores(forzar: bool) -> Path:
    destino = cc.ruta_de_reto("obstaculos")
    if destino.exists() and not forzar:
        print(f"[colores] {destino} ya existe (usa --forzar para rehacerlo)")
        return destino

    datos_open = cc.cargar(cc.ruta_de_reto("open"), crear_si_falta=False)
    base = cc.obtener(datos_open)
    print(f"[colores] copiando el perfil open '{base['nombre']}'")

    colores = copy.deepcopy(base["colores"])
    colores["rojo"]["rangos"] = copy.deepcopy(ROJO)
    colores["magenta"]["rangos"] = copy.deepcopy(MAGENTA)
    # El magenta pasa a ser un objeto de pleno derecho: el esquive necesita su
    # caja para vetar los falsos pilares rojos, no solo su mascara.
    colores["magenta"]["max_objetos"] = 4
    colores["magenta"]["area_min"] = 300

    datos = {"version": cc.VERSION_ESQUEMA, "activo": "obstaculos_base",
             "perfiles": []}
    cc.guardar_perfil(datos, "obstaculos_base", colores,
                      camara=base.get("camara"),
                      notas="rojo y magenta separados segun el reglamento 2026",
                      categoria="obstaculos")
    cc.guardar(datos, destino)
    print(f"[colores] escrito {destino}")
    return destino


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--desde", default="",
                    help="perfil de open del que copiar la conduccion "
                         "(por defecto, el activo)")
    ap.add_argument("--forzar", action="store_true",
                    help="rehacer aunque config/obstaculos/ ya exista")
    args = ap.parse_args()

    sembrar_params(args.desde, args.forzar)
    sembrar_colores(args.forzar)
    print("\nListo. Arranca el reto con:\n"
          "    python3 main.py --reto obstaculos\n"
          "Lo que guardes desde la web ira a config/obstaculos/ y NO tocara "
          "la configuracion del Open Challenge.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
