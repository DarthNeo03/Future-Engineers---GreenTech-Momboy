#!/usr/bin/env python3
"""
selftest_obstaculos.py — Comprobar en el PC que el lado de paso sale bien.

    python3 tools/selftest_obstaculos.py

Esto NO necesita camara, ni ESP32, ni pista. Fabrica detecciones sinteticas
-un pilar de 50x50x100 mm puesto a tal distancia y tal desplazamiento lateral,
proyectado con la MISMA geometria que usa el carro- y comprueba que el
esquivador decide lo que manda el reglamento:

  * rojo   -> el carro pasa por su DERECHA   (regla 9.19)
  * verde  -> el carro pasa por su IZQUIERDA
  * magenta-> no es una señal: no manda lado, y sobre todo NO puede acabar
              tratado como un pilar rojo

Y las tres trampas que hacian que el lado saliera mal en pista:

  1. un delimitador magenta que el rango de rojo tambien detecta;
  2. una mancha del color con tamaño imposible para un pilar a esa distancia;
  3. un detector que parpadea y cambia de color un frame de cada tres.

Antes de tocar el carro, esto tiene que salir en verde. Si falla el caso 2,
mira la calibracion de geometria (fy/fx/altura/inclinacion): la prueba de
tamaño se apoya en ella.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Optional

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import params as params_mod        # noqa: E402
from src import vision                      # noqa: E402
from src.geometria import Geometria         # noqa: E402
from src.obstaculos import Esquivador       # noqa: E402

ANCHO, ALTO = 640, 480


def _geo() -> Geometria:
    cfg = params_mod.valores_por_defecto()["geometria"]
    return Geometria(cfg, ANCHO, ALTO)


def objeto(geo: Geometria, color: str, dist_mm: float, lat_mm: float,
           ancho_mm: float = 50.0, alto_mm: float = 100.0) -> vision.Deteccion:
    """Proyecta un paralelepipedo apoyado en el suelo y devuelve su deteccion,
    igual que la sacaria el detector de color a partir de la imagen."""
    u_izq, v_base = geo.suelo_a_pixel(lat_mm - ancho_mm / 2.0, dist_mm)
    u_der, _ = geo.suelo_a_pixel(lat_mm + ancho_mm / 2.0, dist_mm)
    v_cima = geo.fila_de_altura(dist_mm, alto_mm)
    x, w = min(u_izq, u_der), max(1, abs(u_der - u_izq))
    y, h = v_cima, max(1, v_base - v_cima)
    return vision.Deteccion(color=color, x=x, y=y, w=w, h=h,
                            area=w * h, llenado=1.0, aspecto=h / float(w),
                            cx=x + w / 2.0, cy=y + h / 2.0)


def cfg_obstaculos(**cambios) -> Dict:
    c = params_mod.valores_por_defecto()["obstaculos"]
    c["activo"] = True
    c.update(cambios)
    return c


def lado(esq: Esquivador, dets: Dict[str, List[vision.Deteccion]],
         geo: Geometria, frames: int = 1) -> Optional[str]:
    """Corre 'frames' pasos con las mismas detecciones y devuelve el lado."""
    for _ in range(frames):
        esq.paso(dets, None, geo)
    return esq.info.get("lado")


# ---------------------------------------------------------------------------
PRUEBAS = []


def prueba(nombre):
    def deco(f):
        PRUEBAS.append((nombre, f))
        return f
    return deco


@prueba("rojo se pasa por su DERECHA, este donde este")
def t_rojo():
    geo, fallos = _geo(), []
    for lat in (-400.0, -150.0, 0.0, 150.0, 400.0):
        esq = Esquivador(cfg_obstaculos())
        d = objeto(geo, "rojo", 900.0, lat)
        res = lado(esq, {"rojo": [d], "verde": [], "magenta": []}, geo, 4)
        if res != "derecha":
            fallos.append(f"lat={lat:+.0f} -> {res}")
        # el punto de paso tiene que quedar a la derecha del pilar
        obj = esq.info.get("objetivo_mm", 0)
        if obj <= lat:
            fallos.append(f"lat={lat:+.0f}: objetivo {obj} no esta a su derecha")
    return fallos


@prueba("verde se pasa por su IZQUIERDA, este donde este")
def t_verde():
    geo, fallos = _geo(), []
    for lat in (-400.0, -150.0, 0.0, 150.0, 400.0):
        esq = Esquivador(cfg_obstaculos())
        d = objeto(geo, "verde", 900.0, lat)
        res = lado(esq, {"rojo": [], "verde": [d], "magenta": []}, geo, 4)
        if res != "izquierda":
            fallos.append(f"lat={lat:+.0f} -> {res}")
        obj = esq.info.get("objetivo_mm", 0)
        if obj >= lat:
            fallos.append(f"lat={lat:+.0f}: objetivo {obj} no esta a su izquierda")
    return fallos


@prueba("el delimitador MAGENTA no se trata como pilar rojo")
def t_veto_magenta():
    geo, fallos = _geo(), []
    # El caso real: el rango de rojo empieza en H=150 y tambien dispara sobre
    # el magenta, asi que el MISMO objeto sale en las dos listas.
    mag = objeto(geo, "magenta", 900.0, 300.0, ancho_mm=200.0)
    falso = objeto(geo, "rojo", 900.0, 300.0, ancho_mm=200.0)
    esq = Esquivador(cfg_obstaculos())
    lado(esq, {"rojo": [falso], "verde": [], "magenta": [mag]}, geo, 4)
    if esq.info.get("color") != "magenta":
        fallos.append(f"lo identifico como '{esq.info.get('color')}'")
    if esq.info.get("senal", True):
        fallos.append("lo dio por señal de transito")
    if not esq.info.get("veto_magenta"):
        fallos.append("no conto el veto")
    # esta a la derecha del carro: se rodea por su izquierda
    if esq.info.get("lado") != "izquierda":
        fallos.append(f"lo rodea por {esq.info.get('lado')}, hay mas sitio al otro lado")
    return fallos


@prueba("sin mascara magenta, el tamano real delata al delimitador")
def t_tamano():
    geo, fallos = _geo(), []
    # Segunda linea de defensa: si el magenta esta mal calibrado y no aparece,
    # el delimitador sigue midiendo 200 mm de ancho y un pilar mide 50.
    falso = objeto(geo, "rojo", 900.0, 300.0, ancho_mm=200.0)
    esq = Esquivador(cfg_obstaculos())
    lado(esq, {"rojo": [falso], "verde": [], "magenta": []}, geo, 4)
    if esq.info.get("color"):
        fallos.append(f"se lo trago como '{esq.info['color']}'")
    if not esq.info.get("fuera_tamano"):
        fallos.append("no lo descarto por tamano")
    return fallos


@prueba("una mancha en la pared no pasa por pilar")
def t_mancha():
    geo, fallos = _geo(), []
    # Trozo de zocalo rojizo: a 1.5 m se veria como un pilar de 20 mm de alto.
    d = objeto(geo, "rojo", 1500.0, 0.0, ancho_mm=50.0, alto_mm=20.0)
    esq = Esquivador(cfg_obstaculos())
    lado(esq, {"rojo": [d], "verde": [], "magenta": []}, geo, 4)
    if esq.info.get("color"):
        fallos.append(f"se lo trago como '{esq.info['color']}'")
    return fallos


@prueba("un detector que parpadea NO puede cambiar el lado")
def t_parpadeo():
    geo, fallos = _geo(), []
    # Pilar rojo de verdad, pero un frame de cada tres el detector lo lee como
    # verde. Antes, esos frames mandaban el objetivo al otro lado del pilar.
    esq = Esquivador(cfg_obstaculos(votos_color=3))
    lados = []
    for i in range(15):
        # el carro se acerca: el pilar cambia de sitio poco a poco
        dist = 1400.0 - i * 40.0
        color = "verde" if (i % 3 == 2 and i > 4) else "rojo"
        d = objeto(geo, color, dist, 120.0)
        dets = {"rojo": [], "verde": [], "magenta": []}
        dets[color] = [d]
        esq.paso(dets, None, geo)
        lados.append(esq.info.get("lado"))
    malos = [i for i, l in enumerate(lados) if l != "derecha"]
    if malos:
        fallos.append(f"cambio de lado en los frames {malos}: {lados}")
    if not esq.info.get("fijo"):
        fallos.append("nunca llego a fijar el lado")
    return fallos


@prueba("con obstaculos.activo apagado no se toca la direccion")
def t_apagado():
    geo, fallos = _geo(), []
    esq = Esquivador(cfg_obstaculos(activo=False))
    d = objeto(geo, "rojo", 700.0, 0.0)
    direccion, peso = esq.paso({"rojo": [d], "verde": [], "magenta": []},
                               None, geo)
    if (direccion, peso) != (0.0, 0.0):
        fallos.append(f"devolvio ({direccion}, {peso})")
    return fallos


@prueba("dos pilares: manda el mas cercano y cada uno guarda su lado")
def t_dos():
    geo, fallos = _geo(), []
    esq = Esquivador(cfg_obstaculos())
    cerca = objeto(geo, "verde", 700.0, -150.0)
    lejos = objeto(geo, "rojo", 1400.0, 200.0)
    lado(esq, {"rojo": [lejos], "verde": [cerca], "magenta": []}, geo, 4)
    if esq.info.get("color") != "verde":
        fallos.append(f"manda el '{esq.info.get('color')}' en vez del cercano")
    if esq.info.get("lado") != "izquierda":
        fallos.append(f"al verde cercano lo pasa por {esq.info.get('lado')}")
    if esq.info.get("vistos") != 2:
        fallos.append(f"solo vio {esq.info.get('vistos')} objetos de 2")
    return fallos


def main() -> int:
    print("Comprobando el lado de paso (reglamento 2026, 9.19)\n")
    mal = 0
    for nombre, f in PRUEBAS:
        try:
            fallos = f()
        except Exception as e:                          # noqa: BLE001
            fallos = [f"excepcion: {type(e).__name__}: {e}"]
        if fallos:
            mal += 1
            print(f"  FALLA  {nombre}")
            for d in fallos:
                print(f"           - {d}")
        else:
            print(f"  ok     {nombre}")
    print()
    if mal:
        print(f"{mal} de {len(PRUEBAS)} pruebas FALLAN: no lo subas al carro.")
    else:
        print(f"Las {len(PRUEBAS)} pruebas pasan.")
    return 1 if mal else 0


if __name__ == "__main__":
    raise SystemExit(main())
