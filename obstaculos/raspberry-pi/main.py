#!/usr/bin/env python3
"""
main.py — Programa piloto del reto de OBSTACULOS.

USO EN COMPETENCIA (lo unico que hace falta saber el dia de la carrera):

    python3 main.py

y despues pulsar el boton del carro cuando el juez diga "¡ya!". El programa
arranca en estado de ESPERA, con el motor cortado por hardware, y no se mueve
hasta ese boton — que es exactamente lo que pide el reglamento (9.11, 9.13).

MODOS DE TRABAJO EN EL TALLER

    --sin-motor      todo corre igual (camara, vision, FSM, telemetria) pero
                     el mando sale siempre con velocidad 0. Es como se depura
                     la logica sin perseguir el carro por el pasillo.
    --ver            abre una ventana con lo que ve el carro. NO usar en
                     competencia: dibujar cuesta ~8 ms por frame.
    --puerto COM7    fuerza el puerto del ESP32 en vez de buscarlo.
    --tiempo 180     corta a los N segundos (para pruebas repetibles).
"""

from __future__ import annotations

import argparse
import sys

from src import config as cfg_mod
from src.piloto import Piloto


def construir_argumentos() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Piloto WRO — reto de obstaculos")
    p.add_argument("--config", default=cfg_mod.RUTA_POR_DEFECTO,
                   help="ruta del JSON de calibracion")
    p.add_argument("--puerto", default=None,
                   help="puerto serial del ESP32 (por defecto, se busca)")
    p.add_argument("--camara", type=int, default=None,
                   help="forzar el indice de camara (/dev/videoN)")
    p.add_argument("--listar-camaras", action="store_true",
                   help="decir que /dev/video* hay y quien los usa, y salir")
    p.add_argument("--sin-motor", action="store_true",
                   help="no mover el motor: solo vision y decisiones")
    p.add_argument("--ver", action="store_true",
                   help="ventana de depuracion (no usar en competencia)")
    p.add_argument("--tiempo", type=float, default=0.0,
                   help="segundos maximos de ejecucion (0 = sin limite)")
    p.add_argument("--silencio", action="store_true",
                   help="sin trazas por consola")
    return p


def main(argv=None) -> int:
    args = construir_argumentos().parse_args(argv)

    if args.listar_camaras:
        from src.camara import dispositivos, quien_la_usa
        hallados = dispositivos()
        if not hallados:
            print("No hay ningun /dev/video*: la camara no esta conectada.")
            return 1
        for i in hallados:
            ocupa = quien_la_usa(i)
            print(f"  /dev/video{i:<3} {'OCUPADO por ' + ocupa if ocupa else 'libre'}")
        print()
        print("Cual de ellos es la de captura: v4l2-ctl --list-devices")
        return 0

    cfg = cfg_mod.cargar(args.config, verbose=not args.silencio)
    if args.camara is not None:
        cfg["camara"]["indice"] = args.camara

    if args.sin_motor:
        # La forma mas segura de "no mover el motor" no es un if en el lazo de
        # decision —que se puede olvidar en una rama— sino poner el techo de
        # PWM a cero: el firmware recorta contra el, asi que ninguna ruta del
        # codigo puede saltarselo.
        cfg["traccion"]["vmax_pwm"] = 0
        if not args.silencio:
            print("[main] MODO SIN MOTOR: vmax_pwm = 0")

    piloto = Piloto(cfg, puerto=args.puerto, verbose=not args.silencio)
    if not piloto.preparar():
        return 1

    if args.ver:
        from src.panel import bucle_con_ventana
        bucle_con_ventana(piloto, duracion_max_s=args.tiempo)
    else:
        piloto.correr(duracion_max_s=args.tiempo)
    return 0


if __name__ == "__main__":
    sys.exit(main())
