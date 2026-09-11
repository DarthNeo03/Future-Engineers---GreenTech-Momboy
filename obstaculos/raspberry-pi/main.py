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
    --lineas         monitor del sensor de color, sin camara y sin motor.
                     Se empuja el carro A MANO sobre cada linea y dice, con
                     numeros, por que la ve o por que no.
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
    p.add_argument("--lineas", action="store_true",
                   help="monitor del sensor de color (empujar el carro a mano)")
    return p


def monitor_lineas(puerto=None) -> int:
    """Mirar el sensor de color con los numeros delante.

    POR QUE ESTO EXISTE. "El carro no ve las lineas" no es un diagnostico: es
    una queja. Puede ser que el piso no devuelva bastante luz, que el color no
    llegue, que el sensor vaya demasiado alto, que este mirando entre dos
    lineas... y cada una se arregla de una forma distinta. El clasificador
    decide con DOS numeros, asi que lo unico que hace falta es verlos mientras
    se empuja el carro a mano por encima de cada linea.

    Lo que hay que mirar es el RESUMEN del final, no el chorro de lineas: dice
    el |sep| maximo que se llego a ver sobre cada color. Si sobre la naranja
    el maximo se queda en 20, el umbral no se toca —ningun umbral arregla una
    señal que no esta— y lo que hay que revisar es la altura del sensor y la
    lente. Si llega a 120 y el umbral esta en 45, es que la linea se cruzo
    demasiado rapido y el problema es de muestreo, no de color.
    """
    import time
    from src import protocolo as proto
    from src.enlace import Enlace

    enlace = Enlace(puerto=puerto, verbose=True)
    if not enlace.abrir():
        print("[lineas] no se pudo abrir el enlace con el ESP32")
        return 1
    enlace.parar()

    print()
    print("Empuja el carro A MANO sobre una linea naranja y sobre una azul.")
    print("Ctrl-C para terminar y ver el resumen.")
    print()
    print(f"{'claro':>7} {'blanco':>7} {'%':>5} {'sep':>6} {'clase':>8}  barra")
    print("-" * 62)

    # Maximos de |sep| vistos, por lo que el firmware decidio que era.
    picos = {"naranja": 0, "azul": 0, "": 0}
    n_nar = n_azu = None
    vistos = {"naranja": 0, "azul": 0}
    sin_diag = False
    try:
        while True:
            est = enlace.estado()
            s_ = est.sens
            if not est.conectado or not est.sensores_frescos:
                print("  ...sin telemetria del ESP32")
                time.sleep(0.5)
                continue
            if not s_.tcs_ok:
                print("  TCS ausente: el ESP32 no lo ve en el bus I2C")
                time.sleep(0.5)
                continue
            if not s_.lineas_diag:
                sin_diag = True

            # Cruces nuevos, para saber cuales llego a CONTAR de verdad.
            if n_nar is None:
                n_nar, n_azu = s_.cruces_naranja, s_.cruces_azul
            if (s_.cruces_naranja - n_nar) & 0xFF:
                vistos["naranja"] += (s_.cruces_naranja - n_nar) & 0xFF
                n_nar = s_.cruces_naranja
            if (s_.cruces_azul - n_azu) & 0xFF:
                vistos["azul"] += (s_.cruces_azul - n_azu) & 0xFF
                n_azu = s_.cruces_azul

            clase = proto.NOMBRE_LINEA.get(s_.clase_linea, "")
            sep = s_.separacion
            picos[clase] = max(picos[clase], abs(sep))
            # El pico del color que TOCARIA por el signo, aunque el firmware
            # no lo haya admitido: es justo el caso que interesa.
            probable = "naranja" if sep > 0 else "azul"
            picos[probable] = max(picos[probable], abs(sep))

            pct = round(100.0 * s_.claro / s_.blanco) if s_.blanco else 0
            barra = "#" * min(40, abs(sep) // 10)
            print(f"{s_.claro:>7} {s_.blanco:>7} {pct:>4}% {sep:>+6} "
                  f"{clase or '-':>8}  {barra}")
            time.sleep(0.08)
    except KeyboardInterrupt:
        pass
    finally:
        enlace.cerrar()

    print()
    print("=" * 62)
    if sin_diag:
        print("El ESP32 tiene firmware viejo: no manda blanco ni separacion.")
        print("Vuelve a subirlo y repite, o esto no dice nada.")
        return 1
    print(f"|sep| maximo sobre NARANJA: {picos['naranja']:>5}   "
          f"(umbral de entrada: 45)")
    print(f"|sep| maximo sobre AZUL:    {picos['azul']:>5}")
    print(f"cruces CONTADOS: naranja {vistos['naranja']}, azul {vistos['azul']}")
    print()
    for color in ("naranja", "azul"):
        pico = picos[color]
        if pico >= 45 and vistos[color] > 0:
            print(f"{color}: bien. Llega color de sobra y se cuenta.")
        elif pico >= 45:
            print(f"{color}: el color LLEGA (|sep| {pico}) pero no se conto "
                  f"ningun cruce.\n"
                  f"    No es umbral: es que la linea se cruzo demasiado "
                  f"rapido o el sensor\n"
                  f"    salio de ella antes de los ms_minimo. Cruzala mas "
                  f"despacio y repite;\n"
                  f"    si asi cuenta, el ajuste es de muestreo, no de color.")
        elif pico > 0:
            print(f"{color}: el color NO llega (|sep| maximo {pico}, hace "
                  f"falta 45).\n"
                  f"    Ningun umbral arregla una señal que no esta: baja el "
                  f"sensor mas cerca\n"
                  f"    del piso, limpia la lente y comprueba que el LED "
                  f"blanco enciende.")
        else:
            print(f"{color}: no se llego a ver nada de ese color.")
    return 0


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

    if args.lineas:
        return monitor_lineas(args.puerto)

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
