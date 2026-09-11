#!/usr/bin/env python3
"""
main.py — Piloto WRO 2026 (Open Challenge + Reto con Obstaculos).

    python3 main.py                     # camara + ESP32 + web
    python3 main.py --simulado          # en el PC, sin ESP32
    python3 main.py --imagen foto.png   # sin camara, sobre una foto
    python3 main.py --vmax 90           # tope de PWM solo para esta prueba
    python3 main.py --puerto COM7       # forzar el puerto serie
    python3 main.py --perfil pista_casa # perfil de parametros a cargar
    python3 main.py --reto obstaculos   # RETO CON OBSTACULOS (otra carpeta)
    python3 main.py --sin-web           # COMPETENCIA: solo los pulsadores
    ./piloto.sh arrancar                # lo mismo, como demonio (SSH/VNC)

CADA RETO, SU CARPETA. --reto elige de donde se leen y donde se guardan los
parametros y los colores:

    open        (por defecto)  config/params.json        config/colors.json
    obstaculos                 config/obstaculos/...     config/obstaculos/...
    estacionar                 config/estacionar/...     config/estacionar/...

Asi, calibrar el reto de obstaculos NO puede tocar la calibracion del Open
Challenge, que es la que ya funciona. Sin --reto todo queda exactamente como
estaba.

Web de depuracion: http://carrito.local:8080/ (o la IP de la Pi).

SEGURIDAD, en orden de quien reacciona antes:
  1. El navegador frena solo si el pasillo se cierra (ve venir el muro).
  2. Si el lazo de vision se atasca >250 ms, el enlace manda velocidad 0.
  3. Si el serial se calla >300 ms, el ESP32 corta el motor y centra el servo.
  4. Si la tarea de control del ESP32 se cuelga >200 ms, su vigilante corta.
  5. Ctrl+C manda parada de emergencia antes de salir.
El carro ARRANCA DESARMADO. Se arma con el pulsador de competencia (el start
del reglamento) o con el boton ARMAR de la web. El pulsador es UNO SOLO y
hace las dos cosas: pulsar con el carro parado arma, pulsar con el carro
andando desarma. Cuelga del ESP32, y el ESP32 corta la traccion en el acto
cuando se pulsa con el carro armado. Con --sin-web es el UNICO mando:
comprueba botones.activo antes de la ronda.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

RAIZ = Path(__file__).resolve().parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc          # noqa: E402
from src import params as params_mod        # noqa: E402
from src import robot as robot_mod          # noqa: E402
from src import servidor as srv_mod         # noqa: E402


def avisos_deteccion(p) -> list:
    """Las dos cosas que dejan al carro sin ver los pilares.

    Las dos son silenciosas: el carro no falla, simplemente no detecta, y desde
    el video no se distingue de "el esquive no funciona". Salen en cada
    arranque del reto de obstaculos porque cuestan una tarde cada una.
    """
    avisos = []
    cam, geo = p["camara"], p["geometria"]

    if float(cam.get("balance_blancos", -1.0)) < 0 or \
            float(cam.get("exposicion", -1.0)) == -1.0:
        cuales = []
        if float(cam.get("exposicion", -1.0)) == -1.0:
            cuales.append("exposicion")
        if float(cam.get("balance_blancos", -1.0)) < 0:
            cuales.append("balance de blancos")
        avisos.append(
            "AVISO: " + " y ".join(cuales) + " en AUTO. El tapete es blanco y "
            "brillante: la camara se reajusta sola al girar hacia una pared "
            "clara, el tono de los pilares se mueve con ella y la mascara se "
            "cae unos frames si y otros no. Es la causa numero uno de 'a veces "
            "no ve los cubos'. Congelalos en la pestaña Ajustes.")

    # fx se guarda referido a 640 de ancho y se escala con el ancho de captura.
    # Capturando a 1920 con un fx calibrado a 640 sale tres veces mayor: los
    # milimetros laterales salen tres veces cortos y el punto de paso apunta a
    # donde no es. En el Open Challenge casi no se nota (la distancia al muro
    # sale de las FILAS, o sea de fy); aqui decide por que lado se pasa.
    # Con fx_auto la focal horizontal se toma de la vertical y esto no puede
    # pasar, asi que no hay nada que avisar.
    ancho = int(cam.get("ancho", 640))
    if not bool(geo.get("fx_auto", False)) and ancho != 640:
        fx = float(geo.get("fx_px", 460.0)) * (ancho / 640.0)
        fy = float(geo.get("fy_px", 460.0)) * (int(cam.get("alto", 480)) / 480.0)
        if abs(fx - fy) / max(1.0, fy) > 0.25:
            avisos.append(
                f"AVISO: capturando a {ancho} de ancho, fx efectivo sale "
                f"{fx:.0f} px contra fy {fy:.0f}. En una camara de pixeles "
                "cuadrados son casi iguales, asi que uno de los dos esta mal "
                "calibrado y los milimetros laterales no valen. Comprueba con "
                "'python3 tools/diagnostico_pilares.py' cuanto mide un pilar: "
                "tiene que dar ~50 mm de ancho.")
    return avisos


def main() -> int:
    ap = argparse.ArgumentParser(description="Piloto WRO 2026")
    ap.add_argument("--reto", default="open",
                    choices=("open", "obstaculos", "estacionar"),
                    help="de que carpeta de config se lee y en cual se guarda")
    ap.add_argument("--perfil", default=None, help="perfil de parametros")
    ap.add_argument("--perfil-color", default=None, help="perfil de color")
    ap.add_argument("--imagen", default=None, help="foto fija en vez de camara")
    ap.add_argument("--simulado", action="store_true", help="sin ESP32")
    ap.add_argument("--sin-web", action="store_true")
    ap.add_argument("--vmax", type=int, default=None)
    ap.add_argument("--puerto", default=None)
    args = ap.parse_args()

    ruta_params = params_mod.ruta_de_reto(args.reto)
    ruta_colores = cc.ruta_de_reto(args.reto)
    datos_params = params_mod.cargar(ruta_params)
    if args.perfil:
        datos_params["activo"] = args.perfil
    datos_colores = cc.cargar(ruta_colores)
    if args.perfil_color:
        cc.fijar_activo(datos_colores, args.perfil_color)

    r = robot_mod.Robot(datos_params, datos_colores,
                        simulado=args.simulado, fuente_imagen=args.imagen,
                        reto=args.reto)
    if args.vmax is not None:
        r.p["limites"]["vmax"] = max(0, min(255, args.vmax))
    if args.puerto:
        r.p["enlace"]["puerto"] = args.puerto

    print(f"[main] reto: {args.reto} | config: {ruta_params.parent} | "
          f"perfil params: {datos_params.get('activo')} | "
          f"colores: {r.perfil_color['nombre']} | vmax={r.p['limites']['vmax']}")
    if args.reto != "open" and not bool(r.p["obstaculos"].get("activo")):
        print("[main] AVISO: --reto " + args.reto + " pero obstaculos.activo "
              "esta APAGADO: el carro no va a esquivar nada. Enciendelo en la "
              "web o carga un perfil del reto.")
    if args.reto != "open":
        for aviso in avisos_deteccion(r.p):
            print("[main] " + aviso)
    r.iniciar()

    servidor = None
    if not args.sin_web:
        servidor = srv_mod.Servidor(r)
        url = servidor.iniciar()
        r.log(f"[web] {url} (o http://<ip-de-la-pi>:{r.p['red']['puerto_http']}/)")
    else:
        # Modo competencia: sin web el unico mando son los pulsadores, asi
        # que si estan apagados el carro se queda mudo y mas vale decirlo
        # aqui que descubrirlo con el juez delante.
        b = r.p["botones"]
        if bool(b.get("activo")):
            r.log("[main] sin web: manda el pulsador del ESP32 "
                  "(la misma pulsacion arma y desarma)")
        else:
            r.log("[main] AVISO: --sin-web con botones.activo APAGADO: el "
                  "carro se queda sin ningun mando. Enciende botones.activo "
                  "o arranca sin --sin-web.")

    cerrando = {"si": False}

    def apagar(*_a):
        if cerrando["si"]:
            return
        cerrando["si"] = True
        print("\n[main] parando el carro...")
        try:
            r.emergencia()
            time.sleep(0.15)
        except Exception:
            pass
        try:
            if servidor:
                servidor.cerrar()
        except Exception:
            pass
        try:
            r.cerrar()
        except Exception:
            pass

    signal.signal(signal.SIGINT, lambda *a: (apagar(), sys.exit(0)))
    try:
        signal.signal(signal.SIGTERM, lambda *a: (apagar(), sys.exit(0)))
    except Exception:
        pass

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        apagar()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
