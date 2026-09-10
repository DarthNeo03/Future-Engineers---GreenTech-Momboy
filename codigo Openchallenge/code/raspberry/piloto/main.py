#!/usr/bin/env python3
"""
main.py — Piloto WRO 2026 (Open Challenge + deteccion de obstaculos).

    python3 main.py                     # camara + ESP32 + web
    python3 main.py --simulado          # en el PC, sin ESP32
    python3 main.py --imagen foto.png   # sin camara, sobre una foto
    python3 main.py --vmax 90           # tope de PWM solo para esta prueba
    python3 main.py --puerto COM7       # forzar el puerto serie
    python3 main.py --perfil pista_casa # perfil de parametros a cargar

Web de depuracion: http://carrito.local:8080/ (o la IP de la Pi).

SEGURIDAD, en orden de quien reacciona antes:
  1. El navegador frena solo si el pasillo se cierra (ve venir el muro).
  2. Si el lazo de vision se atasca >250 ms, el enlace manda velocidad 0.
  3. Si el serial se calla >300 ms, el ESP32 corta el motor y centra el servo.
  4. Si la tarea de control del ESP32 se cuelga >200 ms, su vigilante corta.
  5. Ctrl+C manda parada de emergencia antes de salir.
El carro ARRANCA DESARMADO: hay que pulsar ARMAR en la web.
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


def main() -> int:
    ap = argparse.ArgumentParser(description="Piloto WRO 2026")
    ap.add_argument("--perfil", default=None, help="perfil de parametros")
    ap.add_argument("--perfil-color", default=None, help="perfil de color")
    ap.add_argument("--imagen", default=None, help="foto fija en vez de camara")
    ap.add_argument("--simulado", action="store_true", help="sin ESP32")
    ap.add_argument("--sin-web", action="store_true")
    ap.add_argument("--vmax", type=int, default=None)
    ap.add_argument("--puerto", default=None)
    args = ap.parse_args()

    datos_params = params_mod.cargar()
    if args.perfil:
        datos_params["activo"] = args.perfil
    datos_colores = cc.cargar()
    if args.perfil_color:
        cc.fijar_activo(datos_colores, args.perfil_color)

    r = robot_mod.Robot(datos_params, datos_colores,
                        simulado=args.simulado, fuente_imagen=args.imagen)
    if args.vmax is not None:
        r.p["limites"]["vmax"] = max(0, min(255, args.vmax))
    if args.puerto:
        r.p["enlace"]["puerto"] = args.puerto

    print(f"[main] perfil params: {datos_params.get('activo')} | "
          f"colores: {r.perfil_color['nombre']} | vmax={r.p['limites']['vmax']}")
    r.iniciar()

    servidor = None
    if not args.sin_web:
        servidor = srv_mod.Servidor(r)
        url = servidor.iniciar()
        r.log(f"[web] {url} (o http://<ip-de-la-pi>:{r.p['red']['puerto_http']}/)")

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
