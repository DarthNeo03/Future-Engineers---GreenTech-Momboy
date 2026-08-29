#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Punto de entrada del vehiculo.

    python3 run.py                 # robot + panel web en http://<ip>:8000
    python3 run.py --no-web        # SOLO robot (modo competencia)
    python3 run.py --autostart     # arranca armado, sin esperar boton

En competencia (regla 11.10: prohibida la comunicacion inalambrica) hay que
arrancar con --no-web y ademas apagar la Wi-Fi de la Pi:

    sudo rfkill block wifi bluetooth
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from wro.params import Config          # noqa: E402
from wro.robot import Robot            # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="WRO Future Engineers 2026")
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    ap.add_argument("--no-web", action="store_true",
                    help="no levantar el servidor web (modo competencia)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--mode", default="open",
                    choices=["open", "obstacle", "manual"])
    ap.add_argument("--autostart", action="store_true",
                    help="armar automaticamente al arrancar")
    args = ap.parse_args()

    cfg = Config(args.config)
    if not os.path.isfile(args.config):
        cfg.save()
        print("[wro] creado %s con los valores por defecto" % args.config)

    robot = Robot(cfg, HERE)
    robot.set_mode(args.mode)
    robot.start()
    print("[wro] robot en marcha (modo %s)" % args.mode)

    stopping = {"v": False}

    def shutdown(*_):
        if stopping["v"]:
            return
        stopping["v"] = True
        print("\n[wro] parando...")
        robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    if args.autostart:
        time.sleep(2.0)               # deja que la IMU calibre y la camara abra
        robot.arm()

    if args.no_web:
        print("[wro] servidor web DESACTIVADO. Ctrl+C para salir.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            shutdown()
        return

    from wro.server import create_app
    port = args.port if args.port else int(cfg.web_port)
    app = create_app(robot, cfg)
    print("[wro] panel en http://<ip-de-la-pi>:%d" % port)
    try:
        app.run(host=args.host, port=port, threaded=True,
                debug=False, use_reloader=False)
    finally:
        shutdown()


if __name__ == "__main__":
    main()
