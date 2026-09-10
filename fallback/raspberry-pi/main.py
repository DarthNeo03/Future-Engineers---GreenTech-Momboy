#!/usr/bin/env python3
"""
main.py — Programa principal del carro WRO Future Engineers.

    python3 main.py                     # camara + ESP32 + web + panel
    python3 main.py --sin-panel         # tipico en la Pi sin monitor
    python3 main.py --sin-web
    python3 main.py --simulado          # en el PC, sin ESP32 conectado
    python3 main.py --imagen foto.png   # sin camara, sobre una foto
    python3 main.py --perfil calib_0819_1501
    python3 main.py --vmax 90           # tope de velocidad para esta prueba

Objetivo de esta fase: dar vueltas a la pista vacia sin tocar los muros.

SEGURIDAD, en orden de quien reacciona antes:
  1. El navegador frena solo si el pasillo se cierra (ve venir el muro).
  2. Si el lazo de vision se atasca >250 ms, el enlace manda velocidad 0.
  3. Si el serial se calla >300 ms, el ESP32 corta el motor y centra el servo.
  4. Si la tarea de control del ESP32 se cuelga >200 ms, su vigilante corta el
     PWM directamente sobre el hardware.
  5. Ctrl+C o cerrar la ventana manda parada de emergencia antes de salir.
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

from src import color_config as cc, robot_config, robot as robot_mod  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="Carro WRO Future Engineers")
    ap.add_argument("--config", default=None, help="ruta de robot.json")
    ap.add_argument("--perfil", default=None, help="perfil de color a usar")
    ap.add_argument("--imagen", default=None, help="usar una foto en vez de la camara")
    ap.add_argument("--simulado", action="store_true",
                    help="no abrir el puerto serie (pruebas en el PC)")
    ap.add_argument("--sin-web", action="store_true")
    ap.add_argument("--sin-panel", action="store_true")
    ap.add_argument("--vmax", type=int, default=None,
                    help="tope de PWM 0-255 solo para esta ejecucion")
    ap.add_argument("--puerto", default=None, help="forzar el puerto serie")
    args = ap.parse_args()

    cfg = robot_config.cargar(args.config)
    if args.vmax is not None:
        cfg["limites"]["vmax"] = max(0, min(255, args.vmax))
    if args.puerto:
        cfg["enlace"]["puerto"] = args.puerto

    perfil_color = cc.obtener(cc.cargar(), args.perfil)
    print(f"[main] perfil de color: {perfil_color['nombre']}")
    print(f"[main] vmax = {cfg['limites']['vmax']} (tope duro en el ESP32)")

    r = robot_mod.Robot(cfg, perfil_color, simulado=args.simulado,
                        fuente_imagen=args.imagen)
    r.iniciar()

    servidor = None
    if not args.sin_web:
        from src import servidor as srv_mod
        servidor = srv_mod.Servidor(r, cfg["red"])
        url = servidor.iniciar()
        r.log(f"[web] {url}   (o http://<ip-de-la-pi>:{cfg['red']['puerto_http']}/)")

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

    if not args.sin_panel:
        try:
            sys.path.insert(0, str(RAIZ / "tools"))
            import panel
            panel.Panel(r).ejecutar()      # bloquea hasta cerrar la ventana
            apagar()
            return 0
        except Exception as e:
            r.log(f"[main] sin panel grafico ({e}); sigo solo con la web")

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
