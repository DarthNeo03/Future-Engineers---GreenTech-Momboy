#!/usr/bin/env python3
"""
calibrar_piso.py — Mide como ve TU sensor el blanco, el naranja y el azul.

    python3 tools/calibrar_piso.py              # interactivo, guarda al final
    python3 tools/calibrar_piso.py --ver        # solo mirar, sin guardar
    python3 tools/calibrar_piso.py --integracion 2.4

POR QUE HAY QUE CALIBRARLO
--------------------------
Los perfiles que trae el codigo son un punto de partida, no una medida. El
color que devuelve un TCS34725 depende de su LED concreto, de la altura a la
que lo montes, del angulo y del tapete que tengas delante. Dos sensores del
mismo lote sobre el mismo naranja dan numeros distintos.

Se calibra en cromaticidad (cada canal dividido por el CLEAR), asi que lo que
se mide aqui aguanta que luego cambie el brillo. Lo que NO aguanta es que
cambies la altura del sensor: si lo remontas, vuelve a calibrar.

COMO
----
Con el carro montado y el sensor a su altura definitiva, se coloca encima de
cada color y se pulsa la tecla. Se toman muchas muestras y se queda la
mediana, que ignora el ruido y algun destello.

    b  blanco (el piso de la pista, no un folio)
    n  naranja
    a  azul
    m  mostrar la lectura actual
    g  guardar en config/robot.json
    q  salir sin guardar

CONSEJO QUE AHORRA UNA TARDE
----------------------------
Calibra sobre el TAPETE DE COMPETENCIA, no sobre una impresion casera. El
naranja CMYK(0,60,100,0) de la lona oficial y el de una impresora de casa no
se parecen tanto como crees, y el sensor si nota la diferencia.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_piso as cp, robot_config  # noqa: E402

TECLAS = {"b": cp.BLANCO, "n": "naranja", "a": "azul"}


def _tecla() -> str:
    """Una tecla sin Enter. En Linux se usa termios; si no, se cae a input()."""
    try:
        import termios
        import tty
        fd = sys.stdin.fileno()
        viejo = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, viejo)
    except Exception:
        return (input("tecla + Enter: ").strip().lower() or " ")[0]


def medir(sensor: cp.SensorPiso, n: int = 40) -> Tuple[float, float, float, int]:
    """n muestras -> mediana de la cromaticidad y del clear."""
    rs: List[float] = []
    gs: List[float] = []
    bs: List[float] = []
    cs: List[int] = []
    for _ in range(n):
        c, r, g, b = sensor.crudo
        s = float(r + g + b)
        if s > 0 and c > 0:
            rs.append(r / s)
            gs.append(g / s)
            bs.append(b / s)
            cs.append(c)
        time.sleep(0.03)
    if not rs:
        raise RuntimeError("no llegaron muestras validas del sensor")
    return (statistics.median(rs), statistics.median(gs),
            statistics.median(bs), int(statistics.median(cs)))


def main() -> int:
    ap = argparse.ArgumentParser(description="Calibra el TCS34725 del suelo")
    ap.add_argument("--integracion", type=float, default=None,
                    help="ms de integracion (24 por defecto; 2.4 para ir rapido)")
    ap.add_argument("--ganancia", type=int, default=None, choices=(1, 4, 16, 60))
    ap.add_argument("--muestras", type=int, default=40)
    ap.add_argument("--ver", action="store_true", help="solo mirar, sin guardar")
    args = ap.parse_args()

    cfg = robot_config.cargar()
    piso = dict(cfg.get("piso", {}))
    if args.integracion:
        piso["integracion_ms"] = args.integracion
    if args.ganancia:
        piso["ganancia"] = args.ganancia
    piso["activo"] = True

    sensor = cp.SensorPiso(piso)
    if not sensor.iniciar():
        print(f"No hay sensor: {sensor.motivo}")
        print("Comprueba con:  i2cdetect -y 1   (debe salir 29)")
        return 2
    print(f"[piso] {sensor.motivo}")
    time.sleep(0.3)

    if args.ver:
        print("Mostrando lecturas. Ctrl+C para salir.")
        try:
            while True:
                c, r, g, b = sensor.crudo
                rn, gn, bn = sensor.cromatico
                print(f"\r{sensor.color:<12s} c={c:6d}  "
                      f"r={rn:.3f} g={gn:.3f} b={bn:.3f}  "
                      f"{sensor.hz_real:4.1f} Hz   ", end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()
        sensor.parar()
        return 0

    medidos: Dict[str, Dict[str, float]] = {}
    print(__doc__.split("COMO")[1].split("CONSEJO")[0])

    try:
        while True:
            falta = [k for k, v in TECLAS.items() if v not in medidos]
            pend = ", ".join(TECLAS[k] for k in falta) if falta else "nada, ya puedes guardar"
            print(f"\nfalta: {pend}   |   lectura actual: {sensor.color}")
            t = _tecla()

            if t == "q":
                print("sin guardar")
                break
            if t == "m":
                c, r, g, b = sensor.crudo
                rn, gn, bn = sensor.cromatico
                print(f"  {sensor.color}  c={c}  r={rn:.3f} g={gn:.3f} b={bn:.3f}"
                      f"  {sensor.hz_real:.1f} Hz")
                continue
            if t in TECLAS:
                nombre = TECLAS[t]
                print(f"  midiendo {nombre}... no muevas el carro")
                try:
                    rn, gn, bn, c = medir(sensor, args.muestras)
                except RuntimeError as e:
                    print(f"  {e}")
                    continue
                medidos[nombre] = {"r": round(rn, 4), "g": round(gn, 4),
                                   "b": round(bn, 4)}
                print(f"  {nombre:8s} r={rn:.3f} g={gn:.3f} b={bn:.3f}  (clear {c})")
                continue
            if t == "g":
                if len(medidos) < len(TECLAS):
                    print("  faltan colores por medir")
                    continue
                perfiles = _con_tolerancias(medidos)
                _informe(perfiles)
                piso_guardar = dict(cfg.get("piso", {}))
                piso_guardar.update({
                    "activo": True,
                    "integracion_ms": piso.get("integracion_ms", 24.0),
                    "ganancia": piso.get("ganancia", 4),
                    "perfiles": perfiles,
                })
                cfg["piso"] = piso_guardar
                ruta = robot_config.guardar(cfg)
                print(f"\nguardado en {ruta}")
                break
    finally:
        sensor.parar()
    return 0


def _con_tolerancias(medidos: Dict[str, Dict[str, float]]) -> List[Dict]:
    """Reparte las tolerancias segun lo separados que quedaron los colores.

    Se toma la mitad de la distancia al vecino mas cercano, acotada: asi dos
    colores parecidos no se pisan y uno aislado no se queda con una ventana
    absurdamente estrecha.
    """
    nombres = list(medidos)
    perfiles = []
    for n in nombres:
        a = medidos[n]
        d_min = min(
            (((a["r"] - medidos[m]["r"]) ** 2 + (a["g"] - medidos[m]["g"]) ** 2
              + (a["b"] - medidos[m]["b"]) ** 2) ** 0.5)
            for m in nombres if m != n) if len(nombres) > 1 else 0.20
        tol = max(0.030, min(0.110, d_min * 0.45))
        perfiles.append({"nombre": n, **a, "tol": round(tol, 3)})
    return perfiles


def _informe(perfiles: List[Dict]) -> None:
    print("\n  color      r      g      b     tol")
    for p in perfiles:
        print(f"  {p['nombre']:<9s}{p['r']:.3f}  {p['g']:.3f}  {p['b']:.3f}  {p['tol']:.3f}")
    peor = 1e9
    for i, a in enumerate(perfiles):
        for b in perfiles[i + 1:]:
            d = ((a["r"] - b["r"]) ** 2 + (a["g"] - b["g"]) ** 2
                 + (a["b"] - b["b"]) ** 2) ** 0.5
            peor = min(peor, d)
            if d < a["tol"] + b["tol"]:
                print(f"  AVISO: {a['nombre']} y {b['nombre']} se solapan "
                      f"(distancia {d:.3f}). Sube el sensor o baja la ganancia.")
    print(f"  separacion minima entre colores: {peor:.3f}"
          f"   {'(comoda)' if peor > 0.15 else '(justa: revisa la altura)'}")


if __name__ == "__main__":
    raise SystemExit(main())
