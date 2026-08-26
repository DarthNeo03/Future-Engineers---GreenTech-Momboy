#!/usr/bin/env python3
"""
calibrar_suelo.py — Mide la homografia imagen -> suelo. Se hace UNA vez.

    python3 tools/calibrar_suelo.py --tablero          # AUTOMATICO (recomendado)
    python3 tools/calibrar_suelo.py --tablero --cuadro 40 --distancias 250 550 900
    python3 tools/calibrar_suelo.py                    # a mano, cuatro clics
    python3 tools/calibrar_suelo.py --imagen foto.png

DOS MODOS
---------
`--tablero` detecta un tablero de ajedrez y saca ~54 esquinas por toma, sin
clics. Es el bueno, por dos razones que no son comodidad:

  * COBERTURA. Se toman varias capturas con el tablero a distintas
    distancias, y la homografia se ajusta a TODAS las esquinas a la vez. El
    metodo de cuatro marcas solo cubre el rectangulo que forman: mas alla
    extrapola, y justo ahi es donde peor se porta una homografia.

  * TE DICE SI TE EQUIVOCASTE. Con cuatro puntos la homografia pasa
    exactamente por ellos y el error de reproyeccion es CERO por
    construccion, midieras bien o mal. Con ciento y pico el ajuste es por
    minimos cuadrados y el error que sale es una medida de verdad.

El modo manual se queda por si no tienes tablero a mano.

POR QUE NO PUEDE SER AUTOMATICO DEL TODO
----------------------------------------
Una sola camara NO puede recuperar la escala de una foto: dos escenas, una
del doble de tamano al doble de distancia, dan exactamente la misma imagen.
Es geometria, no falta de ganas. Hace falta UNA medida real de referencia.

Con tablero esa medida son dos numeros que se toman una vez: el lado de la
casilla (que ya sabes, lo imprimiste tu) y la distancia del carro a la fila
mas cercana. Todo lo demas -encontrar las esquinas, ordenarlas, ajustar y
comprobar el error- va solo.

============================================================================
MODO MANUAL (sin --tablero)
============================================================================
QUE HAY QUE MONTAR
------------------
Cuatro marcas en el suelo formando un rectangulo, con el carro quieto y la
camara en su posicion definitiva. Por defecto:

                        Z = 700 mm    [4]-------------[3]
                                       |               |
                        Z = 300 mm    [1]-------------[2]
                                            ^ carro
                                    X=-150      X=+150

Las X se miden desde el EJE LONGITUDINAL del carro (no desde la camara si va
descentrada) y las Z desde el eje delantero. Cinta metrica y cinta adhesiva de
color: no hace falta nada mas. Cuanto mas separadas esten las marcas, mejor
sale la homografia; procura que la mas lejana quede cerca del limite util de
la imagen.

COMO SE USA
-----------
Se hace clic en las cuatro marcas EN ORDEN 1, 2, 3, 4 (cerca-izq, cerca-der,
lejos-der, lejos-izq). Teclas:

    u   deshacer el ultimo punto
    r   empezar de cero
    g   guardar en config/suelo.json  (solo con los 4 puntos puestos)
    v   ver/ocultar la rejilla metrica de comprobacion
    q   salir sin guardar

LA COMPROBACION IMPORTA
-----------------------
Con la rejilla activada se dibujan lineas cada 200 mm proyectadas de vuelta a
la imagen. Si esas lineas NO caen sobre el suelo real, la homografia esta mal
y todo lo que venga despues (seguir el muro interno, disparar el giro por la
esquina) heredara el error. Vale mas repetir la medida que ajustar ganancias
encima de una calibracion torcida.

DISTORSION DE LA LENTE
----------------------
Una homografia asume pinhole. Con gran angular hay barrilete y conviene pasar
antes `--intrinsecos` con un YAML/JSON de cv2.calibrateCamera. Sin eso la
calibracion sigue siendo mucho mejor que el modelo aproximado, pero se
degrada en los bordes de la imagen.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import geometria as geo, robot_config, camera  # noqa: E402

VENTANA = "Calibracion del suelo  (1,2,3,4 -> u=deshacer r=reset g=guardar v=rejilla q=salir)"
ETIQUETAS = ["1 cerca-izq", "2 cerca-der", "3 lejos-der", "4 lejos-izq"]


def main() -> int:
    ap = argparse.ArgumentParser(description="Homografia imagen -> suelo")
    ap.add_argument("--camara", type=int, default=None)
    ap.add_argument("--imagen", default=None)
    ap.add_argument("--x", type=float, default=150.0, help="semiancho del rectangulo (mm)")
    ap.add_argument("--z1", type=float, default=300.0, help="Z de las marcas cercanas (mm)")
    ap.add_argument("--z2", type=float, default=700.0, help="Z de las marcas lejanas (mm)")
    ap.add_argument("--intrinsecos", default=None,
                    help="JSON con K y dist de cv2.calibrateCamera")
    ap.add_argument("--salida", default=None)
    ap.add_argument("--tablero", nargs="?", const="9x6", default=None,
                    help="modo automatico. Esquinas INTERNAS, p.ej. 9x6")
    ap.add_argument("--cuadro", type=float, default=40.0,
                    help="lado de cada casilla en mm")
    ap.add_argument("--distancias", type=float, nargs="+",
                    default=[250.0, 550.0, 900.0],
                    help="distancias del carro a la fila mas cercana, en mm")
    args = ap.parse_args()

    cfg = robot_config.cargar()
    cam_cfg = cfg["camara"]

    # Coordenadas reales de las 4 marcas, en el mismo orden que se piden.
    mundo = np.array([
        [-args.x, args.z1],
        [+args.x, args.z1],
        [+args.x, args.z2],
        [-args.x, args.z2],
    ], dtype=np.float64)

    if args.tablero:
        return modo_tablero(args, cfg, cam_cfg)

    K = dist = None
    if args.intrinsecos:
        with open(args.intrinsecos, "r", encoding="utf-8") as f:
            d = json.load(f)
        K = np.array(d["K"], dtype=np.float64).reshape(3, 3)
        dist = np.array(d.get("dist", [0, 0, 0, 0, 0]), dtype=np.float64)
        print("[calibrar] usando intrinsecos: se corregira la distorsion")

    cap = None
    fija: Optional[np.ndarray] = None
    if args.imagen:
        fija = cv2.imread(args.imagen)
        if fija is None:
            print(f"No se pudo leer {args.imagen}")
            return 2
    else:
        idx = args.camara if args.camara is not None else int(cam_cfg["indice"])
        cap = camera.abrir(indice=idx, ancho=cam_cfg["ancho"], alto=cam_cfg["alto"],
                           fps=cam_cfg["fps"], fourcc=cam_cfg["fourcc"])
        if cap is None:
            print("No se pudo abrir la camara")
            return 2

    puntos: List[Tuple[int, int]] = []
    ver_rejilla = [False]

    def al_raton(evento, x, y, _flags, _param):
        if evento == cv2.EVENT_LBUTTONDOWN and len(puntos) < 4:
            puntos.append((x, y))

    cv2.namedWindow(VENTANA, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(VENTANA, al_raton)
    print(__doc__.split("COMO SE USA")[1].split("LA COMPROBACION")[0])

    H: Optional[np.ndarray] = None
    while True:
        if fija is not None:
            frame = fija.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                continue
            if cam_cfg.get("voltear"):
                frame = cv2.flip(frame, -1)

        for i, (px, py) in enumerate(puntos):
            cv2.drawMarker(frame, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(frame, ETIQUETAS[i], (px + 8, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        if len(puntos) == 4:
            cv2.polylines(frame, [np.array(puntos, np.int32)], True, (0, 200, 0), 1)

        if len(puntos) == 4:
            img = np.array(puntos, dtype=np.float64).reshape(-1, 1, 2)
            if K is not None:
                img = cv2.undistortPoints(img, K, dist, P=K)
            H = cv2.getPerspectiveTransform(img.reshape(4, 2).astype(np.float32),
                                            mundo.astype(np.float32))
            if ver_rejilla[0]:
                _dibujar_rejilla(frame, H, K, dist)
        else:
            H = None

        falta = ETIQUETAS[len(puntos)] if len(puntos) < 4 else "listo: pulsa g para guardar"
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(frame, f"clic en: {falta}", (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(VENTANA, frame)

        t = cv2.waitKey(20) & 0xFF
        if t in (ord("q"), 27):
            break
        elif t == ord("u") and puntos:
            puntos.pop()
        elif t == ord("r"):
            puntos.clear()
        elif t == ord("v"):
            ver_rejilla[0] = not ver_rejilla[0]
        elif t == ord("g") and H is not None:
            s = geo.Suelo(cam_cfg)
            s.H = H
            s.K, s.dist = K, dist
            ruta = s.guardar(Path(args.salida) if args.salida else None,
                             notas=f"rect x=+-{args.x} z={args.z1}..{args.z2} mm")
            print(f"\n[calibrar] guardado en {ruta}")
            _informe(H, mundo, puntos, K, dist)
            break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    return 0



def modo_tablero(args, cfg, cam_cfg) -> int:
    """Calibracion automatica: tablero de ajedrez a varias distancias.

    QUE HAY QUE MEDIR, Y POR QUE NO SE PUEDE EVITAR
    -----------------------------------------------
    Una sola camara NO puede recuperar la escala de una foto. Dos escenas, una
    del doble de tamano al doble de distancia, dan exactamente la misma imagen.
    Es geometria, no falta de ganas: hace falta UNA medida real de referencia.

    Aqui esa medida son dos numeros que solo se toman una vez: el lado de la
    casilla del tablero (que ya sabes, lo imprimiste tu) y la distancia del
    carro a la fila de esquinas mas cercana. Todo lo demas -encontrar las
    esquinas, ordenarlas, ajustar y comprobar- va solo.
    """
    import cv2

    try:
        nx, ny = (int(v) for v in str(args.tablero).lower().split("x"))
    except Exception:
        print(f"--tablero mal escrito: '{args.tablero}'. Se espera algo como 9x6")
        return 2

    cap = None
    fija = None
    if args.imagen:
        fija = cv2.imread(args.imagen)
        if fija is None:
            print(f"No se pudo leer {args.imagen}")
            return 2
    else:
        idx = args.camara if args.camara is not None else int(cam_cfg["indice"])
        cap = camera.abrir(indice=idx, ancho=cam_cfg["ancho"], alto=cam_cfg["alto"],
                           fps=cam_cfg["fps"], fourcc=cam_cfg["fourcc"])
        if cap is None:
            print("No se pudo abrir la camara")
            return 2

    print(f"""
Tablero de {nx}x{ny} esquinas internas, casillas de {args.cuadro:.0f} mm.

Colocalo PLANO en el suelo, con las filas ATRAVESADAS respecto a la marcha
y centrado en el eje del carro. Para cada distancia de la lista, mide del
carro a la fila de esquinas MAS CERCANA.

  espacio  capturar (solo si el tablero esta detectado, en verde)
  s        saltarse esta distancia
  v        ver la rejilla de comprobacion (con 2 tomas o mas)
  g        ajustar y guardar
  q        salir sin guardar
""")

    vistas = []
    pendientes = list(args.distancias)
    H = None
    ver_rejilla = False
    ventana = "Calibracion por tablero"
    cv2.namedWindow(ventana, cv2.WINDOW_NORMAL)

    while True:
        if fija is not None:
            frame = fija.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                continue
            if cam_cfg.get("voltear"):
                frame = cv2.flip(frame, -1)

        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        esq = geo.detectar_tablero(gris, nx, ny)

        if esq is not None:
            cv2.drawChessboardCorners(frame, (nx, ny),
                                      esq.reshape(-1, 1, 2), True)
            # La esquina de referencia (fila 0, columna 0) marcada aparte: si
            # sale en el sitio equivocado, el tablero esta mal puesto.
            cv2.circle(frame, tuple(esq[0, 0].astype(int)), 9, (0, 255, 255), 2)

        z0 = pendientes[0] if pendientes else None
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 46), (0, 0, 0), -1)
        if z0 is not None:
            txt = f"tablero a {z0:.0f} mm  |  {'DETECTADO' if esq is not None else 'no se ve'}"
        else:
            txt = "todas las distancias hechas: pulsa g para guardar"
        cv2.putText(frame, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if esq is not None else (0, 165, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"tomas: {len(vistas)}   faltan: "
                           f"{', '.join(f'{d:.0f}' for d in pendientes) or '-'}",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1,
                    cv2.LINE_AA)

        if ver_rejilla and H is not None:
            _dibujar_rejilla(frame, H, None, None)

        cv2.imshow(ventana, frame)
        t = cv2.waitKey(20) & 0xFF

        if t in (ord("q"), 27):
            break
        elif t == ord("s") and pendientes:
            pendientes.pop(0)
        elif t == ord("v"):
            ver_rejilla = not ver_rejilla
        elif t == ord(" ") and esq is not None and z0 is not None:
            mundo = geo.mundo_tablero(nx, ny, args.cuadro, z0)
            vistas.append((esq, mundo))
            pendientes.pop(0)
            print(f"  capturada a {z0:.0f} mm  ({esq.size // 2} esquinas, "
                  f"{len(vistas)} tomas)")
            if len(vistas) >= 2:
                try:
                    H, err, peor = geo.homografia_desde_tableros(vistas)
                    print(f"    ajuste provisional: error medio {err:.1f} mm, "
                          f"peor {peor:.1f} mm")
                except Exception as e:
                    print(f"    aun no se puede ajustar: {e}")
        elif t == ord("g"):
            if len(vistas) < 2:
                print("  hacen falta al menos 2 tomas a distintas distancias: "
                      "con una sola, la homografia extrapola y se va lejos")
                continue
            try:
                H, err, peor = geo.homografia_desde_tableros(vistas)
            except Exception as e:
                print(f"  no se pudo ajustar: {e}")
                continue
            n_esq = sum(v[0].size // 2 for v in vistas)
            print(f"\n  {len(vistas)} tomas, {n_esq} esquinas")
            print(f"  error de reproyeccion: medio {err:.1f} mm, peor {peor:.1f} mm")
            if err > 25.0:
                print("  AVISO: mas de 25 mm de media. Revisa que mediste bien las\n"
                      "         distancias y que el tablero estaba plano y centrado.")
            else:
                print("  Buen ajuste.")
            su = geo.Suelo(cam_cfg)
            su.H = H
            ruta = su.guardar(Path(args.salida) if args.salida else None,
                              notas=f"tablero {nx}x{ny} de {args.cuadro:.0f} mm, "
                                    f"{len(vistas)} tomas, error medio {err:.1f} mm")
            print(f"  guardado en {ruta}")
            print("  Pulsa v para superponer la rejilla y comprobarlo sobre el suelo.")
            ver_rejilla = True

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    return 0


def _dibujar_rejilla(frame, H, K, dist) -> None:
    """Proyecta una rejilla metrica DE VUELTA a la imagen. Si no cae sobre el
    suelo real, la calibracion esta mal."""
    Hinv = np.linalg.inv(H)
    zs = np.arange(200, 1401, 200, dtype=np.float64)
    xs = np.arange(-600, 601, 200, dtype=np.float64)

    def a_pixel(pts_mundo):
        p = np.asarray(pts_mundo, np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, Hinv).reshape(-1, 2)

    for z in zs:
        linea = a_pixel([[x, z] for x in np.arange(-600, 601, 40)])
        pts = linea.astype(np.int32)
        cv2.polylines(frame, [pts], False, (255, 160, 0), 1)
        cv2.putText(frame, f"{int(z)}", tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 160, 0), 1, cv2.LINE_AA)
    for x in xs:
        linea = a_pixel([[x, z] for z in np.arange(200, 1401, 40)])
        cv2.polylines(frame, [linea.astype(np.int32)], False, (120, 120, 255), 1)


def _informe(H, mundo, puntos, K, dist) -> None:
    """Error de reproyeccion: cuanto se desvia cada marca respecto a su medida."""
    img = np.array(puntos, dtype=np.float64).reshape(-1, 1, 2)
    if K is not None:
        img = cv2.undistortPoints(img, K, dist, P=K)
    est = cv2.perspectiveTransform(img, H).reshape(-1, 2)
    print("\n  marca        medido (X,Z)      calculado (X,Z)     error")
    for i, (m, e) in enumerate(zip(mundo, est)):
        err = float(np.hypot(*(e - m)))
        print(f"  {ETIQUETAS[i]:<13} ({m[0]:+7.1f},{m[1]:7.1f})  "
              f"({e[0]:+7.1f},{e[1]:7.1f})  {err:5.1f} mm")
    print("\n  Con 4 puntos el error de reproyeccion es cero por construccion:\n"
          "  esta tabla solo confirma que no te equivocaste tecleando las medidas.\n"
          "  La comprobacion de verdad es la rejilla (tecla v) sobre el suelo real.")


if __name__ == "__main__":
    raise SystemExit(main())
