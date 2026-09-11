"""
panel.py — Ventana de depuracion: ver lo que el carro cree que esta viendo.

ESTO NO CORRE EN COMPETENCIA. Dibujar cuesta unos 8 ms por frame en la Pi 5,
que es un cuarto del presupuesto de un ciclo. Se usa en el taller, con
--ver, y se apaga para rodar.

QUE SE DIBUJA Y PARA QUE SIRVE CADA COSA

  linea del horizonte       si un pilar aparece por encima, la calibracion de
                            inclinacion o altura esta mal. Es el primer sitio
                            donde mirar cuando el carro "ve cosas raras".
  corredor de las ruedas    por donde va a pasar el carro si sigue recto. Si
                            un pilar cae dentro, hay que esquivarlo; si no,
                            no. Verlo dibujado ahorra discusiones.
  cajas de los pilares      con la distancia combinada y la de ALTURA. Si las
                            dos se separan mucho, la inclinacion configurada no
                            es la real: el pilar se sigue viendo, pero conviene
                            volver a medir el mastil.
  contador de descartes     cuantos contornos se tiraron y por que filtro. Es
                            LA linea que hay que mirar cuando el carro parece
                            ignorar los colores: dice si la mascara no ve nada
                            o si un filtro se los esta comiendo.
  perfil de espacio libre   la curva de distancia por sector. Es literalmente
                            lo que come el seguidor de carril.
  punto de paso             el objetivo lateral del esquive. Si esta del lado
                            equivocado, el fallo es de regla, no de control.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

import cv2
import numpy as np

from .geometria import DIST_MAX_MM
from .vision import Escena

COLOR_TEXTO = (240, 240, 240)
COLOR_HORIZONTE = (90, 90, 220)
COLOR_CORREDOR = (70, 200, 255)
COLOR_PERFIL = (120, 255, 120)
COLOR_OBJETIVO = (255, 120, 255)
CAJAS = {"rojo": (60, 60, 255), "verde": (60, 220, 60),
         "magenta": (220, 60, 220)}


def _sentido(u: Dict[str, Any]) -> str:
    """El sentido de la ronda, con de donde salio: importa tanto como el valor.

    "horario (provisional)" quiere decir que se dedujo de UNA linea y todavia
    puede estar al reves si el TCS se salto la de entrada; "horario (firme)",
    que lo confirmo el par ordenado de una esquina entera.
    """
    s = u.get("sentido", 0)
    if not s:
        return "sin saber"
    nombre = "horario" if s > 0 else "antihorario"
    if u.get("sentido_firme"):
        return f"{nombre} (firme)"
    return f"{nombre} (provisional)"


def _tcs(u: Dict[str, Any]) -> str:
    """Lo que el clasificador de lineas esta viendo, en una linea.

    "El carro no ve las lineas" es una queja, no un diagnostico. Con estos
    dos numeros pasa a ser una resta:

      claro por encima del 78 % del blanco  -> falla la PUERTA DE LUZ: el
          sensor va demasiado alto, o hay reflejo, o el blanco aprendido se
          quedo bajo.
      claro por debajo pero |sep| < 60      -> falla el DISCRIMINANTE: el
          color no llega, casi siempre por altura o por suciedad en la lente.

    `sep` positivo es naranja y negativo azul, asi que tambien dice si el
    sensor esta confundiendo los dos colores.
    """
    if not u.get("tcs_ok"):
        return "TCS: ausente (las esquinas iran solo por camara)"
    if not u.get("lineas_diag"):
        return (f"TCS: claro {u.get('claro')} "
                f"(firmware viejo: sin blanco ni separacion)")
    pct = u.get("pct_claro")
    sep = u.get("separacion", 0)
    quien = "naranja" if sep > 0 else ("azul" if sep < 0 else "-")
    return (f"TCS: claro {u.get('claro')}/{u.get('blanco')} = {pct}% "
            f"(entra <78)   sep {sep:+} -> {quien} (|sep|>60)")


def dibujar(frame: np.ndarray, piloto: Any, esc: Escena) -> np.ndarray:
    img = frame.copy()
    alto, ancho = img.shape[:2]
    geo = piloto.geo
    u = piloto.ultimo

    # --- horizonte ------------------------------------------------------
    hy = geo.fila_horizonte()
    if 0 <= hy < alto:
        cv2.line(img, (0, hy), (ancho, hy), COLOR_HORIZONTE, 1)
        cv2.putText(img, "horizonte", (6, max(12, hy - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_HORIZONTE, 1)

    # --- corredor de las ruedas -----------------------------------------
    try:
        poli = geo.poligono_corredor(150.0, 1800.0, 10)
        cv2.polylines(img, [poli], True, COLOR_CORREDOR, 1)
    except Exception:
        pass

    # --- perfil de espacio libre ----------------------------------------
    if esc.perfil_mm.size:
        n = esc.perfil_mm.size
        for i, d in enumerate(esc.perfil_mm):
            x0 = int(i * ancho / n)
            x1 = int((i + 1) * ancho / n)
            # Mas cerca = barra mas alta. Saturado a 2 m, que es donde deja de
            # importar para decidir.
            h = int(np.clip(1.0 - min(d, 2000.0) / 2000.0, 0, 1) * 60)
            cv2.rectangle(img, (x0, alto - h), (x1 - 1, alto), COLOR_PERFIL, -1)

    # --- detecciones -----------------------------------------------------
    for d in list(esc.pilares) + list(esc.magenta):
        col = CAJAS.get(d.color, (200, 200, 200))
        cv2.rectangle(img, (d.x, d.y), (d.x + d.w, d.y + d.h), col, 2)
        cv2.putText(img, f"{d.dist_mm:.0f}/{d.dist_alto_mm:.0f}mm",
                    (d.x, max(12, d.y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    col, 1)
        cv2.putText(img, f"lat {d.lat_mm:+.0f}", (d.x, d.y + d.h + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)

    # --- punto de paso del esquive ---------------------------------------
    m = piloto.esquivador
    if u.get("esquive_peso", 0) > 0.05:
        try:
            obj = piloto.ultimo.get("objetivo_mm")
            if obj is not None:
                px, py = geo.suelo_a_pixel(float(obj), 600.0)
                cv2.drawMarker(img, (px, py), COLOR_OBJETIVO,
                               cv2.MARKER_TILTED_CROSS, 18, 2)
        except Exception:
            pass

    # --- texto de estado --------------------------------------------------
    giro = {1: "derecha", -1: "izquierda"}.get(u.get("sentido_curva", 0),
                                               "sin aprender")
    lineas = [
        f"{u.get('estado','?'):<11} {u.get('nota','')}",
        f"vel {u.get('vel',0):>5}%   dir {u.get('dir',0):>6}%",
        f"carril {u.get('carril_dir',0):>6}  esquive {u.get('esquive_dir',0):>6}"
        f" x{u.get('esquive_peso',0)}  ({u.get('esquive_fase','')})",
        f"frente {u.get('frente_mm',0)}mm   lateral izq {u.get('izq_mm')}"
        f"  der {u.get('der_mm')}"
        + (f"   margen pedido {u['margen_pilar']}mm"
           if u.get("margen_pilar") is not None else ""),
        f"hueco a {u.get('rumbo_hueco',0)} deg   curvas hacia {giro}"
        f"   sesgo {u.get('sesgo',0):+}   [{u.get('carril_motivo','')}]",
        f"yaw {u.get('yaw',0):>6}  vuelta {u.get('vueltas',0)}"
        f"  seccion {u.get('secciones',0)}  sentido {_sentido(u)}",
    ]
    lineas.append(_tcs(u))
    if u.get("esquina_abierta"):
        lineas.append(f"LINEA {u.get('linea','').upper()}: empieza la curva")
    if u.get("tras_pilar"):
        lineas.append("TRAS PILAR: sin sesgo de curva y sin aprender sentido")
    if u.get("muro_encima"):
        lineas.append("MURO ENCIMA: manda el centrado")
    # Las dos fuentes del sentido enfrentadas. Si esto sale, una esta mal, y
    # lo mas probable es color_entrada_horario al reves: el aprendizaje del
    # carril no depende de ningun parametro.
    if u.get("sentido_discrepa"):
        lineas.append("OJO: las lineas y las curvas NO dicen el mismo sentido")
    lineas.append(f"pilares vistos: {u.get('n_pilares', 0)}"
                  f"   descartes: {u.get('descartes') or 'ninguno'}")
    if u.get("magenta") is not None:
        lineas.append(f"MAGENTA cerca: empujon {u['magenta']}%")
    y = 18
    for t in lineas:
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.46,
                    COLOR_TEXTO, 1, cv2.LINE_AA)
        y += 18
    return img


def bucle_con_ventana(piloto: Any, duracion_max_s: float = 0.0) -> None:
    """Como Piloto.correr pero dibujando. Teclas: q sale, espacio pausa."""
    from .fsm import Estado

    t0 = time.time()
    pausado = False
    try:
        while True:
            if not pausado:
                piloto.ciclo()
            frame, _, _ = piloto.camara.leer()
            esc = getattr(piloto, "_esc_prev", Escena())
            if frame is not None:
                cv2.imshow("piloto obstaculos", dibujar(frame, piloto, esc))
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord("q"):
                break
            if tecla == ord(" "):
                pausado = not pausado
                if pausado:
                    piloto.enlace.parar()
            if piloto.fsm.estado == Estado.FIN:
                print(f"[panel] ronda terminada: {piloto.contador.resumen()}")
                break
            if duracion_max_s and (time.time() - t0) > duracion_max_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        piloto.cerrar()
