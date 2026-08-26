#!/usr/bin/env python3
"""
selftest_robot.py — Pruebas del sistema completo SIN carro.

    python3 tools/selftest_robot.py

Cubre:
  1. Protocolo Python <-> C++ (los mismos bytes en las dos implementaciones)
  2. El lector de tramas frente a ruido, truncados y arranques a media trama
  3. Geometria: la camara como LIDAR metrico, contiguidad, desconocido!=libre
  3b. El salto de rango que distingue el muro interno del externo
  3c. Navegacion: los cuatro sintomas de pista, en los dos sentidos
  3d. Senales rojas y verdes del reto de obstaculos
  10. VUELTAS COMPLETAS en lazo cerrado sobre la pista del reglamento
  4. Enlace serie de verdad, contra un ESP32 falso al otro lado de un pty
  5. Servidor web: pagina, JSON de estado, ordenes y stream MJPEG
  6. Seguridad del Robot: sin imagen se desarma, en manual no te deja
     empotrarte, y el mando manual caduca si el joystick deja de refrescar

Lo unico que no se puede probar aqui es el ESP32 real; para eso esta
tools/test_firmware.cpp, que compila su logica con g++.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

# /tmp no existe en Windows y el README pide correr esto en los dos sistemas.
TMP = Path(tempfile.gettempdir())

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc, navegacion as nav, protocolo as P  # noqa: E402
from src import geometria as geo, obstaculos as obs                     # noqa: E402
from src import robot_config, vision                                    # noqa: E402

_fallos: List[str] = []
_ok = 0


def check(cond, nombre, detalle=""):
    global _ok
    if cond:
        _ok += 1
        print(f"  ok   {nombre}")
    else:
        _fallos.append(nombre)
        print(f"  FALLA {nombre}  {detalle}")


# ===========================================================================
# 1. Protocolo cruzado con el firmware
# ===========================================================================
def test_protocolo_cruzado():
    print("\n[1] Protocolo: Python contra la implementacion del ESP32")
    binario = TMP / "tfw_selftest"
    fuente = RAIZ / "tools" / "test_firmware.cpp"
    inc = RAIZ / "firmware" / "esp32_carro"
    try:
        subprocess.run(["g++", "-std=c++17", "-O1", "-I", str(inc), str(fuente),
                        "-o", str(binario)], check=True, capture_output=True)
    except Exception as e:
        print(f"       (sin g++ o sin compilar: {e}); me salto el cruce")
        return
    salida = subprocess.run([str(binario), "vectores"], capture_output=True,
                            text=True, check=True).stdout

    n_mando = n_tele = 0
    for linea in salida.strip().splitlines():
        campos = linea.split()
        if campos[0] == "MANDO":
            seq, flags, vel, dirn, vmax = (int(x) for x in campos[1:6])
            esperado = campos[6].upper()
            m = P.Mando(seq=seq, vel=vel, direccion=dirn, vmax=vmax,
                        armado=bool(flags & P.F_ARMADO),
                        parada=bool(flags & P.F_PARADA),
                        centrar=bool(flags & P.F_CENTRAR),
                        limpiar=bool(flags & P.F_LIMPIAR))
            obtenido = m.a_bytes().hex().upper()
            check(obtenido == esperado, f"mando seq={seq} vel={vel} dir={dirn}",
                  f"py={obtenido} cpp={esperado}")
            n_mando += 1
        elif campos[0] == "TELE":
            vals = [int(x) for x in campos[1:8]]
            esperado = campos[8].upper()
            t = P.Telemetria(*vals)
            check(t.a_bytes().hex().upper() == esperado,
                  f"telemetria {vals[:2]}", f"py={t.a_bytes().hex().upper()} cpp={esperado}")
            n_tele += 1
        elif campos[0] == "SERVO":
            grados = [int(x) for x in campos[1:]]
            check(all(50 <= g <= 145 for g in grados),
                  "el firmware nunca sale del rango del servo", str(grados))
            check(grados[grados.index(min(grados))] == 65 and max(grados) == 135,
                  "los topes del servo son 65 y 135", str(grados))
    check(n_mando >= 5 and n_tele >= 3, "se cruzaron todos los vectores",
          f"{n_mando} mandos, {n_tele} telemetrias")

    # Y al reves: lo que empaqueta Python lo entiende el lector de Python
    for vel, dirn in ((0, 0), (100, -100), (-100, 100), (37, -12)):
        m = P.Mando(seq=5, vel=vel, direccion=dirn, vmax=200, armado=True)
        L = P.Lector()
        tramas = L.alimentar(m.a_bytes())
        check(len(tramas) == 1 and P.Mando.desde_payload(tramas[0][1]).vel == vel,
              f"ida y vuelta vel={vel} dir={dirn}")


def test_lector_robusto():
    print("\n[2] Lector de tramas frente a ruido")
    trama = P.Mando(seq=9, vel=50, direccion=-30, vmax=120, armado=True).a_bytes()

    L = P.Lector()
    check(len(L.alimentar(b"\x00\xff\xa5\xa5\x13\x5a\x7f")) == 0, "la basura no genera tramas")
    check(len(L.alimentar(trama)) == 1, "y despues engancha la trama buena")

    L2 = P.Lector()
    res = L2.alimentar(trama[:6] + b"\xee" + trama)     # truncada + buena
    check(len(res) == 1, "una trama truncada no se traga la siguiente", len(res))

    L3 = P.Lector()
    total = 0
    for k in range(200):
        total += len(L3.alimentar(P.Mando(seq=k, vel=1, direccion=0).a_bytes()))
    check(total == 200, "200 tramas seguidas = 200 lecturas", total)
    check(L3.crc_malos == 0 and L3.descartados == 0, "sin descartes en flujo limpio")

    L4 = P.Lector()
    res = L4.alimentar(trama[5:] + trama + trama)       # arranque a media trama
    check(len(res) == 2, "arranque a mitad de trama: engancha desde la siguiente", len(res))

    L5 = P.Lector()
    mala = bytearray(trama)
    mala[6] ^= 0xFF
    res = L5.alimentar(bytes(mala) + trama)
    check(len(res) == 1 and L5.crc_malos >= 1, "un bit cambiado invalida la trama")

    # byte a byte, como sale de verdad del UART
    L6 = P.Lector()
    cuenta = 0
    for b in trama + trama:
        cuenta += len(L6.alimentar(bytes([b])))
    check(cuenta == 2, "funciona alimentando byte a byte", cuenta)

    # el buffer no crece sin control si nunca llega un sync
    L7 = P.Lector()
    for _ in range(50):
        L7.alimentar(b"\x11" * 100)
    check(len(L7._buf) < 8, "no acumula memoria con basura infinita", len(L7._buf))


# ===========================================================================
# 3. Geometria: la camara como LIDAR metrico
# ===========================================================================
def pista(alturas, ancho=640, alto=480, ruido=5):
    """Vista con muro negro arriba cuyo borde inferior sigue 'alturas'."""
    img = np.full((alto, ancho, 3), 235, np.uint8)
    xs = np.array([p[0] for p in alturas], dtype=np.float32)
    ys = np.array([p[1] for p in alturas], dtype=np.float32)
    col = np.arange(ancho, dtype=np.float32)
    borde = np.interp(col, xs, ys).astype(np.int32)
    for x in range(ancho):
        img[0:max(0, borde[x]), x] = (18, 18, 20)
    r = np.random.default_rng(4).normal(0, ruido, img.shape)
    return np.clip(img.astype(np.int16) + r, 0, 255).astype(np.uint8)


def _mascara_muro(img, colores):
    v = vision.Vision(colores)
    _d, m = v.procesar(img, solo=["negro"])
    return m["negro"]


def _cortar_rayo(ox, oz, dx, dz, seg):
    """Interseccion rayo-segmento. Devuelve la distancia o None."""
    (x1, z1), (x2, z2) = seg
    ex, ez = x2 - x1, z2 - z1
    den = dx * ez - dz * ex
    if abs(den) < 1e-9:
        return None
    t = ((x1 - ox) * ez - (z1 - oz) * ex) / den
    u = ((x1 - ox) * dz - (z1 - oz) * dx) / den
    if t > 1e-6 and -1e-6 <= u <= 1 + 1e-6:
        return t
    return None


def escaneo_mundo(segmentos, ancho=640, hfov=100.0, ruido_mm=0.0, semilla=1):
    """Traza un mundo 2D de paredes y devuelve un Escaneo metrico.

    Es mucho mas directo que fabricar imagenes: prueba la NAVEGACION sin
    arrastrar el detector de color ni la proyeccion. La cadena completa
    (imagen -> mascara -> escaneo) se prueba aparte, en test_pipeline_imagen.
    """
    rng = np.random.default_rng(semilla)
    med = math.radians(hfov) / 2.0
    ang = np.linspace(-med, med, ancho)
    X = np.full(ancho, np.nan)
    Z = np.full(ancho, np.nan)
    val = np.zeros(ancho, bool)
    for i, a in enumerate(ang):
        dx, dz = math.sin(a), math.cos(a)
        mejor = None
        for seg in segmentos:
            t = _cortar_rayo(0.0, 0.0, dx, dz, seg)
            if t is not None and (mejor is None or t < mejor):
                mejor = t
        if mejor is None or mejor > geo.Z_MAX_MM:
            continue
        r = mejor + (rng.normal(0, ruido_mm) if ruido_mm else 0.0)
        X[i], Z[i] = r * dx, r * dz
        val[i] = True
    return geo.Escaneo(x=X, z=Z, valido=val,
                       y_contacto=np.zeros(ancho, np.int32), ancho=ancho, alto=480)


def corredor(ancho_mm=1000.0, lado_interno=-1, z_esquina=None, largo=2500.0):
    """Un tramo recto de corredor.

    El carro va a 250 mm del muro interno (que es el objetivo por defecto).
    Si z_esquina no es None, el muro interno se ACABA ahi -esquina convexa- y
    el muro externo cierra por delante en concavo: una esquina de pista.
    """
    x_int = lado_interno * 250.0
    x_ext = -lado_interno * (ancho_mm - 250.0)
    segs = [((x_ext, -200.0), (x_ext, largo))]
    if z_esquina is None:
        segs.append(((x_int, -200.0), (x_int, largo)))
    else:
        # El muro interno se ACABA en la esquina convexa.
        segs.append(((x_int, -200.0), (x_int, z_esquina)))
        # El muro externo cierra por delante en concavo, y su pared frontal
        # se extiende MUCHO mas alla del lado interno: es la pared exterior
        # del siguiente pasillo. Eso es lo que produce el salto de rango al
        # asomarse por encima de la esquina interna.
        z_f = z_esquina + ancho_mm
        x_lejos = x_int + lado_interno * 3000.0
        segs.append(((x_lejos, z_f), (x_ext, z_f)))
    return segs


def test_geometria():
    print("\n[3] Geometria: contacto, contiguidad y escaneo metrico")
    cfg = robot_config.POR_DEFECTO["navegacion"]
    suelo = geo.Suelo(robot_config.POR_DEFECTO["camara"])

    check(not suelo.calibrado, "sin suelo.json usa el modelo aproximado")

    # --- contiguidad vertical: muro si, sombra no ---
    muro = np.zeros((480, 640), np.uint8)
    muro[100:300, 200:400] = 255            # bloque alto = muro
    y, v = geo.contacto_muro(muro, cfg)
    check(v[300] and 250 < y[300] < 300, "un muro alto da contacto en su base",
          f"y={y[300]} v={v[300]}")
    check(not v[100], "y fuera del muro no hay contacto")

    sombra = np.zeros((480, 640), np.uint8)
    sombra[290:298, 200:400] = 255          # franja de 8 px = sombra en el piso
    _y2, v2 = geo.contacto_muro(sombra, cfg)
    check(not v2[300], "una franja fina NO cuenta como muro (mata las sombras)")

    # --- desconocido no es libre ---
    vacio = np.zeros((480, 640), np.uint8)
    e = geo.escanear(vacio, suelo, cfg)
    check(e.cobertura() == 0.0, "sin muro visible la cobertura es 0")
    check(not e.valido.any(), "y ninguna columna se marca valida: "
                              "desconocido no es lo mismo que despejado")

    # --- proyeccion metrica coherente ---
    e2 = geo.escanear(muro, suelo, cfg)
    _xs, zs = e2.puntos()
    check(len(zs) > 50 and bool(np.all(zs > 0)), "la proyeccion da distancias positivas")
    check(200 < float(np.median(zs)) < 3000, "y en un rango fisico razonable",
          float(np.median(zs)))

    # --- el escaneo sintetico se comporta ---
    e3 = escaneo_mundo(corredor(1000.0, lado_interno=-1))
    izq, der = e3.lateral(-1), e3.lateral(+1)
    check(izq is not None and abs(izq - 250) < 40, "mide bien la pared izquierda", izq)
    check(der is not None and abs(der - 750) < 80, "y la derecha", der)
    r = e3.recta(-1)
    check(r is not None and abs(r[1]) < 3.0, "una pared paralela da angulo ~0",
          None if r is None else r[1])


# ===========================================================================
# 4. El discriminador: cual de los dos muros negros es el interno
# ===========================================================================
def test_salto():
    print("\n[4] Esquina convexa: distinguir muro interno de externo")
    cfg = dict(robot_config.POR_DEFECTO["navegacion"])

    for lado, nombre in ((-1, "izquierda"), (+1, "derecha")):
        e = escaneo_mundo(corredor(1000.0, lado_interno=lado, z_esquina=900.0))
        s = geo.buscar_salto(e, cfg)
        check(s is not None and s.lado == lado,
              f"muro interno a la {nombre}: lo detecta por el salto de rango",
              None if s is None else s.lado)
        if s is not None:
            check(abs(s.z - 900.0) < 250,
                  f"  y situa la esquina cerca de donde esta ({nombre})", s.z)

    # Un corredor recto SIN esquina no debe inventarse un salto.
    e = escaneo_mundo(corredor(1000.0, lado_interno=-1))
    s = geo.buscar_salto(e, cfg)
    check(s is None, "en recta sin esquina no aparece una esquina falsa",
          None if s is None else (s.lado, s.magnitud))

    # El muro EXTERNO tiene esquina concava: no debe generar salto a su lado.
    e = escaneo_mundo(corredor(1000.0, lado_interno=-1, z_esquina=700.0))
    s = geo.buscar_salto(e, cfg)
    check(s is not None and s.lado == -1,
          "la esquina concava del externo NO se confunde con la del interno",
          None if s is None else s.lado)


# ===========================================================================
# 5. Navegacion: los cuatro sintomas observados en pista
# ===========================================================================
def _ang(a):
    return (a + 180.0) % 360.0 - 180.0


def _nav(**over):
    cfg = dict(robot_config.POR_DEFECTO["navegacion"])
    cfg.update(over)
    lim = dict(robot_config.POR_DEFECTO["limites"])
    return nav.Navegador(cfg, lim)


def _correr(n, segs, yaw=None, veces=8, **kw):
    d = None
    for _ in range(veces):
        d = n.paso(escaneo_mundo(segs), yaw, **kw)
    return d


def test_navegacion():
    print("\n[5] Navegacion: se corrigen los sintomas de pista")

    # --- 1) se arrima al muro INTERNO, no al externo --------------------
    for lado, nombre in ((-1, "izquierda"), (+1, "derecha")):
        n = _nav(pared_objetivo_mm=180.0)
        n.fijar_lado_interno(lado)
        # El corredor sintetico deja el carro a 250 mm del interno y el
        # objetivo es 180: tiene que acercarse AL INTERNO.
        d = _correr(n, corredor(1000.0, lado_interno=lado))
        check(np.sign(d.direccion) == lado,
              f"interno a la {nombre}: si esta lejos, gira HACIA el muro interno",
              f"dir={d.direccion}")

    # --- 2) el giro se dispara por la ESQUINA INTERNA -------------------
    n = _nav(min_recto_ms=0, giro_z_mm=350.0, giro_frente_mm=120.0)
    n.fijar_lado_interno(-1)
    d = _correr(n, corredor(1000.0, lado_interno=-1, z_esquina=1200.0), veces=3)
    check(d.estado != nav.GIRO, "no gira solo porque haya muro de frente lejano",
          d.estado)
    d = n.paso(escaneo_mundo(corredor(1000.0, lado_interno=-1, z_esquina=300.0)), 0.0)
    check(d.estado == nav.GIRO, "gira cuando la ESQUINA INTERNA entra en rango",
          f"{d.estado} {d.motivo}")
    check("esquina interna" in d.motivo,
          "  y el motivo dice que la referencia fue la esquina, no el frente",
          d.motivo)

    # --- 3) funciona en los dos sentidos --------------------------------
    for lado, nombre in ((-1, "antihorario"), (+1, "horario")):
        n = _nav(min_recto_ms=0, frames_para_fijar_lado=2)
        segs = corredor(1000.0, lado_interno=lado, z_esquina=1100.0)
        for _ in range(6):
            n.paso(escaneo_mundo(segs), 0.0)
        check(n.lado_interno == lado,
              f"deduce el sentido {nombre} sin que se lo digan", n.lado_interno)

    # --- 4) el giro es de 90 grados, no mas -----------------------------
    n = _nav(min_recto_ms=0, giro_z_mm=400.0)
    n.fijar_lado_interno(+1)
    segs = corredor(1000.0, lado_interno=+1, z_esquina=300.0)
    n.paso(escaneo_mundo(segs), 0.0)
    n.paso(escaneo_mundo(segs), 0.0)
    check(n.estado == nav.GIRO, "entra en giro", n.estado)
    check(n.rumbo_objetivo is not None and abs(_ang(n.rumbo_objetivo - 90.0)) < 1e-6,
          "el objetivo de rumbo es exactamente 90 grados", n.rumbo_objetivo)

    d40 = n.paso(escaneo_mundo(segs), 50.0)     # faltan 40 grados
    d10 = n.paso(escaneo_mundo(segs), 80.0)     # faltan 10
    check(abs(d10.direccion) < abs(d40.direccion),
          "la salida del giro es proporcional: afloja al acercarse",
          f"{d40.direccion} -> {d10.direccion}")
    n.paso(escaneo_mundo(segs), 88.0)           # dentro de tolerancia
    check(n.estado != nav.GIRO, "y cierra el giro en 90 grados, sin pasarse",
          f"{n.estado} yaw=88")

    # --- sin giroscopio se cierra por PARALELISMO, no por espacio libre --
    n = _nav(min_recto_ms=0, giro_z_mm=400.0, giro_min_ms=0)
    n.fijar_lado_interno(+1)
    esq = escaneo_mundo(corredor(1000.0, +1, z_esquina=300.0))
    n.paso(esq, None)
    check(n.estado == nav.GIRO, "sin yaw tambien entra en giro", n.estado)
    # Mientras la esquina siga delante NO puede darse por terminado, aunque el
    # muro previo a la esquina este perfectamente paralelo.
    n.paso(esq, None)
    check(n.estado == nav.GIRO,
          "y no se cierra solo porque el muro de antes de la esquina sea paralelo",
          n.estado)
    time.sleep(0.3)
    d = n.paso(escaneo_mundo(corredor(1000.0, lado_interno=+1)), None)
    check(n.estado != nav.GIRO,
          "sale cuando la esquina ya quedo atras y el muro esta paralelo",
          f"{n.estado} {d.motivo}")


def test_seguridad_y_vueltas():
    print("\n[5b] Guardia del muro externo, cobertura y conteo de vueltas")

    # --- guardia contra el muro externo (regla 9.18) --------------------
    n = _nav(min_externo_mm=250.0, pared_objetivo_mm=250.0)
    n.fijar_lado_interno(-1)
    # Pasillo estrecho: el externo queda a 150 mm, por debajo del minimo.
    segs = [((-250.0, -200.0), (-250.0, 2500.0)), ((150.0, -200.0), (150.0, 2500.0))]
    d = _correr(n, segs)
    check(d.direccion < 0, "si el muro EXTERNO se acerca, empuja hacia el interno",
          d.direccion)
    check("EXTERNO" in d.motivo, "  y lo avisa en el motivo", d.motivo)

    # --- no acelerar hacia lo desconocido -------------------------------
    n = _nav()
    n.fijar_lado_interno(-1)
    vacio = geo.Escaneo(x=np.full(640, np.nan), z=np.full(640, np.nan),
                        valido=np.zeros(640, bool),
                        y_contacto=np.zeros(640, np.int32), ancho=640, alto=480)
    d = n.paso(vacio, 0.0)
    lim = robot_config.POR_DEFECTO["limites"]
    check(d.vel <= lim["vel_giro"],
          "con cobertura 0 no acelera: desconocido no es despejado", d.vel)

    # --- parada de seguridad --------------------------------------------
    n = _nav()
    n.fijar_lado_interno(-1)
    pegado = [((-250.0, -200.0), (-250.0, 2500.0)),
              ((750.0, -200.0), (750.0, 2500.0)),
              ((-400.0, 150.0), (900.0, 150.0))]      # muro a 150 mm de frente
    d = _correr(n, pegado, veces=3)
    check(d.estado == nav.BLOQUEADO and d.vel < 0,
          "muro encima: retrocede en vez de empujar la pared",
          f"{d.estado} vel={d.vel}")

    # --- un muro que sale del encuadre NO es via libre --------------------
    # Es el fallo que hace que el carro acelere contra la pared en el ultimo
    # palmo: por debajo del suelo de medida (~200 mm) el muro desaparece del
    # recorte y el escaneo crudo dice "despejado".
    n = _nav()
    n.fijar_lado_interno(-1)
    cerca = [((-250.0, -200.0), (-250.0, 2500.0)),
             ((750.0, -200.0), (750.0, 2500.0)),
             ((-400.0, 420.0), (900.0, 420.0))]
    d = _correr(n, cerca, veces=4)
    check(abs(n.frente_mm - 420) < 90, "mide el muro de frente mientras se ve",
          round(n.frente_mm))

    # ahora el muro se "esfuma": ninguna columna valida en el pasillo
    vacio = geo.Escaneo(x=np.full(640, np.nan), z=np.full(640, np.nan),
                        valido=np.zeros(640, bool),
                        y_contacto=np.zeros(640, np.int32), ancho=640, alto=480)
    d = n.paso(vacio, 0.0)
    check(d.metricas["frente_mm"] < 700,
          "si se esfuma estando cerca, NO se asume via libre",
          d.metricas["frente_mm"])
    check(d.vel <= robot_config.POR_DEFECTO["limites"]["vel_giro"],
          "y no acelera: sigue tratandolo como un muro que se acerca", d.vel)

    # en cambio, si lo ultimo visto estaba lejos, si es via libre
    n2 = _nav()
    n2.fijar_lado_interno(-1)
    n2.frente_mm = 2500.0
    d2 = n2.paso(vacio, 0.0)
    check(d2.metricas["frente_mm"] > 1000,
          "pero un frente que ya estaba lejos si cuenta como despejado",
          d2.metricas["frente_mm"])

    # --- conteo de vueltas ------------------------------------------------
    n = _nav()
    n.fijar_lado_interno(+1)
    for _ in range(12):
        n._iniciar_giro(0.0, +1)
        n._terminar_giro(0.0)
    check(n.giros == 12, "12 esquinas contadas", n.giros)
    check(n.vueltas == 3, "son exactamente 3 vueltas", n.vueltas)
    check(n.estado == nav.FINALIZANDO, "y entra en la fase de parada", n.estado)

    # --- yaw acumulado sin envolver ---------------------------------------
    n = _nav()
    n.fijar_lado_interno(+1)
    segs = corredor(1000.0, +1)
    for k in range(0, 1100, 20):
        n.paso(escaneo_mundo(segs), _ang(k))
    check(abs(n.yaw_acumulado - 1080) < 80,
          "el yaw acumulado llega a ~1080 grados sin envolverse en +-180",
          round(n.yaw_acumulado, 1))



def test_desatasco():
    """Salir de un atasco es un requisito, no un extra.

    La regla 9.23 solo deja pedir una reparacion por ronda, y no la conceden
    con el carro en movimiento. Un carro que se queda restregandose contra la
    pared sin avanzar ni parar del todo es el peor de los casos: pierde la
    vuelta y encima puede que no le dejen tocarlo.
    """
    print("\n[5e] Salir de un atasco")
    sys.path.insert(0, str(RAIZ / "tools"))
    try:
        import simulador as sim
    except Exception as e:                       # pragma: no cover
        print(f"       (sin simulador: {e}); me lo salto")
        return

    cfg = robot_config.cargar(TMP / "robot_bloq_test.json")
    pista = sim.Pista((600., 600., 600., 600.))

    # Morro contra la esquina exterior y atravesado: encajonado de verdad.
    carro = sim.Carro(x=2830., y=280., theta=math.radians(-40))
    n = nav.Navegador(dict(cfg["navegacion"]), dict(cfg["limites"]))
    n.fijar_lado_interno(-1)

    dt, t, salio = 1 / 30, 0.0, None
    for i in range(300):
        e = sim.escanear_pista(pista, carro, hfov=100.)
        d = n.paso(e, nav._norm_angulo(carro.yaw_deg), ahora=t)
        if i == 0:
            check(d.estado == nav.BLOQUEADO,
                  "encajonado contra la pared: detecta el atasco", d.estado)
        if i > 3 and d.estado != nav.BLOQUEADO and salio is None:
            salio = t
        carro.avanzar(d.vel, d.direccion, dt)
        t += dt
        if salio is not None:
            break

    check(salio is not None, "y SALE de el", f"seguia atascado tras {t:.1f} s")
    if salio is not None:
        check(salio < 6.0, "en pocos segundos", f"{salio:.1f} s")

    # --- el lado de la maniobra no puede recalcularse cada frame ---------
    # Ese era el fallo: al retroceder cambia lo que se ve, el lado "mas
    # despejado" se da la vuelta, el volante oscila y el carro se restriega
    # sin salir. Aqui se comprueba que la eleccion se MANTIENE.
    carro = sim.Carro(x=2830., y=280., theta=math.radians(-40))
    n2 = nav.Navegador(dict(cfg["navegacion"]), dict(cfg["limites"]))
    n2.fijar_lado_interno(-1)
    t = 0.0
    lados = []
    for _ in range(20):
        e = sim.escanear_pista(pista, carro, hfov=100.)
        d = n2.paso(e, nav._norm_angulo(carro.yaw_deg), ahora=t)
        if d.estado == nav.BLOQUEADO:
            lados.append(n2.bloq_lado)
        carro.avanzar(d.vel, d.direccion, dt)
        t += dt
    check(len(set(lados)) == 1,
          "el lado de la maniobra se fija y no oscila cada frame", set(lados))

    # --- y alterna si el primer intento no sirve -------------------------
    n3 = nav.Navegador(dict(cfg["navegacion"]), dict(cfg["limites"]))
    n3.fijar_lado_interno(-1)
    n3.estado = nav.BLOQUEADO
    n3.bloq_lado = 1
    n3.bloq_t0 = 0.0
    n3._t = 0.0
    vacio = geo.Escaneo(x=np.full(640, np.nan), z=np.full(640, np.nan),
                        valido=np.zeros(640, bool),
                        y_contacto=np.zeros(640, np.int32), ancho=640, alto=480)
    fases = set()
    for k in range(60):
        n3._t = k * 0.5
        n3._desatascar(vacio, 0.0, 32.0, 100.0, 100.0)
        fases.add(n3.bloq_fase)
    check(fases == {0, 1}, "alterna marcha atras y marcha adelante", fases)
    check(n3.bloq_intentos > 0, "y cuenta los intentos para invertir el lado",
          n3.bloq_intentos)


# ===========================================================================
# 5c. Señales de trafico (Obstacle Challenge)
# ===========================================================================
def test_senales():
    print("\n[5c] Senales verdes y rojas")
    cfg = dict(robot_config.POR_DEFECTO["obstaculos"])
    det = obs.DetectorSenales(cfg)

    rojo = obs.Senal(obs.ROJO, 0.0, 800.0, 20, 40, None)
    verde = obs.Senal(obs.VERDE, 0.0, 800.0, 20, 40, None)

    check(rojo.lado_paso == +1, "ROJO: el carro pasa por la DERECHA del pilar")
    check(verde.lado_paso == -1, "VERDE: el carro pasa por la IZQUIERDA del pilar")

    o_rojo = det.objetivo(rojo)
    o_verde = det.objetivo(verde)
    check(o_rojo is not None and o_rojo > 0,
          "con un pilar rojo centrado, el objetivo se va a la derecha", o_rojo)
    check(o_verde is not None and o_verde < 0,
          "con uno verde centrado, a la izquierda", o_verde)
    check(abs(o_rojo) >= 150,
          "y el margen deja hueco de sobra para no tocarlo", o_rojo)

    rojo2 = obs.Senal(obs.ROJO, -200.0, 700.0, 20, 40, None)
    check(det.objetivo(rojo2) < o_rojo,
          "si el pilar esta a la izquierda, el objetivo se desplaza con el")

    lejano = obs.Senal(obs.ROJO, 900.0, 700.0, 20, 40, None)
    check(abs(det.objetivo(lejano)) <= cfg["senal_desvio_max_mm"] + 1,
          "el desvio nunca supera el tope configurado", det.objetivo(lejano))

    det.activa = None
    encima = obs.Senal(obs.ROJO, 0.0, 100.0, 20, 40, None)
    check(det.elegir([encima]) is None,
          "una senal ya pegada al carro se suelta (no se corrige a ciegas)")

    # --- la senal desvia en RECTO pero no toca la logica de giro --------
    segs = corredor(1000.0, lado_interno=-1)
    n1 = _nav(min_recto_ms=0)
    n1.fijar_lado_interno(-1)
    d_sin = _correr(n1, segs)
    n2 = _nav(min_recto_ms=0)
    n2.fijar_lado_interno(-1)
    d_con = _correr(n2, segs, objetivo_lateral=300.0, motivo_extra="rojo")
    check(d_con.direccion > d_sin.direccion,
          "un objetivo lateral a la derecha empuja la direccion a la derecha",
          f"{d_sin.direccion} -> {d_con.direccion}")


# ===========================================================================
# 5d. La cadena completa: imagen -> mascara -> escaneo -> decision
# ===========================================================================
def test_pipeline_imagen():
    print("\n[5d] Cadena completa desde una imagen sintetica")
    colores = cc.colores_por_defecto()
    cfg = dict(robot_config.POR_DEFECTO["navegacion"])
    suelo = geo.Suelo(robot_config.POR_DEFECTO["camara"])

    img = pista([(0, 200), (320, 230), (639, 200)])
    e = geo.escanear(_mascara_muro(img, colores), suelo, cfg)
    check(e.cobertura() > 0.6, "detecta el muro en casi todas las columnas",
          round(e.cobertura(), 2))

    n = _nav()
    n.fijar_lado_interno(-1)
    d = n.paso(e, 0.0)
    check(isinstance(d.vel, int) and -100 <= d.vel <= 100, "la decision es valida")
    check(-100 <= d.direccion <= 100, "y la direccion no se sale de rango")

    # Muro lo mas cerca que permite la geometria: `ignorar_abajo` recorta la
    # franja del chasis, asi que hay un minimo medible por debajo del cual la
    # camara sencillamente no ve. Ahi la respuesta correcta es frenar a fondo;
    # el retroceso por BLOQUEADO se prueba con escaneo metrico en [5b].
    cerca = pista([(0, 465), (639, 465)])
    e2 = geo.escanear(_mascara_muro(cerca, colores), suelo, cfg)
    frente = e2.frente(110.0)
    check(frente < 400, "un muro pegado se mide cerca", round(frente))
    n2 = _nav()
    n2.fijar_lado_interno(-1)
    d2 = None
    for _ in range(3):
        d2 = n2.paso(e2, 0.0)
    lim = robot_config.POR_DEFECTO["limites"]
    check(d2.vel <= lim["vel_giro"],
          "con el muro encima frena hasta la velocidad minima",
          f"vel={d2.vel} frente={frente:.0f} estado={d2.estado}")


# ===========================================================================
# 6. Enlace serie contra un ESP32 falso
# ===========================================================================
class ESP32Falso:
    """Habla el protocolo por el otro extremo de un pty. Imita al firmware:
    contesta PONG, manda telemetria a 20 Hz y aplica el failsafe."""

    def __init__(self):
        self.maestro, self.esclavo = os.openpty()
        self.ruta = os.ttyname(self.esclavo)
        self.lector = P.Lector()
        self.mandos: List[P.Mando] = []
        self.ultimo_mando_t = 0.0
        self.failsafe = True
        self.parar = threading.Event()
        self.hilo = threading.Thread(target=self._bucle, daemon=True)

    def iniciar(self):
        self.hilo.start()

    def cerrar(self):
        self.parar.set()
        self.hilo.join(timeout=1.0)
        for fd in (self.maestro, self.esclavo):
            try:
                os.close(fd)
            except OSError:
                pass

    def _bucle(self):
        import select
        t_tele = 0.0
        seq_eco = 0
        while not self.parar.is_set():
            r, _, _ = select.select([self.maestro], [], [], 0.02)
            if r:
                try:
                    datos = os.read(self.maestro, 512)
                except OSError:
                    break
                for tipo, pl in self.lector.alimentar(datos):
                    if tipo == P.TIPO_MANDO:
                        m = P.Mando.desde_payload(pl)
                        self.mandos.append(m)
                        seq_eco = m.seq
                        self.ultimo_mando_t = time.time()
                    elif tipo == P.TIPO_PING:
                        self._escribir(P.empaquetar(P.TIPO_PONG, pl[:1]))
            ahora = time.time()
            self.failsafe = (ahora - self.ultimo_mando_t) > 0.3
            if ahora - t_tele > 0.05:
                t_tele = ahora
                ult = self.mandos[-1] if self.mandos else P.Mando()
                estado = 0
                if ult.armado and not self.failsafe:
                    estado |= P.E_ARMADO
                if self.failsafe:
                    estado |= P.E_FAILSAFE
                pwm = 0 if self.failsafe else abs(ult.vel) * ult.vmax // 100
                t = P.Telemetria(seq_eco=seq_eco, estado=estado, pwm=pwm,
                                 angulo=100 + ult.direccion * 35 // 100,
                                 ms_desde_mando=int((ahora - self.ultimo_mando_t) * 1000)
                                 if self.ultimo_mando_t else 9999,
                                 tramas_malas=self.lector.crc_malos, version=2)
                self._escribir(t.a_bytes())

    def _escribir(self, datos: bytes):
        try:
            os.write(self.maestro, datos)
        except OSError:
            pass


def test_enlace():
    print("\n[6] Enlace serie contra un ESP32 falso (pty)")
    if sys.platform.startswith("win"):
        print("       (los pty no existen en Windows; me lo salto)")
        return
    try:
        import serial  # noqa: F401
    except Exception as e:
        print(f"       (sin pyserial: {e}); me lo salto")
        return

    from src import enlace as enl

    esp = ESP32Falso()
    esp.iniciar()
    cfg = dict(robot_config.POR_DEFECTO["enlace"], puerto=esp.ruta, reintento_s=0.2)
    e = enl.Enlace(cfg, al_log=lambda s: None)
    e.fijar_vmax(120)
    e.iniciar()

    t0 = time.time()
    while not e.conectado and time.time() - t0 < 6:
        time.sleep(0.05)
    check(e.conectado, f"autodetecta el puerto ({esp.ruta})", e.motivo)

    # Como haria el lazo de control real: refrescando la orden todo el rato
    n0 = len(esp.mandos)
    t0 = time.time()
    while time.time() - t0 < 0.5:
        e.mandar(40, -25, armado=True)
        time.sleep(0.02)
    check(len(esp.mandos) - n0 > 10, "manda a ~50 Hz",
          f"{len(esp.mandos) - n0} tramas en 0.5 s")
    ult = esp.mandos[-1]
    check(ult.vel == 40 and ult.direccion == -25, "los valores llegan intactos",
          f"vel={ult.vel} dir={ult.direccion}")
    check(ult.vmax == 120, "vmax viaja en cada trama", ult.vmax)
    check(ult.armado, "la bandera de armado llega")
    check(e.telemetria.version == 2 and not e.telemetria.failsafe,
          "vuelve telemetria valida", e.telemetria)
    check(0 <= e.latencia_ms < 500, "mide latencia con el ping", e.latencia_ms)

    # secuencia creciente (para detectar tramas perdidas)
    seqs = [m.seq for m in esp.mandos[-20:]]
    saltos = sum(1 for a, b in zip(seqs, seqs[1:]) if (b - a) % 256 != 1)
    check(saltos == 0, "el numero de secuencia no salta", f"{saltos} saltos")

    # Si el lazo de arriba se calla, el enlace manda cero por su cuenta.
    # La orden caduca a los 250 ms, asi que despues de 0.6 s las ultimas tramas
    # ya tienen que ir a cero.
    t_corte = time.time()
    time.sleep(0.6)
    recientes = esp.mandos[-10:]
    check(all(m.vel == 0 for m in recientes),
          "si nadie refresca la orden, manda velocidad 0 (no repite la vieja)",
          [m.vel for m in recientes])
    check(not any(m.armado for m in recientes), "y quita el armado")

    # y comprobamos que la caducidad ocurre cerca de los 250 ms, no antes
    primeros_cero = [i for i, m in enumerate(esp.mandos) if m.vel == 0
                     and i > len(esp.mandos) - 40]
    check(bool(primeros_cero), "hay tramas a cero tras el silencio")

    e.mandar(30, 0, armado=True)
    time.sleep(0.05)
    e.parar(emergencia=True)
    time.sleep(0.2)
    check(esp.mandos[-1].parada and esp.mandos[-1].vel == 0,
          "la parada de emergencia llega con vel 0")

    # sin mandos, el ESP32 falso entra en failsafe
    e.cerrar()
    time.sleep(0.5)
    check(esp.failsafe, "cerrar el enlace deja al ESP32 en failsafe")
    esp.cerrar()

    # puerto inexistente: no revienta, solo avisa
    e2 = enl.Enlace(dict(robot_config.POR_DEFECTO["enlace"],
                         puerto="/dev/no_existe_xyz", reintento_s=0.1),
                    al_log=lambda s: None)
    t_busca = time.time()
    e2.iniciar()
    t0 = time.time()
    while e2.motivo in ("sin iniciar", "buscando el ESP32...") and time.time() - t0 < 20:
        time.sleep(0.1)
    tardanza = time.time() - t_busca
    check(not e2.conectado, "un puerto inexistente deja el enlace desconectado")
    check("no se encontro" in e2.motivo or "pyserial" in e2.motivo,
          "y lo explica", e2.motivo)
    check(tardanza < 15, "la busqueda completa no se eterniza",
          f"{tardanza:.1f} s recorriendo {len(enl.candidatos())} candidatos")
    e2.cerrar()


# ===========================================================================
# 7. Robot + servidor web
# ===========================================================================
def test_robot_y_web():
    print("\n[7] Robot completo y servidor web")
    from src import robot as robot_mod, servidor as srv_mod

    img = pista([(0, 200), (320, 210), (639, 320)])
    ruta = str(TMP / "pista_test.png")
    cv2.imwrite(ruta, img)

    cfg = robot_config.cargar(TMP / "robot_test.json")
    cfg["red"]["puerto_http"] = 8391
    cfg["navegacion"]["min_recto_ms"] = 0
    perfil = cc.obtener(cc.cargar(TMP / "colors_test.json"))

    r = robot_mod.Robot(cfg, perfil, simulado=True, fuente_imagen=ruta)
    r.iniciar()
    srv = srv_mod.Servidor(r, cfg["red"])
    url_base = f"http://127.0.0.1:{cfg['red']['puerto_http']}"
    srv.iniciar()
    time.sleep(0.8)

    check(r.frame_anotado is not None, "el hilo de control produce imagen anotada")
    check(not r.armado, "el robot NACE DESARMADO (esto es lo importante)")
    check(r.decision.vel == 0, "y con velocidad 0 mientras este desarmado")

    def get(ruta_rel, n=None):
        with urllib.request.urlopen(url_base + ruta_rel, timeout=5) as resp:
            return resp.read() if n is None else resp.read(n)

    pagina = get("/")
    check(b"Carrito WRO" in pagina and b"stream.mjpg" in pagina, "sirve la pagina")

    est = json.loads(get("/api/estado"))
    check(est["armado"] is False and "enlace" in est, "el JSON de estado responde")
    check("frente_mm" in est["decision"]["metricas"],
          "e incluye las metricas metricas del muro", est["decision"]["metricas"])
    check(est["reto"] == "abierto" and "carrera" in est,
          "y el estado del reto y de la carrera")

    get("/api/cmd?armar=1")
    time.sleep(0.3)
    check(r.armado, "la web puede armar")
    est = json.loads(get("/api/estado"))
    check(est["decision"]["vel"] != 0, "y entonces si decide velocidad",
          est["decision"])

    get("/api/cmd?vmax=77")
    check(r.cfg["limites"]["vmax"] == 77 and r.enlace._vmax == 77,
          "cambiar vmax desde la web llega hasta el enlace", r.enlace._vmax)

    get("/api/cmd?reto=obstaculos")
    check(r.reto == "obstaculos", "la web cambia al reto de obstaculos", r.reto)
    get("/api/cmd?reto=abierto")
    check(r.reto == "abierto", "y vuelve al open challenge", r.reto)
    get("/api/cmd?pared_objetivo_mm=210&aprox_max_grados=18")
    check(r.cfg["navegacion"]["pared_objetivo_mm"] == 210.0,
          "cambia la distancia al muro interno en caliente")
    check(r.cfg["navegacion"]["aprox_max_grados"] == 18.0,
          "y el angulo de aproximacion")
    get("/api/cmd?lado_interno=der")
    check(r.navegador.lado_interno == 1, "y puede forzarse el lado interno")

    get("/api/cmd?emergencia=1")
    time.sleep(0.2)
    check(not r.armado and r.modo == "parado", "la parada de emergencia desarma")

    # --- stream MJPEG ---
    datos = get("/stream.mjpg", 60000)
    check(datos.count(b"--FRAME") >= 2, "el stream MJPEG entrega varios frames",
          datos.count(b"--FRAME"))
    check(b"\xff\xd8" in datos, "y son JPEG de verdad")
    masc = get("/mascara.mjpg", 30000)
    check(masc.count(b"--FRAME") >= 1, "tambien sirve la mascara para depurar")

    # --- seguridad en manual ---
    r.armar(True)
    r.fijar_modo("manual")
    check(r.manual == {"vel": 0, "dir": 0}, "entrar en manual empieza siempre parado")
    r.mando_manual(60, 0)
    time.sleep(0.25)
    check(r.decision.vel == 60, "en manual obedece el mando")

    cerca = pista([(0, 430), (639, 430)])
    ruta_muro = str(TMP / "pista_muro.png")
    cv2.imwrite(ruta_muro, cerca)
    r._imagen_fija = cv2.imread(ruta_muro)
    # Hay que seguir refrescando el mando mientras se comprueba: si no, lo que
    # pararia el carro seria el hombre-muerto y no la capa de seguridad, y la
    # prueba pasaria por el motivo equivocado.
    for _ in range(8):
        r.mando_manual(60, 0)
        time.sleep(0.05)
    check(r.decision.vel <= 0, "pero NO te deja empotrarte aunque lo pidas",
          f"vel={r.decision.vel} motivo={r.decision.motivo}")
    check("bloqueado" in r.decision.motivo,
          "y lo para la seguridad, no el hombre-muerto", r.decision.motivo)

    # --- hombre-muerto del mando manual (el joystick de la web) ---
    r._imagen_fija = cv2.imread(ruta)          # pista despejada otra vez
    r.mando_manual(55, 20)
    time.sleep(0.25)
    check(r.decision.vel == 55, "en pista libre el mando manual vuelve a obedecer",
          f"vel={r.decision.vel}")

    time.sleep(0.6)                            # dejar de refrescar mas de 400 ms
    check(r.decision.vel == 0 and r.decision.direccion == 0,
          "si el joystick deja de refrescar, para y centra el servo",
          f"vel={r.decision.vel} dir={r.decision.direccion}")
    check(r.manual == {"vel": 0, "dir": 0}, "y el mando guardado se pone a cero")

    r.mando_manual(40, 0)
    time.sleep(0.25)
    check(r.decision.vel == 40, "y se recupera en cuanto vuelve a llegar mando",
          f"vel={r.decision.vel}")
    r.fijar_modo("auto")
    check(r.manual == {"vel": 0, "dir": 0}, "cambiar de modo limpia el mando manual")

    # --- sin imagen se desarma ---
    r.fijar_modo("auto")
    r.armar(True)
    r._imagen_fija = None
    r._cap = None
    time.sleep(0.6)
    check(not r.armado, "si se pierde la camara, el robot se desarma solo")

    srv.cerrar()
    r.cerrar()
    check(True, "todo se cierra sin colgarse")


# ===========================================================================
def test_config():
    print("\n[8] Configuracion del robot")
    ruta = TMP / "robot_cfg_test.json"
    if ruta.exists():
        ruta.unlink()
    cfg = robot_config.cargar(ruta)
    check(ruta.exists(), "crea robot.json si no existe")
    cfg["limites"]["vmax"] = 99
    robot_config.guardar(cfg, ruta)
    check(robot_config.cargar(ruta)["limites"]["vmax"] == 99, "guarda y relee")

    # un archivo al que le faltan claves nuevas
    ruta.write_text(json.dumps({"limites": {"vmax": 55}}), encoding="utf-8")
    cfg2 = robot_config.cargar(ruta)
    check(cfg2["limites"]["vmax"] == 55, "respeta lo que hay")
    check("navegacion" in cfg2 and "pared_objetivo_mm" in cfg2["navegacion"]
          and "obstaculos" in cfg2,
          "y rellena las claves que falten")
    ruta.write_text("{roto", encoding="utf-8")
    check(robot_config.cargar(ruta)["limites"]["vmax"] == 130,
          "si esta roto usa los valores por defecto sin explotar")

    print("\n[9] Colores nuevos")
    colores = cc.colores_por_defecto()
    for c in ("rojo", "verde", "negro", "magenta", "naranja", "azul"):
        check(c in colores, f"existe el color '{c}'")
    # un perfil viejo, sin los colores nuevos, los hereda
    viejo = {"nombre": "antiguo", "colores": {"rojo": colores["rojo"]}}
    migrado = cc.normalizar_perfil(viejo)
    check(set(migrado["colores"]) >= set(colores),
          "un perfil guardado antes hereda los colores nuevos",
          list(migrado["colores"]))
    check(migrado["colores"]["rojo"]["rangos"] == colores["rojo"]["rangos"],
          "sin pisar lo que ya estaba calibrado")


# ===========================================================================
# 10. Vueltas completas en lazo cerrado (el simulador)
# ===========================================================================
def test_vuelta_completa():
    """La prueba de integracion de verdad.

    Todo lo anterior valida piezas: que el salto se detecta, que el PD tiene
    el signo bueno, que el giro cierra a 90 grados. Ninguna de ellas puede
    ver que el conjunto se sale de la pista en la tercera esquina, porque
    para eso hay que CERRAR EL LAZO: lo que decide el navegador mueve el
    carro, y mover el carro cambia lo que la camara ve al frame siguiente.

    Aqui se corre un subconjunto representativo. El barrido completo de las
    16 combinaciones de ancho por los dos sentidos, mas deriva, ruido y
    lentes, esta en `python3 tools/simulador.py --todas`.
    """
    print("\n[10] Vueltas completas en lazo cerrado")
    sys.path.insert(0, str(RAIZ / "tools"))
    try:
        import simulador as sim
    except Exception as e:                       # pragma: no cover
        print(f"       (no se pudo cargar el simulador: {e}); me lo salto")
        return

    cfg = robot_config.cargar(TMP / "robot_sim_test.json")

    casos = [
        ("corredor ancho, antihorario", (1000., 1000., 1000., 1000.), "ccw", {}),
        ("corredor ancho, horario", (1000., 1000., 1000., 1000.), "cw", {}),
        ("corredor estrecho, antihorario", (600., 600., 600., 600.), "ccw", {}),
        ("anchos mezclados (el caso real)", (600., 1000., 600., 1000.), "ccw", {}),
        ("sin giroscopio", (1000., 600., 1000., 600.), "ccw", {"usar_yaw": False}),
        ("con 2 grados/s de deriva", (600., 600., 1000., 1000.), "cw",
         {"deriva_yaw_grados_s": 2.0}),
        ("con 25 mm de ruido en el escaneo", (1000., 1000., 600., 1000.), "ccw",
         {"ruido_mm": 25.0}),
    ]

    for nombre, anchos, sentido, extra in casos:
        inf = sim.simular(sim.Pista(anchos), sentido=sentido, cfg=cfg, **extra)
        check(not inf.toco_exterior,
              f"{nombre}: NO toca el muro exterior (regla 9.18)",
              f"min_ext={inf.min_exterior:.0f} mm  {inf.motivo}")
        check(inf.vueltas_reales >= 2.9,
              f"{nombre}: completa las tres vueltas",
              f"{inf.vueltas_reales:.2f} vueltas  {inf.motivo}")
        check(inf.vueltas_contadas == 3 and inf.giros == 12,
              f"{nombre}: cuenta 3 vueltas y 12 esquinas",
              f"vueltas={inf.vueltas_contadas} giros={inf.giros}")

    # --- el sentido se DEDUCE, en los dos casos --------------------------
    for sentido, esperado, nombre in (("ccw", -1, "antihorario -> interno izq"),
                                      ("cw", +1, "horario -> interno der")):
        inf = sim.simular(sim.Pista((1000., 1000., 1000., 1000.)),
                          sentido=sentido, cfg=cfg)
        check(inf.lado_interno == esperado,
              f"deduce el sentido solo: {nombre}", inf.lado_interno)

    # --- para sola, sin que nadie se lo diga -----------------------------
    inf = sim.simular(sim.Pista((1000., 1000., 1000., 1000.)), "ccw", cfg=cfg)
    check("paro solo" in inf.motivo,
          "para sola en la seccion de salida tras las tres vueltas", inf.motivo)
    check(inf.segundos < 170.0,
          "y lo hace dentro de los 3 minutos de la ronda", f"{inf.segundos:.0f} s")


# ===========================================================================
if __name__ == "__main__":
    print(f"OpenCV {cv2.__version__} · Python {sys.version.split()[0]}")
    for f in (test_protocolo_cruzado, test_lector_robusto, test_geometria,
              test_salto, test_navegacion, test_seguridad_y_vueltas,
              test_desatasco, test_senales, test_pipeline_imagen, test_enlace,
              test_robot_y_web, test_config, test_vuelta_completa):
        try:
            f()
        except Exception as e:
            import traceback
            traceback.print_exc()
            _fallos.append(f"{f.__name__} lanzo {e}")
    print(f"\n{_ok} pruebas ok, {len(_fallos)} fallos")
    for f in _fallos:
        print("  -", f)
    sys.exit(1 if _fallos else 0)
