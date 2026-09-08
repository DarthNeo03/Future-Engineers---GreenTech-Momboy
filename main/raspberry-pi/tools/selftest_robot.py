#!/usr/bin/env python3
"""
selftest_robot.py — Pruebas del sistema completo SIN carro.

    python3 tools/selftest_robot.py

Cubre:
  1. Protocolo Python <-> C++ (los mismos bytes en las dos implementaciones)
  2. El lector de tramas frente a ruido, truncados y arranques a media trama
  3. Navegacion sobre pistas sinteticas: recta, muro a un lado, esquina, choque
  4. Enlace serie de verdad, contra un ESP32 falso al otro lado de un pty
  5. Servidor web: pagina, JSON de estado, ordenes y stream MJPEG
  6. Seguridad del Robot: sin imagen se desarma, y en manual no te deja
     empotrarte

Lo unico que no se puede probar aqui es el ESP32 real; para eso esta
tools/test_firmware.cpp, que compila su logica con g++.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import color_config as cc, navegacion as nav, protocolo as P  # noqa: E402
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
    binario = Path("/tmp/tfw_selftest")
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
# 3. Navegacion
# ===========================================================================
def pista(alturas, ancho=640, alto=480, ruido=5):
    """Dibuja una vista con muro negro arriba cuyo borde inferior sigue
    'alturas' (lista de (x, y) que se interpola). Piso blanco debajo."""
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


def test_perfil():
    print("\n[3] Perfil del muro (el 'LIDAR pobre')")
    colores = cc.colores_por_defecto()
    cfg = robot_config.POR_DEFECTO["navegacion"]

    # muro lejos y parejo
    m = _mascara_muro(pista([(0, 150), (639, 150)]), colores)
    p = nav.perfil_desde_mascara(m, cfg)
    check(abs(p.izq - p.der) < 0.03, "muro parejo: izquierda y derecha iguales",
          f"{p.izq:.3f} vs {p.der:.3f}")
    check(0.6 < p.pasillo < 0.8, "pasillo despejado", f"{p.pasillo:.3f}")

    # muro que baja por la izquierda (mas cerca a la izquierda)
    m = _mascara_muro(pista([(0, 380), (320, 220), (639, 150)]), colores)
    p2 = nav.perfil_desde_mascara(m, cfg)
    check(p2.der > p2.izq + 0.15, "muro cerca por la izquierda: mas libre a la derecha",
          f"izq={p2.izq:.3f} der={p2.der:.3f}")

    # sin muro ninguno
    m = _mascara_muro(np.full((480, 640, 3), 235, np.uint8), colores)
    p3 = nav.perfil_desde_mascara(m, cfg)
    check(p3.pasillo > 0.95 and not p3.hay_muro, "sin muro, todo libre",
          f"{p3.pasillo:.3f}")

    # una mota negra suelta no debe leerse como muro pegado
    img = np.full((480, 640, 3), 235, np.uint8)
    cv2.circle(img, (320, 460), 3, (10, 10, 10), -1)
    m = _mascara_muro(img, colores)
    p4 = nav.perfil_desde_mascara(m, cfg)
    check(p4.pasillo > 0.9, "una mota no cierra el pasillo", f"{p4.pasillo:.3f}")

    # el chasis en la franja inferior se ignora
    img = np.full((480, 640, 3), 235, np.uint8)
    img[455:, :] = (12, 12, 12)
    m = _mascara_muro(img, colores)
    p5 = nav.perfil_desde_mascara(m, dict(cfg, ignorar_abajo=0.08))
    check(p5.pasillo > 0.9, "la franja del chasis no cuenta como muro",
          f"{p5.pasillo:.3f}")
    p6 = nav.perfil_desde_mascara(m, dict(cfg, ignorar_abajo=0.0))
    check(p6.pasillo < 0.2, "y sin ignorarla, si la ve (la prueba comprueba el filtro)",
          f"{p6.pasillo:.3f}")


def test_estrategias():
    print("\n[4] Estrategias de esquive")
    colores = cc.colores_por_defecto()
    base = dict(robot_config.POR_DEFECTO["navegacion"], min_recto_ms=0)
    lim = dict(robot_config.POR_DEFECTO["limites"])

    def decidir(img, cfg=None, pasos=3, yaw=None, navegador=None):
        cfg = cfg or base
        m = _mascara_muro(img, colores)
        p = nav.perfil_desde_mascara(m, cfg)
        n = navegador or nav.Navegador(cfg, lim)
        d = None
        for _ in range(pasos):
            d = n.paso(p, yaw)
            time.sleep(0.01)
        return d, p, n

    # --- recta despejada ---
    d, p, _ = decidir(pista([(0, 150), (639, 150)]))
    check(d.estado == nav.RECTO, "muro lejos: sigue recto", d.estado)
    check(abs(d.direccion) < 12, "y casi no gira", d.direccion)
    check(d.vel >= lim["vel_giro"], "con velocidad de crucero", d.vel)

    # --- muro cerca por la izquierda -> girar a la derecha ---
    d, p, _ = decidir(pista([(0, 400), (320, 250), (639, 160)]))
    check(d.direccion > 10, "muro a la izquierda: gira a la derecha",
          f"dir={d.direccion} izq={p.izq:.2f} der={p.der:.2f}")

    # --- muro cerca por la derecha -> girar a la izquierda ---
    d, p, _ = decidir(pista([(0, 160), (320, 250), (639, 400)]))
    check(d.direccion < -10, "muro a la derecha: gira a la izquierda",
          f"dir={d.direccion} izq={p.izq:.2f} der={p.der:.2f}")

    # --- esquina: pared de frente todavia a distancia, hueco a la derecha ---
    # Base del muro en y=300 dentro del pasillo -> libre 0.375, por debajo de
    # girar_bajo pero por encima de parar_bajo: justo la situacion de esquina.
    cfg_esq = dict(base, girar_bajo=0.45, parar_bajo=0.20)
    d, p, n = decidir(pista([(0, 300), (430, 300), (520, 190), (639, 160)]), cfg_esq)
    check(n.estado == nav.GIRO, "pared de frente a distancia: entra en modo giro",
          f"{n.estado} pasillo={p.pasillo:.2f}")
    check(d.direccion > 40, "y gira fuerte hacia el hueco", d.direccion)
    check(d.vel > 0, "sin pararse (girar parado no sirve con direccion Ackermann)", d.vel)

    # Y si ya es demasiado tarde, la seguridad manda sobre la estrategia
    d2, p2, n2 = decidir(pista([(0, 425), (430, 425), (520, 200), (639, 150)]), cfg_esq)
    check(n2.estado == nav.BLOQUEADO, "si ya esta encima, bloqueado en vez de giro",
          f"{n2.estado} pasillo={p2.pasillo:.2f}")
    check(d2.vel < 0 and d2.direccion < 0,
          "retrocede girando al reves para reencuadrar hacia el hueco derecho",
          f"vel={d2.vel} dir={d2.direccion}")

    # --- muro encima: parada / retroceso ---
    d, p, _ = decidir(pista([(0, 465), (639, 465)]))
    check(d.estado == nav.BLOQUEADO, "muro encima: bloqueado", d.estado)
    check(d.vel < 0, "retrocede en vez de empujar la pared", d.vel)

    # --- frenado progresivo ---
    lejos, _, _ = decidir(pista([(0, 140), (639, 140)]))
    cerca, pc, _ = decidir(pista([(0, 330), (639, 330)]))
    check(cerca.vel < lejos.vel, "cuanto mas cerca el muro, mas despacio",
          f"{cerca.vel} vs {lejos.vel} (pasillo {pc.pasillo:.2f})")

    # --- seguir pared izquierda (con el pasillo despejado, para aislar el PD) ---
    cfg_p = dict(base, estrategia="pared", lado_pared="izq", pared_objetivo=0.40)
    lejos_de_pared = pista([(0, 250), (200, 250), (260, 150), (639, 150)])
    cerca_de_pared = pista([(0, 340), (200, 340), (260, 150), (639, 150)])
    d1, p1, _ = decidir(lejos_de_pared, cfg_p)
    d2, p2, _ = decidir(cerca_de_pared, cfg_p)
    check(p1.izq > p2.izq, "la banda izquierda mide bien la distancia a esa pared",
          f"lejos={p1.izq:.2f} cerca={p2.izq:.2f}")
    check(d1.direccion < d2.direccion,
          "lejos de la pared se arrima (izquierda) y cerca se separa (derecha)",
          f"lejos dir={d1.direccion}  cerca dir={d2.direccion}")
    check(d1.direccion < 0 < d2.direccion,
          "y los signos son los correctos para la pared izquierda",
          f"{d1.direccion} / {d2.direccion}")

    cfg_pd = dict(cfg_p, lado_pared="der")
    d3, _, _ = decidir(pista([(0, 150), (380, 150), (440, 250), (639, 250)]), cfg_pd)
    d4, _, _ = decidir(pista([(0, 150), (380, 150), (440, 340), (639, 340)]), cfg_pd)
    check(d3.direccion > d4.direccion,
          "con la pared derecha los signos se invierten, como debe ser",
          f"lejos dir={d3.direccion}  cerca dir={d4.direccion}")

    # --- la direccion nunca se pasa del tope ---
    peor = 0
    for izq in range(120, 460, 40):
        for der in range(120, 460, 40):
            d, _, _ = decidir(pista([(0, izq), (639, der)]), pasos=2)
            if abs(d.direccion) > lim["dir_max"] or abs(d.vel) > 100:
                peor = max(peor, abs(d.direccion))
    check(peor == 0, "ninguna combinacion saca la direccion de rango", peor)


def test_yaw():
    print("\n[5] Ayuda del giroscopio")
    colores = cc.colores_por_defecto()
    cfg = dict(robot_config.POR_DEFECTO["navegacion"], min_recto_ms=0, usar_yaw=True)
    lim = dict(robot_config.POR_DEFECTO["limites"])
    m = _mascara_muro(pista([(0, 150), (639, 150)]), colores)
    p = nav.perfil_desde_mascara(m, cfg)

    n = nav.Navegador(cfg, lim)
    n.paso(p, yaw=0.0)                       # fija el rumbo objetivo en 0
    d_recto = n.paso(p, yaw=0.0)
    d_torcido = n.paso(p, yaw=-10.0)         # el carro se fue 10 grados
    check(d_torcido.direccion > d_recto.direccion,
          "si el carro se desvia, el yaw corrige hacia el otro lado",
          f"{d_recto.direccion} -> {d_torcido.direccion}")

    n2 = nav.Navegador(dict(cfg, usar_yaw=False), lim)
    n2.paso(p, yaw=0.0)
    a = n2.paso(p, yaw=0.0)
    b = n2.paso(p, yaw=-30.0)
    check(a.direccion == b.direccion, "con usar_yaw=False el giroscopio se ignora")

    # sin IMU (yaw=None) todo sigue funcionando
    n3 = nav.Navegador(cfg, lim)
    d = n3.paso(p, yaw=None)
    check(d.estado == nav.RECTO, "sin giroscopio la navegacion funciona igual")

    # la correccion esta acotada
    n4 = nav.Navegador(dict(cfg, yaw_max=20.0), lim)
    n4.paso(p, yaw=0.0)
    d = n4.paso(p, yaw=-170.0)
    check(abs(d.direccion) <= lim["dir_max"], "la correccion por yaw no desborda",
          d.direccion)


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
    ruta = "/tmp/pista_test.png"
    cv2.imwrite(ruta, img)

    cfg = robot_config.cargar("/tmp/robot_test.json")
    cfg["red"]["puerto_http"] = 8391
    cfg["navegacion"]["min_recto_ms"] = 0
    perfil = cc.obtener(cc.cargar("/tmp/colors_test.json"))

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
    check("pasillo" in est["decision"]["metricas"], "e incluye las metricas del muro")

    get("/api/cmd?armar=1")
    time.sleep(0.3)
    check(r.armado, "la web puede armar")
    est = json.loads(get("/api/estado"))
    check(est["decision"]["vel"] != 0, "y entonces si decide velocidad",
          est["decision"])

    get("/api/cmd?vmax=77")
    check(r.cfg["limites"]["vmax"] == 77 and r.enlace._vmax == 77,
          "cambiar vmax desde la web llega hasta el enlace", r.enlace._vmax)

    get("/api/cmd?estrategia=pared")
    check(r.cfg["navegacion"]["estrategia"] == "pared", "cambia de estrategia")
    get("/api/cmd?estrategia=centrado&kp=123")
    check(r.cfg["navegacion"]["kp"] == 123.0, "cambia ganancias en caliente")

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
    r.mando_manual(60, 0)
    time.sleep(0.3)
    check(r.decision.vel == 60, "en manual obedece el mando")

    cerca = pista([(0, 470), (639, 470)])
    cv2.imwrite("/tmp/pista_muro.png", cerca)
    r._imagen_fija = cv2.imread("/tmp/pista_muro.png")
    time.sleep(0.4)
    check(r.decision.vel <= 0, "pero NO te deja empotrarte aunque lo pidas",
          f"vel={r.decision.vel} motivo={r.decision.motivo}")

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
    ruta = Path("/tmp/robot_cfg_test.json")
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
    check("navegacion" in cfg2 and "kp" in cfg2["navegacion"],
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
if __name__ == "__main__":
    print(f"OpenCV {cv2.__version__} · Python {sys.version.split()[0]}")
    for f in (test_protocolo_cruzado, test_lector_robusto, test_perfil,
              test_estrategias, test_yaw, test_enlace, test_robot_y_web,
              test_config):
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
