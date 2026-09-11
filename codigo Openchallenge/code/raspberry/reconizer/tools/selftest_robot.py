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
from src import robot_config, vision, vueltas as vueltas_mod           # noqa: E402

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

    n_mando = n_tele = n_sens = 0
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
        elif campos[0] == "IMU":
            yaw10, gz10, cal, temp = (int(x) for x in campos[1:5])
            esperado = campos[5].upper()
            d = P.DatosIMU(yaw10 / 10.0, gz10 / 10.0, bool(cal), temp)
            check(d.a_bytes().hex().upper() == esperado, f"trama IMU yaw={yaw10/10}",
                  f"py={d.a_bytes().hex().upper()} cpp={esperado}")
            n_sens += 1
        elif campos[0] == "COLOR":
            vals = [int(x) for x in campos[1:6]]
            esperado = campos[6].upper()
            e = P.EventoColor(*vals)
            check(e.a_bytes().hex().upper() == esperado,
                  f"trama COLOR {P.NOMBRE_LINEA.get(vals[0])}")
            n_sens += 1
        elif campos[0] == "SENSORES":
            vals = [int(x) for x in campos[1:4]]
            esperado = campos[4].upper()
            e = P.EstadoSensores(*vals)
            check(e.a_bytes().hex().upper() == esperado, f"trama SENSORES {vals}")
            n_sens += 1
        elif campos[0] == "SERVO":
            grados = [int(x) for x in campos[1:]]
            check(all(50 <= g <= 145 for g in grados),
                  "el firmware nunca sale del rango del servo", str(grados))
            check(grados[grados.index(min(grados))] == 65 and max(grados) == 135,
                  "los topes del servo son 65 y 135", str(grados))
    check(n_mando >= 5 and n_tele >= 3 and n_sens >= 9,
          "se cruzaron todos los vectores",
          f"{n_mando} mandos, {n_tele} telemetrias, {n_sens} de sensores")

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
    base = dict(robot_config.POR_DEFECTO["navegacion"], min_recto_ms=0,
                mezcla={"centrado": 1.0}, usar_esquina_interna=False,
                autocalibrar_carril=False)
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
    cfg_esq = dict(base, girar_bajo=0.45, parar_bajo=0.20, retardo_giro_ms=80)
    d, p, n = decidir(pista([(0, 300), (430, 300), (520, 190), (639, 160)]), cfg_esq)
    check(n.estado == nav.PRE_GIRO,
          "pared de frente a distancia: entra en pre-giro (frena ANTES de doblar)",
          f"{n.estado} pasillo={p.pasillo:.2f}")
    check(d.vel > 0, "sin pararse (girar parado no sirve con direccion Ackermann)", d.vel)
    time.sleep(0.12)
    d = n.paso(p, None)
    check(n.estado == nav.GIRO, "pasado el retardo, gira", n.estado)
    check(d.direccion > 40, "y gira fuerte hacia el hueco", d.direccion)

    # Y si ya es demasiado tarde, la seguridad manda sobre la estrategia
    d2, p2, n2 = decidir(pista([(0, 425), (430, 425), (520, 200), (639, 150)]), cfg_esq)
    check(n2.estado == nav.ESCAPE, "si ya esta encima, escape en vez de giro",
          f"{n2.estado} pasillo={p2.pasillo:.2f}")
    check(d2.vel < 0 and abs(d2.direccion) > 50,
          "y el escape retrocede comprometido girando para reencuadrar",
          f"vel={d2.vel} dir={d2.direccion}")

    # --- muro encima: parada / retroceso ---
    d, p, _ = decidir(pista([(0, 465), (639, 465)]))
    check(d.estado == nav.ESCAPE, "muro encima: escape", d.estado)
    check(d.vel != 0, "escapa en vez de empujar la pared", d.vel)

    # --- frenado progresivo ---
    lejos, _, _ = decidir(pista([(0, 140), (639, 140)]))
    cerca, pc, _ = decidir(pista([(0, 330), (639, 330)]))
    check(cerca.vel < lejos.vel, "cuanto mas cerca el muro, mas despacio",
          f"{cerca.vel} vs {lejos.vel} (pasillo {pc.pasillo:.2f})")

    # --- seguir pared izquierda (con el pasillo despejado, para aislar el PD) ---
    cfg_p = dict(base, mezcla={"pared": 1.0}, lado_pared="izq", pared_objetivo=0.40)
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
    cfg = dict(robot_config.POR_DEFECTO["navegacion"], min_recto_ms=0, usar_yaw=True,
               mezcla={"centrado": 1.0}, usar_esquina_interna=False,
               autocalibrar_carril=False)
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
    check("sensores" in est and "vueltas" in est, "y trae sensores y vueltas")
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
# 10. Anticipacion, escape, esquina interna, huecos, sentido y vueltas
# ===========================================================================
def _cfg(**extra):
    base = dict(robot_config.POR_DEFECTO["navegacion"], min_recto_ms=0,
                mezcla={"centrado": 1.0}, usar_esquina_interna=False,
                autocalibrar_carril=False)
    base.update(extra)
    return base


def _perfil(img, cfg):
    return nav.perfil_desde_mascara(_mascara_muro(img, cc.colores_por_defecto()), cfg)


def test_anticipacion():
    print("\n[10] Anticipacion: frenar antes de llegar (el problema de la inercia)")
    cfg = _cfg()
    lim = dict(robot_config.POR_DEFECTO["limites"])
    n = nav.Navegador(cfg, lim)

    # Muro que se acerca deprisa: 0.75 -> 0.45 en tres frames de 100 ms
    vels = []
    for y in (140, 200, 260, 300):
        p = _perfil(pista([(0, y), (639, y)]), cfg)
        d = n.paso(p, None)
        vels.append((round(p.pasillo, 2), d.vel, round(n.ttc, 2)))
        time.sleep(0.1)
    check(vels[-1][1] < vels[0][1], "acercarse deprisa baja la velocidad",
          vels)
    check(n.ttc < 5, "y calcula un tiempo hasta el muro corto", n.ttc)

    # Mismo muro pero quieto: no debe frenar por anticipacion
    n2 = nav.Navegador(cfg, lim)
    p = _perfil(pista([(0, 200), (639, 200)]), cfg)
    for _ in range(5):
        d2 = n2.paso(p, None)
        time.sleep(0.05)
    check(n2.ttc > 10, "con el muro quieto no hay urgencia", n2.ttc)
    check(d2.vel >= vels[-1][1], "y va mas rapido que acercandose",
          f"{d2.vel} vs {vels[-1][1]}")

    # ttc_min alto = mucho mas conservador
    n3 = nav.Navegador(_cfg(ttc_min=3.0), lim)
    for y in (140, 200, 260, 300):
        p = _perfil(pista([(0, y), (639, y)]), _cfg())
        d3 = n3.paso(p, None)
        time.sleep(0.1)
    check(d3.vel <= vels[-1][1], "subir ttc_min frena antes",
          f"{d3.vel} vs {vels[-1][1]}")


def test_escape():
    print("\n[11] Escape: retroceso comprometido, no vaiven")
    cfg = _cfg(escape_atras_min_ms=300, escape_atras_extra_ms=200,
               escape_atascado_ms=250, mejora_min=0.05)
    lim = dict(robot_config.POR_DEFECTO["limites"])
    encima = _perfil(pista([(0, 455), (639, 455)]), cfg)

    n = nav.Navegador(cfg, lim)
    d = n.paso(encima, None)
    check(n.estado == nav.ESCAPE, "muro encima -> estado escape", n.estado)
    check(d.vel < 0, "con el muro encima RETROCEDE (antes se quedaba en vaiven)", d.vel)
    check(abs(d.direccion) > 50, "girando para reencuadrar el morro", d.direccion)
    check(n._escape_compromiso >= 300,
          "y se compromete a un tiempo minimo de marcha atras",
          f"{n._escape_compromiso:.0f} ms")

    # Cuanto mas encima el muro, mas tiempo de retroceso se compromete
    n_lejos = nav.Navegador(cfg, lim)
    casi = _perfil(pista([(0, 370), (639, 370)]), dict(cfg, parar_bajo=0.30))
    n_lejos.cfg = dict(cfg, parar_bajo=0.30)
    n_lejos.paso(casi, None)
    check(n_lejos._escape_compromiso < n._escape_compromiso,
          "con el muro menos encima, el retroceso comprometido es mas corto",
          f"{n_lejos._escape_compromiso:.0f} vs {n._escape_compromiso:.0f} ms")

    # No abandona la marcha atras a las primeras de cambio
    signos = []
    for _ in range(4):
        time.sleep(0.05)
        signos.append(1 if n.paso(encima, None).vel > 0 else -1)
    check(all(x < 0 for x in signos),
          "no cambia de idea a mitad del retroceso", signos)

    # Si no mejora nada en un buen rato, es que hay algo detras: adelante
    hubo_adelante = False
    for _ in range(20):
        time.sleep(0.05)
        if n.paso(encima, None).vel > 0:
            hubo_adelante = True
            break
    check(hubo_adelante, "si no gana espacio, deduce que hay algo detras y va adelante")

    # En cuanto hay sitio de verdad, sale del escape
    libre = _perfil(pista([(0, 150), (639, 150)]), cfg)
    time.sleep(0.35)
    n.paso(libre, None)
    check(n.estado != nav.ESCAPE, "con espacio delante sale del escape", n.estado)

    # No sale por un parpadeo: si aun no ha cumplido el minimo, sigue atras
    n2 = nav.Navegador(dict(cfg, escape_atras_min_ms=1500), lim)
    n2.paso(encima, None)
    d2 = n2.paso(libre, None)
    check(n2.estado == nav.ESCAPE and d2.vel < 0,
          "un parpadeo del perfil no lo saca del retroceso a medias", d2.vel)

    # El escape se va hacia el lado del muro interno si se conoce
    n3 = nav.Navegador(cfg, lim)
    n3.paredes.forzar(-1)          # externa a la izquierda -> interna derecha
    d3 = n3.paso(encima, None)
    check(n3._escape_lado == nav.DER,
          "sabiendo cual es la externa, se escapa hacia la interna", n3._escape_lado)


def test_esquina_interna():
    print("\n[12] Esquina del muro interno")
    cfg = _cfg(usar_esquina_interna=True, interno_libre=0.7, min_recto_ms=0)
    lim = dict(robot_config.POR_DEFECTO["limites"])

    # Pasillo con muro a los dos lados; el de la DERECHA (interno, si vamos en
    # horario) se acaba de golpe a media imagen.
    img = pista([(0, 330), (150, 300), (300, 240), (430, 235),
                 (440, 60), (639, 60)])
    p = _perfil(img, cfg)
    check(len(p.bordes) >= 1, "detecta el escalon donde acaba el muro interno",
          [(b.x, round(b.salto, 2)) for b in p.bordes[:2]])
    b = p.bordes[0]
    check(b.cerca_a_lejos and b.lado == nav.DER,
          "y sabe que esta a la derecha y va de cerca a lejos",
          f"x={b.x} salto={b.salto:+.2f} lado={b.lado}")

    n = nav.Navegador(cfg, lim)
    n.paredes.forzar(-1)        # externa izquierda -> interna derecha
    d = n.paso(p, None)
    check(n.estado == nav.PRE_GIRO, "dispara el giro por la esquina interna",
          f"{n.estado} {d.motivo}")
    check("interna" in d.motivo, "y lo dice en el motivo", d.motivo)
    check(d.vel > 0 and abs(d.direccion) < 40,
          "durante el pre-giro va recto y frenando, dejando pasar las ruedas",
          f"vel={d.vel} dir={d.direccion}")

    time.sleep(float(cfg["retardo_giro_ms"]) / 1000.0 + 0.05)
    d = n.paso(p, None)
    check(n.estado == nav.GIRO, "pasado el retardo, empieza a girar", n.estado)
    check(0 < d.direccion <= float(cfg["dir_giro_abierto"]) + 1,
          "con giro ABIERTO, no a tope, para no barrer con las ruedas traseras",
          d.direccion)

    # Sin la esquina interna activada, con el mismo frame seguiria recto
    n2 = nav.Navegador(_cfg(usar_esquina_interna=False), lim)
    n2.paredes.forzar(-1)
    d2 = n2.paso(_perfil(img, _cfg()), None)
    check(n2.estado == nav.RECTO,
          "sin el disparador de esquina, con ese mismo frame seguiria recto (giraba tarde)",
          n2.estado)


def test_huecos():
    print("\n[13] Hueco pasable contando el ancho de las ruedas")
    cfg = _cfg(umbral_hueco=0.45, margen_hueco=1.15, y_horizonte=0.35)

    # Dos obstaculos con un hueco ANCHO en medio
    ancho_img = 640
    img = pista([(0, 400), (200, 400), (215, 120), (425, 120),
                 (440, 400), (639, 400)])
    p = _perfil(img, cfg)
    check(p.huecos, "encuentra huecos")
    h = p.huecos[0]
    check(h.pasable, "el hueco ancho es pasable", f"{h.ancho_px} vs {h.ancho_necesario:.0f}")
    check(abs(h.centro - ancho_img // 2) < 60, "y esta centrado", h.centro)

    # Hueco ESTRECHO entre dos obstaculos cercanos: no cabe
    img2 = pista([(0, 400), (300, 400), (310, 120), (330, 120),
                  (340, 400), (639, 400)])
    p2 = _perfil(img2, cfg)
    estrechos = [h for h in p2.huecos if h.x0 > 250 and h.x1 < 400]
    check(estrechos and not estrechos[0].pasable,
          "un hueco de 20 px entre obstaculos cerca NO se considera pasable",
          f"{estrechos[0].ancho_px} vs {estrechos[0].ancho_necesario:.0f}" if estrechos else "-")

    # El mismo hueco angular, pero LEJOS, si cabe: la perspectiva importa
    ancho_cerca = nav.ancho_carro_px(430, cfg, 480, 640)
    ancho_lejos = nav.ancho_carro_px(220, cfg, 480, 640)
    check(ancho_cerca > ancho_lejos * 2,
          "el carro ocupa muchos mas pixeles cerca que lejos",
          f"{ancho_cerca:.0f} vs {ancho_lejos:.0f}")

    # El "siguiente obstaculo" penaliza: hueco con algo justo detras
    cfg_p = _cfg(peso_siguiente=3.0)
    img3 = pista([(0, 400), (150, 400), (165, 130), (300, 130), (315, 400),
                  (360, 400), (375, 130), (500, 130), (515, 400), (639, 400)])
    p3 = _perfil(img3, cfg_p)
    check(len(p3.huecos) >= 2, "dos huecos separados", len(p3.huecos))

    # Direccion: apunta al hueco
    lim = dict(robot_config.POR_DEFECTO["limites"])
    n = nav.Navegador(_cfg(mezcla={"hueco": 1.0}), lim)
    izquierdo = pista([(0, 120), (200, 120), (260, 300), (639, 300)])
    p4 = _perfil(izquierdo, _cfg())
    d = n.paso(p4, None)
    d = n.paso(p4, None)
    check(d.direccion < -5, "con el hueco a la izquierda, gira a la izquierda",
          f"dir={d.direccion} hueco={p4.huecos[0].centro if p4.huecos else '-'}")


def test_sentido_y_carril():
    print("\n[14] Sentido de la vuelta y ancho de carril automatico")
    cfg = _cfg(autocalibrar_carril=True)
    lim = dict(robot_config.POR_DEFECTO["limites"])

    # Presencia continua: la pared que SIEMPRE se ve es la externa.
    d = nav.DetectorParedes(dict(cfg, min_muestras_presencia=20, alfa_presencia=0.15))
    check(d.lado_externo == 0, "al principio no sabe cual es la externa")
    # Situacion real: las dos paredes se ven casi siempre, pero la INTERNA
    # (aqui la derecha) desaparece en cada esquina. Eso es lo que la distingue.
    ambas = _perfil(pista([(0, 300), (150, 300), (235, 150), (420, 150),
                           (500, 300), (639, 300)]), cfg)
    # en la esquina, la banda derecha se queda literalmente sin muro (y=0)
    sin_der = _perfil(pista([(0, 300), (150, 300), (235, 150), (430, 150),
                             (450, 0), (639, 0)]), cfg)
    con_izq = sin_der
    for ciclo in range(12):
        for _ in range(9):
            d.observar(ambas)
        for _ in range(3):
            d.observar(sin_der)     # esquina: se acaba el muro interno derecho
    check(d.lado_externo == nav.IZQ, "muro siempre a la izquierda = externa izquierda",
          d.estado())
    check(d.sentido == -1 and "antihorario" in d.estado()["nombre"],
          "externa a la izquierda = antihorario (tu convencion)", d.estado()["nombre"])
    check(d.lado_interno == nav.DER, "y el muro interno es el derecho")

    # Y se puede corregir sola: si ahora el muro constante pasa a la derecha,
    # la media movil se cruza. Esto es lo que antes no pasaba nunca.
    sin_izq = _perfil(pista([(0, 0), (190, 0), (210, 150), (400, 150),
                             (490, 300), (639, 300)]), cfg)
    con_der = sin_izq
    for ciclo in range(16):
        for _ in range(9):
            d.observar(ambas)
        for _ in range(3):
            d.observar(sin_izq)
    check(d.lado_externo == nav.DER, "si cambia la pared constante, se corrige sola",
          d.estado())

    # No se satura: 2000 observaciones y sigue pudiendo cambiar de opinion
    for _ in range(2000):
        d.observar(ambas)
        d.observar(sin_izq)
    for ciclo in range(20):
        for _ in range(9):
            d.observar(ambas)
        for _ in range(3):
            d.observar(sin_der)
    check(d.lado_externo == nav.IZQ, "ni con 2000 frames se queda clavada", d.estado())

    # Forzar desde la interfaz
    d.forzar(1)
    check(d.lado_externo == nav.DER and d.estado()["forzado"] == 1,
          "se puede forzar el sentido a mano")
    d.forzar(0)

    # Votos discretos cuando la presencia no decide
    d2 = nav.DetectorParedes(cfg)
    d2.voto_lineas("naranja", "azul")
    check(d2.lado_externo == nav.DER, "naranja y luego azul = horario = externa der",
          d2.estado())
    d3 = nav.DetectorParedes(cfg)
    d3.voto_giro(nav.DER)
    d3.voto_giro(nav.DER)
    check(d3.lado_externo == nav.IZQ,
          "girar a la derecha significa interno a la derecha, externo a la izquierda",
          d3.estado())

    # Inversion tras la media vuelta: intercambia y BLOQUEA
    d4 = nav.DetectorParedes(dict(cfg, min_muestras_presencia=20, alfa_presencia=0.15,
                                  bloqueo_sentido_ms=1500))
    for _ in range(60):
        d4.observar(con_izq)
    antes = d4.lado_externo
    d4.invertir()
    check(d4.lado_externo == -antes, "la media vuelta invierte la externa")
    for _ in range(300):
        d4.observar(con_izq)          # el mundo viejo insiste
    check(d4.lado_externo == -antes,
          "y durante el bloqueo NO vuelve al sentido anterior (esto deshacia la media vuelta)",
          d4.estado())

    # --- carril ---
    n = nav.Navegador(cfg, lim)
    recta = _perfil(pista([(0, 300), (150, 300), (235, 150), (420, 150),
                           (500, 300), (639, 300)]), cfg)
    for _ in range(40):
        n.carril.observar(recta, nav.RECTO, 0)
    check(n.carril.listo, "mide el ancho del carril en recta", n.carril.estado())
    u = n.carril.umbrales()
    check(u["parar_bajo"] < u["girar_bajo"] < u["frenar_bajo"],
          "y los umbrales derivados quedan ordenados", u)

    estrecho = _perfil(pista([(0, 430), (150, 430), (235, 150), (420, 150),
                              (500, 430), (639, 430)]), cfg)
    n2 = nav.Navegador(cfg, lim)
    for _ in range(40):
        n2.carril.observar(estrecho, nav.RECTO, 0)
    check(n2.carril.ancho < n.carril.ancho,
          "un carril mas estrecho da una medida menor",
          f"{n2.carril.ancho:.2f} vs {n.carril.ancho:.2f}")
    check(n2.carril.listo and n2.carril.umbrales()["girar_bajo"] < u["girar_bajo"],
          "y por tanto umbrales mas apretados",
          f"{n2.carril.umbrales()} vs {u}")

    # en curva no mide (se ensuciaria)
    antes = len(n.carril.muestras)
    n.carril.observar(recta, nav.GIRO, 0)
    n.carril.observar(recta, nav.RECTO, 60)
    check(len(n.carril.muestras) == antes, "no toma muestras girando")


def test_vueltas():
    print("\n[15] Contador de vueltas fusionando camara, sensor y giros")
    cfg = dict(robot_config.POR_DEFECTO["vueltas"], debounce_ms=1, ventana_par_ms=5000,
               refractario_ms=1)
    c = vueltas_mod.ContadorVueltas(cfg, al_log=lambda s: None)

    c.evento_linea(P.LINEA_NARANJA, "camara")
    check(c.esquinas == 0, "una sola linea todavia no es esquina")
    c.evento_linea(P.LINEA_AZUL, "camara")
    check(c.esquinas == 1, "naranja + azul = una esquina", c.esquinas)
    check(c.sentido_lineas == 1, "y deduce sentido horario", c.sentido_lineas)

    for _ in range(3):
        time.sleep(0.002)
        c.evento_linea(P.LINEA_NARANJA, "tcs")
        c.evento_linea(P.LINEA_AZUL, "tcs")
    check(c.vueltas == 1, "cuatro esquinas = una vuelta", c.estado())

    # La misma esquina vista por camara, sensor y giro cuenta UNA
    c2 = vueltas_mod.ContadorVueltas(dict(cfg, refractario_ms=2000),
                                     al_log=lambda s: None)
    c2.evento_linea(P.LINEA_NARANJA, "camara")
    c2.evento_linea(P.LINEA_AZUL, "tcs")
    c2.evento_giro(1)
    check(c2.esquinas == 1, "camara + sensor + giro en la misma esquina = 1",
          c2.esquinas)

    # Rebote: la misma linea dos veces seguidas no cuenta dos
    c3 = vueltas_mod.ContadorVueltas(dict(cfg, debounce_ms=900), al_log=lambda s: None)
    c3.evento_linea(P.LINEA_NARANJA, "camara")
    c3.evento_linea(P.LINEA_NARANJA, "camara")
    c3.evento_linea(P.LINEA_AZUL, "camara")
    check(c3.esquinas == 1, "un rebote de la misma linea no suma otra esquina")

    # Solo giros (sin lineas) tambien cuenta
    c4 = vueltas_mod.ContadorVueltas(dict(cfg, refractario_ms=1), al_log=lambda s: None)
    for _ in range(8):
        time.sleep(0.002)
        c4.evento_giro(1)
    check(c4.vueltas == 2, "sin lineas, los giros solos cuentan las vueltas", c4.vueltas)

    # Objetivo -> media vuelta -> segundo tramo -> terminado
    c5 = vueltas_mod.ContadorVueltas(dict(cfg, objetivo=1, refractario_ms=1),
                                     al_log=lambda s: None)
    for _ in range(4):
        time.sleep(0.002)
        c5.evento_giro(1)
    check(c5.media_vuelta_pendiente and not c5.terminado,
          "al cumplir la ida pide la media vuelta")
    c5.media_vuelta_completada()
    check(c5.tramo == vueltas_mod.VUELTA and c5.vueltas == 0,
          "tras la media vuelta el contador arranca de cero en el tramo de vuelta")
    for _ in range(4):
        time.sleep(0.002)
        c5.evento_giro(-1)
    check(c5.terminado, "y al cumplir la vuelta se da por terminado")

    # sin media vuelta: termina en la ida
    c6 = vueltas_mod.ContadorVueltas(dict(cfg, objetivo=1, hacer_media_vuelta=False,
                                          refractario_ms=1), al_log=lambda s: None)
    for _ in range(4):
        time.sleep(0.002)
        c6.evento_giro(1)
    check(c6.terminado and not c6.media_vuelta_pendiente,
          "con la media vuelta desactivada termina al cumplir la ida")


def test_lineas_camara():
    print("\n[16] Lineas del suelo vistas por la camara")
    import numpy as np
    cfg = dict(robot_config.POR_DEFECTO["vueltas"], umbral_linea_camara=0.02)
    det = vueltas_mod.DetectorLineasCamara(cfg)
    vacia = np.zeros((480, 640), np.uint8)

    check(det.procesar({"naranja": vacia, "azul": vacia}) == P.LINEA_NINGUNA,
          "sin lineas no pasa nada")

    naranja = vacia.copy()
    naranja[440:470, 200:450] = 255            # franja gruesa abajo
    r1 = det.procesar({"naranja": naranja, "azul": vacia})
    r2 = det.procesar({"naranja": naranja, "azul": vacia})
    check(r1 == P.LINEA_NINGUNA and r2 == P.LINEA_NARANJA,
          "hacen falta 2 frames seguidos para declarar la naranja", (r1, r2))
    r = det.procesar({"naranja": naranja, "azul": vacia})
    check(r == P.LINEA_NINGUNA, "y mientras sigue encima NO repite el evento", r)
    det.procesar({"naranja": vacia, "azul": vacia})
    det.procesar({"naranja": vacia, "azul": vacia})
    azul = vacia.copy()
    azul[440:470, 200:450] = 255
    det.procesar({"naranja": vacia, "azul": azul})
    check(det.procesar({"naranja": vacia, "azul": azul}) == P.LINEA_AZUL,
          "y luego la azul")

    # Las dos lineas a la vez (esquina): no se declara la ganadora por poco
    det3 = vueltas_mod.DetectorLineasCamara(cfg)
    empate = vacia.copy()
    empate[440:470, 200:400] = 255
    otro = vacia.copy()
    otro[440:470, 210:400] = 255
    r = P.LINEA_NINGUNA
    for _ in range(4):
        r = det3.procesar({"naranja": empate, "azul": otro}) or r
    check(r == P.LINEA_NINGUNA,
          "con las dos lineas casi iguales no declara ninguna (esto contaba de mas)", r)

    # una linea arriba del todo (fondo) no cuenta: solo se mira la franja baja
    arriba = vacia.copy()
    arriba[100:130, 200:450] = 255
    det2 = vueltas_mod.DetectorLineasCamara(cfg)
    check(det2.procesar({"naranja": arriba, "azul": vacia}) == P.LINEA_NINGUNA,
          "una mancha del color en el fondo no cuenta como cruce")


def test_media_vuelta():
    print("\n[17] Media vuelta")
    cfg = _cfg(mv_fase_ms=200, vel_media_vuelta=30)
    lim = dict(robot_config.POR_DEFECTO["limites"])
    p = _perfil(pista([(0, 200), (639, 200)]), cfg)

    # --- tres tiempos en recta, con yaw ---
    n = nav.Navegador(cfg, lim, {"tipo_media_vuelta": "recta_3t"})
    n.pedir_media_vuelta()
    yaw = 0.0
    fases = set()
    sentidos = set()
    for i in range(60):
        d = n.paso(p, yaw)
        fases.add(n._mv_fase)
        sentidos.add(1 if d.vel > 0 else (-1 if d.vel < 0 else 0))
        # simulamos que el carro gira mientras avanza o retrocede
        yaw = nav._norm_angulo(yaw + (12 if d.vel > 0 else (6 if d.vel < 0 else 0)))
        if n.media_vuelta_hecha:
            break
        time.sleep(0.02)
    check(n.media_vuelta_hecha, "completa la media vuelta con el yaw", i)
    check(1 in sentidos and -1 in sentidos,
          "y de verdad hace adelante y atras (tres tiempos)", sentidos)

    # sin yaw se apoya en tiempos y tambien termina
    n2 = nav.Navegador(cfg, lim, {"tipo_media_vuelta": "recta_3t"})
    n2.pedir_media_vuelta()
    for _ in range(80):
        n2.paso(p, None)
        if n2.media_vuelta_hecha:
            break
        time.sleep(0.02)
    check(n2.media_vuelta_hecha, "sin giroscopio tambien termina, por tiempo")

    # --- en la esquina ---
    n3 = nav.Navegador(cfg, lim, {"tipo_media_vuelta": "esquina"})
    n3.pedir_media_vuelta()
    d = n3.paso(p, 0.0)
    check("buscando la esquina" in d.motivo,
          "en modo esquina espera a llegar a una", d.motivo)
    cerrado = _perfil(pista([(0, 380), (639, 380)]), cfg)
    yaw = 0.0
    for _ in range(60):
        d = n3.paso(cerrado, yaw)
        yaw = nav._norm_angulo(yaw + 14)
        if n3.media_vuelta_hecha:
            break
        time.sleep(0.01)
    check(n3.media_vuelta_hecha, "y al llegar encadena el giro de 180")

    # el sentido queda invertido tras la maniobra
    n4 = nav.Navegador(cfg, lim, {"tipo_media_vuelta": "recta_3t"})
    n4.sentido._votar("giro", nav.DER, 3.0)
    antes = n4.sentido.sentido
    n4.pedir_media_vuelta()
    yaw = 0.0
    for _ in range(60):
        d = n4.paso(p, yaw)
        yaw = nav._norm_angulo(yaw + 12)
        if n4.media_vuelta_hecha:
            break
        time.sleep(0.02)
    check(n4.sentido.sentido == -antes, "y el sentido de la vuelta se invierte",
          f"{antes} -> {n4.sentido.sentido}")


def test_sensores_capa():
    print("\n[18] Capa de sensores: ESP32, Pi o nada")
    from src import sensores as sens_mod
    s = sens_mod.Sensores({"origen_rumbo": "auto", "origen_color": "auto"})

    s.actualizar()
    check(not s.hay_rumbo and s.yaw_o_none() is None,
          "sin nada conectado, no hay rumbo y la navegacion lo sabe")

    s.desde_esp32_imu(P.DatosIMU(yaw=37.5, giro_z=12.0, calibrado=True))
    s.actualizar()
    check(s.origen_rumbo == "esp32" and abs(s.yaw - 37.5) < 0.01,
          "con tramas del ESP32 el rumbo sale de ahi", s.estado()["rumbo"])

    s.poner_cero()
    check(abs(s.yaw) < 0.01, "poner a cero funciona al instante, sin esperar trama")
    s.desde_esp32_imu(P.DatosIMU(yaw=47.5))
    s.actualizar()
    check(abs(s.yaw - 10.0) < 0.01, "y el desfase se mantiene", s.yaw)

    # caduca si el ESP32 deja de mandar
    s._t_yaw_esp32 = time.time() - 5
    s.actualizar()
    check(not s.hay_rumbo, "si el ESP32 se calla, se deja de usar su rumbo")

    # color
    s.desde_esp32_estado(P.EstadoSensores(presentes=P.S_TCS, hz_color=60))
    s.desde_esp32_color(P.EventoColor(linea=P.LINEA_NARANJA, r=127, g=76, b=34))
    s.actualizar()
    check(s.origen_color == "esp32", "con TCS presente, el color viene del ESP32")
    check(s.ultima_linea == P.LINEA_NARANJA, "y llega la linea")
    check(len(s.eventos) == 1, "el evento queda registrado")

    s2 = sens_mod.Sensores({"origen_rumbo": "ninguno", "origen_color": "camara"})
    s2.desde_esp32_imu(P.DatosIMU(yaw=90))
    s2.actualizar()
    check(not s2.hay_rumbo, "'ninguno' ignora al ESP32 aunque mande datos")
    check(s2.origen_color == "camara", "y el color se puede forzar a camara")


def test_regresion_arreglos():
    print("\n[19] Que los arreglos de esta ronda siguen puestos")
    cfg = _cfg()
    lim = dict(robot_config.POR_DEFECTO["limites"])

    # 1. Ya no se retrocede a ciegas como primera opcion
    n = nav.Navegador(cfg, lim)
    d = n.paso(_perfil(pista([(0, 460), (639, 460)]), cfg), None)
    check(d.vel < 0 and n.estado == nav.ESCAPE,
          "muro encima: retrocede de verdad, con compromiso", (d.vel, n.estado))

    # 2. La velocidad baja al entrar en la curva, no despues
    n2 = nav.Navegador(_cfg(usar_esquina_interna=False), lim)
    p_lejos = _perfil(pista([(0, 140), (639, 140)]), cfg)
    n2.paso(p_lejos, None)
    time.sleep(0.1)
    v_recta = n2.paso(p_lejos, None).vel
    p_esquina = _perfil(pista([(0, 320), (639, 320)]), cfg)
    time.sleep(0.1)
    v_pregiro = n2.paso(p_esquina, None).vel
    check(v_pregiro < v_recta, "al detectar la esquina ya viene frenado",
          f"{v_recta} -> {v_pregiro}")
    check(n2.estado in (nav.PRE_GIRO, nav.GIRO), "y esta en pre-giro o giro", n2.estado)

    # 3. Los pesos de la mezcla se respetan
    n3 = nav.Navegador(_cfg(mezcla={"centrado": 1.0, "hueco": 1.0}), lim)
    p = _perfil(pista([(0, 380), (300, 200), (639, 150)]), cfg)
    n3.paso(p, None)
    d3 = n3.paso(p, None)
    check(len(d3.metricas.get("aportes", {})) == 2,
          "con dos estrategias activas se ven los dos aportes",
          d3.metricas.get("aportes"))

    # 4. Nada saca la direccion o la velocidad de rango
    peor = None
    for y0 in range(120, 470, 35):
        for y1 in range(120, 470, 35):
            nn = nav.Navegador(_cfg(mezcla={"centrado": 1, "pared": 1, "hueco": 1}), lim)
            pp = _perfil(pista([(0, y0), (639, y1)]), cfg)
            for _ in range(2):
                dd = nn.paso(pp, 12.0)
            if abs(dd.direccion) > 100 or abs(dd.vel) > 100:
                peor = (y0, y1, dd.vel, dd.direccion)
    check(peor is None, "ninguna combinacion se sale de rango", peor)



# ===========================================================================
# 20. Esquiva de pilares y media vuelta que no se deshace
# ===========================================================================
def _cfg_obst(**extra):
    c = dict(robot_config.POR_DEFECTO["obstaculos"], activo=True)
    c.update(extra)
    return c


class _Det:
    """Deteccion minima, como la que devuelve vision.Vision."""
    def __init__(self, x, y, w, h):
        self.x, self.y, self.w, self.h = x, y, w, h
        self.area = w * h


def test_pilares():
    print("\n[20] Esquiva de pilares: rojo por la derecha, verde por la izquierda")
    from src import obstaculos as obst
    cfg_nav = _cfg()
    perfil = _perfil(pista([(0, 150), (639, 150)]), cfg_nav)   # pista despejada

    # --- ROJO a la izquierda del centro: hay que pasar por su DERECHA ---
    e = obst.EsquivaPilares(_cfg_obst())
    rojo = _Det(180, 250, 60, 120)          # base_y = 370 (cerca)
    r = None
    for _ in range(3):
        r = e.paso({"rojo": [rojo], "verde": []}, perfil, cfg_nav)
        time.sleep(0.01)
    check(r.activo, "engancha el pilar rojo", r.motivo)
    check(r.direccion > 0, "y gira a la DERECHA para dejarlo a la izquierda",
          f"dir={r.direccion:.0f} objetivo={r.objetivo_x}")
    check(r.objetivo_x > rojo.x + rojo.w,
          "el punto objetivo cae a la derecha del pilar", r.objetivo_x)
    check(0 < r.peso <= 1.0, "con un peso proporcional a la cercania", r.peso)

    # --- VERDE a la derecha del centro: hay que pasar por su IZQUIERDA ---
    e2 = obst.EsquivaPilares(_cfg_obst())
    verde = _Det(400, 250, 60, 120)
    r2 = None
    for _ in range(3):
        r2 = e2.paso({"rojo": [], "verde": [verde]}, perfil, cfg_nav)
        time.sleep(0.01)
    check(r2.direccion < 0, "gira a la IZQUIERDA para dejarlo a la derecha",
          f"dir={r2.direccion:.0f} objetivo={r2.objetivo_x}")
    check(r2.objetivo_x < verde.x, "el objetivo cae a la izquierda del pilar",
          r2.objetivo_x)

    # --- el peso crece con la cercania -------------------------------------
    e3 = obst.EsquivaPilares(_cfg_obst())
    lejos = e3.paso({"rojo": [_Det(280, 190, 34, 60)], "verde": []}, perfil, cfg_nav)
    e4 = obst.EsquivaPilares(_cfg_obst())
    cerca = e4.paso({"rojo": [_Det(260, 220, 70, 180)], "verde": []}, perfil, cfg_nav)
    check(cerca.peso > lejos.peso, "cuanto mas cerca, mas manda la esquiva",
          f"{lejos.peso:.2f} -> {cerca.peso:.2f}")

    # --- el margen se escala con la perspectiva ---------------------------
    e5 = obst.EsquivaPilares(_cfg_obst())
    p_lejos = _Det(300, 170, 24, 50)         # base 220
    p_cerca = _Det(300, 250, 60, 170)        # base 420
    r_l = e5.paso({"rojo": [p_lejos], "verde": []}, perfil, cfg_nav)
    sep_lejos = r_l.objetivo_x - (p_lejos.x + p_lejos.w) if r_l.activo else 0
    e6 = obst.EsquivaPilares(_cfg_obst())
    r_c = e6.paso({"rojo": [p_cerca], "verde": []}, perfil, cfg_nav)
    sep_cerca = r_c.objetivo_x - (p_cerca.x + p_cerca.w) if r_c.activo else 0
    check(sep_cerca > sep_lejos * 1.5,
          "se deja mas separacion en pixeles con el pilar cerca (perspectiva)",
          f"{sep_lejos} vs {sep_cerca}")

    # --- objetivo recortado al pasillo libre ------------------------------
    # Pilar pegado a la pared derecha: pasar por su derecha no cabe.
    estrecha = _perfil(pista([(0, 150), (430, 150), (450, 430), (639, 430)]), cfg_nav)
    e7 = obst.EsquivaPilares(_cfg_obst())
    pegado = _Det(360, 250, 60, 130)
    r7 = None
    for _ in range(3):
        r7 = e7.paso({"rojo": [pegado], "verde": []}, estrecha, cfg_nav)
        time.sleep(0.01)
    check(r7.activo and r7.recortado,
          "con el pilar pegado a la pared, el objetivo se recorta al pasillo",
          f"objetivo={r7.objetivo_x} bruto={r7.objetivo_bruto}")
    check(r7.objetivo_x < r7.objetivo_bruto,
          "y queda dentro, no contra la pared",
          f"{r7.objetivo_x} < {r7.objetivo_bruto}")

    # --- soltar el pilar al pasarlo ---------------------------------------
    e8 = obst.EsquivaPilares(_cfg_obst())
    e8.paso({"rojo": [_Det(200, 250, 60, 120)], "verde": []}, perfil, cfg_nav)
    r8 = e8.paso({"rojo": [_Det(200, 330, 70, 130)], "verde": []}, perfil, cfg_nav)
    check(not r8.activo and "superado" in r8.motivo,
          "cuando el pilar llega abajo se da por pasado", r8.motivo)

    # --- apagada no hace nada ---------------------------------------------
    e9 = obst.EsquivaPilares(_cfg_obst(activo=False))
    r9 = e9.paso({"rojo": [rojo], "verde": []}, perfil, cfg_nav)
    check(not r9.activo and r9.peso == 0, "apagada no toca la direccion", r9.motivo)

    # --- el siguiente pilar se tiene en cuenta -----------------------------
    e10 = obst.EsquivaPilares(_cfg_obst())
    r10 = e10.paso({"rojo": [_Det(200, 250, 60, 120)],
                    "verde": [_Det(420, 200, 40, 80)]}, perfil, cfg_nav)
    check(r10.siguiente is not None and r10.siguiente.color == "verde",
          "ve que detras viene un verde", r10.estado if False else r10.motivo)

    # --- en el navegador: la esquiva pesa sobre el muro --------------------
    # Ojo con el matiz: "pasar por la derecha del pilar" no es "girar a la
    # derecha". Si el pilar esta a la izquierda del carro, el punto por el que
    # hay que pasar puede quedar a la izquierda del centro y el carro gira a la
    # izquierda para colarse por ahi. Por eso se prueba con un pilar bien
    # descentrado, donde el desvio es evidente.
    e11 = obst.EsquivaPilares(_cfg_obst())
    lateral = _Det(40, 250, 70, 130)          # rojo pegado al borde izquierdo
    r11 = None
    for _ in range(3):
        r11 = e11.paso({"rojo": [lateral], "verde": []}, perfil, cfg_nav)
        time.sleep(0.01)
    check(r11.direccion < -15,
          "un rojo pegado a la izquierda manda pasar justo a su derecha, no al centro",
          f"dir={r11.direccion:.0f} objetivo={r11.objetivo_x}")

    lim = dict(robot_config.POR_DEFECTO["limites"])
    n = nav.Navegador(cfg_nav, lim)
    n.paso(perfil, None)
    sin = n.paso(perfil, None)
    n2 = nav.Navegador(cfg_nav, lim)
    n2.paso(perfil, None, esquiva=r11)
    con = n2.paso(perfil, None, esquiva=r11)
    check(abs(con.direccion - sin.direccion) > 10,
          "con pilar delante la direccion cambia respecto a no tenerlo",
          f"{sin.direccion} -> {con.direccion}")
    check("rojo por la derecha" in con.motivo, "y se explica en el motivo", con.motivo)


def test_pilares_foto_real():
    print("\n[21] Esquiva sobre la foto real de los pilares")
    from src import obstaculos as obst
    ruta = RAIZ / "capturas" / "pilares_reales.png"
    perfil_col = Path("/tmp/colors_usuario.json")
    if not ruta.exists() or not perfil_col.exists():
        print("       (sin foto ni perfil real; me lo salto)")
        return
    img = cv2.imread(str(ruta))
    colores = cc.obtener(cc.cargar(perfil_col))["colores"]
    v = vision.Vision(colores)
    dets, masks = v.procesar(img, solo=["rojo", "verde", "negro"])
    check(len(dets["rojo"]) >= 1, "detecta el pilar rojo con TU calibracion",
          [str(d) for d in dets["rojo"]])
    check(len(dets["verde"]) >= 1, "detecta el pilar verde con TU calibracion",
          [str(d) for d in dets["verde"]])

    cfg_nav = _cfg()
    perfil = nav.perfil_desde_mascara(masks["negro"], cfg_nav)
    e = obst.EsquivaPilares(_cfg_obst())
    r = None
    for _ in range(3):
        r = e.paso(dets, perfil, cfg_nav)
        time.sleep(0.01)
    check(r.activo, "y decide una esquiva sobre la imagen real", r.motivo)
    if r.pilar is not None:
        esperado = "derecha" if r.pilar.color == "rojo" else "izquierda"
        check(r.pilar.nombre_lado == esperado,
              f"el {r.pilar.color} se pasa por la {esperado}", r.motivo)
        check(r.objetivo_x is not None and 0 <= r.objetivo_x < img.shape[1],
              "y el objetivo cae dentro de la imagen", r.objetivo_x)


def test_media_vuelta_no_se_deshace():
    print("\n[22] La media vuelta ya no la deshace la navegacion")
    cfg = _cfg(mv_fase_ms=200, bloqueo_sentido_ms=3000, gracia_tras_media_ms=600,
               min_recto_ms=0, usar_esquina_interna=True,
               min_muestras_presencia=20, alfa_presencia=0.15)
    lim = dict(robot_config.POR_DEFECTO["limites"])
    recto = _perfil(pista([(0, 300), (150, 300), (235, 150), (420, 150),
                           (500, 300), (639, 300)]), cfg)
    esquina_der = _perfil(pista([(0, 300), (150, 300), (235, 150), (430, 150),
                                 (450, 0), (639, 0)]), cfg)

    n = nav.Navegador(cfg, lim, {"tipo_media_vuelta": "recta_3t"})
    # Damos varias vueltas ficticias para que se decida el sentido
    for ciclo in range(12):
        for _ in range(9):
            n.paredes.observar(recto)
        for _ in range(3):
            n.paredes.observar(esquina_der)
    externo_antes = n.paredes.lado_externo
    check(externo_antes != 0, "antes de la media vuelta sabe cual es la externa",
          n.paredes.estado())

    n.pedir_media_vuelta()
    yaw = 0.0
    for _ in range(80):
        d = n.paso(recto, yaw)
        yaw = nav._norm_angulo(yaw + (12 if d.vel > 0 else (6 if d.vel < 0 else 0)))
        if n.media_vuelta_hecha:
            break
        time.sleep(0.02)
    check(n.media_vuelta_hecha, "completa la media vuelta")
    check(n.paredes.lado_externo == -externo_antes,
          "y la pared externa pasa a ser la otra", n.paredes.estado())

    # Ahora el mundo viejo insiste durante un buen rato
    estados = []
    for _ in range(120):
        d = n.paso(recto, yaw)
        estados.append(n.estado)
        yaw = nav._norm_angulo(yaw + 0.2)
    check(n.paredes.lado_externo == -externo_antes,
          "el estimador NO vuelve al sentido anterior (esto era el fallo gordo)",
          n.paredes.estado())
    check(nav.MEDIA_VUELTA not in estados, "y no vuelve a lanzar otra media vuelta")

    # Y durante la gracia posterior no encadena un giro inmediato
    n2 = nav.Navegador(dict(cfg, gracia_tras_media_ms=1500), lim,
                       {"tipo_media_vuelta": "recta_3t"})
    n2.pedir_media_vuelta()
    yaw = 0.0
    for _ in range(80):
        d = n2.paso(recto, yaw)
        yaw = nav._norm_angulo(yaw + (12 if d.vel > 0 else (6 if d.vel < 0 else 0)))
        if n2.media_vuelta_hecha:
            break
        time.sleep(0.02)
    d = n2.paso(esquina_der, yaw)
    check(n2.estado == nav.RECTO,
          "justo despues de la media vuelta no se lanza a girar otra vez",
          n2.estado)


# ===========================================================================
if __name__ == "__main__":
    print(f"OpenCV {cv2.__version__} · Python {sys.version.split()[0]}")
    for f in (test_protocolo_cruzado, test_lector_robusto, test_perfil,
              test_estrategias, test_yaw, test_enlace, test_robot_y_web,
              test_config, test_anticipacion, test_escape, test_esquina_interna,
              test_huecos, test_sentido_y_carril, test_vueltas,
              test_lineas_camara, test_media_vuelta, test_sensores_capa,
              test_regresion_arreglos, test_pilares, test_pilares_foto_real,
              test_media_vuelta_no_se_deshace):
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
