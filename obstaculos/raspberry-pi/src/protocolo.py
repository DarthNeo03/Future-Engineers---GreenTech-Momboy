"""
protocolo.py — Trama binaria Raspberry Pi 5 <-> ESP32 (reto de OBSTACULOS).

GEMELO EXACTO de firmware/esp32/protocolo.h. Si tocas uno, toca el otro:
tools/selftest.py genera vectores con este modulo y los cruza contra los del
firmware compilado en el PC.

    A5 5A | LEN | TIPO | payload (LEN bytes) | CRC8

El razonamiento de por que binario y por que CRC esta en protocolo.h y no se
repite aqui, para que no se puedan desincronizar dos explicaciones.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

SYNC1 = 0xA5
SYNC2 = 0x5A
MAX_PAYLOAD = 16
VERSION_PROTOCOLO = 1

# ------------------------------------------------------------- tipos de trama
TIPO_MANDO = 0x01
TIPO_PING = 0x02
TIPO_CONFIG = 0x03
TIPO_CAL = 0x06
TIPO_TELE = 0x81
TIPO_LOG = 0x82
TIPO_PONG = 0x83
TIPO_SENS = 0x84

# ---------------------------------------------------------- banderas de mando
F_ARMADO = 0x01
F_PARADA = 0x02
F_CENTRAR = 0x04
F_LIMPIAR = 0x08

# ----------------------------------------------------- bits de la telemetria
E_ARMADO = 0x01
E_MOTOR = 0x02
E_FAILSAFE = 0x04
E_SERVO_TOPE = 0x08
E_INV_BLOQUEADA = 0x10

# --------------------------------------------- bits de la trama de sensores
S_MPU_OK = 0x01
S_TCS_OK = 0x02
S_CALIBRANDO = 0x04
S_SOBRE_LINEA = 0x08
S_MPU_INT = 0x10
S_TCS_INT = 0x20

LINEA_NADA = 0
LINEA_NARANJA = 1
LINEA_AZUL = 2
NOMBRE_LINEA = {LINEA_NADA: "", LINEA_NARANJA: "naranja", LINEA_AZUL: "azul"}

# --------------------------------------------------- bits del byte de boton
B_NIVEL = 0x01
B_CORTE = 0x02

# -------------------------------------------------- comandos de calibracion
CAL_SESGO_GIRO = 1
CAL_CERO_YAW = 2
CAL_REDETECTAR = 3
CAL_CERO_LINEAS = 4


def crc8(datos: bytes) -> int:
    """CRC-8/ATM (poly 0x07, init 0x00). Copia byte a byte de crc8() en C++."""
    c = 0
    for b in datos:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def empaquetar(tipo: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload de {len(payload)} B excede {MAX_PAYLOAD}")
    cuerpo = bytes([len(payload), tipo]) + payload
    return bytes([SYNC1, SYNC2]) + cuerpo + bytes([crc8(cuerpo)])


# ============================================================== Pi -> ESP32 ==
def trama_mando(seq: int, vel: int, direccion: int, *, armado: bool = False,
                parada: bool = False, centrar: bool = False,
                limpiar: bool = False, vmax: int = 255, aux: int = 0) -> bytes:
    """vel y direccion en % con signo. El firmware traduce a PWM y a grados:
    aqui no se sabe (ni hace falta saber) cuanto abre el servo."""
    flags = ((F_ARMADO if armado else 0) | (F_PARADA if parada else 0) |
             (F_CENTRAR if centrar else 0) | (F_LIMPIAR if limpiar else 0))
    v = max(-100, min(100, int(round(vel))))
    d = max(-100, min(100, int(round(direccion))))
    return empaquetar(TIPO_MANDO, struct.pack(
        "<BBbbBB", seq & 0xFF, flags, v, d, max(0, min(255, int(vmax))),
        aux & 0xFF))


def trama_ping(seq: int = 0) -> bytes:
    return empaquetar(TIPO_PING, bytes([seq & 0xFF]))


def trama_config(centro: int, izquierda: int, derecha: int,
                 grados_por_seg: int, rampa_por_tick: int,
                 ms_freno_inversion: int) -> bytes:
    """Los topes de compilacion del firmware SIEMPRE ganan: esto solo puede
    estrechar el rango del servo, nunca ampliarlo."""
    return empaquetar(TIPO_CONFIG, struct.pack(
        "<BBBBBB",
        max(0, min(255, int(centro))),
        max(0, min(255, int(izquierda))),
        max(0, min(255, int(derecha))),
        max(1, min(255, int(grados_por_seg) // 10)),
        max(1, min(255, int(rampa_por_tick))),
        max(0, min(255, int(ms_freno_inversion) // 10))))


def trama_cal(comando: int) -> bytes:
    return empaquetar(TIPO_CAL, bytes([comando & 0xFF]))


# ============================================================== ESP32 -> Pi ==
@dataclass
class Telemetria:
    seq_eco: int = 0
    estado: int = 0
    pwm: int = 0
    angulo: int = 0
    ms_desde_mando: int = 0
    tramas_malas: int = 0
    version: int = 0

    @property
    def armado(self) -> bool:
        return bool(self.estado & E_ARMADO)

    @property
    def failsafe(self) -> bool:
        return bool(self.estado & E_FAILSAFE)


@dataclass
class Sensores:
    estado: int = 0
    yaw: float = 0.0          # grados, -180..180
    gz: float = 0.0           # grados/s
    claro: int = 0
    cruces_naranja: int = 0
    cruces_azul: int = 0
    botones: int = 0
    pulsaciones: int = 0
    version: int = 0

    @property
    def mpu_ok(self) -> bool:
        return bool(self.estado & S_MPU_OK)

    @property
    def tcs_ok(self) -> bool:
        return bool(self.estado & S_TCS_OK)

    @property
    def calibrando(self) -> bool:
        return bool(self.estado & S_CALIBRANDO)

    @property
    def clase_linea(self) -> int:
        return (self.estado >> 6) & 0x03

    @property
    def boton_pisado(self) -> bool:
        return bool(self.botones & B_NIVEL)

    @property
    def cortado_por_boton(self) -> bool:
        return bool(self.botones & B_CORTE)


def decodificar_telemetria(p: bytes) -> Optional[Telemetria]:
    if len(p) < 8:
        return None
    seq, est, pwm, ang, ms, malas, ver = struct.unpack("<BBBBHBB", p[:8])
    return Telemetria(seq, est, pwm, ang, ms, malas, ver)


def decodificar_sensores(p: bytes) -> Optional[Sensores]:
    if len(p) < 12:
        return None
    est, yaw, gz, claro, cn, ca, bot, pul, ver = struct.unpack("<BhhHBBBBB", p[:12])
    return Sensores(est, yaw / 10.0, gz / 10.0, claro, cn, ca, bot, pul, ver)


class Lector:
    """Maquina de estados byte a byte, gemela de proto::Lector en C++.

    Se alimenta con lo que haya llegado al puerto (puede ser medio frame) y
    devuelve la lista de tramas completas. No bloquea y no crece sin limite:
    un LEN imposible resincroniza en vez de esperar bytes que nunca llegan.
    """

    def __init__(self) -> None:
        self.tramas_malas = 0
        self._reiniciar()

    def _reiniciar(self) -> None:
        self._paso = 0
        self._n = 0
        self._tipo = 0
        self._buf = bytearray()

    def alimentar(self, datos: bytes) -> List[Tuple[int, bytes]]:
        salida: List[Tuple[int, bytes]] = []
        for b in datos:
            paso = self._paso
            if paso == 0:                      # esperando SYNC1
                if b == SYNC1:
                    self._paso = 1
            elif paso == 1:                    # esperando SYNC2
                # Un A5 repetido no pierde el sincronismo: sigue esperando 5A.
                self._paso = 2 if b == SYNC2 else (1 if b == SYNC1 else 0)
            elif paso == 2:                    # LEN
                if b > MAX_PAYLOAD:
                    self.tramas_malas += 1
                    self._paso = 0
                    continue
                self._n = b
                self._buf = bytearray()
                self._paso = 3
            elif paso == 3:                    # TIPO
                self._tipo = b
                self._paso = 5 if self._n == 0 else 4
            elif paso == 4:                    # payload
                self._buf.append(b)
                if len(self._buf) >= self._n:
                    self._paso = 5
            else:                              # paso 5: este byte ES el CRC
                esperado = crc8(bytes([self._n, self._tipo]) + bytes(self._buf))
                tipo, cuerpo = self._tipo, bytes(self._buf)
                self._reiniciar()
                if esperado == b:
                    salida.append((tipo, cuerpo))
                else:
                    self.tramas_malas += 1
        return salida
