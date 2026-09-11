"""
protocolo.py — Trama binaria entre la Raspberry Pi y el ESP32.

Formato (identico en firmware/esp32_carro/protocolo.h):

    A5 5A | LEN | TIPO | payload (LEN bytes) | CRC8
     sync    1      1        0..16               1

  * LEN y TIPO entran en el CRC; los dos bytes de sync no.
  * CRC8 poly 0x07, init 0x00 (CRC-8/ATM). Sin tabla en el ESP32, con tabla
    aqui porque en Python la tabla si compensa.
  * Si el CRC falla se descarta la trama y el lector busca el siguiente A5 5A.
    No hay reenvio: a 50 Hz una trama perdida se sustituye sola en 20 ms, y un
    reenvio tardio seria peor que nada (el carro obedeceria una orden vieja).

El mando ocupa 11 bytes en total. A 115200 baudios eso es ~1 ms de linea.

Por que no texto: un "M D 200 \n" obliga a parsear en el ESP32 mientras el
motor espera, y un byte de ruido puede convertir "A95" en "A9" (el servo se va
a 9 grados). Con LEN + CRC, una trama con ruido simplemente no existe.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

SYNC1 = 0xA5
SYNC2 = 0x5A
MAX_PAYLOAD = 16
VERSION_PROTOCOLO = 1

# --- tipos de trama --------------------------------------------------------
TIPO_MANDO = 0x01     # Pi -> ESP32 (6 bytes)
TIPO_PING = 0x02      # Pi -> ESP32 (1 byte: seq)
TIPO_CONFIG = 0x03    # Pi -> ESP32 (6 bytes) ajustes en caliente
TIPO_TELE = 0x81      # ESP32 -> Pi (8 bytes)
TIPO_LOG = 0x82       # ESP32 -> Pi (texto)
TIPO_PONG = 0x83      # ESP32 -> Pi (1 byte: seq eco)
TIPO_IMU = 0x84       # ESP32 -> Pi (6 bytes) rumbo del MPU6050
TIPO_COLOR = 0x85     # ESP32 -> Pi (6 bytes) evento del TCS34725
TIPO_SENSORES = 0x86  # ESP32 -> Pi (4 bytes) que sensores hay conectados

# --- comandos hacia los sensores, en el byte 'aux' del mando ---------------
# Van ahi en vez de en una trama nueva: son ordenes raras (una al arrancar) y
# asi no hay una segunda ruta de escritura que mantener.
AUX_CERO_YAW = 0x01       # pone el rumbo actual como cero
AUX_CALIB_IMU = 0x02      # recalibra la deriva del giroscopio (carro quieto)
AUX_CALIB_COLOR = 0x04    # toma el color actual como "piso blanco"
AUX_REINIT_SENSORES = 0x08  # vuelve a buscar el MPU6050 y el TCS34725 en el I2C

# --- identificadores de linea del suelo ------------------------------------
LINEA_NINGUNA = 0
LINEA_NARANJA = 1
LINEA_AZUL = 2
NOMBRE_LINEA = {LINEA_NINGUNA: "ninguna", LINEA_NARANJA: "naranja", LINEA_AZUL: "azul"}

# --- bits de 'presentes' en TIPO_SENSORES ---------------------------------
S_MPU = 0x01
S_TCS = 0x02

# --- banderas del mando ----------------------------------------------------
F_ARMADO = 0x01       # sin esto el ESP32 no mueve el motor pase lo que pase
F_PARADA = 0x02       # parada de emergencia: ignora vel
F_CENTRAR = 0x04      # lleva el servo al centro
F_LIMPIAR = 0x08      # reinicia los contadores de error

# --- bits de estado en la telemetria --------------------------------------
E_ARMADO = 0x01
E_MOTOR = 0x02
E_FAILSAFE = 0x04     # el ESP32 paro solo por silencio en el serial
E_SERVO_TOPE = 0x08   # se pidio un angulo fuera de limite y se recorto
E_INV_BLOQUEADA = 0x10  # se pidio invertir el giro sin pasar por cero


# ---------------------------------------------------------------------------
def _tabla_crc8(poly: int = 0x07) -> Tuple[int, ...]:
    tabla = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFF if (c & 0x80) else ((c << 1) & 0xFF)
        tabla.append(c)
    return tuple(tabla)


TABLA_CRC8 = _tabla_crc8()


def crc8(datos: bytes) -> int:
    c = 0
    for b in datos:
        c = TABLA_CRC8[c ^ b]
    return c


def empaquetar(tipo: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload de {len(payload)} bytes, maximo {MAX_PAYLOAD}")
    cuerpo = bytes((len(payload), tipo)) + payload
    return bytes((SYNC1, SYNC2)) + cuerpo + bytes((crc8(cuerpo),))


# ---------------------------------------------------------------------------
def _lim(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else (hi if v > hi else v)


@dataclass
class Mando:
    """Lo que la Pi le pide al ESP32, ya normalizado.

    vel y direccion van en PORCENTAJE con signo, no en unidades de hardware.
    Asi la Pi no necesita saber nada del servo ni del puente H: el mapeo a
    grados y a PWM lo hace el firmware, que es quien tiene los limites fisicos.
      vel:  -100 (reversa a fondo) .. 0 (parado) .. +100 (avance a fondo)
      dir:  -100 (izquierda a fondo) .. 0 (recto) .. +100 (derecha a fondo)
      vmax: tope absoluto de PWM 0..255 que el firmware nunca supera
    """
    seq: int = 0
    vel: int = 0
    direccion: int = 0
    vmax: int = 160
    armado: bool = False
    parada: bool = False
    centrar: bool = False
    limpiar: bool = False
    aux: int = 0                 # ordenes para los sensores (AUX_*)

    def banderas(self) -> int:
        f = 0
        if self.armado:
            f |= F_ARMADO
        if self.parada:
            f |= F_PARADA
        if self.centrar:
            f |= F_CENTRAR
        if self.limpiar:
            f |= F_LIMPIAR
        return f

    def a_bytes(self) -> bytes:
        payload = struct.pack(
            "<BBbbBB",
            self.seq & 0xFF,
            self.banderas(),
            _lim(int(self.vel), -100, 100),
            _lim(int(self.direccion), -100, 100),
            _lim(int(self.vmax), 0, 255),
            _lim(int(self.aux), 0, 255),
        )
        return empaquetar(TIPO_MANDO, payload)

    @staticmethod
    def desde_payload(payload: bytes) -> "Mando":
        seq, flags, vel, direccion, vmax, aux = struct.unpack("<BBbbBB", payload[:6])
        return Mando(seq=seq, vel=vel, direccion=direccion, vmax=vmax,
                     armado=bool(flags & F_ARMADO),
                     parada=bool(flags & F_PARADA),
                     centrar=bool(flags & F_CENTRAR),
                     limpiar=bool(flags & F_LIMPIAR),
                     aux=aux)


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
    def motor(self) -> bool:
        return bool(self.estado & E_MOTOR)

    @property
    def failsafe(self) -> bool:
        return bool(self.estado & E_FAILSAFE)

    @property
    def servo_en_tope(self) -> bool:
        return bool(self.estado & E_SERVO_TOPE)

    def a_bytes(self) -> bytes:
        return empaquetar(TIPO_TELE, struct.pack(
            "<BBBBHBB", self.seq_eco & 0xFF, self.estado & 0xFF,
            _lim(self.pwm, 0, 255), _lim(self.angulo, 0, 255),
            _lim(self.ms_desde_mando, 0, 65535),
            _lim(self.tramas_malas, 0, 255), self.version & 0xFF))

    @staticmethod
    def desde_payload(payload: bytes) -> "Telemetria":
        (seq, estado, pwm, ang, ms, malas, ver) = struct.unpack("<BBBBHBB", payload[:8])
        return Telemetria(seq, estado, pwm, ang, ms, malas, ver)


@dataclass
class DatosIMU:
    """Rumbo que calcula el ESP32 con el MPU6050 y manda ya masticado.

    Se manda el YAW INTEGRADO, no las muestras crudas: el MPU escupe cientos de
    lecturas por segundo y mandarlas todas saturaria el serial para nada. El
    ESP32 integra a 200 Hz y publica 6 bytes a 50 Hz (o antes si el rumbo se
    mueve mas de un umbral). Eso es lo que la navegacion necesita.
    """
    yaw: float = 0.0          # grados, -180..180
    giro_z: float = 0.0       # grados/s
    calibrado: bool = False
    temp: int = 0             # grados C, util para ver la deriva termica

    def a_bytes(self) -> bytes:
        return empaquetar(TIPO_IMU, struct.pack(
            "<hhBB",
            _lim(int(round(self.yaw * 10)), -1800, 1800),
            _lim(int(round(self.giro_z * 10)), -32000, 32000),
            1 if self.calibrado else 0,
            _lim(int(self.temp), 0, 255)))

    @staticmethod
    def desde_payload(pl: bytes) -> "DatosIMU":
        yaw, gz, cal, temp = struct.unpack("<hhBB", pl[:6])
        return DatosIMU(yaw / 10.0, gz / 10.0, bool(cal), temp)


@dataclass
class EventoColor:
    """Cambio de linea visto por el TCS34725. Solo se manda cuando CAMBIA."""
    linea: int = LINEA_NINGUNA
    r: int = 0                # normalizados 0..255 contra el canal claro
    g: int = 0
    b: int = 0
    luz: int = 0              # canal claro / 256, para ver si esta cegado

    @property
    def nombre(self) -> str:
        return NOMBRE_LINEA.get(self.linea, "?")

    def a_bytes(self) -> bytes:
        return empaquetar(TIPO_COLOR, struct.pack(
            "<BBBBBB", _lim(self.linea, 0, 255), _lim(self.r, 0, 255),
            _lim(self.g, 0, 255), _lim(self.b, 0, 255), _lim(self.luz, 0, 255), 0))

    @staticmethod
    def desde_payload(pl: bytes) -> "EventoColor":
        linea, r, g, b, luz, _ = struct.unpack("<BBBBBB", pl[:6])
        return EventoColor(linea, r, g, b, luz)


@dataclass
class EstadoSensores:
    presentes: int = 0
    hz_imu: int = 0
    hz_color: int = 0

    @property
    def mpu(self) -> bool:
        return bool(self.presentes & S_MPU)

    @property
    def tcs(self) -> bool:
        return bool(self.presentes & S_TCS)

    def a_bytes(self) -> bytes:
        return empaquetar(TIPO_SENSORES, struct.pack(
            "<BBBB", _lim(self.presentes, 0, 255), _lim(self.hz_imu, 0, 255),
            _lim(self.hz_color, 0, 255), 0))

    @staticmethod
    def desde_payload(pl: bytes) -> "EstadoSensores":
        presentes, hz_i, hz_c, _ = struct.unpack("<BBBB", pl[:4])
        return EstadoSensores(presentes, hz_i, hz_c)


def empaquetar_config(servo_centro: int, servo_min: int, servo_max: int,
                      rampa_pwm: int, servo_grados_s: int) -> bytes:
    """Ajustes en caliente. El firmware los vuelve a recortar contra sus
    propios topes de compilacion: nada que llegue por el cable puede ampliar
    el rango fisico del servo, solo estrecharlo."""
    return empaquetar(TIPO_CONFIG, struct.pack(
        "<BBBBBB", _lim(servo_centro, 0, 255), _lim(servo_min, 0, 255),
        _lim(servo_max, 0, 255), _lim(rampa_pwm, 1, 255),
        _lim(servo_grados_s // 10, 1, 255), 0))


# ---------------------------------------------------------------------------
class Lector:
    """Lector con reintento hacia atras. Gemelo de proto::Lector en C++.

    Acumula los bytes que llegan del puerto y reescanea el buffer. Un lector
    "de una pasada" tiene un agujero: si llega una trama truncada, se traga los
    bytes de la SIGUIENTE creyendo que son su payload y pierde las dos. Aqui,
    si el CRC falla, se avanza un solo byte y se vuelve a buscar el sync, asi
    que una trama buena escondida detras de basura se recupera igual.
    """

    CAP = 256          # en Python podemos permitirnos mas holgura que en el ESP32

    def __init__(self):
        self.reiniciar()
        self.frames_ok = 0
        self.crc_malos = 0
        self.descartados = 0

    def reiniciar(self) -> None:
        self._buf = bytearray()

    def alimentar(self, datos: bytes) -> List[Tuple[int, bytes]]:
        self._buf.extend(datos)
        if len(self._buf) > self.CAP:
            sobra = len(self._buf) - self.CAP
            del self._buf[:sobra]
            self.descartados += sobra

        salida: List[Tuple[int, bytes]] = []
        buf = self._buf
        while True:
            # tirar lo que no puede ser el arranque de una trama
            while buf and buf[0] != SYNC1:
                del buf[0]
                self.descartados += 1
            if len(buf) >= 2 and buf[1] != SYNC2:
                del buf[0]
                self.descartados += 1
                continue
            if len(buf) < 5:
                break
            n = buf[2]
            if n > MAX_PAYLOAD:
                del buf[0]
                self.descartados += 1
                continue
            total = n + 5
            if len(buf) < total:
                break
            if crc8(bytes(buf[2:total - 1])) == buf[total - 1]:
                salida.append((buf[3], bytes(buf[4:4 + n])))
                self.frames_ok += 1
                del buf[:total]
            else:
                self.crc_malos += 1
                del buf[0]
        return salida
