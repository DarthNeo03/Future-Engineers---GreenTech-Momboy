"""
protocolo.py — Trama binaria entre la Raspberry Pi y el ESP32 (version 2).

Formato (identico en code/esp32_carro/protocolo.h):

    A5 5A | LEN | TIPO | payload (LEN bytes) | CRC8
     sync    1      1        0..16               1

  * LEN y TIPO entran en el CRC; los dos bytes de sync no.
  * CRC8 poly 0x07, init 0x00 (CRC-8/ATM).
  * Si el CRC falla se descarta la trama y el lector busca el siguiente A5 5A.

Cambios de la version 2 (compatible hacia atras: las tramas viejas no cambian):

  TIPO_SENSORES (0x84, ESP32 -> Pi): el MPU6050 y el TCS34725 ahora cuelgan
  del I2C del ESP32, asi que el ESP32 integra el yaw y clasifica las lineas
  del piso, y manda el resultado aqui. Los cruces de linea viajan como
  CONTADORES (no banderas): aunque se pierdan tramas, la Pi ve el contador
  avanzar y no se le escapa ningun cruce.

  TIPO_CFG_TCS (0x05, Pi -> ESP32): umbrales de clasificacion naranja/azul
  calibrables desde la web, mas tiempo de integracion y ganancia del TCS.

  TIPO_CMD_CAL (0x06, Pi -> ESP32): calibrar giroscopio / cero de yaw /
  reintentar la deteccion I2C.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import List, Tuple

SYNC1 = 0xA5
SYNC2 = 0x5A
MAX_PAYLOAD = 16
VERSION_PROTOCOLO = 2

# --- tipos de trama --------------------------------------------------------
TIPO_MANDO = 0x01      # Pi -> ESP32 (6 bytes)
TIPO_PING = 0x02       # Pi -> ESP32 (1 byte: seq)
TIPO_CONFIG = 0x03     # Pi -> ESP32 (6 bytes) servo/motor en caliente
TIPO_CFG_TCS = 0x05    # Pi -> ESP32 (10 bytes) umbrales de linea + TCS
TIPO_CMD_CAL = 0x06    # Pi -> ESP32 (1 byte) calibraciones
TIPO_TELE = 0x81       # ESP32 -> Pi (8 bytes)
TIPO_LOG = 0x82        # ESP32 -> Pi (texto)
TIPO_PONG = 0x83       # ESP32 -> Pi (1 byte: seq eco)
TIPO_SENSORES = 0x84   # ESP32 -> Pi (14 bytes) yaw + TCS + eventos de linea

# --- banderas del mando ----------------------------------------------------
F_ARMADO = 0x01
F_PARADA = 0x02
F_CENTRAR = 0x04
F_LIMPIAR = 0x08

# --- bits de estado en la telemetria --------------------------------------
E_ARMADO = 0x01
E_MOTOR = 0x02
E_FAILSAFE = 0x04
E_SERVO_TOPE = 0x08
E_INV_BLOQUEADA = 0x10

# --- bits de estado en la trama de sensores -------------------------------
S_MPU_OK = 0x01        # el MPU6050 responde
S_TCS_OK = 0x02        # el TCS34725 responde
S_CALIBRANDO = 0x04    # giroscopio calibrando: yaw congelado, NO MOVER
S_SOBRE_LINEA = 0x08   # ahora mismo el TCS ve una linea (la de 'clase')
S_MPU_INT = 0x10       # la pata INT del MPU esta dando flancos (dt exacto)
S_TCS_INT = 0x20       # la pata INT del TCS esta cableada (borde por hardware)

# clase de linea (2 bits altos del byte de estado de sensores)
LINEA_NADA = 0
LINEA_NARANJA = 1
LINEA_AZUL = 2

# --- comandos de calibracion ----------------------------------------------
CAL_GIRO = 1           # medir sesgo del giroscopio (carro QUIETO) y cero yaw
CAL_CERO_YAW = 2       # solo poner el yaw a cero
CAL_REDETECTAR = 3     # volver a sondear el bus I2C ya mismo


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


def _lim(v: int, lo: int, hi: int) -> int:
    return lo if v < lo else (hi if v > hi else v)


# ---------------------------------------------------------------------------
@dataclass
class Mando:
    """Lo que la Pi le pide al ESP32. vel y direccion en % con signo: el
    firmware es el unico que conoce grados de servo y PWM."""
    seq: int = 0
    vel: int = 0
    direccion: int = 0
    vmax: int = 160
    armado: bool = False
    parada: bool = False
    centrar: bool = False
    limpiar: bool = False

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
            0,
        )
        return empaquetar(TIPO_MANDO, payload)

    @staticmethod
    def desde_payload(payload: bytes) -> "Mando":
        seq, flags, vel, direccion, vmax, _aux = struct.unpack("<BBbbBB", payload[:6])
        return Mando(seq=seq, vel=vel, direccion=direccion, vmax=vmax,
                     armado=bool(flags & F_ARMADO),
                     parada=bool(flags & F_PARADA),
                     centrar=bool(flags & F_CENTRAR),
                     limpiar=bool(flags & F_LIMPIAR))


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


# ---------------------------------------------------------------------------
@dataclass
class Sensores:
    """Trama 0x84. 14 bytes de payload:

        yaw_deci   int16   yaw en decimas de grado, -1800..1800
        gz_deci    int16   velocidad de giro en decimas de grado/s
        c,r,g,b    uint16  lectura cruda del TCS34725 (canal claro + RGB)
        estado     uint8   bits S_* + (clase de linea << 6)
        cnt_lineas uint8   contador naranja (4 bits bajos) y azul (4 altos).
                           Avanza en cada CRUCE detectado y envuelve en 16:
                           la Pi compara con el ultimo valor visto, asi que
                           perder tramas no pierde cruces.
    """
    yaw_deci: int = 0
    gz_deci: int = 0
    c: int = 0
    r: int = 0
    g: int = 0
    b: int = 0
    estado: int = 0
    cnt_lineas: int = 0

    @property
    def yaw(self) -> float:
        return self.yaw_deci / 10.0

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
    def sobre_linea(self) -> bool:
        return bool(self.estado & S_SOBRE_LINEA)

    @property
    def mpu_int(self) -> bool:
        return bool(self.estado & S_MPU_INT)

    @property
    def tcs_int(self) -> bool:
        return bool(self.estado & S_TCS_INT)

    @property
    def clase_linea(self) -> int:
        return (self.estado >> 6) & 0x03

    @property
    def cnt_naranja(self) -> int:
        return self.cnt_lineas & 0x0F

    @property
    def cnt_azul(self) -> int:
        return (self.cnt_lineas >> 4) & 0x0F

    def a_bytes(self) -> bytes:
        return empaquetar(TIPO_SENSORES, struct.pack(
            "<hhHHHHBB",
            _lim(self.yaw_deci, -32768, 32767),
            _lim(self.gz_deci, -32768, 32767),
            _lim(self.c, 0, 65535), _lim(self.r, 0, 65535),
            _lim(self.g, 0, 65535), _lim(self.b, 0, 65535),
            self.estado & 0xFF, self.cnt_lineas & 0xFF))

    @staticmethod
    def desde_payload(payload: bytes) -> "Sensores":
        (yaw, gz, c, r, g, b, est, cnt) = struct.unpack("<hhHHHHBB", payload[:14])
        return Sensores(yaw, gz, c, r, g, b, est, cnt)


# ---------------------------------------------------------------------------
def empaquetar_config(servo_centro: int, servo_min: int, servo_max: int,
                      rampa_pwm: int, servo_grados_s: int) -> bytes:
    """Ajustes de servo/motor en caliente. El firmware los recorta contra sus
    topes de compilacion: nada que llegue por el cable amplia el rango fisico."""
    return empaquetar(TIPO_CONFIG, struct.pack(
        "<BBBBBB", _lim(servo_centro, 0, 255), _lim(servo_min, 0, 255),
        _lim(servo_max, 0, 255), _lim(rampa_pwm, 1, 255),
        _lim(servo_grados_s // 10, 1, 255), 0))


def empaquetar_cfg_tcs(c_min: int,
                       naranja_r_min: int, naranja_b_max: int,
                       azul_b_min: int, azul_r_max: int,
                       muestras_min: int, refractario_ds: int,
                       atime: int, gain: int,
                       naranja_dif_min: int = 30,
                       azul_dif_min: int = 18,
                       int_umbral_pct: int = 55) -> bytes:
    """Umbrales del clasificador de lineas del ESP32.

    El clasificador trabaja con RATIOS normalizados r*255/c y b*255/c, que casi
    no dependen de la luz:
      naranja: ratio_r >= naranja_r_min  Y  ratio_b <= naranja_b_max
      azul:    ratio_b >= azul_b_min     Y  ratio_r <= azul_r_max
    ademas el canal claro c debe superar c_min (si no, es sombra/borde).

    muestras_min: lecturas seguidas iguales antes de dar el cruce por bueno.
    refractario_ds: decimas de segundo sin admitir OTRO cruce del mismo color
                    (una linea de 20 mm se cruza una vez, no tres).
    atime/gain: registros crudos del TCS34725 (0xF6 = 24 ms; gain 2 = x16).
    """
    return empaquetar(TIPO_CFG_TCS, struct.pack(
        "<HBBBBBBBBBBB", _lim(c_min, 0, 65535),
        _lim(naranja_r_min, 0, 255), _lim(naranja_b_max, 0, 255),
        _lim(azul_b_min, 0, 255), _lim(azul_r_max, 0, 255),
        _lim(muestras_min, 1, 10), _lim(refractario_ds, 1, 255),
        _lim(atime, 0, 255), _lim(gain, 0, 3),
        _lim(naranja_dif_min, 0, 255), _lim(azul_dif_min, 0, 255),
        _lim(int_umbral_pct, 5, 95)))


def empaquetar_cal(cmd: int) -> bytes:
    return empaquetar(TIPO_CMD_CAL, bytes((_lim(cmd, 0, 255),)))


# ---------------------------------------------------------------------------
class Lector:
    """Lector con reintento hacia atras. Gemelo de proto::Lector en C++.

    Si el CRC falla se avanza UN byte y se vuelve a buscar el sync: una trama
    buena escondida detras de basura se recupera igual.
    """

    CAP = 256

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
