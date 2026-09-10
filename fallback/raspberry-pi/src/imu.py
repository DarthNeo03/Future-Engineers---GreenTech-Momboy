"""
imu.py — MPU6050 opcional, conectado al I2C de la Raspberry.

OPCIONAL DE VERDAD: si no hay chip, si no hay bus I2C (Windows), o si falla a
mitad de carrera, `IMU.disponible` queda en False y el resto del programa
funciona igual, solo que sin ayuda de rumbo. Nada llama a la IMU sin
comprobarlo antes.

Por que en la Pi y no en el ESP32: el MPU6050 escupe 100-1000 muestras por
segundo y lo que hace falta es integrarlas y filtrarlas, no reaccionar a cada
una. Metiendo eso en el ESP32 le robas tiempo al lazo de control (que si es
critico) y encima habria que mandar el resultado por el serial. Aqui el hilo
corre a 100 Hz, cuesta menos del 2% de un nucleo de la Pi 5, y la navegacion
lee la ultima estimacion sin bloquearse.

Cableado (Pi 5, cabecera de 40 pines):
    VCC -> pin 1  (3V3)      SDA -> pin 3  (GPIO2)
    GND -> pin 6  (GND)      SCL -> pin 5  (GPIO3)
Activar el bus: sudo raspi-config -> Interface Options -> I2C -> Yes
Comprobar:     i2cdetect -y 1     (debe aparecer 68 o 69)
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Dict, List, Optional

# Registros del MPU6050
REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_WHO_AM_I = 0x75
REG_ACCEL_XOUT_H = 0x3B

ESCALA_GIRO = 131.0      # LSB por grado/s con +-250 dps
ESCALA_ACEL = 16384.0    # LSB por g con +-2 g


class IMU:
    """Lee el MPU6050 en su propio hilo y publica yaw/pitch/roll.

    Uso:
        imu = IMU(cfg)
        imu.iniciar()
        if imu.disponible:
            yaw = imu.yaw
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg or {}
        self.disponible = False
        self.motivo = "sin iniciar"
        self.yaw = 0.0            # grados, acumulado (-180..180)
        self.pitch = 0.0
        self.roll = 0.0
        self.temp = 0.0
        self.hz_real = 0.0
        self.calibrando = False
        self.direccion: Optional[int] = None

        self._bus = None
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._sesgo = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()

    # -- arranque ---------------------------------------------------------
    def iniciar(self) -> bool:
        if not bool(self.cfg.get("activo", True)):
            self.motivo = "desactivada en la configuracion"
            return False
        try:
            from smbus2 import SMBus       # import perezoso: en Windows no existe
        except Exception as e:
            self.motivo = f"sin smbus2 ({e})"
            return False

        bus_n = int(self.cfg.get("bus", 1))
        try:
            bus = SMBus(bus_n)
        except Exception as e:
            self.motivo = f"no se pudo abrir el bus I2C {bus_n} ({e})"
            return False

        direcciones: List[int] = [int(d) for d in self.cfg.get("direcciones", [0x68, 0x69])]
        for dirn in direcciones:
            try:
                quien = bus.read_byte_data(dirn, REG_WHO_AM_I)
            except Exception:
                continue
            # 0x68 = MPU6050, 0x70/0x71/0x73 = variantes MPU6500/9250 compatibles
            if quien in (0x68, 0x70, 0x71, 0x73, 0x98):
                self.direccion = dirn
                break
        if self.direccion is None:
            try:
                bus.close()
            except Exception:
                pass
            self.motivo = f"no hay MPU6050 en {['0x%02X' % d for d in direcciones]}"
            return False

        try:
            bus.write_byte_data(self.direccion, REG_PWR_MGMT_1, 0x01)  # reloj del giro X
            time.sleep(0.05)
            bus.write_byte_data(self.direccion, REG_SMPLRT_DIV, 0x04)  # 1 kHz / 5 = 200 Hz
            bus.write_byte_data(self.direccion, REG_CONFIG, 0x03)      # DLPF 44 Hz
            bus.write_byte_data(self.direccion, REG_GYRO_CONFIG, 0x00)  # +-250 dps
            bus.write_byte_data(self.direccion, REG_ACCEL_CONFIG, 0x00)  # +-2 g
        except Exception as e:
            self.motivo = f"error configurando el MPU6050 ({e})"
            return False

        self._bus = bus
        self.disponible = True
        self.motivo = f"MPU6050 en 0x{self.direccion:02X}, bus {bus_n}"
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="imu")
        self._hilo.start()
        return True

    def parar(self):
        self._parar.set()
        if self._hilo:
            self._hilo.join(timeout=1.0)
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
        self.disponible = False

    # -- lectura ----------------------------------------------------------
    def _leer_crudo(self):
        datos = self._bus.read_i2c_block_data(self.direccion, REG_ACCEL_XOUT_H, 14)

        def s16(hi, lo):
            v = (hi << 8) | lo
            return v - 65536 if v & 0x8000 else v

        ax = s16(datos[0], datos[1]) / ESCALA_ACEL
        ay = s16(datos[2], datos[3]) / ESCALA_ACEL
        az = s16(datos[4], datos[5]) / ESCALA_ACEL
        tp = s16(datos[6], datos[7]) / 340.0 + 36.53
        gx = s16(datos[8], datos[9]) / ESCALA_GIRO
        gy = s16(datos[10], datos[11]) / ESCALA_GIRO
        gz = s16(datos[12], datos[13]) / ESCALA_GIRO
        return ax, ay, az, tp, gx, gy, gz

    def calibrar(self, muestras: Optional[int] = None) -> bool:
        """Mide la deriva del giroscopio CON EL CARRO QUIETO.

        Sin esto el yaw se va solo 1-3 grados por segundo y a la tercera recta
        el rumbo objetivo ya no significa nada. Se llama al arrancar y se puede
        repetir desde el panel.
        """
        if not self.disponible:
            return False
        n = int(muestras or self.cfg.get("muestras_calibracion", 400))
        self.calibrando = True
        suma = [0.0, 0.0, 0.0]
        leidas = 0
        for _ in range(n):
            try:
                _, _, _, _, gx, gy, gz = self._leer_crudo()
            except Exception:
                continue
            suma[0] += gx
            suma[1] += gy
            suma[2] += gz
            leidas += 1
            time.sleep(0.002)
        self.calibrando = False
        if leidas < n * 0.5:
            return False
        with self._lock:
            self._sesgo = [s / leidas for s in suma]
            self.yaw = 0.0
        return True

    def poner_cero(self):
        with self._lock:
            self.yaw = 0.0

    # -- hilo -------------------------------------------------------------
    def _bucle(self):
        periodo = 1.0 / max(10, int(self.cfg.get("hz", 100)))
        alfa = float(self.cfg.get("alfa_complementario", 0.98))
        t_prev = time.perf_counter()
        fallos = 0
        cuenta = 0
        t_hz = t_prev
        while not self._parar.is_set():
            t0 = time.perf_counter()
            if self.calibrando:
                time.sleep(periodo)
                continue
            try:
                ax, ay, az, tp, gx, gy, gz = self._leer_crudo()
                fallos = 0
            except Exception:
                fallos += 1
                if fallos > 20:
                    # Se solto un cable a mitad de carrera: seguimos sin IMU en
                    # vez de tumbar el programa.
                    self.disponible = False
                    self.motivo = "el MPU6050 dejo de responder"
                    return
                time.sleep(periodo)
                continue

            ahora = time.perf_counter()
            dt = min(0.2, ahora - t_prev)
            t_prev = ahora

            with self._lock:
                gx -= self._sesgo[0]
                gy -= self._sesgo[1]
                gz -= self._sesgo[2]
                # Yaw: solo integracion. El acelerometro no puede corregirlo
                # (la gravedad no dice nada del rumbo); por eso importa el sesgo.
                self.yaw = (self.yaw + gz * dt + 180.0) % 360.0 - 180.0
                # Pitch y roll si se mezclan con el acelerometro (filtro
                # complementario): utiles para detectar que el carro se subio
                # a algo o volco.
                try:
                    pitch_acel = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
                    roll_acel = math.degrees(math.atan2(ay, az))
                except ValueError:
                    pitch_acel = self.pitch
                    roll_acel = self.roll
                self.pitch = alfa * (self.pitch + gy * dt) + (1 - alfa) * pitch_acel
                self.roll = alfa * (self.roll + gx * dt) + (1 - alfa) * roll_acel
                self.temp = tp

            cuenta += 1
            if ahora - t_hz >= 1.0:
                self.hz_real = cuenta / (ahora - t_hz)
                cuenta = 0
                t_hz = ahora

            resto = periodo - (time.perf_counter() - t0)
            if resto > 0:
                time.sleep(resto)

    # -- info -------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "disponible": self.disponible,
            "motivo": self.motivo,
            "yaw": round(self.yaw, 1),
            "pitch": round(self.pitch, 1),
            "roll": round(self.roll, 1),
            "hz": round(self.hz_real, 1),
            "calibrando": self.calibrando,
        }
