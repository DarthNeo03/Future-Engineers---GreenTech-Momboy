"""
sensores.py — Rumbo y color de suelo, vengan de donde vengan.

El resto del programa pregunta `sensores.yaw` y `sensores.ultima_linea` y no se
entera de si eso sale del ESP32, del I2C de la Raspberry o de la camara. Eso es
todo el punto de este archivo: hoy el MPU6050 y el TCS34725 van al ESP32, y si
manana los pasas al I2C de la Pi solo cambias una palabra en robot.json.

    origen_rumbo: "auto" | "esp32" | "pi" | "ninguno"
    origen_color: "auto" | "esp32" | "pi" | "camara" | "ninguno"

En "auto" gana el ESP32 si dice que tiene el chip conectado (trama
TIPO_SENSORES); si no, se prueba el I2C de la Pi; si tampoco, se sigue sin el.
La camara siempre sirve como fuente de linea en paralelo, y el contador de
vueltas fusiona las dos (ver vueltas.py).

El cambio de origen es en caliente: si desenchufas el sensor del ESP32 a mitad
de prueba, a los pocos segundos deja de reportarlo y esto se cae a la
alternativa sin que nadie tenga que reiniciar nada.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, Optional, Tuple

from . import protocolo as P

CADUCIDAD_ESP32 = 1.5      # s sin trama del sensor = ese origen esta muerto


class Sensores:
    def __init__(self, cfg: Dict[str, Any], al_log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg or {}
        self.al_log = al_log or (lambda s: None)
        self._lock = threading.Lock()

        # --- rumbo -------------------------------------------------------
        self.yaw: float = 0.0
        self.giro_z: float = 0.0
        self.rumbo_calibrado = False
        self.origen_rumbo = "ninguno"
        self._t_yaw_esp32 = 0.0
        self._offset_yaw = 0.0          # para el "poner a cero" del lado de la Pi

        # --- color -------------------------------------------------------
        self.ultima_linea = P.LINEA_NINGUNA
        self.origen_color = "ninguno"
        self._t_color_esp32 = 0.0
        self.eventos: Deque[Tuple[float, int, str]] = deque(maxlen=64)
        self.rgb = (0, 0, 0)
        self.luz = 0

        # --- estado del ESP32 --------------------------------------------
        self.esp32 = P.EstadoSensores()
        self._t_estado_esp32 = 0.0

        # --- respaldo por I2C directo a la Pi -----------------------------
        self.imu_pi = None
        self.tcs_pi = None

    # ------------------------------------------------------------------
    def iniciar_respaldo_pi(self) -> None:
        """Arranca el MPU6050 en el I2C de la Pi solo si toca."""
        pref = str(self.cfg.get("origen_rumbo", "auto")).lower()
        if pref in ("esp32", "ninguno"):
            return
        try:
            from . import imu as imu_mod
            self.imu_pi = imu_mod.IMU(self.cfg.get("imu_pi", {}))
            if self.imu_pi.iniciar():
                self.al_log(f"[sensores] rumbo por I2C de la Pi: {self.imu_pi.motivo}")
            else:
                self.al_log(f"[sensores] sin MPU6050 en la Pi: {self.imu_pi.motivo}")
                self.imu_pi = None
        except Exception as e:
            self.al_log(f"[sensores] no se pudo abrir el I2C de la Pi ({e})")
            self.imu_pi = None

    def parar(self) -> None:
        if self.imu_pi is not None:
            self.imu_pi.parar()

    # ------------------------------------------------------------------
    # Entradas desde el enlace con el ESP32
    # ------------------------------------------------------------------
    def desde_esp32_imu(self, d: P.DatosIMU) -> None:
        with self._lock:
            self._t_yaw_esp32 = time.time()
            self._yaw_esp32 = d.yaw
            self._giro_esp32 = d.giro_z
            self._cal_esp32 = d.calibrado

    def desde_esp32_color(self, ev: P.EventoColor) -> None:
        with self._lock:
            self._t_color_esp32 = time.time()
            self.rgb = (ev.r, ev.g, ev.b)
            self.luz = ev.luz
            if ev.linea != P.LINEA_NINGUNA:
                self.eventos.append((time.time(), ev.linea, "esp32"))
            self.ultima_linea = ev.linea

    def desde_esp32_estado(self, e: P.EstadoSensores) -> None:
        with self._lock:
            self.esp32 = e
            self._t_estado_esp32 = time.time()

    def desde_camara(self, linea: int) -> None:
        """La vision tambien ve las lineas del suelo. Se registra siempre,
        aunque haya sensor de color: el contador de vueltas usa las dos y una
        tapa los fallos de la otra."""
        if linea != P.LINEA_NINGUNA:
            with self._lock:
                self.eventos.append((time.time(), linea, "camara"))

    # ------------------------------------------------------------------
    def actualizar(self) -> None:
        """Se llama una vez por frame. Decide de donde sale cada dato."""
        ahora = time.time()
        pref_r = str(self.cfg.get("origen_rumbo", "auto")).lower()
        pref_c = str(self.cfg.get("origen_color", "auto")).lower()

        esp32_imu_vivo = (ahora - self._t_yaw_esp32) < CADUCIDAD_ESP32
        esp32_color_vivo = ((ahora - self._t_estado_esp32) < 5.0 and self.esp32.tcs)
        pi_imu_vivo = self.imu_pi is not None and self.imu_pi.disponible

        with self._lock:
            # ---- rumbo ---------------------------------------------------
            if pref_r != "ninguno" and pref_r in ("auto", "esp32") and esp32_imu_vivo:
                self.origen_rumbo = "esp32"
                self.yaw = _norm(self._yaw_esp32 - self._offset_yaw)
                self.giro_z = self._giro_esp32
                self.rumbo_calibrado = self._cal_esp32
            elif pref_r != "ninguno" and pref_r in ("auto", "pi") and pi_imu_vivo:
                self.origen_rumbo = "pi"
                self.yaw = _norm(self.imu_pi.yaw - self._offset_yaw)
                self.giro_z = 0.0
                self.rumbo_calibrado = True
            else:
                self.origen_rumbo = "ninguno"
                self.rumbo_calibrado = False

            # ---- color ---------------------------------------------------
            if pref_c in ("auto", "esp32") and esp32_color_vivo:
                self.origen_color = "esp32"
            elif pref_c in ("auto", "camara", "pi"):
                self.origen_color = "camara"
            else:
                self.origen_color = "ninguno"

    @property
    def hay_rumbo(self) -> bool:
        return self.origen_rumbo != "ninguno"

    def yaw_o_none(self) -> Optional[float]:
        return self.yaw if self.hay_rumbo else None

    # ------------------------------------------------------------------
    def poner_cero(self, enlace=None) -> None:
        """Pone el rumbo actual como cero. Se lo pide al ESP32 si el sensor
        vive alli, y ademas guarda un desfase local para que el numero cuadre
        al instante sin esperar a la siguiente trama."""
        with self._lock:
            if self.origen_rumbo == "esp32":
                self._offset_yaw = self._yaw_esp32
            elif self.origen_rumbo == "pi" and self.imu_pi is not None:
                self._offset_yaw = self.imu_pi.yaw
            self.yaw = 0.0
        if enlace is not None:
            enlace.pedir_a_sensores(P.AUX_CERO_YAW)

    def calibrar_rumbo(self, enlace=None) -> None:
        if self.origen_rumbo == "esp32" and enlace is not None:
            enlace.pedir_a_sensores(P.AUX_CALIB_IMU)
        elif self.imu_pi is not None:
            threading.Thread(target=self.imu_pi.calibrar, daemon=True).start()
        with self._lock:
            self._offset_yaw = 0.0

    def calibrar_color(self, enlace=None) -> None:
        if enlace is not None:
            enlace.pedir_a_sensores(P.AUX_CALIB_COLOR)

    # ------------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "rumbo": {
                "origen": self.origen_rumbo,
                "yaw": round(self.yaw, 1),
                "giro_z": round(self.giro_z, 1),
                "calibrado": self.rumbo_calibrado,
            },
            "color": {
                "origen": self.origen_color,
                "linea": P.NOMBRE_LINEA.get(self.ultima_linea, "?"),
                "rgb": list(self.rgb),
                "luz": self.luz,
            },
            "esp32": {
                "mpu": self.esp32.mpu, "tcs": self.esp32.tcs,
                "hz_imu": self.esp32.hz_imu, "hz_color": self.esp32.hz_color,
            },
        }

    # atributos que solo existen tras la primera trama
    _yaw_esp32: float = 0.0
    _giro_esp32: float = 0.0
    _cal_esp32: bool = False


def _norm(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0
