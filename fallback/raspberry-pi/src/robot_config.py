"""
robot_config.py — Ajustes del robot (config/robot.json).

Separado de colors.json a proposito: los colores se recalibran en cada pista y
rotan entre 5 perfiles; esto otro (puerto serie, ganancias, limites) cambia poco
y no quieres perderlo al cambiar de perfil de color.

Todo se puede tocar en caliente desde el panel o desde la web y guardarse.
"""

from __future__ import annotations

import copy
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional, Union

RAIZ_PROYECTO = Path(__file__).resolve().parent.parent
RUTA_ROBOT = RAIZ_PROYECTO / "config" / "robot.json"

POR_DEFECTO: Dict[str, Any] = {
    "version": 1,

    "enlace": {
        # vacio = autodeteccion. Orden de busqueda en Linux:
        #   /dev/serial0, /dev/ttyAMA0, /dev/ttyAMA1, /dev/ttyUSB*, /dev/ttyACM*
        # y en Windows todos los COM. Se prueba cada uno y se queda con el
        # primero que conteste una trama de telemetria valida.
        "puerto": "",
        "baudios": 115200,
        "hz_envio": 50,          # tramas de mando por segundo
        "timeout_tele_ms": 500,  # sin telemetria = enlace caido
        "reintento_s": 2.0,
    },

    "camara": {
        "indice": 0, "ancho": 640, "alto": 480, "fps": 30,
        "fourcc": "MJPG", "voltear": False,
    },

    "limites": {
        "vmax": 130,             # tope de PWM 0-255 que el ESP32 nunca supera
        "vel_crucero": 55,       # % de vmax en recta
        "vel_giro": 38,          # % de vmax girando
        "dir_max": 100,          # % de direccion que se permite pedir
    },

    "navegacion": {
        "estrategia": "centrado",       # "centrado" | "pared"
        "color_muro": "negro",

        # --- lectura del muro ---
        "px_min_columna": 6,     # menos pixeles negros que esto en una columna
                                 # = ahi no hay muro (mata el ruido)
        "suavizado": 15,         # promedio movil del perfil, en columnas
        "ignorar_abajo": 0.06,   # franja inferior tapada por el chasis

        # --- zonas ---
        "banda_lateral": 0.28,   # ancho de las bandas izquierda y derecha
        "ruedas_izq": 0.32,      # por donde pasan las ruedas (la camara no las ve)
        "ruedas_der": 0.68,

        # --- PD del centrado ---
        "kp": 95.0,
        "kd": 22.0,

        # --- PD del seguimiento de pared ---
        "lado_pared": "izq",
        "pared_objetivo": 0.45,
        "kp_pared": 130.0,
        "kd_pared": 28.0,

        # --- umbrales (sobre el espacio libre normalizado 0..1) ---
        "girar_bajo": 0.40,      # por debajo de esto, hay esquina: girar
        "salir_giro_sobre": 0.55,
        "frenar_bajo": 0.50,     # empieza a bajar la velocidad
        "parar_bajo": 0.24,      # parada de seguridad
        "giro_max_ms": 3000,
        "min_recto_ms": 600,     # espera minima entre dos esquinas seguidas
        "dir_giro": 90.0,        # % de direccion durante el giro

        # --- giroscopio (si hay MPU6050) ---
        "usar_yaw": True,
        "yaw_kp": 1.6,           # % de direccion por grado de error
        "yaw_max": 45.0,         # tope de la correccion por yaw
        "giro_grados": 90.0,     # la pista es cuadrada
        "giro_tolerancia": 8.0,
    },

    "imu": {
        "activo": True,          # si no aparece el chip, se sigue sin el
        "bus": 1,
        "direcciones": [104, 105],   # 0x68 y 0x69
        "hz": 100,
        "alfa_complementario": 0.98,
        "muestras_calibracion": 400,
    },

    "red": {
        "puerto_http": 8080,
        "calidad_jpeg": 70,
        "fps_stream": 15,
        "ancho_stream": 640,
        "hostname": "carrito",
    },
}


def _fusionar(base: Dict[str, Any], encima: Any) -> Dict[str, Any]:
    salida = copy.deepcopy(base)
    if isinstance(encima, dict):
        for k, v in encima.items():
            if k in salida and isinstance(salida[k], dict) and isinstance(v, dict):
                salida[k] = _fusionar(salida[k], v)
            else:
                salida[k] = v
    return salida


def cargar(ruta: Optional[Union[str, Path]] = None) -> Dict[str, Any]:
    """Nunca falla: si el archivo no existe o esta roto, devuelve los valores
    por defecto. Las claves nuevas de una version futura aparecen solas."""
    ruta = Path(ruta) if ruta else RUTA_ROBOT
    if not ruta.exists():
        cfg = copy.deepcopy(POR_DEFECTO)
        guardar(cfg, ruta)
        return cfg
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            datos = json.load(f)
    except Exception as e:
        print(f"[robot_config] {ruta} ilegible ({e}); uso valores por defecto")
        return copy.deepcopy(POR_DEFECTO)
    return _fusionar(POR_DEFECTO, datos)


def guardar(cfg: Dict[str, Any], ruta: Optional[Union[str, Path]] = None) -> Path:
    ruta = Path(ruta) if ruta else RUTA_ROBOT
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(ruta.parent), prefix=".robot_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(_fusionar(POR_DEFECTO, cfg), f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, ruta)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return ruta
