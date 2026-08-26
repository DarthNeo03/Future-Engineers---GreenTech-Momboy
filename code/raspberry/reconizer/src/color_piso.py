"""
color_piso.py — TCS34725 mirando al suelo: cruces de linea naranja y azul.

============================================================================
POR QUE UN SENSOR Y NO LA CAMARA
============================================================================
La camara ya ve el suelo, asi que la pregunta es legitima. Tres razones,
en orden de peso:

1. ILUMINACION. Todo el sistema HSV depende de la luz del pabellon, y una
   linea naranja sobre blanco es de lo mas fragil que hay. El TCS34725 lleva
   su PROPIO LED a 10-15 mm del suelo: la luz ambiente queda despreciable
   frente a la propia. Es inmune justo a lo que mas se rompe el dia de la
   competencia.

2. GEOMETRIA. La camara mira al frente y ve el suelo en escorzo extremo: una
   linea de 20 mm a un metro ocupa 2-3 pixeles de alto. Y la franja inferior
   esta descartada por `ignorar_abajo`, que es justo donde mejor se veria.

3. ES UN EVENTO CON MARCA DE TIEMPO. La camara dice "hay algo naranja ahi
   adelante"; el sensor dice "acabo de cruzar, AHORA". Para contar vueltas y
   sobre todo para disparar la parada final -donde el margen es de unos
   +-150 mm- esa diferencia decide si se consiguen los 8 puntos.

============================================================================
EL TIEMPO DE INTEGRACION ES EL PARAMETRO QUE DECIDE TODO
============================================================================
Es el error que hace que a la mayoria "no le funcione" el sensor de color, y
no es evidente. El TCS34725 integra entre 2,4 y 700 ms. A 0,4 m/s una linea
de 20 mm pasa por debajo del sensor en 50 ms:

    integracion 700 ms  -> la linea es el 7% de la muestra: invisible
    integracion 154 ms  -> la linea es el 32%: se diluye con el blanco
    integracion  24 ms  -> caben 2 muestras dentro de la linea  <- bien
    integracion 2,4 ms  -> caben 20 muestras                    <- mejor

Por defecto se usan 24 ms, que da ~40 Hz y es de sobra. Bajar a 2,4 ms
multiplica el ruido sin ganar nada a estas velocidades.

============================================================================
COMO SE CLASIFICA
============================================================================
No se usan los valores crudos: se normaliza cada canal por el canal CLEAR.
Eso quita el brillo de la ecuacion y deja solo el color, asi que la
clasificacion aguanta que el sensor quede un poco mas alto o mas bajo, que
el LED envejezca o que el tapete tenga brillo desigual. Luego se compara con
los perfiles medidos con `tools/calibrar_piso.py` y gana el mas cercano, si
esta lo bastante cerca; si no, "desconocido", que NO es lo mismo que blanco.

OPCIONAL DE VERDAD: si no hay chip, si no hay bus I2C (Windows) o si se
suelta un cable a mitad de carrera, `disponible` queda en False y el resto
del programa sigue igual. Nadie llama a este modulo sin comprobarlo antes.

Cableado (Pi 5, cabecera de 40 pines) — comparte bus con el MPU6050:
    VIN -> pin 17 (3V3)      SDA -> pin 3  (GPIO2)
    GND -> pin 9  (GND)      SCL -> pin 5  (GPIO3)
La direccion 0x29 es FIJA y no se puede cambiar: solo cabe un TCS34725 por
bus. Para dos harian falta un multiplexor TCA9548A.
Comprobar:  i2cdetect -y 1   (debe salir 29, y 68 o 69 del MPU6050)
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

# --- registros del TCS34725 -------------------------------------------------
DIR_I2C = 0x29
CMD = 0x80                # bit de comando, obligatorio en cada acceso
CMD_AUTO = 0xA0           # comando + autoincremento, para leer los 8 bytes
REG_ENABLE = 0x00
REG_ATIME = 0x01
REG_CONTROL = 0x0F
REG_ID = 0x12
REG_CDATAL = 0x14

ENABLE_PON = 0x01         # alimentar el oscilador
ENABLE_AEN = 0x02         # habilitar el ADC

IDS_VALIDOS = (0x44, 0x4D)   # 0x44 = TCS34725, 0x4D = TCS34727

# gain -> valor de registro
GANANCIAS = {1: 0x00, 4: 0x01, 16: 0x02, 60: 0x03}

BLANCO = "blanco"
DESCONOCIDO = "desconocido"


def atime_desde_ms(ms: float) -> int:
    """Milisegundos -> registro ATIME. t = (256 - ATIME) * 2,4 ms."""
    pasos = max(1, min(256, int(round(ms / 2.4))))
    return (256 - pasos) & 0xFF


def ms_desde_atime(atime: int) -> float:
    return (256 - atime) * 2.4


# ---------------------------------------------------------------------------
@dataclass
class Cruce:
    """Una linea cruzada. Es el evento que consume el resto del programa."""
    color: str
    t: float                  # time.monotonic() del inicio del cruce
    duracion_s: float = 0.0
    muestras: int = 0


@dataclass
class Perfil:
    """Color calibrado, en cromaticidad (r, g, b normalizados por clear)."""
    nombre: str
    r: float
    g: float
    b: float
    tol: float = 0.06         # radio de aceptacion en ese espacio

    def distancia(self, r: float, g: float, b: float) -> float:
        return math.sqrt((r - self.r) ** 2 + (g - self.g) ** 2 + (b - self.b) ** 2)


# Perfiles de partida. Son un punto de arranque razonable, NO una calibracion:
# el color que ve un sensor depende de su LED, de la altura y del tapete.
# Hay que medirlos con tools/calibrar_piso.py sobre la lona de verdad.
PERFILES_POR_DEFECTO = [
    {"nombre": BLANCO, "r": 0.33, "g": 0.34, "b": 0.33, "tol": 0.05},
    {"nombre": "naranja", "r": 0.55, "g": 0.30, "b": 0.15, "tol": 0.09},
    {"nombre": "azul", "r": 0.20, "g": 0.32, "b": 0.48, "tol": 0.09},
]


# ---------------------------------------------------------------------------
class SensorPiso:
    """Lee el TCS34725 en su propio hilo y publica cruces de linea.

        s = SensorPiso(cfg["piso"])
        if s.iniciar():
            for cruce in s.tomar_cruces():
                print(cruce.color)
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg or {}
        self.disponible = False
        self.motivo = "sin iniciar"
        self.color = BLANCO
        self.crudo: Tuple[int, int, int, int] = (0, 0, 0, 0)
        self.cromatico: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.hz_real = 0.0
        self.cuenta: Dict[str, int] = {}

        self._bus = None
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._lock = threading.Lock()
        self._cruces: List[Cruce] = []
        self._perfiles = [Perfil(**p) for p in
                          (self.cfg.get("perfiles") or PERFILES_POR_DEFECTO)]

        # deteccion de cruce
        self._en_linea: Optional[str] = None
        self._t_linea = 0.0
        self._n_linea = 0
        self._t_ultimo_cruce = -1e9

    # -- arranque ---------------------------------------------------------
    def iniciar(self) -> bool:
        if not bool(self.cfg.get("activo", True)):
            self.motivo = "desactivado en la configuracion"
            return False
        try:
            from smbus2 import SMBus
        except Exception as e:
            self.motivo = f"sin smbus2 ({e})"
            return False

        bus_n = int(self.cfg.get("bus", 1))
        try:
            bus = SMBus(bus_n)
        except Exception as e:
            self.motivo = f"no se pudo abrir el bus I2C {bus_n} ({e})"
            return False

        try:
            ident = bus.read_byte_data(DIR_I2C, CMD | REG_ID)
        except Exception as e:
            bus.close()
            self.motivo = f"no responde en 0x{DIR_I2C:02X} ({e})"
            return False

        if ident not in IDS_VALIDOS:
            bus.close()
            self.motivo = f"ID inesperado 0x{ident:02X} en 0x{DIR_I2C:02X}"
            return False

        try:
            self._configurar(bus)
        except Exception as e:
            bus.close()
            self.motivo = f"no se pudo configurar ({e})"
            return False

        self._bus = bus
        self.disponible = True
        ms = float(self.cfg.get("integracion_ms", 24.0))
        self.motivo = (f"TCS34725 en bus {bus_n}, integracion {ms:.1f} ms, "
                       f"ganancia x{int(self.cfg.get('ganancia', 4))}")
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="piso")
        self._hilo.start()
        return True

    def _configurar(self, bus) -> None:
        bus.write_byte_data(DIR_I2C, CMD | REG_ENABLE, ENABLE_PON)
        time.sleep(0.003)                       # el oscilador tarda 2,4 ms
        atime = atime_desde_ms(float(self.cfg.get("integracion_ms", 24.0)))
        bus.write_byte_data(DIR_I2C, CMD | REG_ATIME, atime)
        g = int(self.cfg.get("ganancia", 4))
        bus.write_byte_data(DIR_I2C, CMD | REG_CONTROL, GANANCIAS.get(g, 0x01))
        bus.write_byte_data(DIR_I2C, CMD | REG_ENABLE, ENABLE_PON | ENABLE_AEN)
        time.sleep(ms_desde_atime(atime) / 1000.0 + 0.005)

    def parar(self) -> None:
        self._parar.set()
        if self._hilo:
            self._hilo.join(timeout=1.0)
        if self._bus is not None:
            try:
                self._bus.write_byte_data(DIR_I2C, CMD | REG_ENABLE, 0x00)
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        self.disponible = False

    # -- lectura ----------------------------------------------------------
    def _leer(self) -> Tuple[int, int, int, int]:
        d = self._bus.read_i2c_block_data(DIR_I2C, CMD_AUTO | REG_CDATAL, 8)
        c = d[0] | (d[1] << 8)
        r = d[2] | (d[3] << 8)
        g = d[4] | (d[5] << 8)
        b = d[6] | (d[7] << 8)
        return c, r, g, b

    def clasificar(self, c: int, r: int, g: int, b: int) -> str:
        """Cromaticidad -> nombre de color, o DESCONOCIDO.

        Se divide por el CLEAR y no por la suma: clear es la medida de brillo
        del propio sensor y es lo que compensa que el LED envejezca o que el
        sensor quede un poco mas alto de la cuenta.
        """
        c_min = int(self.cfg.get("clear_min", 60))
        if c < c_min:
            return DESCONOCIDO                  # a oscuras no se decide nada
        if c >= int(self.cfg.get("clear_max", 65535)):
            return DESCONOCIDO                  # saturado: la medida no vale
        s = float(r + g + b)
        if s <= 0:
            return DESCONOCIDO
        rn, gn, bn = r / s, g / s, b / s
        self.cromatico = (rn, gn, bn)

        mejor, d_mejor = DESCONOCIDO, 1e9
        for p in self._perfiles:
            d = p.distancia(rn, gn, bn)
            if d < d_mejor and d <= p.tol:
                mejor, d_mejor = p.nombre, d
        return mejor

    # -- hilo -------------------------------------------------------------
    def _bucle(self) -> None:
        periodo = ms_desde_atime(
            atime_desde_ms(float(self.cfg.get("integracion_ms", 24.0)))) / 1000.0
        min_muestras = int(self.cfg.get("muestras_min", 2))
        separacion = float(self.cfg.get("separacion_min_s", 0.35))
        t_prev = time.monotonic()

        while not self._parar.is_set():
            try:
                c, r, g, b = self._leer()
            except Exception as e:
                self.disponible = False
                self.motivo = f"lectura fallida ({e})"
                return

            ahora = time.monotonic()
            dt = ahora - t_prev
            t_prev = ahora
            if dt > 0:
                self.hz_real = 0.9 * self.hz_real + 0.1 / dt

            color = self.clasificar(c, r, g, b)
            self.crudo = (c, r, g, b)
            self.color = color
            self.cuenta[color] = self.cuenta.get(color, 0) + 1

            es_linea = color not in (BLANCO, DESCONOCIDO)
            if es_linea:
                if self._en_linea == color:
                    self._n_linea += 1
                else:
                    # Empieza una linea (o cambia de color a mitad, que solo
                    # pasa con ruido: se reinicia la cuenta y ya).
                    self._en_linea = color
                    self._t_linea = ahora
                    self._n_linea = 1
            elif self._en_linea is not None:
                # Se acaba de salir de la linea: aqui es donde se emite, no al
                # entrar. Asi se conoce la duracion y se pueden descartar los
                # destellos de una sola muestra.
                if (self._n_linea >= min_muestras
                        and (self._t_linea - self._t_ultimo_cruce) > separacion):
                    cr = Cruce(color=self._en_linea, t=self._t_linea,
                               duracion_s=ahora - self._t_linea,
                               muestras=self._n_linea)
                    with self._lock:
                        self._cruces.append(cr)
                        if len(self._cruces) > 64:
                            del self._cruces[:32]
                    self._t_ultimo_cruce = self._t_linea
                self._en_linea = None
                self._n_linea = 0

            resto = periodo - (time.monotonic() - ahora)
            if resto > 0:
                time.sleep(resto)

    # -- API para el robot -------------------------------------------------
    def tomar_cruces(self) -> List[Cruce]:
        """Devuelve los cruces pendientes y vacia la cola.

        Se consume, no se consulta: si el lazo de control se salta un frame no
        puede perderse un cruce, que es un evento que ocurre una sola vez.
        """
        with self._lock:
            fuera, self._cruces = self._cruces, []
        return fuera

    def estado(self) -> Dict[str, Any]:
        c, r, g, b = self.crudo
        rn, gn, bn = self.cromatico
        return {
            "disponible": bool(self.disponible),
            "motivo": self.motivo,
            "color": self.color,
            "hz": round(self.hz_real, 1),
            "crudo": {"c": c, "r": r, "g": g, "b": b},
            "cromatico": [round(rn, 3), round(gn, 3), round(bn, 3)],
        }


# ---------------------------------------------------------------------------
def sentido_desde_orden(primero: str, segundo: str,
                        orden_horario: Tuple[str, str] = ("naranja", "azul")
                        ) -> int:
    """Sentido de la vuelta a partir de dos lineas seguidas.

    Devuelve +1 si el muro interno queda a la DERECHA (vuelta horaria), -1 si
    queda a la izquierda (antihoraria), 0 si no se puede decidir.

    OJO: `orden_horario` hay que CONFIRMARLO sobre el tapete. El reglamento
    2026 solo especifica que las lineas existen, su grosor (20 mm) y su color
    (CMYK), pero no dice donde estan ni en que orden: eso solo aparece en el
    plano del campo. Mientras no este verificado, este atajo se queda
    desactivado y el sentido se deduce de la primera esquina, que es
    geometrico y no depende de ningun plano.
    """
    if primero == segundo or BLANCO in (primero, segundo):
        return 0
    if (primero, segundo) == tuple(orden_horario):
        return +1
    if (segundo, primero) == tuple(orden_horario):
        return -1
    return 0
