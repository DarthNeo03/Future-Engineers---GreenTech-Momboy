"""
enlace.py — Hilo que habla con el ESP32 por serial.

Autodeteccion del puerto: se prueban los candidatos en orden, se manda un PING
y se espera respuesta. Como el firmware contesta por la boca por la que recibio
la ultima trama valida, el PING sirve a la vez de "hola" y de "contesta por
aqui". Asi el mismo codigo funciona con el cable en los GPIO o en el USB, y en
Windows con COMx, sin tocar nada.

Orden de busqueda:
    Linux : /dev/serial0, /dev/ttyAMA0, /dev/ttyAMA1, /dev/ttyS0,
            /dev/ttyUSB*, /dev/ttyACM*
    Windows: COM1..COM32 (o lo que liste pyserial)

El hilo manda un mando cada 20 ms (50 Hz) pase lo que pase. Si el lazo de
control de arriba deja de refrescar la orden, se manda velocidad 0: el
failsafe del ESP32 es la ultima red, no la primera.
"""

from __future__ import annotations

import glob
import platform
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from . import protocolo as P

ES_WINDOWS = platform.system().lower().startswith("win")


def candidatos(preferido: str = "") -> List[str]:
    lista: List[str] = []
    if preferido:
        lista.append(preferido)
    if ES_WINDOWS:
        try:
            from serial.tools import list_ports
            lista += [p.device for p in list_ports.comports()]
        except Exception:
            lista += [f"COM{i}" for i in range(1, 33)]
    else:
        lista += ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyAMA1", "/dev/ttyS0"]
        lista += sorted(glob.glob("/dev/ttyUSB*"))
        lista += sorted(glob.glob("/dev/ttyACM*"))
    vistos = set()
    salida = []
    for p in lista:
        if p and p not in vistos:
            vistos.add(p)
            salida.append(p)
    return salida


class Enlace:
    """Envio periodico + recepcion de telemetria, en un hilo aparte."""

    def __init__(self, cfg: Dict[str, Any], simulado: bool = False,
                 al_log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg or {}
        self.simulado = simulado
        self.al_log = al_log or (lambda s: None)

        self.conectado = False
        self.puerto: str = ""
        self.motivo = "sin iniciar"
        self.buscando = False
        self.telemetria = P.Telemetria()
        self.ultima_tele = 0.0
        self.latencia_ms = 0.0
        self.enviados = 0
        self.recibidos = 0
        self.errores_crc = 0

        self._ser = None
        self._lector = P.Lector()
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._lock = threading.Lock()

        self._seq = 0
        self._vel = 0
        self._dir = 0
        self._vmax = 130
        self._armado = False
        self._parada = False
        self._centrar = False
        self._cal_imu = 0            # tramas que quedan pidiendo calibracion
        self._t_mando = 0.0
        self._config_pendiente: Optional[bytes] = None
        self._ping_t: Dict[int, float] = {}

    # -- API para el lazo de control --------------------------------------
    def mandar(self, vel: int, direccion: int, armado: bool = True) -> None:
        with self._lock:
            self._vel = int(max(-100, min(100, vel)))
            self._dir = int(max(-100, min(100, direccion)))
            self._armado = bool(armado)
            self._t_mando = time.time()

    def parar(self, emergencia: bool = False) -> None:
        with self._lock:
            self._vel = 0
            self._armado = False
            self._parada = bool(emergencia)
            self._t_mando = time.time()

    def rearmar(self) -> None:
        with self._lock:
            self._parada = False

    def fijar_vmax(self, vmax: int) -> None:
        """Tope absoluto de PWM. Viaja en cada trama, asi que el ESP32 lo aplica
        aunque la Pi se reinicie a mitad de prueba."""
        with self._lock:
            self._vmax = int(max(0, min(255, vmax)))

    def centrar_servo(self, activo: bool = True) -> None:
        with self._lock:
            self._centrar = bool(activo)

    def calibrar_imu(self, tramas: int = 10) -> None:
        """Pide al ESP32 que recalibre el giroscopio. El carro tiene que estar
        QUIETO. Se manda la bandera en varias tramas seguidas porque una sola
        se puede perder, y el ESP32 la trata como idempotente."""
        with self._lock:
            self._cal_imu = max(1, int(tramas))

    def enviar_config(self, centro: int, izq: int, der: int,
                      rampa: int, grados_s: int) -> None:
        with self._lock:
            self._config_pendiente = P.empaquetar_config(centro, izq, der, rampa, grados_s)

    # -- ciclo de vida ----------------------------------------------------
    def iniciar(self) -> None:
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="enlace")
        self._hilo.start()

    def cerrar(self) -> None:
        self._parar.set()
        if self._hilo:
            self._hilo.join(timeout=2.0)
        self._cerrar_puerto()

    def _cerrar_puerto(self):
        if self._ser is not None:
            try:
                # Ultimo mando: todo a cero, por si el ESP32 sigue vivo
                self._ser.write(P.Mando(seq=self._seq, vel=0, direccion=0,
                                        vmax=0, armado=False).a_bytes())
                self._ser.flush()
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.conectado = False

    # -- deteccion --------------------------------------------------------
    def _abrir(self) -> bool:
        try:
            import serial
        except Exception as e:
            self.motivo = f"falta pyserial ({e}); pip install pyserial"
            return False

        baud = int(self.cfg.get("baudios", 115200))
        self.buscando = True
        self.motivo = "buscando el ESP32..."
        for puerto in candidatos(str(self.cfg.get("puerto", ""))):
            try:
                s = serial.Serial(puerto, baud, timeout=0, write_timeout=0.5)
            except Exception:
                continue
            try:
                # Solo los puertos USB reinician el ESP32 al abrirse (DTR/RTS);
                # esperar 350 ms en CADA candidato haria que la busqueda tardara
                # una eternidad cuando hay varios ttyS/ttyAMA sueltos.
                if "USB" in puerto.upper() or "ACM" in puerto.upper() or \
                        puerto.upper().startswith("COM"):
                    time.sleep(0.35)
                s.reset_input_buffer()
                lector = P.Lector()
                visto = False
                for intento in range(3):
                    s.write(P.empaquetar(P.TIPO_PING, bytes((intento,))))
                    t0 = time.time()
                    while time.time() - t0 < 0.2:
                        datos = s.read(256)
                        if datos:
                            for tipo, _pl in lector.alimentar(datos):
                                if tipo in (P.TIPO_PONG, P.TIPO_TELE, P.TIPO_LOG):
                                    visto = True
                                    break
                        if visto:
                            break
                        time.sleep(0.01)
                    if visto:
                        break
                if not visto:
                    s.close()
                    continue
            except Exception:
                try:
                    s.close()
                except Exception:
                    pass
                continue

            self._ser = s
            self.puerto = puerto
            self.conectado = True
            self.buscando = False
            self.motivo = f"conectado a {puerto} @ {baud}"
            self._lector = P.Lector()
            self.al_log(f"[enlace] {self.motivo}")
            return True

        self.buscando = False
        self.motivo = "no se encontro el ESP32 en ningun puerto"
        return False

    # -- hilo -------------------------------------------------------------
    def _bucle(self):
        hz = max(5, int(self.cfg.get("hz_envio", 50)))
        periodo = 1.0 / hz
        timeout_tele = float(self.cfg.get("timeout_tele_ms", 500)) / 1000.0
        reintento = float(self.cfg.get("reintento_s", 2.0))
        t_ping = 0.0
        t_ultimo_intento = 0.0

        while not self._parar.is_set():
            t0 = time.perf_counter()

            if self.simulado:
                self.conectado = True
                self.motivo = "modo simulado (sin ESP32)"
                time.sleep(periodo)
                continue

            if self._ser is None:
                if time.time() - t_ultimo_intento >= reintento:
                    t_ultimo_intento = time.time()
                    if not self._abrir():
                        self.al_log(f"[enlace] {self.motivo}")
                if self._ser is None:
                    time.sleep(0.2)
                    continue

            # ---- enviar ---------------------------------------------------
            try:
                with self._lock:
                    edad = time.time() - self._t_mando
                    # Si el lazo de arriba se durmio, no repetimos su ultima
                    # orden: mandamos cero. Repetir una orden vieja es lo que
                    # hace que un carro siga a fondo contra la pared.
                    vencido = edad > 0.25
                    m = P.Mando(
                        seq=self._seq,
                        vel=0 if (vencido or self._parada) else self._vel,
                        direccion=self._dir,
                        vmax=self._vmax,
                        armado=self._armado and not vencido and not self._parada,
                        parada=self._parada,
                        centrar=self._centrar,
                        cal_imu=self._cal_imu > 0,
                    )
                    if self._cal_imu > 0:
                        self._cal_imu -= 1
                    cfg_pend = self._config_pendiente
                    self._config_pendiente = None
                self._seq = (self._seq + 1) & 0xFF
                if cfg_pend:
                    self._ser.write(cfg_pend)
                self._ser.write(m.a_bytes())
                self.enviados += 1

                if time.time() - t_ping > 1.0:
                    t_ping = time.time()
                    n = self._seq
                    self._ping_t[n] = t_ping
                    if len(self._ping_t) > 8:
                        self._ping_t.pop(next(iter(self._ping_t)))
                    self._ser.write(P.empaquetar(P.TIPO_PING, bytes((n,))))
            except Exception as e:
                self.al_log(f"[enlace] error escribiendo: {e}")
                self._cerrar_puerto()
                continue

            # ---- recibir --------------------------------------------------
            try:
                datos = self._ser.read(512)
            except Exception as e:
                self.al_log(f"[enlace] error leyendo: {e}")
                self._cerrar_puerto()
                continue

            if datos:
                for tipo, pl in self._lector.alimentar(datos):
                    self.recibidos += 1
                    if tipo == P.TIPO_TELE and len(pl) >= 8:
                        self.telemetria = P.Telemetria.desde_payload(pl)
                        self.ultima_tele = time.time()
                    elif tipo == P.TIPO_PONG and pl:
                        t = self._ping_t.pop(pl[0], None)
                        if t:
                            self.latencia_ms = (time.time() - t) * 1000.0
                    elif tipo == P.TIPO_LOG:
                        self.al_log("[esp32] " + pl.decode("ascii", "replace"))
                self.errores_crc = self._lector.crc_malos

            if self.ultima_tele and (time.time() - self.ultima_tele) > timeout_tele:
                self.al_log("[enlace] sin telemetria, reabriendo puerto")
                self._cerrar_puerto()
                continue

            resto = periodo - (time.perf_counter() - t0)
            if resto > 0:
                time.sleep(resto)

    # -- info -------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        t = self.telemetria
        return {
            "conectado": bool(self.conectado),
            "puerto": self.puerto,
            "motivo": self.motivo,
            "latencia_ms": round(self.latencia_ms, 1),
            "enviados": self.enviados,
            "recibidos": self.recibidos,
            "crc_malos": self.errores_crc,
            "tele": {
                "armado": t.armado, "motor": t.motor, "failsafe": t.failsafe,
                "servo_tope": t.servo_en_tope, "pwm": t.pwm, "angulo": t.angulo,
                "ms_desde_mando": t.ms_desde_mando, "tramas_malas": t.tramas_malas,
                "version": t.version,
            },
        }
