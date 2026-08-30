"""
enlace.py — Hilo que habla con el ESP32 por serial (USB o GPIO).

Autodeteccion del puerto: se prueban los candidatos, se manda un PING y se
espera respuesta. El firmware contesta por la boca por la que recibio la
ultima trama valida, asi que el mismo codigo funciona por USB o por GPIO.

El hilo manda un mando cada 20 ms (50 Hz) pase lo que pase. Si el lazo de
vision se atasca >250 ms se manda velocidad CERO, no la ultima orden:
repetir una orden vieja es exactamente lo que empotra un carro en la pared.

NUEVO (v2): el ESP32 ahora manda la trama de SENSORES (yaw del MPU6050 +
color del TCS34725 + contadores de cruce de linea). Aqui se convierten los
contadores en EVENTOS con hora local: los consume lineas.py.
"""

from __future__ import annotations

import glob
import platform
import threading
import time
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Tuple

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
    def __init__(self, cfg: Dict[str, Any], simulado: bool = False,
                 al_log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg or {}
        self.simulado = simulado
        self.al_log = al_log or (lambda s: None)

        self.conectado = False
        self.puerto: str = ""
        self.motivo = "sin iniciar"
        self.telemetria = P.Telemetria()
        self.sensores = P.Sensores()
        self.ultima_tele = 0.0
        self.ultimos_sensores = 0.0
        self.latencia_ms = 0.0
        self.enviados = 0
        self.recibidos = 0
        self.errores_crc = 0
        self.historial_tcs: deque = deque(maxlen=60)   # (c,r,g,b) para calibrar

        self._eventos_linea: deque = deque(maxlen=32)  # (color, t)
        self._cnt_prev: Optional[Tuple[int, int]] = None

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
        self._t_mando = 0.0
        self._pendientes: List[bytes] = []      # config/cal por mandar
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
        with self._lock:
            self._vmax = int(max(0, min(255, vmax)))

    def enviar_config_servo(self, centro: int, izq: int, der: int,
                            rampa: int, grados_s: int) -> None:
        with self._lock:
            self._pendientes.append(
                P.empaquetar_config(centro, izq, der, rampa, grados_s))

    def enviar_cfg_tcs(self, tcs: Dict[str, Any]) -> None:
        """tcs: el grupo 'tcs' de params.py tal cual."""
        with self._lock:
            self._pendientes.append(P.empaquetar_cfg_tcs(
                int(tcs.get("c_min", 80)),
                int(tcs.get("naranja_r_min", 120)), int(tcs.get("naranja_b_max", 60)),
                int(tcs.get("azul_b_min", 110)), int(tcs.get("azul_r_max", 70)),
                int(tcs.get("muestras_min", 1)), int(tcs.get("refractario_ds", 3)),
                int(tcs.get("atime", 246)), int(tcs.get("gain", 2))))

    def enviar_cal(self, cmd: int) -> None:
        with self._lock:
            self._pendientes.append(P.empaquetar_cal(cmd))

    def eventos_linea(self) -> List[Tuple[str, float]]:
        """Drena los cruces de linea detectados por el TCS desde la ultima
        llamada. Cada uno es ('naranja'|'azul', tiempo_local)."""
        salida = []
        while self._eventos_linea:
            salida.append(self._eventos_linea.popleft())
        return salida

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
        self.motivo = "buscando el ESP32..."
        for puerto in candidatos(str(self.cfg.get("puerto", ""))):
            try:
                s = serial.Serial(puerto, baud, timeout=0, write_timeout=0.5)
            except Exception:
                continue
            try:
                # Los puertos USB reinician el ESP32 al abrirse (DTR/RTS)
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
                                if tipo in (P.TIPO_PONG, P.TIPO_TELE,
                                            P.TIPO_LOG, P.TIPO_SENSORES):
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
            self.motivo = f"conectado a {puerto} @ {baud}"
            self._lector = P.Lector()
            self._cnt_prev = None
            self.al_log(f"[enlace] {self.motivo}")
            return True

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
                    vencido = edad > 0.25
                    m = P.Mando(
                        seq=self._seq,
                        vel=0 if (vencido or self._parada) else self._vel,
                        direccion=self._dir,
                        vmax=self._vmax,
                        armado=self._armado and not vencido and not self._parada,
                        parada=self._parada,
                        centrar=self._centrar,
                    )
                    pendientes = self._pendientes
                    self._pendientes = []
                self._seq = (self._seq + 1) & 0xFF
                for tr in pendientes:
                    self._ser.write(tr)
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
                datos = self._ser.read(1024)
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
                    elif tipo == P.TIPO_SENSORES and len(pl) >= 14:
                        self._procesar_sensores(P.Sensores.desde_payload(pl))
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

    def _procesar_sensores(self, s: P.Sensores) -> None:
        self.sensores = s
        self.ultimos_sensores = time.time()
        if s.tcs_ok:
            self.historial_tcs.append((s.c, s.r, s.g, s.b))
        # contadores -> eventos (aguantan perdida de tramas: son mod 16)
        if self._cnt_prev is None:
            self._cnt_prev = (s.cnt_naranja, s.cnt_azul)
            return
        pn, pa = self._cnt_prev
        dn = (s.cnt_naranja - pn) & 0x0F
        da = (s.cnt_azul - pa) & 0x0F
        ahora = time.time()
        for _ in range(dn):
            self._eventos_linea.append(("naranja", ahora))
        for _ in range(da):
            self._eventos_linea.append(("azul", ahora))
        self._cnt_prev = (s.cnt_naranja, s.cnt_azul)

    # -- lecturas ----------------------------------------------------------
    def yaw(self) -> Optional[float]:
        """Yaw utilizable, o None si no hay MPU o esta calibrando o los datos
        son viejos (mas de 0.4 s)."""
        s = self.sensores
        if self.simulado or not s.mpu_ok or s.calibrando:
            return None
        if time.time() - self.ultimos_sensores > 0.4:
            return None
        return s.yaw

    def estado(self) -> Dict[str, Any]:
        t = self.telemetria
        s = self.sensores
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
                "ms_desde_mando": t.ms_desde_mando,
                "tramas_malas": t.tramas_malas, "version": t.version,
            },
            "sensores": {
                "mpu_ok": s.mpu_ok, "tcs_ok": s.tcs_ok,
                "calibrando": s.calibrando,
                "yaw": round(s.yaw, 1),
                "gz": round(s.gz_deci / 10.0, 1),
                "c": s.c, "r": s.r, "g": s.g, "b": s.b,
                "ratio_r": round(s.r * 255.0 / s.c, 0) if s.c else 0,
                "ratio_b": round(s.b * 255.0 / s.c, 0) if s.c else 0,
                "clase": {0: "-", 1: "naranja", 2: "azul"}.get(s.clase_linea, "-"),
                "cnt_naranja": s.cnt_naranja, "cnt_azul": s.cnt_azul,
                "frescos": time.time() - self.ultimos_sensores < 0.4,
            },
        }
