# -*- coding: utf-8 -*-
"""
Enlace serie con el ESP32 (USB).

Hilo dedicado que:
  * abre y reabre el puerto solo si se desconecta,
  * lee telemetria a ~50 Hz y la deja disponible sin bloquear,
  * envia comandos de direccion/velocidad,
  * reenvia los parametros de hardware cada vez que el ESP32 reinicia
    (se detecta porque su reloj millis() vuelve hacia atras).
"""

from __future__ import annotations

import glob
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

try:
    import serial                      # pyserial
except ImportError:                    # pragma: no cover
    serial = None


@dataclass
class Telemetry:
    connected: bool = False
    stamp: float = 0.0
    t_ms: int = 0
    yaw: float = 0.0
    gz: float = 0.0
    accel_mag: float = 1.0
    line: int = 0                     # color visto ahora (0 nada, 1 naranja, 2 azul)
    n_orange: int = 0
    n_blue: int = 0
    last_event: int = 0               # ultimo color confirmado
    seq: int = 0                      # contador de eventos de linea
    r: float = 0.0
    g: float = 0.0
    b: float = 0.0
    c: int = 0
    button: int = 0
    armed: int = 0
    watchdog: int = 0
    age: float = 99.0


class EspLink:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tel = Telemetry()
        self.port_name = ""
        self.error = ""
        self.rx_lines = deque(maxlen=120)     # consola de depuracion
        self.tx_count = 0
        self.rx_count = 0

        self._ser = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._pending: List[str] = []
        self._cmd = (0, 0)
        self._power = False
        self._last_t_ms = 0
        self._need_params = True
        self._param_cache: Dict[str, float] = {}

    # ------------------------------------------------------------------ ciclo
    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="esp")
        self._thread.start()

    def stop(self):
        try:
            self.set_power(False)
            self.drive(0, 0)
            time.sleep(0.08)
        except Exception:
            pass
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close()

    # ------------------------------------------------------------- comandos
    def drive(self, steer_pct: float, speed_pct: float):
        """steer/speed en porcentaje -100..100 (steer positivo = izquierda)."""
        s = int(max(-1000, min(1000, round(steer_pct * 10))))
        v = int(max(-1000, min(1000, round(speed_pct * 10))))
        with self._lock:
            self._cmd = (s, v)

    def set_power(self, on: bool):
        with self._lock:
            self._power = bool(on)
            self._pending.append("S %d" % (1 if on else 0))

    def zero_yaw(self):
        with self._lock:
            self._pending.append("Z")

    def reset_lines(self):
        with self._lock:
            self._pending.append("L")

    def set_param(self, name: str, value):
        v = float(value)
        with self._lock:
            self._param_cache[name] = v
            self._pending.append("P %s %.4f" % (name, v))

    def push_params(self, values: Dict[str, float]):
        with self._lock:
            for k, v in values.items():
                self._param_cache[k] = float(v)
            self._need_params = True

    def raw(self, line: str):
        with self._lock:
            self._pending.append(line.strip())

    # ------------------------------------------------------------------ hilo
    def _ports(self) -> List[str]:
        want = str(self.cfg.serial_port).strip()
        if want and want.lower() != "auto":
            return [want]
        found = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
        # En Windows, para pruebas de escritorio
        found += ["COM%d" % i for i in range(3, 12)] if not found else []
        return found

    def _open(self) -> bool:
        if serial is None:
            self.error = "pyserial no instalado (pip install pyserial)"
            return False
        for p in self._ports():
            try:
                s = serial.Serial(p, int(self.cfg.serial_baud), timeout=0.05,
                                  write_timeout=0.2)
                time.sleep(0.05)
                s.reset_input_buffer()
                self._ser = s
                self.port_name = p
                self.error = ""
                self._need_params = True
                self._last_t_ms = 0
                return True
            except Exception as exc:
                self.error = "%s: %s" % (p, exc)
        return False

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None
        self.tel.connected = False

    def _run(self):
        buf = b""
        last_cmd = 0.0
        while not self._stop.is_set():
            if self._ser is None:
                if not self._open():
                    time.sleep(1.0)
                    continue

            # ---------------- envio ----------------
            now = time.time()
            try:
                with self._lock:
                    pend = self._pending
                    self._pending = []
                    cmd = self._cmd
                    need = self._need_params
                    params = dict(self._param_cache) if need else {}
                    if need:
                        self._need_params = False

                for k, v in params.items():
                    self._ser.write(("P %s %.4f\n" % (k, v)).encode())
                    time.sleep(0.002)
                for line in pend:
                    self._ser.write((line + "\n").encode())
                if now - last_cmd >= 0.02:          # 50 Hz
                    last_cmd = now
                    self._ser.write(("C %d %d\n" % cmd).encode())
                    self.tx_count += 1
            except Exception as exc:
                self.error = "escritura: %s" % exc
                self._close()
                time.sleep(0.4)
                continue

            # ---------------- recepcion ----------------
            try:
                data = self._ser.read(512)
            except Exception as exc:
                self.error = "lectura: %s" % exc
                self._close()
                time.sleep(0.4)
                continue

            if data:
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("ascii", "ignore").strip()
                    if line:
                        self._handle(line)
            if len(buf) > 4096:
                buf = b""

            self.tel.age = time.time() - self.tel.stamp if self.tel.stamp else 99.0
            if self.tel.age > 1.5:
                self.tel.connected = False
            time.sleep(0.004)

    def _handle(self, line: str):
        self.rx_count += 1
        if line.startswith("T "):
            self._parse_tel(line[2:])
        else:
            self.rx_lines.append("%.1f %s" % (time.time() % 1000, line))

    def _parse_tel(self, body: str):
        t = self.tel
        try:
            for tok in body.split():
                k, _, v = tok.partition("=")
                if not v:
                    continue
                if k == "t":
                    n = int(v)
                    if n < self._last_t_ms - 500:      # el ESP32 reinicio
                        with self._lock:
                            self._need_params = True
                            self._pending.append("S %d" % (1 if self._power else 0))
                    self._last_t_ms = n
                    t.t_ms = n
                elif k == "yaw":  t.yaw = float(v)
                elif k == "gz":   t.gz = float(v)
                elif k == "am":   t.accel_mag = float(v)
                elif k == "line": t.line = int(v)
                elif k == "no":   t.n_orange = int(v)
                elif k == "nb":   t.n_blue = int(v)
                elif k == "ls":   t.last_event = int(v)
                elif k == "seq":  t.seq = int(v)
                elif k == "r":    t.r = float(v)
                elif k == "g":    t.g = float(v)
                elif k == "b":    t.b = float(v)
                elif k == "c":    t.c = int(v)
                elif k == "btn":  t.button = int(v)
                elif k == "arm":  t.armed = int(v)
                elif k == "wd":   t.watchdog = int(v)
            t.stamp = time.time()
            t.age = 0.0
            t.connected = True
        except Exception:
            pass
