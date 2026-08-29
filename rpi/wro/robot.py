# -*- coding: utf-8 -*-
"""
Orquestador: une camara, percepcion, control, enlace serie y depuracion.

Un unico hilo ejecuta el lazo de control a `control_hz`. Las vistas de
depuracion solo se calculan si hay alguien mirandolas (el servidor web marca
que vistas estan activas), de modo que con el navegador cerrado el robot no
gasta nada en dibujar.
"""

from __future__ import annotations

import csv
import math
import os
import threading
import time
from typing import Dict, Optional

import cv2
import numpy as np

from . import obstacles as obs
from . import overlay
from . import perception as perc
from .camera import Camera
from .controller import Controller, ST_DONE, ST_IDLE
from .geometry import Ground, solve_pitch_from_distance
from .link import EspLink
from .params import Config, ESP_KEYS

MODE_OPEN = "open"        # Reto Abierto
MODE_OBSTACLE = "obstacle"  # Reto con Obstaculos (deteccion basica)
MODE_MANUAL = "manual"      # mando manual

VIEWS = ("overlay", "mask", "bev", "raw")

# Ajustes de camara que obligan a reabrir el dispositivo
_CAM_HARD = {"cam_index", "cam_width", "cam_height", "cam_fps", "cam_mjpg"}
_CAM_SOFT = {"cam_auto_exposure", "cam_exposure", "cam_gain"}
_GEO_KEYS = {"cam_height_mm", "cam_pitch_deg", "cam_roll_deg", "cam_hfov_deg",
             "cam_cx_off", "cam_cy_off", "lens_k1", "lens_k2",
             "cam_offset_x_mm", "cam_offset_y_mm"}


class Robot:
    def __init__(self, cfg: Config, root: str):
        self.cfg = cfg
        self.root = root
        self.cam = Camera(cfg)
        self.esp = EspLink(cfg)
        self.ctrl = Controller(cfg)
        self.ground: Optional[Ground] = None

        self.mode = MODE_OPEN
        self.armed = False
        self.scene = perc.Scene()
        self.pillars = []
        self.loop_hz = 0.0
        self.msg = "listo"
        self.last_error = ""

        self._manual = (0.0, 0.0)
        self._manual_t = 0.0
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._jpeg: Dict[str, bytes] = {}
        self._view_req: Dict[str, float] = {}
        self._btn_prev = 1
        self._gpio_btn = None
        self._log = None
        self._log_w = None
        self._pending_geo = True
        self._cal_request = None
        self.cal_result = ""

    # ================================================================= ciclo
    def start(self):
        self.cam.start()
        self.esp.start()
        self.push_esp_params()
        self._setup_button()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="ctrl")
        self._thread.start()

    def stop(self):
        self.disarm()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close_log()
        self.esp.stop()
        self.cam.stop()

    # =============================================================== comandos
    def push_esp_params(self):
        snap = self.cfg.snapshot()
        vals = {}
        for k in ESP_KEYS:
            v = snap.get(k)
            vals[k] = float(1 if v is True else (0 if v is False else v))
        self.esp.push_params(vals)

    def on_config_changed(self, changed: Dict):
        keys = set(changed)
        if keys & _CAM_HARD:
            self.cam.reconfigure()
        elif keys & _CAM_SOFT:
            self.cam.apply_exposure()
        if keys & _GEO_KEYS:
            self._pending_geo = True
        esp = {k: float(1 if changed[k] is True else
                        (0 if changed[k] is False else changed[k]))
               for k in keys if k in ESP_KEYS}
        for k, v in esp.items():
            self.esp.set_param(k, v)

    def set_mode(self, mode: str):
        if mode in (MODE_OPEN, MODE_OBSTACLE, MODE_MANUAL):
            was = self.mode
            self.mode = mode
            if was != mode:
                self.disarm()
            self.msg = "modo: %s" % mode

    def arm(self):
        if self.mode == MODE_MANUAL:
            self.armed = True
            self.esp.set_power(True)
            self.msg = "manual armado"
            return
        self.esp.zero_yaw()
        self.esp.reset_lines()
        time.sleep(0.05)
        self.ctrl.start(self.esp.tel.yaw, self.esp.tel.seq)
        self.armed = True
        self.esp.set_power(True)
        self._open_log()
        self.msg = "en marcha"

    def disarm(self):
        self.armed = False
        self.esp.drive(0, 0)
        self.esp.set_power(False)
        self.ctrl.stop()
        self._close_log()
        self.msg = "detenido"

    def set_manual(self, steer: float, speed: float):
        self._manual = (float(steer), float(speed))
        self._manual_t = time.time()

    def request_view(self, name: str):
        self._view_req[name] = time.time()

    def get_jpeg(self, name: str) -> Optional[bytes]:
        with self._lock:
            return self._jpeg.get(name)

    def request_pitch_calibration(self, true_x_mm: float):
        self._cal_request = float(true_x_mm)

    # =================================================================== boton
    def _setup_button(self):
        pin = int(self.cfg.start_button_pin)
        if pin < 0:
            return
        try:
            from gpiozero import Button
            self._gpio_btn = Button(pin, pull_up=True, bounce_time=0.05)
        except Exception as exc:
            self.last_error = "GPIO boton: %s" % exc
            self._gpio_btn = None

    def _poll_button(self):
        pressed = False
        if self._gpio_btn is not None:
            try:
                pressed = bool(self._gpio_btn.is_pressed)
            except Exception:
                pressed = False
        b = 1 if pressed else 0
        b |= 1 if self.esp.tel.button else 0
        if b and not self._btn_prev:
            if self.armed:
                self.disarm()
            else:
                self.arm()
        self._btn_prev = b

    # =================================================================== lazo
    def _run(self):
        period = 1.0 / max(5.0, float(self.cfg.control_hz))
        t_prev = time.time()
        n, t_fps = 0, t_prev
        last_seq = -1

        while not self._stop.is_set():
            t0 = time.time()
            dt = max(1e-3, min(0.25, t0 - t_prev))
            t_prev = t0
            period = 1.0 / max(5.0, float(self.cfg.control_hz))

            try:
                self._step(dt, last_seq)
            except Exception as exc:                      # nunca tumbar el lazo
                self.last_error = "lazo: %r" % exc

            n += 1
            if t0 - t_fps >= 0.5:
                self.loop_hz = n / (t0 - t_fps)
                n, t_fps = 0, t0

            rest = period - (time.time() - t0)
            if rest > 0:
                time.sleep(rest)

    def _step(self, dt: float, last_seq: int):
        cfg = self.cfg
        frame, stamp, seq = self.cam.read()

        if frame is not None:
            h, w = frame.shape[:2]
            if (self.ground is None or self._pending_geo
                    or self.ground.w != w or self.ground.h != h):
                self.ground = Ground(cfg, w, h)
                self._pending_geo = False

        self._poll_button()

        # ---------------- percepcion ----------------
        want_mask = self._view_active("mask")
        if frame is not None and self.ground is not None:
            self.scene = perc.analyze(frame, self.ground, cfg, want_mask=want_mask)
            if bool(cfg.obstacles_enabled) or self.mode == MODE_OBSTACLE:
                self.pillars = obs.detect(frame, self.ground, cfg,
                                          self.scene.roi_top, self.scene.roi_bottom)
            else:
                self.pillars = []
        else:
            self.scene = perc.Scene()
            self.pillars = []

        # ---------------- calibracion de inclinacion ----------------
        if self._cal_request is not None:
            self._do_pitch_cal(self._cal_request, frame)
            self._cal_request = None

        # ---------------- control ----------------
        tel = self.esp.tel
        if self.mode == MODE_MANUAL:
            steer, speed = self._manual
            if time.time() - self._manual_t > 0.6:        # dead-man del mando
                steer, speed = 0.0, 0.0
                self._manual = (0.0, 0.0)
            if not self.armed:
                steer, speed = 0.0, 0.0
            self.esp.drive(steer, speed)
        elif self.armed:
            pill = None
            if self.mode == MODE_OBSTACLE and self.pillars:
                half = 700.0
                if self.scene.corridor_mm:
                    half = self.scene.corridor_mm * 0.75
                pill = obs.choose_target(self.pillars, cfg, half)
            steer, speed = self.ctrl.update(self.scene, tel, dt, pill)
            if self.ctrl.state == ST_DONE:
                self.esp.drive(0, 0)
                self.esp.set_power(False)
                self.armed = False
                self.msg = "ronda terminada: %s" % self.ctrl.note
                self._close_log()
            else:
                self.esp.drive(steer, speed)
                self._write_log(tel)
        else:
            self.esp.drive(0, 0)

        # ---------------- vistas ----------------
        self._render(frame)

    # ============================================================== auxiliares
    def _view_active(self, name: str) -> bool:
        t = self._view_req.get(name, 0.0)
        return (time.time() - t) < 3.0

    def _render(self, frame):
        if frame is None:
            frame = Camera.placeholder(int(self.cfg.cam_width),
                                       int(self.cfg.cam_height),
                                       self.cam.error or "SIN CAMARA")
        cs = self.ctrl.snapshot()
        q = [int(cv2.IMWRITE_JPEG_QUALITY), 72]
        out = {}
        try:
            if self._view_active("overlay") and self.ground is not None:
                extra = "cam %.0f fps | lazo %.0f Hz | esp %s" % (
                    self.cam.fps, self.loop_hz,
                    "OK" if self.esp.tel.connected else "OFF")
                img = overlay.draw_overlay(frame, self.scene, self.ground,
                                           self.cfg, cs, self.pillars, extra)
                ok, buf = cv2.imencode(".jpg", img, q)
                if ok:
                    out["overlay"] = buf.tobytes()
            if self._view_active("mask"):
                img = overlay.draw_mask(frame, self.scene)
                ok, buf = cv2.imencode(".jpg", img, q)
                if ok:
                    out["mask"] = buf.tobytes()
            if self._view_active("bev"):
                img = overlay.draw_bev(self.scene, self.cfg, cs, self.pillars)
                ok, buf = cv2.imencode(".jpg", img, q)
                if ok:
                    out["bev"] = buf.tobytes()
            if self._view_active("raw"):
                ok, buf = cv2.imencode(".jpg", frame, q)
                if ok:
                    out["raw"] = buf.tobytes()
        except Exception as exc:
            self.last_error = "dibujo: %r" % exc
        if out:
            with self._lock:
                self._jpeg.update(out)

    def _do_pitch_cal(self, true_x_mm: float, frame):
        """Resuelve la inclinacion con una distancia real medida a un muro."""
        sc = self.scene
        if frame is None or self.ground is None or len(sc.boundary_uv) < 5:
            self.cal_result = "no hay contorno visible: apunta a un muro"
            return
        cx = self.ground.cx
        u = sc.boundary_uv[:, 0]
        sel = np.abs(u - cx) < max(12.0, frame.shape[1] * 0.06)
        if sel.sum() < 3:
            self.cal_result = "no veo la base del muro en el centro de la imagen"
            return
        v_row = float(np.median(sc.boundary_uv[sel, 1]))
        try:
            pitch = solve_pitch_from_distance(self.cfg, frame.shape[1],
                                              frame.shape[0], v_row, true_x_mm)
        except Exception as exc:
            self.cal_result = "fallo: %r" % exc
            return
        if not (-10.0 < pitch < 58.0):
            self.cal_result = "resultado fuera de rango (%.1f): revisa la medida" % pitch
            return
        self.cfg.set_many({"cam_pitch_deg": round(pitch, 2)})
        self.cfg.save()
        self._pending_geo = True
        self.cal_result = ("inclinacion = %.2f grados (fila %.0f, %.0f mm)"
                           % (pitch, v_row, true_x_mm))

    # ================================================================== log
    def _open_log(self):
        if not bool(self.cfg.log_enabled) or self._log is not None:
            return
        d = os.path.join(self.root, "logs")
        os.makedirs(d, exist_ok=True)
        p = os.path.join(d, time.strftime("run_%Y%m%d_%H%M%S.csv"))
        try:
            self._log = open(p, "w", newline="", encoding="utf-8")
            self._log_w = csv.writer(self._log)
            self._log_w.writerow(
                ["t", "state", "dir", "corners", "laps", "yaw", "target_yaw",
                 "head_err", "d_inner", "d_outer", "front", "inner_end",
                 "corridor", "steer", "speed", "n_pts"])
        except Exception as exc:
            self.last_error = "log: %s" % exc
            self._log = None

    def _write_log(self, tel):
        if self._log_w is None:
            return
        c = self.ctrl
        try:
            self._log_w.writerow([
                round(time.time() - c.t_start, 3), c.state, c.direction,
                c.corners, c.laps, round(tel.yaw, 2), round(c.target_yaw, 2),
                round(c.head_err, 2),
                round(c.d_inner.value, 1) if c.d_inner.valid else "",
                round(c.d_outer.value, 1) if c.d_outer.valid else "",
                round(c.front.value, 1) if c.front.valid else "",
                round(c.inner_end, 1) if c.inner_end is not None else "",
                round(c.corridor, 1) if c.corridor else "",
                round(c.steer, 1), round(c.speed, 1), self.scene.n_points])
        except Exception:
            pass

    def _close_log(self):
        if self._log is not None:
            try:
                self._log.close()
            except Exception:
                pass
        self._log = None
        self._log_w = None

    # ============================================================== telemetria
    def status(self) -> dict:
        tel = self.esp.tel
        sc = self.scene
        st = {
            "mode": self.mode,
            "armed": self.armed,
            "msg": self.msg,
            "error": self.last_error,
            "loop_hz": round(self.loop_hz, 1),
            "cam_fps": round(self.cam.fps, 1),
            "cam_ok": self.cam.opened,
            "cam_error": self.cam.error,
            "esp": {
                "connected": tel.connected,
                "port": self.esp.port_name,
                "error": self.esp.error,
                "yaw": round(tel.yaw, 2),
                "gz": round(tel.gz, 2),
                "accel": round(tel.accel_mag, 3),
                "line": tel.line,
                "n_orange": tel.n_orange,
                "n_blue": tel.n_blue,
                "last_event": tel.last_event,
                "seq": tel.seq,
                "rgb": [round(tel.r, 3), round(tel.g, 3), round(tel.b, 3)],
                "c": tel.c,
                "button": tel.button,
                "armed": tel.armed,
                "watchdog": tel.watchdog,
                "age": round(tel.age, 2),
            },
            "vision": {
                "ok": sc.ok,
                "n_points": sc.n_points,
                "roi_top": sc.roi_top,
                "thresh": sc.thresh_used,
                "front": round(sc.front_mm, 1),
                "front_min": round(sc.front_min_mm, 1),
                "left": _wall(sc.left),
                "right": _wall(sc.right),
                "corridor": round(sc.corridor_mm, 1) if sc.corridor_mm else None,
                "corridor_check": _corridor_check(sc.corridor_mm),
                "segments": len(sc.segments),
            },
            "ctrl": self.ctrl.snapshot(),
            "pillars": [{"color": p.color, "x": round(p.x_mm),
                         "y": round(p.y_mm), "conf": round(p.conf, 2)}
                        for p in self.pillars[:8]],
            "cal": self.cal_result,
        }
        if self.ground is not None:
            st["geom"] = self.ground.describe()
        return st


def _corridor_check(w):
    """
    Comprobacion de calibracion sin cinta metrica.

    El reglamento solo permite pasillos de 600 o 1000 mm (+-100). Si con los
    dos muros a la vista el ancho medido no cae cerca de uno de esos dos
    valores, la geometria esta mal calibrada; el error porcentual apunta
    directamente a la inclinacion (lo mas sensible), a la altura o al campo de
    vision. Es la forma mas rapida de detectar en vivo una mala calibracion.
    """
    if not w:
        return None
    ref = 600.0 if abs(w - 600.0) < abs(w - 1000.0) else 1000.0
    err = (w - ref) / ref * 100.0
    if abs(err) <= 8.0:
        level = "ok"
    elif abs(err) <= 18.0:
        level = "warn"
    else:
        level = "bad"
    return {"ref": int(ref), "err_pct": round(err, 1), "level": level}


def _wall(w):
    if w is None:
        return None
    return {"dist": round(w.dist_mm, 1), "angle": round(w.angle_deg, 1),
            "end": round(w.end_mm, 1) if w.end_mm is not None else None,
            "q": round(w.quality, 2), "n": w.n}
