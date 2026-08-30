# -*- coding: utf-8 -*-
"""
Orquestador: une camara, percepcion, control, enlace serie y depuracion.

Dos hilos, y la separacion es importante:

  * `_run`          lazo de control, a `control_hz` (40 Hz). Nunca dibuja.
  * `_render_loop`  vistas de depuracion, a 12 Hz y de UNA sola vista.

Dibujar dentro del lazo de control era un error: las cuatro vistas a 40 Hz
cuestan mas de un nucleo entero de la Pi 5, con lo que el lazo dejaba de dormir,
se quedaba con el GIL y el servidor web no llegaba a atender los comandos del
mando manual (avance a tirones). Con el navegador cerrado no se dibuja nada.
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
class _Tmp:
    """Contenedor suelto para probar geometrias sin tocar la config viva."""
    pass


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
        self._last_seq = -1
        self.vision_hz = 0.0
        self._vis_n = 0
        self._vis_t = time.time()
        self._stop = threading.Event()
        self._thread = None
        self._lock = threading.Lock()
        self._jpeg: Dict[str, bytes] = {}
        # Solo se dibuja UNA vista a la vez, la del cliente que la tiene
        # tomada. Ver request_view(): dibujar las cuatro a la vez costaba mas
        # de un nucleo entero de la Pi 5 y ahogaba al hilo de control.
        self._active_view = ""
        self._active_t = 0.0
        self.render_hz = 0.0
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
        self._render_thread = threading.Thread(target=self._render_loop,
                                               daemon=True, name="render")
        self._render_thread.start()

    def stop(self):
        self.disarm()
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if getattr(self, "_render_thread", None):
            self._render_thread.join(timeout=2.0)
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

    def request_view(self, name: str) -> bool:
        """
        Pide dibujar una vista. Devuelve False si otra la tiene tomada.

        Un stream MJPEG abandonado (el navegador cambia de pestana y deja la
        conexion colgando) seguia pidiendo su vista para siempre. Con las
        cuatro pedidas a la vez, el dibujado se comia mas de un nucleo de la
        Pi 5, el lazo de control dejaba de dormir y los comandos del mando
        llegaban a rachas. Ahora manda un solo cliente y el resto se retira.
        """
        now = time.time()
        if self._active_view == name:
            self._active_t = now
            return True
        if now - self._active_t > 1.0:          # el dueno anterior se fue
            self._active_view = name
            self._active_t = now
            return True
        return False

    def release_view(self, name: str):
        if self._active_view == name:
            self._active_t = 0.0

    def get_jpeg(self, name: str) -> Optional[bytes]:
        with self._lock:
            return self._jpeg.get(name)

    def request_pitch_calibration(self, true_x_mm: float):
        self._cal_request = ("pitch", float(true_x_mm))

    def request_hfov_calibration(self, true_corridor_mm: float):
        self._cal_request = ("hfov", float(true_corridor_mm))

    def request_auto_threshold(self):
        self._cal_request = ("umbral", 0.0)

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

        while not self._stop.is_set():
            t0 = time.time()
            dt = max(1e-3, min(0.25, t0 - t_prev))
            t_prev = t0
            period = 1.0 / max(5.0, float(self.cfg.control_hz))

            try:
                self._step(dt)
            except Exception as exc:                      # nunca tumbar el lazo
                self.last_error = "lazo: %r" % exc

            n += 1
            if t0 - t_fps >= 0.5:
                self.loop_hz = n / (t0 - t_fps)
                n, t_fps = 0, t0

            # El sleep minimo NO es opcional: sin el, si un ciclo se pasa de
            # tiempo el hilo se queda con el GIL y Flask no llega a responder
            # los comandos del mando.
            rest = period - (time.time() - t0)
            time.sleep(rest if rest > 0 else 0.002)

    def _step(self, dt: float):
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
        # Solo se analiza cuando llega un fotograma NUEVO. El lazo de control
        # va a control_hz (40 Hz) y la camara suele dar 30: sin esta guarda se
        # analizaria el mismo fotograma varias veces y se gastaria CPU (y por
        # tanto FPS de camara) para nada.
        want_mask = self._view_active("mask")
        if frame is not None and self.ground is not None:
            if seq != self._last_seq:
                self._last_seq = seq
                self.scene = perc.analyze(frame, self.ground, cfg,
                                          want_mask=want_mask)
                if bool(cfg.obstacles_enabled) or self.mode == MODE_OBSTACLE:
                    self.pillars = obs.detect(frame, self.ground, cfg,
                                              self.scene.roi_top,
                                              self.scene.roi_bottom)
                else:
                    self.pillars = []
                self._vis_n += 1
                if time.time() - self._vis_t >= 1.0:
                    self.vision_hz = self._vis_n / (time.time() - self._vis_t)
                    self._vis_n, self._vis_t = 0, time.time()
        else:
            self.scene = perc.Scene()
            self.pillars = []

        # ---------------- calibracion de inclinacion ----------------
        if self._cal_request is not None:
            kind, value = self._cal_request
            self._cal_request = None
            try:
                if kind == "pitch":
                    self._do_pitch_cal(value, frame)
                elif kind == "hfov":
                    self._do_hfov_cal(value, frame)
                elif kind == "umbral":
                    self._do_threshold_cal(frame)
            except Exception as exc:
                self.cal_result = "fallo en la calibracion: %r" % exc

        # ---------------- control ----------------
        tel = self.esp.tel
        if self.mode == MODE_MANUAL:
            steer, speed = self._manual
            # Dead-man del mando: si el navegador deja de enviar durante este
            # tiempo, se frena. 1 s da margen para un tiron de wifi sin que el
            # coche se quede en marcha si se pierde la conexion de verdad.
            if time.time() - self._manual_t > 1.0:
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


    # ============================================================== auxiliares
    def _view_active(self, name: str) -> bool:
        return (self._active_view == name
                and (time.time() - self._active_t) < 1.5)

    def _render_loop(self):
        """
        Dibuja las vistas de depuracion en su propio hilo y a ritmo bajo.

        El dibujado NO puede ir al ritmo del lazo de control: a 40 Hz cuesta
        mas de lo que el navegador puede mostrar y le quita tiempo al control.
        12 Hz se ve fluido y deja la CPU libre.
        """
        period = 1.0 / 12.0
        n, t_ref = 0, time.time()
        while not self._stop.is_set():
            t0 = time.time()
            name = self._active_view
            if name and (t0 - self._active_t) < 1.5:
                frame, _, _ = self.cam.read()
                try:
                    self._render_one(name, frame)
                except Exception as exc:
                    self.last_error = "dibujo: %r" % exc
                n += 1
                if t0 - t_ref >= 1.0:
                    self.render_hz = n / (t0 - t_ref)
                    n, t_ref = 0, t0
            else:
                self.render_hz = 0.0
                n, t_ref = 0, t0
            rest = period - (time.time() - t0)
            time.sleep(rest if rest > 0 else 0.005)

    def _render_one(self, name: str, frame):
        if frame is None:
            frame = Camera.placeholder(int(self.cfg.cam_width),
                                       int(self.cfg.cam_height),
                                       self.cam.error or "SIN CAMARA")
        cs = self.ctrl.snapshot()
        q = [int(cv2.IMWRITE_JPEG_QUALITY), 72]

        if name == "overlay":
            if self.ground is None:
                return
            extra = "cam %.0f fps | vision %.0f Hz | lazo %.0f Hz | esp %s" % (
                self.cam.fps, self.vision_hz, self.loop_hz,
                "OK" if self.esp.tel.connected else "OFF")
            img = overlay.draw_overlay(frame, self.scene, self.ground,
                                       self.cfg, cs, self.pillars, extra)
        elif name == "mask":
            img = overlay.draw_mask(frame, self.scene)
        elif name == "bev":
            img = overlay.draw_bev(self.scene, self.cfg, cs, self.pillars)
        else:
            img = frame

        ok, buf = cv2.imencode(".jpg", img, q)
        if ok:
            with self._lock:
                self._jpeg[name] = buf.tobytes()

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

    def _ground_for(self, overrides, w, h):
        """Construye una geometria alternativa sin tocar la configuracion viva."""
        tmp = _Tmp()
        for k in _GEO_KEYS:
            setattr(tmp, k, self.cfg.get(k))
        for k, v in overrides.items():
            setattr(tmp, k, v)
        return Ground(tmp, w, h)

    def _do_hfov_cal(self, true_corridor_mm: float, frame):
        """
        Resuelve el campo de vision midiendo un pasillo de ancho conocido.

        El ancho medido crece con el campo de vision, asi que basta barrer y
        quedarse con el valor que reproduce la cinta metrica. Se hace por
        barrido y no con una formula porque cambiar fx tambien cambia la
        componente vertical del rayo: no es un simple factor de escala.
        """
        if frame is None:
            self.cal_result = "sin imagen"
            return
        h, w = frame.shape[:2]

        def corridor_for(hfov):
            g = self._ground_for({"cam_hfov_deg": hfov}, w, h)
            sc = perc.analyze(frame, g, self.cfg)
            return sc.corridor_mm

        base = corridor_for(float(self.cfg.cam_hfov_deg))
        if base is None:
            self.cal_result = ("no veo los DOS muros laterales: coloca el robot "
                               "dentro del pasillo y mirando a lo largo de el")
            return

        best, best_err = None, 1e9
        for i in range(0, 111):                      # 40..150 grados, paso 1
            f = 40.0 + i
            c = corridor_for(f)
            if c is None:
                continue
            e = abs(c - true_corridor_mm)
            if e < best_err:
                best, best_err = f, e
        if best is None:
            self.cal_result = "no se pudo medir el pasillo"
            return
        for i in range(-10, 11):                     # refinado de 0.1 en 0.1
            f = best + i * 0.1
            if not (40.0 <= f <= 150.0):
                continue
            c = corridor_for(f)
            if c is not None and abs(c - true_corridor_mm) < best_err:
                best, best_err = f, abs(c - true_corridor_mm)

        if best_err > true_corridor_mm * 0.06:
            self.cal_result = ("no converge (mejor %.1f grados, error %.0f mm). "
                               "Revisa la medida y que se vean los dos muros."
                               % (best, best_err))
            return
        self.cfg.set_many({"cam_hfov_deg": round(best, 1)})
        self.cfg.save()
        self._pending_geo = True
        self.cal_result = ("campo de vision = %.1f grados (medido %.0f mm, "
                           "real %.0f mm). Repite ahora la inclinacion: los dos "
                           "parametros estan acoplados."
                           % (best, base, true_corridor_mm))

    def _do_threshold_cal(self, frame):
        """Fija el umbral de oscuridad con Otsu sobre la region de interes."""
        if frame is None or self.ground is None:
            self.cal_result = "sin imagen"
            return
        top = self.ground.roi_top_row(float(self.cfg.roi_x_max_mm))
        roi = frame[top:frame.shape[0] - int(self.cfg.roi_bottom_crop_px)]
        if roi.size == 0:
            self.cal_result = "region de interes vacia"
            return
        v = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[:, :, 2]
        t, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        below = v < t
        frac = float(below.mean())
        dark = float(v[below].mean()) if below.any() else 0.0
        light = float(v[~below].mean()) if (~below).any() else 255.0
        sep = light - dark

        # Otsu SIEMPRE parte la imagen en dos, tenga sentido o no. Si en la
        # region de interes solo hay tapete, parte el blanco en "blanco algo
        # mas oscuro" y "blanco algo mas claro" y devuelve un umbral altisimo
        # que luego clasifica medio tapete como muro. Hay que comprobar que lo
        # que se ha encontrado parece de verdad un muro negro antes de guardar.
        problemas = []
        if dark > 120:
            problemas.append("la parte oscura sale a %.0f y un muro negro deberia "
                             "estar por debajo de 120: o no hay muro a la vista o "
                             "la imagen esta sobreexpuesta" % dark)
        if sep < 60:
            problemas.append("muro y tapete se parecen demasiado "
                             "(separacion %.0f, hacen falta 60)" % sep)
        if frac < 0.04:
            problemas.append("casi no se ve muro en la zona analizada (%.1f%%)"
                             % (frac * 100))
        if frac > 0.75:
            problemas.append("el muro ocupa casi todo (%.0f%%): alejate o baja "
                             "el rango maximo" % (frac * 100))

        if problemas:
            self.cal_result = ("NO se ha cambiado el umbral. " + "; ".join(problemas)
                               + ". Corrige la exposicion (vista Mascara: tapete "
                                 "negro, muros blancos) y vuelve a intentarlo.")
            return

        # Nos quedamos algo por debajo del corte de Otsu: preferimos perder
        # algun pixel de muro antes que tomar tapete en sombra por muro.
        thr = int(max(20, min(200, dark + sep * 0.45)))
        self.cfg.set_many({"wall_v_max": thr, "wall_auto_thresh": False})
        self.cfg.save()
        self.cal_result = ("umbral = %d  (muro %.0f / tapete %.0f, separacion "
                           "%.0f, muro ocupa %.0f%%)"
                           % (thr, dark, light, sep, frac * 100))

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
            "cam_stall_ms": round(self.cam.stall_ms, 0),
            "cam_ok": self.cam.opened,
            "cam_error": self.cam.error,
            "cam_negotiated": self.cam.negotiated,
            "cam_ctrl": self.cam.ctrl_note,
            "vision_hz": round(self.vision_hz, 1),
            "render_hz": round(self.render_hz, 1),
            "view": self._active_view,
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
