# -*- coding: utf-8 -*-
"""
Captura de la camara USB (UVC) en un hilo aparte.

Se mantiene siempre el ultimo fotograma disponible: el lazo de control nunca
se queda esperando al driver, y si un fotograma se pierde simplemente se usa
el siguiente.

Sobre los FPS
-------------
La causa numero uno de "la camara no pasa de 15 FPS" NO es la Pi ni el USB: es
la EXPOSICION AUTOMATICA. Con poca luz el driver alarga el tiempo de
integracion (por ejemplo a 1/15 s) y, para poder hacerlo, baja la tasa de
fotogramas. Ademas el brillo cambia solo, con lo que el umbral de segmentacion
de los muros deja de valer. Por eso aqui:

  * se fuerza exposicion MANUAL y se fija el tiempo de exposicion,
  * si OpenCV no consigue aplicarlo (el mapeo de CAP_PROP_AUTO_EXPOSURE varia
    entre drivers), se reintenta por `v4l2-ctl`, que habla directo con el
    driver y es mucho mas fiable,
  * se lee de vuelta el formato realmente negociado y se publica en el panel,
    para no tener que adivinar si el MJPG entro o no.

`stall_ms` mide el hueco mas largo entre fotogramas: sirve para distinguir
"voy a 15 FPS constantes" (tolerable) de "se traba" (no tolerable).
"""

from __future__ import annotations

import subprocess
import threading
import time

import cv2
import numpy as np


def _fourcc_str(v) -> str:
    try:
        v = int(v)
        return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))
    except Exception:
        return "?"


def _v4l2_set(device: str, pairs) -> str:
    """Aplica controles con v4l2-ctl. Devuelve '' si fue bien."""
    args = ["v4l2-ctl", "-d", device]
    for k, v in pairs:
        args += ["--set-ctrl", "%s=%d" % (k, int(v))]
    try:
        r = subprocess.run(args, capture_output=True, timeout=2.0, text=True)
        return "" if r.returncode == 0 else (r.stderr or "").strip()
    except FileNotFoundError:
        return "v4l2-ctl no instalado (sudo apt install v4l-utils)"
    except Exception as exc:
        return str(exc)


class Camera:
    def __init__(self, cfg):
        self.cfg = cfg
        self.cap = None
        self._frame = None
        self._stamp = 0.0
        self._seq = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self.fps = 0.0
        self.stall_ms = 0.0          # hueco maximo entre fotogramas (ventana 2 s)
        self.error = ""
        self.ctrl_note = ""          # como se aplico la exposicion
        self.opened = False
        self.negotiated = {}         # formato realmente concedido por el driver

    # ------------------------------------------------------------------ ciclo
    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="camera")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def read(self):
        """Devuelve (frame, stamp, seq). frame puede ser None."""
        with self._lock:
            return self._frame, self._stamp, self._seq

    @property
    def device(self) -> str:
        return "/dev/video%d" % int(self.cfg.cam_index)

    # ------------------------------------------------------------------ open
    def _open(self) -> bool:
        cfg = self.cfg
        idx = int(cfg.cam_index)
        cap = None
        for be in (cv2.CAP_V4L2, cv2.CAP_ANY):
            try:
                cap = cv2.VideoCapture(idx, be)
                if cap.isOpened():
                    break
                cap.release()
                cap = None
            except Exception:
                cap = None
        if cap is None or not cap.isOpened():
            self.error = "no se pudo abrir la camara %d" % idx
            return False

        # ORDEN IMPORTANTE: primero el formato (MJPG), luego la resolucion y
        # por ultimo los FPS. Al reves el driver puede quedarse en YUYV, que a
        # 640x480 no cabe en USB 2.0 a mas de ~15 FPS.
        try:
            if bool(cfg.cam_mjpg):
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.cam_width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.cam_height))
            cap.set(cv2.CAP_PROP_FPS, int(cfg.cam_fps))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as exc:
            self.error = "ajustes de camara: %s" % exc

        self.cap = cap
        self.apply_exposure()

        # Lo que el driver concedio de verdad (no lo que pedimos).
        try:
            self.negotiated = {
                "fourcc": _fourcc_str(cap.get(cv2.CAP_PROP_FOURCC)),
                "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "fps": round(float(cap.get(cv2.CAP_PROP_FPS)), 1),
            }
        except Exception:
            self.negotiated = {}

        self.opened = True
        self.error = ""
        return True

    def apply_exposure(self):
        """
        Fija exposicion y ganancia. Se puede llamar en caliente.

        Se intenta primero por OpenCV y siempre se refuerza con v4l2-ctl,
        porque el valor de CAP_PROP_AUTO_EXPOSURE que significa "manual" no es
        el mismo en todos los drivers y es habitual que la peticion se ignore
        en silencio (y la camara se quede en automatico a 15 FPS).
        """
        cap, cfg = self.cap, self.cfg
        if cap is None:
            return
        auto = bool(cfg.cam_auto_exposure)
        exp = int(cfg.cam_exposure)
        gain = int(cfg.cam_gain)

        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if auto else 1)
            if not auto:
                cap.set(cv2.CAP_PROP_EXPOSURE, exp)
                cap.set(cv2.CAP_PROP_GAIN, gain)
        except Exception:
            pass

        # Refuerzo por v4l2-ctl. Los nombres de control cambiaron de kernel:
        # nuevo = auto_exposure / exposure_time_absolute
        # viejo = exposure_auto / exposure_absolute
        # En ambos, 1 = manual y 3 = prioridad de apertura (automatico).
        mode = 3 if auto else 1
        pairs_new = [("auto_exposure", mode)]
        pairs_old = [("exposure_auto", mode)]
        if not auto:
            pairs_new += [("exposure_time_absolute", exp), ("gain", gain)]
            pairs_old += [("exposure_absolute", exp), ("gain", gain)]

        err = _v4l2_set(self.device, pairs_new)
        if err:
            err2 = _v4l2_set(self.device, pairs_old)
            self.ctrl_note = ("v4l2-ctl (nombres antiguos)" if not err2
                              else "solo OpenCV: %s" % err[:70])
        else:
            self.ctrl_note = "v4l2-ctl"

    def formats(self) -> str:
        """Formatos y controles que soporta la camara (diagnostico del panel)."""
        out = []
        for args in (["v4l2-ctl", "-d", self.device, "--list-formats-ext"],
                     ["v4l2-ctl", "-d", self.device, "--list-ctrls"]):
            try:
                r = subprocess.run(args, capture_output=True, timeout=4.0,
                                   text=True)
                out.append(r.stdout or r.stderr)
            except FileNotFoundError:
                return ("v4l2-ctl no esta instalado.\n"
                        "Instalalo con:  sudo apt install v4l-utils")
            except Exception as exc:
                out.append(str(exc))
        return "\n".join(out)

    def reconfigure(self):
        """Cierra y reabre con los ajustes actuales (resolucion, fps, formato)."""
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None
        self.opened = False

    # ------------------------------------------------------------------ hilo
    def _run(self):
        last_t = time.time()
        last_frame_t = last_t
        n = 0
        gap_max = 0.0
        while not self._stop.is_set():
            if self.cap is None or not self.opened:
                if not self._open():
                    time.sleep(1.0)
                    continue
                last_frame_t = time.time()
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.error = "lectura fallida"
                self.opened = False
                time.sleep(0.2)
                continue
            if bool(self.cfg.cam_flip):
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            now = time.time()
            gap_max = max(gap_max, now - last_frame_t)
            last_frame_t = now
            with self._lock:
                self._frame = frame
                self._stamp = now
                self._seq += 1
            n += 1
            if now - last_t >= 2.0:
                self.fps = n / (now - last_t)
                self.stall_ms = gap_max * 1000.0
                n, gap_max = 0, 0.0
                last_t = now

    # ------------------------------------------------------------- utilidad
    @staticmethod
    def placeholder(w=640, h=480, text="SIN CAMARA"):
        img = np.zeros((h, w, 3), np.uint8)
        img[:] = (28, 28, 32)
        cv2.putText(img, text, (int(w * 0.12), int(h * 0.5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (60, 90, 220), 2)
        return img
