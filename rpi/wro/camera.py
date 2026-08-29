# -*- coding: utf-8 -*-
"""
Captura de la camara USB (UVC) en un hilo aparte.

Se mantiene siempre el ultimo fotograma disponible: el lazo de control nunca
se queda esperando al driver, y si un fotograma se pierde simplemente se usa
el siguiente. Tambien se fuerza exposicion manual, porque con exposicion
automatica el umbral de segmentacion de los muros deja de ser valido en cuanto
cambia la luz de la escena.
"""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np


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
        self.error = ""
        self.opened = False

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

    # ------------------------------------------------------------------ open
    def _open(self) -> bool:
        cfg = self.cfg
        idx = int(cfg.cam_index)
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        cap = None
        for be in backends:
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
        self.opened = True
        self.error = ""
        return True

    def apply_exposure(self):
        """Aplica exposicion / ganancia. Se puede llamar en caliente."""
        cap, cfg = self.cap, self.cfg
        if cap is None:
            return
        try:
            if bool(cfg.cam_auto_exposure):
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)     # V4L2: 3 = auto
            else:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)     # V4L2: 1 = manual
                cap.set(cv2.CAP_PROP_EXPOSURE, int(cfg.cam_exposure))
                cap.set(cv2.CAP_PROP_GAIN, int(cfg.cam_gain))
        except Exception:
            pass

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
        n = 0
        while not self._stop.is_set():
            if self.cap is None or not self.opened:
                if not self._open():
                    time.sleep(1.0)
                    continue
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.error = "lectura fallida"
                self.opened = False
                time.sleep(0.2)
                continue
            if bool(self.cfg.cam_flip):
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            now = time.time()
            with self._lock:
                self._frame = frame
                self._stamp = now
                self._seq += 1
            n += 1
            if now - last_t >= 0.5:
                self.fps = n / (now - last_t)
                n = 0
                last_t = now
        # fin

    # ------------------------------------------------------------- utilidad
    @staticmethod
    def placeholder(w=640, h=480, text="SIN CAMARA"):
        img = np.zeros((h, w, 3), np.uint8)
        img[:] = (28, 28, 32)
        cv2.putText(img, text, (int(w * 0.12), int(h * 0.5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (60, 90, 220), 2)
        return img
