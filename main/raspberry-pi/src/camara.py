"""
camera.py — Apertura de la camara USB igual en Windows 11 y en Raspbian/Debian.

La WN-L1812.K56R (sensor IMX179) es UVC pura, asi que funciona sin drivers en
ambos sistemas; lo que cambia es el backend de OpenCV:

    Windows : CAP_DSHOW (rapido de abrir)  ->  CAP_MSMF  ->  CAP_ANY
    Linux   : CAP_V4L2                     ->  CAP_ANY

Ademas fuerza MJPG. Por USB 2.0 el modo YUYV limita 640x480 a ~10 FPS
(y 1280x720 a ~5); con MJPG el propio sensor entrega comprimido y se alcanzan
30 FPS. En la Pi 5 el decode de MJPG es barato comparado con perder frames.
"""

from __future__ import annotations

import platform
import sys
import time
from typing import List, Optional, Tuple, Union

import cv2

ES_WINDOWS = platform.system().lower().startswith("win")


def _backends() -> List[int]:
    if ES_WINDOWS:
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_V4L2, cv2.CAP_ANY]


def abrir(indice: Union[int, str] = 0,
          ancho: int = 640,
          alto: int = 480,
          fps: int = 30,
          fourcc: str = "MJPG",
          backend: Optional[int] = None,
          verbose: bool = True) -> Optional[cv2.VideoCapture]:
    """Devuelve un VideoCapture listo, o None si no se pudo abrir."""
    candidatos = [backend] if backend is not None else _backends()
    # Una ruta tipo /dev/video0 o un archivo de video: sin backend especifico.
    if isinstance(indice, str) and not indice.isdigit():
        candidatos = [cv2.CAP_ANY] if not ES_WINDOWS else [cv2.CAP_ANY]
    else:
        indice = int(indice)

    for be in candidatos:
        cap = cv2.VideoCapture(indice, be) if be is not None else cv2.VideoCapture(indice)
        if not cap.isOpened():
            cap.release()
            continue

        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        if ancho:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(ancho))
        if alto:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(alto))
        if fps:
            cap.set(cv2.CAP_PROP_FPS, int(fps))
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # menos latencia; no todos lo aceptan
        except Exception:
            pass

        # Algunas camaras entregan basura en los primeros frames.
        ok = False
        for _ in range(8):
            ok, _f = cap.read()
            if ok:
                break
            time.sleep(0.05)
        if not ok:
            cap.release()
            continue

        if verbose:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            f = cap.get(cv2.CAP_PROP_FPS)
            print(f"[camera] abierta idx={indice} backend={be} -> {w}x{h} @ {f:.0f} fps")
        return cap

    if verbose:
        print(f"[camera] no se pudo abrir la camara {indice!r}", file=sys.stderr)
    return None


def listar(maximo: int = 8) -> List[int]:
    """Indices que responden. En Linux tambien puedes mirar /dev/video*."""
    encontrados = []
    for i in range(maximo):
        cap = abrir(i, ancho=0, alto=0, fps=0, fourcc="", verbose=False)
        if cap is not None:
            encontrados.append(i)
            cap.release()
    return encontrados


def fijar_manual(cap: cv2.VideoCapture,
                 exposicion: Optional[float] = None,
                 balance_blancos: Optional[float] = None,
                 ganancia: Optional[float] = None) -> None:
    """Congela exposicion y balance de blancos.

    ESTO IMPORTA MUCHO: si el auto-exposicion sigue activo, el HSV cambia solo
    cuando el robot gira hacia una pared clara y la calibracion deja de servir.
    Los valores utiles varian por camara; prueba en el calibrador.
    En Windows (DSHOW) AUTO_EXPOSURE usa 0.25=manual / 0.75=auto,
    en V4L2 usa 1=manual / 3=auto. Aqui se intentan ambos.
    """
    if exposicion is not None:
        for valor_manual in (0.25, 1):
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, valor_manual)
            except Exception:
                pass
        cap.set(cv2.CAP_PROP_EXPOSURE, float(exposicion))
    if balance_blancos is not None:
        try:
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(balance_blancos))
        except Exception:
            pass
    if ganancia is not None:
        try:
            cap.set(cv2.CAP_PROP_GAIN, float(ganancia))
        except Exception:
            pass


class Camara:
    """Envoltura con context manager:

        with Camara(0, 640, 480) as cam:
            for frame in cam:
                ...
    """

    def __init__(self, indice=0, ancho=640, alto=480, fps=30,
                 fourcc="MJPG", voltear=False, verbose=True):
        self.cfg = dict(indice=indice, ancho=ancho, alto=alto, fps=fps,
                        fourcc=fourcc, verbose=verbose)
        self.voltear = voltear
        self.cap: Optional[cv2.VideoCapture] = None

    def abrir(self) -> bool:
        self.cap = abrir(**self.cfg)
        return self.cap is not None

    def leer(self) -> Tuple[bool, Optional["cv2.Mat"]]:
        if self.cap is None:
            return False, None
        ok, frame = self.cap.read()
        if ok and self.voltear:
            frame = cv2.flip(frame, -1)  # camara montada al reves
        return ok, frame

    def cerrar(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        if not self.abrir():
            raise RuntimeError(f"No se pudo abrir la camara {self.cfg['indice']}")
        return self

    def __exit__(self, *exc):
        self.cerrar()
        return False

    def __iter__(self):
        while True:
            ok, frame = self.leer()
            if not ok:
                break
            yield frame


if __name__ == "__main__":
    print("Sistema:", platform.system())
    print("Camaras detectadas:", listar())
