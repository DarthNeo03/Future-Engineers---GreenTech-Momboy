"""
camara.py — Captura de la WN-L1812.K56R (sensor IMX179) por USB 2.0.

DOS COSAS QUE NO SON NEGOCIABLES Y POR QUE

1. MJPG, no YUYV. Por USB 2.0 el modo sin comprimir limita 640x480 a ~10 fps
   (y 1280x720 a ~5) porque el bus se satura. Con MJPG el sensor entrega ya
   comprimido y se llega a los 30 fps de la hoja de datos. En la Pi 5 el
   decode de MJPG cuesta mucho menos que perder dos de cada tres frames.

2. Hilo propio con buffer de UNO. cv2.VideoCapture acumula frames en una cola
   interna: si el lazo de vision tarda 60 ms en un frame, el siguiente read()
   devuelve el frame viejo que estaba en cola, no lo que la camara ve AHORA.
   A 0.8 m/s, 60 ms de retraso son 5 cm de carro; dos frames de retraso y el
   esquive se decide sobre un pilar que ya se paso. Aqui un hilo vacia la cola
   sin parar y el lazo siempre recoge el ULTIMO frame, aunque eso signifique
   descartar alguno. Para conducir, un frame viejo es peor que ningun frame.
"""

from __future__ import annotations

import glob
import os
import platform
import threading
import time
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np

ES_WINDOWS = platform.system().lower().startswith("win")


def _backends() -> List[int]:
    # En Windows DSHOW abre mucho mas rapido que MSMF y respeta el FOURCC.
    if ES_WINDOWS:
        return [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    return [cv2.CAP_V4L2, cv2.CAP_ANY]


def dispositivos() -> List[int]:
    """Indices /dev/videoN presentes, en orden. Vacio en Windows.

    No todos son camaras: una UVC como la IMX179 suele exponer DOS nodos, uno
    de captura y otro solo de metadatos, y cual de los dos cae en video0
    depende del orden de enumeracion del arranque. Por eso el indice fijo del
    JSON no es de fiar como unica opcion.
    """
    if ES_WINDOWS:
        return []
    idx = []
    for ruta in glob.glob("/dev/video*"):
        cola = ruta[len("/dev/video"):]
        if cola.isdigit():
            idx.append(int(cola))
    return sorted(idx)


def quien_la_usa(indice: int) -> str:
    """Pista sobre quien tiene ocupado el dispositivo, leida de /proc.

    No usa fuser ni lsof (pueden no estar instalados en Raspbian Lite): mira
    los enlaces de /proc/*/fd a pelo. Devuelve texto para el mensaje de error,
    nunca lanza: si no se puede averiguar, se dice que no se pudo.
    """
    if ES_WINDOWS:
        return ""
    destino = f"/dev/video{indice}"
    culpables = []
    try:
        for pid in os.listdir("/proc"):
            if not pid.isdigit() or int(pid) == os.getpid():
                continue
            try:
                for fd in os.listdir(f"/proc/{pid}/fd"):
                    if os.readlink(f"/proc/{pid}/fd/{fd}") == destino:
                        with open(f"/proc/{pid}/comm") as f:
                            culpables.append(f"{f.read().strip()}(pid {pid})")
                        break
            except (OSError, PermissionError):
                continue
    except OSError:
        return ""
    return ", ".join(sorted(set(culpables)))


def _probar(indice: Union[int, str], ancho: int, alto: int, fps: int,
            fourcc: str, verbose: bool) -> Optional[cv2.VideoCapture]:
    """Intenta UN indice con todos los backends. None si ninguno sirve."""
    if isinstance(indice, str) and not indice.isdigit():
        candidatos = [cv2.CAP_ANY]          # ruta /dev/videoN o archivo
    else:
        indice = int(indice)
        candidatos = _backends()

    for be in candidatos:
        cap = cv2.VideoCapture(indice, be)
        if not cap.isOpened():
            cap.release()
            continue
        # El orden importa: primero el FOURCC, luego el tamaño. Al reves,
        # algunos drivers reajustan el tamaño al cambiar de formato.
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(ancho))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(alto))
        cap.set(cv2.CAP_PROP_FPS, int(fps))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # no todos los drivers lo honran
        # Exigir un frame de verdad, no solo que abra: un nodo de METADATOS de
        # una UVC abre sin quejarse y no entrega imagen nunca. Sin esta lectura
        # el programa arrancaria "bien" y se quedaria ciego.
        ok, _ = cap.read()
        if not ok:
            cap.release()
            continue
        if verbose:
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            f = cap.get(cv2.CAP_PROP_FPS)
            print(f"[camara] indice {indice}: {w}x{h} @ {f:.0f} fps  backend={be}")
        return cap
    return None


def abrir(indice: Union[int, str] = 0, ancho: int = 640, alto: int = 480,
          fps: int = 30, fourcc: str = "MJPG", verbose: bool = True,
          autobuscar: bool = True) -> Optional[cv2.VideoCapture]:
    """Devuelve un VideoCapture ya configurado, o None.

    Si el indice configurado falla y autobuscar esta activo, prueba los demas
    /dev/videoN que existan. El dia de la competencia el orden de enumeracion
    puede cambiar por un simple reinicio, y quedarse sin correr por eso seria
    absurdo teniendo el resto del sistema listo.
    """
    cap = _probar(indice, ancho, alto, fps, fourcc, verbose)
    if cap is not None:
        return cap

    if not autobuscar or (isinstance(indice, str) and not indice.isdigit()):
        return None

    otros = [i for i in dispositivos() if i != int(indice)]
    for i in otros:
        cap = _probar(i, ancho, alto, fps, fourcc, verbose)
        if cap is not None:
            if verbose:
                print(f"[camara] AVISO: el indice {indice} de pista.json no "
                      f"sirve; se esta usando el {i}. Cambialo en el JSON "
                      f'("camara": {{"indice": {i}}}) para no depender del '
                      f"orden de arranque.")
            return cap
    return None


def explicar_fallo(indice: Union[int, str]) -> str:
    """Mensaje accionable cuando no hay camara. Se llama solo al fallar."""
    if ES_WINDOWS:
        return "No se abrio ninguna camara. Revisa que no la tenga otra app."
    hallados = dispositivos()
    if not hallados:
        return ("No hay ningun /dev/video*. La camara no esta enchufada o el "
                "kernel no la reconocio: comprueba con 'lsusb' y 'dmesg | tail'.")
    ocupa = ""
    try:
        ocupa = quien_la_usa(int(indice))
    except (TypeError, ValueError):
        pass
    if ocupa:
        return (f"/dev/video{indice} lo tiene abierto: {ocupa}.\n"
                f"  Ciérralo y vuelve a lanzar. Si es una ejecucion anterior "
                f"que quedo viva:  pkill -f 'python3 main.py'")
    return (f"Existen {[f'/dev/video{i}' for i in hallados]} pero ninguno "
            f"entrego imagen.\n"
            f"  - Si dice 'Device is busy', otro proceso la tiene: "
            f"pkill -f 'python3 main.py'\n"
            f"  - Mira cual es la de captura de verdad: v4l2-ctl --list-devices\n"
            f"  - Y fuerza el indice con: python3 main.py --camara N")


class Camara:
    """Captura en hilo propio. El lazo de vision llama a leer() y recibe
    siempre el frame mas reciente, o None si aun no ha llegado ninguno."""

    def __init__(self, indice: Union[int, str] = 0, ancho: int = 640,
                 alto: int = 480, fps: int = 30, fourcc: str = "MJPG",
                 voltear: bool = False, verbose: bool = True) -> None:
        self.cfg = dict(indice=indice, ancho=ancho, alto=alto, fps=fps,
                        fourcc=fourcc, verbose=verbose)
        self.voltear = voltear
        self._cap: Optional[cv2.VideoCapture] = None
        self._frame: Optional[np.ndarray] = None
        self._n = 0
        self._t_frame = 0.0
        self._lock = threading.Lock()
        self._parar = threading.Event()
        self._hilo: Optional[threading.Thread] = None
        self.fps_real = 0.0

    def abrir(self) -> bool:
        self._cap = abrir(**self.cfg)
        if self._cap is None:
            return False
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, name="camara",
                                      daemon=True)
        self._hilo.start()
        return True

    def _bucle(self) -> None:
        t_prev = time.time()
        fallos = 0
        while not self._parar.is_set():
            cap = self._cap
            if cap is None:
                break
            ok, frame = cap.read()
            if not ok or frame is None:
                fallos += 1
                # Un fallo aislado es normal (cambio de exposicion). Veinte
                # seguidos es un cable suelto: mejor decirlo que quedarse
                # conduciendo con el ultimo frame bueno de hace dos segundos.
                if fallos > 20:
                    with self._lock:
                        self._frame = None
                time.sleep(0.005)
                continue
            fallos = 0
            # Voltear 180 grados si la camara quedo montada al reves en el
            # mastil. Es mas barato aqui que en cada modulo aguas abajo.
            if self.voltear:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            ahora = time.time()
            with self._lock:
                self._frame = frame
                self._n += 1
                self._t_frame = ahora
                dt = ahora - t_prev
                if dt > 0:
                    self.fps_real = 0.9 * self.fps_real + 0.1 * (1.0 / dt)
                t_prev = ahora

    def leer(self) -> Tuple[Optional[np.ndarray], int, float]:
        """(frame, numero_de_frame, instante). El numero sirve para saber si
        el lazo esta procesando el mismo frame dos veces."""
        with self._lock:
            return self._frame, self._n, self._t_frame

    def cerrar(self) -> None:
        self._parar.set()
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
            self._hilo = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "Camara":
        self.abrir()
        return self

    def __exit__(self, *_) -> None:
        self.cerrar()
