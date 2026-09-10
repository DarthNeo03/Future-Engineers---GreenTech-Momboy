"""
robot.py — El nucleo: camara -> vision -> navegacion -> ESP32.

Un solo objeto Robot al que se enganchan las dos interfaces (el panel de
escritorio y la web de carrito.local). Las dos leen el mismo estado y mandan
las mismas ordenes, asi que no hay dos verdades.

Hilo de control: captura, procesa el color del muro, calcula el perfil, decide
y manda. El envio al ESP32 lo hace `enlace` en su propio hilo a 50 Hz; aqui
solo se refresca la orden. Si este hilo se atasca, el enlace manda ceros a los
250 ms y el ESP32 corta a los 300 ms.

ARRANQUE SEGURO: el robot nace DESARMADO. Hay que pulsar ARMAR (en el panel o
en la web) para que el motor pueda moverse, y cualquier cosa rara lo desarma.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

import cv2
import numpy as np

from . import camera, color_config as cc, enlace as enl, imu as imu_mod
from . import navegacion as nav
from . import robot_config, vision


class Robot:
    def __init__(self, cfg: Dict[str, Any], perfil_color: Dict[str, Any],
                 simulado: bool = False, fuente_imagen: Optional[str] = None):
        self.cfg = cfg
        self.perfil_color = perfil_color
        self.simulado = simulado
        self.fuente_imagen = fuente_imagen

        self.vision = vision.Vision(perfil_color["colores"])
        self.navegador = nav.Navegador(cfg["navegacion"], cfg["limites"])
        self.enlace = enl.Enlace(cfg["enlace"], simulado=simulado, al_log=self.log)
        self.imu = imu_mod.IMU(cfg["imu"])

        self.armado = False
        self.modo = "auto"          # "auto" | "manual" | "parado"
        self.manual = {"vel": 0, "dir": 0}

        self.frame: Optional[np.ndarray] = None
        self.frame_anotado: Optional[np.ndarray] = None
        self.mascara_muro: Optional[np.ndarray] = None
        self.perfil: Optional[nav.PerfilMuro] = None
        self.decision = nav.Decision()
        self.fps = 0.0
        self.error_camara = ""
        self.registro: List[str] = []

        self._cap = None
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._lock = threading.Lock()
        self._imagen_fija: Optional[np.ndarray] = None

    # -- registro ---------------------------------------------------------
    def log(self, txt: str) -> None:
        marca = time.strftime("%H:%M:%S")
        linea = f"{marca} {txt}"
        print(linea)
        self.registro.append(linea)
        if len(self.registro) > 200:
            del self.registro[:100]

    # -- arranque ---------------------------------------------------------
    def iniciar(self) -> None:
        if self.fuente_imagen:
            img = cv2.imread(self.fuente_imagen)
            if img is None:
                raise SystemExit(f"No se pudo leer {self.fuente_imagen}")
            self._imagen_fija = img
            self.log(f"[robot] usando imagen fija {self.fuente_imagen}")
        else:
            c = self.cfg["camara"]
            self._cap = camera.abrir(indice=c["indice"], ancho=c["ancho"],
                                     alto=c["alto"], fps=c["fps"], fourcc=c["fourcc"])
            if self._cap is None:
                self.error_camara = "no se pudo abrir la camara"
                self.log("[robot] " + self.error_camara)

        self.enlace.fijar_vmax(int(self.cfg["limites"]["vmax"]))
        self.enlace.iniciar()

        if self.imu.iniciar():
            self.log(f"[imu] {self.imu.motivo}; calibrando, NO MUEVAS EL CARRO")
            threading.Thread(target=self._calibrar_imu, daemon=True).start()
        else:
            self.log(f"[imu] sin giroscopio: {self.imu.motivo} (se sigue solo con camara)")

        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="control")
        self._hilo.start()

    def _calibrar_imu(self):
        if self.imu.calibrar():
            self.log("[imu] calibrado, yaw a cero")
        else:
            self.log("[imu] no se pudo calibrar")

    def cerrar(self) -> None:
        self._parar.set()
        if self._hilo:
            self._hilo.join(timeout=2.0)
        try:
            self.enlace.parar(emergencia=True)
            time.sleep(0.1)
        except Exception:
            pass
        self.enlace.cerrar()
        self.imu.parar()
        if self._cap is not None:
            self._cap.release()

    # -- mandos -----------------------------------------------------------
    def armar(self, si: bool) -> None:
        self.armado = bool(si)
        if si:
            self.enlace.rearmar()
            self.navegador.reiniciar()
            if self.imu.disponible:
                self.navegador.rumbo_objetivo = self.imu.yaw
            self.log("[robot] ARMADO")
        else:
            self.enlace.parar()
            self.log("[robot] desarmado")

    def emergencia(self) -> None:
        self.armado = False
        self.modo = "parado"
        self.enlace.parar(emergencia=True)
        self.log("[robot] PARADA DE EMERGENCIA")

    def fijar_modo(self, modo: str) -> None:
        if modo in ("auto", "manual", "parado"):
            self.modo = modo
            self.navegador.reiniciar()
            self.log(f"[robot] modo {modo}")

    def mando_manual(self, vel: int, direccion: int) -> None:
        self.manual = {"vel": int(vel), "dir": int(direccion)}

    def aplicar_config(self) -> None:
        """Releer limites tras cambiarlos desde la interfaz."""
        self.enlace.fijar_vmax(int(self.cfg["limites"]["vmax"]))
        self.navegador.cfg = self.cfg["navegacion"]
        self.navegador.lim = self.cfg["limites"]

    def guardar_config(self) -> None:
        robot_config.guardar(self.cfg)
        self.log("[robot] configuracion guardada")

    def recargar_colores(self, nombre: Optional[str] = None) -> None:
        datos = cc.cargar()
        perfil = cc.obtener(datos, nombre)
        self.perfil_color = perfil
        self.vision.actualizar(perfil["colores"])
        self.log(f"[robot] colores del perfil '{perfil['nombre']}'")

    # -- bucle ------------------------------------------------------------
    def _leer_frame(self) -> Optional[np.ndarray]:
        if self._imagen_fija is not None:
            return self._imagen_fija.copy()
        if self._cap is None:
            return None
        ok, f = self._cap.read()
        if not ok:
            return None
        if self.cfg["camara"].get("voltear"):
            f = cv2.flip(f, -1)
        return f

    def _bucle(self):
        t_prev = time.perf_counter()
        sin_frame = 0
        while not self._parar.is_set():
            frame = self._leer_frame()
            if frame is None:
                sin_frame += 1
                if sin_frame == 5:
                    self.log("[robot] sin imagen: desarmo por seguridad")
                    self.armar(False)
                time.sleep(0.05)
                continue
            sin_frame = 0

            color_muro = str(self.cfg["navegacion"].get("color_muro", "negro"))
            dets, masks = self.vision.procesar(frame, solo=[color_muro])
            mascara = masks.get(color_muro)
            if mascara is None:
                mascara = np.zeros(frame.shape[:2], np.uint8)

            perfil = nav.perfil_desde_mascara(mascara, self.cfg["navegacion"])
            yaw = self.imu.yaw if self.imu.disponible else None

            if self.modo == "auto" and self.armado:
                d = self.navegador.paso(perfil, yaw)
            elif self.modo == "manual" and self.armado:
                d = nav.Decision(vel=self.manual["vel"], direccion=self.manual["dir"],
                                 estado="manual", motivo="mando manual",
                                 metricas=self.navegador._metricas(perfil, yaw))
                # La seguridad manda incluso en manual: si el pasillo se cierra
                # de verdad, no dejamos que el mando meta el carro en la pared.
                if perfil.pasillo < float(self.cfg["navegacion"]["parar_bajo"]) and d.vel > 0:
                    d.vel = 0
                    d.motivo = "manual bloqueado: muro delante"
            else:
                d = nav.Decision(vel=0, direccion=0, estado="parado",
                                 motivo="desarmado" if not self.armado else self.modo,
                                 metricas=self.navegador._metricas(perfil, yaw))

            self.enlace.mandar(d.vel, d.direccion,
                               armado=self.armado and self.modo != "parado")

            anotado = frame.copy()
            vision.dibujar_detecciones(anotado, dets.get(color_muro, []),
                                       self.perfil_color["colores"][color_muro]["color_dibujo"],
                                       etiqueta=False)
            nav.dibujar_navegacion(anotado, perfil, d, self.cfg["navegacion"])
            self._pie(anotado, d)

            ahora = time.perf_counter()
            dt = ahora - t_prev
            t_prev = ahora
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 / dt

            with self._lock:
                self.frame = frame
                self.frame_anotado = anotado
                self.mascara_muro = mascara
                self.perfil = perfil
                self.decision = d

    def _pie(self, img: np.ndarray, d: nav.Decision) -> None:
        H, W = img.shape[:2]
        e = self.enlace
        estado = "ARMADO" if self.armado else "desarmado"
        color = (0, 0, 255) if self.armado else (180, 180, 180)
        cv2.putText(img, f"{estado} | {self.modo} | {self.fps:4.1f} fps",
                    (8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        txt = "ESP32 SIMULADO" if self.simulado else f"ESP32 {'OK' if e.conectado else 'NO'}"
        if e.conectado and not self.simulado:
            txt += f" {e.latencia_ms:.0f}ms pwm={e.telemetria.pwm} ang={e.telemetria.angulo}"
            if e.telemetria.failsafe:
                txt += " FAILSAFE"
        cv2.putText(img, txt, (8, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 255, 0) if e.conectado else (0, 0, 255), 1, cv2.LINE_AA)
        if self.imu.disponible:
            cv2.putText(img, f"yaw {self.imu.yaw:+6.1f}", (W - 110, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1, cv2.LINE_AA)

    # -- lectura para las interfaces ---------------------------------------
    def instantanea(self):
        with self._lock:
            return (None if self.frame_anotado is None else self.frame_anotado.copy(),
                    None if self.mascara_muro is None else self.mascara_muro.copy())

    def estado(self) -> Dict[str, Any]:
        d = self.decision
        return {
            "armado": self.armado,
            "modo": self.modo,
            "fps": round(self.fps, 1),
            "decision": {"vel": d.vel, "dir": d.direccion, "estado": d.estado,
                         "motivo": d.motivo, "metricas": d.metricas},
            "limites": self.cfg["limites"],
            "navegacion": self.cfg["navegacion"],
            "enlace": self.enlace.estado(),
            "imu": self.imu.estado(),
            "camara_error": self.error_camara,
            "perfil_color": self.perfil_color.get("nombre", ""),
        }
