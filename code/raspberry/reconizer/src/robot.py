"""
robot.py — El nucleo: camara -> vision -> navegacion -> ESP32.

Un solo objeto Robot al que se enganchan las dos interfaces (el panel de
escritorio y la web de carrito.local). Las dos leen el mismo estado y mandan
las mismas ordenes, asi que no hay dos verdades.

Hilo de control: captura, saca el muro y las lineas del suelo, calcula el
perfil, decide y manda. El envio al ESP32 lo hace `enlace` en su propio hilo a
50 Hz. Si este hilo se atasca, el enlace manda ceros a los 250 ms y el ESP32
corta a los 300 ms.

ARRANQUE SEGURO: el robot nace DESARMADO. Hay que pulsar ARMAR para que el
motor pueda moverse, y cualquier cosa rara lo desarma.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

import cv2
import numpy as np

from . import camera, color_config as cc, enlace as enl
from . import navegacion as nav
from . import protocolo as P
from . import obstaculos as obst_mod
from . import robot_config, sensores as sens_mod, vision, vueltas as vueltas_mod


class Robot:
    def __init__(self, cfg: Dict[str, Any], perfil_color: Dict[str, Any],
                 simulado: bool = False, fuente_imagen: Optional[str] = None):
        self.cfg = cfg
        self.perfil_color = perfil_color
        self.simulado = simulado
        self.fuente_imagen = fuente_imagen

        self.vision = vision.Vision(perfil_color["colores"])
        self.navegador = nav.Navegador(cfg["navegacion"], cfg["limites"],
                                       cfg.get("vueltas", {}))
        self.sensores = sens_mod.Sensores(cfg.get("sensores", {}), al_log=self.log)
        self.enlace = enl.Enlace(
            cfg["enlace"], simulado=simulado, al_log=self.log,
            al_imu=self.sensores.desde_esp32_imu,
            al_color=self._color_esp32,
            al_sensores=self.sensores.desde_esp32_estado)
        self.contador = vueltas_mod.ContadorVueltas(cfg.get("vueltas", {}), al_log=self.log)
        self.lineas_camara = vueltas_mod.DetectorLineasCamara(cfg.get("vueltas", {}))
        self.esquiva = obst_mod.EsquivaPilares(cfg.get("obstaculos", {}))

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
        self._sentido_lineas_visto = 0

    # -- registro ---------------------------------------------------------
    def log(self, txt: str) -> None:
        linea = f"{time.strftime('%H:%M:%S')} {txt}"
        print(linea)
        self.registro.append(linea)
        if len(self.registro) > 200:
            del self.registro[:100]

    def _color_esp32(self, ev: P.EventoColor) -> None:
        self.sensores.desde_esp32_color(ev)
        if ev.linea != P.LINEA_NINGUNA:
            self.contador.evento_linea(ev.linea, "tcs")

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
        self.sensores.iniciar_respaldo_pi()

        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="control")
        self._hilo.start()

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
        self.sensores.parar()
        if self._cap is not None:
            self._cap.release()

    # -- mandos -----------------------------------------------------------
    def armar(self, si: bool) -> None:
        self.armado = bool(si)
        if si:
            self.enlace.rearmar()
            self.navegador.reiniciar()
            if self.sensores.hay_rumbo:
                self.navegador.rumbo_objetivo = self.sensores.yaw
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

    def nueva_carrera(self) -> None:
        """Contadores a cero para volver a empezar sin reiniciar el programa."""
        self.contador.reiniciar()
        self.navegador.reiniciar(todo=True)
        self.esquiva.reiniciar()
        self.sensores.poner_cero(self.enlace)
        self.log("[robot] carrera reiniciada")

    def aplicar_config(self) -> None:
        self.enlace.fijar_vmax(int(self.cfg["limites"]["vmax"]))
        self.navegador.cfg = self.cfg["navegacion"]
        self.navegador.lim = self.cfg["limites"]
        self.navegador.cfg_vueltas = self.cfg.get("vueltas", {})
        self.navegador.carril.cfg = self.cfg["navegacion"]
        self.navegador.sentido.cfg = self.cfg["navegacion"]
        self.contador.cfg = self.cfg.get("vueltas", {})
        self.lineas_camara.cfg = self.cfg.get("vueltas", {})
        self.sensores.cfg = self.cfg.get("sensores", {})
        self.esquiva.cfg = self.cfg.get("obstaculos", {})

    def reintentar_sensores(self) -> None:
        """Le dice al ESP32 que vuelva a buscar el MPU6050 y el TCS34725.

        Hace falta porque los chips a veces tardan mas que el ESP32 en arrancar
        (sobre todo si comparten alimentacion con el motor) y se quedan fuera
        del sondeo inicial.
        """
        self.sensores.reintentar(self.enlace)
        self.log("[robot] reintentando la conexion de los sensores")

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

            self.sensores.actualizar()

            color_muro = str(self.cfg["navegacion"].get("color_muro", "negro"))
            quiere = [color_muro]
            usar_lineas_camara = self.sensores.origen_color in ("camara", "auto")
            if usar_lineas_camara:
                quiere += ["naranja", "azul"]
            hay_obstaculos = bool(self.cfg.get("obstaculos", {}).get("activo", False))
            if hay_obstaculos:
                quiere += ["rojo", "verde"]
            quiere = [c for c in quiere if c in self.perfil_color["colores"]]
            dets, masks = self.vision.procesar(frame, solo=quiere)
            mascara = masks.get(color_muro)
            if mascara is None:
                mascara = np.zeros(frame.shape[:2], np.uint8)

            # ---- lineas del suelo por camara -----------------------------
            if usar_lineas_camara:
                cruce = self.lineas_camara.procesar(masks)
                if cruce != P.LINEA_NINGUNA:
                    self.sensores.desde_camara(cruce)
                    self.contador.evento_linea(cruce, "camara")

            perfil = nav.perfil_desde_mascara(mascara, self.cfg["navegacion"])
            yaw = self.sensores.yaw_o_none()

            # ---- vueltas y media vuelta ----------------------------------
            lado = self.navegador.tomar_giro()
            if lado is not None:
                self.contador.evento_giro(lado)
            if self._sentido_lineas_visto != self.contador.sentido_lineas:
                self._sentido_lineas_visto = self.contador.sentido_lineas
                if self.contador.sentido_lineas:
                    # horario segun las lineas -> pared externa a la derecha
                    self.navegador.paredes._votar(
                        "lineas",
                        nav.DER if self.contador.sentido_lineas > 0 else nav.IZQ, 2.0)
            if self.contador.media_vuelta_pendiente and not self.navegador.media_vuelta_pedida:
                self.navegador.pedir_media_vuelta()
            if self.navegador.media_vuelta_hecha and self.contador.media_vuelta_pendiente:
                self.contador.media_vuelta_completada()
                self.navegador.media_vuelta_hecha = False
            if self.contador.terminado:
                self.navegador.terminado = True

            # ---- esquiva de pilares ---------------------------------------
            res_esquiva = self.esquiva.paso(dets, perfil, self.cfg["navegacion"]) \
                if hay_obstaculos else None

            # ---- decision -------------------------------------------------
            if self.modo == "auto" and self.armado:
                d = self.navegador.paso(perfil, yaw, esquiva=res_esquiva)
            elif self.modo == "manual" and self.armado:
                d = nav.Decision(vel=self.manual["vel"], direccion=self.manual["dir"],
                                 estado="manual", motivo="mando manual",
                                 metricas=self.navegador._metricas(perfil, yaw))
                if perfil.pasillo < self.navegador._umbral("parar_bajo") and d.vel > 0:
                    d.vel = 0
                    d.motivo = "manual bloqueado: muro delante"
            else:
                d = nav.Decision(vel=0, direccion=0, estado="parado",
                                 motivo="desarmado" if not self.armado else self.modo,
                                 metricas=self.navegador._metricas(perfil, yaw))

            if self.contador.terminado and self.armado:
                self.armar(False)
                self.log("[robot] recorrido completo: desarmado")

            self.enlace.mandar(d.vel, d.direccion,
                               armado=self.armado and self.modo != "parado")

            anotado = frame.copy()
            vision.dibujar_detecciones(anotado, dets.get(color_muro, []),
                                       self.perfil_color["colores"][color_muro]["color_dibujo"],
                                       etiqueta=False)
            nav.dibujar_navegacion(anotado, perfil, d, self.cfg["navegacion"], self.navegador)
            if hay_obstaculos and res_esquiva is not None:
                obst_mod.dibujar_pilares(anotado, dets, res_esquiva)
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

        c = self.contador
        v = f"vuelta {c.vueltas}/{c.objetivo} ({'ida' if c.tramo == 0 else 'vuelta'})"
        v += f"  esq {c.esquinas % c.esquinas_por_vuelta}/{c.esquinas_por_vuelta}"
        cv2.putText(img, v, (W - 260, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 0), 1, cv2.LINE_AA)
        if self.sensores.hay_rumbo:
            cv2.putText(img, f"yaw {self.sensores.yaw:+6.1f} ({self.sensores.origen_rumbo})",
                        (W - 260, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 200, 0), 1, cv2.LINE_AA)

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
            "vueltas_cfg": self.cfg.get("vueltas", {}),
            "vueltas": self.contador.estado(),
            "enlace": self.enlace.estado(),
            "sensores": self.sensores.estado(),
            "obstaculos": self.esquiva.estado(),
            "obstaculos_cfg": self.cfg.get("obstaculos", {}),
            "lineas_camara": self.lineas_camara.fracciones,
            "camara_error": self.error_camara,
            "perfil_color": self.perfil_color.get("nombre", ""),
        }
