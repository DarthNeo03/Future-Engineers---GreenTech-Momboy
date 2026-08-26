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
from . import geometria as geo
from . import navegacion as nav
from . import obstaculos as obs
from . import robot_config, vision


class Robot:
    def __init__(self, cfg: Dict[str, Any], perfil_color: Dict[str, Any],
                 simulado: bool = False, fuente_imagen: Optional[str] = None):
        self.cfg = cfg
        self.perfil_color = perfil_color
        self.simulado = simulado
        self.fuente_imagen = fuente_imagen

        self.vision = vision.Vision(perfil_color["colores"])
        self.suelo = geo.Suelo(cfg["camara"]).cargar()
        self.navegador = nav.Navegador(cfg["navegacion"], cfg["limites"])
        self.senales = obs.DetectorSenales(cfg["obstaculos"])
        self.enlace = enl.Enlace(cfg["enlace"], simulado=simulado, al_log=self.log)
        self.imu = imu_mod.IMU(cfg["imu"])

        # "abierto" = Open Challenge, "obstaculos" = Obstacle Challenge.
        # Los dos comparten la MISMA logica de muros y de giro; el modo de
        # obstaculos solo añade un objetivo lateral cuando ve una señal.
        self.reto = str(cfg.get("reto", "abierto"))

        self.armado = False
        self.modo = "auto"          # "auto" | "manual" | "parado"
        self.manual = {"vel": 0, "dir": 0}
        # Hombre-muerto del mando manual. El joystick de la web reenvia su
        # posicion cada ~150 ms; si dejan de llegar ordenes (el movil se
        # bloquea, se cae el WiFi, se cierra la pestana con el stick empujado)
        # el mando caduca y se va a cero. Sin esto el carro seguiria a fondo:
        # el lazo de vision sigue refrescando el enlace con el ULTIMO valor,
        # asi que ni el watchdog de 250 ms ni el failsafe del ESP32 saltarian
        # nunca. Es el mismo error que el enlace evita al no repetir ordenes
        # viejas, solo que un nivel mas arriba.
        self.manual_caduca_s = 0.4
        self._t_manual = 0.0

        self.frame: Optional[np.ndarray] = None
        self.frame_anotado: Optional[np.ndarray] = None
        self.mascara_muro: Optional[np.ndarray] = None
        self.escaneo: Optional[geo.Escaneo] = None
        self.lista_senales: List[obs.Senal] = []
        self.senal_activa: Optional[obs.Senal] = None
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

        # Diagnostico del campo de vision. Con lente estrecha el muro externo
        # simplemente no entra en el encuadre en campo cercano y el guardia de
        # la regla 9.18 no puede protegerte. Mas vale saberlo al arrancar que
        # descubrirlo rozando la pared.
        nav_cfg = self.cfg["navegacion"]
        self.log("[suelo] " + self.suelo.origen)
        self.log("[suelo] " + self.suelo.diagnostico_fov(
            float(nav_cfg.get("pared_objetivo_mm", 250.0))))
        if not self.suelo.calibrado:
            self.log("[suelo] SIN CALIBRAR: corre tools/calibrar_suelo.py")
        z_min = self.suelo.z_minimo_medible(int(self.cfg["camara"]["alto"]),
                                            float(nav_cfg.get("ignorar_abajo", 0.0)))
        parar = float(nav_cfg.get("parar_mm", 280.0))
        self.log(f"[suelo] distancia minima medible: {z_min:.0f} mm")
        if z_min == z_min and parar < z_min:
            self.log(f"[suelo] AVISO: parar_mm={parar:.0f} esta por DEBAJO del "
                     f"minimo medible ({z_min:.0f}): la parada de seguridad no "
                     f"podria dispararse. Sube parar_mm o baja la camara.")

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
            self.senales.activa = None
            if self.imu.disponible:
                self.navegador.rumbo_objetivo = self.imu.yaw
                self.navegador.yaw_prev = self.imu.yaw
            self.log(f"[robot] ARMADO (reto: {self.reto})")
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
            # Al entrar en manual se empieza SIEMPRE parado: nunca se hereda
            # la ultima posicion del joystick de una sesion anterior.
            self.manual = {"vel": 0, "dir": 0}
            self._t_manual = 0.0
            self.navegador.reiniciar()
            self.log(f"[robot] modo {modo}")

    def fijar_reto(self, reto: str) -> None:
        """Cambia entre Open Challenge y Obstacle Challenge.

        Los dos usan el mismo navegador: lo unico que cambia es que en
        obstaculos se buscan tambien pilares rojos y verdes y se le pasa al
        navegador un objetivo lateral. La logica de esquina no se toca.
        """
        if reto in ("abierto", "obstaculos"):
            self.reto = reto
            self.cfg["reto"] = reto
            self.navegador.reiniciar()
            self.senales.activa = None
            self.log(f"[robot] reto: {reto}")

    def mando_manual(self, vel: int, direccion: int) -> None:
        self.manual = {"vel": int(vel), "dir": int(direccion)}
        self._t_manual = time.monotonic()

    def aplicar_config(self) -> None:
        """Releer limites tras cambiarlos desde la interfaz."""
        self.enlace.fijar_vmax(int(self.cfg["limites"]["vmax"]))
        self.navegador.cfg = self.cfg["navegacion"]
        self.navegador.lim = self.cfg["limites"]
        self.senales.cfg = self.cfg["obstaculos"]

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

            cfg_nav = self.cfg["navegacion"]
            color_muro = str(cfg_nav.get("color_muro", "negro"))
            # En el reto de obstaculos hace falta ademas rojo y verde. Es una
            # sola conversion a HSV para los tres, no tres pasadas.
            quiero = [color_muro]
            if self.reto == "obstaculos":
                quiero += [obs.ROJO, obs.VERDE]
            dets, masks = self.vision.procesar(frame, solo=quiero)
            mascara = masks.get(color_muro)
            if mascara is None:
                mascara = np.zeros(frame.shape[:2], np.uint8)

            escaneo = geo.escanear(mascara, self.suelo, cfg_nav)
            yaw = self.imu.yaw if self.imu.disponible else None

            # --- señales de trafico (solo en el reto de obstaculos) --------
            objetivo_lateral = None
            motivo_extra = ""
            if self.reto == "obstaculos":
                H_img, W_img = frame.shape[:2]
                self.lista_senales = self.senales.desde_detecciones(
                    dets, self.suelo, W_img, H_img)
                self.senal_activa = self.senales.elegir(self.lista_senales)
                # El objetivo lateral solo aplica en recta: una esquina se toma
                # igual haya o no pilares, asi que en GIRO se ignora.
                if self.navegador.estado != nav.GIRO:
                    objetivo_lateral = self.senales.objetivo(self.senal_activa)
                    if objetivo_lateral is not None and self.senal_activa is not None:
                        lado = "der" if self.senal_activa.lado_paso > 0 else "izq"
                        motivo_extra = (f"{self.senal_activa.color} a "
                                        f"{self.senal_activa.z:.0f} mm: paso por {lado}")
            else:
                self.lista_senales = []
                self.senal_activa = None

            if self.modo == "auto" and self.armado:
                d = self.navegador.paso(escaneo, yaw,
                                        objetivo_lateral=objetivo_lateral,
                                        motivo_extra=motivo_extra)
                if self.navegador.terminado and self.armado:
                    self.armar(False)
                    self.log("[robot] tres vueltas: desarmo y me quedo quieto")
            elif self.modo == "manual" and self.armado:
                # Si el mando caduco, cero y servo al centro: el carro se para
                # recto, igual que hace el ESP32 cuando la Pi se calla.
                if time.monotonic() - self._t_manual > self.manual_caduca_s:
                    if self.manual["vel"] or self.manual["dir"]:
                        self.manual = {"vel": 0, "dir": 0}
                        self.log("[robot] mando manual caducado: paro")
                    d = nav.Decision(vel=0, direccion=0, estado="manual",
                                     motivo="mando manual caducado",
                                     metricas=self.navegador.metricas(escaneo, yaw))
                else:
                    d = nav.Decision(vel=self.manual["vel"], direccion=self.manual["dir"],
                                     estado="manual", motivo="mando manual",
                                     metricas=self.navegador.metricas(escaneo, yaw))
                # La seguridad manda incluso en manual: si hay muro delante
                # de verdad, no dejamos que el mando meta el carro en la pared.
                frente = escaneo.frente(float(cfg_nav.get("semiancho_carro_mm", 110.0)))
                if frente < float(cfg_nav.get("parar_mm", 190.0)) and d.vel > 0:
                    d.vel = 0
                    d.motivo = f"manual bloqueado: muro a {frente:.0f} mm"
            else:
                d = nav.Decision(vel=0, direccion=0, estado="parado",
                                 motivo="desarmado" if not self.armado else self.modo,
                                 metricas=self.navegador.metricas(escaneo, yaw))

            self.enlace.mandar(d.vel, d.direccion,
                               armado=self.armado and self.modo != "parado")

            anotado = frame.copy()
            nav.dibujar_navegacion(anotado, escaneo, d, self.navegador, cfg_nav)
            if self.reto == "obstaculos":
                obs.dibujar_senales(anotado, self.lista_senales, self.senal_activa)
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
                self.escaneo = escaneo
                self.decision = d

    def _pie(self, img: np.ndarray, d: nav.Decision) -> None:
        H, W = img.shape[:2]
        e = self.enlace
        estado = "ARMADO" if self.armado else "desarmado"
        color = (0, 0, 255) if self.armado else (180, 180, 180)
        reto = "OPEN" if self.reto == "abierto" else "OBSTACULOS"
        cv2.putText(img, f"{estado} | {self.modo} | {reto} | {self.fps:4.1f} fps",
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
            "reto": self.reto,
            "fps": round(self.fps, 1),
            "suelo": {"calibrado": self.suelo.calibrado, "origen": self.suelo.origen},
            "carrera": {
                "vueltas": self.navegador.vueltas,
                "giros": self.navegador.giros,
                "lado_interno": self.navegador.lado_interno,
                "yaw_acum": round(self.navegador.yaw_acumulado, 1),
                "terminado": self.navegador.terminado,
            },
            "senal": (None if self.senal_activa is None else {
                "color": self.senal_activa.color,
                "z": round(self.senal_activa.z),
                "x": round(self.senal_activa.x),
                "lado": "der" if self.senal_activa.lado_paso > 0 else "izq",
            }),
            "n_senales": len(self.lista_senales),
            "decision": {"vel": d.vel, "dir": d.direccion, "estado": d.estado,
                         "motivo": d.motivo, "metricas": d.metricas},
            "limites": self.cfg["limites"],
            "navegacion": self.cfg["navegacion"],
            "enlace": self.enlace.estado(),
            "imu": self.imu.estado(),
            "camara_error": self.error_camara,
            "perfil_color": self.perfil_color.get("nombre", ""),
        }
