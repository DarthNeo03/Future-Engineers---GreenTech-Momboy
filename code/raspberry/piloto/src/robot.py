"""
robot.py — El nucleo: camara -> vision -> muro -> navegacion -> ESP32.

Un solo objeto Robot; la web lee su estado y le manda ordenes. El hilo de
control captura, procesa y decide; el enlace manda a 50 Hz en su propio hilo.
Si este hilo se atasca >250 ms, el enlace manda ceros; si el serial se calla
>300 ms, el ESP32 corta solo. El robot NACE DESARMADO: hay que pulsar ARMAR.

Al ARMAR en modo auto arranca la carrera (cronometro + conteo de esquinas).
En competencia, ARMAR es el "boton de start" (se puede cablear uno fisico a
la Pi mas adelante; el metodo armar() ya esta listo para eso).
"""

from __future__ import annotations

import copy
import json
import threading
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from . import camara as cam_mod
from . import color_config as cc
from . import dibujo, muro, params as params_mod, vision
from . import protocolo as P
from .carrera import Carrera, LISTO
from .enlace import Enlace
from .geometria import Geometria
from .lineas import GestorLineas
from . import navegacion as nav
from .navegacion import Decision, Navegador
from .obstaculos import Esquivador


class Robot:
    def __init__(self, datos_params: Dict[str, Any], datos_colores: Dict[str, Any],
                 simulado: bool = False, fuente_imagen: Optional[str] = None):
        self.datos_params = datos_params
        self.datos_colores = datos_colores
        self.p: Dict[str, Dict[str, Any]] = params_mod.obtener(datos_params)["valores"]
        self.perfil_color = cc.obtener(datos_colores)
        self.simulado = simulado
        self.fuente_imagen = fuente_imagen

        self.geo = Geometria(self.p["geometria"])
        self.vision = vision.Vision(self.perfil_color["colores"])
        self.lineas = GestorLineas(self.p["lineas"])
        self.carrera = Carrera(self.p["carrera"], self.lineas)
        self.navegador = Navegador(self.p["navegacion"], self.p["limites"],
                                   self.p["escape"], self.p["giro2t"],
                                   al_completar_giro=self._giro_completado)
        self.esquivador = Esquivador(self.p["obstaculos"])
        self.enlace = Enlace(self.p["enlace"], simulado=simulado, al_log=self.log)

        self.armado = False
        self.modo = "auto"                  # "auto" | "manual" | "parado"
        self.manual = {"vel": 0, "dir": 0, "t": 0.0}

        self.frame_crudo: Optional[np.ndarray] = None
        self.hsv: Optional[np.ndarray] = None
        self.frame_anotado: Optional[np.ndarray] = None
        self.masks: Dict[str, np.ndarray] = {}
        self.perfil: Optional[muro.PerfilMuro] = None
        self.decision = Decision()
        self.fps = 0.0
        self.error_camara = ""
        self.registro: List[str] = []
        self.t_linea_reciente = 0.0

        self._cap = None
        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._lock = threading.Lock()
        self._imagen_fija: Optional[np.ndarray] = None
        self._cfg_esp_firma = ""
        self._t_cfg_esp = 0.0
        self._cal_giro_hecha = False

    # -- registro ---------------------------------------------------------
    def log(self, txt: str) -> None:
        linea = f"{time.strftime('%H:%M:%S')} {txt}"
        print(linea)
        self.registro.append(linea)
        if len(self.registro) > 300:
            del self.registro[:150]

    # -- arranque ---------------------------------------------------------
    def iniciar(self) -> None:
        if self.fuente_imagen:
            img = cv2.imread(self.fuente_imagen)
            if img is None:
                raise SystemExit(f"No se pudo leer {self.fuente_imagen}")
            self._imagen_fija = img
            self.log(f"[robot] usando imagen fija {self.fuente_imagen}")
        else:
            self._abrir_camara()

        self.enlace.fijar_vmax(int(self.p["limites"]["vmax"]))
        self.enlace.iniciar()

        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True, name="control")
        self._hilo.start()

    def _abrir_camara(self) -> None:
        c = self.p["camara"]
        self._cap = cam_mod.abrir(indice=c["indice"], ancho=c["ancho"],
                                  alto=c["alto"], fps=c["fps"], fourcc="MJPG")
        if self._cap is None:
            self.error_camara = "no se pudo abrir la camara"
            self.log("[robot] " + self.error_camara)
        else:
            self.error_camara = ""
            self.aplicar_camara()

    def aplicar_camara(self) -> None:
        """Aplica exposicion/balance manual si estan configurados."""
        if self._cap is None:
            return
        c = self.p["camara"]
        exp = float(c.get("exposicion", -1))
        bb = float(c.get("balance_blancos", -1))
        cam_mod.fijar_manual(self._cap,
                             exposicion=None if exp < 0 else exp,
                             balance_blancos=None if bb < 0 else bb)
        self.log(f"[camara] exposicion={'auto' if exp < 0 else exp} "
                 f"balance={'auto' if bb < 0 else bb}")

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
        if self._cap is not None:
            self._cap.release()

    # -- mandos -----------------------------------------------------------
    def armar(self, si: bool) -> None:
        self.armado = bool(si)
        if si:
            self.enlace.rearmar()
            self.navegador.reiniciar()
            if self.modo == "auto":
                self.carrera.arrancar()
                self.log("[robot] ARMADO: carrera en marcha")
            else:
                self.log("[robot] ARMADO (manual)")
        else:
            self.enlace.parar()
            self.log("[robot] desarmado")

    def emergencia(self) -> None:
        self.armado = False
        self.enlace.parar(emergencia=True)
        self.log("[robot] PARADA DE EMERGENCIA")

    def fijar_modo(self, modo: str) -> None:
        if modo in ("auto", "manual", "parado"):
            self.modo = modo
            self.navegador.reiniciar()
            if modo == "auto":
                self.carrera.reiniciar()
                if self.armado:
                    self.carrera.arrancar()
            self.log(f"[robot] modo {modo}")

    def mando_manual(self, vel: int, direccion: int) -> None:
        tope = int(self.p["manual"]["vel_max_manual"])
        self.manual = {"vel": max(-tope, min(tope, int(vel))),
                       "dir": max(-100, min(100, int(direccion))),
                       "t": time.time()}

    def _giro_completado(self, lado: int) -> None:
        self.lineas.giro_completado(lado)

    # -- parametros y perfiles --------------------------------------------
    def fijar_param(self, grupo: str, clave: str, valor: Any) -> Any:
        v = params_mod.validar(grupo, clave, valor)
        self.p[grupo][clave] = v
        if grupo == "limites" and clave == "vmax":
            self.enlace.fijar_vmax(int(v))
        if grupo in ("servo", "tcs"):
            self._cfg_esp_firma = ""          # forzar reenvio al ESP32
        if grupo == "camara" and clave in ("exposicion", "balance_blancos"):
            self.aplicar_camara()
        return v

    def guardar_perfil_params(self, nombre: str) -> None:
        params_mod.guardar_perfil(self.datos_params, nombre, self.p)
        params_mod.guardar_archivo(self.datos_params)
        self.log(f"[params] perfil '{nombre}' guardado")

    def cargar_perfil_params(self, nombre: str) -> None:
        perfil = params_mod.obtener(self.datos_params, nombre)
        nuevos = perfil["valores"]
        # actualizacion EN SITIO: todos los modulos comparten estos dicts
        for g, claves in nuevos.items():
            if g in self.p:
                self.p[g].update(copy.deepcopy(claves))
        self.datos_params["activo"] = perfil["nombre"]
        params_mod.guardar_archivo(self.datos_params)
        self.enlace.fijar_vmax(int(self.p["limites"]["vmax"]))
        self._cfg_esp_firma = ""
        self.log(f"[params] perfil '{perfil['nombre']}' cargado")

    def guardar_perfil_colores(self, nombre: str) -> None:
        cc.guardar_perfil(self.datos_colores, nombre,
                          self.perfil_color["colores"],
                          camara=self.perfil_color.get("camara"),
                          notas="calibrado desde la web")
        cc.guardar(self.datos_colores)
        self.perfil_color = cc.obtener(self.datos_colores, nombre)
        self.log(f"[colores] perfil '{nombre}' guardado")

    def cargar_perfil_colores(self, nombre: str) -> None:
        cc.fijar_activo(self.datos_colores, nombre)
        cc.guardar(self.datos_colores)
        self.perfil_color = copy.deepcopy(cc.obtener(self.datos_colores, nombre))
        self.vision.actualizar(self.perfil_color["colores"])
        self.log(f"[colores] perfil '{nombre}' activo")

    def fijar_color(self, color: str, clave: str, valor: Any) -> None:
        """Cambia en caliente un parametro de un color (sin guardar aun)."""
        c = self.perfil_color["colores"].get(color)
        if c is None:
            raise ValueError(f"color '{color}' no existe")
        c[clave] = valor
        self.perfil_color["colores"][color] = cc.normalizar_color(c)
        self.vision.actualizar(self.perfil_color["colores"])

    def clic_color(self, color: str, x_rel: float, y_rel: float,
                   acumular: bool = False) -> List:
        """Toma el color del pixel clicado (como el calibrador viejo):
        el pixel exacto manda y del parche solo sobreviven los parecidos.
        x_rel/y_rel vienen en 0..1 (fraccion de la imagen): asi da igual a
        que resolucion capture la camara o se reescale el stream."""
        with self._lock:
            hsv = None if self.hsv is None else self.hsv.copy()
        if hsv is None:
            raise ValueError("sin imagen aun")
        H, W = hsv.shape[:2]
        px = int(max(0, min(W - 1, x_rel * W)))
        py = int(max(0, min(H - 1, y_rel * H)))
        r = 4
        parche = hsv[max(0, py - r):py + r + 1, max(0, px - r):px + r + 1]
        ancla = hsv[py, px]
        nucleo = vision.nucleo_de_parche(parche.reshape(-1, 3), ancla)
        if acumular:
            previos = getattr(self, "_muestra_acumulada", None)
            if previos is not None and previos[0] == color:
                nucleo = np.vstack([previos[1], nucleo])
        self._muestra_acumulada = (color, nucleo)
        rangos = vision.rangos_desde_pixeles(nucleo)
        self.fijar_color(color, "rangos", rangos)
        return rangos

    # -- calibraciones ----------------------------------------------------
    def calibrar_giro(self) -> None:
        self.enlace.enviar_cal(P.CAL_GIRO)
        self.log("[imu] calibrando giroscopio: NO MUEVAS EL CARRO (~1 s)")

    def cero_yaw(self) -> None:
        self.enlace.enviar_cal(P.CAL_CERO_YAW)

    def redetectar_i2c(self) -> None:
        self.enlace.enviar_cal(P.CAL_REDETECTAR)
        self.log("[i2c] reintentando deteccion de sensores")

    def muestrear_tcs(self, que: str) -> Dict[str, Any]:
        """Calibra los umbrales del TCS con el sensor puesto SOBRE la
        superficie pedida ('blanco', 'naranja' o 'azul')."""
        historial = list(self.enlace.historial_tcs)[-25:]
        if len(historial) < 5:
            raise ValueError("sin lecturas del TCS (revisa que este conectado)")
        rs, bs, cs = [], [], []
        for (c, r, g, b) in historial:
            if c <= 0:
                continue
            rs.append(r * 255.0 / c)
            bs.append(b * 255.0 / c)
            cs.append(c)
        if not rs:
            raise ValueError("lecturas del TCS sin canal claro")
        rm, bm, cm = (sum(rs) / len(rs)), (sum(bs) / len(bs)), (sum(cs) / len(cs))
        t = self.p["tcs"]
        if que == "naranja":
            t["naranja_r_min"] = int(rm * 0.85)
            t["naranja_b_max"] = int(bm * 1.6 + 10)
        elif que == "azul":
            t["azul_b_min"] = int(bm * 0.85)
            t["azul_r_max"] = int(rm * 1.6 + 10)
        elif que == "blanco":
            # el blanco define la zona muerta: los umbrales de color deben
            # quedar bien lejos de estos ratios
            t["c_min"] = int(cm * 0.25)
        else:
            raise ValueError("que debe ser blanco/naranja/azul")
        self._cfg_esp_firma = ""
        self.log(f"[tcs] muestreado {que}: ratio_r={rm:.0f} ratio_b={bm:.0f} c={cm:.0f}")
        return {"ratio_r": round(rm), "ratio_b": round(bm), "c": round(cm)}

    def calibrar_fy(self, y_rel: float, distancia_mm: float) -> float:
        fy = self.geo.calibrar_fy(y_rel * self.geo.H, distancia_mm)
        self.fijar_param("geometria", "fy_px", fy)
        self.log(f"[geo] fy calibrada = {fy:.0f} px")
        return fy

    def calibrar_fx(self, x_rel: float, y_rel: float, lateral_mm: float) -> float:
        fx = self.geo.calibrar_fx(x_rel * self.geo.W, y_rel * self.geo.H,
                                  lateral_mm)
        self.fijar_param("geometria", "fx_px", fx)
        self.log(f"[geo] fx calibrada = {fx:.0f} px")
        return fx

    # -- bucle ------------------------------------------------------------
    def _leer_frame(self) -> Optional[np.ndarray]:
        if self._imagen_fija is not None:
            time.sleep(0.03)                 # simular ~30 fps
            return self._imagen_fija.copy()
        if self._cap is None:
            return None
        ok, f = self._cap.read()
        if not ok:
            return None
        if self.p["camara"].get("voltear"):
            f = cv2.flip(f, -1)
        return f

    def _mandar_cfg_esp32(self) -> None:
        """Manda la config de servo y TCS al ESP32 cuando cambie o reconecte."""
        if not self.enlace.conectado or self.simulado:
            return
        ahora = time.time()
        if ahora - self._t_cfg_esp < 2.0:
            return
        self._t_cfg_esp = ahora
        firma = json.dumps([self.p["servo"], self.p["tcs"]], sort_keys=True) + \
            self.enlace.puerto
        if firma == self._cfg_esp_firma:
            return
        self._cfg_esp_firma = firma
        s = self.p["servo"]
        self.enlace.enviar_config_servo(int(s["centro"]), int(s["izquierda"]),
                                        int(s["derecha"]), int(s["rampa_pwm"]),
                                        int(s["grados_s"]))
        self.enlace.enviar_cfg_tcs(self.p["tcs"])
        self.log("[esp32] configuracion de servo y TCS enviada")

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

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- vision ---------------------------------------------------
            masks = self.vision.solo_mascaras(hsv, ["blanco", "negro", "magenta"])
            quiere_det = ["naranja", "azul"]
            if bool(self.p["obstaculos"].get("activo")):
                quiere_det += ["rojo", "verde"]
            dets, masks_det = self.vision.detectar_en(hsv, quiere_det)
            masks.update(masks_det)

            # El perfil necesita saber cuanto se desvia el carro del rumbo de
            # la recta (giroscopio) y en que sentido corre la ronda: con eso
            # distingue la pared de su carril de la de enfrente aunque llegue
            # torcido a la curva.
            yaw_ahora = self.enlace.yaw()
            error_rumbo = self.navegador.error_de_rumbo(yaw_ahora)
            sentido_ahora = self.carrera.sentido()
            perfil: Optional[muro.PerfilMuro] = None
            try:
                perfil = muro.perfil(masks, self.geo, self.p["muro"],
                                     error_rumbo, sentido_ahora)
            except Exception as e:
                self.log(f"[muro] error en el perfil: {e}")

            # --- sensores y conteo ---------------------------------------
            for color, _t in self.enlace.eventos_linea():
                self.lineas.evento_tcs(color)
                self.t_linea_reciente = time.time()
            self.lineas.paso_camara(dets, perfil, self.geo)
            if self.lineas.ultimo_evento and \
                    time.time() - self.lineas._t_evento < 0.1:
                self.t_linea_reciente = time.time()

            self.lineas.paso_zona()          # timeout de la zona de esquina
            yaw = yaw_ahora
            sentido = self.carrera.sentido()
            linea_reciente = time.time() - self.t_linea_reciente < 1.2
            # "Dentro de la curva" tiene dos fuentes, y basta con una:
            #   - las lineas del piso (lo fiable, via TCS o camara);
            #   - que el propio navegador ya este girando.
            # La segunda importa porque el TCS puede no estar montado y las
            # lineas se ven fatal si la camara va sobreexpuesta: sin ella, el
            # anti-bucle no protegeria en el caso mas comun de todos.
            # Dos niveles, a proposito:
            #  - en_esquina: basta con que la vision haya decidido girar. Sirve
            #    para el anti-bucle, que no puede depender de tener el TCS
            #    montado ni de que se vean las lineas.
            #  - esquina_confirmada: SOLO las lineas del piso. Es la unica
            #    prueba fisica de que hay curva, y es lo que habilita el giro
            #    de dos tiempos, que es el que retrocede.
            esquina_confirmada = self.lineas.en_esquina
            en_esquina = (esquina_confirmada or
                          self.navegador.estado in (nav.PRE_GIRO, nav.GIRO,
                                                    nav.GIRO_2T))
            debe_parar = self.carrera.paso()

            bias = self.esquivador.paso(dets, perfil, self.geo)

            # --- decidir ---------------------------------------------------
            if self.modo == "auto" and self.armado and perfil is not None:
                if debe_parar:
                    d = Decision(vel=0, direccion=0, estado="meta",
                                 motivo="carrera terminada: parado en meta")
                else:
                    d = self.navegador.paso(perfil, yaw, sentido,
                                            linea_reciente, bias, en_esquina,
                                            esquina_confirmada)
            elif self.modo == "manual" and self.armado:
                caducado = (time.time() - self.manual["t"]) * 1000 > \
                    float(self.p["manual"]["timeout_ms"])
                v = 0 if caducado else self.manual["vel"]
                motivo = "joystick sin señal" if caducado else "mando manual"
                if (bool(self.p["manual"]["manual_seguro"]) and v > 0 and
                        perfil is not None and
                        perfil.pasillo_mm < float(self.p["navegacion"]["parar_bajo_mm"])):
                    v = 0
                    motivo = "manual bloqueado: muro delante"
                d = Decision(vel=v, direccion=self.manual["dir"],
                             estado="manual", motivo=motivo)
            else:
                d = Decision(vel=0, direccion=0, estado="parado",
                             motivo="desarmado" if not self.armado else self.modo)

            self.enlace.mandar(d.vel, d.direccion,
                               armado=self.armado and self.modo != "parado")
            self._mandar_cfg_esp32()

            # calibracion automatica del giroscopio al detectarlo (el carro
            # esta quieto durante la preparacion; en carrera ya no se repite)
            if (not self._cal_giro_hecha and not self.armado and
                    self.enlace.sensores.mpu_ok):
                self._cal_giro_hecha = True
                self.calibrar_giro()

            # --- dibujar ---------------------------------------------------
            anotado = frame.copy()
            hud = self._hud(yaw)
            try:
                dibujo.anotar(anotado, perfil, d, self.geo, dets,
                              self.perfil_color["colores"],
                              self.carrera.estado_dict(),
                              self.esquivador.info, hud)
            except Exception as e:
                self.log(f"[dibujo] {e}")

            ahora = time.perf_counter()
            dt = ahora - t_prev
            t_prev = ahora
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 / dt

            with self._lock:
                self.frame_crudo = frame
                self.hsv = hsv
                self.frame_anotado = anotado
                self.masks = masks
                self.perfil = perfil
                self.decision = d

    def _hud(self, yaw: Optional[float]) -> Dict[str, Any]:
        e = self.enlace
        if self.simulado:
            enlace_txt = "ESP32 SIMULADO"
            enlace_ok = True
        elif e.conectado:
            s = e.sensores
            enlace_txt = (f"ESP32 {e.puerto} {e.latencia_ms:.0f}ms "
                          f"mpu={'si' if s.mpu_ok else 'NO'} "
                          f"tcs={'si' if s.tcs_ok else 'NO'}")
            if e.telemetria.failsafe:
                enlace_txt += " FAILSAFE"
            enlace_ok = True
        else:
            enlace_txt = "ESP32 desconectado"
            enlace_ok = False
        return {
            "armado_txt": "ARMADO" if self.armado else "desarmado",
            "modo": self.modo,
            "fps": self.fps,
            "enlace_txt": enlace_txt,
            "enlace_ok": enlace_ok,
            "yaw": yaw,
            "tcs_clase": e.estado()["sensores"]["clase"] if e.conectado else "-",
        }

    # -- lectura para la web ----------------------------------------------
    def instantanea(self, vista: str = "normal",
                    color: str = "negro") -> Optional[np.ndarray]:
        with self._lock:
            if vista == "cruda":
                return None if self.frame_crudo is None else self.frame_crudo.copy()
            if vista == "mascara":
                m = self.masks.get(color)
                if m is None and self.hsv is not None:
                    det = self.vision.detectores.get(color)
                    if det is not None:
                        m = det.construir_mascara(self.hsv)
                return None if m is None else cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            if vista == "piso":
                return dibujo.vista_piso(self.masks, self.perfil)
            return None if self.frame_anotado is None else self.frame_anotado.copy()

    def estado(self) -> Dict[str, Any]:
        d = self.decision
        p = self.perfil
        return {
            "armado": self.armado,
            "modo": self.modo,
            "fps": round(self.fps, 1),
            "simulado": self.simulado,
            "decision": {"vel": d.vel, "dir": d.direccion, "estado": d.estado,
                         "motivo": d.motivo, "metricas": d.metricas},
            "carrera": self.carrera.estado_dict(),
            "perfil": None if p is None else {
                "pasillo_mm": round(p.pasillo_mm),
                "izq": round(p.izq, 2), "der": round(p.der, 2),
                "cob_izq": round(p.cobertura_izq, 2),
                "cob_der": round(p.cobertura_der, 2),
                "segmentos": len(p.segmentos),
                "interna_mm": None if p.interna_mm is None else round(p.interna_mm),
                "externa_mm": None if p.externa_mm is None else round(p.externa_mm),
                "frontal_mm": None if p.frontal_mm is None else round(p.frontal_mm),
                "desvio_recta": None if p.error_rumbo is None else round(p.error_rumbo, 1),
                "esquinas_vistas": [
                    {"x": round(e.x), "y": round(e.y), "tipo": e.tipo}
                    for e in p.esquinas[:4]],
            },
            "obstaculo": self.esquivador.info,
            "enlace": self.enlace.estado(),
            "geometria": self.geo.estado(),
            "camara_error": self.error_camara,
            "perfil_color": self.perfil_color.get("nombre", ""),
            "perfil_params": self.datos_params.get("activo", ""),
        }
