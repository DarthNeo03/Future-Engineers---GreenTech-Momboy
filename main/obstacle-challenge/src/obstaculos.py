"""
obstaculos.py — Esquivar los pilares por lo que se VE, como lo hace un piloto.

REFERENCIA: el piloto de ANTi (WRO 2025 Future Engineers, 35 s de reto de
obstaculos con puntuacion completa, github.com/atakanersoy/WRO2025_FE_ANTi).
No calcula puntos de paso en milimetros, no recorta huecos contra el muro, no
mezcla pesos: mantiene el rumbo con el giroscopio y, cuando ve un pilar, lo
EMPUJA hacia el canto de la imagen por el que tiene que salir. Eso es todo,
y por eso funciona: el error es en pixeles, que es lo unico que la camara
mide bien de un pilar a cualquier distancia.

LA REGLA, Y NO HAY MAS
    pilar ROJO   -> se pasa por SU DERECHA   -> sale por el canto IZQUIERDO
    pilar VERDE  -> se pasa por SU IZQUIERDA -> sale por el canto DERECHO

Y esos lados NO se invierten con el sentido de la ronda: son la derecha y la
izquierda DEL VEHICULO (reglamento, "el lado del carril por el que debe
circular"), y como la camara mira hacia adelante, trabajar en pixeles de la
imagen ES trabajar en el marco del carro. No hay interruptor para invertirlo:
pasar por el lado incorrecto termina la ronda (9.25.5).

TRES ESTADOS
  LIBRE        no hay pilar: la navegacion centra y el giroscopio manda.
  SIGUIENDO    se ve un pilar dentro del alcance. Se mide cuantos pixeles le
               faltan al BORDE INTERIOR del pilar (el que da al centro de la
               imagen) para llegar a la columna objetivo, pegada al canto, y
               se pide direccion proporcional HACIA EL LADO DE PASO. Es
               ASIMETRICO a proposito: el pilar solo puede pedir alejarse de
               el; si ya esta mas afuera que la columna objetivo, no pide
               nada, y volver a la recta es trabajo del rumbo. Un servo
               simetrico "mantendria" el pilar en el canto girando HACIA el,
               y eso es un rumbo de colision con angulo fijo.
  ADELANTANDO  el pilar se perdio de CERCA: salio por el canto o por abajo
               (la camara no llega tan abajo). Se CONGELA el rumbo en el que
               quedo el carro, sin centrado, el tiempo que tarda el carro
               ENTERO en pasarlo (distancia + largo del carro, a la velocidad
               real). Es lo que impide que la cola lo barra: con direccion
               Ackermann la rueda trasera corta por dentro si se vuelve hacia
               la recta antes de tiempo. Se suelta antes si aparece otro pilar
               cerca (lo visible manda) o si el pasillo se cierra (el muro
               manda).

QUE SE DEJO FUERA A PROPOSITO, Y POR QUE
  * El punto de paso en mm y su recorte al hueco. Fue la fuente de los tres
    fallos de pista (volantazo, cola que barre, pilar que manda sobre el
    muro). Aqui el pilar no compite con el muro: los pilares ya no aparecen
    como muro en el perfil (muro._mascara_piso), asi que el muro solo frena y
    escapa ante muros de verdad.
  * La busqueda del pilar perdido de lejos (ANTi: lost_color). No hace falta:
    si salio por el canto estando lejos es que el carro giro de sobra; el
    rumbo lo vuelve a traer a la vista y el servo lo vuelve a empujar.
  * El filtro de pilares por detras de la linea del piso. El usuario no lo
    quiere: si en pista el carro se pega a la esquina interna por hacer caso
    a un pilar del tramo siguiente, ESO es lo que hay que volver a poner.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria

# Lado por el que hay que pasar CADA color, visto desde el carro.
# +1 = el carro pasa por la DERECHA del pilar (el pilar sale por el canto
# izquierdo de la imagen); -1 al reves.
LADO = {"rojo": +1, "verde": -1}

LIBRE = "libre"
SIGUIENDO = "siguiendo"
ADELANTANDO = "adelantando"


@dataclass
class Maniobra:
    """Lo que el esquivador le pide a la navegacion en este frame."""
    direccion: float = 0.0            # % con signo (+ derecha) que pide el pilar
    peso: float = 0.0                 # 0..1: cuanto cede el centrado del muro
    yaw_factor: float = 1.0           # cuanta correccion de rumbo sobrevive
    rumbo_fijo: Optional[float] = None   # yaw a mantener mientras se adelanta
    vel_pct: Optional[float] = None      # tope de velocidad mientras hay pilar
    estado: str = LIBRE
    color: str = ""


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


class Esquivador:
    def __init__(self, cfg: Dict[str, Any], cfg_geo: Dict[str, Any],
                 cfg_vel: Dict[str, Any],
                 cfg_lim: Optional[Dict[str, Any]] = None):
        self.cfg = cfg
        self.geo_cfg = cfg_geo
        self.vel_cfg = cfg_vel
        self.lim_cfg = cfg_lim or {}
        self.info: Dict[str, Any] = {}     # telemetria y dibujo
        self.reiniciar()

    def reiniciar(self) -> None:
        self.estado = LIBRE
        self.info = {}
        self._color = ""              # pilar que se sigue o se adelanta
        self._color_prev = ""         # el ultimo que mando (para telemetria)
        self._motivo_fin = ""
        self._dir_prev = 0.0
        self._t_prev = 0.0
        self._dist_ult = 0.0          # ultima distancia vista del pilar seguido
        self._fin_compromiso = 0.0
        self._rumbo_fijo: Optional[float] = None

    # ------------------------------------------------------------------
    @property
    def lado_en_juego(self) -> int:
        """+1 / -1 si hay un pilar mandando ahora mismo (visto o en
        compromiso), 0 si no. La navegacion lo usa para soltar el giro de
        esquina y para elegir el lado del rescate."""
        if self.estado == LIBRE:
            return 0
        return LADO.get(self._color, 0)

    @property
    def en_juego(self) -> bool:
        return self.lado_en_juego != 0

    # ------------------------------------------------------------------
    def paso(self, dets: Dict[str, List[vision.Deteccion]], geo: Geometria,
             yaw: Optional[float] = None, vel_pct: float = 0.0,
             pasillo_mm: Optional[float] = None,
             ahora: Optional[float] = None) -> Maniobra:
        """Una vez por frame.

        yaw:        rumbo actual del giroscopio (None si no hay); se congela
                    al perder el pilar de cerca.
        vel_pct:    velocidad que lleva el carro, en % de vmax, para estimar
                    cuanto tarda en adelantar.
        pasillo_mm: lo que ve el perfil del muro delante; si se cierra durante
                    el compromiso, el muro manda y el compromiso se suelta.
        """
        ahora = time.time() if ahora is None else ahora
        cfg = self.cfg
        self.info = {}
        if not bool(cfg.get("activo", True)):
            self.reiniciar()
            return Maniobra()

        activar = float(cfg.get("activar_desde_mm", 1500.0))
        mandar = float(cfg.get("mandar_desde_mm", 800.0))
        morro = float(geo.cfg.get("morro_mm", 60.0))

        elegido = self._elegir(dets, geo, activar, morro)

        # --- adelantando: rumbo congelado hasta que la cola pase ----------
        if self.estado == ADELANTANDO:
            interrumpe = elegido is not None and elegido[0] <= mandar
            cerrado = (pasillo_mm is not None and
                       pasillo_mm < float(cfg.get("soltar_pasillo_mm", 600.0)))
            if ahora >= self._fin_compromiso:
                self._a_libre("cumplido")
            elif cerrado:
                self._a_libre("pasillo cerrado")
            elif interrumpe:
                self._a_libre("otro pilar cerca")
            else:
                return self._maniobra_compromiso(ahora)

        # --- no se ve ninguno -----------------------------------------------
        if elegido is None:
            if self.estado == SIGUIENDO:
                if self._dist_ult <= float(cfg.get("compromiso_bajo_mm", 450.0)):
                    return self._iniciar_compromiso(yaw, vel_pct, ahora)
                self._a_libre("salio de cuadro lejos")
            if self._motivo_fin:
                self.info = {"estado": LIBRE, "ultimo": self._color_prev,
                             "fin": self._motivo_fin}
            return Maniobra()

        dist, lat, color, d = elegido
        return self._seguir(dist, lat, color, d, geo, activar, mandar, ahora)

    # ------------------------------------------------------------------
    def _elegir(self, dets: Dict[str, List[vision.Deteccion]], geo: Geometria,
                activar: float, morro: float
                ) -> Optional[Tuple[float, float, str, vision.Deteccion]]:
        """El pilar MAS CERCANO dentro del alcance, con ventaja para el que ya
        se venia siguiendo (no cambiar de pilar por ruido)."""
        lateral_max = float(self.cfg.get("lateral_max_mm", 900.0))
        ventaja = float(self.cfg.get("preferir_mm", 150.0))
        mejor = None
        for color in LADO:
            for d in dets.get(color, []):
                dist = float(geo.fila_a_distancia(d.base_y)) - morro
                if dist <= 0 or dist > activar:
                    continue
                lat = float(geo.lateral_mm(d.cx, d.base_y))
                if abs(lat) > lateral_max:
                    continue          # tan afuera que es de otro carril
                clave = dist
                if self.estado == SIGUIENDO and color == self._color:
                    clave -= ventaja
                if mejor is None or clave < mejor[0]:
                    mejor = (clave, dist, lat, color, d)
        return None if mejor is None else mejor[1:]

    # ------------------------------------------------------------------
    def _seguir(self, dist: float, lat: float, color: str,
                d: vision.Deteccion, geo: Geometria, activar: float,
                mandar: float, ahora: float) -> Maniobra:
        cfg = self.cfg
        lado = LADO[color]
        W = float(max(1, geo.W))
        frac = float(cfg.get("borde_frac", 0.06))

        # El borde INTERIOR del pilar (el que da al centro de la imagen) tiene
        # que llegar a la columna objetivo, pegada al canto de salida.
        if lado > 0:                      # rojo: al canto izquierdo
            x_obj = W * frac
            x_int = float(d.x + d.w)
            falta = (x_int - x_obj) / W
        else:                             # verde: al canto derecho
            x_obj = W * (1.0 - frac)
            x_int = float(d.x)
            falta = (x_obj - x_int) / W
        falta = max(0.0, falta)           # asimetrico: nunca hacia el pilar

        direccion = lado * float(cfg.get("k_borde", 300.0)) * falta
        empuje = False
        if dist <= float(cfg.get("empuje_bajo_mm", 450.0)):
            # Ya encima y todavia en cuadro: el canto de la imagen no da
            # holgura para un carro de 20 cm, asi que se añade un empujon
            # hacia el lado de paso aunque el borde ya este en el objetivo.
            direccion += lado * float(cfg.get("empuje_pct", 20.0))
            empuje = True
        tope = float(cfg.get("dir_max_pct", 75.0))
        direccion = self._rampa(_lim(direccion, -tope, tope), ahora)

        t = (activar - dist) / max(1.0, activar - mandar)
        peso_lejos = float(cfg.get("peso_lejos", 0.35))
        peso = peso_lejos + (1.0 - peso_lejos) * _lim(t, 0.0, 1.0)

        self.estado = SIGUIENDO
        self._color = color
        self._dist_ult = dist
        self._motivo_fin = ""
        self.info = {"estado": SIGUIENDO, "color": color, "lado": lado,
                     "dist_mm": round(dist), "lat_mm": round(lat),
                     "borde_px": int(round(x_int)), "obj_px": int(round(x_obj)),
                     "borde_y": int(d.base_y), "falta_pct": round(100.0 * falta),
                     "dir": round(direccion), "peso": round(peso, 2),
                     "empuje": empuje}
        return Maniobra(direccion=direccion, peso=peso,
                        yaw_factor=float(cfg.get("yaw_al_esquivar", 0.3)),
                        vel_pct=float(cfg.get("vel_pct", 45)),
                        estado=SIGUIENDO, color=color)

    # ------------------------------------------------------------------
    def _iniciar_compromiso(self, yaw: Optional[float], vel_pct: float,
                            ahora: float) -> Maniobra:
        largo = float(self.geo_cfg.get("largo_carro_mm", 250.0))
        seg = (self._dist_ult + largo) / self._vel_mm_s(vel_pct)
        seg = min(seg, float(self.cfg.get("compromiso_max_ms", 3000)) / 1000.0)
        self._fin_compromiso = ahora + seg
        self._rumbo_fijo = yaw          # None si no hay giroscopio: recto
        self.estado = ADELANTANDO
        self._dir_prev = 0.0
        return self._maniobra_compromiso(ahora)

    def _maniobra_compromiso(self, ahora: float) -> Maniobra:
        queda = max(0.0, self._fin_compromiso - ahora)
        self.info = {"estado": ADELANTANDO, "color": self._color,
                     "lado": LADO.get(self._color, 0), "ultimo": self._color,
                     "adelantando_s": round(queda, 2),
                     "rumbo_fijo": None if self._rumbo_fijo is None
                     else round(self._rumbo_fijo, 1)}
        return Maniobra(direccion=0.0, peso=1.0, yaw_factor=1.0,
                        rumbo_fijo=self._rumbo_fijo,
                        vel_pct=float(self.cfg.get("vel_pct", 45)),
                        estado=ADELANTANDO, color=self._color)

    def _vel_mm_s(self, vel_pct: float) -> float:
        """Velocidad real estimada. OJO: vel_pct es % de vmax, y vmax es un
        tope de PWM sobre 255; sin ese factor el compromiso duraba la mitad
        de lo necesario."""
        v_max = float(self.vel_cfg.get("vel_max_mm_s", 850.0))
        vmax = float(self.lim_cfg.get("vmax", 255)) / 255.0
        v = v_max * vmax * max(10.0, float(vel_pct)) / 100.0
        return max(120.0, v)

    def _a_libre(self, motivo: str) -> None:
        self._color_prev = self._color
        self._motivo_fin = motivo
        self.estado = LIBRE
        self._color = ""
        self._dir_prev = 0.0
        self._rumbo_fijo = None

    # ------------------------------------------------------------------
    def _rampa(self, objetivo: float, ahora: float) -> float:
        """Tope de cuanto puede moverse la direccion por segundo: el
        volantazo de un solo frame es lo que cruza el carro."""
        rampa = float(self.cfg.get("rampa_dir_pct_s", 400.0))
        dt = 0.0 if self._t_prev == 0.0 else max(0.0, ahora - self._t_prev)
        self._t_prev = ahora
        if rampa > 0 and dt > 0:
            paso = rampa * dt
            objetivo = _lim(objetivo, self._dir_prev - paso,
                            self._dir_prev + paso)
        self._dir_prev = objetivo
        return objetivo
