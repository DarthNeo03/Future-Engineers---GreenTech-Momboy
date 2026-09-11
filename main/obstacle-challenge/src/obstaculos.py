"""
obstaculos.py — Esquivar los pilares de colores. A proposito, lo mas corto
posible.

LA REGLA, Y NO HAY MAS
    pilar ROJO   -> se pasa por SU DERECHA
    pilar VERDE  -> se pasa por SU IZQUIERDA

Y esos lados NO se invierten con el sentido de la ronda. El reglamento habla
de la derecha y la izquierda DEL VEHICULO ("el lado del carril por el que debe
circular"), asi que trabajando en el marco del carro -que es lo que se hace
aqui, porque la camara mira hacia adelante- sale solo en los dos sentidos. No
hay interruptor para invertirlo: pasar por el lado incorrecto termina la ronda
(9.25.5) y un interruptor asi solo puede estar mal puesto.

COMO
  1. Se toma el pilar MAS CERCANO (rojo o verde) dentro del alcance.
  2. Su base se proyecta a milimetros sobre el suelo (geometria calibrada).
  3. El punto de paso = su costado correcto + medio pilar + medio carro + una
     holgura.
  4. Ese punto se aprieta contra el hueco libre que ve el perfil del muro,
     PERO NUNCA SE CRUZA AL OTRO LADO DEL PILAR. Esto costo una ronda: al
     ajustar el punto al hueco, un pilar pegado a la pared podia acabar con el
     punto de paso del lado prohibido. Ahora, si no cabe, se aprieta hasta el
     minimo fisico y se avisa (sin_sitio); cruzar, jamas.
  5. Se devuelve (direccion_pct, peso 0..1) y la navegacion lo mezcla.

LOS TRES FALLOS QUE YA SE PAGARON EN PISTA, Y SU CURA
  a. EL VOLANTAZO. El angulo al punto de paso se calculaba con la distancia al
     pilar: al acercarse, el MISMO desvio lateral pedia cada vez mas angulo,
     asi que el volante acababa al tope y el carro se cruzaba. Cura: la
     distancia que se usa para el angulo tiene un suelo (mirada_min_mm) y la
     direccion que puede pedir el esquive tiene un techo (dir_max_pct).
  b. LA COLA SE LLEVABA EL PILAR. Al quedar muy cerca, el pilar sale del
     cuadro (la camara no llega tan abajo), el esquive desaparece y el
     centrado tira del carro hacia el medio; con direccion Ackermann la rueda
     trasera corta por dentro y barre el pilar. Cura: COMPROMISO de
     adelantamiento, ir recto hasta que la cola haya pasado, calculado con el
     largo del carro y su velocidad real.
  c. EL PILAR MANDABA MAS QUE EL MURO. Un pilar pegado al muro interior
     llevaba el carro de frente contra la esquina. Cura: ceder_ante_muro
     desvanece el peso del pilar segun se cierra el pasillo.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro

# Lado por el que hay que pasar CADA color, visto desde el carro.
# +1 = el punto de paso queda a la derecha del pilar (el pilar se queda a la
# izquierda del carro); -1 al reves.
LADO = {"rojo": +1, "verde": -1}


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


class Esquivador:
    def __init__(self, cfg: Dict[str, Any], cfg_geo: Dict[str, Any],
                 cfg_vel: Dict[str, Any]):
        self.cfg = cfg
        self.geo_cfg = cfg_geo
        self.vel_cfg = cfg_vel
        self.info: Dict[str, Any] = {}     # telemetria y dibujo
        self._dir_prev = 0.0
        self._t_prev = 0.0
        self._fin_compromiso = 0.0
        self._dir_compromiso = 0.0
        self._color_compromiso = ""

    def reiniciar(self) -> None:
        self.info = {}
        self._dir_prev = 0.0
        self._fin_compromiso = 0.0
        self._color_compromiso = ""

    # ------------------------------------------------------------------
    @property
    def lado_en_juego(self) -> int:
        """+1 / -1 si hay un pilar mandando ahora mismo (visto o en
        compromiso), 0 si no. La navegacion lo usa para girar hacia el lado
        que toca en vez de abandonar la maniobra."""
        if self.info.get("color") in LADO:
            return LADO[self.info["color"]]
        if self._color_compromiso in LADO:
            return LADO[self._color_compromiso]
        return 0

    @property
    def en_juego(self) -> bool:
        return self.lado_en_juego != 0

    # ------------------------------------------------------------------
    def paso(self, dets: Dict[str, List[vision.Deteccion]],
             perfil: Optional[PerfilMuro], geo: Geometria,
             vel_pct: float = 0.0,
             ahora: Optional[float] = None) -> Tuple[float, float]:
        """Devuelve (direccion_pct, peso 0..1). vel_pct es la velocidad que
        lleva el carro, en % de vmax, para estimar cuanto tarda en adelantar."""
        ahora = time.time() if ahora is None else ahora
        cfg = self.cfg
        self.info = {}
        if not bool(cfg.get("activo", True)):
            self._color_compromiso = ""
            return 0.0, 0.0

        activar = float(cfg.get("activar_desde_mm", 1600.0))
        mandar = float(cfg.get("mandar_desde_mm", 700.0))
        morro = float(geo.cfg.get("morro_mm", 60.0))

        # --- pilar mas cercano dentro del alcance --------------------------
        mejor: Optional[Tuple[float, float, str]] = None   # (dist, lat, color)
        for color in LADO:
            for d in dets.get(color, []):
                dist = float(geo.fila_a_distancia(d.base_y)) - morro
                if dist <= 0 or dist > activar:
                    continue
                lat = float(geo.lateral_mm(d.cx, d.base_y))
                if abs(lat) > float(cfg.get("lateral_max_mm", 900.0)):
                    continue          # tan afuera que es de otro carril
                if mejor is None or dist < mejor[0]:
                    mejor = (dist, lat, color)

        if mejor is None:
            return self._compromiso(ahora)

        dist, lat, color = mejor
        lado = LADO[color]

        # --- punto de paso -------------------------------------------------
        semi_carro = float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0
        semi_pilar = float(cfg.get("semi_pilar_mm", 25.0))
        minimo = semi_carro + semi_pilar          # rozando: el limite fisico
        deseado = minimo + float(cfg.get("margen_mm", 70.0))
        objetivo = lat + lado * deseado

        sin_sitio = False
        if perfil is not None and perfil.hay_muro:
            apretado = self._apretar(objetivo, dist, perfil, geo, semi_carro)
            # El recorte solo puede ACERCAR el punto al pilar, nunca cruzarlo:
            # el lado de paso es intocable.
            tope = lat + lado * minimo
            if lado > 0:
                apretado = max(apretado, tope)
            else:
                apretado = min(apretado, tope)
            sin_sitio = abs(apretado - objetivo) > 1.0
            objetivo = apretado

        # --- a direccion ---------------------------------------------------
        # La mirada tiene suelo: sin el, el mismo desvio pide cada vez mas
        # angulo segun te acercas y el volante termina al tope (fallo (a)).
        mirada = max(float(cfg.get("mirada_min_mm", 350.0)), dist)
        ang = math.degrees(math.atan2(objetivo, mirada))
        direccion = _lim(ang * float(cfg.get("k_dir", 1.4)),
                         -float(cfg.get("dir_max_pct", 55.0)),
                         float(cfg.get("dir_max_pct", 55.0)))
        direccion = self._suavizar(direccion, ahora)

        # --- peso ----------------------------------------------------------
        t = (activar - dist) / max(1.0, activar - mandar)
        peso = float(cfg.get("peso_max", 0.8)) * _lim(t, 0.0, 1.0)
        if bool(cfg.get("ceder_ante_muro", True)) and perfil is not None:
            # Con el pasillo cerrandose manda el muro: un pilar pegado a la
            # pared interior no puede llevarse el carro contra la esquina.
            holgura = float(cfg.get("ceder_bajo_mm", 700.0))
            if perfil.pasillo_mm < holgura:
                peso *= _lim(perfil.pasillo_mm / max(1.0, holgura), 0.0, 1.0)

        self._armar_compromiso(dist, direccion, color, vel_pct, ahora)
        self.info = {"color": color, "lado": lado, "dist_mm": round(dist),
                     "lat_mm": round(lat), "objetivo_mm": round(objetivo),
                     "peso": round(peso, 2), "dir": round(direccion),
                     "sin_sitio": sin_sitio}
        return direccion, peso

    # ------------------------------------------------------------------
    def _apretar(self, objetivo: float, dist: float, p: PerfilMuro,
                 geo: Geometria, semi_carro: float) -> float:
        """Mete el punto de paso dentro del hueco libre que hay a la
        profundidad del pilar."""
        libre_min = dist + 250.0      # la pared debe quedar mas lejos que el pilar
        margen = semi_carro + 40.0
        izq_lim, der_lim = -1500.0, 1500.0
        for c in range(0, p.ancho, 6):
            if not p.valido[c] or p.dist_mm[c] > libre_min:
                continue
            lat_c = float(geo.lateral_mm(c, max(1, int(p.y_contacto[c]))))
            if lat_c < 0:
                izq_lim = max(izq_lim, lat_c)
            else:
                der_lim = min(der_lim, lat_c)
        lo, hi = izq_lim + margen, der_lim - margen
        if lo > hi:                   # el hueco es mas estrecho que el carro:
            return (izq_lim + der_lim) / 2.0      # apuntar a su centro
        return _lim(objetivo, lo, hi)

    def _suavizar(self, direccion: float, ahora: float) -> float:
        """Tope de cuanto puede moverse la direccion por segundo: el volantazo
        de un solo frame es lo que cruza el carro."""
        rampa = float(self.cfg.get("rampa_dir_pct_s", 220.0))
        dt = 0.0 if self._t_prev == 0.0 else max(0.0, ahora - self._t_prev)
        self._t_prev = ahora
        if rampa > 0 and dt > 0:
            paso = rampa * dt
            direccion = _lim(direccion, self._dir_prev - paso,
                             self._dir_prev + paso)
        self._dir_prev = direccion
        return direccion

    def _armar_compromiso(self, dist: float, direccion: float, color: str,
                          vel_pct: float, ahora: float) -> None:
        """Mientras el pilar se ve de cerca se va anotando cuanto habria que
        seguir recto si desapareciera AHORA. Ver fallo (b)."""
        if dist > float(self.cfg.get("compromiso_bajo_mm", 500.0)):
            return
        largo = float(self.geo_cfg.get("largo_carro_mm", 250.0))
        v_max = float(self.vel_cfg.get("vel_max_mm_s", 850.0))
        v = max(80.0, v_max * max(10.0, vel_pct) / 100.0)   # mm/s, con suelo
        seg = (largo + dist) / v
        tope = float(self.cfg.get("compromiso_max_ms", 2500)) / 1000.0
        self._fin_compromiso = ahora + min(seg, tope)
        self._dir_compromiso = direccion
        self._color_compromiso = color

    def _compromiso(self, ahora: float) -> Tuple[float, float]:
        """No se ve ningun pilar. Si acabamos de adelantar uno, seguir el rumbo
        del adelantamiento hasta que la cola lo haya pasado."""
        if ahora >= self._fin_compromiso:
            self._color_compromiso = ""
            self._dir_prev = 0.0
            return 0.0, 0.0
        queda = self._fin_compromiso - ahora
        self.info = {"color": "", "adelantando_s": round(queda, 2),
                     "ultimo": self._color_compromiso,
                     "dir": round(self._dir_compromiso)}
        return self._dir_compromiso, float(self.cfg.get("peso_compromiso", 0.7))
