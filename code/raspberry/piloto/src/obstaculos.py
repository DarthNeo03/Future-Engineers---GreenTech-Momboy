"""
obstaculos.py — Esquive basico de las señales de transito (version inicial).

Regla del juego: el pilar ROJO se pasa por su DERECHA; el VERDE por su
IZQUIERDA. Matiz importante: "pasar por la derecha del pilar" NO es "girar a
la derecha" — si el pilar esta a la izquierda del carro, el punto de paso
puede quedar a la izquierda del centro de la imagen.

Como funciona:
  1. Se toma el pilar MAS CERCANO (rojo o verde) dentro del alcance.
  2. Su posicion se proyecta a mm sobre el suelo (geometria calibrada).
  3. El punto de paso = costado correcto del pilar + medio carro + margen.
  4. El punto se recorta al espacio libre del perfil (esquivar un pilar
     pegado a la pared no puede mandar el carro contra la pared).
  5. Se devuelve (direccion_deseada_pct, peso). El peso sube de 0 a peso_max
     entre activar_desde_mm y mandar_desde_mm; la navegacion mezcla.

LO QUE HAY MAS ALLA DE LA LINEA DEL PISO NO ES DE ESTA RECTA
Las lineas naranja y azul marcan el limite de la seccion. Un pilar que se ve
por detras de ellas esta en el tramo SIGUIENTE: todavia no es asunto nuestro.
Si se le hace caso desde la recta, el esquive tira del carro justo cuando hay
que prepararse para la curva; el carro se pega a la esquina interna, no le
queda sitio para abrirse y engancha el canto al girar.

Por eso se descartan los pilares mas lejanos que la linea mientras se viene
por la recta. En cuanto se cruza (el carro entra en la zona de esquina), el
filtro se levanta y esos pilares pasan a contar, que es cuando de verdad hay
que esquivarlos.

Version basica a proposito: suficiente para probar el reto con obstaculos.
Cuando el Open Challenge este solido se mejora (siguiente pilar, arcos).
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro


class Esquivador:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.info: Dict[str, Any] = {}     # para telemetria y dibujo

    def paso(self, dets: Dict[str, List[vision.Deteccion]],
             perfil: Optional[PerfilMuro], geo: Geometria,
             dist_lineas: Optional[Dict[str, float]] = None,
             en_esquina: bool = False) -> Tuple[float, float]:
        """Devuelve (direccion_deseada_pct, peso 0..1).

        dist_lineas: distancia a las lineas del piso que se ven delante
                     (de GestorLineas). Marca el final de esta seccion.
        en_esquina:  si el carro ya entro en la curva, el limite se levanta:
                     los pilares del tramo nuevo pasan a ser asunto suyo.
        """
        self.info = {}
        if not bool(self.cfg.get("activo", False)):
            return 0.0, 0.0

        activar = float(self.cfg.get("activar_desde_mm", 1600.0))
        mandar = float(self.cfg.get("mandar_desde_mm", 700.0))
        morro = float(geo.cfg.get("morro_mm", 60.0))

        # --- hasta donde llega ESTA seccion --------------------------------
        limite = None
        if (bool(self.cfg.get("limitar_por_lineas", True)) and not en_esquina
                and dist_lineas):
            cerca = min(dist_lineas.values())
            limite = cerca + float(self.cfg.get("margen_linea_mm", 60.0))
            self.info["limite_mm"] = round(limite)

        # --- pilar mas cercano dentro del alcance --------------------------
        descartados = 0
        mejor: Optional[Tuple[float, float, str]] = None   # (dist, lat, color)
        for color in ("rojo", "verde"):
            for d in dets.get(color, []):
                dist = float(geo.fila_a_distancia(d.base_y)) - morro
                if dist <= 0 or dist > activar:
                    continue
                if limite is not None and dist > limite:
                    descartados += 1     # esta detras de la linea: otra seccion
                    continue
                lat = float(geo.lateral_mm(d.cx, d.base_y))
                if abs(lat) > 900:                 # muy afuera: otro carril
                    continue
                if mejor is None or dist < mejor[0]:
                    mejor = (dist, lat, color)
        if descartados:
            self.info["tras_linea"] = descartados
        if mejor is None:
            return 0.0, 0.0
        dist, lat, color = mejor

        # --- punto de paso -------------------------------------------------
        semi_carro = float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0
        despeje = (semi_carro + float(self.cfg.get("margen_mm", 70.0)) +
                   float(self.cfg.get("semi_pilar_mm", 25.0)))
        if color == "rojo":
            objetivo = lat + despeje       # pasar por su derecha
        else:
            objetivo = lat - despeje       # pasar por su izquierda

        # --- recortar al pasillo libre a esa distancia ---------------------
        if perfil is not None and perfil.hay_muro:
            objetivo = self._recortar(objetivo, dist, perfil, geo, semi_carro)

        # --- a direccion ---------------------------------------------------
        ang = math.degrees(math.atan2(objetivo, max(150.0, dist)))
        direccion = ang * float(self.cfg.get("k_dir", 1.4))

        t = (activar - dist) / max(1.0, activar - mandar)
        peso = float(self.cfg.get("peso_max", 0.8)) * min(1.0, max(0.0, t))

        self.info.update({"color": color, "dist_mm": round(dist),
                          "lat_mm": round(lat), "objetivo_mm": round(objetivo),
                          "peso": round(peso, 2)})
        return direccion, peso

    def _recortar(self, objetivo: float, dist: float, p: PerfilMuro,
                  geo: Geometria, semi_carro: float) -> float:
        """Busca cuanto muro hay a la distancia del pilar y no manda el carro
        mas alla de donde el perfil dice que hay pared."""
        libre_min = dist + 250.0     # la pared debe quedar mas lejos que el pilar
        margen = semi_carro + 40.0
        # limites laterales del hueco a esa profundidad
        izq_lim, der_lim = -1500.0, 1500.0
        for c in range(0, p.ancho, 6):
            if not p.valido[c] or p.dist_mm[c] > libre_min:
                continue
            lat_c = float(geo.lateral_mm(c, max(1, int(p.y_contacto[c]))))
            if lat_c < 0:
                izq_lim = max(izq_lim, lat_c)
            else:
                der_lim = min(der_lim, lat_c)
        return max(izq_lim + margen, min(der_lim - margen, objetivo))
