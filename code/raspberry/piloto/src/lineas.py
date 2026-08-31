"""
lineas.py — Las lineas del piso: sentido de la ronda, esquinas y vueltas.

El tapete 2026 tiene en cada esquina dos lineas diagonales que cruzan todo el
ancho de la pista: una NARANJA y una AZUL. En sentido HORARIO se cruza primero
la naranja; en ANTIHORARIO primero la azul. (Configurable por si un tapete
viniera impreso al reves: lineas.naranja_es_horario.)

Fuentes, de mas a menos fiable:
  1. TCS34725 bajo el carro (via ESP32): cruce por contacto, casi infalible.
     Llega como contadores que solo avanzan; aqui se convierten en eventos.
  2. Camara: ve las lineas ANTES de cruzarlas (sirve para saber el sentido
     temprano) y confirma el cruce cuando la linea llega al morro.
  3. Giros de 90 grados completados por la navegacion (respaldo: una esquina
     sin lineas vistas igual cuenta).

Reglas de conteo (lecciones del programa viejo):
  * Las DOS lineas de una esquina llegan en una ventana corta: todo lo que
    caiga dentro de ventana_par_ms es LA MISMA esquina.
  * Despues de contar una esquina hay un refractario de verdad
    (refractario_esquina_ms) durante el cual no se admite otra, venga del
    sensor que venga.

ZONA: DENTRO DE LA ESQUINA O EN RECTA
Ademas de contar, este modulo dice DONDE esta el carro, y eso resuelve el
bucle de las esquinas. El problema: cuando el muro interno se acaba, queda un
hueco de piso blanco enorme que la navegacion por espacio libre lee como
"camino". El carro se mete, desde el sitio nuevo ve otro hueco, se vuelve a
meter, y da vueltas dentro de la curva sin salir.

Las lineas del piso marcan FISICAMENTE donde esta la curva:

    ... recta ...  |naranja|   <-- ENTRA en la esquina
                    ( curva )   <-- aqui NO se decide por espacio libre:
                                    se ejecuta un giro de 90 comprometido
                   |azul|      <-- SALE de la esquina
    ... recta ...

Entrar es fiable (la primera linea del par); salir se confirma con el giro de
90 completado, y hay un timeout de seguridad por si el giroscopio falla.
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from . import vision
from .geometria import Geometria
from .muro import PerfilMuro

HORARIO = 1
ANTIHORARIO = -1
DESCONOCIDO = 0

ZONA_RECTA = "recta"
ZONA_ESQUINA = "esquina"


class GestorLineas:
    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self.sentido = DESCONOCIDO
        self.sugerencia = DESCONOCIDO       # lo que la camara cree ver delante
        self.esquinas = 0
        self.orden_observado: List[str] = []   # colores 1a y 2a de la ultima esquina
        self._t_esquina = 0.0               # cuando se conto la ultima esquina
        self._t_evento = 0.0                # ultimo evento de linea (del par)
        self._colores_esquina: List[str] = []
        self._t_cam = {"naranja": 0.0, "azul": 0.0}   # refractario camara
        self.ultimo_evento = ""             # para telemetria
        self.dist_lineas: Dict[str, float] = {}       # distancia visible (mm)
        self.zona = ZONA_RECTA
        self._t_zona = 0.0
        self.motivo_zona = ""

    # -- entrada 1: eventos del TCS (ya convertidos por el enlace) ---------
    def evento_tcs(self, color: str) -> None:
        if bool(self.cfg.get("usar_tcs", True)):
            self._evento(color, "tcs")

    # -- entrada 2: la camara --------------------------------------------
    def paso_camara(self, dets: Dict[str, List[vision.Deteccion]],
                    perfil: Optional[PerfilMuro], geo: Geometria) -> None:
        """Mira las detecciones naranja/azul que estan SOBRE EL PISO (por
        debajo de la linea de contacto del muro) y dentro del alcance."""
        if not bool(self.cfg.get("usar_camara", True)):
            return
        self.dist_lineas = {}
        morro = float(geo.cfg.get("morro_mm", 60.0))
        for color in ("naranja", "azul"):
            mejor: Optional[float] = None
            for d in dets.get(color, []):
                # sobre el piso: su base debe quedar por debajo del contacto
                # del muro en esa columna (si hay perfil)
                if perfil is not None:
                    c = min(max(int(d.cx), 0), perfil.ancho - 1)
                    if perfil.valido[c] and d.base_y < perfil.y_contacto[c] - 4:
                        continue
                dist = float(geo.fila_a_distancia(d.base_y)) - morro
                if dist < 0:
                    dist = 0.0
                if mejor is None or dist < mejor:
                    mejor = dist
            if mejor is not None:
                self.dist_lineas[color] = mejor

        # sugerencia de sentido: si se ven las dos lineas, la mas cercana es
        # la que se cruzara primero
        if len(self.dist_lineas) == 2:
            dn = self.dist_lineas["naranja"]
            da = self.dist_lineas["azul"]
            if abs(dn - da) > 80.0:
                primera = "naranja" if dn < da else "azul"
                self.sugerencia = self._sentido_de(primera)

        # cruce por camara: la linea llego al morro
        umbral = float(self.cfg.get("umbral_cruce_mm", 260.0))
        ahora = time.time()
        for color, dist in self.dist_lineas.items():
            if dist <= umbral and ahora - self._t_cam[color] > 1.2:
                self._t_cam[color] = ahora
                self._evento(color, "cam")

    # -- entrada 3: giro de 90 completado (respaldo) ----------------------
    def giro_completado(self, lado: int) -> None:
        """lado: +1 giro a la derecha, -1 a la izquierda.

        Ademas de contar (si no lo hicieron ya las lineas), esto es lo que
        SACA al carro de la zona de esquina: el giro de 90 esta hecho, la
        curva quedo atras."""
        ahora = time.time()
        if self.sentido == DESCONOCIDO and lado != 0:
            # girar a la derecha en las esquinas = sentido horario
            self.sentido = HORARIO if lado > 0 else ANTIHORARIO
        self.salir_de_esquina("giro de 90 completado")
        # si la esquina ya se conto por lineas hace poco, no contar doble
        if (ahora - self._t_esquina) * 1000 < float(
                self.cfg.get("refractario_esquina_ms", 3000)):
            return
        self.esquinas += 1
        self._t_esquina = ahora
        self._colores_esquina = []
        self.ultimo_evento = "esquina por giro"

    # -- zona: dentro de la curva o en recta -------------------------------
    @property
    def en_esquina(self) -> bool:
        return self.zona == ZONA_ESQUINA

    def entrar_en_esquina(self, motivo: str) -> None:
        if self.zona != ZONA_ESQUINA:
            self.zona = ZONA_ESQUINA
            self._t_zona = time.time()
            self.motivo_zona = motivo

    def salir_de_esquina(self, motivo: str = "") -> None:
        if self.zona != ZONA_RECTA:
            self.zona = ZONA_RECTA
            self._t_zona = time.time()
            self.motivo_zona = motivo

    def paso_zona(self) -> None:
        """Red de seguridad: nadie puede quedarse eternamente 'en la esquina'.
        Si el giro no se confirmo (giroscopio caido, patinazo), a los
        esquina_max_ms se vuelve a recta y la navegacion normal retoma."""
        if self.zona == ZONA_ESQUINA:
            limite = float(self.cfg.get("esquina_max_ms", 6000)) / 1000.0
            if time.time() - self._t_zona > limite:
                self.salir_de_esquina("timeout de esquina")

    def tiempo_en_zona(self) -> float:
        return time.time() - self._t_zona if self._t_zona else 0.0

    # -- interno ----------------------------------------------------------
    def _sentido_de(self, primer_color: str) -> int:
        naranja_horario = bool(self.cfg.get("naranja_es_horario", True))
        if primer_color == "naranja":
            return HORARIO if naranja_horario else ANTIHORARIO
        return ANTIHORARIO if naranja_horario else HORARIO

    def _evento(self, color: str, fuente: str) -> None:
        ahora = time.time()
        self.ultimo_evento = f"{color} ({fuente})"
        ventana = float(self.cfg.get("ventana_par_ms", 2500)) / 1000.0
        refract = float(self.cfg.get("refractario_esquina_ms", 3000)) / 1000.0

        if self._colores_esquina and ahora - self._t_evento <= ventana:
            # segunda linea de la MISMA esquina: sigue DENTRO de la curva
            self._t_evento = ahora
            if color not in self._colores_esquina:
                self._colores_esquina.append(color)
                self.orden_observado = list(self._colores_esquina)
            return

        if ahora - self._t_esquina < refract:
            # zona muerta: probablemente rebote de la misma esquina
            self._t_evento = ahora
            return

        # primera linea de una esquina nueva: el carro ENTRA en la curva
        self.esquinas += 1
        self._t_esquina = ahora
        self._t_evento = ahora
        self._colores_esquina = [color]
        self.orden_observado = [color]
        if self.sentido == DESCONOCIDO:
            self.sentido = self._sentido_de(color)
        self.entrar_en_esquina(f"linea {color} ({fuente})")

    # -- salida ------------------------------------------------------------
    def vueltas(self, esquinas_por_vuelta: int = 4) -> int:
        return self.esquinas // max(1, esquinas_por_vuelta)

    def sentido_efectivo(self, forzado: str = "auto") -> int:
        """Lo que la navegacion debe usar: forzado desde la web > detectado >
        sugerido por la camara > desconocido."""
        if forzado == "horario":
            return HORARIO
        if forzado == "antihorario":
            return ANTIHORARIO
        if self.sentido != DESCONOCIDO:
            return self.sentido
        return self.sugerencia

    def estado(self) -> Dict[str, Any]:
        nombres = {HORARIO: "horario", ANTIHORARIO: "antihorario",
                   DESCONOCIDO: "?"}
        return {
            "sentido": nombres[self.sentido],
            "sugerencia": nombres[self.sugerencia],
            "esquinas": self.esquinas,
            "orden": "+".join(self.orden_observado),
            "ultimo": self.ultimo_evento,
            "dist_lineas": {k: round(v, 0) for k, v in self.dist_lineas.items()},
            "zona": self.zona,
            "zona_motivo": self.motivo_zona,
            "zona_s": round(self.tiempo_en_zona(), 1),
        }
