"""
carrera.py — El director de la ronda del Open Challenge 2026.

Reglamento 2026 que importa aqui:
  * 3 vueltas, TODAS en la misma direccion (ya no hay media vuelta).
  * La direccion es aleatoria por ronda y esta PROHIBIDO meterle datos al
    robot antes del start: el carro la deduce solo (lineas del piso).
  * Tras las 3 vueltas hay que DETENERSE con la proyeccion completa del carro
    dentro de la seccion de meta (la recta donde arranco) para el bono.
  * Maximo 3 minutos.

Como para: la meta es la recta que viene despues de la ultima esquina. Cuando
se completa la esquina numero (vueltas x esquinas_por_vuelta), se avanza
parada_ms mas (para meter el carro ENTERO en la seccion) y se corta a cero.
"""

from __future__ import annotations

import time
from typing import Any, Dict

from .lineas import GestorLineas, HORARIO, ANTIHORARIO

LISTO = "listo"
CORRIENDO = "corriendo"
PARANDO = "parando"
TERMINADO = "terminado"


class Carrera:
    def __init__(self, cfg: Dict[str, Any], lineas: GestorLineas):
        self.cfg = cfg
        self.lineas = lineas
        self.estado = LISTO
        self.t_inicio = 0.0
        self.t_parada = 0.0

    # ------------------------------------------------------------------
    def arrancar(self) -> None:
        self.estado = CORRIENDO
        self.t_inicio = time.time()
        self.lineas.reiniciar()

    def reiniciar(self) -> None:
        self.estado = LISTO
        self.lineas.reiniciar()

    # ------------------------------------------------------------------
    @property
    def esquinas_meta(self) -> int:
        return int(self.cfg.get("vueltas", 3)) * int(
            self.cfg.get("esquinas_por_vuelta", 4))

    def sentido(self) -> int:
        return self.lineas.sentido_efectivo(str(self.cfg.get("sentido", "auto")))

    def transcurrido(self) -> float:
        return time.time() - self.t_inicio if self.t_inicio else 0.0

    # ------------------------------------------------------------------
    def paso(self) -> bool:
        """Llamar una vez por frame. Devuelve True si el carro DEBE PARARSE
        (carrera terminada o fuera de tiempo)."""
        if self.estado == LISTO:
            return False
        ahora = time.time()

        if self.estado == CORRIENDO:
            if not bool(self.cfg.get("autostop", True)):
                return False
            if self.transcurrido() > float(self.cfg.get("tiempo_max_s", 180)):
                self.estado = TERMINADO
                return True
            if self.lineas.esquinas >= self.esquinas_meta:
                # ultima esquina completada: estamos entrando a la seccion de
                # meta; avanzar un poco mas para meter el carro entero
                self.estado = PARANDO
                self.t_parada = ahora + float(self.cfg.get("parada_ms", 1400)) / 1000.0

        if self.estado == PARANDO:
            if ahora >= self.t_parada:
                self.estado = TERMINADO
            return self.estado == TERMINADO

        return self.estado == TERMINADO

    # ------------------------------------------------------------------
    def estado_dict(self) -> Dict[str, Any]:
        epv = max(1, int(self.cfg.get("esquinas_por_vuelta", 4)))
        nombres = {HORARIO: "horario", ANTIHORARIO: "antihorario", 0: "?"}
        return {
            "estado": self.estado,
            "sentido": nombres[self.sentido()],
            "esquinas": self.lineas.esquinas,
            "vueltas": self.lineas.esquinas // epv,
            "meta_esquinas": self.esquinas_meta,
            "transcurrido_s": round(self.transcurrido(), 1),
            "lineas": self.lineas.estado(),
        }
