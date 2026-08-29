"""
vueltas.py — Contar vueltas fusionando tres medidores.

En la pista hay una linea naranja y una azul en cada esquina. El orden en que
se cruzan dice el sentido: naranja y luego azul = horario (configurable, por si
tu tapete va al reves).

TRES FUENTES, PORQUE UNA SOLA FALLA:
  1. La CAMARA ve las lineas del suelo (colores ya calibrados en colors.json).
     Falla si la luz cambia o si la linea queda fuera del recorte de abajo.
  2. El TCS34725 las ve por contacto, mirando al piso. Falla si el carro pasa
     por el borde de la linea o si el sensor queda alto.
  3. Los GIROS de 90 grados que cuenta la navegacion. No falla casi nunca pero
     no distingue esquina de un esquive brusco.

Se fusionan asi: una esquina se da por buena cuando se completa un PAR de
lineas (naranja + azul en cualquier orden dentro de una ventana de tiempo) o
cuando termina un giro. Lo que llegue segundo dentro de la ventana no vuelve a
contar, asi que ver la misma esquina por los tres caminos suma UNA.

Cuatro esquinas = una vuelta. Al llegar al objetivo se pide la media vuelta y
se empieza a contar otra vez en sentido contrario.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from . import protocolo as P

IDA, VUELTA = 0, 1


@dataclass
class Cruce:
    t: float
    linea: int
    origen: str


class ContadorVueltas:
    def __init__(self, cfg: Dict[str, Any],
                 al_log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg or {}
        self.al_log = al_log or (lambda s: None)
        self.reiniciar()

    # ------------------------------------------------------------------
    def reiniciar(self) -> None:
        self.esquinas = 0
        self.vueltas = 0
        self.tramo = IDA
        self.terminado = False
        self.media_vuelta_pendiente = False
        self.cruces: Deque[Cruce] = deque(maxlen=64)
        self.historial: List[str] = []
        self._t_ultima_esquina = 0.0
        self._t_ultimo_cruce: Dict[int, float] = {}
        self._par_abierto: Optional[Cruce] = None
        self.sentido_lineas = 0          # +1 horario segun el orden de lineas
        self.ultima_esquina_por = ""
        self.ignoradas = 0               # esquinas descartadas por el refractario

    @property
    def objetivo(self) -> int:
        return max(1, int(self.cfg.get("objetivo", 3)))

    @property
    def esquinas_por_vuelta(self) -> int:
        return max(1, int(self.cfg.get("esquinas_por_vuelta", 4)))

    # ------------------------------------------------------------------
    def evento_linea(self, linea: int, origen: str = "camara") -> None:
        """Cruce de una linea del suelo, venga de la camara o del TCS34725."""
        if linea == P.LINEA_NINGUNA or self.terminado:
            return
        ahora = time.time()
        # Dentro del periodo refractario no se apunta nada: son rebotes de la
        # esquina que se acaba de contar.
        refractario = float(self.cfg.get("refractario_ms", 3000)) / 1000.0
        if ahora - self._t_ultima_esquina < refractario:
            return
        deb = float(self.cfg.get("debounce_ms", 900)) / 1000.0
        # el mismo color otra vez enseguida es la misma linea vista dos veces
        if ahora - self._t_ultimo_cruce.get(linea, 0.0) < deb:
            return
        self._t_ultimo_cruce[linea] = ahora
        c = Cruce(ahora, linea, origen)
        self.cruces.append(c)
        self.historial.append(
            f"{time.strftime('%H:%M:%S')} {P.NOMBRE_LINEA[linea]} ({origen})")
        if len(self.historial) > 60:
            del self.historial[:20]

        ventana = float(self.cfg.get("ventana_par_ms", 2500)) / 1000.0
        if self._par_abierto and (ahora - self._par_abierto.t) > ventana:
            self._par_abierto = None          # el par se quedo a medias, caduca
        if self._par_abierto and self._par_abierto.linea != linea:
            self._deducir_sentido(self._par_abierto.linea, linea)
            self._par_abierto = None
            self._registrar_esquina("lineas")
        elif bool(self.cfg.get("una_linea_basta", False)):
            # Modo tolerante: si el tapete solo deja ver una de las dos lineas,
            # una sola ya cuenta como esquina (el refractario evita el triple).
            self._registrar_esquina("linea suelta")
        else:
            self._par_abierto = c

    def evento_giro(self, lado: int = 0) -> None:
        """La navegacion termino un giro de esquina."""
        if self.terminado:
            return
        self._registrar_esquina("giro")

    # ------------------------------------------------------------------
    def _deducir_sentido(self, primera: int, segunda: int) -> None:
        orden = [str(x) for x in self.cfg.get("orden_horario", ["naranja", "azul"])]
        nombres = [P.NOMBRE_LINEA.get(primera, "?"), P.NOMBRE_LINEA.get(segunda, "?")]
        if nombres == orden:
            self.sentido_lineas = 1
        elif nombres == orden[::-1]:
            self.sentido_lineas = -1

    def _registrar_esquina(self, por: str) -> None:
        """Una esquina, una cuenta.

        QUE FALLABA: la ventana de deduplicacion era de 2.2 s y solo tapaba lo
        que llegaba MUY seguido. Al cruzar una esquina real llegan la linea
        naranja, la azul, a veces otra vez la naranja porque el carro entra en
        diagonal y la fraccion de pixeles sube y baja, y ademas el giro. Con
        2.2 s eso son dos o tres esquinas contadas.

        Ahora hay un PERIODO REFRACTARIO de verdad: tras registrar una esquina
        no se admite otra hasta pasados 'refractario_ms' (3 s por defecto), da
        igual de que fuente venga. En una vuelta las esquinas estan separadas
        varios segundos, asi que no se pierde ninguna real.
        """
        ahora = time.time()
        refractario = float(self.cfg.get("refractario_ms",
                                         self.cfg.get("ventana_esquina_ms", 3000)))
        if ahora - self._t_ultima_esquina < refractario / 1000.0:
            self.ignoradas += 1
            return
        self._t_ultima_esquina = ahora
        self._par_abierto = None      # empezamos limpios para la siguiente
        self.esquinas += 1
        self.ultima_esquina_por = por
        if self.esquinas % self.esquinas_por_vuelta == 0:
            self.vueltas += 1
            self.al_log(f"[vueltas] vuelta {self.vueltas}/{self.objetivo} "
                        f"({'ida' if self.tramo == IDA else 'vuelta'})")
            self._revisar_objetivo()
        else:
            self.al_log(f"[vueltas] esquina {self.esquinas % self.esquinas_por_vuelta}"
                        f"/{self.esquinas_por_vuelta} por {por}")

    def _revisar_objetivo(self) -> None:
        if self.vueltas < self.objetivo:
            return
        if self.tramo == IDA and bool(self.cfg.get("hacer_media_vuelta", True)):
            self.media_vuelta_pendiente = True
            self.al_log("[vueltas] objetivo de ida cumplido: media vuelta")
        else:
            self.terminado = True
            self.al_log("[vueltas] recorrido completo")

    def media_vuelta_completada(self) -> None:
        self.media_vuelta_pendiente = False
        self.tramo = VUELTA
        self.esquinas = 0
        self.vueltas = 0
        self.sentido_lineas = -self.sentido_lineas
        self._par_abierto = None
        self.al_log("[vueltas] ahora en sentido contrario, contador a cero")

    # ------------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "vueltas": self.vueltas,
            "objetivo": self.objetivo,
            "esquinas": self.esquinas % self.esquinas_por_vuelta,
            "esquinas_por_vuelta": self.esquinas_por_vuelta,
            "tramo": "ida" if self.tramo == IDA else "vuelta",
            "terminado": self.terminado,
            "media_vuelta_pendiente": self.media_vuelta_pendiente,
            "sentido_lineas": self.sentido_lineas,
            "ultima_por": self.ultima_esquina_por,
            "ignoradas": self.ignoradas,
            "historial": self.historial[-8:],
        }


# ---------------------------------------------------------------------------
class DetectorLineasCamara:
    """Ve las lineas del suelo en la franja de abajo de la imagen.

    Solo mira el recorte inferior: ahi la linea esta cerca, se ve gruesa y no
    se confunde con nada del fondo. Hace falta que la fraccion de pixeles del
    color supere un umbral y luego BAJE para contar un cruce (flanco), que es
    lo que evita contar diez veces la misma linea mientras se pasa por encima.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg or {}
        self.estado_actual = P.LINEA_NINGUNA
        self.fracciones: Dict[str, float] = {"naranja": 0.0, "azul": 0.0}
        self._t_ultimo = 0.0
        self._candidato = P.LINEA_NINGUNA
        self._repeticiones = 0

    def procesar(self, mascaras: Dict[str, Any]) -> int:
        """Devuelve la linea recien cruzada, o LINEA_NINGUNA.

        Tres cosas que evitan contar de mas:
          * DOMINANCIA: cerca de una esquina se ven las dos lineas a la vez. Si
            la ganadora no es claramente mayor que la otra, no se declara nada;
            si no, el maximo va saltando entre naranja y azul y cada salto
            parecia un cruce nuevo.
          * CONFIRMACION: hacen falta varios frames seguidos para declarar.
          * HISTERESIS: para soltar la linea la fraccion tiene que bajar bien
            por debajo del umbral, no solo rozarlo.
        """
        import numpy as np
        umbral = float(self.cfg.get("umbral_linea_camara", 0.02))
        arriba = float(self.cfg.get("roi_linea_arriba", 0.78))
        dominancia = float(self.cfg.get("dominancia_linea", 1.6))
        conf = int(self.cfg.get("frames_confirmar_linea", 2))
        soltar = float(self.cfg.get("histeresis_linea", 0.55))

        fracs: Dict[int, float] = {}
        for nombre, ident in (("naranja", P.LINEA_NARANJA), ("azul", P.LINEA_AZUL)):
            m = mascaras.get(nombre)
            if m is None:
                self.fracciones[nombre] = 0.0
                fracs[ident] = 0.0
                continue
            H = m.shape[0]
            recorte = m[int(H * arriba):]
            frac = float(np.count_nonzero(recorte)) / max(1, recorte.size)
            self.fracciones[nombre] = round(frac, 4)
            fracs[ident] = frac

        orden = sorted(fracs.items(), key=lambda kv: -kv[1])
        mejor, mejor_frac = orden[0]
        segunda = orden[1][1] if len(orden) > 1 else 0.0

        if self.estado_actual != P.LINEA_NINGUNA:
            # ya estamos encima de una: se suelta con histeresis
            nuevo = (self.estado_actual
                     if fracs.get(self.estado_actual, 0.0) >= umbral * soltar
                     else P.LINEA_NINGUNA)
        elif mejor_frac >= umbral and mejor_frac >= segunda * dominancia:
            nuevo = mejor
        else:
            nuevo = P.LINEA_NINGUNA

        if nuevo == self._candidato:
            self._repeticiones += 1
        else:
            self._candidato = nuevo
            self._repeticiones = 1

        salida = P.LINEA_NINGUNA
        hacen_falta = 1 if nuevo == P.LINEA_NINGUNA else conf
        if nuevo != self.estado_actual and self._repeticiones >= hacen_falta:
            if nuevo != P.LINEA_NINGUNA:
                salida = nuevo          # flanco de subida: acabamos de pisarla
            self.estado_actual = nuevo
        return salida
