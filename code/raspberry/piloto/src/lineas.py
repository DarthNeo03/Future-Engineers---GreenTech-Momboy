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

# ---------------------------------------------------------------------------
# GEMELO EN PYTHON del clasificador que corre en el ESP32 (code/esp32_carro/
# lineas.h). El que decide de verdad es el firmware; este sirve para poder
# probar la regla sin carro y para enseñar en la web que diria con los
# umbrales actuales, que es lo unico que hace calibrable el sensor.
# Si tocas uno, toca el otro.
#
# Lo que manda es la DIFERENCIA entre los ratios, no su valor absoluto. Medido
# en la pista del equipo sobre la linea azul (C=678 R=186 B=284): r=70, b=107.
# El azul solo saca 22 puntos al blanco en su propio canal (107 contra ~85),
# margen tan fino que un umbral absoluto de 110 no la veia; en la diferencia
# saca 37 (b-r = +37 contra ~0 del blanco).
def clase_tcs(c: int, r: int, g: int, b: int,
              cfg: Optional[Dict[str, Any]] = None) -> str:
    """Devuelve 'naranja', 'azul' o '-' para una lectura cruda del TCS34725."""
    cfg = cfg or {}
    if c is None or c <= 0 or c < int(cfg.get("c_min", 80)):
        return "-"
    rr = r * 255 // c
    rb = b * 255 // c
    dif = rb - rr                       # >0 azulado, <0 anaranjado
    if (-dif >= int(cfg.get("naranja_dif_min", 30))
            and rr >= int(cfg.get("naranja_r_min", 110))
            and rb <= int(cfg.get("naranja_b_max", 90))):
        return "naranja"
    if (dif >= int(cfg.get("azul_dif_min", 18))
            and rb >= int(cfg.get("azul_b_min", 95))
            and rr <= int(cfg.get("azul_r_max", 95))):
        return "azul"
    return "-"


def umbrales_desde_muestra(que: str, ratio_r: float, ratio_b: float,
                           c_medio: float) -> Dict[str, int]:
    """Umbrales a partir de una lectura tomada SOBRE la superficie indicada.

    Se ajusta sobre todo la diferencia, que es el discriminador: se pone al
    55 % de la medida, asi que hay que perder casi la mitad del contraste para
    dejar de ver la linea. Los absolutos quedan holgados (reja, no criterio).

    CUIDADO CON c_min, que costo caro: una linea de color absorbe mucha luz y
    devuelve un canal claro MUCHO menor que el piso blanco. Medido en la pista
    del equipo, la linea azul daba C=678 con un blanco de ~3400, o sea un 20 %.
    Con un c_min sacado del 25 % del blanco (842) la linea azul se descartaba
    antes de mirar siquiera su color, y el TCS no la vio nunca. Por eso el
    suelo de luz se saca ahora del 8 % del blanco, y ademas muestrear una
    linea BAJA el c_min si hiciera falta para que esa linea entre.
    """
    dif = ratio_b - ratio_r
    if que == "naranja":
        return {"naranja_dif_min": max(8, int(-dif * 0.55)),
                "naranja_r_min": max(0, int(ratio_r * 0.80)),
                "naranja_b_max": min(255, int(ratio_b * 1.5) + 20),
                "c_min": max(1, int(c_medio * 0.70))}
    if que == "azul":
        return {"azul_dif_min": max(8, int(dif * 0.55)),
                "azul_b_min": max(0, int(ratio_b * 0.80)),
                "azul_r_max": min(255, int(ratio_r * 1.5) + 20),
                "c_min": max(1, int(c_medio * 0.70))}
    if que == "blanco":
        # Solo el suelo de luz (por debajo es sombra o sensor tapado), y bien
        # bajo: las lineas son mucho mas oscuras que el piso.
        return {"c_min": max(1, int(c_medio * 0.08))}
    raise ValueError("que debe ser blanco/naranja/azul")


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
        # Orden canonico del par de lineas de la ronda (p.ej. naranja+azul en
        # horario). Lo fija el primer par completo y sirve para descartar
        # lecturas incoherentes.
        self.orden_esperado: Optional[List[str]] = None
        self.incoherencias = 0
        self.pares_incompletos = 0
        self._pares_invertidos = 0

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
        esquina_max_ms se vuelve a recta y la navegacion normal retoma.

        Aqui tambien caduca el par de lineas a medias: si la segunda linea no
        llego dentro de la ventana, se olvida la primera para que no se empareje
        con la de la esquina SIGUIENTE (eso daria un par al reves y una
        incoherencia inventada). Esa esquina la contara el giro de 90.
        """
        ahora = time.time()
        if self._colores_esquina and len(self._colores_esquina) == 1:
            ventana = float(self.cfg.get("ventana_par_ms", 2500)) / 1000.0
            if ahora - self._t_evento > ventana:
                self.pares_incompletos += 1
                self._colores_esquina = []
        if self.zona == ZONA_ESQUINA:
            limite = float(self.cfg.get("esquina_max_ms", 6000)) / 1000.0
            if ahora - self._t_zona > limite:
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
            self._t_evento = ahora
            if color not in self._colores_esquina:
                # PAR COMPLETO: las dos lineas de la esquina, en el orden en
                # que se cruzaron. Aqui es donde se decide si cuenta.
                self._colores_esquina.append(color)
                self.orden_observado = list(self._colores_esquina)
                self._cerrar_par(list(self._colores_esquina))
            return

        if ahora - self._t_esquina < refract:
            # zona muerta: probablemente rebote de la misma esquina
            self._t_evento = ahora
            return

        # Primera linea de una esquina nueva. El carro ENTRA en la curva ya
        # mismo (eso es lo que corta el bucle), pero el CONTADOR todavia no
        # se toca: se espera a ver la segunda linea del par.
        self._t_evento = ahora
        self._colores_esquina = [color]
        self.orden_observado = [color]
        if self.sentido == DESCONOCIDO:
            self.sentido = self._sentido_de(color)
        self.entrar_en_esquina(f"linea {color} ({fuente})")

    def _cerrar_par(self, orden: List[str]) -> None:
        """Decide si un par de lineas completo cuenta como esquina.

        Las cuatro esquinas de una vuelta se cruzan SIEMPRE en el mismo orden:
        yendo en horario, naranja y luego azul en todas; en antihorario, al
        reves. Por eso el orden es una comprobacion de coherencia gratis: si
        aparece un par al reves, o el carro se dio la vuelta, o esa lectura es
        basura. Contar una esquina de mas estropea la cuenta de vueltas y hace
        que el carro se pare donde no debe, asi que ante la duda no se cuenta.
        """
        ahora = time.time()
        if self.orden_esperado is None:
            # el primer par completo fija la referencia de toda la ronda
            self.orden_esperado = list(orden)
            self.sentido = self._sentido_de(orden[0])
            self._contar_esquina(ahora, f"par {'+'.join(orden)} (referencia)")
            return

        if orden == self.orden_esperado:
            self._pares_invertidos = 0
            self._contar_esquina(ahora, f"par {'+'.join(orden)}")
            return

        # --- par al reves ---------------------------------------------------
        self.incoherencias += 1
        self._pares_invertidos += 1
        minimo = int(self.cfg.get("pares_para_invertir", 2))
        if self._pares_invertidos >= minimo:
            # ya no es una lectura suelta: el carro va de verdad al reves
            self.orden_esperado = list(orden)
            self.sentido = self._sentido_de(orden[0])
            self._pares_invertidos = 0
            self._contar_esquina(ahora, f"par {'+'.join(orden)}: SENTIDO INVERTIDO")
        else:
            # una sola vez: incoherente, no se cuenta (pero si se esta en la
            # curva, que de eso se encarga la zona)
            self.ultimo_evento = (f"par {'+'.join(orden)} INCOHERENTE "
                                  f"(esperaba {'+'.join(self.orden_esperado)}), no cuenta")

    def _contar_esquina(self, ahora: float, motivo: str) -> None:
        self.esquinas += 1
        self._t_esquina = ahora
        self.ultimo_evento = motivo

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
            "orden_esperado": "+".join(self.orden_esperado or []),
            "incoherencias": self.incoherencias,
            "pares_incompletos": self.pares_incompletos,
        }
