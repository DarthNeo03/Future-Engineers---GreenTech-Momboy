"""
vision.py — De un frame BGR a una descripcion del mundo en milimetros.

QUE SE BUSCA Y POR QUE (reglamento 2026, seccion 13)

    rojo    RGB(238, 39, 55)   señal de transito: se pasa por su DERECHA (9.19)
    verde   RGB( 68,214, 44)   señal de transito: se pasa por su IZQUIERDA
    magenta RGB(255,  0,255)   delimitadores del cajon. TOCARLOS TERMINA LA
                               RONDA (9.25.7). Aqui son muro, no objetivo.
    negro                      muros interior y exterior: definen el carril
    naranja / azul             lineas del piso, de 20 mm. Marcan el limite de
                               seccion. Las ve tambien el TCS bajo el carro;
                               verlas con la camara las anticipa ~1 m.

POR QUE HSV Y NO RGB
El tapete se ilumina con lo que haya en el pabellon: focos amarillos, luz de
ventana, sombras del propio carro. En RGB, "rojo" cambia de valor con cada
nube. En HSV el TONO es casi el mismo y lo que se mueve es el brillo, que es
justo el canal que se deja abierto de par en par en los rangos.

EL ROJO NECESITA DOS RANGOS porque el tono es circular y el rojo esta a
caballo del cero: hay que coger 0-10 y 150-179 y unirlos.

DOS FILTROS QUE DESCARTAN, Y UNA TERCERA MEDIDA QUE SOLO PONDERA

  1. HORIZONTE. Un contorno cuya base quede muy por encima de la fila del
     horizonte no es pista. Los muros miden 100 mm y la camara va a 125: nada
     del tapete puede aparecer ahi arriba. Esto es lo que hace que una
     camiseta roja del publico no sea un pilar. Lleva un margen configurable
     porque la fila del horizonte se calcula con la inclinacion CONFIGURADA,
     y un mastil torcido la desplaza entera.
  2. FORMA. Un pilar es un rectangulo vertical de 50x100 mm: relacion de
     aspecto y llenado del contorno dentro de su caja.

  Y LUEGO, LA DISTANCIA, QUE YA NO DESCARTA NADA. Se mide de dos maneras
  —por la base y por la altura aparente— pero no se exige que coincidan. Ver
  el comentario largo dentro de _pilares: exigirlo borraba todos los pilares
  en cuanto la inclinacion estaba mal medida, que es el estado normal el dia
  de la competencia.

TODO DESCARTE QUEDA CONTADO en Escena.descartes. "No ve los pilares" tiene que
ser una pregunta respondible desde la telemetria, no una tarde de pista.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .geometria import DIST_MAX_MM, Geometria

# Colores que se buscan. El orden no importa salvo para dibujar.
COLORES_PILAR = ("rojo", "verde")
COLORES_LINEA = ("naranja", "azul")


@dataclass
class Deteccion:
    """Un objeto visto, ya traducido a milimetros sobre el suelo."""
    color: str
    x: int = 0            # caja en pixeles
    y: int = 0
    w: int = 0
    h: int = 0
    area: float = 0.0
    dist_mm: float = DIST_MAX_MM      # distancia adelante, desde la camara
    lat_mm: float = 0.0               # + a la derecha del eje del carro
    dist_alto_mm: float = DIST_MAX_MM  # segunda medida, por altura aparente
    confianza: float = 0.0

    @property
    def centro_u(self) -> float:
        return self.x + self.w / 2.0

    @property
    def base_v(self) -> float:
        return float(self.y + self.h)


@dataclass
class Escena:
    """Todo lo que el carro sabe del mundo en este frame."""
    ancho: int = 640
    alto: int = 480
    pilares: List[Deteccion] = field(default_factory=list)
    magenta: List[Deteccion] = field(default_factory=list)
    # Perfil de distancia libre: para cada sector de columnas, a que distancia
    # aparece el primer obstaculo solido (muro negro o delimitador magenta).
    perfil_mm: np.ndarray = field(default_factory=lambda: np.zeros(0))
    perfil_lat_mm: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Rumbo de cada sector en grados (+ derecha). Existe haya muro o no, que
    # es justo lo que hace falta para apuntar a un hueco.
    perfil_rumbo_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Distancia a la linea de piso que se ve delante, por color, o None.
    lineas_mm: Dict[str, Optional[float]] = field(default_factory=dict)
    mascaras: Dict[str, np.ndarray] = field(default_factory=dict)
    # Por que se tiro cada contorno. Sin esto, "no ve los pilares" es un
    # callejon sin salida: la mascara esta bien, el filtro se los come y no
    # queda rastro. Viaja a la telemetria y al panel de depuracion.
    descartes: Dict[str, int] = field(default_factory=dict)

    def pilar_mas_cercano(self) -> Optional[Deteccion]:
        return min(self.pilares, key=lambda d: d.dist_mm, default=None)


def _mascara(hsv: np.ndarray, cfg: Dict[str, Any]) -> np.ndarray:
    """Umbral HSV + morfologia. Un solo sitio para todos los colores: asi la
    calibracion de uno no puede quedar con un tratamiento distinto del resto."""
    m = None
    for lo, hi in cfg.get("rangos", []):
        parcial = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        m = parcial if m is None else cv2.bitwise_or(m, parcial)
    if m is None:
        return np.zeros(hsv.shape[:2], np.uint8)
    # ABRIR quita el ruido de sal (pixeles sueltos del sensor a ISO alto).
    k_ab = int(cfg.get("abrir", 3))
    if k_ab > 1:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_ab, k_ab)))
    # CERRAR tapa el brillo especular que parte un pilar en dos manchas.
    k_ce = int(cfg.get("cerrar", 5))
    if k_ce > 1:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_ce, k_ce)))
    return m


class Detector:
    def __init__(self, colores: Dict[str, Any], geo: Geometria) -> None:
        self.colores = colores
        self.geo = geo

    # ------------------------------------------------------------ publico
    def procesar(self, frame: np.ndarray, *, con_mascaras: bool = False) -> Escena:
        alto, ancho = frame.shape[:2]
        self.geo.redimensionar(ancho, alto)
        esc = Escena(ancho=ancho, alto=alto)

        # Un solo desenfoque y una sola conversion para todos los colores: es
        # la parte cara del pipeline y no tiene sentido repetirla siete veces.
        suave = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(suave, cv2.COLOR_BGR2HSV)
        horizonte = self.geo.fila_horizonte()

        # --- señales de transito ---------------------------------------
        for color in COLORES_PILAR:
            cfg = self.colores.get(color)
            if not cfg:
                continue
            m = _mascara(hsv, cfg)
            if con_mascaras:
                esc.mascaras[color] = m
            esc.pilares.extend(self._pilares(m, color, cfg, horizonte,
                                             esc.descartes))
        esc.pilares.sort(key=lambda d: d.dist_mm)

        # --- delimitadores del cajon (prohibido tocarlos) ---------------
        cfg_mag = self.colores.get("magenta")
        m_mag = _mascara(hsv, cfg_mag) if cfg_mag else np.zeros((alto, ancho), np.uint8)
        if con_mascaras:
            esc.mascaras["magenta"] = m_mag
        if cfg_mag:
            esc.magenta = self._bloques(m_mag, "magenta", cfg_mag, horizonte)

        # --- muros: perfil de espacio libre ------------------------------
        cfg_neg = self.colores.get("negro")
        m_neg = _mascara(hsv, cfg_neg) if cfg_neg else np.zeros((alto, ancho), np.uint8)
        # Los delimitadores magenta se suman a los muros: para navegar son lo
        # mismo, un solido que no se puede tocar. La diferencia es que tocar
        # el magenta termina la ronda, asi que se les da MAS margen, no menos.
        solidos = cv2.bitwise_or(m_neg, m_mag)
        if con_mascaras:
            esc.mascaras["solidos"] = solidos
        (esc.perfil_mm, esc.perfil_lat_mm,
         esc.perfil_rumbo_deg) = self._perfil(solidos, horizonte)

        # --- lineas del piso --------------------------------------------
        for color in COLORES_LINEA:
            cfg = self.colores.get(color)
            if not cfg:
                esc.lineas_mm[color] = None
                continue
            m = _mascara(hsv, cfg)
            if con_mascaras:
                esc.mascaras[color] = m
            esc.lineas_mm[color] = self._linea(m, cfg, horizonte)

        return esc

    # ------------------------------------------------------------ privados
    def _contornos(self, mascara: np.ndarray, cfg: Dict[str, Any],
                   horizonte: int, margen_horizonte: int = 0,
                   descartes: Optional[Dict[str, int]] = None,
                   etiqueta: str = "") -> List[Tuple[int, int, int, int, float]]:
        cont, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        area_min = float(cfg.get("area_min", 300))
        # FILTRO 1: la base por encima del horizonte no puede ser pista.
        #
        # OJO CON EL MARGEN. La fila del horizonte se calcula con la
        # inclinacion configurada, asi que un error de montaje la desplaza
        # entera: con 4 grados de mas, un pilar legitimo a 1.5 m cae por
        # encima del horizonte "teorico" y se tira. El margen compra esa
        # tolerancia. Cuanto mas grande, mas robusto al montaje y mas
        # expuesto a ver cosas del publico; 70 px aguanta ~8 grados de error.
        corte = horizonte + 2 - max(0, int(margen_horizonte))
        salida = []
        for c in cont:
            area = cv2.contourArea(c)
            if area < area_min:
                if descartes is not None:
                    descartes[f"{etiqueta}:area"] = descartes.get(f"{etiqueta}:area", 0) + 1
                continue
            x, y, w, h = cv2.boundingRect(c)
            if (y + h) <= corte:
                if descartes is not None:
                    descartes[f"{etiqueta}:horizonte"] = descartes.get(f"{etiqueta}:horizonte", 0) + 1
                continue
            salida.append((x, y, w, h, area))
        return salida

    def _pilares(self, mascara: np.ndarray, color: str, cfg: Dict[str, Any],
                 horizonte: int, descartes: Dict[str, int]) -> List[Deteccion]:
        maxn = int(cfg.get("max_objetos", 4))
        alto_img = mascara.shape[0]
        dets: List[Deteccion] = []
        for x, y, w, h, area in self._contornos(
                mascara, cfg, horizonte, int(cfg.get("margen_horizonte_px", 70)),
                descartes, color):
            # FILTRO 2: forma. Un pilar de 50x100 mm visto de frente da una
            # caja mas alta que ancha; de canto, casi cuadrada. Fuera de ese
            # margen es una mancha o dos pilares pegados.
            aspecto = h / max(1.0, float(w))
            if not (float(cfg.get("aspecto_min", 0.6)) <= aspecto <=
                    float(cfg.get("aspecto_max", 4.5))):
                descartes[f"{color}:forma"] = descartes.get(f"{color}:forma", 0) + 1
                continue
            llenado = area / max(1.0, float(w * h))
            if llenado < float(cfg.get("llenado_min", 0.5)):
                descartes[f"{color}:llenado"] = descartes.get(f"{color}:llenado", 0) + 1
                continue

            # ===== DISTANCIA: MANDA LA ALTURA APARENTE =====================
            # Aqui hubo un fallo que costo una tarde de pista: se exigia que
            # la distancia por BASE y la distancia por ALTURA coincidieran, y
            # si no, se descartaba el pilar. El problema es que las dos NO
            # dependen de lo mismo:
            #
            #   por altura -> solo necesita fy. Es exacta aunque el mastil de
            #                 la camara este torcido.
            #   por base   -> necesita fy, la altura de la camara Y la
            #                 inclinacion. Con 4 grados de error de montaje,
            #                 un pilar a 1.2 m se reporta a 4.9 m.
            #
            # O sea que el "desacuerdo" no era senal de deteccion falsa: era
            # senal de calibracion imperfecta, que es el estado normal de un
            # carro el dia de la competencia. El filtro borraba TODOS los
            # pilares en silencio y el carro pasaba de largo sin mirar el
            # color, que es exactamente lo que se vio en la pista.
            #
            # Ahora la altura es la medida PRIMARIA y la base solo corrobora:
            # si coinciden, se promedia y sube la confianza; si no, se cree a
            # la altura y se baja la confianza, pero el pilar NO se pierde.
            d_alto = self.geo.distancia_por_altura(h)
            base_cortada = (y + h) >= (alto_img - 2)
            d_base = (None if base_cortada
                      else float(self.geo.fila_a_distancia(float(y + h))))

            dist = d_alto
            conf = 0.8
            if d_base is not None:
                if self.geo.coherente(d_base, d_alto):
                    dist = 0.5 * (d_base + d_alto)
                    conf = 1.0
                else:
                    conf = 0.5
                    descartes["geometria_discrepa"] = descartes.get(
                        "geometria_discrepa", 0) + 1

            if not (40.0 <= dist < DIST_MAX_MM):
                descartes[f"{color}:distancia"] = descartes.get(f"{color}:distancia", 0) + 1
                continue

            # El lateral tambien sale de la distancia, no de la fila: asi el
            # lado por el que hay que rebasar no depende de la inclinacion.
            lat = self.geo.lateral_por_distancia(x + w / 2.0, dist)
            dets.append(Deteccion(color=color, x=x, y=y, w=w, h=h, area=area,
                                  dist_mm=dist, lat_mm=lat,
                                  dist_alto_mm=d_alto, confianza=conf))
        dets.sort(key=lambda d: d.dist_mm)
        return dets[:maxn]

    def _bloques(self, mascara: np.ndarray, color: str, cfg: Dict[str, Any],
                 horizonte: int) -> List[Deteccion]:
        """Como _pilares pero sin filtro de forma: los delimitadores son
        tumbados (200 x 20 x 100 mm) y su caja cambia mucho con el angulo."""
        dets: List[Deteccion] = []
        alto_img = mascara.shape[0]
        for x, y, w, h, area in self._contornos(
                mascara, cfg, horizonte, int(cfg.get("margen_horizonte_px", 70))):
            # Los delimitadores tambien miden 100 mm de alto, asi que valen las
            # dos medidas. Se toma la MAS CERCANA de las dos a proposito:
            # tocarlos termina la ronda (9.25.7), asi que equivocarse por
            # creerlos mas cerca de lo que estan no cuesta nada, y al reves si.
            d_alto = self.geo.distancia_por_altura(h)
            base_cortada = (y + h) >= (alto_img - 2)
            dist = d_alto
            if not base_cortada:
                dist = min(d_alto, float(self.geo.fila_a_distancia(float(y + h))))
            if not (40.0 <= dist < DIST_MAX_MM):
                continue
            lat = self.geo.lateral_por_distancia(x + w / 2.0, dist)
            dets.append(Deteccion(color=color, x=x, y=y, w=w, h=h, area=area,
                                  dist_mm=dist, lat_mm=lat,
                                  dist_alto_mm=d_alto, confianza=1.0))
        dets.sort(key=lambda d: d.dist_mm)
        return dets[:int(cfg.get("max_objetos", 3))]

    def _perfil(self, solidos: np.ndarray, horizonte: int, sectores: int = 32
                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Distancia libre por sector de columnas.

        Se busca, en cada columna, el pixel solido MAS BAJO: ese es el punto
        donde el muro se apoya en el tapete, o sea el final del suelo libre en
        esa direccion. Se hace vectorizado sobre todo el frame de una vez
        porque en Python un bucle por columna cuesta mas que el resto del
        pipeline junto.
        """
        alto, ancho = solidos.shape[:2]
        blk = solidos > 0
        # Nada por encima del horizonte cuenta: ahi no hay pista.
        v0 = max(0, horizonte + 1)
        blk[:v0, :] = False

        hay = blk.any(axis=0)
        # argmax sobre el array invertido = primer True desde abajo.
        idx = alto - 1 - np.argmax(blk[::-1, :], axis=0)
        v_base = np.where(hay, idx, alto - 1).astype(np.float32)
        dist_col = self.geo.fila_a_distancia(v_base)
        dist_col = np.where(hay, dist_col, DIST_MAX_MM)

        # Agrupar en sectores con el MINIMO, no con la media: para no chocar
        # importa el obstaculo mas cercano del sector, no el tipico.
        borde = np.linspace(0, ancho, sectores + 1).astype(int)
        perfil = np.empty(sectores, np.float32)
        rumbo = np.empty(sectores, np.float32)
        for i in range(sectores):
            a, b = borde[i], max(borde[i] + 1, borde[i + 1])
            perfil[i] = float(np.min(dist_col[a:b]))
            # RUMBO, no lateral: el rumbo de un sector existe tenga muro o no.
            # El lateral de un sector LIBRE no existe (no hay nada que situar
            # en el suelo), y calcularlo con una fila inventada daba ~20 mm en
            # vez de "lejos hacia ese lado": el carro no giraba en las curvas.
            rumbo[i] = float(self.geo.rumbo_de_columna((a + b) / 2.0))
        # El lateral sale del rumbo y de la distancia, y solo tiene sentido
        # donde de verdad hay muro. Donde no lo hay queda enorme, que es la
        # respuesta honesta: "por ahi no hay nada en 6 metros".
        lat = perfil * np.sin(np.radians(rumbo))
        return perfil, lat, rumbo

    def _linea(self, mascara: np.ndarray, cfg: Dict[str, Any],
               horizonte: int) -> Optional[float]:
        """Distancia a la linea de piso mas cercana de ese color, o None.

        Se busca una mancha ANCHA Y BAJA: las lineas cruzan el carril entero,
        asi que ocupan mucho ancho y poca altura. Ese criterio descarta por si
        solo un pilar del mismo tono visto de lejos.
        """
        mejor: Optional[float] = None
        alto, ancho = mascara.shape[:2]
        cont, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        for c in cont:
            area = cv2.contourArea(c)
            if area < float(cfg.get("area_min", 500)):
                continue
            x, y, w, h = cv2.boundingRect(c)
            if (y + h) <= horizonte + 2:
                continue
            if w < float(cfg.get("ancho_min_frac", 0.18)) * ancho:
                continue
            if h > w:            # mas alta que ancha: no es una linea de piso
                continue
            d = float(self.geo.fila_a_distancia(float(y + h)))
            if d < DIST_MAX_MM and (mejor is None or d < mejor):
                mejor = d
        return mejor
