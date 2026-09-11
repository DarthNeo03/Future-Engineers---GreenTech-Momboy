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

TRES FILTROS, EN ESTE ORDEN, Y NINGUNO SOBRA

  1. HORIZONTE. Todo contorno cuya base quede por encima de la fila del
     horizonte se tira sin mirar el color. Los muros miden 100 mm y la camara
     va a 125: nada de la pista puede aparecer ahi arriba. Esto es lo que
     hace que una camiseta roja del publico no sea un pilar.
  2. FORMA. Un pilar es un rectangulo vertical de 50x100 mm: relacion de
     aspecto y llenado del contorno dentro de su caja.
  3. COHERENCIA GEOMETRICA. La distancia deducida de la BASE (donde toca el
     suelo) y la deducida de la ALTURA APARENTE tienen que parecerse. Un
     reflejo en el tapete falla este filtro aunque pase los dos anteriores.
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
    # Distancia a la linea de piso que se ve delante, por color, o None.
    lineas_mm: Dict[str, Optional[float]] = field(default_factory=dict)
    mascaras: Dict[str, np.ndarray] = field(default_factory=dict)

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
            esc.pilares.extend(self._pilares(m, color, cfg, horizonte))
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
        esc.perfil_mm, esc.perfil_lat_mm = self._perfil(solidos, horizonte)

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
                   horizonte: int) -> List[Tuple[int, int, int, int, float]]:
        cont, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        area_min = float(cfg.get("area_min", 300))
        salida = []
        for c in cont:
            area = cv2.contourArea(c)
            if area < area_min:
                continue
            x, y, w, h = cv2.boundingRect(c)
            # FILTRO 1: base por encima del horizonte -> no es pista.
            if (y + h) <= horizonte + 2:
                continue
            salida.append((x, y, w, h, area))
        return salida

    def _pilares(self, mascara: np.ndarray, color: str, cfg: Dict[str, Any],
                 horizonte: int) -> List[Deteccion]:
        maxn = int(cfg.get("max_objetos", 4))
        dets: List[Deteccion] = []
        for x, y, w, h, area in self._contornos(mascara, cfg, horizonte):
            # FILTRO 2: forma. Un pilar de 50x100 mm visto de frente da una
            # caja mas alta que ancha; de canto, casi cuadrada. Fuera de ese
            # margen es una mancha o dos pilares pegados.
            aspecto = h / max(1.0, float(w))
            if not (float(cfg.get("aspecto_min", 0.7)) <= aspecto <=
                    float(cfg.get("aspecto_max", 4.0))):
                continue
            llenado = area / max(1.0, float(w * h))
            if llenado < float(cfg.get("llenado_min", 0.55)):
                continue

            base_v = float(y + h)
            # Si la caja toca el borde inferior, la base real esta FUERA del
            # frame: la distancia por suelo saldria demasiado grande. En ese
            # caso solo vale la medida por altura, y ademas ya sabemos que
            # esta muy cerca.
            base_cortada = (y + h) >= (mascara.shape[0] - 2)
            d_base = float(self.geo.fila_a_distancia(base_v))
            d_alto = self.geo.distancia_por_altura(h)

            if base_cortada:
                dist = d_alto
                conf = 0.6
            else:
                # FILTRO 3: las dos medidas tienen que parecerse.
                if not self.geo.coherente(d_base, d_alto):
                    continue
                dist = 0.5 * (d_base + d_alto)
                conf = 1.0
            if dist >= DIST_MAX_MM:
                continue

            lat = float(self.geo.lateral_mm(x + w / 2.0, base_v))
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
        for x, y, w, h, area in self._contornos(mascara, cfg, horizonte):
            base_v = float(y + h)
            dist = float(self.geo.fila_a_distancia(base_v))
            if dist >= DIST_MAX_MM:
                continue
            lat = float(self.geo.lateral_mm(x + w / 2.0, base_v))
            dets.append(Deteccion(color=color, x=x, y=y, w=w, h=h, area=area,
                                  dist_mm=dist, lat_mm=lat, confianza=1.0))
        dets.sort(key=lambda d: d.dist_mm)
        return dets[:int(cfg.get("max_objetos", 3))]

    def _perfil(self, solidos: np.ndarray, horizonte: int,
                sectores: int = 32) -> Tuple[np.ndarray, np.ndarray]:
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
        lat = np.empty(sectores, np.float32)
        for i in range(sectores):
            a, b = borde[i], max(borde[i] + 1, borde[i + 1])
            trozo = dist_col[a:b]
            perfil[i] = float(np.min(trozo))
            u_mid = (a + b) / 2.0
            v_mid = float(np.mean(v_base[a:b]))
            lat[i] = float(self.geo.lateral_mm(u_mid, v_mid))
        return perfil, lat

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
