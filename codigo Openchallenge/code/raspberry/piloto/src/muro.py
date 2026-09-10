"""
muro.py — Donde el muro toca el piso, robusto al brillo de la pared.

EL PROBLEMA DEL METODO VIEJO
El programa anterior buscaba "el pixel NEGRO mas bajo" por columna. Dos fallas
documentadas con capturas reales:
  1. La pared brillante de cerca deja de pasar el umbral de negro, y el pixel
     negro mas bajo pasa a ser una silla del fondo: la linea de contacto se
     dibuja POR ENCIMA del muro y el carro cree que tiene via libre.
  2. El brillo parte la pared en trozos y la linea se deforma/descontinua.

EL METODO NUEVO ('piso')
Dos ideas, las dos geometricas y no de color de pared:

  a. HORIZONTE. La camara esta a 125 mm y mira 7.5 grados hacia abajo; los
     muros miden 100 mm. TODO lo que es pista (piso y muros) queda por debajo
     de la fila del horizonte. Sillas, mesas y publico quedan por encima:
     se recortan por geometria antes de mirar un solo color.

  b. PRIMERA TRANSICION PISO -> NO-PISO, DESDE ABAJO. Lo unico que suponemos
     es que el PISO se ve como piso (blanco, o linea naranja/azul/magenta,
     que tambien son piso). Subiendo por cada columna desde el borde inferior,
     la primera racha de k filas que NO parecen piso es la base del muro.
     No importa si la pared brilla, es gris o es carbono tejido: basta con
     que no parezca piso blanco. Y como se toma la PRIMERA transicion, lo que
     haya detras del muro no puede adelantarse.

Despues, el contorno se convierte a milimetros sobre el suelo (geometria.py)
y se ajustan RECTAS por tramos: las rectas casi colineales separadas por un
hueco se fusionan (eso puentea los cortes que el brillo todavia cause) y las
intersecciones entre rectas dan las esquinas, con su tipo.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .geometria import Geometria, DIST_MAX_MM


# ---------------------------------------------------------------------------
@dataclass
class Segmento:
    """Recta ajustada a un tramo del contacto muro-piso, en mm sobre el suelo
    (x lateral +derecha, y hacia adelante)."""
    x0: float
    y0: float
    x1: float
    y1: float
    n_puntos: int = 0
    col0: int = 0          # columnas de imagen que abarca (para dibujar)
    col1: int = 0

    @property
    def angulo(self) -> float:
        """Angulo en grados respecto al eje de avance (0 = paralelo al carro)."""
        return math.degrees(math.atan2(self.y1 - self.y0, self.x1 - self.x0))

    @property
    def largo(self) -> float:
        return math.hypot(self.x1 - self.x0, self.y1 - self.y0)


@dataclass
class Esquina:
    """Interseccion de dos segmentos de muro."""
    x: float               # mm sobre el suelo
    y: float
    angulo: float          # angulo entre los dos segmentos (grados)
    tipo: str              # "saliente" = esquina que apunta al carro (tipico
    #                        fin de muro interno); "rincon" = concava (tipico
    #                        rincon del muro externo)


@dataclass
class PerfilMuro:
    """El 'LIDAR pobre', ahora en milimetros."""
    y_contacto: np.ndarray          # (W,) fila del contacto; 0 = sin muro
    valido: np.ndarray              # (W,) bool: esa columna ve muro
    dist_mm: np.ndarray             # (W,) distancia al muro desde el morro
    libre: np.ndarray               # (W,) dist/alcance, 0..1
    alto: int = 0
    ancho: int = 0
    y_horizonte: int = 0
    alcance_mm: float = 2500.0

    izq: float = 1.0                # media de libre en la banda izquierda
    der: float = 1.0
    pasillo_mm: float = DIST_MAX_MM  # percentil 15 de lo que hay EN el camino
    pasillo: float = 1.0            # pasillo_mm / alcance, 0..1
    min_mm: float = DIST_MAX_MM
    cobertura_izq: float = 0.0      # fraccion de la banda que ve muro a rango
    cobertura_der: float = 0.0
    hay_muro: bool = False

    bordes: List[Tuple[int, float, float]] = field(default_factory=list)
    #        (columna, dist_antes_mm, dist_despues_mm) saltos de profundidad
    segmentos: List[Segmento] = field(default_factory=list)
    esquinas: List[Esquina] = field(default_factory=list)


def _media_movil(v: np.ndarray, k: int) -> np.ndarray:
    if k < 3:
        return v
    if k % 2 == 0:
        k += 1
    pad = k // 2
    ext = np.pad(v, pad, mode="edge")
    nucleo = np.ones(k, dtype=np.float32) / k
    return np.convolve(ext, nucleo, mode="valid")


# ---------------------------------------------------------------------------
def perfil(masks: Dict[str, np.ndarray], geo: Geometria,
           cfg: Dict[str, Any]) -> PerfilMuro:
    """masks: mascaras binarias por color (necesita 'blanco'; usa tambien
    'naranja', 'azul', 'magenta' como piso y 'negro' para el metodo viejo)."""
    metodo = str(cfg.get("metodo", "piso"))
    blanco = masks.get("blanco")
    negro = masks.get("negro")
    ref = blanco if blanco is not None else negro
    if ref is None:
        raise ValueError("hacen falta mascaras ('blanco' o 'negro')")
    H, W = ref.shape[:2]
    geo.redimensionar(W, H)

    y_hor = max(0, geo.fila_horizonte() + int(cfg.get("margen_horizonte_px", 4)))
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.05))))
    y_fin = max(y_hor + 2, min(H, y_fin))

    if metodo == "negro" and negro is not None:
        y_cont, valido = _contacto_negro(negro, y_hor, y_fin, cfg)
    else:
        y_cont, valido = _contacto_piso(masks, y_hor, y_fin, cfg)

    p = PerfilMuro(
        y_contacto=y_cont, valido=valido,
        dist_mm=np.zeros(W, np.float32), libre=np.zeros(W, np.float32),
        alto=H, ancho=W, y_horizonte=y_hor,
        alcance_mm=float(cfg.get("alcance_mm", 2500.0)),
    )

    # --- a milimetros ------------------------------------------------------
    morro = float(geo.cfg.get("morro_mm", 60.0))
    d = geo.fila_a_distancia(y_cont.astype(np.float32)) - morro
    d = np.clip(d, 0.0, DIST_MAX_MM)
    d = np.where(valido, d, DIST_MAX_MM)
    # suavizado en distancia (no en filas: las filas lejanas comprimen mucho)
    d_suave = _media_movil(d.astype(np.float32), int(cfg.get("suavizado", 7)))
    p.dist_mm = np.where(valido, d_suave, DIST_MAX_MM).astype(np.float32)
    p.libre = np.clip(p.dist_mm / p.alcance_mm, 0.0, 1.0).astype(np.float32)

    # --- bandas laterales --------------------------------------------------
    banda = float(cfg.get("banda_lateral", 0.28))
    n_lat = max(1, int(W * banda))
    p.izq = float(p.libre[:n_lat].mean())
    p.der = float(p.libre[-n_lat:].mean())
    a_rango_izq = valido[:n_lat] & (p.dist_mm[:n_lat] < p.alcance_mm * 0.98)
    a_rango_der = valido[-n_lat:] & (p.dist_mm[-n_lat:] < p.alcance_mm * 0.98)
    p.cobertura_izq = float(np.count_nonzero(a_rango_izq)) / n_lat
    p.cobertura_der = float(np.count_nonzero(a_rango_der)) / n_lat

    # --- pasillo: lo que de verdad esta EN el camino de las ruedas ---------
    # Un punto de contacto esta "en el camino" si su desplazamiento lateral a
    # SU distancia cabe dentro del semi-ancho del carro + margen.
    semi = (float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0 +
            float(geo.cfg.get("margen_ruedas_mm", 30.0)))
    cols = np.arange(W, dtype=np.float32)
    lat = geo.lateral_mm(cols, np.maximum(y_cont, 1).astype(np.float32))
    en_camino = valido & (np.abs(lat) <= semi)
    if en_camino.any():
        # percentil bajo, no minimo: una columna con ruido no frena el carro
        p.pasillo_mm = float(np.percentile(p.dist_mm[en_camino], 15))
    else:
        p.pasillo_mm = p.alcance_mm
    p.pasillo = min(1.0, p.pasillo_mm / p.alcance_mm)
    p.min_mm = float(p.dist_mm.min()) if valido.any() else DIST_MAX_MM
    p.hay_muro = bool(valido.any())

    # --- bordes (saltos de profundidad) ------------------------------------
    salto = float(cfg.get("salto_borde_mm", 400.0))
    dd = np.abs(np.diff(p.dist_mm))
    ambos = valido[1:] | valido[:-1]
    idx = np.nonzero((dd > salto) & ambos)[0]
    # compactar bordes vecinos (un salto real produce 1-3 columnas seguidas)
    ultimo = -10
    for i in idx:
        if i - ultimo > 4:
            p.bordes.append((int(i), float(p.dist_mm[i]), float(p.dist_mm[i + 1])))
        ultimo = i

    # --- rectas y esquinas --------------------------------------------------
    try:
        p.segmentos, p.esquinas = _segmentos(p, geo, cfg, lat)
    except Exception:
        pass  # el ajuste de rectas es informativo: nunca debe tumbar el lazo
    return p


# ---------------------------------------------------------------------------
def _contacto_piso(masks: Dict[str, np.ndarray], y_hor: int, y_fin: int,
                   cfg: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """Metodo nuevo: primera racha de k filas no-piso subiendo desde abajo."""
    # Piso = blanco + las lineas naranja/azul de las esquinas (estan PINTADAS
    # en el piso). El magenta NO: los delimitadores del estacionamiento 2026
    # son muros fisicos de 10 cm — deben aparecer como obstaculo, no como piso.
    blanco = masks["blanco"]
    H, W = blanco.shape[:2]
    piso = blanco.astype(bool)
    for extra in ("naranja", "azul"):
        m = masks.get(extra)
        if m is not None:
            piso |= m.astype(bool)

    sub = ~piso[y_hor:y_fin]              # True = no parece piso
    volteado = sub[::-1]                  # fila 0 = la mas cercana al carro
    k = max(1, int(cfg.get("k_transicion", 6)))
    n = volteado.shape[0]
    if n < k + 1:
        vac = np.zeros(W, np.int32)
        return vac, np.zeros(W, bool)

    c = np.cumsum(volteado, axis=0, dtype=np.int32)
    # ventana[i] = filas no-piso en [i, i+k): la primera ventana llena marca
    # el arranque del muro
    ventana = c[k - 1:].copy()
    ventana[1:] -= c[:-k]
    llena = ventana >= k
    tiene = llena.any(axis=0)
    primera = np.argmax(llena, axis=0)    # indice desde abajo
    y_cont = (y_fin - 1) - primera
    y_cont = np.where(tiene, y_cont, 0).astype(np.int32)
    return y_cont, tiene


def _contacto_negro(negro: np.ndarray, y_hor: int, y_fin: int,
                    cfg: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """Metodo viejo (pixel negro mas bajo), con el corte de horizonte añadido.
    Se conserva seleccionable para comparar en pista."""
    H, W = negro.shape[:2]
    m = negro[y_hor:y_fin] > 0
    cuenta = m.sum(axis=0)
    idx = (y_fin - 1 - y_hor) - np.argmax(m[::-1], axis=0)
    valido = cuenta >= int(cfg.get("px_min_columna", 4))
    y_cont = np.where(valido, idx + y_hor, 0).astype(np.int32)
    return y_cont, valido


# ---------------------------------------------------------------------------
def _segmentos(p: PerfilMuro, geo: Geometria, cfg: Dict[str, Any],
               lat: np.ndarray) -> Tuple[List[Segmento], List[Esquina]]:
    """Contacto -> cadenas de puntos en mm -> rectas (split & merge) ->
    fusion de colineales -> esquinas por interseccion."""
    W = p.ancho
    paso = 4                                        # una muestra cada 4 columnas
    cols = np.arange(0, W, paso)
    ok = p.valido[cols] & (p.dist_mm[cols] < p.alcance_mm * 1.5)
    tol = float(cfg.get("seg_tolerancia_mm", 45.0))
    gap_max = float(cfg.get("seg_gap_max_mm", 350.0))
    ang_fus = float(cfg.get("seg_angulo_fusion_deg", 12.0))

    # cadenas: se cortan donde falte contacto o donde haya un borde declarado
    cortes = set(b[0] // paso for b in p.bordes)
    cadenas: List[List[Tuple[float, float, int]]] = []
    actual: List[Tuple[float, float, int]] = []
    for j, c in enumerate(cols):
        if not ok[j] or j in cortes:
            if len(actual) >= 4:
                cadenas.append(actual)
            actual = []
            continue
        x = float(lat[c])
        y = float(p.dist_mm[c])
        actual.append((x, y, int(c)))
    if len(actual) >= 4:
        cadenas.append(actual)

    # split recursivo (Douglas-Peucker sobre la cadena en mm)
    brutos: List[Segmento] = []

    def dividir(pts: List[Tuple[float, float, int]]):
        if len(pts) < 3:
            if len(pts) == 2:
                brutos.append(_seg(pts))
            return
        x0, y0, _ = pts[0]
        x1, y1, _ = pts[-1]
        largo = math.hypot(x1 - x0, y1 - y0)
        if largo < 1e-6:
            return
        peor, d_peor = 0, 0.0
        for i in range(1, len(pts) - 1):
            x, y, _ = pts[i]
            d = abs((x1 - x0) * (y0 - y) - (x0 - x) * (y1 - y0)) / largo
            if d > d_peor:
                peor, d_peor = i, d
        if d_peor > tol:
            dividir(pts[:peor + 1])
            dividir(pts[peor:])
        else:
            brutos.append(_seg(pts))

    def _seg(pts) -> Segmento:
        return Segmento(x0=pts[0][0], y0=pts[0][1], x1=pts[-1][0], y1=pts[-1][1],
                        n_puntos=len(pts), col0=pts[0][2], col1=pts[-1][2])

    for cadena in cadenas:
        dividir(cadena)

    # fusion de colineales separados por un hueco (puentea el brillo)
    brutos.sort(key=lambda s: s.col0)
    fusionados: List[Segmento] = []
    for s in brutos:
        if s.largo < 30:                             # astillas: fuera
            continue
        if fusionados:
            u = fusionados[-1]
            hueco = math.hypot(s.x0 - u.x1, s.y0 - u.y1)
            d_ang = abs(_dif_ang(u.angulo, s.angulo))
            if hueco < gap_max and d_ang < ang_fus:
                fusionados[-1] = Segmento(u.x0, u.y0, s.x1, s.y1,
                                          u.n_puntos + s.n_puntos, u.col0, s.col1)
                continue
        fusionados.append(s)

    # esquinas: interseccion de segmentos consecutivos con angulo franco
    esquinas: List[Esquina] = []
    for a, b in zip(fusionados, fusionados[1:]):
        d_ang = abs(_dif_ang(a.angulo, b.angulo))
        if not (35.0 <= d_ang <= 145.0):
            continue
        pt = _interseccion(a, b)
        if pt is None:
            continue
        x, y = pt
        if y < 0 or y > p.alcance_mm * 1.5:
            continue
        # tipo: producto cruzado de las direcciones (recorriendo de izq a der
        # de la imagen). Cruz > 0 = el contorno quiebra ALEJANDOSE (rincon
        # concavo del muro externo); cruz < 0 = quiebra hacia el carro
        # (esquina saliente: el canto del muro interno).
        cruz = ((a.x1 - a.x0) * (b.y1 - b.y0) - (a.y1 - a.y0) * (b.x1 - b.x0))
        esquinas.append(Esquina(x=x, y=y, angulo=d_ang,
                                tipo="rincon" if cruz > 0 else "saliente"))
    return fusionados, esquinas


def _dif_ang(a: float, b: float) -> float:
    d = (b - a + 90.0) % 180.0 - 90.0
    return d


def _interseccion(a: Segmento, b: Segmento) -> Optional[Tuple[float, float]]:
    d1x, d1y = a.x1 - a.x0, a.y1 - a.y0
    d2x, d2y = b.x1 - b.x0, b.y1 - b.y0
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-9:
        return None
    t = ((b.x0 - a.x0) * d2y - (b.y0 - a.y0) * d2x) / den
    return a.x0 + t * d1x, a.y0 + t * d1y


# ---------------------------------------------------------------------------
class DetectorEsquinaInterna:
    """Avisa cuando una banda lateral que VENIA viendo muro deja de verlo:
    el muro interno desaparece en cada esquina. No necesita saber el sentido,
    asi que funciona desde la primera esquina.

    Media movil por lado (sin acumuladores con tope: la leccion del programa
    viejo es que un acumulador saturado tarda una eternidad en corregirse)."""

    def __init__(self):
        self.media_izq = 0.0
        self.media_der = 0.0
        self._t_aviso = 0.0

    def paso(self, p: PerfilMuro, cfg: Dict[str, Any]) -> Optional[str]:
        alfa = 0.06
        umbral = float(cfg.get("cobertura_esquina", 0.22))
        aviso: Optional[str] = None
        ahora = time.time()
        for lado in ("izq", "der"):
            media = self.media_izq if lado == "izq" else self.media_der
            cob = p.cobertura_izq if lado == "izq" else p.cobertura_der
            if media > 0.55 and cob < umbral and ahora - self._t_aviso > 1.5:
                aviso = lado
                self._t_aviso = ahora
            media = (1 - alfa) * media + alfa * cob
            if lado == "izq":
                self.media_izq = media
            else:
                self.media_der = media
        return aviso
