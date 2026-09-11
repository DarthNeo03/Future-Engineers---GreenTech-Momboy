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
    y_fin: int = 0                  # ultima fila util (encima del chasis)
    alcance_mm: float = 2500.0

    izq: float = 1.0                # media de libre en la banda izquierda
    der: float = 1.0
    pasillo_mm: float = DIST_MAX_MM  # percentil 15 de lo que hay EN el camino
    pasillo: float = 1.0            # pasillo_mm / alcance, 0..1
    min_mm: float = DIST_MAX_MM
    cobertura_izq: float = 0.0      # fraccion de la banda que ve muro a rango
    cobertura_der: float = 0.0
    hay_muro: bool = False

    # CUANTO PISO SE VE POR CADA MITAD, 0..1. No es lo mismo que izq/der:
    # aquellas miden A QUE DISTANCIA esta el muro, y en un rincon el muro esta
    # igual de cerca por los dos lados, asi que no dicen nada. Esto cuenta
    # PIXELES DE PISO, que es la pregunta util cuando el carro esta metido: por
    # donde queda sitio para salir. Se mide sobre TODA la franja util, sin
    # recortar por el horizonte, porque justo ahi es donde esta la diferencia.
    piso_izq: float = 1.0
    piso_der: float = 1.0

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
    """masks: mascaras binarias por color. Necesita 'blanco'; usa tambien
    'naranja' y 'azul' como piso (estan pintadas EN el piso)."""
    blanco = masks.get("blanco")
    if blanco is None:
        raise ValueError("hace falta la mascara 'blanco'")
    H, W = blanco.shape[:2]
    geo.redimensionar(W, H)

    piso = _mascara_piso(masks)
    y_hor = max(0, geo.fila_horizonte() + int(cfg.get("margen_horizonte_px", 4)))
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.05))))
    y_fin = max(y_hor + 2, min(H, y_fin))

    y_cont, valido = _contacto_piso(piso, y_hor, y_fin, cfg)

    p = PerfilMuro(
        y_contacto=y_cont, valido=valido,
        dist_mm=np.zeros(W, np.float32), libre=np.zeros(W, np.float32),
        alto=H, ancho=W, y_horizonte=y_hor, y_fin=y_fin,
        alcance_mm=float(cfg.get("alcance_mm", 2500.0)),
    )

    # --- piso por mitades: por donde hay sitio para salir -------------------
    # A proposito NO se recorta por el horizonte. Recortar ahi es lo que hace
    # que, con el carro metido en un rincon, las dos mitades den lo mismo y el
    # carro no sepa hacia donde salir.
    mitad = W // 2
    franja = piso[:y_fin]
    alto_franja = max(1, franja.shape[0])
    p.piso_izq = float(np.count_nonzero(franja[:, :mitad])) / (alto_franja * max(1, mitad))
    p.piso_der = float(np.count_nonzero(franja[:, mitad:])) / (alto_franja * max(1, W - mitad))

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
def _mascara_piso(masks: Dict[str, np.ndarray]) -> np.ndarray:
    """Piso = blanco + las lineas naranja/azul de las esquinas, que estan
    PINTADAS en el piso. El magenta NO: los delimitadores del estacionamiento
    2026 son muros fisicos de 10 cm y deben salir como obstaculo."""
    piso = masks["blanco"].astype(bool)
    for extra in ("naranja", "azul"):
        m = masks.get(extra)
        if m is not None:
            piso |= m.astype(bool)
    return piso


def _contacto_piso(piso: np.ndarray, y_hor: int, y_fin: int,
                   cfg: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """Primera racha de k filas no-piso subiendo desde abajo."""
    W = piso.shape[1]
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


# ---------------------------------------------------------------------------
class DetectorAtrapado:
    """EL TRIANGULO DE LA ESQUINA.

    Sintoma, sacado de una captura real: el carro entra en la curva y no sale.
    El negro cubre casi todo el cuadro y el piso que queda es un TRIANGULO con
    el vertice arriba: las dos paredes del rincon cerrando por los dos lados.
    Ni el escape ni el anti-bucle valen ahi. El escape retrocede, abre un palmo
    de pasillo y devuelve el mando al centrado, que se vuelve a meter en el
    mismo hueco, y vuelta a empezar.

    Se mide sobre el perfil que ya esta calculado, sin vision nueva:

      1. hay muro en casi todas las columnas   (no queda hueco)
      2. hasta lo mas lejos que se ve, esta cerca (percentil 80 <= dist_max_mm)
      3. el perfil tiene RELIEVE, o sea hay un vertice y no una pared plana
      4. queda poco piso
      5. y todo eso SOSTENIDO, sin que el vertice se aleje (si se aleja es que
         el carro si esta saliendo y no hay nada que rescatar)

    El relieve va en PORCENTAJE y no en milimetros. Medido en escenas
    sinteticas, la misma esquina de 90 grados da 27 % desde 700 mm y 31 % desde
    400, pero en mm baja de 147 a 89: un umbral en mm fallaria justo cuando mas
    atrapado esta el carro. Y es lo que separa "esquina" (rescate girando) de
    "pared plana de frente" (escape en reversa, que ya existe): una pared plana
    da 0 %.

    Ojo con el signo: NO se exige que las dos orillas del cuadro esten mas
    cerca que el centro. Metido de lado en el rincon, el vertice se va a un
    tercio de la imagen y un ala queda mas lejos que el centro; lo que sigue
    siendo cierto siempre es que HAY relieve.
    """

    def __init__(self):
        self.info: Dict[str, Any] = {"patron": False, "atrapado": False}
        self._t_patron = 0.0        # desde cuando se ve el patron sin cortes
        self._vertice_min = 0.0     # lo mas cerca que ha estado el vertice

    def reiniciar(self) -> None:
        self._t_patron = 0.0
        self._vertice_min = 0.0
        self.info = {"patron": False, "atrapado": False}

    # ------------------------------------------------------------------
    def paso(self, p: PerfilMuro, cfg: Dict[str, Any],
             ahora: Optional[float] = None) -> bool:
        ahora = time.time() if ahora is None else ahora
        W = int(p.ancho)
        if W < 16 or p.y_fin <= p.y_horizonte:
            self.reiniciar()
            return False

        # --- 4. cuanto piso queda (el area del triangulo blanco) ----------
        banda = float(p.y_fin - p.y_horizonte)
        # columna sin contacto = piso hasta arriba del todo
        altura = np.where(p.valido,
                          np.clip(p.y_fin - p.y_contacto, 0, banda), banda)
        piso_pct = 100.0 * float(altura.sum()) / (banda * W)

        # --- 1. cobertura y 2. nada lejos ---------------------------------
        cobertura = float(np.count_nonzero(p.valido)) / W
        ### recortado al alcance: una columna sin contacto vale "lejos", pero
        ### 6000 mm desbarataria las medias y los percentiles
        d = _media_movil(np.minimum(p.dist_mm, p.alcance_mm).astype(np.float32), 11)
        p80 = float(np.percentile(d, 80))

        # --- 3. relieve: hay un vertice, o es una pared plana? ------------
        # Por BANDAS y con percentil, nunca con el maximo: una sola columna sin
        # contacto (pasa siempre que una pared se sale del cuadro) se llevaria
        # el argmax al canto y la medida dejaria de significar nada.
        n_borde = max(2, int(W * float(cfg.get("borde_frac", 0.25))))
        n_borde = min(n_borde, W // 3)
        d_izq = float(np.percentile(d[:n_borde], 60))
        d_der = float(np.percentile(d[-n_borde:], 60))
        centro = d[n_borde:W - n_borde]
        d_centro = float(np.percentile(centro, 60))
        vertice = max(d_izq, d_der, d_centro)
        relieve_pct = 100.0 * (vertice - min(d_izq, d_der, d_centro)) / max(vertice, 1.0)
        i_pico = n_borde + int(np.argmax(centro))

        # --- veredicto de este frame --------------------------------------
        fallo = ""
        if cobertura < float(cfg.get("cobertura_min", 0.90)):
            fallo = "hay hueco sin muro"
        elif p80 > float(cfg.get("dist_max_mm", 800.0)):
            fallo = "queda salida (algo lejos)"
        elif relieve_pct < float(cfg.get("relieve_min_pct", 20.0)):
            fallo = "perfil llano: es pared, no esquina"
        elif piso_pct > float(cfg.get("piso_max_pct", 75.0)):
            fallo = "todavia se ve mucho piso"
        patron = not fallo

        # --- 5. sostenido y SIN avanzar -----------------------------------
        if not patron:
            self._t_patron = 0.0
        elif self._t_patron == 0.0:
            self._t_patron = ahora
            self._vertice_min = vertice
        else:
            self._vertice_min = min(self._vertice_min, vertice)
            if vertice - self._vertice_min > float(cfg.get("avance_max_mm", 250.0)):
                ### el vertice se aleja: el carro SI esta saliendo de la
                ### curva. No esta atrapado; se reinicia la cuenta.
                self._t_patron = ahora
                self._vertice_min = vertice
                fallo = "el vertice se aleja: esta saliendo"

        sostenido_ms = 0.0 if self._t_patron == 0.0 else (ahora - self._t_patron) * 1000.0
        atrapado = bool(patron and
                        sostenido_ms >= float(cfg.get("confirmar_ms", 1200)))

        self.info = {
            "patron": patron,
            "atrapado": atrapado,
            "piso_pct": round(piso_pct, 1),
            "cobertura": round(cobertura, 2),
            "lejos_mm": round(p80),
            "vertice_mm": round(vertice),
            "vertice_col": i_pico,
            "relieve_pct": round(relieve_pct),
            "sostenido_ms": round(sostenido_ms),
            "fallo": fallo,
        }
        return atrapado
