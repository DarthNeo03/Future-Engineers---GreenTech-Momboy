"""
geometria.py — Convierte la camara en un LIDAR 2D metrico.

============================================================================
POR QUE ESTE MODULO EXISTE
============================================================================
El perfil de contacto muro-piso que ya teniamos daba un numero 0..1 sin
unidades. Servia, pero obligaba a ajustar todos los umbrales a ojo y, peor,
hacia que "girar cuando el pasillo baja de 0.40" significase distancias
fisicas DISTINTAS segun el ancho del corredor (1000 o 600 mm, sorteados por
seccion en cada ronda).

Aqui se proyecta cada punto de contacto al plano del suelo y se obtiene
(X, Z) en milimetros respecto al carro. A partir de ese momento el perfil es
un escaneo laser de verdad: 640 rayos, ~90 grados de abanico. Todo lo que
viene despues (seguir el muro interno, detectar la esquina, decidir cuando
girar) se expresa en milimetros y deja de depender de la resolucion, del
encuadre y del ancho del pasillo.

DOS MODOS, y funciona sin calibrar:
  1. Homografia medida (config/suelo.json). Es la buena. Cuatro marcas en el
     suelo en posiciones conocidas y cv2.getPerspectiveTransform.
  2. Modelo pinhole a partir de altura, cabeceo y FOV declarados en
     robot.json. Aproximado pero utilizable desde el primer arranque, para
     no bloquear el desarrollo mientras no haya tapete delante.

============================================================================
LA REGLA DEL PIXEL MAS BAJO, Y POR QUE ES TAN ROBUSTA
============================================================================
Para cada columna se busca el pixel negro mas bajo: ahi el muro toca el
suelo. Esa regla tiene una propiedad regalada: CUALQUIER cosa que este
detras de un muro se proyecta POR ENCIMA de la base de ese muro. El publico,
las sillas, los focos y las piernas oscuras del juez quedan descartados
solos, sin ROI ni trucos. Y que el muro interno y el externo se fundan en
una sola mancha negra tampoco importa, porque solo se usa el contacto mas
cercano.

Lo que esa regla NO filtraba era una sombra en el piso blanco: una mancha
oscura plana daba un contacto falso y cercano, y el carro frenaba o esquivaba
un muro que no existe. Peor todavia: cuando el muro real NO se detectaba, el
codigo anterior marcaba esa columna como "libre = 1.0", es decir, despejado.
Tratar "no lo veo" como "esta despejado" es exactamente al reves de lo que
conviene, y es la causa mas probable de los choques contra pared que se
observaron. Aqui se arregla de dos formas:

  - CONTIGUIDAD VERTICAL: solo cuenta como muro un pixel negro que tenga
    encima una racha continua de al menos `alto_min_muro_px` pixeles negros.
    Un muro de 100 mm siempre la tiene; una sombra en el piso, no. Se hace
    con una erosion de nucleo vertical anclada abajo, que es una sola pasada
    de OpenCV.
  - DESCONOCIDO != LIBRE: las columnas sin contacto fiable se marcan como
    invalidas, no como despejadas. Quien consume el escaneo decide, y la
    politica de este proyecto es no acelerar hacia lo desconocido.
"""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

RAIZ_PROYECTO = Path(__file__).resolve().parent.parent
RUTA_SUELO = RAIZ_PROYECTO / "config" / "suelo.json"

# Mas alla de esto la proyeccion no es fiable: cerca del horizonte un pixel
# vale metros. El corredor mas largo de la pista es de 3 m, asi que recortar
# aqui no pierde nada util.
Z_MAX_MM = 3000.0
Z_MIN_MM = 40.0


# ---------------------------------------------------------------------------
@dataclass
class Escaneo:
    """Un barrido laser sintetico. Un punto por columna de la imagen."""
    x: np.ndarray            # (W,) lateral en mm, + = derecha del carro
    z: np.ndarray            # (W,) hacia adelante en mm
    valido: np.ndarray       # (W,) bool: hubo contacto fiable en esa columna
    y_contacto: np.ndarray   # (W,) fila de la imagen, para dibujar
    ancho: int
    alto: int
    # Desviacion tipica maxima admitida al ajustar una recta a una pared. Una
    # pared plana da 5-15 mm; una nube de puntos de tres superficies distintas,
    # cientos.
    residuo_max_mm: float = 45.0
    # Una pared de corredor que se sigue esta aproximadamente paralela. Mas
    # inclinada que esto es la pared DE ENFRENTE, que al cruzar el encuadre
    # aparece a los dos lados y se colaba como "muro lateral a 768 mm con -74
    # grados", mandando el control a paseo.
    ang_max_pared: float = 40.0

    @property
    def rango(self) -> np.ndarray:
        return np.hypot(self.x, self.z)

    def puntos(self) -> Tuple[np.ndarray, np.ndarray]:
        """Solo los puntos validos, como (x, z)."""
        return self.x[self.valido], self.z[self.valido]

    def cobertura(self) -> float:
        """Fraccion de columnas con contacto fiable. Baja = mala calibracion
        de color o iluminacion rara; conviene no fiarse y frenar."""
        return float(self.valido.mean()) if self.valido.size else 0.0

    # -- consultas que usa la navegacion ---------------------------------
    def frente(self, semiancho_mm: float, z_max: float = Z_MAX_MM) -> float:
        """Distancia libre por delante dentro del pasillo de las ruedas.

        Devuelve Z_MAX si no se ve nada; quien llama debe mirar tambien
        `cobertura()` para saber si ese "libre" es de fiar.
        """
        sel = self.valido & (np.abs(self.x) <= semiancho_mm) & (self.z > 0)
        if not sel.any():
            return z_max
        # percentil bajo y no el minimo: un pixel con ruido no frena el carro
        return float(np.percentile(self.z[sel], 8))

    def lateral(self, lado: int, z_desde: float = 100.0,
                z_hasta: float = 700.0) -> Optional[float]:
        """Distancia perpendicular al muro de un lado (+1 derecha, -1 izq),
        medida en una ventana de profundidad. None si no hay puntos."""
        sel = (self.valido & (np.sign(self.x) == np.sign(lado))
               & (self.z >= z_desde) & (self.z <= z_hasta))
        if sel.sum() < 5:
            return None
        return float(np.median(np.abs(self.x[sel])))

    def recta(self, lado: int, z_desde: float = 100.0,
              z_hasta: float = 900.0) -> Optional[Tuple[float, float, int]]:
        """Ajusta una recta al muro de un lado.

        Devuelve (distancia_mm, angulo_grados, n_puntos):
          distancia: perpendicular del carro a esa pared, siempre positiva.
          angulo:    pendiente en grados de x respecto a z. Positivo = la
                     pared se va hacia la derecha segun avanzas, o sea que
                     el carro apunta hacia la izquierda respecto a ella.

        Es lo que convierte el seguimiento de pared de "PD sobre la
        distancia" a "PD sobre error lateral MAS error de rumbo", que es la
        diferencia entre seguir una pared y serpentear a su lado.
        """
        sel = (self.valido & (np.sign(self.x) == np.sign(lado))
               & (self.z >= z_desde) & (self.z <= z_hasta))
        n = int(sel.sum())
        if n < 8:
            return None
        zz = self.z[sel]
        xx = self.x[sel]
        if float(zz.max() - zz.min()) < 60.0:
            return None                     # todos apelotonados: sin base
        m, c = np.polyfit(zz, xx, 1)
        dist = abs(float(c)) / math.sqrt(1.0 + float(m) * float(m))
        ang = math.degrees(math.atan(float(m)))

        # El ajuste tiene que parecerse a una PARED DE CORREDOR, no a lo que
        # sea que haya de ese lado. Mirando por encima de la esquina interna se
        # ve el muro exterior del pasillo siguiente, casi perpendicular y a un
        # metro largo: sin este filtro entra como "muro interno a 878 mm con
        # 46 grados" y el control se vuelve loco persiguiendolo.
        residuo = float(np.std(xx - (m * zz + c)))
        if residuo > float(self.residuo_max_mm):
            return None
        if abs(ang) > float(self.ang_max_pared):
            return None
        return dist, ang, n


# ---------------------------------------------------------------------------
class Suelo:
    """Proyeccion imagen -> plano del suelo."""

    def __init__(self, cfg_camara: Optional[Dict[str, Any]] = None):
        self.cfg = dict(cfg_camara or {})
        self.H: Optional[np.ndarray] = None       # homografia medida
        self.K: Optional[np.ndarray] = None       # intrinsecos
        self.dist: Optional[np.ndarray] = None
        self.origen = "modelo pinhole aproximado (sin calibrar)"
        self._cache_forma: Optional[Tuple[int, int]] = None
        self._cache_lut: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None

    # -- carga / guardado -------------------------------------------------
    @property
    def calibrado(self) -> bool:
        return self.H is not None

    def cargar(self, ruta: Optional[Path] = None) -> "Suelo":
        ruta = Path(ruta) if ruta else RUTA_SUELO
        if not ruta.exists():
            return self
        try:
            with open(ruta, "r", encoding="utf-8") as f:
                d = json.load(f)
            self.H = np.array(d["homografia"], dtype=np.float64).reshape(3, 3)
            if d.get("K"):
                self.K = np.array(d["K"], dtype=np.float64).reshape(3, 3)
                self.dist = np.array(d.get("dist", [0, 0, 0, 0, 0]), dtype=np.float64)
            self.origen = f"homografia de {ruta.name} ({d.get('fecha', '?')})"
            self._cache_forma = None
        except Exception as e:                       # nunca tumbar el arranque
            print(f"[geometria] {ruta} ilegible ({e}); sigo con el modelo aproximado")
        return self

    def guardar(self, ruta: Optional[Path] = None, notas: str = "") -> Path:
        import datetime as _dt
        ruta = Path(ruta) if ruta else RUTA_SUELO
        ruta.parent.mkdir(parents=True, exist_ok=True)
        d = {
            "fecha": _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "notas": notas,
            "homografia": np.asarray(self.H).reshape(-1).tolist(),
        }
        if self.K is not None:
            d["K"] = np.asarray(self.K).reshape(-1).tolist()
            d["dist"] = np.asarray(self.dist).reshape(-1).tolist()
        fd, tmp = tempfile.mkstemp(dir=str(ruta.parent), prefix=".suelo_", suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, ruta)
        return ruta

    # -- diagnostico ------------------------------------------------------
    def z_visible(self, x_mm: float) -> float:
        """A que distancia HACIA ADELANTE empieza a verse algo que esta a
        `x_mm` de lado. Es pura trigonometria del campo de vision y es la
        limitacion mas subestimada de una camara frontal:

            HFOV  70 grados -> un muro a 250 mm no se ve hasta z = 357 mm,
                               y uno a 750 mm no se ve hasta z = 1071 mm.
            HFOV 100 grados -> 210 mm y 629 mm respectivamente.

        Con lente estrecha el guardia contra el muro EXTERNO no puede
        dispararse nunca en campo cercano, que es justo donde importa. Por eso
        el Apendice D del reglamento pide "a wide-angle camera": no es un
        detalle de comodidad, es estructural.
        """
        hfov = math.radians(float(self.cfg.get("hfov_deg", 70.0)))
        t = math.tan(hfov / 2.0)
        return abs(x_mm) / t if t > 1e-6 else float("inf")

    def z_minimo_medible(self, alto: int, ignorar_abajo: float) -> float:
        """Distancia mas corta que la camara puede medir.

        Por debajo de esta el muro cae fuera del recorte inferior y sus
        columnas pasan a INVALIDAS. Si el umbral de parada se pone por debajo
        de este numero, la parada de seguridad no puede dispararse nunca: el
        muro no se acerca, se DESAPARECE. Es la misma trampa que "no veo =
        esta despejado", un nivel mas abajo.
        """
        y = max(0, int(alto * (1.0 - ignorar_abajo)) - 1)
        import numpy as _np
        _X, Z = self.proyectar(_np.array([self.cfg.get("ancho", 640) / 2.0]),
                               _np.array([float(y)]),
                               int(self.cfg.get("ancho", 640)), int(alto))
        z = float(Z[0])
        return z if z == z else float("nan")     # NaN si no proyecta

    def diagnostico_fov(self, objetivo_mm: float, corredor_mm: float = 1000.0) -> str:
        z_int = self.z_visible(objetivo_mm)
        z_ext = self.z_visible(corredor_mm - objetivo_mm)
        return (f"HFOV {self.cfg.get('hfov_deg', 70.0):.0f} grados: muro interno "
                f"visible desde {z_int:.0f} mm, externo desde {z_ext:.0f} mm")

    # -- proyeccion -------------------------------------------------------
    def _lut(self, ancho: int, alto: int):
        """Tabla fila -> (Z, factor lateral) para el modelo pinhole.

        Solo depende de la fila, asi que se calcula una vez por resolucion y
        luego proyectar un perfil entero son dos multiplicaciones.
        """
        if self._cache_forma == (ancho, alto) and self._cache_lut is not None:
            return self._cache_lut

        h = float(self.cfg.get("altura_mm", 200.0))
        phi = math.radians(float(self.cfg.get("cabeceo_deg", 20.0)))
        hfov = math.radians(float(self.cfg.get("hfov_deg", 70.0)))
        vfov = math.radians(float(self.cfg.get("vfov_deg", 0.0)) or 0.0)
        fx = (ancho / 2.0) / math.tan(hfov / 2.0)
        fy = ((alto / 2.0) / math.tan(vfov / 2.0)) if vfov > 0 else fx
        cx, cy = ancho / 2.0, alto / 2.0

        v = np.arange(alto, dtype=np.float64)
        a = (v - cy) / fy
        # Direccion del rayo en mundo (X der, Y arriba, Z adelante) con la
        # camara cabeceada phi hacia abajo.
        dY = -math.cos(phi) * a - math.sin(phi)
        dZ = -math.sin(phi) * a + math.cos(phi)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(dY < -1e-6, -h / dY, np.inf)     # corte con Y = 0
        z = dZ * t
        # factor lateral: X = ((u - cx) / fx) * t
        fac = t / fx
        malo = ~np.isfinite(z) | (z < Z_MIN_MM) | (z > Z_MAX_MM)
        z = np.where(malo, np.nan, z)
        fac = np.where(malo, np.nan, fac)
        self._cache_forma = (ancho, alto)
        self._cache_lut = (z, fac, np.full(alto, cx))
        return self._cache_lut

    def proyectar(self, us: np.ndarray, vs: np.ndarray,
                  ancho: int, alto: int) -> Tuple[np.ndarray, np.ndarray]:
        """Pixeles -> (X, Z) en mm. Los invalidos salen como NaN."""
        us = np.asarray(us, dtype=np.float64)
        vs = np.asarray(vs, dtype=np.float64)

        if self.H is not None:
            pts = np.stack([us, vs], axis=1).reshape(-1, 1, 2)
            if self.K is not None and self.dist is not None:
                pts = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
            sal = cv2.perspectiveTransform(pts.astype(np.float64), self.H)
            X = sal[:, 0, 0]
            Z = sal[:, 0, 1]
            malo = ~np.isfinite(Z) | (Z < Z_MIN_MM) | (Z > Z_MAX_MM)
            return np.where(malo, np.nan, X), np.where(malo, np.nan, Z)

        z_lut, fac_lut, cx_lut = self._lut(ancho, alto)
        vi = np.clip(vs.astype(np.int32), 0, alto - 1)
        Z = z_lut[vi]
        X = (us - cx_lut[vi]) * fac_lut[vi]
        return X, Z



# ---------------------------------------------------------------------------
# Calibracion automatica con tablero de ajedrez
# ---------------------------------------------------------------------------
def detectar_tablero(gris: np.ndarray, nx: int, ny: int
                     ) -> Optional[np.ndarray]:
    """Esquinas internas de un tablero, ORDENADAS: fila 0 la mas cercana al
    carro, columna 0 la de mas a la izquierda.

    Ese orden hay que imponerlo a mano. OpenCV devuelve las esquinas en un
    orden consistente con el patron, pero no sabe como esta puesto el tablero:
    girado 180 grados devuelve la misma lista al reves, y con eso la
    calibracion sale reflejada sin avisar de nada.
    """
    for detector in ("SB", "clasico"):
        if detector == "SB" and hasattr(cv2, "findChessboardCornersSB"):
            ok, esq = cv2.findChessboardCornersSB(gris, (nx, ny))
        else:
            ok, esq = cv2.findChessboardCorners(
                gris, (nx, ny),
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ok:
                cv2.cornerSubPix(
                    gris, esq, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        if ok:
            break
    else:
        return None

    e = np.asarray(esq, np.float32).reshape(ny, nx, 2)
    # Fila 0 = la mas ABAJO en la imagen = la mas cercana al carro.
    if e[0, :, 1].mean() < e[-1, :, 1].mean():
        e = e[::-1]
    # Columna 0 = la mas a la IZQUIERDA en la imagen.
    if e[:, 0, 0].mean() > e[:, -1, 0].mean():
        e = e[:, ::-1]
    return e


def mundo_tablero(nx: int, ny: int, cuadro_mm: float, z0_mm: float) -> np.ndarray:
    """Coordenadas reales de las esquinas, en el marco del carro.

    El tablero se coloca plano en el suelo, con las filas ATRAVESADAS respecto
    a la marcha y centrado en el eje longitudinal del carro. `z0_mm` es lo
    unico que hay que medir: la distancia del carro a la fila mas cercana.
    """
    xs = (np.arange(nx, dtype=np.float64) - (nx - 1) / 2.0) * cuadro_mm
    zs = z0_mm + np.arange(ny, dtype=np.float64) * cuadro_mm
    X, Z = np.meshgrid(xs, zs)
    return np.stack([X, Z], axis=-1).astype(np.float32)


def homografia_desde_tableros(vistas: Sequence[Tuple[np.ndarray, np.ndarray]]
                              ) -> Tuple[np.ndarray, float, float]:
    """Ajusta UNA homografia a todas las esquinas de todas las tomas.

    Devuelve (H, error_medio_mm, error_peor_mm).

    Con cuatro puntos la homografia pasa exactamente por ellos y el error de
    reproyeccion es CERO por construccion: no te enteras de si mediste mal.
    Con ciento y pico puntos el ajuste es por minimos cuadrados y el error que
    sale es una medida de verdad de lo bien que ha quedado.
    """
    img = np.concatenate([v[0].reshape(-1, 2) for v in vistas]).astype(np.float64)
    mun = np.concatenate([v[1].reshape(-1, 2) for v in vistas]).astype(np.float64)
    if len(img) < 8:
        raise ValueError("hacen falta al menos 8 esquinas")

    H, _mask = cv2.findHomography(img.reshape(-1, 1, 2), mun.reshape(-1, 1, 2),
                                  cv2.RANSAC, 20.0)
    if H is None:
        raise ValueError("no se pudo ajustar la homografia")

    est = cv2.perspectiveTransform(img.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(est - mun, axis=1)
    return H, float(err.mean()), float(err.max())

# ---------------------------------------------------------------------------
def contacto_muro(mascara: np.ndarray, cfg: Dict[str, Any]
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """Mascara binaria -> (fila de contacto por columna, validez).

    El contacto solo cuenta si por encima hay una racha CONTINUA de pixeles
    de muro. Es lo que separa un muro de verdad de una sombra en el piso.
    """
    H, W = mascara.shape[:2]
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.0))))
    y_fin = max(2, min(H, y_fin))

    m = (mascara[:y_fin] > 0).astype(np.uint8)

    k = int(cfg.get("alto_min_muro_px", 12))
    if k > 1:
        k = min(k, max(2, y_fin - 1))
        # Nucleo vertical anclado ABAJO: un pixel sobrevive solo si los k-1
        # de encima tambien son muro. Una pasada de OpenCV, ~0.2 ms.
        nucleo = np.ones((k, 1), np.uint8)
        m_ok = cv2.erode(m, nucleo, anchor=(0, k - 1), borderType=cv2.BORDER_CONSTANT,
                         borderValue=0)
    else:
        m_ok = m

    hay = m_ok.astype(bool)
    cuenta = hay.sum(axis=0)
    idx = (y_fin - 1) - np.argmax(hay[::-1], axis=0)
    valido = cuenta >= 1
    y_cont = np.where(valido, idx, 0).astype(np.int32)
    return y_cont, valido


def escanear(mascara: np.ndarray, suelo: Suelo, cfg: Dict[str, Any]) -> Escaneo:
    """Mascara del muro -> escaneo metrico."""
    H, W = mascara.shape[:2]
    y_cont, valido = contacto_muro(mascara, cfg)

    us = np.arange(W, dtype=np.float64)
    X, Z = suelo.proyectar(us, y_cont.astype(np.float64), W, H)
    valido = valido & np.isfinite(X) & np.isfinite(Z)
    X = np.where(valido, X, np.nan)
    Z = np.where(valido, Z, np.nan)

    e = Escaneo(x=X, z=Z, valido=valido, y_contacto=y_cont, ancho=W, alto=H)

    sua = int(cfg.get("suavizado_mm", 0) or 0)
    if sua >= 3:
        e = suavizar(e, sua)
    return e


def suavizar(e: Escaneo, k: int) -> Escaneo:
    """Media movil sobre los puntos VALIDOS.

    Ojo: esto es para el control, nunca para buscar la esquina interna. El
    salto de rango que delata al muro interno mide pocas columnas y un
    suavizado de 15 lo borra por completo; por eso la deteccion de esquina
    trabaja siempre sobre el escaneo crudo.
    """
    if k % 2 == 0:
        k += 1
    pad = k // 2
    v = e.valido.astype(np.float64)
    xs = np.where(e.valido, e.x, 0.0)
    zs = np.where(e.valido, e.z, 0.0)
    nuc = np.ones(k)
    ext = lambda a: np.pad(a, pad, mode="edge")
    peso = np.convolve(ext(v), nuc, mode="valid")
    with np.errstate(invalid="ignore", divide="ignore"):
        xm = np.convolve(ext(xs), nuc, mode="valid") / peso
        zm = np.convolve(ext(zs), nuc, mode="valid") / peso
    ok = e.valido & (peso > 0)
    return Escaneo(x=np.where(ok, xm, np.nan), z=np.where(ok, zm, np.nan),
                   valido=ok, y_contacto=e.y_contacto, ancho=e.ancho, alto=e.alto)


# ---------------------------------------------------------------------------
@dataclass
class Salto:
    """Discontinuidad de rango: la firma de una esquina CONVEXA."""
    lado: int            # -1 = a la izquierda del carro, +1 = a la derecha
    x: float             # posicion del vertice, en mm
    z: float
    magnitud: float      # cuanto salta el rango, en mm
    columna: int


def buscar_salto(e: Escaneo, cfg: Dict[str, Any]) -> Optional[Salto]:
    """Encuentra la esquina convexa del muro INTERNO.

    ============ EL DISCRIMINADOR GEOMETRICO ============
    Los dos muros son negros, miden 100 mm y no hay color que los separe.
    Pero en una esquina de la pista presentan geometrias opuestas:

      * el muro INTERNO tiene una esquina CONVEXA que apunta al carro. Justo
        pasado ese vertice el corredor sigue, asi que el rango SALTA hacia
        fuera: de pronto se ve el siguiente pasillo.
      * el muro EXTERNO tiene una esquina CONCAVA que envuelve al carro. Sus
        dos caras se juntan sin hueco y el rango es CONTINUO.

    O sea: el lado que produce el salto es el interno. El lado continuo es el
    externo. Es geometrico, sale en un solo frame y no necesita saber de
    antemano en que sentido se da la vuelta.

    Y ademas ese vertice es justo el PIVOTE del giro de 90 grados, asi que la
    misma deteccion contesta "cual es el muro interno" y "cuando girar".
    """
    umbral = float(cfg.get("salto_min_mm", 260.0))
    corrida = int(cfg.get("salto_corrida", 6))

    v = e.valido
    if v.sum() < corrida * 2 + 2:
        return None
    r = e.rango

    mejor: Optional[Salto] = None
    idx = np.flatnonzero(v)
    for a, b in zip(idx[:-1], idx[1:]):
        if b - a > 3:                      # hueco de invalidos: no es un salto
            continue
        d = float(r[b] - r[a])
        if abs(d) < umbral:
            continue
        # El lado cercano del salto tiene que ser un trozo de pared de verdad,
        # no un pixel suelto: se exige una corrida de puntos validos pegados.
        if d > 0:
            cerca, lado_col = a, slice(max(0, a - corrida), a + 1)
        else:
            cerca, lado_col = b, slice(b, min(e.ancho, b + corrida + 1))
        if int(v[lado_col].sum()) < corrida:
            continue
        if not np.isfinite(e.x[cerca]) or not np.isfinite(e.z[cerca]):
            continue
        # Escaneando de izquierda a derecha, un salto POSITIVO significa que
        # la pared cercana estaba a la izquierda: el interno es el izquierdo.
        lado = -1 if d > 0 else +1
        cand = Salto(lado=lado, x=float(e.x[cerca]), z=float(e.z[cerca]),
                     magnitud=abs(d), columna=int(cerca))
        if mejor is None or cand.magnitud > mejor.magnitud:
            mejor = cand

    if mejor is not None and mejor.z > float(cfg.get("salto_z_max_mm", 1600.0)):
        return None
    return mejor
