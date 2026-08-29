"""
navegacion.py — No chocar con los muros, y saber donde estas.

============================================================================
LAS OPCIONES QUE HAY, Y POR QUE ESTAN IMPLEMENTADAS ESTAS
============================================================================

1. AREA DE NEGRO EN DOS VENTANAS.  Contar pixeles negros a izquierda y derecha
   y girar hacia el que tenga menos. Cuatro lineas, pero el muro del fondo
   (lejos, inofensivo) pesa igual que el de al lado (cerca, peligroso).
   Descartada: no distingue distancia.

2. PERFIL DE CONTACTO MURO-PISO, COLUMNA POR COLUMNA.  <-- BASE DE TODO
   Para cada columna se busca el pixel negro MAS BAJO: ahi el muro toca el
   suelo. Cuanto mas abajo, mas cerca. Sale un perfil de distancia de ancho
   completo, como un LIDAR pobre. Las tres estrategias de abajo leen de aqui.

3. CENTRADO POR ESPACIO LIBRE (estrategia "centrado").
   Compara el espacio libre de la banda izquierda y la derecha y gira hacia la
   despejada con un PD. Es lo mas tolerante a una calibracion imperfecta.

4. SEGUIR UNA PARED A DISTANCIA FIJA (estrategia "pared").
   Trayectorias limpias y repetibles, pero por si sola gira tarde en las
   esquinas. Por eso ahora va acompanada del disparador de esquina interna.

5. HUECO PASABLE, CONTANDO EL ANCHO DE LAS RUEDAS (estrategia "hueco").
   Busca los tramos del perfil por donde el carro CABE de verdad a la
   distancia a la que esta el obstaculo, y apunta al mejor. Es la que
   propusiste tu y la que sirve para el reto de obstaculos.

6. GIROSCOPIO COMO RUMBO.  La pista es un cuadrado: los giros son de 90
   grados. La camara decide CUANDO girar, el giroscopio CUANTO.

7. ESQUINA DEL MURO INTERNO.  El muro externo siempre se ve; el interno SE
   ACABA en cada esquina. Ese final es un escalon en el perfil, y llega antes
   de que el muro de enfrente este encima: es el aviso de giro que faltaba.

Las estrategias 3, 4 y 5 se pueden MEZCLAR con pesos desde la interfaz. Lo de
arriba (anticipacion, esquina, escape, seguridad) corre igual con cualquiera.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

# Estados de la maquina
RECTO = "recto"
PRE_GIRO = "pre_giro"
GIRO = "giro"
ESCAPE = "escape"
MEDIA_VUELTA = "media_vuelta"
FIN = "fin"

IZQ, DER = -1, 1


# ===========================================================================
# Lectura del muro
# ===========================================================================
@dataclass
class Borde:
    """Un escalon del perfil: donde el muro se acaba de golpe."""
    x: int
    salto: float          # + = de cerca a lejos segun crece x
    lado: int             # IZQ o DER respecto al centro de la imagen

    @property
    def cerca_a_lejos(self) -> bool:
        return self.salto > 0


@dataclass
class Hueco:
    """Un tramo por el que quizas cabe el carro."""
    x0: int
    x1: int
    libre_min: float
    libre_medio: float
    libre_fondo: float        # lo despejado que esta el FINAL del hueco
    ancho_px: int
    ancho_necesario: float
    puntuacion: float = 0.0

    @property
    def centro(self) -> int:
        return (self.x0 + self.x1) // 2

    @property
    def margen(self) -> float:
        return self.ancho_px / max(1.0, self.ancho_necesario)

    @property
    def pasable(self) -> bool:
        return self.ancho_px >= self.ancho_necesario


@dataclass
class PerfilMuro:
    libre: np.ndarray
    y_contacto: np.ndarray
    alto: int
    ancho: int
    izq: float = 0.0
    der: float = 0.0
    pasillo: float = 0.0
    pasillo_medio: float = 0.0
    min_global: float = 0.0
    hay_muro: bool = False
    # Fraccion de columnas de cada banda lateral que VEN muro. Es distinto de
    # 'izq'/'der', que miden a que distancia esta: aqui solo importa si HAY o
    # no. Es la senal que distingue el muro externo (siempre presente) del
    # interno (desaparece en cada esquina).
    cobertura_izq: float = 0.0
    cobertura_der: float = 0.0
    bordes: List[Borde] = field(default_factory=list)
    huecos: List[Hueco] = field(default_factory=list)

    def banda(self, lado: int) -> float:
        return self.der if lado == DER else self.izq

    def cobertura(self, lado: int) -> float:
        return self.cobertura_der if lado == DER else self.cobertura_izq


def _media_movil(v: np.ndarray, k: int) -> np.ndarray:
    if k < 3:
        return v
    if k % 2 == 0:
        k += 1
    pad = k // 2
    ext = np.pad(v, pad, mode="edge")
    return np.convolve(ext, np.ones(k, dtype=np.float32) / k, mode="valid")


def perfil_desde_mascara(mascara: np.ndarray, cfg: Dict[str, Any]) -> PerfilMuro:
    """Mascara binaria del muro -> perfil de espacio libre.

    El pixel mas bajo por columna es un argmax sobre la mascara del reves:
    una pasada vectorizada, ~0.3 ms en 640x480.
    """
    H, W = mascara.shape[:2]
    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.0))))
    y_fin = max(1, min(H, y_fin))

    m = mascara[:y_fin] > 0
    cuenta = m.sum(axis=0)
    idx = (y_fin - 1) - np.argmax(m[::-1], axis=0)
    sin_muro = cuenta < int(cfg.get("px_min_columna", 1))
    y_cont = np.where(sin_muro, 0, idx).astype(np.int32)

    libre = np.where(sin_muro, 1.0, (H - y_cont) / float(H)).astype(np.float32)
    libre = np.clip(_media_movil(libre, int(cfg.get("suavizado", 0))), 0.0, 1.0)

    p = PerfilMuro(libre=libre, y_contacto=y_cont, alto=H, ancho=W)

    n_lat = max(1, int(W * float(cfg.get("banda_lateral", 0.28))))
    p.izq = float(libre[:n_lat].mean())
    p.der = float(libre[-n_lat:].mean())
    con_muro = ~sin_muro
    p.cobertura_izq = float(con_muro[:n_lat].mean())
    p.cobertura_der = float(con_muro[-n_lat:].mean())

    a = max(0, min(W - 1, int(W * float(cfg.get("ruedas_izq", 0.32)))))
    b = max(a + 1, min(W, int(W * float(cfg.get("ruedas_der", 0.68)))))
    corredor = libre[a:b]
    p.pasillo = float(np.percentile(corredor, 15))
    p.pasillo_medio = float(corredor.mean())
    p.min_global = float(libre.min())
    p.hay_muro = bool((~sin_muro).any())

    p.bordes = detectar_bordes(p, cfg)
    p.huecos = detectar_huecos(p, cfg)
    return p


def detectar_bordes(p: PerfilMuro, cfg: Dict[str, Any]) -> List[Borde]:
    """Escalones del perfil: donde el muro se acaba de golpe.

    Esto es lo que distingue el muro INTERNO del EXTERNO sin medir nada:
      * el externo dobla en la esquina, asi que su base cambia de pendiente
        pero sigue ahi -> el perfil varia suave;
      * el interno SE ACABA -> detras hay piso o el muro de enfrente, mucho
        mas lejos, y el perfil da un salto brusco.
    El salto avisa de la esquina bastante antes de que el muro de enfrente
    este encima, que era justo el problema de girar tarde.
    """
    salto_min = float(cfg.get("salto_min", 0.12))
    k = max(2, int(cfg.get("ventana_salto", 6)))
    v = p.libre
    if v.size < 2 * k + 2:
        return []
    d = v[k:] - v[:-k]                       # diferencia a k columnas
    bordes: List[Borde] = []
    umbral = salto_min
    # solo el maximo de cada racha, para no sacar 30 bordes del mismo escalon
    signo = np.sign(d) * (np.abs(d) >= umbral)
    i = 0
    while i < signo.size:
        if signo[i] == 0:
            i += 1
            continue
        j = i
        while j + 1 < signo.size and signo[j + 1] == signo[i]:
            j += 1
        tramo = d[i:j + 1]
        pos = i + int(np.argmax(np.abs(tramo)))
        x = pos + k // 2
        bordes.append(Borde(x=int(x), salto=float(d[pos]),
                            lado=DER if x > p.ancho / 2 else IZQ))
        i = j + 1
    bordes.sort(key=lambda b: -abs(b.salto))
    return bordes[:6]


def ancho_carro_px(y: float, cfg: Dict[str, Any], H: int, W: int) -> float:
    """Cuantos pixeles de ancho ocupa el carro a la altura de fila 'y'.

    Perspectiva basica de plano de suelo: un ancho real fijo se ve proporcional
    a (y - horizonte). Se ancla en las lineas de las ruedas, que ya calibraste
    a mano en la fila de abajo del todo, asi que no hace falta patron de
    calibracion ni conocer la focal.
    """
    y_h = float(cfg.get("y_horizonte", 0.35)) * H
    base = (float(cfg.get("ruedas_der", 0.68)) -
            float(cfg.get("ruedas_izq", 0.32))) * W
    denom = max(1.0, H - y_h)
    return max(4.0, base * max(0.0, y - y_h) / denom)


def detectar_huecos(p: PerfilMuro, cfg: Dict[str, Any]) -> List[Hueco]:
    """Tramos consecutivos despejados, con su ancho comparado con el del carro.

    Esta es la idea que propusiste: no basta con que se vea hueco, tiene que
    CABER el carro contando por donde pasan las ruedas, y a la distancia a la
    que esta el obstaculo que lo delimita (un hueco lejano necesita menos
    pixeles que el mismo hueco cerca).
    """
    umbral = float(cfg.get("umbral_hueco", 0.45))
    v = p.libre
    W, H = p.ancho, p.alto
    dentro = v >= umbral
    huecos: List[Hueco] = []
    i = 0
    while i < W:
        if not dentro[i]:
            i += 1
            continue
        j = i
        while j + 1 < W and dentro[j + 1]:
            j += 1
        tramo = v[i:j + 1]
        # obstaculo que delimita el hueco: el mas CERCANO de los dos lados
        y_izq = p.y_contacto[max(0, i - 1)]
        y_der = p.y_contacto[min(W - 1, j + 1)]
        y_borde = float(max(y_izq, y_der))
        if y_borde <= 0:
            y_borde = H * 0.75          # sin obstaculo: exigimos como si estuviera cerca
        necesario = ancho_carro_px(y_borde, cfg, H, W) * float(cfg.get("margen_hueco", 1.15))
        n = tramo.size
        tercio = max(1, n // 3)
        huecos.append(Hueco(
            x0=i, x1=j,
            libre_min=float(tramo.min()),
            libre_medio=float(tramo.mean()),
            # "el siguiente obstaculo": lo despejado que esta la parte del
            # hueco por la que el carro saldria. Si detras hay otra cosa, esto
            # baja y el hueco pierde puntos aunque su boca sea ancha.
            libre_fondo=float(np.percentile(tramo[tercio:n - tercio] if n > 3 * tercio
                                            else tramo, 30)),
            ancho_px=int(n),
            ancho_necesario=float(necesario),
        ))
        i = j + 1

    for h in huecos:
        alineacion = 1.0 - abs(h.centro - W / 2.0) / (W / 2.0)
        h.puntuacion = (
            float(cfg.get("peso_margen", 1.0)) * min(2.0, h.margen)
            + float(cfg.get("peso_profundidad", 1.2)) * h.libre_medio
            + float(cfg.get("peso_siguiente", 1.0)) * h.libre_fondo
            + float(cfg.get("peso_alineacion", 0.6)) * alineacion
        )
        if not h.pasable:
            h.puntuacion -= 5.0          # no cabe: solo se elige si no hay otro
    huecos.sort(key=lambda h: -h.puntuacion)
    return huecos


# ===========================================================================
# Autocalibracion del ancho de carril
# ===========================================================================
class CalibradorCarril:
    """Mide el ancho del carril con la propia camara y ajusta los umbrales.

    En competencia abierta el carril cambia de una pista a otra, asi que
    umbrales fijos no valen. En recta, con poca direccion, la suma de espacio
    libre de las dos bandas laterales es practicamente constante y no depende
    de por donde vayas dentro del carril: esa suma ES el ancho del carril en
    las unidades del perfil. Con eso se escalan los umbrales de frenar, girar
    y parar, y todo se adapta solo.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.muestras: List[float] = []
        self.ancho: float = 0.0
        self.listo = False

    def observar(self, p: PerfilMuro, estado: str, direccion: float) -> None:
        if estado != RECTO or abs(direccion) > 22:
            return
        if p.pasillo < 0.5:                 # hay algo delante: no es recta limpia
            return
        self.muestras.append(p.izq + p.der)
        if len(self.muestras) > 240:
            del self.muestras[:80]
        if len(self.muestras) >= 25:
            self.ancho = float(np.median(self.muestras[-120:]))
            self.listo = True

    def umbrales(self) -> Dict[str, float]:
        """Devuelve los umbrales derivados, ya acotados a valores sensatos."""
        if not self.listo:
            return {}
        a = self.ancho
        return {
            "parar_bajo": float(np.clip(0.34 * a, 0.10, 0.42)),
            "girar_bajo": float(np.clip(0.58 * a, 0.22, 0.70)),
            "frenar_bajo": float(np.clip(0.82 * a, 0.32, 0.92)),
        }

    def estado(self) -> Dict[str, Any]:
        return {"listo": self.listo, "ancho": round(self.ancho, 3),
                "muestras": len(self.muestras)}


# ===========================================================================
# Sentido de la vuelta
# ===========================================================================
class DetectorParedes:
    """Cual de las dos paredes es la EXTERNA, y por tanto en que sentido vamos.

    QUE FALLABA ANTES: se acumulaban votos con un tope de +-6 y se votaba en
    CADA frame. El acumulador se saturaba en un segundo y a partir de ahi hacian
    falta 48 frames en contra para moverlo, asi que se quedaba clavado en un
    valor (siempre "horario") y despues de la media vuelta volvia al de antes en
    cuanto empezaba a mirar paredes otra vez. Eso es lo que deshacia la media
    vuelta: el carro se reorientaba al sentido viejo.
    ------------------------------------------------------------------------

    Ahora la senal principal es CONTINUA y se puede corregir sola:

        el muro EXTERNO se ve practicamente siempre;
        el muro INTERNO desaparece en cada esquina.

    Se lleva una media movil de "esta banda ve muro" para cada lado. El lado con
    presencia claramente mayor es el externo. Como es una media movil, si el
    carro se da la vuelta las dos medias se cruzan solas en pocos segundos, y
    ademas al hacer la media vuelta se intercambian a mano y se bloquea el
    estimador un rato para que no dude mientras las paredes cambian de sitio.

    Convencion de la etiqueta (la tuya): pared externa a la IZQUIERDA =
    antihorario; pared externa a la DERECHA = horario. La etiqueta solo es un
    nombre; lo que de verdad manda el comportamiento es 'lado_externo'.

    Y siempre se puede FORZAR desde la interfaz, por si en competencia la
    deteccion automatica se pone tonta.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self.pres_izq = 0.5
        self.pres_der = 0.5
        self.muestras = 0
        self.votos = 0.0            # eventos discretos (giros, lineas), decaen
        self.forzado = 0            # 0 = automatico, +1 horario, -1 antihorario
        self.bloqueado_hasta = 0.0
        self._externo = 0
        self.fuentes: Dict[str, Any] = {}

    # -- entrada continua -------------------------------------------------
    def observar(self, p: PerfilMuro, ahora: Optional[float] = None) -> None:
        ahora = ahora if ahora is not None else time.time()
        if ahora < self.bloqueado_hasta:
            return
        alfa = float(self.cfg.get("alfa_presencia", 0.03))
        umbral = float(self.cfg.get("cobertura_muro", 0.45))
        self.pres_izq += alfa * ((1.0 if p.cobertura_izq >= umbral else 0.0) - self.pres_izq)
        self.pres_der += alfa * ((1.0 if p.cobertura_der >= umbral else 0.0) - self.pres_der)
        self.muestras += 1
        self.votos *= float(self.cfg.get("decaimiento_votos", 0.995))
        self._resolver()

    # -- eventos discretos ------------------------------------------------
    def voto_giro(self, lado_giro: int) -> None:
        """Se gira hacia el INTERIOR de la pista, asi que el externo es el otro."""
        if lado_giro:
            self._votar("giro", -lado_giro, 1.5)

    def voto_muro_desaparecido(self, lado: int) -> None:
        """La pared que se acaba es la interna: la externa es la contraria."""
        if lado:
            self._votar("desaparece", -lado, 1.0)

    def voto_lineas(self, primera: str, segunda: str) -> None:
        orden = [str(x) for x in self.cfg.get("orden_horario", ["naranja", "azul"])]
        if [primera, segunda] == orden:
            self._votar("lineas", DER, 2.0)      # horario -> externa a la derecha
        elif [primera, segunda] == orden[::-1]:
            self._votar("lineas", IZQ, 2.0)

    def _votar(self, fuente: str, lado_externo: int, peso: float) -> None:
        if time.time() < self.bloqueado_hasta:
            return
        self.fuentes[fuente] = "der" if lado_externo > 0 else "izq"
        self.votos = max(-4.0, min(4.0, self.votos + lado_externo * peso))
        self._resolver()

    # -- resolucion -------------------------------------------------------
    def _resolver(self) -> None:
        if self.forzado:
            self._externo = DER if self.forzado > 0 else IZQ
            return
        d = self.pres_der - self.pres_izq
        margen = float(self.cfg.get("margen_presencia", 0.15))
        minimo = int(self.cfg.get("min_muestras_presencia", 60))

        ext = 0
        if self.muestras >= minimo and abs(d) >= margen:
            ext = DER if d > 0 else IZQ
        if ext == 0 and abs(self.votos) >= 1.5:
            ext = DER if self.votos > 0 else IZQ
        elif ext != 0 and abs(self.votos) >= 3.0:
            voto_ext = DER if self.votos > 0 else IZQ
            if voto_ext != ext and abs(d) < margen * 2:
                ext = voto_ext          # eventos muy claros y presencia dudosa
        self._externo = ext

    # -- cambios de estado ------------------------------------------------
    def invertir(self, ahora: Optional[float] = None) -> None:
        """Tras la media vuelta las dos paredes cambian de sitio.

        Se intercambian las presencias (que es lo que fisicamente pasa) y se
        BLOQUEA el estimador unos segundos: mientras el carro se reencuadra ve
        cosas raras, y sin el bloqueo volvia a decidir el sentido viejo y se
        ponia a deshacer la media vuelta.
        """
        ahora = ahora if ahora is not None else time.time()
        self.pres_izq, self.pres_der = self.pres_der, self.pres_izq
        self.votos = -self.votos
        self._externo = -self._externo
        self.fuentes = {k: ("izq" if v == "der" else "der") for k, v in self.fuentes.items()}
        self.bloqueado_hasta = ahora + float(self.cfg.get("bloqueo_sentido_ms", 4000)) / 1000.0
        if self.forzado:
            self.forzado = -self.forzado

    def forzar(self, sentido: int) -> None:
        """sentido: +1 horario, -1 antihorario, 0 automatico."""
        self.forzado = int(max(-1, min(1, sentido)))
        self._resolver()

    # -- salidas ----------------------------------------------------------
    @property
    def lado_externo(self) -> int:
        return self._externo

    @property
    def lado_interno(self) -> int:
        return -self._externo

    @property
    def sentido(self) -> int:
        """Etiqueta: externa a la derecha = horario (+1)."""
        return self._externo

    def estado(self) -> Dict[str, Any]:
        nombre = {1: "horario", -1: "antihorario", 0: "sin determinar"}[self._externo]
        return {
            "sentido": self._externo,
            "nombre": nombre,
            "externa": {1: "der", -1: "izq", 0: "?"}[self._externo],
            "interna": {1: "izq", -1: "der", 0: "?"}[self._externo],
            "presencia": [round(self.pres_izq, 2), round(self.pres_der, 2)],
            "muestras": self.muestras,
            "votos": round(self.votos, 2),
            "forzado": self.forzado,
            "bloqueado": time.time() < self.bloqueado_hasta,
            "fuentes": dict(self.fuentes),
        }


# Nombre viejo, para no romper nada que lo importe
EstimadorSentido = DetectorParedes


class DetectorEsquinaInterna:
    """Avisa cuando una pared que SE VEIA deja de verse.

    Es el disparo de esquina que mejor funciona, y lo mejor es que NO necesita
    saber en que sentido vamos: si una banda tenia muro y de golpe no lo tiene,
    ahi se acabo el muro interno y esa es la direccion del giro. Con eso el
    carro gira en el momento correcto desde la primera esquina, sin esperar a
    que el estimador de paredes se decida.
    """

    def __init__(self, cfg: Dict[str, Any]):
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self.tenia = {IZQ: False, DER: False}
        self.cuenta_sin = {IZQ: 0, DER: 0}
        self.cuenta_con = {IZQ: 0, DER: 0}
        self.ultimo_evento = 0.0

    def paso(self, p: PerfilMuro, ahora: Optional[float] = None) -> int:
        """Devuelve IZQ/DER si acaba de desaparecer ese muro, o 0."""
        ahora = ahora if ahora is not None else time.time()
        alto = float(self.cfg.get("cobertura_alta", 0.55))
        bajo = float(self.cfg.get("cobertura_baja", 0.22))
        lejos = float(self.cfg.get("interno_lejos", 0.75))
        cerca = float(self.cfg.get("interno_cerca", 0.62))
        conf = int(self.cfg.get("frames_confirmar_esquina", 3))
        estable = int(self.cfg.get("frames_confirmar_muro", 4))

        evento = 0
        for lado in (IZQ, DER):
            # "Ya no hay muro por aqui" es cualquiera de las dos cosas: que la
            # banda se quede sin pixeles de muro (el interno se sale del cuadro)
            # o que lo que se ve ahi este tan lejos que ya no es esta pared,
            # sino la de enfrente. En una pista cuadrada pasan las dos segun el
            # angulo con el que llegues a la esquina.
            c = p.cobertura(lado)
            if p.banda(lado) >= lejos:
                c = 0.0
            elif p.banda(lado) <= cerca and c >= alto:
                c = 1.0
            if c >= alto:
                self.cuenta_con[lado] += 1
                self.cuenta_sin[lado] = 0
                if self.cuenta_con[lado] >= estable:
                    self.tenia[lado] = True
            elif c <= bajo:
                self.cuenta_sin[lado] += 1
                self.cuenta_con[lado] = 0
                if self.tenia[lado] and self.cuenta_sin[lado] >= conf:
                    self.tenia[lado] = False
                    self.ultimo_evento = ahora
                    evento = lado
            else:
                self.cuenta_con[lado] = 0
                self.cuenta_sin[lado] = 0
        return evento

    def estado(self) -> Dict[str, Any]:
        return {"izq": self.tenia[IZQ], "der": self.tenia[DER]}


# ===========================================================================
@dataclass
class Decision:
    vel: int = 0
    direccion: int = 0
    estado: str = RECTO
    motivo: str = ""
    metricas: Dict[str, Any] = field(default_factory=dict)


class _PD:
    def __init__(self):
        self.prev: Optional[float] = None
        self.t_prev: float = 0.0

    def paso(self, err: float, kp: float, kd: float, ahora: float) -> float:
        d = 0.0
        if self.prev is not None:
            dt = max(1e-3, ahora - self.t_prev)
            d = (err - self.prev) / dt
        self.prev = err
        self.t_prev = ahora
        return kp * err + kd * d * 0.1

    def reiniciar(self):
        self.prev = None


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _norm_angulo(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _dif_angulo(objetivo: float, actual: float) -> float:
    return _norm_angulo(objetivo - actual)


# ===========================================================================
class Navegador:
    def __init__(self, cfg_nav: Dict[str, Any], cfg_lim: Dict[str, Any],
                 cfg_vueltas: Optional[Dict[str, Any]] = None):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.cfg_vueltas = cfg_vueltas or {}

        self.pd_centrado = _PD()
        self.pd_pared = _PD()
        self.pd_hueco = _PD()
        self.carril = CalibradorCarril(cfg_nav)
        self.paredes = DetectorParedes(cfg_nav)
        self.sentido = self.paredes          # alias historico
        self.esquinas = DetectorEsquinaInterna(cfg_nav)
        self._esquina_pendiente = 0
        self._t_esquina_pendiente = 0.0

        self.estado = RECTO
        self.t_estado = time.time()
        self.lado_giro = 1
        self.rumbo_objetivo: Optional[float] = None
        self.ultimo = Decision()

        # anticipacion
        self._libre_prev: Optional[float] = None
        self._t_prev = time.time()
        self._v_cierre = 0.0
        self.ttc = 99.0

        # escape
        self._escape_modo = "atras"
        self._escape_ref = 0.0
        self._escape_t = 0.0
        self._escape_mejor = 0.0
        self._escape_cambios = 0
        self._escape_lado = 1
        self._escape_objetivo = 0.0
        self._escape_compromiso = 0.0

        # media vuelta
        self.media_vuelta_pedida = False
        self._mv_fase = 0
        self._mv_t = 0.0
        self._mv_yaw0: Optional[float] = None
        self._mv_yaw_prev: Optional[float] = None
        self._mv_girado = 0.0
        self._mv_lado = 1
        self.media_vuelta_hecha = False
        self.terminado = False
        self.giros = 0
        self.giro_nuevo = False
        self.ultimo_lado_giro = 0

    # -- utilidades -------------------------------------------------------
    def reiniciar(self, todo: bool = False):
        self.pd_centrado.reiniciar()
        self.pd_pared.reiniciar()
        self.pd_hueco.reiniciar()
        self._cambiar(RECTO)
        self.rumbo_objetivo = None
        self._libre_prev = None
        self._escape_cambios = 0
        self.giro_nuevo = False
        self._esquina_pendiente = 0
        self.esquinas.reiniciar()
        if todo:
            self.giros = 0
            self.carril = CalibradorCarril(self.cfg)
            self.paredes.reiniciar()
            self.sentido = self.paredes
            self.media_vuelta_pedida = False
            self.media_vuelta_hecha = False
            self.terminado = False
            self._mv_fase = 0

    def _cambiar(self, estado: str):
        if estado != self.estado:
            # Terminar un giro es "he doblado una esquina": el contador de
            # vueltas lo usa como tercera fuente, junto a las dos lineas.
            if self.estado == GIRO and estado == RECTO:
                self.giros += 1
                self.giro_nuevo = True
                self.ultimo_lado_giro = self.lado_giro
            self.estado = estado
            self.t_estado = time.time()

    def tomar_giro(self) -> Optional[int]:
        """Devuelve el lado del giro recien terminado (y lo consume)."""
        if not self.giro_nuevo:
            return None
        self.giro_nuevo = False
        return self.ultimo_lado_giro

    def _ms_en_estado(self, ahora: float) -> float:
        return (ahora - self.t_estado) * 1000.0

    def _umbral(self, nombre: str) -> float:
        """Umbral efectivo: el autocalibrado manda si esta listo."""
        if bool(self.cfg.get("autocalibrar_carril", True)):
            u = self.carril.umbrales()
            if nombre in u:
                return u[nombre]
        return float(self.cfg.get(nombre, 0.4))

    def pedir_media_vuelta(self):
        self.media_vuelta_pedida = True

    # -- estrategias ------------------------------------------------------
    def _dir_centrado(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        err = p.der - p.izq
        d = self.pd_centrado.paso(err, float(self.cfg.get("kp", 95.0)),
                                  float(self.cfg.get("kd", 22.0)), ahora)
        return d, f"cen{err:+.2f}"

    def _dir_pared(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        lado_cfg = str(self.cfg.get("lado_pared", "auto")).lower()
        if lado_cfg.startswith("a") and self.sentido.sentido != 0:
            # "auto": seguir la pared EXTERNA, que es la contraria a la interna
            lado = IZQ if self.sentido.lado_interno == DER else DER
        else:
            lado = IZQ if lado_cfg.startswith("i") else DER
        d_actual = p.banda(lado)
        objetivo = float(self.cfg.get("pared_objetivo", 0.45))
        err = d_actual - objetivo
        salida = self.pd_pared.paso(err, float(self.cfg.get("kp_pared", 130.0)),
                                    float(self.cfg.get("kd_pared", 28.0)), ahora)
        signo = -1.0 if lado == IZQ else 1.0
        return signo * salida, f"par{'I' if lado == IZQ else 'D'}{err:+.2f}"

    def _dir_hueco(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        if not p.huecos:
            return 0.0, "hueco: ninguno"
        h = p.huecos[0]
        objetivo = h.centro
        # Si el hueco es mas ancho de lo necesario, se apunta al punto del hueco
        # mas cercano al centro de la imagen que aun deja margen a los dos
        # lados: se pasa recto en vez de dar un volantazo hacia el centro geometrico.
        margen_px = (h.ancho_px - h.ancho_necesario) / 2.0
        if margen_px > 2:
            objetivo = int(_lim(p.ancho / 2.0, h.x0 + h.ancho_necesario / 2.0,
                                h.x1 - h.ancho_necesario / 2.0))
        err = (objetivo - p.ancho / 2.0) / (p.ancho / 2.0)
        d = self.pd_hueco.paso(err, float(self.cfg.get("kp_hueco", 105.0)),
                               float(self.cfg.get("kd_hueco", 20.0)), ahora)
        return d, f"hue x{objetivo} m{h.margen:.2f}"

    def _mezcla(self, p: PerfilMuro, ahora: float) -> Tuple[float, str, Dict[str, float]]:
        """Combina las estrategias activas con sus pesos."""
        pesos = dict(self.cfg.get("mezcla", {}) or {})
        if not pesos or sum(abs(v) for v in pesos.values()) <= 0:
            pesos = {str(self.cfg.get("estrategia", "centrado")): 1.0}
        total = sum(max(0.0, float(v)) for v in pesos.values())
        if total <= 0:
            pesos, total = {"centrado": 1.0}, 1.0

        aportes: Dict[str, float] = {}
        motivos = []
        suma = 0.0
        for nombre, peso in pesos.items():
            peso = max(0.0, float(peso))
            if peso <= 0:
                continue
            if nombre.startswith("par"):
                d, m = self._dir_pared(p, ahora)
            elif nombre.startswith("hue"):
                d, m = self._dir_hueco(p, ahora)
            else:
                d, m = self._dir_centrado(p, ahora)
            aportes[nombre] = round(d, 1)
            motivos.append(m)
            suma += peso * d
        return suma / total, " ".join(motivos), aportes

    # -- anticipacion -----------------------------------------------------
    def _anticipar(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        """Techo de velocidad segun lo que queda por delante Y lo rapido que se
        esta cerrando.

        El problema de la inercia no se arregla solo con la distancia: cuando el
        umbral salta ya llevas la velocidad encima. Aqui se mide tambien la
        VELOCIDAD DE CIERRE del pasillo y se calcula el tiempo que falta para
        llegar al muro. Si ese tiempo baja del minimo, se frena aunque la
        distancia todavia parezca aceptable.
        """
        dt = max(1e-3, ahora - self._t_prev)
        self._t_prev = ahora
        libre = p.pasillo
        if self._libre_prev is not None:
            cierre = (self._libre_prev - libre) / dt         # >0 = acercandose
            self._v_cierre = 0.7 * self._v_cierre + 0.3 * cierre
        self._libre_prev = libre

        parar = self._umbral("parar_bajo")
        frenar = self._umbral("frenar_bajo")
        vel_crucero = float(self.lim.get("vel_crucero", 55))
        vel_giro = float(self.lim.get("vel_giro", 38))

        # 1) envolvente por distancia
        if libre >= frenar:
            techo = vel_crucero
            motivo = ""
        else:
            t = (libre - parar) / max(1e-3, frenar - parar)
            techo = vel_giro + (vel_crucero - vel_giro) * _lim(t, 0.0, 1.0)
            motivo = "freno"

        # 2) tiempo hasta el muro
        margen = max(0.0, libre - parar)
        self.ttc = margen / self._v_cierre if self._v_cierre > 1e-3 else 99.0
        ttc_min = float(self.cfg.get("ttc_min", 1.1))
        if self.ttc < ttc_min:
            factor = _lim(self.ttc / max(0.05, ttc_min), 0.0, 1.0)
            techo = min(techo, vel_giro * 0.6 + (techo - vel_giro * 0.6) * factor)
            motivo = f"ttc{self.ttc:.1f}"
        return techo, motivo

    # -- esquina interna --------------------------------------------------
    def _esquina_interna(self, p: PerfilMuro, ahora: float) -> Tuple[bool, int, str]:
        """True cuando el muro interno ha dejado de verse: hay que girar.

        Prioridad de senales:
          a) UNA PARED QUE SE VEIA HA DESAPARECIDO. Es la mejor, y no necesita
             saber el sentido: si esa banda tenia muro y ya no, ahi se acabo el
             interno y hacia alli se gira. Vale desde la primera esquina.
          b) Si ya sabemos cual es el interno, que su banda este despejada.
          c) Un escalon grande de cerca a lejos en el lado interno.
        Girar aqui, y no cuando el muro de enfrente ya esta encima, es lo que
        evita llegar tarde a la esquina.
        """
        if not bool(self.cfg.get("usar_esquina_interna", True)):
            return False, 0, ""

        # (a) evento de desaparicion, con una ventana corta de validez
        if self._esquina_pendiente:
            venc = float(self.cfg.get("validez_esquina_ms", 900)) / 1000.0
            if (ahora - self._t_esquina_pendiente) <= venc:
                lado = self._esquina_pendiente
                self._esquina_pendiente = 0
                return True, lado, "el muro interno dejo de verse"
            self._esquina_pendiente = 0

        lado = self.paredes.lado_interno
        if lado == 0:
            return False, 0, ""
        # (b) banda interna despejada
        umbral = float(self.cfg.get("interno_libre", 0.72))
        if p.banda(lado) >= umbral:
            return True, lado, f"interno libre {p.banda(lado):.2f}"
        # (c) escalon
        for b in p.bordes[:2]:
            if b.lado == lado and abs(b.salto) > float(self.cfg.get("salto_min", 0.12)) * 1.6:
                if b.cerca_a_lejos == (lado == IZQ):
                    return True, lado, f"escalon interno {b.salto:+.2f}"
        return False, 0, ""

    # -- ciclo principal --------------------------------------------------
    def paso(self, perfil: PerfilMuro, yaw: Optional[float] = None,
             esquiva: Optional[Any] = None) -> Decision:
        ahora = time.time()
        cfg = self.cfg
        dir_max = float(self.lim.get("dir_max", 100))
        vel_giro = float(self.lim.get("vel_giro", 38))
        libre = perfil.pasillo

        usar_yaw = bool(cfg.get("usar_yaw", True)) and yaw is not None
        if usar_yaw and self.rumbo_objetivo is None:
            self.rumbo_objetivo = yaw

        # Solo se mira la pista cuando el carro esta navegando de verdad: en
        # media vuelta o escapando ve cosas raras y aprender de ahi es lo que
        # hacia que el sentido se volviera loco.
        if self.estado in (RECTO, PRE_GIRO, GIRO):
            self.paredes.observar(perfil, ahora)
            lado_desaparecido = self.esquinas.paso(perfil, ahora)
            if lado_desaparecido:
                self.paredes.voto_muro_desaparecido(lado_desaparecido)
                if self.estado == RECTO:
                    self._esquina_pendiente = lado_desaparecido
                    self._t_esquina_pendiente = ahora
        self.carril.observar(perfil, self.estado, self.ultimo.direccion)
        techo, motivo_vel = self._anticipar(perfil, ahora)

        if self.terminado:
            return self._salida(0, 0, perfil, yaw, "recorrido terminado")

        # ---------- MEDIA VUELTA -----------------------------------------
        if self.estado == MEDIA_VUELTA or (self.media_vuelta_pedida
                                           and not self.media_vuelta_hecha):
            return self._media_vuelta(perfil, yaw, ahora)

        # ---------- ESCAPE ------------------------------------------------
        parar = self._umbral("parar_bajo")
        girar = self._umbral("girar_bajo")
        if libre < parar and self.estado != ESCAPE:
            self._entrar_escape(perfil, ahora)
        if self.estado == ESCAPE:
            # Se sale cuando de verdad hay sitio Y se ha cumplido el compromiso
            # minimo de retroceso. Sin lo segundo bastaba un parpadeo del perfil
            # para volver a la navegacion pegado al muro, y de ahi el vaiven.
            hecho = (ahora - self._escape_t) * 1000.0 >= min(
                self._escape_compromiso, float(cfg.get("escape_atras_min_ms", 900)))
            if libre >= self._escape_objetivo and hecho:
                self._cambiar(RECTO)
                self.pd_centrado.reiniciar()
                self.esquinas.reiniciar()
            else:
                return self._escape(perfil, yaw, ahora)

        # ---------- GIRO ---------------------------------------------------
        if self.estado == GIRO:
            venc = self._ms_en_estado(ahora) > float(cfg.get("giro_max_ms", 3000))
            if usar_yaw and self.rumbo_objetivo is not None:
                err = _dif_angulo(self.rumbo_objetivo, yaw)
                if abs(err) < float(cfg.get("giro_tolerancia", 8.0)) or venc:
                    self._cambiar(RECTO)
                    if venc:
                        self.rumbo_objetivo = yaw
                else:
                    d = _lim(err * float(cfg.get("yaw_kp", 1.6)) * 3.0, -dir_max, dir_max)
                    d = _lim(d, -float(cfg.get("dir_giro", 90.0)),
                             float(cfg.get("dir_giro", 90.0)))
                    d = self._mezclar_diagonal(d, perfil)
                    return self._salida(min(techo, vel_giro), d, perfil, yaw,
                                        f"giro yaw {err:+.0f}")
            else:
                if libre > float(cfg.get("salir_giro_sobre", 0.55)) or venc:
                    self._cambiar(RECTO)
                    self.pd_centrado.reiniciar()
                else:
                    d = self.lado_giro * float(cfg.get("dir_giro_abierto", 65.0))
                    d = self._mezclar_diagonal(d, perfil)
                    return self._salida(min(techo, vel_giro), d, perfil, yaw,
                                        f"giro vision {libre:.2f}")

        # ---------- PRE_GIRO: frenar ANTES de entrar en la curva ----------
        if self.estado == PRE_GIRO:
            if self._ms_en_estado(ahora) >= float(cfg.get("retardo_giro_ms", 250)):
                self._cambiar(GIRO)
                if usar_yaw:
                    paso = float(cfg.get("giro_grados", 90.0)) * self.lado_giro
                    self.rumbo_objetivo = _norm_angulo((self.rumbo_objetivo or yaw) + paso)
                self.sentido.voto_giro(self.lado_giro)
                abierto = float(cfg.get("dir_giro_abierto", 65.0))
                return self._salida(min(techo, vel_giro), self.lado_giro * abierto,
                                    perfil, yaw, "arranca el giro")
            # durante el retardo: recto y frenando, para que las ruedas
            # traseras terminen de pasar la esquina interna antes de girar
            d, _m, _a = self._mezcla(perfil, ahora)
            return self._salida(min(techo, vel_giro), d * 0.4, perfil, yaw,
                                "pre-giro: frenando y dejando pasar la esquina")

        # ---------- RECTO: decidir si toca esquina -------------------------
        recto_estable = self._ms_en_estado(ahora) >= float(cfg.get("min_recto_ms", 600))
        esquina, lado_int, razon = self._esquina_interna(perfil, ahora)
        por_frente = libre < girar

        if recto_estable and (esquina or por_frente):
            if esquina and lado_int != 0:
                self.lado_giro = lado_int
                motivo = f"esquina interna ({razon})"
            else:
                self.lado_giro = DER if perfil.der > perfil.izq else IZQ
                motivo = f"muro de frente {libre:.2f}"
            self._cambiar(PRE_GIRO)
            return self._salida(min(techo, vel_giro), 0, perfil, yaw, motivo)

        # ---------- RECTO normal ------------------------------------------
        direccion, motivo, aportes = self._mezcla(perfil, ahora)
        self._aportes = aportes

        # ---- esquiva de pilares -------------------------------------------
        # Solo en recta: en una esquina o escapando mandan el muro y la
        # seguridad, no un pilar. El peso sube segun se acerca el pilar, asi
        # que de lejos solo corrige un poco y de cerca manda ella.
        if esquiva is not None and getattr(esquiva, "activo", False) and esquiva.peso > 0:
            w = _lim(float(esquiva.peso), 0.0, 1.0)
            direccion = (1.0 - w) * direccion + w * float(esquiva.direccion)
            self._aportes = dict(aportes, pilar=round(float(esquiva.direccion), 1))
            motivo = f"{esquiva.motivo} | {motivo}"

        if usar_yaw and self.rumbo_objetivo is not None:
            err = _dif_angulo(self.rumbo_objetivo, yaw)
            correccion = _lim(err * float(cfg.get("yaw_kp", 1.6)),
                              -float(cfg.get("yaw_max", 45.0)),
                              float(cfg.get("yaw_max", 45.0)))
            direccion += correccion
            motivo += f" yaw{err:+.0f}"

        vel = techo * (1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max)))
        if motivo_vel:
            motivo += " " + motivo_vel
        return self._salida(vel, direccion, perfil, yaw, motivo)

    def _mezclar_diagonal(self, d: float, p: PerfilMuro) -> float:
        """Cruzar en diagonal a la siguiente pared externa en vez de barrer la
        esquina con un angulo fijo.

        Si hay un hueco pasable, se apunta a el: eso traza la diagonal desde la
        pared externa que se deja hasta la siguiente. Se mezcla con el angulo
        de giro para no perder el compromiso del viraje.
        """
        if not bool(self.cfg.get("giro_diagonal", True)) or not p.huecos:
            return d
        h = p.huecos[0]
        if not h.pasable:
            return d
        err = (h.centro - p.ancho / 2.0) / (p.ancho / 2.0)
        objetivo = _lim(err * float(self.cfg.get("kp_diagonal", 110.0)),
                        -float(self.lim.get("dir_max", 100)),
                        float(self.lim.get("dir_max", 100)))
        peso = _lim(float(self.cfg.get("peso_diagonal", 0.45)), 0.0, 1.0)
        # Nunca al lado contrario del giro: si el hueco tira para el otro lado,
        # se ignora (es el hueco de detras, no el de la salida).
        if self.lado_giro and objetivo * self.lado_giro < 0:
            return d
        return (1.0 - peso) * d + peso * objetivo

    # -- escape -----------------------------------------------------------
    def _entrar_escape(self, p: PerfilMuro, ahora: float):
        self._cambiar(ESCAPE)
        self._escape_ref = p.pasillo
        self._escape_mejor = p.pasillo
        self._escape_t = ahora
        self._escape_cambios = 0
        # Espacio que hay que recuperar antes de volver a navegar
        self._escape_objetivo = self._umbral("girar_bajo") * float(
            self.cfg.get("escape_salir_factor", 1.15))
        # Cuanto retroceder: proporcional a lo cerca que esta el muro. Cuanto
        # mas encima lo tengamos, mas hay que echarse atras.
        deficit = _lim((self._escape_objetivo - p.pasillo) /
                       max(0.05, self._escape_objetivo), 0.0, 1.0)
        self._escape_compromiso = (float(self.cfg.get("escape_atras_min_ms", 900))
                                   + deficit * float(self.cfg.get("escape_atras_extra_ms", 1600)))
        self._escape_modo = "atras"
        # Lado libre al que queremos acabar apuntando
        if self.paredes.lado_interno != 0:
            self._escape_lado = self.paredes.lado_interno
        else:
            self._escape_lado = DER if p.der > p.izq else IZQ

    def _escape(self, p: PerfilMuro, yaw: Optional[float], ahora: float) -> Decision:
        """Salir de un muro encima.

        QUE FALLABA: se evaluaba cada 700 ms y se alternaba adelante/atras. Con
        el muro encima, 700 ms de marcha atras no dan para nada, asi que el
        carro se pasaba la vida yendo y viniendo sin ganar sitio hasta que
        chocaba. Ahora el retroceso va COMPROMETIDO: se calcula cuanto espacio
        falta y se retrocede al menos ese tiempo, sin reevaluar a mitad.

        Solo se abandona la marcha atras si el espacio de delante no mejora
        NADA en un buen rato, que es la firma de tener algo pegado detras; solo
        entonces se prueba el giro hacia adelante.
        """
        cfg = self.cfg
        dir_max = float(self.lim.get("dir_max", 100))
        vel_escape = float(cfg.get("vel_escape", 26))
        mejora = float(cfg.get("mejora_min", 0.035))
        atascado_ms = float(cfg.get("escape_atascado_ms", 1300))
        transcurrido = (ahora - self._escape_t) * 1000.0
        self._escape_mejor = max(self._escape_mejor, p.pasillo)

        if self._escape_modo == "atras":
            cumplido = transcurrido >= self._escape_compromiso
            atascado = (transcurrido >= atascado_ms and
                        (self._escape_mejor - self._escape_ref) < mejora)
            if atascado:
                # Hay algo detras: el unico camino es hacia adelante girando
                self._escape_modo = "adelante"
                self._escape_t = ahora
                self._escape_ref = p.pasillo
                self._escape_mejor = p.pasillo
                self._escape_cambios += 1
            elif cumplido and p.pasillo >= self._escape_objetivo:
                pass          # el bucle principal ya sale del escape
            # Marcha atras con la direccion hacia el muro: asi el morro se
            # separa de el (como al salir de un aparcamiento).
            d = -self._escape_lado * dir_max
            vel = -vel_escape
            motivo = (f"atras {transcurrido / 1000:.1f}/{self._escape_compromiso / 1000:.1f}s"
                      f"  pasillo {p.pasillo:.2f}->{self._escape_objetivo:.2f}")
        else:
            if transcurrido >= atascado_ms:
                if (self._escape_mejor - self._escape_ref) < mejora:
                    self._escape_lado = -self._escape_lado
                    self._escape_cambios += 1
                self._escape_modo = "atras"
                self._escape_t = ahora
                self._escape_ref = p.pasillo
                self._escape_mejor = p.pasillo
                self._escape_compromiso = float(cfg.get("escape_atras_min_ms", 900))
            d = self._escape_lado * dir_max
            vel = vel_escape * 0.8
            motivo = (f"adelante hacia {'der' if self._escape_lado > 0 else 'izq'}"
                      f" (algo detras)  pasillo {p.pasillo:.2f}")

        if self._escape_cambios >= 4:
            motivo += "  ATASCADO"
        return self._salida(vel, d, p, yaw, motivo)

    # -- media vuelta -----------------------------------------------------
    def _media_vuelta(self, p: PerfilMuro, yaw: Optional[float], ahora: float) -> Decision:
        cfg = self.cfg
        tipo = str(self.cfg_vueltas.get("tipo_media_vuelta", "recta_3t"))
        dir_max = float(self.lim.get("dir_max", 100))
        vel = float(cfg.get("vel_media_vuelta", 30))
        tol = float(cfg.get("giro_tolerancia", 8.0))

        if self.estado != MEDIA_VUELTA:
            self._cambiar(MEDIA_VUELTA)
            self._mv_fase = 0
            self._mv_t = ahora
            self._mv_yaw0 = yaw
            self._mv_yaw_prev = yaw
            self._mv_girado = 0.0
            # Girar hacia el interior deja mas sitio: el carril es mas ancho
            # por dentro de la curva que pegado al muro externo.
            self._mv_lado = self.sentido.lado_interno or (DER if p.der > p.izq else IZQ)

        # OJO: para medir 180 grados NO sirve la diferencia angular contra el
        # rumbo inicial, porque se envuelve: al pasar de 180 empieza a BAJAR y
        # la maniobra no termina nunca. Hay que ACUMULAR el giro paso a paso.
        # Se acumula con signo para que un bandazo hacia el otro lado reste.
        if yaw is not None:
            if self._mv_yaw_prev is not None:
                paso_yaw = _dif_angulo(yaw, self._mv_yaw_prev)
                if abs(paso_yaw) > 0.15:            # zona muerta contra el ruido
                    self._mv_girado += paso_yaw
            self._mv_yaw_prev = yaw
        girado = abs(self._mv_girado)

        # ---- por esquina: encadenar dos giros donde sobra sitio ----------
        if tipo.startswith("esq"):
            if p.pasillo < self._umbral("girar_bajo") or girado > 10:
                if yaw is not None:
                    if girado >= 180.0 - tol:
                        return self._fin_media_vuelta(p, yaw)
                    return self._salida(vel, self._mv_lado * dir_max, p, yaw,
                                        f"media vuelta en esquina {girado:.0f}/180")
                if (ahora - self._mv_t) * 1000 > float(cfg.get("mv_giro_ms", 2600)):
                    return self._fin_media_vuelta(p, yaw)
                return self._salida(vel, self._mv_lado * dir_max, p, yaw,
                                    "media vuelta en esquina (sin yaw)")
            return self._salida(min(vel, float(self.lim.get("vel_giro", 38))),
                                0, p, yaw, "media vuelta: buscando la esquina")

        # ---- tres tiempos en recta --------------------------------------
        # fase 0 adelante girando, 1 atras girando al reves, 2 adelante otra vez
        t_fase = (ahora - self._mv_t) * 1000
        limite = float(cfg.get("mv_fase_ms", 1400))
        sin_sitio = p.pasillo < self._umbral("parar_bajo") * 1.15

        if self._mv_fase == 0:
            if yaw is not None and girado >= 180.0 - tol:
                return self._fin_media_vuelta(p, yaw)
            if t_fase > limite or sin_sitio:
                self._mv_fase = 1
                self._mv_t = ahora
            return self._salida(vel, self._mv_lado * dir_max, p, yaw,
                                f"media vuelta 1/3 ({girado:.0f} grados)")
        if self._mv_fase == 1:
            if t_fase > limite:
                self._mv_fase = 2
                self._mv_t = ahora
            return self._salida(-vel * 0.85, -self._mv_lado * dir_max, p, yaw,
                                f"media vuelta 2/3 ({girado:.0f} grados)")
        if yaw is not None and girado >= 180.0 - tol:
            return self._fin_media_vuelta(p, yaw)
        if t_fase > limite:
            if yaw is None:
                return self._fin_media_vuelta(p, yaw)
            self._mv_fase = 1                  # otra pasada de vaiven
            self._mv_t = ahora
        return self._salida(vel, self._mv_lado * dir_max, p, yaw,
                            f"media vuelta 3/3 ({girado:.0f} grados)")

    def _fin_media_vuelta(self, p: PerfilMuro, yaw: Optional[float]) -> Decision:
        self.media_vuelta_hecha = True
        self.media_vuelta_pedida = False
        # Las dos paredes se intercambian y el estimador queda BLOQUEADO unos
        # segundos: sin eso volvia a decidir el sentido viejo y el carro se
        # ponia a deshacer la media vuelta que acababa de hacer.
        self.paredes.invertir()
        self.esquinas.reiniciar()
        self._esquina_pendiente = 0
        self.pd_centrado.reiniciar()
        self.pd_pared.reiniciar()
        self.pd_hueco.reiniciar()
        self._libre_prev = None
        self._cambiar(RECTO)
        # El rumbo objetivo es el de AHORA (ya girado). Si se dejara en None se
        # recalcularia solo, pero explicitarlo evita un tiron en el primer frame.
        self.rumbo_objetivo = yaw
        # Y no se admite otra esquina inmediatamente: el carro esta atravesado
        # y el perfil todavia no significa nada.
        self.t_estado = time.time() + float(self.cfg.get("gracia_tras_media_ms", 800)) / 1000.0
        return self._salida(0, 0, p, yaw, "media vuelta completada")

    # -- salida -----------------------------------------------------------
    def _salida(self, vel: float, direccion: float, perfil: PerfilMuro,
                yaw: Optional[float], motivo: str) -> Decision:
        dir_max = float(self.lim.get("dir_max", 100))
        d = Decision(
            vel=int(round(_lim(vel, -100, 100))),
            direccion=int(round(_lim(direccion, -dir_max, dir_max))),
            estado=self.estado,
            motivo=motivo,
            metricas=self._metricas(perfil, yaw),
        )
        self.ultimo = d
        return d

    _aportes: Dict[str, float] = {}

    def _metricas(self, perfil: PerfilMuro, yaw: Optional[float]) -> Dict[str, Any]:
        m: Dict[str, Any] = {
            "izq": round(perfil.izq, 3),
            "der": round(perfil.der, 3),
            "pasillo": round(perfil.pasillo, 3),
            "min": round(perfil.min_global, 3),
            "ttc": round(min(99.0, self.ttc), 2),
            "cierre": round(self._v_cierre, 3),
            "sentido": self.paredes.estado(),
            "muros_vistos": self.esquinas.estado(),
            "cobertura": [round(perfil.cobertura_izq, 2), round(perfil.cobertura_der, 2)],
            "carril": self.carril.estado(),
            "umbrales": {k: round(self._umbral(k), 3)
                         for k in ("parar_bajo", "girar_bajo", "frenar_bajo")},
            "aportes": dict(self._aportes),
        }
        if perfil.huecos:
            h = perfil.huecos[0]
            m["hueco"] = {"x": h.centro, "ancho": h.ancho_px,
                          "necesario": round(h.ancho_necesario, 1),
                          "margen": round(h.margen, 2), "pasable": h.pasable,
                          "n": len(perfil.huecos)}
        if perfil.bordes:
            b = perfil.bordes[0]
            m["borde"] = {"x": b.x, "salto": round(b.salto, 3)}
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["yaw_obj"] = round(self.rumbo_objetivo, 1)
        return m


# ===========================================================================
# Dibujo
# ===========================================================================
def dibujar_navegacion(frame: np.ndarray, perfil: PerfilMuro, d: Decision,
                       cfg: Dict[str, Any], nav: Optional[Navegador] = None) -> np.ndarray:
    H, W = frame.shape[:2]

    # perfil de contacto
    pts = []
    for x in range(0, W, 4):
        y = int(perfil.y_contacto[min(x, len(perfil.y_contacto) - 1)])
        if y > 0:
            pts.append((x, y))
    for i in range(1, len(pts)):
        if pts[i][0] - pts[i - 1][0] <= 8:
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 255), 2)

    # huecos pasables
    for h in perfil.huecos[:3]:
        y = H - 30
        color = (0, 220, 0) if h.pasable else (0, 0, 200)
        cv2.line(frame, (h.x0, y), (h.x1, y), color, 2)
        cv2.line(frame, (h.centro, y - 6), (h.centro, y + 6), color, 2)
    if perfil.huecos and perfil.huecos[0].pasable:
        h = perfil.huecos[0]
        n = int(h.ancho_necesario)
        cv2.rectangle(frame, (h.centro - n // 2, H - 40), (h.centro + n // 2, H - 22),
                      (255, 255, 0), 1)

    # escalones (final del muro interno)
    for b in perfil.bordes[:2]:
        color = (255, 0, 255) if b.cerca_a_lejos else (255, 120, 0)
        cv2.line(frame, (b.x, int(H * 0.42)), (b.x, int(H * 0.62)), color, 2)

    # lineas de las ruedas
    xi = int(W * float(cfg.get("ruedas_izq", 0.32)))
    xd = int(W * float(cfg.get("ruedas_der", 0.68)))
    for x in (xi, xd):
        cv2.line(frame, (x, int(H * 0.45)), (x, H), (255, 255, 255), 1, cv2.LINE_AA)
    cv2.line(frame, (xi, H - 2), (xd, H - 2), (255, 255, 255), 1)

    y_fin = int(H * (1.0 - float(cfg.get("ignorar_abajo", 0.0))))
    if y_fin < H:
        cv2.line(frame, (0, y_fin), (W, y_fin), (120, 120, 120), 1)

    def barra(x0, x1, valor, etiqueta, color):
        alto = int(44 * _lim(valor, 0, 1))
        cv2.rectangle(frame, (x0, H - 12 - alto), (x1, H - 12), color, -1)
        cv2.putText(frame, f"{etiqueta}{valor:.2f}", (x0 - 2, H - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    n_lat = max(1, int(W * float(cfg.get("banda_lateral", 0.28))))
    barra(4, 30, perfil.izq, "I", (60, 220, 60))
    barra(W - 30, W - 4, perfil.der, "D", (60, 220, 60))
    barra(W // 2 - 13, W // 2 + 13, perfil.pasillo, "C", (60, 200, 255))
    cv2.line(frame, (n_lat, int(H * 0.5)), (n_lat, H), (80, 80, 80), 1)
    cv2.line(frame, (W - n_lat, int(H * 0.5)), (W - n_lat, H), (80, 80, 80), 1)

    cx, cy = W // 2, 26
    x2 = int(cx + int(W * 0.22) * (d.direccion / 100.0))
    cv2.line(frame, (cx, cy), (x2, cy), (0, 165, 255), 4)
    cv2.circle(frame, (cx, cy), 4, (255, 255, 255), -1)

    color_estado = {RECTO: (0, 255, 0), PRE_GIRO: (0, 220, 220), GIRO: (0, 200, 255),
                    ESCAPE: (0, 0, 255), MEDIA_VUELTA: (255, 0, 255),
                    FIN: (200, 200, 200)}.get(d.estado, (255, 255, 255))
    cv2.putText(frame, f"{d.estado.upper()}  vel={d.vel:+d}%  dir={d.direccion:+d}%",
                (8, H - 94), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_estado, 1, cv2.LINE_AA)
    if d.motivo:
        cv2.putText(frame, d.motivo[:58], (8, H - 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    if nav is not None:
        s = nav.sentido
        txt = f"sentido {s.estado()['nombre']}"
        if nav.carril.listo:
            txt += f" | carril {nav.carril.ancho:.2f}"
        txt += f" | ttc {min(99, nav.ttc):.1f}"
        cv2.putText(frame, txt, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (200, 200, 100), 1, cv2.LINE_AA)
    return frame


def dibujar_roi(frame: np.ndarray, roi_arriba: float, roi_abajo: float,
                color_bgr: Sequence[int] = (0, 255, 255)) -> np.ndarray:
    h, w = frame.shape[:2]
    c = tuple(int(v) for v in color_bgr)
    cv2.line(frame, (0, int(roi_arriba * h)), (w, int(roi_arriba * h)), c, 1)
    cv2.line(frame, (0, int(roi_abajo * h) - 1), (w, int(roi_abajo * h) - 1), c, 1)
    return frame
