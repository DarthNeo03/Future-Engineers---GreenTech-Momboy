"""
geometria.py — De pixeles a milimetros sobre el plano del suelo.

La camara esta a ALTURA fija (125 mm) e INCLINADA hacia abajo un angulo fijo
(7.5 grados). Con eso, cada fila de la imagen corresponde a UNA distancia sobre
el suelo, y cada columna a un desplazamiento lateral proporcional. No hace
falta homografia con patron de ajedrez: basta la altura, la inclinacion y la
focal en pixeles, que se calibra con UN clic sobre un objeto a distancia
conocida (pestaña Calibracion de la web).

Modelo (pinhole, sin distorsion de lente):

    angulo bajo el horizonte de la fila v:   theta = inclinacion + atan((v-cy)/fy)
    distancia sobre el suelo:                Y = altura / tan(theta)
    rango inclinado hasta ese punto:         R = altura / sin(theta)
    desplazamiento lateral:                  X = (u-cx)/fx * R

Consecuencia importante que arregla el error del programa viejo ("detecta
objetos por ARRIBA de las paredes"): la fila del horizonte es fija. Todo lo
que este por encima de ella no puede ser pista (los muros miden 100 mm y la
camara esta a 125: hasta el borde superior del muro queda siempre bajo el
horizonte). Sillas, mesas y publico quedan cortados por geometria, no por
color.

NOTA sobre lentes anchas: si la camara tiene mucha distorsion de barril, el
modelo pinhole se desvia en los bordes. Para navegar (comparar izquierda vs
derecha, medir el pasillo) es suficiente; para medir con precision en las
esquinas de la imagen, calibrar fx con un objeto lateral y dejar margen.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import numpy as np

DIST_MAX_MM = 6000.0     # mas alla de esto se reporta "infinito util"


class Geometria:
    """Todas las conversiones dependen de cfg (dict vivo de params.json):

        alto_cam_mm      altura del centro optico sobre el suelo (125)
        inclinacion_deg  inclinacion hacia abajo desde la horizontal (7.5)
        fy_px, fx_px     focales en pixeles (se calibran con un clic)
        ancho_carro_mm   ancho total del carro con ruedas (200)
        margen_ruedas_mm margen extra por lado para el corredor
        adelanto_cam_mm  distancia del eje delantero a la camara (para que
                         "distancia al muro" sea desde el morro, no la lente)
    """

    def __init__(self, cfg: Dict[str, Any], ancho_img: int = 640, alto_img: int = 480):
        self.cfg = cfg
        self.W = int(ancho_img)
        self.H = int(alto_img)

    def redimensionar(self, ancho_img: int, alto_img: int) -> None:
        self.W = int(ancho_img)
        self.H = int(alto_img)

    # -- parametros ------------------------------------------------------
    @property
    def _h(self) -> float:
        return float(self.cfg.get("alto_cam_mm", 125.0))

    @property
    def _tilt(self) -> float:
        return math.radians(float(self.cfg.get("inclinacion_deg", 7.5)))

    @property
    def _fy(self) -> float:
        # escala con la resolucion: fy se guarda referido a 480 de alto
        return float(self.cfg.get("fy_px", 460.0)) * (self.H / 480.0)

    @property
    def _fx(self) -> float:
        """Focal horizontal EFECTIVA, en pixeles de la captura de ahora.

        LA TRAMPA QUE SE LLEVA EL RETO DE OBSTACULOS. fx_px se guarda referido
        a 640 de ancho y fy_px a 480 de alto, y cada uno se escala con SU lado
        de la captura. Mientras se capture a 640x480 los dos salen iguales,
        que es lo que tiene que pasar: el sensor tiene pixeles CUADRADOS y la
        focal es una sola. Pero basta con que la camara entregue otra cosa
        -y las entregan: se pide 1920x480 y la WN-L1812 da 1280x720- para que
        se separen. Medido en este carro: fx efectivo 920 contra fy 690, un
        33 % de diferencia.

        Y no se nota igual en los dos retos. En el Open Challenge la distancia
        al muro sale de las FILAS (o sea de fy) y el error apenas asoma; aqui
        decide los milimetros LATERALES, y con ellos:

          * donde esta el pilar de lado -> por donde hay que pasarlo;
          * a que distancia esta la pared -> cuanto sitio cree que tiene;
          * que columnas caen dentro del corredor de las ruedas -> el pasillo.

        Con fx un 33 % de mas, los laterales se miden un 25 % CORTOS: la pared
        a 350 mm se lee a 260, el corredor se ensancha y el pasillo se cierra
        solo. El sintoma en pista es exactamente "no se atreve a pasar aunque
        tiene sitio de sobra", y no hay parametro del esquive que lo arregle.

        Con fx_auto (recomendado, y encendido en config/obstaculos/) la focal
        horizontal se toma de la vertical: pixeles cuadrados, una sola focal,
        y da igual a que resolucion entregue la camara. Se apaga solo en
        cuanto alguien calibra fx a mano desde la web, que es decir "usa mi
        numero"; y para una lente muy anamorfica, a mano.
        """
        if bool(self.cfg.get("fx_auto", False)):
            return self._fy
        return float(self.cfg.get("fx_px", 460.0)) * (self.W / 640.0)

    @property
    def _cx(self) -> float:
        """Columna por la que pasa el EJE DEL CARRO.

        Todo lo demas (el corredor de las ruedas, el desplazamiento lateral,
        comparar izquierda contra derecha) se mide desde aqui, y hasta ahora se
        daba por hecho que era el centro exacto de la imagen: o sea, que la
        camara esta clavada en el eje del carro y perfectamente encarada. Un
        par de milimetros de desplazamiento, o un par de grados de guiñada del
        mastil, meten un sesgo CONSTANTE en izquierda-contra-derecha, y ese
        sesgo no es neutral: en un sentido de la ronda empuja al carro hacia el
        muro externo (donde sobra sitio y el control lo corrige sin que se note)
        y en el otro contra el interno. Es una de las pocas cosas que pueden
        hacer que los MISMOS parametros funcionen en horario y no en
        antihorario.

        centro_lateral_px lo corrige. Vale 0 por defecto: sin tocarlo, esto es
        exactamente lo de siempre.
        """
        return self.W / 2.0 + float(self.cfg.get("centro_lateral_px", 0.0))

    def offset_lateral_px(self) -> int:
        """Desplazamiento del eje del carro respecto al centro de la imagen,
        en columnas. Lo usa el perfil para que las bandas laterales queden
        simetricas respecto al CARRO y no respecto a la imagen."""
        return int(round(float(self.cfg.get("centro_lateral_px", 0.0))))

    @property
    def _cy(self) -> float:
        return self.H / 2.0

    # -- filas <-> distancias --------------------------------------------
    def fila_horizonte(self) -> int:
        """Fila de la imagen donde esta el horizonte. Por encima: NO es pista."""
        return int(round(self._cy - self._fy * math.tan(self._tilt)))

    def fila_a_distancia(self, v) -> np.ndarray:
        """Fila(s) de imagen -> distancia sobre el suelo en mm (desde la camara).
        Acepta escalar o array. Filas en el horizonte o encima -> DIST_MAX_MM."""
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        with np.errstate(divide="ignore", invalid="ignore"):
            d = self._h / np.tan(theta)
        d = np.where((theta <= 1e-4) | ~np.isfinite(d), DIST_MAX_MM, d)
        return np.clip(d, 0.0, DIST_MAX_MM)

    def distancia_a_fila(self, d_mm: float) -> int:
        """Distancia sobre el suelo (mm) -> fila de imagen."""
        d_mm = max(1.0, float(d_mm))
        theta = math.atan2(self._h, d_mm)
        av = theta - self._tilt
        return int(round(self._cy + self._fy * math.tan(av)))

    def fila_de_altura(self, y_mm: float, z_mm: float = 0.0) -> int:
        """Fila de imagen de un punto que esta a y_mm de distancia sobre el
        suelo y a z_mm de ALTURA sobre el.

        distancia_a_fila() solo sabe de puntos apoyados en el suelo (z=0). Para
        saber cuanto debe MEDIR en pixeles un pilar de 100 mm que esta a 1.2 m
        hace falta tambien la fila de su cima, y esa esta z_mm por encima del
        plano del suelo. Con z_mm = 0 devuelve exactamente lo mismo que
        distancia_a_fila().
        """
        y_mm = max(1.0, float(y_mm))
        theta = math.atan2(self._h - float(z_mm), y_mm)
        return int(round(self._cy + self._fy * math.tan(theta - self._tilt)))

    def alto_esperado_px(self, y_mm: float, z_mm: float) -> int:
        """Cuantos pixeles de alto ocupa un objeto de z_mm de alto apoyado en
        el suelo a y_mm de distancia. Es la prueba mas barata que existe para
        distinguir un pilar de verdad de una mancha de color en la pared."""
        return max(1, self.fila_de_altura(y_mm, 0.0) -
                   self.fila_de_altura(y_mm, float(z_mm)))

    def lateral_mm(self, u, v) -> np.ndarray:
        """Columna(s)+fila(s) -> desplazamiento lateral en mm (+derecha)."""
        u = np.asarray(u, dtype=np.float32)
        v = np.asarray(v, dtype=np.float32)
        theta = self._tilt + np.arctan((v - self._cy) / self._fy)
        theta = np.maximum(theta, 1e-3)
        rango = self._h / np.sin(theta)
        return (u - self._cx) / self._fx * rango

    def punto_suelo(self, u: float, v: float) -> Tuple[float, float]:
        """Pixel -> (x_lateral_mm, y_adelante_mm) sobre el suelo."""
        y = float(self.fila_a_distancia(v))
        x = float(self.lateral_mm(u, v))
        return x, y

    def suelo_a_pixel(self, x_mm: float, y_mm: float) -> Tuple[int, int]:
        """(lateral, adelante) en mm -> pixel. Inverso de punto_suelo."""
        v = self.distancia_a_fila(y_mm)
        theta = math.atan2(self._h, max(1.0, y_mm))
        rango = self._h / math.sin(theta)
        u = int(round(self._cx + x_mm * self._fx / rango))
        return u, v

    # -- corredor de las ruedas ------------------------------------------
    def corredor_en_fila(self, v: float) -> Tuple[int, int]:
        """Columnas (izq, der) por donde pasa el carro a la distancia de esa
        fila. La camara no ve las ruedas: esto las proyecta."""
        semi = (float(self.cfg.get("ancho_carro_mm", 200.0)) / 2.0 +
                float(self.cfg.get("margen_ruedas_mm", 30.0)))
        theta = self._tilt + math.atan((v - self._cy) / self._fy)
        theta = max(theta, 1e-3)
        rango = self._h / math.sin(theta)
        dx = semi * self._fx / rango
        return int(round(self._cx - dx)), int(round(self._cx + dx))

    def poligono_corredor(self, y_cerca_mm: float = 120.0,
                          y_lejos_mm: float = 2500.0, pasos: int = 12) -> np.ndarray:
        """Poligono (en pixeles) del pasillo que va a barrer el carro si sigue
        recto. Para dibujarlo y para recortar objetivos de esquive."""
        ys = np.linspace(y_cerca_mm, y_lejos_mm, pasos)
        izq, der = [], []
        for y in ys:
            v = self.distancia_a_fila(y)
            a, b = self.corredor_en_fila(v)
            izq.append((a, v))
            der.append((b, v))
        return np.array(izq + der[::-1], dtype=np.int32)

    # -- calibracion ------------------------------------------------------
    def calibrar_fy(self, v_clic: float, distancia_mm: float) -> float:
        """El usuario pone un objeto a distancia_mm del morro de la camara y
        hace clic en el punto donde TOCA EL SUELO. Devuelve el fy resuelto
        (referido a 480 de alto); el llamador decide si guardarlo."""
        d = max(50.0, float(distancia_mm))
        theta = math.atan2(self._h, d)
        av = theta - self._tilt
        if abs(math.tan(av)) < 1e-6 or abs(v_clic - self._cy) < 1.0:
            raise ValueError("clic demasiado cerca del horizonte para resolver fy")
        fy = (float(v_clic) - self._cy) / math.tan(av)
        if fy <= 50 or fy > 5000:
            raise ValueError(f"fy={fy:.0f} fuera de rango: revisa distancia/altura/inclinacion")
        return fy * (480.0 / self.H)

    def calibrar_fx(self, u_clic: float, v_clic: float, lateral_mm: float) -> float:
        """Objeto a un desplazamiento lateral conocido (mm, + a la derecha):
        clic en su base. Devuelve el fx resuelto (referido a 640 de ancho)."""
        if abs(lateral_mm) < 20:
            raise ValueError("usa un objeto claramente a un lado (>= 5 cm)")
        theta = self._tilt + math.atan((float(v_clic) - self._cy) / self._fy)
        if theta <= 1e-3:
            raise ValueError("el clic quedo en o sobre el horizonte")
        rango = self._h / math.sin(theta)
        fx = (float(u_clic) - self._cx) * rango / float(lateral_mm)
        if fx <= 50 or fx > 5000:
            raise ValueError(f"fx={fx:.0f} fuera de rango")
        return fx * (640.0 / self.W)

    # -- info -------------------------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "horizonte": self.fila_horizonte(),
            "fy_px": round(float(self.cfg.get("fy_px", 460.0)), 1),
            "fx_px": round(float(self.cfg.get("fx_px", 460.0)), 1),
            ### las EFECTIVAS, ya escaladas a la captura de ahora: son las que
            ### de verdad se usan, y si no se parecen entre ellas los
            ### milimetros laterales no valen (ver _fx)
            "fx_efectiva": round(self._fx, 1),
            "fy_efectiva": round(self._fy, 1),
            "fx_auto": bool(self.cfg.get("fx_auto", False)),
            "captura": f"{self.W}x{self.H}",
            "alto_cam_mm": self._h,
            "inclinacion_deg": float(self.cfg.get("inclinacion_deg", 7.5)),
            "dist_centro_mm": round(float(self.fila_a_distancia(self._cy)), 0),
            "dist_abajo_mm": round(float(self.fila_a_distancia(self.H - 1)), 0),
        }
