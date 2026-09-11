"""
carril.py — Mantenerse en la pista cuando NO hay que esquivar nada.

En el reto de obstaculos la distancia entre muros es siempre 1000 mm
(reglamento, rondas con obstaculos) y el carro mide 200 de ancho: sobran 400
por lado. Parece mucho hasta que hay un pilar pegado a un muro, porque
entonces el hueco util baja a ~350 mm y el carro solo cabe si viene BIEN
COLOCADO. Por eso el seguidor de carril no es "no chocar": es "llegar al pilar
por el sitio desde el que el esquive es posible".

TRES TERMINOS, Y CADA UNO ARREGLA UN FALLO DISTINTO

  1. CENTRADO (proporcional a la posicion). Compara cuanto suelo libre hay a
     izquierda y a derecha en el campo cercano. Solo con esto el carro oscila:
     corrige tarde y se pasa, corrige al otro lado y se vuelve a pasar.

  2. RUMBO (proporcional al angulo). Mira hacia donde esta el hueco mas
     profundo. Esto es lo que endereza el carro ANTES de estar descentrado, y
     es lo que le hace entrar limpio en las curvas.

  3. AMORTIGUACION (derivativo, del giroscopio). El termino derivativo se toma
     de gz del MPU-6050, no de derivar el error de la camara. Derivar una
     señal de vision a 30 fps con ruido de segmentacion amplifica el ruido; el
     giroscopio da la misma magnitud a 200 Hz y limpia. Es el unico sitio del
     programa donde la fusion de sensores es imprescindible y no un adorno.

LA CURVA SE DETECTA POR PROFUNDIDAD, NO POR COLOR. Cuando el frente se cierra
por debajo de 'dist_curva_mm', hay muro delante: es una seccion de curva. Ahi
el centrado estorba (no hay dos muros paralelos que centrar) y se pasa a
mandar casi solo el termino de rumbo, buscando el hueco.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .geometria import DIST_MAX_MM
from .vision import Escena


@dataclass
class SalidaCarril:
    direccion: float = 0.0       # % con signo, + derecha
    en_curva: bool = False
    dist_frente_mm: float = DIST_MAX_MM
    dist_izq_mm: float = DIST_MAX_MM
    dist_der_mm: float = DIST_MAX_MM
    err_centrado: float = 0.0    # normalizado -1..1
    err_rumbo: float = 0.0       # normalizado -1..1
    hueco_lat_mm: float = 0.0    # lateral del sector mas profundo


class SeguidorCarril:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self._dir_prev = 0.0

    # ------------------------------------------------------------------
    def paso(self, esc: Escena, gz: float = 0.0) -> SalidaCarril:
        """gz: velocidad angular del MPU en grados/s (+ derecha)."""
        s = SalidaCarril()
        perfil = esc.perfil_mm
        lat = esc.perfil_lat_mm
        if perfil.size == 0:
            return s

        n = perfil.size
        tercio = max(1, n // 3)

        # --- que hay delante y a los lados ------------------------------
        # El frente se mide con el tercio central; los lados con los tercios
        # exteriores. Se usa el MINIMO de cada zona: lo que importa no es la
        # distancia tipica sino lo mas cercano que hay por ese lado.
        s.dist_frente_mm = float(np.min(perfil[tercio:2 * tercio]))
        s.dist_izq_mm = float(np.min(perfil[:tercio]))
        s.dist_der_mm = float(np.min(perfil[-tercio:]))

        dist_curva = float(self.cfg.get("dist_curva_mm", 700.0))
        s.en_curva = s.dist_frente_mm < dist_curva

        # --- termino 1: centrado ----------------------------------------
        # Positivo = hay mas sitio a la derecha = hay que ir a la derecha.
        ancho_carril = float(self.cfg.get("ancho_carril_mm", 1000.0))
        bruto = (s.dist_der_mm - s.dist_izq_mm)
        s.err_centrado = float(np.clip(bruto / ancho_carril, -1.0, 1.0))

        # --- termino 2: rumbo hacia el hueco mas profundo ----------------
        # Se toma el sector mas profundo, pero suavizado con sus vecinos para
        # que un agujero de un solo sector (un brillo en el muro, un hueco
        # entre dos piezas) no tire del volante.
        suave = np.convolve(perfil, np.ones(3) / 3.0, mode="same")
        i_mejor = int(np.argmax(suave))
        s.hueco_lat_mm = float(lat[i_mejor]) if lat.size == n else 0.0
        # Angulo hacia ese hueco, con MIRADA MINIMA: sin ella, cuando el hueco
        # esta cerca el angulo crece hasta pedir el volante a tope, que es
        # justo lo que hace que el carro entre a las curvas dando un volantazo.
        mirada = max(float(self.cfg.get("mirada_min_mm", 500.0)),
                     float(suave[i_mejor]))
        ang = math.degrees(math.atan2(s.hueco_lat_mm, mirada))
        s.err_rumbo = float(np.clip(ang / 45.0, -1.0, 1.0))

        # --- mezcla ------------------------------------------------------
        if s.en_curva:
            # En curva no hay dos muros paralelos que centrar: el centrado
            # empuja contra la esquina interior. Se baja mucho su peso.
            kc = float(self.cfg.get("kp_centrado", 55.0)) * 0.25
            kr = float(self.cfg.get("kp_rumbo", 70.0)) * 1.35
        else:
            kc = float(self.cfg.get("kp_centrado", 55.0))
            kr = float(self.cfg.get("kp_rumbo", 70.0))

        # --- termino 3: amortiguacion con el giroscopio -------------------
        kd = float(self.cfg.get("kd_giro", 0.28))
        direccion = kc * s.err_centrado + kr * s.err_rumbo - kd * gz

        # Suavizado de primer orden: el servo ya tiene su propio limite de
        # grados por segundo en el firmware, pero filtrar aqui evita pedirle
        # cosas que no puede hacer y que se acumulan como retardo.
        alfa = float(self.cfg.get("suavizado", 0.45))
        direccion = alfa * direccion + (1.0 - alfa) * self._dir_prev
        self._dir_prev = direccion

        s.direccion = float(np.clip(direccion, -100.0, 100.0))
        return s

    # ------------------------------------------------------------------
    def velocidad(self, s: SalidaCarril) -> float:
        """Velocidad sugerida en %, segun lo despejado que este el frente.

        Frenar en las curvas no es prudencia: con direccion Ackermann y un
        carro de 200 mm, entrar rapido a una curva de 1000 mm de carril
        significa salir tocando el muro exterior. La velocidad es parte de la
        trayectoria, no un ajuste aparte.
        """
        v_max = float(self.cfg.get("vel_recta", 42.0))
        v_min = float(self.cfg.get("vel_curva", 24.0))
        d0 = float(self.cfg.get("dist_curva_mm", 700.0))
        d1 = float(self.cfg.get("dist_recta_mm", 1800.0))
        if s.dist_frente_mm >= d1:
            v = v_max
        elif s.dist_frente_mm <= d0:
            v = v_min
        else:
            t = (s.dist_frente_mm - d0) / max(1.0, d1 - d0)
            v = v_min + t * (v_max - v_min)
        # Y ademas se frena por volante: si se esta girando fuerte, es que la
        # trayectoria no es la prevista y conviene darle tiempo a la camara.
        castigo = abs(s.direccion) / 100.0
        v *= 1.0 - float(self.cfg.get("freno_por_volante", 0.35)) * castigo
        return max(v_min * 0.6, v)


def margen_magenta(esc: Escena, semiancho_mm: float) -> Optional[float]:
    """Correccion extra para NO TOCAR los delimitadores del cajon.

    Regla 9.25.7: tocar un delimitador termina la ronda en el acto. No hay
    penalizacion parcial, no hay "casi". Por eso los delimitadores no se
    tratan solo como muro: si aparece uno cerca del corredor, se devuelve un
    empujon lateral que se SUMA a lo que mande el seguidor de carril, aunque
    eso sacrifique el centrado.

    Devuelve el empujon en % (+ derecha) o None si no hay nada que esquivar.
    """
    if not esc.magenta:
        return None
    peor = None
    for d in esc.magenta:
        if d.dist_mm > 900.0:
            continue
        holgura = abs(d.lat_mm) - semiancho_mm
        if holgura > 120.0:        # pasa de largo con sitio de sobra
            continue
        # Empujar hacia el lado contrario, mas fuerte cuanto menos holgura y
        # cuanto mas cerca este.
        urgencia = max(0.0, 1.0 - holgura / 120.0)
        cercania = max(0.0, 1.0 - d.dist_mm / 900.0)
        empujon = -math.copysign(45.0 * urgencia * cercania, d.lat_mm)
        if peor is None or abs(empujon) > abs(peor):
            peor = empujon
    return peor
