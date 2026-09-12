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

...PERO "FRENTE CERRADO" NO SIEMPRE ES "ESQUINA", y confundirlo costaba la
ronda. Justo despues de rebasar un pilar el carro queda desplazado hacia un
lado y APUNTANDO A UN MURO: el frente se cierra sin que haya ninguna curva. Si
en ese momento se le trata como curva pasan las dos cosas a la vez —el
centrado, que es lo unico que sacaria al carro del muro, se queda en el 25 %, y
el sesgo de curva empuja hacia el lado donde giran las esquinas—, y el
resultado es el giro brusco contra la pared que se veia despues de un esquive
limpio. De ahi los dos frenos de este modulo:

  - `tras_pilar` (lo dice la FSM): ni se aprende el sentido, ni se aplica el
    sesgo. Los grados de rumbo de esos ciclos no son de ninguna curva.
  - `muro_encima` (lo dice la geometria): con un muro a menos de
    `guardia_muro_mm` el centrado recupera toda su autoridad y el sesgo nunca
    empuja hacia ese lado.
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
    lat_izq_mm: Optional[float] = None   # separacion LATERAL al muro izquierdo
    lat_der_mm: Optional[float] = None   # idem al derecho; None = no se ve
    err_centrado: float = 0.0    # normalizado -1..1
    err_rumbo: float = 0.0       # normalizado -1..1
    rumbo_hueco_deg: float = 0.0  # hacia donde esta la salida
    sentido_curva: int = 0       # +1 las curvas van a la derecha, -1 izquierda
    holgura_min_mm: Optional[float] = None  # el muro mas cercano, de cualquier lado
    muro_critico: bool = False   # a punto de rozar: manda la guardia y se frena
    muro_encima: bool = False    # algun muro por debajo de guardia_muro_mm
    sesgo: float = 0.0           # cuanto puso el sesgo de curva, en %
    motivo: str = ""             # que mando este ciclo, para la telemetria


class SeguidorCarril:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self._dir_prev = 0.0
        # Sentido de las curvas, APRENDIDO de la primera que se toma.
        self.sentido_curva = 0
        self._acum_curva = 0.0
        self._n_curva = 0
        self._en_curva_prev = False

    # ------------------------------------------------------------------
    def paso(self, esc: Escena, gz: float = 0.0,
             sentido_pista: int = 0, tras_pilar: bool = False,
             espera_linea: bool = False) -> SalidaCarril:
        """gz: velocidad angular del MPU en grados/s (+ derecha).

        sentido_pista: +1 horario / -1 antihorario si ya se dedujo de las
        lineas del piso; 0 si aun no se sabe. Solo se usa como pista mientras
        no se haya aprendido el sentido de las curvas de verdad.

        tras_pilar: hay un rebase en curso o recien terminado. UN SOLO DATO
        CON TRES CONSECUENCIAS, y las tres salen del mismo hecho: cuando el
        carro acaba de rebasar un pilar, el frente se cierra porque quedo
        APUNTANDO A UN MURO, no porque haya llegado a una esquina.

          1. No se aprende el sentido de las curvas: esos grados de rumbo no
             son de una curva.
          2. No se aplica el sesgo de curva: empujar hacia "donde giran las
             curvas" con el morro contra la pared es el giro sin sentido que
             se veia en pista justo despues de un esquive limpio.
          3. El centrado conserva toda su autoridad: es el termino que saca
             al carro del muro, y en curva se le baja al 25 %.

        espera_linea: el TCS esta vivo, o sea que la esquina la va a disparar
        la linea del piso (fsm.Estado.ESQUINA). Entonces el sesgo de curva de
        aqui se guarda como ULTIMO RECURSO y solo habla si el frente se cierra
        por debajo de `sesgo_desde_mm`. Sin esto el carro empieza a girar en
        cuanto la camara ve el muro de la esquina —a 820 mm— y llega a la
        linea ya torcido, que es lo que se veia como "recorta la curva". Con
        el TCS ausente no hay nada que esperar y el sesgo vuelve a mandar
        desde `dist_curva_mm`, como antes.
        """
        s = SalidaCarril()
        perfil = esc.perfil_mm
        rumbo = esc.perfil_rumbo_deg
        if perfil.size == 0 or rumbo.size != perfil.size:
            return s

        # --- que hay delante --------------------------------------------
        # El frente es un cono angular de verdad, no "el tercio central de la
        # imagen": asi no cambia de significado si se toca la resolucion.
        cono = float(self.cfg.get("cono_frente_deg", 12.0))
        frente = perfil[np.abs(rumbo) <= cono]
        s.dist_frente_mm = float(np.min(frente)) if frente.size else DIST_MAX_MM
        s.en_curva = s.dist_frente_mm < float(self.cfg.get("dist_curva_mm", 820.0))

        # --- termino 1: centrado, con separacion LATERAL de verdad --------
        # Antes se comparaba la distancia MINIMA del tercio izquierdo contra la
        # del derecho. Eso es distancia HACIA DELANTE, no separacion lateral:
        # respondia a lo lejos que esta el muro, no a lo pegado que va el carro.
        # Ahora cada sector con muro se convierte en un punto del suelo y se
        # mide de verdad cuanto hay a cada lado.
        ang = np.radians(rumbo)
        hay_muro = perfil < DIST_MAX_MM * 0.99
        x = perfil * np.sin(ang)        # lateral, + derecha
        y = perfil * np.cos(ang)        # hacia delante

        # UN MURO DE FRENTE NO ES UN MURO AL COSTADO, y confundirlos hacia
        # que el carro creyera tener las paredes encima en cada esquina. La cuenta
        # es facil de seguir: un muro frontal a 700 mm ocupa TODOS los rumbos,
        # y el sector que cae a 4 grados del morro da x = 700*sen(4) = 49 mm.
        # O sea que el codigo leia "muro a 49 mm a mi derecha" estando el carro
        # perfectamente centrado en un carril de 1000. De ahi salian un
        # centrado que respondia a la esquina en vez de a los costados y una
        # guardia anti-muro que empujaba sin que hubiera nada al lado.
        #
        # Lo que hay delante ya se mide aparte, en dist_frente_mm. Aqui solo
        # cuentan los solidos que quedan MAS CERCA que el frente: esos son los
        # que el carro va a pasar de largo por un costado, que es exactamente
        # lo que significa "separacion lateral".
        tope = min(float(self.cfg.get("ventana_centrado_mm", 1500.0)),
                   float(self.cfg.get("frac_frente_lateral", 0.85)) *
                   s.dist_frente_mm)
        ventana = hay_muro & (y > 120.0) & (perfil < tope)
        izq = x[ventana & (x < -20.0)]
        der = x[ventana & (x > 20.0)]
        # El muro mas cercano de cada lado: el x menos alejado de cero.
        s.lat_izq_mm = float(-np.max(izq)) if izq.size else None
        s.lat_der_mm = float(np.min(der)) if der.size else None

        objetivo = float(self.cfg.get("ancho_carril_mm", 1000.0)) / 2.0
        if s.lat_izq_mm is not None and s.lat_der_mm is not None:
            # Positivo = sobra sitio a la derecha = hay que ir a la derecha.
            s.err_centrado = float(np.clip(
                (s.lat_der_mm - s.lat_izq_mm) / objetivo, -1.0, 1.0))
        elif s.lat_izq_mm is not None:
            # Solo se ve un muro: se sigue a distancia fija de el. Es el caso
            # normal en las curvas, donde el muro exterior sale de cuadro.
            s.err_centrado = float(np.clip(
                (objetivo - s.lat_izq_mm) / objetivo, -1.0, 1.0))
        elif s.lat_der_mm is not None:
            s.err_centrado = float(np.clip(
                -(objetivo - s.lat_der_mm) / objetivo, -1.0, 1.0))

        # --- termino 2: rumbo hacia la salida -----------------------------
        # Suavizado con los vecinos para que un agujero de un solo sector (un
        # brillo en el muro, la junta entre dos piezas) no tire del volante.
        suave = np.convolve(perfil, np.ones(3) / 3.0, mode="same")

        # SE APUNTA AL CENTRO DEL HUECO, NO AL SECTOR MAS PROFUNDO.
        #
        # En una recta, media imagen esta libre y TODOS esos sectores empatan
        # en la distancia maxima. np.argmax devuelve el primero, o sea el mas
        # a la izquierda, asi que el carro pedia volante a la izquierda en
        # cada recta aunque estuviera perfectamente centrado. El sesgo no se
        # ve en un solo frame: se ve tres metros despues, contra el muro.
        #
        # Lo correcto es quedarse con el TRAMO CONTIGUO de sectores libres mas
        # ancho y apuntar a su centro. Contiguo y no "todos los libres" porque
        # con dos huecos separados —un pilar en medio del carril— la media de
        # los dos apunta justo al obstaculo que hay entre ellos.
        libre = suave >= 0.9 * float(suave.max())
        ini, mejor_ini, mejor_largo = None, 0, 0
        for i, v in enumerate(libre):
            if v and ini is None:
                ini = i
            elif not v and ini is not None:
                if i - ini > mejor_largo:
                    mejor_ini, mejor_largo = ini, i - ini
                ini = None
        if ini is not None and (libre.size - ini) > mejor_largo:
            mejor_ini, mejor_largo = ini, libre.size - ini

        if mejor_largo > 0:
            s.rumbo_hueco_deg = float(np.mean(
                rumbo[mejor_ini:mejor_ini + mejor_largo]))
        else:
            s.rumbo_hueco_deg = float(rumbo[int(np.argmax(suave))])
        tope = float(self.cfg.get("rumbo_max_deg", 30.0))
        s.err_rumbo = float(np.clip(s.rumbo_hueco_deg / tope, -1.0, 1.0))

        # --- el sentido de las curvas se APRENDE --------------------------
        # En una pista WRO todas las curvas van hacia el mismo lado: en sentido
        # horario todas a la derecha, en antihorario todas a la izquierda. O
        # sea que basta con acertar UNA para saber las once restantes.
        #
        # Se aprende del rumbo del hueco durante la primera curva, no del
        # giroscopio ni del color de las lineas. Motivo: el signo del gz
        # depende de como este montado el MPU, y el color depende del
        # parametro adivinado color_entrada_horario. El rumbo del hueco es la
        # misma magnitud con la que ya se esta conduciendo: si esa se
        # equivocara, el carro ya estaria perdido de todas formas.
        #
        # PERO SOLO SE APRENDE DE CURVAS DE VERDAD. Lo que cierra el frente
        # justo despues de rebasar un pilar no es una esquina: es el muro al
        # que el carro quedo apuntando. Si esos ciclos entran en el promedio,
        # el sentido que se "aprende" es el del ultimo esquive, y a partir de
        # ahi el sesgo de curva empuja hacia ese lado en todas las esquinas.
        # Eso es una vuelta perdida por un dato sucio, asi que con `tras_pilar`
        # se tira lo acumulado y no se concluye nada.
        if tras_pilar:
            self._acum_curva = 0.0
            self._n_curva = 0
        elif s.en_curva:
            self._acum_curva += s.rumbo_hueco_deg
            self._n_curva += 1
        elif self._en_curva_prev and self._n_curva > 3:
            medio = self._acum_curva / self._n_curva
            if abs(medio) > float(self.cfg.get("aprender_curva_deg", 6.0)):
                self.sentido_curva = 1 if medio > 0 else -1
            self._acum_curva = 0.0
            self._n_curva = 0
        self._en_curva_prev = s.en_curva and not tras_pilar

        # Mientras no se haya aprendido, vale la pista que dan las lineas del
        # piso: horario = curvas a la derecha.
        sentido = self.sentido_curva or int(np.sign(sentido_pista))
        s.sentido_curva = sentido

        # --- hay un muro encima? -------------------------------------------
        # Misma separacion que usa la guardia anti-muro, a proposito: es la
        # distancia a la que dejan de importar las trayectorias bonitas.
        umbral_muro = float(self.cfg.get("guardia_muro_mm", 240.0))
        laterales = [d for d in (s.lat_izq_mm, s.lat_der_mm) if d is not None]
        s.muro_encima = bool(laterales) and min(laterales) < umbral_muro

        # --- mezcla --------------------------------------------------------
        if s.en_curva and not s.muro_encima:
            # En curva no hay dos muros paralelos que centrar: el centrado
            # empuja contra la esquina interior. Se baja mucho su peso.
            kc = float(self.cfg.get("kp_centrado", 55.0)) * 0.25
            kr = float(self.cfg.get("kp_rumbo", 70.0)) * 1.35
            s.motivo = "curva"
        else:
            # CON UN MURO A MENOS DE UN PALMO MANDA EL CENTRADO, curva o no.
            # Bajarle el peso al 25 % tiene sentido cuando sobra sitio y lo
            # unico que hace es estorbar al trazado; con el muro encima es el
            # unico termino que aparta al carro, y callarlo ahi es como se
            # acaba raspando la pared en la salida de un esquive.
            kc = float(self.cfg.get("kp_centrado", 55.0))
            kr = float(self.cfg.get("kp_rumbo", 70.0))
            s.motivo = "muro encima" if s.muro_encima else "recta"

        direccion = kc * s.err_centrado + kr * s.err_rumbo

        # --- sesgo de curva: cuando el hueco no se decide ------------------
        # De frente a una esquina, el muro exterior puede tapar el hueco casi
        # entero y dejar el rumbo casi centrado. Ahi es donde el carro se iba
        # recto contra la pared. Si sabemos hacia donde giran las curvas de
        # esta ronda, se empuja hacia ese lado en vez de esperar a ver.
        #
        # DOS CANDADOS, Y LOS DOS SON EL MISMO FALLO DE PISTA: el carro
        # rebasaba un pilar bien y acto seguido daba un giro hacia el lado del
        # esquive hasta clavarse en la pared. El sesgo era el que lo daba, y lo
        # daba porque "frente cerrado" no significa "esquina":
        #
        #   - tras un rebase el frente esta cerrado por el muro al que el
        #     carro quedo apuntando. Ahi no hay esquina que anticipar.
        #   - y nunca, en ninguna situacion, se empuja HACIA un muro que ya
        #     esta a menos de `guardia_muro_mm`. El sesgo existe para resolver
        #     un hueco indeciso, no para meter el morro en la pared: si la
        #     esquina es de verdad, el termino de rumbo la resuelve igual en
        #     cuanto haya sitio.
        #
        # Y UN TERCER CANDADO: ESPERAR A LA LINEA. Quien manda en las esquinas
        # es el piso, no la camara — la linea cae donde de verdad empieza la
        # curva, y la camara ve el muro un metro antes. Girar con la camara es
        # recortar: el carro llega a la linea ya torcido. Asi que con el TCS
        # vivo este sesgo es solo la red por si el sensor se pierde la linea, y
        # no habla hasta que el frente esta tan cerca que ya no se puede
        # esperar mas.
        cerca_der = s.lat_der_mm is not None and s.lat_der_mm < umbral_muro
        cerca_izq = s.lat_izq_mm is not None and s.lat_izq_mm < umbral_muro
        muro_de_ese_lado = (sentido > 0 and cerca_der) or (sentido < 0 and cerca_izq)
        sin_margen = s.dist_frente_mm < float(
            self.cfg.get("sesgo_desde_mm", 520.0))
        toca_ya = sin_margen or not espera_linea
        if (s.en_curva and sentido != 0 and not tras_pilar
                and not muro_de_ese_lado and toca_ya):
            sesgo = float(self.cfg.get("sesgo_curva", 45.0))
            indeciso = abs(s.rumbo_hueco_deg) < float(
                self.cfg.get("hueco_indeciso_deg", 10.0))
            s.sesgo = sentido * sesgo * (1.0 if indeciso else 0.45)
            direccion += s.sesgo
            s.motivo = "curva " + ("(por sentido)" if indeciso else "(por hueco)")
        elif s.en_curva and sentido != 0:
            if tras_pilar:
                s.motivo = "curva (sesgo callado: tras pilar)"
            elif muro_de_ese_lado:
                s.motivo = "curva (sesgo callado: muro de ese lado)"
            else:
                s.motivo = "recto hasta la linea"


        # --- termino 3: amortiguacion con el giroscopio --------------------
        direccion -= float(self.cfg.get("kd_giro", 0.28)) * gz

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


def guardia_muro(s: SalidaCarril, cfg: Dict[str, Any]) -> float:
    """Empujon para no rozar un muro. NO centra: solo evita el golpe.

    Existe por un caso muy concreto. Mientras se adelanta un pilar, el
    seguidor de carril esta callado a proposito: si opinara, tiraria del carro
    hacia el centro del carril y la rueda trasera barreria el pilar. Pero
    callarlo del todo deja al carro sin nadie que mire los muros justo cuando
    va desviado hacia uno de ellos, que es como se acaba contra la pared
    despues de un esquive limpio.

    La solucion no es devolverle la voz al centrado —eso reintroduce el
    problema del pilar— sino dejar SOLO esta guardia, que se calla mientras
    haya sitio y solo habla cuando de verdad no lo hay.

    Devuelve % con signo (+ = empujar a la derecha).
    """
    umbral = float(cfg.get("guardia_muro_mm", 240.0))
    critico = float(cfg.get("muro_critico_mm", 130.0))
    err = 0.0
    if s.lat_izq_mm is not None and s.lat_izq_mm < umbral:
        err += (umbral - s.lat_izq_mm) / umbral        # muro izq cerca -> derecha
    if s.lat_der_mm is not None and s.lat_der_mm < umbral:
        err -= (umbral - s.lat_der_mm) / umbral        # muro der cerca -> izquierda
    salida = float(np.clip(err, -1.0, 1.0)) * float(cfg.get("k_guardia", 70.0))

    # REGIMEN CRITICO. Por debajo de muro_critico_mm ya no se pondera nada:
    # se manda al tope hacia el lado libre. El reglamento no distingue entre
    # rozar y estrellarse —9.18, tocar el muro y moverlo termina la ronda— y
    # un roce a 0.9 m/s mueve el muro. Aqui no hay nada que negociar.
    lados = [d for d in (s.lat_izq_mm, s.lat_der_mm) if d is not None]
    if lados:
        s.holgura_min_mm = min(lados)
        if s.holgura_min_mm < critico:
            s.muro_critico = True
            # Hacia el lado con mas sitio; si solo se ve un muro, lejos de el.
            if s.lat_izq_mm is not None and (s.lat_der_mm is None or
                                             s.lat_izq_mm < s.lat_der_mm):
                salida = max(salida, float(cfg.get("k_guardia", 70.0)))
            else:
                salida = min(salida, -float(cfg.get("k_guardia", 70.0)))
    return salida


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
