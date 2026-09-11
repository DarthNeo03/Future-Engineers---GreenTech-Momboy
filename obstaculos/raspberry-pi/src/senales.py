"""
senales.py — Rebasar las señales de transito por el lado que manda la regla.

REGLA DEL JUEGO (reglamento 2026, punto 9.19)
    pilar ROJO  -> se pasa por su DERECHA
    pilar VERDE -> se pasa por su IZQUIERDA

"Derecha" e "izquierda" son las del PILAR visto desde el carro que avanza, no
las del tapete. Por eso NO hay que invertir nada cuando la ronda se corre en
sentido contrario: el mismo pilar fisico se rebasa por un lado distinto en
horario que en antihorario, y eso sale solo de trabajar en el marco del carro.

MATIZ QUE SE OLVIDA SIEMPRE: "pasar por la derecha del pilar" NO es "girar a
la derecha". Si el pilar ya esta a la izquierda del carro, lo correcto puede
ser seguir recto o incluso corregir a la izquierda para no invadir el carril
contrario. Lo que se calcula no es un giro: es un PUNTO DE PASO.

LOS TRES ACTOS DE UN REBASE

  1. APROXIMACION. Se apunta a un punto al costado correcto del pilar,
     separado medio carro + medio pilar + margen. Se convierte en direccion
     con una MIRADA MINIMA: sin ella, al acercarse, el angulo al punto crece
     hasta pedir el volante a tope.

  2. COMPROMISO. Cuando el pilar queda muy cerca deja de verse: la camara no
     llega tan abajo. Si en ese momento el esquive desapareciera, el seguidor
     de carril tiraria del carro hacia el centro y la RUEDA TRASERA barreria
     el pilar — con direccion Ackermann la cola corta por dentro. Asi que
     desde que se pierde de vista se MANTIENE EL RUMBO, sin volver hacia el
     pilar, el tiempo que el carro necesita para adelantarlo con todo su
     largo. Ese tiempo se calcula con la velocidad real, no es un valor fijo.

     MANTENER EL RUMBO NO ES MANTENER EL VOLANTE, y confundirlo cuesta la
     ronda. Un volante fijo describe un ARCO: en la primera version se
     congelaban los ~59 % del ultimo frame con pilar a la vista y el carro
     seguia cerrando la curva hacia el lado por el que acababa de esquivar
     hasta comerse el muro. Lo que se congela es el YAW del MPU, y un
     proporcional devuelve el carro a el. Sin MPU, se suelta el volante
     progresivamente: peor, pero sigue sin cerrar el arco.

     Durante el compromiso el carril esta callado —si opinara, barreria el
     pilar— pero NO el todo: queda la guardia anti-muro de carril.py, que se
     calla mientras haya sitio y solo habla cuando de verdad no lo hay.

  3. SALIDA. El compromiso no termina soltando el carro donde este: TERMINA
     DEVOLVIENDOLO ALINEADO. En la primera version se sostenia el rumbo
     congelado hasta el ultimo instante y despues se le pasaba el mando al
     carril; el problema es que ese rumbo congelado apunta hacia el lado por
     el que se rebaso, asi que el carril recibia un carro cruzado y a un palmo
     del muro, y lo que veia delante era pared. Ahora, pasada la mitad del
     compromiso —cuando el pilar ya quedo atras y volver hacia el no barre
     nada—, el rumbo objetivo se lleva poco a poco del rumbo congelado al del
     PASILLO LIBRE que ve el carril. El carro sale del rebase mirando por donde
     tiene que seguir, que es lo unico que evita el volantazo de despues.

LO QUE ESTA MAS ALLA DE LA LINEA DEL PISO NO ES DE ESTA RECTA
Las lineas naranja y azul marcan el limite de seccion. Un pilar que se ve por
detras de ellas pertenece al tramo SIGUIENTE. Hacerle caso desde la recta tira
del carro justo cuando toca prepararse para la curva, y el carro se pega a la
esquina interior. Se descartan mientras se viene por la recta, y cuentan al
entrar en la curva.

RECUPERACION POR LADO INCORRECTO (Apendice A, seccion 5)
Empezar a rebasar por el lado equivocado NO termina la ronda: la ronda termina
cuando el carro cruza COMPLETAMENTE el radio (la linea muro interior - muro
exterior) donde esta la señal. O sea que hay un margen real para darse cuenta
y corregir, y el reglamento lo dice explicitamente. Este modulo lo usa.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from .geometria import DIST_MAX_MM
from .vision import Deteccion, Escena

# +1 = el carro debe quedar a la DERECHA del pilar; -1 = a su izquierda.
LADO_OBLIGADO = {"rojo": +1, "verde": -1}
MITAD_PILAR_MM = 25.0        # el pilar mide 50 x 50 mm en planta

def _envolver(grados: float) -> float:
    """Diferencia de rumbos llevada a -180..180. Sin esto, cruzar el +-180
    del yaw se lee como un giro de 350 grados y el carro da un volantazo."""
    return (float(grados) + 180.0) % 360.0 - 180.0


FASE_NADA = "nada"
FASE_APROXIMACION = "aproximacion"
FASE_COMPROMISO = "compromiso"
FASE_CORRECCION = "correccion"


@dataclass
class Maniobra:
    direccion: float = 0.0       # % con signo
    peso: float = 0.0            # 0 = manda el carril, 1 = manda el esquive
    fase: str = FASE_NADA
    color: str = ""
    lado: int = 0                # lado por el que se pasa (+1 derecha)
    dist_mm: float = DIST_MAX_MM
    lat_mm: float = 0.0
    objetivo_mm: float = 0.0     # lateral del punto de paso
    lado_incorrecto: bool = False
    pedir_reversa: bool = False
    rumbo_objetivo: Optional[float] = None  # yaw que se sostiene al adelantar
    err_rumbo_deg: float = 0.0
    progreso: float = 0.0                   # 0..1 dentro del compromiso
    saliendo: bool = False                  # ya reenfilando hacia el pasillo
    info: Dict[str, Any] = field(default_factory=dict)


class Esquivador:
    def __init__(self, cfg: Dict[str, Any], semiancho_mm: float = 130.0) -> None:
        self.cfg = cfg
        self.semiancho_mm = semiancho_mm
        self.reiniciar()

    def reiniciar(self) -> None:
        self._fase = FASE_NADA
        self._dir_prev = 0.0
        self._comp_hasta = 0.0
        self._comp_dur = 1.0
        self._comp_dir = 0.0
        self._comp_yaw: Optional[float] = None
        self._comp_color = ""
        self._mem_color = ""
        self._mem_lado = 0
        self._mem_t = 0.0

    # ------------------------------------------------------------ eleccion
    def elegir(self, esc: Escena, lineas_mm: Dict[str, Optional[float]],
               en_curva: bool) -> Optional[Deteccion]:
        """El pilar que toca atender AHORA, o None.

        No es "el mas cercano" a secas: hay que descartar lo que pertenece a
        la seccion siguiente y lo que todavia esta demasiado lejos para que
        valga la pena desviarse.
        """
        activar = float(self.cfg.get("activar_desde_mm", 1600.0))
        # Limite de seccion: la linea de piso mas cercana que se vea delante.
        #
        # DOS CAUTELAS, las dos aprendidas por las malas:
        #
        #  a) Una linea A PUNTO DE PISARSE es la que estamos cruzando AHORA,
        #     no la frontera de la seccion siguiente. Usarla como limite pone
        #     el tope a 30 cm y borra todos los pilares de la recta. Por eso
        #     solo cuentan las lineas que estan de verdad lejos.
        #  b) Esta puerta puede silenciar el esquive entera, asi que tiene
        #     interruptor. Si en la pista se ve que ignora pilares legitimos,
        #     se apaga en pista.json y se sigue corriendo.
        limite = DIST_MAX_MM
        if not en_curva and bool(self.cfg.get("usar_limite_seccion", True)):
            min_valida = float(self.cfg.get("linea_valida_desde_mm", 450.0))
            vistas = [d for d in lineas_mm.values()
                      if d is not None and d >= min_valida]
            if vistas:
                limite = min(vistas) + float(self.cfg.get("holgura_linea_mm", 80.0))

        candidatos = [p for p in esc.pilares
                      if p.color in LADO_OBLIGADO
                      and p.dist_mm <= activar
                      and p.dist_mm <= limite]
        if not candidatos:
            return None
        # Entre varios, el mas cercano. Con dos pilares seguidos, resolver el
        # primero bien deja el carro colocado para ver el segundo; intentar
        # planear los dos a la vez con una sola camara sale peor.
        return min(candidatos, key=lambda p: p.dist_mm)

    # ---------------------------------------------------------------- paso
    def paso(self, esc: Escena, lineas_mm: Dict[str, Optional[float]],
             en_curva: bool, vel_mm_s: float,
             yaw: Optional[float] = None, gz: float = 0.0,
             rumbo_hueco_deg: float = 0.0,
             ahora: Optional[float] = None) -> Maniobra:
        """rumbo_hueco_deg: hacia donde ve el carril el pasillo libre, en
        grados relativos al morro. Solo se usa en la SALIDA del compromiso,
        para devolver el carro alineado en vez de cruzado.

        ahora: reloj inyectable. Existe para las pruebas: el compromiso dura
        menos de dos segundos y sin poder adelantar el reloj no hay forma de
        comprobar en un selftest lo que hace al final — que es justamente
        donde estaba el fallo.
        """
        ahora = time.time() if ahora is None else float(ahora)
        m = Maniobra()
        objetivo = self.elegir(esc, lineas_mm, en_curva)

        # ---------------------------------------------- fase de compromiso
        # Se evalua ANTES de mirar si hay pilar: el compromiso existe
        # justamente porque el pilar ya no se ve.
        if objetivo is None and ahora < self._comp_hasta:
            m.fase = FASE_COMPROMISO
            m.color = self._comp_color
            m.peso = 1.0
            m.progreso = float(np.clip(
                1.0 - (self._comp_hasta - ahora) / max(1e-3, self._comp_dur),
                0.0, 1.0))

            # MANTENER EL RUMBO NO ES MANTENER EL VOLANTE.
            #
            # Aqui habia un fallo que se veia clarisimo en pista: se congelaba
            # el ANGULO DE VOLANTE del ultimo frame en que se vio el pilar
            # —unos 59 %— y se sostenia tres cuartos de segundo. Un volante
            # fijo no traza una recta: traza un ARCO. El carro seguia girando
            # hacia el lado por el que acababa de esquivar y se comia el muro.
            #
            # Lo que hay que congelar es el RUMBO: se guarda el yaw del
            # instante en que el pilar entro en la zona ciega y se vuelve a
            # el con un proporcional. Asi el carro sigue DERECHO en la
            # direccion que llevaba, que es lo que hace falta para adelantar
            # al pilar sin barrerlo con la rueda trasera.
            if self._comp_yaw is not None and yaw is not None:
                # SALIDA: del rumbo congelado al rumbo del pasillo.
                #
                # Sostener el rumbo congelado hasta el ultimo instante resuelve
                # el adelantamiento y crea el problema siguiente: ese rumbo
                # apunta hacia el lado por el que se rebaso, asi que al acabar
                # el compromiso el carro le llega al seguidor de carril
                # cruzado y con el muro delante. Pasada `salida_desde` del
                # compromiso el pilar ya quedo atras —volver hacia el no barre
                # nada con la cola— y el objetivo se lleva poco a poco hasta
                # el rumbo del pasillo libre. Con peso 1 al final, el error de
                # rumbo ES el rumbo del hueco: el relevo con el carril queda
                # sin escalon.
                objetivo_yaw = self._comp_yaw
                desde = float(self.cfg.get("salida_desde", 0.55))
                if m.progreso > desde:
                    w = min(1.0, (m.progreso - desde) / max(1e-3, 1.0 - desde))
                    hacia_pasillo = yaw + float(rumbo_hueco_deg)
                    objetivo_yaw = self._comp_yaw + w * _envolver(
                        hacia_pasillo - self._comp_yaw)
                    m.saliendo = True
                err = _envolver(objetivo_yaw - yaw)
                m.rumbo_objetivo = objetivo_yaw
                m.err_rumbo_deg = err
                direccion = (float(self.cfg.get("kp_rumbo_compromiso", 2.6)) * err
                             - float(self.cfg.get("kd_rumbo_compromiso", 0.22)) * gz)
            else:
                # SIN MPU NO HAY RUMBO QUE SOSTENER, y entonces lo unico
                # honesto es enderezar. Antes el volante se desvanecia a lo
                # largo de TODO el compromiso, o sea que se mantenia casi
                # entero durante la primera mitad: un volante fijo no traza
                # una recta, traza un arco, y ese arco es el giro hacia el
                # lado del esquive que terminaba contra la pared. Ahora se
                # suelta en `soltar_sin_mpu_s`, tiempo de reloj y no fraccion
                # del compromiso, y el resto del adelantamiento se hace con
                # las ruedas rectas: sin rumbo que seguir, recto es lo mejor
                # que se puede hacer, y la guardia anti-muro sigue despierta.
                soltar = max(0.05, float(self.cfg.get("soltar_sin_mpu_s", 0.35)))
                transcurrido = m.progreso * self._comp_dur
                direccion = self._comp_dir * max(0.0, 1.0 - transcurrido / soltar)
                m.info["sin_mpu"] = True

            m.direccion = float(np.clip(direccion, -100.0, 100.0))
            m.info["queda_s"] = round(self._comp_hasta - ahora, 2)
            self._fase = FASE_COMPROMISO
            self._dir_prev = m.direccion
            return m

        if objetivo is None:
            self._fase = FASE_NADA
            self._comp_yaw = None        # el compromiso termino: rumbo libre
            self._dir_prev *= 0.6        # suelta el volante sin dar un tiron
            return m

        # ------------------------------------------------------- geometria
        lado = LADO_OBLIGADO[objetivo.color]
        m.color = objetivo.color
        m.lado = lado
        m.dist_mm = objetivo.dist_mm
        m.lat_mm = objetivo.lat_mm

        # Punto de paso: al costado correcto del pilar, separado lo que ocupa
        # medio carro + medio pilar + el margen de seguridad.
        margen = float(self.cfg.get("margen_mm", 70.0))
        m.objetivo_mm = objetivo.lat_mm + lado * (MITAD_PILAR_MM +
                                                  self.semiancho_mm + margen)

        # ¿Estamos ya del lado correcto? El carro esta en x = 0; el pilar en
        # lat_mm. Si lat_mm * lado < 0, el pilar queda al otro lado: bien.
        holgura_actual = -objetivo.lat_mm * lado    # >0 = vamos bien
        # PERO ESTO SOLO SE JUZGA DE CERCA.
        #
        # Un pilar visto de frente a 1.5 m da holgura ~0, y eso no es un error:
        # es una aproximacion normal, todavia hay metro y medio para colocarse.
        # Juzgarlo desde lejos metia el carro en CORRECCION —al 22 % de
        # velocidad y con el volante casi a tope— en CADA pilar, que es de
        # donde venia la sensacion de que el carro se traba. La correccion es
        # para cuando ya no queda sitio, no para cuando aun sobra.
        cerca = objetivo.dist_mm <= float(self.cfg.get("juzgar_lado_desde_mm", 700.0))
        m.lado_incorrecto = cerca and holgura_actual < (
            MITAD_PILAR_MM + self.semiancho_mm * 0.35)

        # --------------------------------------------------------- direccion
        morro = float(self.cfg.get("morro_mm", 60.0))
        mirada = max(float(self.cfg.get("mirada_min_mm", 420.0)),
                     objetivo.dist_mm - morro)
        ang = math.degrees(math.atan2(m.objetivo_mm, mirada))
        k = float(self.cfg.get("ganancia", 1.9))
        direccion = k * ang

        # ----------------------------------------- recuperacion por mal lado
        # Mientras no se haya cruzado el radio del pilar, corregir es legal y
        # es lo correcto. Cuanto mas cerca, mas agresivo hay que ser.
        dist_reversa = float(self.cfg.get("dist_reversa_mm", 230.0))
        if m.lado_incorrecto:
            m.fase = FASE_CORRECCION
            urgencia = 1.0 - min(1.0, objetivo.dist_mm / 900.0)
            direccion = lado * (55.0 + 45.0 * urgencia)
            # Demasiado cerca para meter el morro por el lado bueno: no hay
            # radio de giro que valga. Se pide reversa; quien manda decide.
            if objetivo.dist_mm < dist_reversa:
                m.pedir_reversa = True
        else:
            m.fase = FASE_APROXIMACION

        # ------------------------------------------------------------- peso
        # Transicion suave entre "manda el carril" y "manda el esquive": un
        # salto brusco de autoridad se ve como un volantazo a 1.5 m del pilar.
        activar = float(self.cfg.get("activar_desde_mm", 1600.0))
        mandar = float(self.cfg.get("mandar_desde_mm", 750.0))
        if objetivo.dist_mm <= mandar:
            m.peso = 1.0
        else:
            t = (activar - objetivo.dist_mm) / max(1.0, activar - mandar)
            m.peso = float(min(1.0, max(0.0, t)))
        if m.fase == FASE_CORRECCION:
            m.peso = 1.0            # corregir no se negocia con el carril

        # --------------------------------------------------------- suavizado
        alfa = float(self.cfg.get("suavizado", 0.5))
        direccion = alfa * direccion + (1.0 - alfa) * self._dir_prev
        self._dir_prev = direccion
        m.direccion = float(max(-100.0, min(100.0, direccion)))

        # ------------------------------------------- armar el compromiso
        # Desde que el pilar entra en la zona ciega se guarda cuanto tiempo
        # hay que sostener este rumbo para adelantarlo entero.
        ciego = float(self.cfg.get("ciego_desde_mm", 400.0))
        if objetivo.dist_mm <= ciego and not m.pedir_reversa:
            largo = float(self.cfg.get("largo_carro_mm", 200.0))
            recorrido = objetivo.dist_mm + largo + float(self.cfg.get("extra_mm", 120.0))
            v = max(120.0, vel_mm_s)      # nunca dividir por una velocidad ~0
            dur = min(float(self.cfg.get("compromiso_max_s", 1.6)), recorrido / v)
            self._comp_hasta = ahora + dur
            self._comp_dur = dur
            self._comp_dir = m.direccion
            # El rumbo de AHORA es el que hay que sostener mientras se adelanta.
            if yaw is not None and self._comp_yaw is None:
                self._comp_yaw = yaw
            self._comp_color = objetivo.color

        self._fase = m.fase
        self._mem_color = objetivo.color
        self._mem_lado = lado
        self._mem_t = ahora
        m.info = {
            "holgura_mm": round(holgura_actual),
            "mirada_mm": round(mirada),
            "angulo_deg": round(ang, 1),
        }
        return m

    # ------------------------------------------------------------ estado
    def ocupado(self) -> bool:
        """Hay un rebase en marcha (aproximacion, compromiso o correccion).

        Lo pregunta el piloto para avisar al seguidor de carril con
        `tras_pilar`: mientras esto sea cierto, lo que cierra el frente puede
        ser el pilar o el muro al que el carro apunta, no una esquina.
        """
        return self._fase != FASE_NADA

    # ------------------------------------------------------------ memoria
    def memoria_viva(self, ahora: Optional[float] = None) -> bool:
        """El ultimo pilar sigue contando aunque ya no se vea. Sirve sobre todo
        en la curva: el pilar sale de cuadro al girar, y sin memoria el carro
        se olvida de que lo tiene al lado justo cuando mas cerca esta."""
        if not self._mem_color:
            return False
        ahora = ahora or time.time()
        return (ahora - self._mem_t) <= float(self.cfg.get("memoria_s", 3.0))

    def memoria(self) -> Dict[str, Any]:
        return {"color": self._mem_color,
                "lado": "derecha" if self._mem_lado > 0 else "izquierda",
                "edad_s": round(time.time() - self._mem_t, 2) if self._mem_t else None}


def combinar(dir_carril: float, maniobra: Maniobra,
             empujon_magenta: Optional[float]) -> float:
    """Mezcla final de las tres autoridades que pueden mover el volante.

    El orden no es arbitrario:
      - el esquive se MEZCLA con el carril segun su peso (transicion suave),
      - el empujon del magenta se SUMA por encima de todo, porque tocarlo
        termina la ronda y no hay nada que negociar con eso.
    """
    d = (1.0 - maniobra.peso) * dir_carril + maniobra.peso * maniobra.direccion
    if empujon_magenta is not None:
        d += empujon_magenta
    return float(max(-100.0, min(100.0, d)))
