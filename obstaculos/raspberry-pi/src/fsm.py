"""
fsm.py — La maquina de estados que decide QUE esta haciendo el carro.

POR QUE UNA FSM Y NO UN MONTON DE ifs
Un lazo con condiciones sueltas funciona hasta el dia en que dos condiciones
son ciertas a la vez: "hay un pilar cerca" y "el frente se cerro, es una
curva". Sin estados, el carro alterna entre las dos respuestas a 30 Hz y se
queda temblando delante del pilar. Con estados, esa situacion tiene UN nombre,
UNA salida y UNA condicion de salida, y ademas se puede leer en la telemetria:
cuando algo sale mal en la pista, el registro dice en que estado estaba.

REGLA DE ORO DE ESTA FSM: LAS TRANSICIONES SON DE UN SOLO SENTIDO Y CON
HISTERESIS. Nunca se vuelve de ESQUIVE a PISTA porque el pilar "ya no se ve"
—es justo cuando no se ve cuando hay que seguir esquivando— sino porque se
cumplio el compromiso. Todo lo que pueda oscilar lleva tiempo minimo.

    ESPERA ──boton──> ARRANQUE ──asentado──> PISTA
                                               │
            ┌──────────────────────────────────┤
            │                                  │
            v                                  v
         SENAL ──cerca──> ESQUIVE ──cumplido──> PISTA
            │                 │
            │lado malo        │
            v                 │
        CORRECCION ──sin sitio──> REVERSA ──hueco──> SENAL
            │
            └──corregido──> SENAL

    PISTA ──3 vueltas + distancia──> META ──quieto──> FIN
    cualquiera ──enlace o camara caidos──> FALLO ──recuperado──> PISTA

ESPERA existe por el reglamento, no por comodidad: 9.11 obliga a que el
vehiculo quede esperando el boton de inicio despues de encenderse, y 9.13 a
que el movimiento empiece justo al pulsarlo.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional

from . import protocolo as proto
from .carril import SalidaCarril
from .senales import FASE_COMPROMISO, FASE_CORRECCION, Maniobra
from .vision import Escena
from .vueltas import EstadoVueltas


# Bit 0 del byte aux del mando: avisa al firmware de que estamos en maniobra
# fina. Hoy solo viaja para que quede en el registro de telemetria; el firmware
# lo tiene reservado para endurecer la rampa del motor si hiciera falta.
AUX_LENTO = 0x01


class Estado(str, Enum):
    ESPERA = "espera"          # encendido, quieto, esperando el boton (9.11)
    ARRANQUE = "arranque"      # boton pulsado, asentando antes de rodar
    PISTA = "pista"            # avanzando y centrandose en el carril
    SENAL = "senal"            # señal detectada, aproximando al punto de paso
    ESQUIVE = "esquive"        # rebasando a ciegas, compromiso en curso
    CORRECCION = "correccion"  # se iba por el lado malo, corrigiendo
    REVERSA = "reversa"        # sin radio para corregir: retroceder y repetir
    META = "meta"              # tres vueltas hechas, buscando donde parar
    FIN = "fin"                # parado en la seccion de meta
    FALLO = "fallo"            # sin camara o sin ESP32: no se conduce a ciegas


@dataclass
class Contexto:
    t: float = 0.0
    enlace_ok: bool = False
    hay_frame: bool = False
    arranque_pedido: bool = False       # flanco del boton visto en esta vuelta
    cortado_por_boton: bool = False
    sens: proto.Sensores = field(default_factory=proto.Sensores)
    escena: Escena = field(default_factory=Escena)
    carril: SalidaCarril = field(default_factory=SalidaCarril)
    maniobra: Maniobra = field(default_factory=Maniobra)
    vueltas: EstadoVueltas = field(default_factory=EstadoVueltas)
    vel_mm_s: float = 0.0
    direccion_mezclada: float = 0.0
    velocidad_sugerida: float = 0.0


@dataclass
class Orden:
    """Lo que se le pide al ESP32 este ciclo. Es la unica salida de la FSM."""
    vel: float = 0.0
    direccion: float = 0.0
    armado: bool = False
    parada: bool = True
    centrar: bool = True
    vmax: int = 255
    aux: int = 0          # bit0 = maniobra fina; viaja al firmware en el mando
    nota: str = ""


class MaquinaEstados:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self.estado = Estado.ESPERA
        self.t_estado = time.time()
        self.t_arranque: Optional[float] = None
        self.t_fin: Optional[float] = None
        self._t_reversa_hasta = 0.0
        self._t_atasco_desde: Optional[float] = None
        self._frente_atasco_mm: Optional[float] = None
        self._historial: list = []

    # ------------------------------------------------------------------
    def _ir(self, nuevo: Estado, motivo: str = "") -> None:
        if nuevo == self.estado:
            return
        ahora = time.time()
        self._historial.append((round(ahora - (self.t_arranque or ahora), 2),
                                self.estado.value, nuevo.value, motivo))
        del self._historial[:-60]
        self.estado = nuevo
        self.t_estado = ahora

    @property
    def en_estado_s(self) -> float:
        return time.time() - self.t_estado

    @property
    def tiempo_ronda_s(self) -> float:
        return 0.0 if self.t_arranque is None else time.time() - self.t_arranque

    # ------------------------------------------------------------------
    def paso(self, c: Contexto) -> Orden:
        """Una transicion + una orden. Sin efectos secundarios fuera de self."""
        # --- condiciones que mandan sobre cualquier estado ----------------
        # Perder la camara o el ESP32 no es "seguir con lo ultimo que sabia":
        # es dejar de conducir. Un carro ciego a 0.8 m/s recorre 80 cm en el
        # segundo que se tarda en darse cuenta.
        if self.estado not in (Estado.ESPERA, Estado.FIN):
            if not c.enlace_ok or not c.hay_frame:
                self._ir(Estado.FALLO, "sin enlace" if not c.enlace_ok else "sin frame")
            elif c.cortado_por_boton:
                self._ir(Estado.ESPERA, "corte por boton")

        manejador = {
            Estado.ESPERA: self._espera,
            Estado.ARRANQUE: self._arranque,
            Estado.PISTA: self._pista,
            Estado.SENAL: self._senal,
            Estado.ESQUIVE: self._esquive,
            Estado.CORRECCION: self._correccion,
            Estado.REVERSA: self._reversa,
            Estado.META: self._meta,
            Estado.FIN: self._fin,
            Estado.FALLO: self._fallo,
        }[self.estado]
        return manejador(c)

    # ============================================================ estados
    def _espera(self, c: Contexto) -> Orden:
        # El carro no se mueve ni un milimetro hasta que se pulse el boton.
        # El servo SI se centra: dejar las ruedas rectas ayuda al juez a
        # colocarlo y no cuenta como movimiento.
        if c.arranque_pedido and c.enlace_ok and c.hay_frame:
            if c.sens.calibrando:
                return Orden(nota="giroscopio calibrando: no mover")
            self.t_arranque = time.time()
            self._ir(Estado.ARRANQUE, "boton de inicio")
        return Orden(nota="esperando boton de inicio")

    def _arranque(self, c: Contexto) -> Orden:
        # Medio segundo de arranque suave con el volante recto. Sirve para que
        # el odometro empiece con una velocidad creible y para que la primera
        # decision de vision no se tome con el carro aun quieto.
        t = float(self.cfg.get("arranque_s", 0.4))
        if self.en_estado_s >= t:
            self._ir(Estado.PISTA, "asentado")
        return Orden(vel=float(self.cfg.get("vel_arranque", 26.0)), direccion=0.0,
                     armado=True, parada=False, centrar=False,
                     nota="arrancando")

    def _pista(self, c: Contexto) -> Orden:
        if self._parada_por_vueltas(c):
            return self._meta(c)
        if self._detectar_atasco(c):
            return self._entrar_reversa("atascado contra algo")
        if c.maniobra.fase == FASE_CORRECCION:
            self._ir(Estado.CORRECCION, "lado incorrecto")
            return self._correccion(c)
        if c.maniobra.peso > 0.15:
            self._ir(Estado.SENAL, f"pilar {c.maniobra.color}")
            return self._senal(c)
        return Orden(vel=c.velocidad_sugerida, direccion=c.direccion_mezclada,
                     armado=True, parada=False, centrar=False,
                     nota="curva" if c.carril.en_curva else "recta")

    def _senal(self, c: Contexto) -> Orden:
        if self._parada_por_vueltas(c):
            return self._meta(c)
        if c.maniobra.fase == FASE_CORRECCION:
            self._ir(Estado.CORRECCION, "lado incorrecto")
            return self._correccion(c)
        if c.maniobra.fase == FASE_COMPROMISO:
            self._ir(Estado.ESQUIVE, "pilar en zona ciega")
            return self._esquive(c)
        if c.maniobra.peso <= 0.05 and self.en_estado_s > 0.3:
            self._ir(Estado.PISTA, "pilar resuelto o perdido")
            return self._pista(c)
        # Aproximando: se baja la velocidad para que la camara tenga mas
        # frames por metro justo donde la decision es mas fina.
        v = min(c.velocidad_sugerida, float(self.cfg.get("vel_senal", 32.0)))
        return Orden(vel=v, direccion=c.direccion_mezclada, armado=True,
                     parada=False, centrar=False, aux=AUX_LENTO,
                     nota=f"rebasando {c.maniobra.color} por su "
                          f"{'derecha' if c.maniobra.lado > 0 else 'izquierda'}")

    def _esquive(self, c: Contexto) -> Orden:
        # Aqui el pilar YA NO SE VE. Mandar el compromiso y nada mas: si se
        # deja que el carril opine, tira del carro hacia el centro y la rueda
        # trasera barre el pilar.
        if c.maniobra.fase != FASE_COMPROMISO:
            self._ir(Estado.PISTA, "compromiso cumplido")
            return self._pista(c)
        v = min(c.velocidad_sugerida, float(self.cfg.get("vel_esquive", 30.0)))
        return Orden(vel=v, direccion=c.maniobra.direccion, armado=True,
                     parada=False, centrar=False,
                     nota=f"compromiso {c.maniobra.color}")

    def _correccion(self, c: Contexto) -> Orden:
        if c.maniobra.pedir_reversa:
            return self._entrar_reversa("sin radio para corregir")
        if c.maniobra.fase != FASE_CORRECCION:
            self._ir(Estado.SENAL, "lado recuperado")
            return self._senal(c)
        # Corregir despacio: el margen del reglamento (Apendice A.5) es hasta
        # cruzar el radio del pilar, no hasta tocarlo. Ir lento alarga ese
        # margen en tiempo, que es lo unico que se puede comprar aqui.
        v = float(self.cfg.get("vel_correccion", 22.0))
        return Orden(vel=v, direccion=c.maniobra.direccion, armado=True,
                     parada=False, centrar=False,
                     nota=f"corrigiendo lado de {c.maniobra.color}")

    def _entrar_reversa(self, motivo: str) -> Orden:
        self._t_reversa_hasta = time.time() + float(self.cfg.get("reversa_s", 0.9))
        self._ir(Estado.REVERSA, motivo)
        return Orden(vel=0.0, direccion=0.0, armado=True, parada=False,
                     centrar=True, nota="frenando para reversa")

    def _reversa(self, c: Contexto) -> Orden:
        if time.time() >= self._t_reversa_hasta:
            self._t_atasco_desde = None
            self._ir(Estado.PISTA, "reversa terminada")
            return self._pista(c)
        # Se retrocede girando HACIA el obstaculo, para que el morro se separe:
        # es el mismo truco que sacar un coche de un hueco de aparcamiento.
        # El firmware no invierte de golpe: pasa por cero y espera (seguridad.h).
        lado = c.maniobra.lado if c.maniobra.lado else 1
        return Orden(vel=-float(self.cfg.get("vel_reversa", 24.0)),
                     direccion=float(self.cfg.get("dir_reversa", 70.0)) * lado,
                     armado=True, parada=False, centrar=False,
                     nota="reversa")

    def _meta(self, c: Contexto) -> Orden:
        self._ir(Estado.META, "tres vueltas")
        if c.vueltas.listo_para_parar:
            self.t_fin = time.time()
            self._ir(Estado.FIN, c.vueltas.motivo or "en la seccion de meta")
            return self._fin(c)
        # Ultimo tramo: despacio y centrado, para que la proyeccion del carro
        # quede ENTERA dentro de la seccion (9.25.2 pide parada limpia).
        v = min(c.velocidad_sugerida, float(self.cfg.get("vel_meta", 26.0)))
        return Orden(vel=v, direccion=c.direccion_mezclada, armado=True,
                     parada=False, centrar=False, nota="buscando la meta")

    def _fin(self, c: Contexto) -> Orden:
        # Parado y DESARMADO. La nota 2 del 9.25.2 avisa: si el carro sigue
        # moviendose 15 s despues, los jueces pueden no dar los puntos de
        # parada. Desarmar quita cualquier duda.
        return Orden(vel=0.0, direccion=0.0, armado=False, parada=True,
                     centrar=True, nota="ronda terminada")

    def _fallo(self, c: Contexto) -> Orden:
        if c.enlace_ok and c.hay_frame:
            # Volver a PISTA, no al estado donde se estaba: lo que hubiera en
            # curso (un compromiso, una correccion) se calculo con datos que
            # ya son viejos.
            self._ir(Estado.PISTA, "recuperado")
            return self._pista(c)
        return Orden(nota="sin camara o sin ESP32")

    # ========================================================== auxiliares
    def _parada_por_vueltas(self, c: Contexto) -> bool:
        return bool(c.vueltas.vueltas >= int(self.cfg.get("vueltas", 3)))

    def _detectar_atasco(self, c: Contexto) -> bool:
        """Se pide velocidad, hay algo delante, y el mundo no se mueve.

        OJO CON LA SEÑAL QUE SE USA. El carro no tiene encoder: c.vel_mm_s se
        DEDUCE del porcentaje de mando, asi que vale para el odometro pero es
        exactamente la señal que NO sirve aqui — si el carro esta clavado
        contra un muro, el mando sigue pidiendo 40 % y la velocidad estimada
        sigue diciendo 880 mm/s. Preguntarle a eso si estamos atascados es
        preguntarle al acelerador si el coche se mueve.

        Lo unico que de verdad se mueve o no se mueve es LA IMAGEN: si el
        frente lleva un segundo largo a 20 cm y esa distancia no cambia
        mientras se pide marcha adelante, o hay un muro o hay un pilar caido.

        El reglamento permite UNA accion de reparacion por ronda (9.24) y
        parte la puntuacion por la mitad. Salir solo de un atasco vale, en la
        practica, media ronda.
        """
        ahora = time.time()
        pidiendo = c.velocidad_sugerida > 5.0
        cerca = c.carril.dist_frente_mm < float(self.cfg.get("atasco_dist_mm", 260.0))
        if not (pidiendo and cerca):
            self._t_atasco_desde = None
            self._frente_atasco_mm = None
            return False

        if self._t_atasco_desde is None:
            self._t_atasco_desde = ahora
            self._frente_atasco_mm = c.carril.dist_frente_mm
            return False

        # Si el frente se ha despejado mas de 'movimiento_mm', el carro SI se
        # esta moviendo: se reinicia la cuenta.
        movimiento = float(self.cfg.get("atasco_movimiento_mm", 45.0))
        if abs(c.carril.dist_frente_mm - (self._frente_atasco_mm or 0.0)) > movimiento:
            self._t_atasco_desde = ahora
            self._frente_atasco_mm = c.carril.dist_frente_mm
            return False

        return (ahora - self._t_atasco_desde) > float(self.cfg.get("atasco_s", 1.2))

    # ------------------------------------------------------------------
    def resumen(self) -> Dict[str, Any]:
        return {
            "estado": self.estado.value,
            "en_estado_s": round(self.en_estado_s, 2),
            "ronda_s": round(self.tiempo_ronda_s, 1),
            "transiciones": self._historial[-8:],
        }
