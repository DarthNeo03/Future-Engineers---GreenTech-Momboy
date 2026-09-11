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
         SENAL ──cerca──> ESQUIVE ──cumplido──> REINCORPORACION ──> PISTA
            │                 │                      │
            │lado malo        │                      └──otro pilar──> SENAL
            v                 │
        CORRECCION ──sin sitio──> REVERSA ──hueco──> SENAL
            │
            └──corregido──> SENAL

    PISTA ──linea de esquina + frente cerrado──> ESQUINA ──girado──> PISTA
    PISTA ──3 vueltas + distancia──> META ──quieto──> FIN
    cualquiera ──enlace o camara caidos──> FALLO ──recuperado──> PISTA

QUIEN MANDA EN LAS ESQUINAS ES LA LINEA DEL PISO, NO LA CAMARA. La camara solo
sabe que "el frente se cerro", y eso le pasa en una esquina igual que cuando el
carro quedo apuntando a un muro despues de rebasar un pilar: por eso giraba
antes de tiempo y por eso hacia un giro a medias, porque el seguidor de carril
negocia cada ciclo con el centrado en vez de comprometerse. La linea del piso
no se presta a esa confusion —o se pisa o no— y ademas cae donde de verdad
empieza la curva. Asi que la linea dice CUANDO, la camara confirma que la
esquina esta ahi de verdad (el frente tiene que estar cerrado), y el sentido de
la ronda dice HACIA DONDE. Entre esquina y esquina el carro va recto.

POR QUE HAY UN ESTADO ENTRE EL ESQUIVE Y LA PISTA. Porque lo que sale de un
rebase no es un carro en pista: es un carro descolocado. Va pegado a un muro,
apuntando hacia el, y por eso su camara ve el frente cerrado. Devolverlo
directamente a PISTA hacia que el seguidor de carril leyera "frente cerrado =
esquina", bajara el centrado al 25 % y aplicara el sesgo hacia el lado de las
curvas: el giro brusco contra la pared que se veia en pista justo despues de
un esquive limpio. REINCORPORACION es el estado que dice, explicitamente,
"todavia me estoy recolocando": el carril no interpreta esquinas, la guardia
anti-muro sigue mandando y solo se vuelve a PISTA cuando el carro esta de
verdad centrado y enfilado. Un pilar nuevo lo interrumpe, claro — recolocarse
no puede ser una excusa para ignorar la siguiente señal.

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
from .geometria import envolver_grados
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
    REINCORPORACION = "reincorporacion"   # rebasado: volviendo al carril
    ESQUINA = "esquina"        # linea pisada: giro comprometido de la curva
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
    # Empujon anti-muro. Se usa en los estados donde el seguidor de carril
    # esta callado o a medio mandar (ESQUIVE, CORRECCION, REINCORPORACION):
    # ver carril.guardia_muro.
    guardia_muro: float = 0.0


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
        self._yaw_recto: Optional[float] = None     # rumbo que se sostiene
        self._yaw_esquina: Optional[float] = None   # yaw al entrar en la curva
        self._dist_ultima_esquina_mm = -1e9         # una esquina, un giro
        self._sentido_esquina = 0
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
            Estado.REINCORPORACION: self._reincorporacion,
            Estado.ESQUINA: self._esquina,
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
        if self._toca_esquina(c):
            return self._entrar_esquina(c)
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
            # EL RUMBO QUE SE VA A SOSTENER. El compromiso termina reenfilando
            # hacia el pasillo, asi que el rumbo de ESTE instante es el bueno:
            # se congela aqui y no se vuelve a tocar hasta salir del estado.
            self._yaw_recto = c.sens.yaw if c.sens.mpu_ok else None
            self._ir(Estado.REINCORPORACION, "compromiso cumplido")
            return self._reincorporacion(c)
        v = min(c.velocidad_sugerida, float(self.cfg.get("vel_esquive", 30.0)))
        # La guardia sube con el progreso del adelantamiento: al principio el
        # pilar sigue al costado y corregir hacia el seria barrerlo con la
        # cola; al final ya quedo atras y lo unico que importa es el muro.
        peso = 0.3 + 0.7 * c.maniobra.progreso
        direccion = c.maniobra.direccion + peso * c.guardia_muro
        return Orden(vel=v, direccion=max(-100.0, min(100.0, direccion)),
                     armado=True, parada=False, centrar=False,
                     nota=f"compromiso {c.maniobra.color}")

    def _reincorporacion(self, c: Contexto) -> Orden:
        """Ya se rebaso el pilar. Ahora TOCA IR RECTO.

        Y "recto" hay que tomarselo al pie de la letra, porque la version
        anterior no lo hacia: dejaba mandar al seguidor de carril, y el carril
        no va recto — va al CENTRO. Son cosas distintas. Un carro que sale del
        rebase desplazado hacia un lado y recibe la orden de centrarse hace
        una ese: se cruza hacia el centro, se pasa, y vuelve. Eso es la
        desviacion que se seguia viendo justo despues de esquivar, y no venia
        de que el carril estuviera mal, sino de haberle pedido lo que no era.

        Asi que aqui se sostiene el RUMBO con el yaw, igual que en el
        compromiso. El rumbo bueno ya lo dejo puesto el propio compromiso, que
        termina reenfilando hacia el pasillo libre. Lo unico que se le suma es
        la guardia anti-muro: no centra, solo impide el golpe. Recentrarse es
        cosa de la recta siguiente, con sitio y sin prisa.

        Sin MPU no hay rumbo que sostener y no queda mas remedio que dejar
        mandar al carril; se nota, y es otro motivo para tener el I2C sano.

        Se sale cuando el carro esta enfilado y despegado del muro, cuando la
        linea del piso dice que empieza una curva, o cuando se acaba el
        tiempo.
        """
        if self._parada_por_vueltas(c):
            return self._meta(c)
        if self._detectar_atasco(c):
            return self._entrar_reversa("atascado despues de rebasar")
        # Un pilar nuevo manda sobre la reincorporacion: recolocarse no puede
        # ser motivo para pasar de largo la señal siguiente.
        if c.maniobra.fase == FASE_CORRECCION:
            self._ir(Estado.CORRECCION, "lado incorrecto")
            return self._correccion(c)
        if c.maniobra.fase == FASE_COMPROMISO:
            self._ir(Estado.ESQUIVE, "otro compromiso")
            return self._esquive(c)
        if c.maniobra.peso > 0.15:
            self._ir(Estado.SENAL, f"pilar {c.maniobra.color}")
            return self._senal(c)

        # Si la linea llega mientras todavia se esta recolocando, la curva
        # manda: para eso se espero a la linea.
        if self._toca_esquina(c):
            return self._entrar_esquina(c)

        # SE SALE POR ESTAR ENFILADO, NO POR ESTAR CENTRADO. Exigir centrado
        # mientras se va recto a proposito es pedir algo que este estado no
        # hace: se saldria siempre por tiempo. Centrarse es cosa de la recta.
        enfilado = abs(c.carril.rumbo_hueco_deg) < float(
            self.cfg.get("reincorporado_deg", 14.0))
        agotado = self.en_estado_s > float(
            self.cfg.get("reincorporacion_max_s", 1.2))
        if (enfilado and not c.carril.muro_encima) or agotado:
            self._yaw_recto = None
            self._ir(Estado.PISTA,
                     "tiempo de reincorporacion" if agotado else "enfilado")
            return self._pista(c)

        v = min(c.velocidad_sugerida,
                float(self.cfg.get("vel_reincorporacion", 34.0)))
        if self._yaw_recto is not None and c.sens.mpu_ok:
            err = envolver_grados(self._yaw_recto - c.sens.yaw)
            direccion = (float(self.cfg.get("kp_recto", 2.6)) * err
                         - float(self.cfg.get("kd_recto", 0.22)) * c.sens.gz
                         + c.guardia_muro)
            nota = f"recto ({err:+.0f} deg)"
        else:
            # Sin MPU no hay rumbo que sostener: manda el carril y se nota.
            direccion = c.direccion_mezclada + c.guardia_muro
            nota = "reincorporandose sin MPU"
        return Orden(vel=v, direccion=max(-100.0, min(100.0, direccion)),
                     armado=True, parada=False, centrar=False, nota=nota)

    # ---------------------------------------------------------- esquinas
    def _toca_esquina(self, c: Contexto) -> bool:
        """La linea del piso acaba de decir que aqui empieza una curva.

        TRES CONDICIONES, Y CADA UNA TAPA UN AGUJERO DISTINTO:

          1. Se acaba de pisar la PRIMERA linea de una esquina. La segunda no
             dispara nada: cierra el par y ya esta.
          2. El frente esta de verdad cerrado. Es el contraste de la camara
             contra el TCS, y hace falta por un caso muy concreto: si el
             sensor se pierde la linea de ENTRADA, la de SALIDA pasa a ser la
             primera de un par nuevo y dispararia un giro al salir de la
             curva, o sea contra el muro de la recta siguiente. Al salir de
             una curva el frente esta despejado, asi que esta condicion sola
             lo descarta.
          3. Se sabe hacia donde girar. Sin sentido no hay giro que
             comprometer; manda el seguidor de carril, como antes.

        Y encima, UNA ESQUINA UN GIRO: despues de girar se bloquea durante
        `dist_entre_esquinas_mm` de odometro. La seccion de curva mide 1000 mm
        y las dos lineas caen dentro, asi que esto impide que la segunda linea
        —o un rebote del sensor— encadene un segundo giro de 90 grados. Ese
        fallo existe y se ha visto: dos giros seguidos son 180 grados y la
        pared.
        """
        # EL SENTIDO, DE DONDE SALGA. El aprendido por el carril manda —no
        # depende de ningun parametro— pero en el ciclo del PRIMER cruce de la
        # ronda todavia vale 0: el piloto calcula el carril antes que el
        # contador, asi que el carril arrastra el sentido del ciclo anterior.
        # Como el evento de esquina dura un solo ciclo, exigirle solo al
        # carril dejaba la primera curva de la ronda sin disparar — para
        # siempre, porque la linea no se vuelve a pisar.
        sentido = c.carril.sentido_curva or c.vueltas.sentido
        if not c.vueltas.esquina_abierta or not sentido:
            return False
        lejos = float(self.cfg.get("esquina_frente_max_mm", 1400.0))
        if c.carril.dist_frente_mm > lejos:
            return False
        hueco = c.vueltas.dist_mm - self._dist_ultima_esquina_mm
        return hueco >= float(self.cfg.get("dist_entre_esquinas_mm", 1200.0))

    def _entrar_esquina(self, c: Contexto) -> Orden:
        self._dist_ultima_esquina_mm = c.vueltas.dist_mm
        self._sentido_esquina = c.carril.sentido_curva or c.vueltas.sentido
        self._yaw_esquina = c.sens.yaw if c.sens.mpu_ok else None
        self._ir(Estado.ESQUINA,
                 f"linea de esquina, giro a la "
                 f"{'derecha' if self._sentido_esquina > 0 else 'izquierda'}")
        return self._esquina(c)

    def _esquina(self, c: Contexto) -> Orden:
        """Giro comprometido de la curva. Se gira Y SE SIGUE GIRANDO.

        La diferencia con dejarselo al seguidor de carril no es la fuerza del
        volante, es el compromiso. El carril renegocia el volante cada ciclo
        contra el centrado, asi que en cuanto asoma un poco de hueco afloja y
        el giro se queda a medias; aqui el volante se mantiene hasta haber
        girado de verdad. Lo unico que se le suma es la guardia anti-muro, que
        no negocia el giro: solo impide rozar.

        COMO SE SABE QUE YA SE GIRO. Con el yaw del MPU, que es la unica
        medida directa de "cuanto he girado". Sin MPU solo queda el
        cronometro, que depende de la velocidad y del agarre: funciona, pero
        es el motivo de que `esquina_max_s` exista y de que convenga tener el
        I2C sano.
        """
        if self._parada_por_vueltas(c):
            return self._meta(c)
        if self._detectar_atasco(c):
            return self._entrar_reversa("atascado en la curva")
        # UN PILAR MANDA SOBRE LA CURVA. Rebasar por el lado que toca vale 8 o
        # 10 puntos (1.6, 1.7) y hacerlo por el lado malo termina la ronda;
        # trazar bien la esquina no vale ninguno. Asi que si aparece una
        # señal, se atiende y la curva la termina el seguidor de carril.
        if c.maniobra.fase == FASE_CORRECCION:
            self._ir(Estado.CORRECCION, "lado incorrecto en plena curva")
            return self._correccion(c)
        if c.maniobra.peso > float(self.cfg.get("esquina_cede_peso", 0.35)):
            self._ir(Estado.SENAL, f"pilar {c.maniobra.color} en la curva")
            return self._senal(c)

        girado = 0.0
        if self._yaw_esquina is not None and c.sens.mpu_ok:
            girado = abs(envolver_grados(c.sens.yaw - self._yaw_esquina))
        bastante = girado >= float(self.cfg.get("esquina_grados", 70.0))
        agotado = self.en_estado_s > float(self.cfg.get("esquina_max_s", 2.5))
        if bastante or agotado:
            self._ir(Estado.PISTA,
                     "tiempo de curva" if agotado else f"girados {girado:.0f} deg")
            return self._pista(c)

        v = min(c.velocidad_sugerida, float(self.cfg.get("vel_esquina", 30.0)))
        direccion = (self._sentido_esquina *
                     float(self.cfg.get("dir_esquina", 85.0)) + c.guardia_muro)
        return Orden(vel=v, direccion=max(-100.0, min(100.0, direccion)),
                     armado=True, parada=False, centrar=False, aux=AUX_LENTO,
                     nota=f"curva comprometida ({girado:.0f} de "
                          f"{self.cfg.get('esquina_grados', 70.0):.0f} deg)")

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
        direccion = c.maniobra.direccion + c.guardia_muro
        return Orden(vel=v, direccion=max(-100.0, min(100.0, direccion)),
                     armado=True, parada=False, centrar=False,
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
