"""
vueltas.py — Contar secciones y vueltas, y saber donde parar.

POR QUE ESTO VALE PUNTOS DE VERDAD
El reglamento paga 1 punto por cada seccion superada (hasta 24), 1 por vuelta
completa (hasta 3) y 3 por PARAR en la seccion de meta despues de tres vueltas
(10.2, tabla 1.1-1.3). Ademas, los puntos gordos por las señales (1.6 y 1.7,
8 y 10 puntos) solo se cobran "despues de completar tres vueltas". Un contador
que se despista no pierde un punto: pierde la mitad de la ronda.

QUIEN CUENTA: EL TCS34725, NO LA CAMARA
El sensor de color va bajo el carro, mirando el tapete. Cuando pasa sobre una
linea no hay interpretacion posible: o esta encima o no. La camara ve las
lineas antes —util para anticipar la curva— pero se le cuelan reflejos y
pierde las lineas en diagonal. Asi que la camara ANTICIPA y el TCS CUENTA.

EL CONTADOR VIENE ACUMULADO DESDE EL ESP32, no como eventos. Si se pierde una
trama serial no se pierde un cruce: se ve un ciclo mas tarde. Esa decision es
la que permite contar bien sin ACK ni retransmisiones.

DE QUE LADO SE CORRE: SE APRENDE, NO SE CONFIGURA
El reglamento (9.3) sortea el sentido de la marcha antes de cada ronda, y
9.4/9.9 prohiben meterle esa informacion al programa a mano. Asi que hay que
deducirlo de la pista, y la pista lo dice de dos maneras:

  a) PROVISIONAL, con la PRIMERA linea que se pisa. Llega enseguida —hace
     falta antes de la primera curva— pero se apoya en que esa linea sea de
     verdad la de ENTRADA a la curva. Si el TCS se salta la naranja y cuenta
     la azul, la respuesta sale del reves.

  b) FIRME, con el PAR ORDENADO de una esquina entera. Cada curva tiene una
     linea naranja y una azul, y el ORDEN en que se pisan solo depende del
     sentido de la marcha: no hay forma de cruzarlas al reves corriendo en el
     mismo sentido. Una esquina a la que le falto una linea no forma par y no
     contesta nada, que es justo lo que se quiere: mas vale seguir con la
     respuesta provisional que cambiarla por una peor.

Las dos salen del MISMO parametro, `color_entrada_horario`: el color que se
pisa al entrar en curva corriendo en horario. De ahi salen tanto la respuesta
provisional como el orden del par (horario = entrada + el otro color). Es un
dato del tapete y del montaje del sensor, no del sorteo: se comprueba UNA VEZ
en la pista de practica empujando el carro a mano, y se deja fijo.

EL SENTIDO NO CAMBIA A MITAD DE RONDA. Una vez firme no se vuelve a tocar: si
un par posterior sale al reves, lo que hay es un cruce mal leido, no un carro
que se dio la vuelta. Se anota como incoherencia en la telemetria y se sigue.

DONDE PARAR
La seccion de meta es la de arranque. El carro no empieza pegado a una linea,
empieza en algun punto de la recta, asi que "volver a la seccion de arranque"
no coincide con un cruce. Lo que se hace es medir cuanto se recorrio desde el
arranque hasta el PRIMER cruce, y despues de tres vueltas repetir esa misma
distancia a partir del cruce equivalente. No es exacto al milimetro, pero la
zona de arranque mide 500 mm de largo: sobra margen.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from . import protocolo as proto

SECCIONES_POR_VUELTA = 8      # 4 rectas + 4 curvas (reglamento, seccion 5)
CRUCES_POR_VUELTA = 4         # cruces de la linea de ENTRADA a curva


@dataclass
class EstadoVueltas:
    sentido: int = 0                  # +1 horario, -1 antihorario, 0 sin saber
    sentido_firme: bool = False       # confirmado por un par de esquina entero
    origen_sentido: str = ""          # "primera linea" | "par de esquina"
    pares_incoherentes: int = 0       # pares que contradicen al sentido firme
    # EVENTOS DE ESTE CICLO. Se ponen a falso al principio de cada
    # actualizar(), asi que solo son ciertos en el ciclo en que ocurrieron:
    # quien los lee no tiene que acordarse de consumirlos.
    linea_nueva: str = ""             # color pisado en este ciclo, o ""
    esquina_abierta: bool = False     # ese cruce ABRE una esquina (1a linea)
    dist_esquina_mm: float = 0.0      # odometro de la ultima esquina abierta
    vueltas: int = 0
    secciones: int = 0
    cruces_totales: int = 0
    cruces_entrada: int = 0     # solo los del color de entrada a curva
    dist_mm: float = 0.0              # odometro desde el arranque
    dist_desde_cruce_mm: float = 0.0
    dist_arranque_a_primer_cruce_mm: Optional[float] = None
    listo_para_parar: bool = False
    motivo: str = ""
    info: Dict[str, Any] = field(default_factory=dict)


class Contador:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.reiniciar()

    def reiniciar(self) -> None:
        self._base_naranja: Optional[int] = None
        self._base_azul: Optional[int] = None
        self._prev_naranja = 0
        self._prev_azul = 0
        self._color_entrada = ""      # el color que se pisa al ENTRAR en curva
        # Emparejador de esquinas: (color de la primera linea, odometro).
        self._par_abierto: Optional[Tuple[str, float]] = None
        self.e = EstadoVueltas()

    # ------------------------------------------------------------------
    def actualizar(self, sens: proto.Sensores, vel_mm_s: float, dt: float,
                   contar: bool = True) -> EstadoVueltas:
        """contar=False mientras la ronda no haya empezado de verdad.

        En ESPERA el carro esta quieto delante del juez, a veces varios
        minutos. Todo lo que entre por el contador en ese rato es ruido, y si
        se acumula, el carro sale del boton creyendo que ya dio tres vueltas.
        Pasando contar=False se sigue la pista del contador del ESP32 (para no
        ver un salto enorme al empezar) pero no se cuenta nada.
        """
        e = self.e
        # Los eventos duran UN ciclo. Dejarlos pegados obligaria a que el
        # piloto los borrara despues de leerlos, y ese es exactamente el tipo
        # de contrato que se olvida en una rama y dispara dos giros.
        e.linea_nueva = ""
        e.esquina_abierta = False

        # --- odometro ----------------------------------------------------
        # Sin encoder en las ruedas, la distancia se integra de la velocidad
        # estimada. El error es de un 10-15 % y no importa: solo se usa para
        # medir un tramo de recta corto, no para navegar.
        if contar:
            avance = max(0.0, vel_mm_s) * max(0.0, dt)
            e.dist_mm += avance
            e.dist_desde_cruce_mm += avance

        if not sens.tcs_ok:
            e.info["aviso"] = "TCS ausente: contando solo por camara"
            return e

        # --- diferencias de los contadores acumulados ---------------------
        if self._base_naranja is None:
            # Primera trama: se toma como cero. Los contadores del ESP32 no
            # arrancan necesariamente en 0 (pudo haber pruebas antes de armar).
            self._base_naranja = sens.cruces_naranja
            self._base_azul = sens.cruces_azul
            self._prev_naranja = sens.cruces_naranja
            self._prev_azul = sens.cruces_azul
            return e

        d_nar = (sens.cruces_naranja - self._prev_naranja) & 0xFF
        d_azu = (sens.cruces_azul - self._prev_azul) & 0xFF
        self._prev_naranja = sens.cruces_naranja
        self._prev_azul = sens.cruces_azul

        # --- SALTO IMPOSIBLE = RESINCRONIZAR, NO CONTAR -------------------
        # Un delta grande NO es "han pasado muchas lineas": es que el contador
        # del ESP32 se reinicio (CAL_CERO_LINEAS) o se perdio el sincronismo.
        # La resta de 8 bits convierte 200 -> 0 en un delta de 56, y el codigo
        # se creia 56 cruces: catorce vueltas de golpe, con el carro quieto y
        # con 0.3 m en el odometro. Asi termino una ronda nada mas pulsar el
        # boton.
        #
        # Fisicamente, entre dos ciclos de la Pi (~33 ms) no caben mas de uno
        # o dos cruces: a 1 m/s una linea de 20 mm dura 20 ms y las lineas de
        # una esquina estan separadas. Por encima del limite, lo unico honesto
        # es tomar el valor actual como nueva referencia y NO inventarse
        # vueltas que no ocurrieron.
        tope = int(self.cfg.get("max_cruces_por_ciclo", 3))
        if d_nar > tope or d_azu > tope:
            e.info["resync"] = e.info.get("resync", 0) + 1
            return e

        if d_nar == 0 and d_azu == 0:
            return e

        if not contar:
            return e

        # --- procesar cada cruce nuevo ------------------------------------
        # SI LOS DOS COLORES SUBEN EN EL MISMO CICLO, EL ORDEN NO SE SABE.
        # Los cruces llegan como contadores acumulados, no como eventos con
        # marca de tiempo: dentro de un ciclo se procesan naranja y luego azul
        # porque hay que elegir un orden, no porque ese sea el orden real. Las
        # dos lineas de una esquina estan a ~1 m, asi que esto solo pasa tras
        # una perdida de tramas — y justo ahi es donde un par inventado daria
        # un sentido invertido. Se cuentan los cruces, pero no forman par.
        orden_fiable = not (d_nar > 0 and d_azu > 0)
        for _ in range(d_nar):
            self._cruce("naranja", e, orden_fiable)
        for _ in range(d_azu):
            self._cruce("azul", e, orden_fiable)
        return e

    # ------------------------------------------------------------------
    def _cruce(self, color: str, e: EstadoVueltas,
               orden_fiable: bool = True) -> None:
        e.cruces_totales += 1

        # --- primer cruce: fija el sentido y el color de entrada ----------
        if not self._color_entrada:
            self._color_entrada = color
            color_horario = str(self.cfg.get("color_entrada_horario", "naranja"))
            e.sentido = +1 if color == color_horario else -1
            e.origen_sentido = "primera linea"
            e.dist_arranque_a_primer_cruce_mm = e.dist_mm
            e.info["sentido"] = "horario" if e.sentido > 0 else "antihorario"

        # --- y el par ordenado de la esquina lo CONFIRMA ------------------
        self._emparejar(color, e, orden_fiable)

        # Cada cruce es una frontera de seccion: recta -> curva o curva ->
        # recta. Ocho por vuelta, que es exactamente como el reglamento cuenta
        # las secciones para el punto 1.1.
        e.secciones += 1

        # Las vueltas se cuentan solo con el color de ENTRADA, porque es el
        # que aparece una vez por curva. Contar los dos colores duplicaria.
        if color == self._color_entrada:
            e.cruces_entrada += 1
            if e.cruces_entrada % CRUCES_POR_VUELTA == 0:
                e.vueltas = e.cruces_entrada // CRUCES_POR_VUELTA

        e.dist_desde_cruce_mm = 0.0
        e.linea_nueva = color
        e.info["ultimo_cruce"] = color

    # ------------------------------------------------------------------
    def _emparejar(self, color: str, e: EstadoVueltas,
                   orden_fiable: bool = True) -> None:
        """El ORDEN de las dos lineas de una esquina dice el sentido.

        POR QUE EL PAR Y NO LA PRIMERA LINEA A SECAS. La primera linea solo
        contesta bien si de verdad es la de entrada. En pista el TCS se salta
        lineas —una linea de 20 mm a 1 m/s dura 20 ms— y cuando se salta la de
        entrada, la de salida se toma por la primera y el sentido sale
        invertido. Un carro que cree que las curvas van al otro lado se estampa
        en la primera esquina de frente, y el fallo no se parece en nada a su
        causa. El par lo cierra: da igual cual de las dos se vea primero si se
        ven las dos, porque lo que se mira es el orden, no la identidad.

        LO QUE NO SE HACE: emparejar a cualquier precio. Si a una esquina le
        falta una linea, la que quedo suelta NO se empareja con la de la
        esquina siguiente —estan a media pista de distancia— sino que caduca
        por `ventana_par_mm` y abre un par nuevo. Un par inventado es peor que
        ningun par: contesta, y contesta al reves.
        """
        if not orden_fiable:
            self._par_abierto = None
            e.info["par_sin_orden"] = e.info.get("par_sin_orden", 0) + 1
            return

        ventana = float(self.cfg.get("ventana_par_mm", 1500.0))
        abierto = self._par_abierto
        if abierto is not None:
            color0, dist0 = abierto
            # Mismo color otra vez, o demasiada pista de por medio: a esa
            # esquina le falto una linea. Se descarta y se abre un par nuevo.
            if color == color0 or (e.dist_mm - dist0) > ventana:
                abierto = None
        if abierto is None:
            # PRIMERA LINEA DE UNA ESQUINA. Este es el evento que de verdad
            # sirve para conducir: dice DONDE empieza la curva, que es algo
            # que la camara no sabe decir a tiempo. La vision ve que el frente
            # se cierra, pero eso pasa tanto en una esquina como cuando el
            # carro quedo apuntando a un muro despues de rebasar un pilar; la
            # linea del piso no se presta a esa confusion: o se pisa o no.
            #
            # El SENTIDO del giro no sale del color de ESTA linea —los dos
            # colores aparecen en las cuatro esquinas— sino del sentido de la
            # ronda, que es lo que el color de la PRIMERA linea de la ronda
            # dejo fijado (naranja = horario con la configuracion de fabrica).
            self._par_abierto = (color, e.dist_mm)
            e.esquina_abierta = True
            e.dist_esquina_mm = e.dist_mm
            return

        color0, _ = abierto
        self._par_abierto = None
        color_horario = str(self.cfg.get("color_entrada_horario", "naranja"))
        sentido = +1 if color0 == color_horario else -1
        e.info["ultimo_par"] = f"{color0}+{color}"

        if not e.sentido_firme:
            e.sentido = sentido
            e.sentido_firme = True
            e.origen_sentido = "par de esquina"
            e.info["sentido"] = "horario" if sentido > 0 else "antihorario"
            return

        # Ya estaba firme. El sentido NO cambia a mitad de ronda: un par al
        # reves es un cruce mal leido, no un carro que se dio la vuelta.
        if sentido != e.sentido:
            e.pares_incoherentes += 1
            e.info["par_incoherente"] = e.pares_incoherentes

    # ------------------------------------------------------------------
    def evaluar_parada(self, vueltas_objetivo: int = 3) -> EstadoVueltas:
        """¿Toca pararse ya?

        Condicion: tres vueltas hechas Y haber recorrido, desde el ultimo
        cruce de entrada, la misma distancia que habia entre el arranque y el
        primer cruce. Eso deja el carro dentro de la seccion de arranque, que
        es la de meta (9.23).
        """
        e = self.e
        if e.vueltas < vueltas_objetivo:
            e.listo_para_parar = False
            return e

        # SUELO DE DISTANCIA: tres vueltas no caben en tres metros.
        # Es la ultima red contra un contador que se desboca. Aunque un fallo
        # de sincronismo cuele vueltas falsas, el odometro no miente tanto: el
        # recorrido de una vuelta ronda los 8 m, asi que exigir 4 m por vuelta
        # deja margen de sobra para el 15 % de error del odometro y aun asi
        # hace imposible terminar la ronda nada mas arrancar.
        minimo = float(self.cfg.get("min_dist_por_vuelta_mm", 4000.0))
        if e.dist_mm < vueltas_objetivo * minimo:
            e.listo_para_parar = False
            e.motivo = (f"{e.vueltas} vueltas pero solo {e.dist_mm/1000:.1f} m: "
                        f"el contador no cuadra con el odometro")
            return e

        d0 = e.dist_arranque_a_primer_cruce_mm
        if d0 is None:
            # Nunca se vio una linea: no se puede saber donde esta la meta.
            # Mas vale seguir rodando que parar en el sitio equivocado, que
            # ademas cuenta como "no paro en meta" igual.
            e.listo_para_parar = False
            e.motivo = "sin referencia de meta"
            return e
        margen = float(self.cfg.get("margen_meta_mm", 120.0))
        if e.dist_desde_cruce_mm >= max(0.0, d0 - margen):
            e.listo_para_parar = True
            e.motivo = f"{e.vueltas} vueltas y {e.dist_desde_cruce_mm:.0f} mm de recta"
        return e

    # ------------------------------------------------------------------
    def resumen(self) -> Dict[str, Any]:
        e = self.e
        return {
            "sentido": e.sentido,
            "sentido_firme": e.sentido_firme,
            "origen_sentido": e.origen_sentido,
            "pares_incoherentes": e.pares_incoherentes,
            "ultima_esquina_mm": round(e.dist_esquina_mm),
            "vueltas": e.vueltas,
            "secciones": e.secciones,
            "cruces": e.cruces_totales,
            "dist_m": round(e.dist_mm / 1000.0, 2),
            "desde_cruce_mm": round(e.dist_desde_cruce_mm),
            "listo_para_parar": e.listo_para_parar,
        }
