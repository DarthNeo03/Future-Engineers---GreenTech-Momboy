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
9.4/9.9 prohiben meterle esa informacion al programa a mano. Asi que el
sentido se deduce del PRIMER cruce: el color de la linea que se pisa primero
dice hacia donde se esta girando. Cual de los dos colores corresponde a
horario depende de como este puesto el sensor y del tapete, asi que es un
parametro que se comprueba UNA VEZ en la pista de practica y se deja fijo.

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
from typing import Any, Dict, Optional

from . import protocolo as proto

SECCIONES_POR_VUELTA = 8      # 4 rectas + 4 curvas (reglamento, seccion 5)
CRUCES_POR_VUELTA = 4         # cruces de la linea de ENTRADA a curva


@dataclass
class EstadoVueltas:
    sentido: int = 0                  # +1 horario, -1 antihorario, 0 sin saber
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
        self.e = EstadoVueltas()

    # ------------------------------------------------------------------
    def actualizar(self, sens: proto.Sensores, vel_mm_s: float,
                   dt: float) -> EstadoVueltas:
        e = self.e

        # --- odometro ----------------------------------------------------
        # Sin encoder en las ruedas, la distancia se integra de la velocidad
        # estimada. El error es de un 10-15 % y no importa: solo se usa para
        # medir un tramo de recta corto, no para navegar.
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
        if d_nar == 0 and d_azu == 0:
            return e

        # --- procesar cada cruce nuevo ------------------------------------
        for _ in range(d_nar):
            self._cruce("naranja", e)
        for _ in range(d_azu):
            self._cruce("azul", e)
        return e

    # ------------------------------------------------------------------
    def _cruce(self, color: str, e: EstadoVueltas) -> None:
        e.cruces_totales += 1

        # --- primer cruce: fija el sentido y el color de entrada ----------
        if not self._color_entrada:
            self._color_entrada = color
            color_horario = str(self.cfg.get("color_entrada_horario", "naranja"))
            e.sentido = +1 if color == color_horario else -1
            e.dist_arranque_a_primer_cruce_mm = e.dist_mm
            e.info["sentido"] = "horario" if e.sentido > 0 else "antihorario"

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
        e.info["ultimo_cruce"] = color

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
            "vueltas": e.vueltas,
            "secciones": e.secciones,
            "cruces": e.cruces_totales,
            "dist_m": round(e.dist_mm / 1000.0, 2),
            "desde_cruce_mm": round(e.dist_desde_cruce_mm),
            "listo_para_parar": e.listo_para_parar,
        }
