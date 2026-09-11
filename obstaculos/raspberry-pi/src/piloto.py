"""
piloto.py — El lazo principal. Une camara, vision, decision y enlace.

ESTRUCTURA DEL CICLO, Y POR QUE ESTE ORDEN

    1. leer el ultimo frame         (camara.leer, nunca bloquea)
    2. leer el ultimo estado del ESP32 (enlace.estado, copia bajo lock)
    3. percibir                     (vision -> escena en milimetros)
    4. calcular las dos opiniones   (carril y senales, por separado)
    5. mezclarlas                   (senales.combinar)
    6. decidir el estado            (fsm.paso -> Orden)
    7. dejar la orden               (enlace.mandar, que NO escribe: el hilo TX
                                     a 50 Hz la repite hasta la siguiente)

Los pasos 4 y 5 estan separados a proposito. Si el esquive y el carril se
calcularan mezclados, no habria forma de mirar la telemetria y saber cual de
los dos pidio el volantazo. Separados, el registro dice "carril: -12, esquive:
+64, peso 0.8" y el fallo se ve de un vistazo.

SOBRE EL RITMO: este lazo NO tiene que ser regular. Corre a la velocidad de la
camara (~30 Hz) y puede saltarse un ciclo sin consecuencias, porque el mando
sale a 50 Hz por su cuenta desde el hilo TX. Esa separacion es lo que permite
hacer trabajo caro aqui —segmentar seis colores— sin que el carro lo note.
"""

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from . import protocolo as proto
from .camara import Camara
from .carril import SeguidorCarril, margen_magenta
from .enlace import Enlace
from .fsm import Contexto, Estado, MaquinaEstados, Orden
from .geometria import Geometria
from .senales import Esquivador, combinar
from .vision import Detector, Escena
from .vueltas import Contador


class Piloto:
    def __init__(self, cfg: Dict[str, Any], *, puerto: Optional[str] = None,
                 verbose: bool = True) -> None:
        self.cfg = cfg
        self.verbose = verbose

        cam = cfg["camara"]
        self.camara = Camara(indice=cam["indice"], ancho=cam["ancho"],
                             alto=cam["alto"], fps=cam["fps"],
                             fourcc=cam["fourcc"], voltear=cam["voltear"],
                             verbose=verbose)
        self.enlace = Enlace(puerto=puerto, verbose=verbose)

        self.geo = Geometria(cfg["geometria"], cam["ancho"], cam["alto"])
        self.detector = Detector(cfg["colores"], self.geo)
        self.carril = SeguidorCarril(cfg["carril"])
        self.esquivador = Esquivador(cfg["senales"], self.geo.semiancho_mm())
        self.contador = Contador(cfg["vueltas"])
        self.fsm = MaquinaEstados(cfg["fsm"])

        self._pulsaciones_prev: Optional[int] = None
        self._n_frame_prev = -1
        self._t_prev = 0.0
        self._vel_pct = 0.0
        self.ultimo: Dict[str, Any] = {}

    # ------------------------------------------------------------ arranque
    def preparar(self) -> bool:
        if not self.camara.abrir():
            print("[piloto] no se pudo abrir la camara")
            return False
        if not self.enlace.abrir():
            print("[piloto] no se pudo abrir el enlace con el ESP32")
            return False
        # Configurar el servo ANTES de armar nada: si el centro estuviera mal,
        # el primer movimiento seria contra el tope.
        s = self.cfg["servo"]
        self.enlace.configurar_servo(s["centro"], s["izquierda"], s["derecha"],
                                     s["grados_por_seg"], s["rampa_motor"],
                                     s["ms_freno_inversion"])
        time.sleep(0.1)
        # Poner a cero los contadores de linea: si hubo pruebas antes de la
        # ronda, el ESP32 arrastra cruces que no son de esta carrera.
        self.enlace.calibrar(proto.CAL_CERO_LINEAS)
        self.enlace.parar()
        if self.verbose:
            print("[piloto] listo. Pulsa el boton de inicio en el carro.")
        return True

    # --------------------------------------------------------------- ciclo
    def ciclo(self) -> Orden:
        ahora = time.time()
        dt = min(0.25, ahora - self._t_prev) if self._t_prev else 0.033
        self._t_prev = ahora

        frame, n_frame, _ = self.camara.leer()
        hay_frame = frame is not None and n_frame != self._n_frame_prev
        self._n_frame_prev = n_frame

        est = self.enlace.estado()
        sens = est.sens
        enlace_ok = est.conectado and est.sensores_frescos

        # --- flanco del boton de inicio ---------------------------------
        # Se detecta por CONTADOR, no por nivel: si la trama en que se pulso se
        # pierde, el contador sigue habiendo subido y el flanco no se escapa.
        arranque = False
        if self._pulsaciones_prev is None:
            self._pulsaciones_prev = sens.pulsaciones
        elif sens.pulsaciones != self._pulsaciones_prev:
            arranque = True
            self._pulsaciones_prev = sens.pulsaciones

        # --- percepcion ---------------------------------------------------
        esc: Escena = (self.detector.procesar(frame) if hay_frame
                       else getattr(self, "_esc_prev", Escena()))
        if hay_frame:
            self._esc_prev = esc

        # --- velocidad estimada -------------------------------------------
        # Sin encoder: se deduce del % de mando que se esta pidiendo. Basta
        # para el odometro y para calcular cuanto dura un compromiso.
        vel_mm_s = abs(self._vel_pct) * float(
            self.cfg["traccion"].get("mm_s_por_pct", 22.0))

        # --- las dos opiniones, por separado -------------------------------
        sal_carril = self.carril.paso(esc, gz=sens.gz)
        maniobra = self.esquivador.paso(esc, esc.lineas_mm,
                                        sal_carril.en_curva, vel_mm_s)
        empujon = margen_magenta(esc, self.geo.semiancho_mm())
        direccion = combinar(sal_carril.direccion, maniobra, empujon)

        # --- vueltas -------------------------------------------------------
        ev = self.contador.actualizar(sens, vel_mm_s, dt)
        self.contador.evaluar_parada(int(self.cfg["fsm"].get("vueltas", 3)))

        # --- decidir -------------------------------------------------------
        ctx = Contexto(
            t=ahora, enlace_ok=enlace_ok, hay_frame=frame is not None,
            arranque_pedido=arranque,
            cortado_por_boton=sens.cortado_por_boton,
            sens=sens, escena=esc, carril=sal_carril, maniobra=maniobra,
            vueltas=ev, vel_mm_s=vel_mm_s,
            direccion_mezclada=direccion,
            velocidad_sugerida=self.carril.velocidad(sal_carril))
        orden = self.fsm.paso(ctx)

        # --- actuar --------------------------------------------------------
        self._vel_pct = orden.vel
        self.enlace.mandar(orden.vel, orden.direccion, armado=orden.armado,
                           parada=orden.parada, centrar=orden.centrar,
                           vmax=int(self.cfg["traccion"].get("vmax_pwm", 210)),
                           aux=orden.aux)

        self.ultimo = {
            "estado": self.fsm.estado.value,
            "nota": orden.nota,
            "vel": round(orden.vel, 1),
            "dir": round(orden.direccion, 1),
            "carril_dir": round(sal_carril.direccion, 1),
            "esquive_dir": round(maniobra.direccion, 1),
            "esquive_peso": round(maniobra.peso, 2),
            "esquive_fase": maniobra.fase,
            "objetivo_mm": round(maniobra.objetivo_mm) if maniobra.peso > 0 else None,
            "pilar": maniobra.color,
            "frente_mm": round(sal_carril.dist_frente_mm),
            "izq_mm": round(sal_carril.dist_izq_mm),
            "der_mm": round(sal_carril.dist_der_mm),
            "magenta": round(empujon, 1) if empujon is not None else None,
            "yaw": round(sens.yaw, 1),
            "vueltas": ev.vueltas,
            "secciones": ev.secciones,
            "sentido": ev.sentido,
        }
        return orden

    # -------------------------------------------------------------- bucle
    def correr(self, duracion_max_s: float = 0.0) -> None:
        """Bucle hasta terminar la ronda, o hasta Ctrl-C.

        duracion_max_s = 0 significa "sin limite". El reglamento da 3 minutos
        (9.2), pero pararse solo al llegar al limite no da ningun punto: si
        queda tiempo se sigue rodando, que es lo que puntua.
        """
        t0 = time.time()
        ultimo_aviso = 0.0
        try:
            while True:
                self.ciclo()
                if self.fsm.estado == Estado.FIN:
                    if self.verbose:
                        print(f"[piloto] ronda terminada: {self.contador.resumen()}")
                    break
                if duracion_max_s and (time.time() - t0) > duracion_max_s:
                    if self.verbose:
                        print("[piloto] limite de tiempo alcanzado")
                    break
                if self.verbose and (time.time() - ultimo_aviso) > 0.5:
                    ultimo_aviso = time.time()
                    u = self.ultimo
                    print(f"  {u['estado']:<10} v={u['vel']:>5} d={u['dir']:>6} "
                          f"| carril {u['carril_dir']:>6} esq {u['esquive_dir']:>6}"
                          f" x{u['esquive_peso']:<4} | frente {u['frente_mm']:>4} mm"
                          f" | vuelta {u['vueltas']} | {u['nota']}")
                # Cede el resto del intervalo de camara. No es un temporizador:
                # si el frame ya llego, se vuelve enseguida.
                time.sleep(0.004)
        except KeyboardInterrupt:
            if self.verbose:
                print("\n[piloto] interrumpido")
        finally:
            self.cerrar()

    def cerrar(self) -> None:
        try:
            self.enlace.parar()
            time.sleep(0.1)
        finally:
            self.enlace.cerrar()
            self.camara.cerrar()
