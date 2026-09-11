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
from .camara import Camara, explicar_fallo
from .carril import SeguidorCarril, guardia_muro, margen_magenta
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
            # Un "no se pudo abrir la camara" a secas obliga a adivinar. El
            # 90 % de las veces es una ejecucion anterior que quedo viva, y
            # eso se puede decir en vez de dejarlo al detective de turno.
            print("[piloto] no se pudo abrir la camara")
            print("  " + explicar_fallo(self.cfg["camara"]["indice"]))
            return False
        if not self.enlace.abrir():
            print("[piloto] no se pudo abrir el enlace con el ESP32")
            return False
        # Configurar el servo ANTES de armar nada: si el centro estuviera mal,
        # el primer movimiento seria contra el tope.
        s = self.cfg["servo"]
        self.enlace.configurar_servo(s["centro"], s["izquierda"], s["derecha"],
                                     s["grados_por_seg"], s["rampa_motor"],
                                     s["ms_freno_inversion"],
                                     s.get("pwm_min_motor", 0))
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
        # El sentido que dedujo el contador de vueltas (por el color de la
        # primera linea pisada) viaja al seguidor de carril como PISTA: le
        # dice hacia donde giran las curvas de esta ronda antes de haber
        # tomado ninguna. En cuanto toma la primera, el seguidor lo aprende
        # por su cuenta y deja de depender de ese parametro.
        #
        # TRAS_PILAR: "lo que cierra el frente puede no ser una esquina".
        # Vale mientras hay un rebase en marcha y hasta que la FSM da por
        # reincorporado al carro. Se compone de dos fuentes porque ninguna
        # basta sola: el esquivador sabe si sigue habiendo maniobra, pero se
        # apaga en el instante en que caduca el compromiso, y es JUSTO
        # despues cuando el carro va pegado al muro y apuntandolo.
        tras_pilar = (self.esquivador.ocupado() or
                      self.fsm.estado in (Estado.ESQUIVE, Estado.CORRECCION,
                                          Estado.REINCORPORACION))
        #
        # ESPERA_LINEA: con el TCS vivo, la esquina la dispara la linea del
        # piso (fsm.Estado.ESQUINA) y el sesgo de curva del carril se queda de
        # red. Sin TCS no hay linea que esperar y el sesgo vuelve a mandar.
        sal_carril = self.carril.paso(esc, gz=sens.gz,
                                      sentido_pista=self.contador.e.sentido,
                                      tras_pilar=tras_pilar,
                                      espera_linea=sens.tcs_ok)
        # El yaw entra en el esquive para que el compromiso sostenga el RUMBO
        # y no el angulo de volante: un volante fijo describe un arco y el
        # carro se iba girando hacia el lado del pilar que acababa de pasar.
        # Y el rumbo del hueco entra para la SALIDA del compromiso: el rebase
        # termina devolviendo el carro enfilado al pasillo, no cruzado.
        maniobra = self.esquivador.paso(
            esc, esc.lineas_mm, sal_carril.en_curva, vel_mm_s,
            yaw=(sens.yaw if sens.mpu_ok else None), gz=sens.gz,
            rumbo_hueco_deg=sal_carril.rumbo_hueco_deg)
        empujon = margen_magenta(esc, self.geo.semiancho_mm())
        direccion = combinar(sal_carril.direccion, maniobra, empujon)

        # --- vueltas -------------------------------------------------------
        # NO se cuenta mientras la ronda no haya empezado. En ESPERA el carro
        # lleva minutos quieto delante del juez y cualquier ruido que entre se
        # acumula: asi es como salio del boton creyendo que llevaba 63 vueltas.
        corriendo = self.fsm.estado not in (Estado.ESPERA, Estado.FIN,
                                            Estado.FALLO)
        ev = self.contador.actualizar(sens, vel_mm_s, dt, contar=corriendo)
        self.contador.evaluar_parada(int(self.cfg["fsm"].get("vueltas", 3)))

        # --- decidir -------------------------------------------------------
        ctx = Contexto(
            t=ahora, enlace_ok=enlace_ok, hay_frame=frame is not None,
            arranque_pedido=arranque,
            cortado_por_boton=sens.cortado_por_boton,
            sens=sens, escena=esc, carril=sal_carril, maniobra=maniobra,
            vueltas=ev, vel_mm_s=vel_mm_s,
            direccion_mezclada=direccion,
            guardia_muro=guardia_muro(sal_carril, self.cfg["carril"]),
            velocidad_sugerida=self.carril.velocidad(sal_carril))
        estado_antes = self.fsm.estado
        orden = self.fsm.paso(ctx)

        # --- la ronda empieza AQUI, no en preparar() ------------------------
        # Poner los contadores a cero en preparar() abria una carrera: la Pi
        # tomaba su linea base antes de que el ESP32 llegara a aplicar el
        # CAL_CERO_LINEAS, y la resta de 8 bits se leia como decenas de cruces.
        # Reiniciando en el flanco del boton, el cero de los dos lados cae en
        # el mismo instante que el cronometro del juez, que es donde debe caer.
        if estado_antes == Estado.ESPERA and self.fsm.estado == Estado.ARRANQUE:
            self.contador.reiniciar()
            self.enlace.calibrar(proto.CAL_CERO_LINEAS)
            self.carril.reiniciar()
            self.esquivador.reiniciar()
            if self.verbose:
                print("[piloto] ronda iniciada: contadores a cero")

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
            "n_pilares": len(esc.pilares),
            # Por que se descarto lo que no llego a pilar. Si el carro ignora
            # los colores, esto dice si es que la mascara no ve nada o si es
            # un filtro el que se los come.
            "descartes": dict(esc.descartes),
            "frente_mm": round(sal_carril.dist_frente_mm),
            "izq_mm": (round(sal_carril.lat_izq_mm)
                       if sal_carril.lat_izq_mm is not None else None),
            "der_mm": (round(sal_carril.lat_der_mm)
                       if sal_carril.lat_der_mm is not None else None),
            "rumbo_hueco": round(sal_carril.rumbo_hueco_deg, 1),
            "sentido_curva": sal_carril.sentido_curva,
            "muro_encima": sal_carril.muro_encima,
            "sesgo": round(sal_carril.sesgo, 1),
            "tras_pilar": tras_pilar,
            "carril_motivo": sal_carril.motivo,
            "guardia": round(guardia_muro(sal_carril, self.cfg["carril"]), 1),
            "comp_err_rumbo": round(maniobra.err_rumbo_deg, 1),
            "magenta": round(empujon, 1) if empujon is not None else None,
            "yaw": round(sens.yaw, 1),
            "linea": ev.linea_nueva,
            "esquina_abierta": ev.esquina_abierta,
            "margen_pilar": maniobra.info.get("margen_pedido_mm"),
            "vueltas": ev.vueltas,
            "secciones": ev.secciones,
            "sentido": ev.sentido,
            "sentido_firme": ev.sentido_firme,
            "origen_sentido": ev.origen_sentido,
            # LAS DOS FUENTES DEL SENTIDO, ENFRENTADAS. Las lineas del piso lo
            # dicen por el orden del par de la esquina; el carril lo aprende
            # del rumbo del hueco en la primera curva. Si discrepan, una de las
            # dos esta mal y conviene enterarse en el taller y no en la ronda:
            # lo mas probable es `color_entrada_horario` al reves, porque el
            # aprendizaje del carril no depende de ningun parametro.
            "sentido_discrepa": bool(
                ev.sentido_firme and sal_carril.sentido_curva and
                ev.sentido != sal_carril.sentido_curva),
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
