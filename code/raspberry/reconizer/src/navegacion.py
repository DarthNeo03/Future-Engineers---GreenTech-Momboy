"""
navegacion.py — No chocar con los muros, y dar tres vueltas exactas.

============================================================================
QUE ESTABA MAL EN LA VERSION ANTERIOR (y como se arregla aqui)
============================================================================
Se observaron cuatro sintomas en pista. Los cuatro salian de la MISMA causa
de fondo: la estrategia era "centrado", que mide espacio libre a izquierda y
derecha sin saber cual de los dos muros es cual.

  1. "Se mantiene mas del lado externo que del interno."
     El centrado busca el punto medio del corredor. Como el muro interno se
     acaba en cada esquina, la media del lado interno sale mas grande y el
     PD empuja hacia fuera. Ahora se sigue explicitamente el muro INTERNO a
     una distancia objetivo en milimetros (`pared_objetivo_mm`), asi que el
     sesgo es hacia dentro por construccion. Ademas hay un guardia duro
     contra el muro externo, que el reglamento prohibe tocar (regla 9.18).

  2. "Parece que toma el muro externo para decidir cuando girar."
     Es exactamente lo que hacia: disparaba cuando el espacio de FRENTE (que
     es el muro externo) bajaba de un umbral. Y eso no puede funcionar,
     porque la distancia correcta a la que hay que girar depende de donde
     esta la esquina INTERNA, y esa cambia con el ancho del pasillo (1000 o
     600 mm, sorteado por seccion). Un umbral frontal fijo acierta en uno de
     los dos casos y falla en el otro. Ahora el disparo es
     `z_esquina_interna <= giro_z_mm`, que es invariante.

  3. "Solo funciona en sentido horario."
     No habia deteccion de sentido: `lado_giro` se recalculaba en cada
     esquina comparando bandas, y con el sesgo del punto 1 salia siempre el
     mismo. Ahora el sentido se DEDUCE de la geometria (el lado que tiene la
     esquina convexa es el interno) y se BLOQUEA para toda la carrera.

  4. "Los giros no son de 90 grados, parecen mas."
     Sin MPU6050 el codigo salia del giro cuando el pasillo volvia a abrirse
     (`salir_giro_sobre`), y el pasillo se abre ANTES de haber completado los
     90 grados: sobregiro sistematico. Ahora el giro se cierra sobre el yaw
     con salida proporcional, y si no hay giroscopio se cierra sobre el
     ANGULO del muro interno ajustado por minimos cuadrados (paralelo = giro
     terminado), que es mucho mejor que un umbral de espacio libre.

Y el quinto sintoma, "a veces no detecta las paredes negras y choca", se
arregla en geometria.py: contigüidad vertical para no confundir sombras, y
sobre todo dejar de tratar "no veo muro" como "esta despejado".

============================================================================
LA MAQUINA DE ESTADOS
============================================================================
    BUSCANDO -> RECTO <-> GIRO -> ... -> FINALIZANDO -> FINALIZADO
                  \\-> BLOQUEADO -/

  BUSCANDO    Aun no se sabe el sentido. Avanza despacio centrado y espera a
              ver la primera esquina convexa. Dura unos metros como mucho.
  RECTO       Sigue el muro interno con PD sobre error lateral Y de rumbo.
  GIRO        90 grados cerrados por giroscopio (o por paralelismo del muro).
  BLOQUEADO   Algo muy cerca: retrocede girando al reves para reencuadrar.
  FINALIZANDO Tres vueltas hechas: entra en la seccion de salida y para.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

from . import geometria as geo

BUSCANDO = "buscando"
RECTO = "recto"
GIRO = "giro"
BLOQUEADO = "bloqueado"
FINALIZANDO = "finalizando"
FINALIZADO = "finalizado"

GIROS_POR_VUELTA = 4
VUELTAS_OBJETIVO = 3


# ---------------------------------------------------------------------------
@dataclass
class Decision:
    vel: int = 0                 # % con signo
    direccion: int = 0           # % -100 izquierda .. +100 derecha
    estado: str = BUSCANDO
    motivo: str = ""
    metricas: Dict[str, float] = field(default_factory=dict)


class _PD:
    """PD con derivada sobre el tiempo real (los frames no llegan parejos)."""

    def __init__(self):
        self.prev: Optional[float] = None
        self.t_prev: float = 0.0

    def paso(self, err: float, kp: float, kd: float, ahora: float) -> float:
        d = 0.0
        if self.prev is not None:
            dt = max(1e-3, ahora - self.t_prev)
            d = (err - self.prev) / dt
        self.prev = err
        self.t_prev = ahora
        return kp * err + kd * d

    def reiniciar(self):
        self.prev = None


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _norm_angulo(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def _dif_angulo(objetivo: float, actual: float) -> float:
    return _norm_angulo(objetivo - actual)


# ---------------------------------------------------------------------------
class Navegador:
    """Escaneo metrico -> velocidad y direccion.

    La configuracion se lee en cada llamada, asi que mover un slider cambia
    el comportamiento en el mismo frame.
    """

    def __init__(self, cfg_nav: Dict[str, Any], cfg_lim: Dict[str, Any]):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.pd_pared = _PD()
        self.pd_senal = _PD()
        self.reiniciar()

    # -- ciclo de vida ----------------------------------------------------
    def reiniciar(self) -> None:
        self.estado = BUSCANDO
        self._t = time.time()
        self.t_estado = self._t
        self.lado_interno: int = 0          # 0 = aun sin decidir, -1 izq, +1 der
        self.confianza_lado = 0             # frames seguidos viendo el mismo lado
        self._lado_tentativo = 0
        self.rumbo_objetivo: Optional[float] = None
        self.yaw_prev: Optional[float] = None
        self.yaw_acumulado = 0.0            # sin envolver: 3 vueltas = 1080 grados
        self.giros = 0
        self.vueltas = 0
        self.dist_desde_giro_mm = 0.0
        self._t = time.time()
        self.t_ultimo_paso = self._t
        self.pd_pared.reiniciar()
        self.pd_senal.reiniciar()
        self.ultimo = Decision()
        self.ultimo_salto: Optional[geo.Salto] = None
        self.aviso_sin_yaw = False
        self.frente_mm = geo.Z_MAX_MM
        self._frente_visto = 0.0
        self._dyaw = 0.0
        self.ancho_corredor_mm = float(self.cfg.get("ancho_corredor_inicial_mm", 800.0))
        self.ancho_visto_t = -1e9        # cuando se midio el ancho por ultima vez
        self.ref_lateral = "?"          # que muro se esta usando de referencia
        self.esq_x: Optional[float] = None    # esquina interna seguida por estima
        self.esq_z: Optional[float] = None
        self.esq_edad_s = 0.0
        self.esq_medida = False               # True si se vio en este frame
        self.esq_rechazos = 0                 # medidas incoherentes seguidas
        self._obj_lat = None                  # ultimo desvio pedido por una señal
        self.bloq_lado = 0                    # maniobra de desatasco
        self.bloq_t0 = 0.0
        self.bloq_fase = 0                    # 0 = atras, 1 = adelante
        self.bloq_intentos = 0
        self._obj_lat_z = None
        self._obj_lat_mm = 0.0                # mm recorridos desde que se solto

    def fijar_lado_interno(self, lado: int, motivo: str = "") -> None:
        """Permite que otra fuente (el sensor de color leyendo el orden de las
        lineas naranja/azul) fije el sentido antes de la primera esquina."""
        if lado in (-1, 1) and self.lado_interno == 0:
            self.lado_interno = lado
            self.confianza_lado = 99
            if self.estado == BUSCANDO:
                self._cambiar(RECTO)

    def _cambiar(self, estado: str) -> None:
        if estado != self.estado:
            self.estado = estado
            self.t_estado = self._t

    @property
    def terminado(self) -> bool:
        return self.estado == FINALIZADO

    # -- integracion de yaw y de distancia --------------------------------
    def _integrar(self, yaw: Optional[float], vel_pct: float, dt: float) -> None:
        """vel_pct va CON SIGNO: retroceder resta distancia, no suma."""
        self._dyaw = 0.0
        if yaw is not None:
            if self.yaw_prev is not None:
                # Convenio de brujula: positivo = giro a la DERECHA.
                self._dyaw = _norm_angulo(yaw - self.yaw_prev)
                self.yaw_acumulado += self._dyaw
            self.yaw_prev = yaw
        # Odometria pobre: sin encoder solo se puede integrar la velocidad
        # pedida por una constante medida a mano. Sirve para "avanza medio
        # metro y para", no para navegar. Con encoder, sustituir por el
        # contador real que llega en la telemetria.
        mm_s = float(self.cfg.get("mm_por_seg_a_100", 900.0)) * (vel_pct / 100.0)
        self.dist_desde_giro_mm += mm_s * dt

    # -- deteccion de sentido ---------------------------------------------
    def _actualizar_lado(self, salto: Optional[geo.Salto]) -> None:
        """El lado con la esquina convexa es el INTERNO. Se exige verlo varios
        frames seguidos antes de bloquearlo: bloquear el sentido al reves es
        la forma mas rapida de perder la ronda, asi que mas vale tardar."""
        if self.lado_interno != 0:
            return
        if salto is None:
            self.confianza_lado = max(0, self.confianza_lado - 1)
            return
        if self.confianza_lado and salto.lado != self._lado_tentativo:
            self.confianza_lado = 0
        self._lado_tentativo = salto.lado
        self.confianza_lado += 1
        if self.confianza_lado >= int(self.cfg.get("frames_para_fijar_lado", 4)):
            self.lado_interno = salto.lado
            self._cambiar(RECTO)

    # -- ciclo principal --------------------------------------------------
    def paso(self, e: geo.Escaneo, yaw: Optional[float] = None,
             objetivo_lateral: Optional[float] = None,
             motivo_extra: str = "", ahora: Optional[float] = None,
             objetivo_z: Optional[float] = None) -> Decision:
        """Un frame. `objetivo_lateral` en mm lo usa el modo de obstaculos
        para desviar la trayectoria alrededor de una señal sin tocar nada mas
        de la logica de giro."""
        # El reloj se puede INYECTAR. Por defecto es el de pared, pero el
        # simulador y las pruebas pasan tiempo simulado: si no, un bucle sin
        # espera hace creer al navegador que entre frame y frame pasan 1.5 ms,
        # la estima de la esquina no avanza y los temporizadores en ms no
        # llegan a cumplirse nunca. Ademas asi las pruebas son deterministas.
        ahora = time.time() if ahora is None else float(ahora)
        # Si el reloj cambia de fuente -de time.time() a uno inyectado, que es
        # lo que hacen el simulador y las pruebas- las marcas de tiempo que se
        # pusieron con el reloj viejo quedan EN EL FUTURO. Entonces los
        # "milisegundos transcurridos" salen negativos y todas las esperas
        # minimas dejan de cumplirse jamas: el disparo del giro se queda
        # desactivado sin que nada lo diga.
        if self.t_estado > ahora or self.t_ultimo_paso > ahora + 1.0:
            self.t_estado = ahora
            self.t_ultimo_paso = ahora
            self._frente_visto = ahora
            self.bloq_t0 = ahora
        dt = min(0.2, max(1e-3, ahora - self.t_ultimo_paso))
        self.t_ultimo_paso = ahora
        self._t = ahora
        self._integrar(yaw, self.ultimo.vel, dt)

        cfg = self.cfg
        v_crucero = float(self.lim.get("vel_crucero", 45))
        v_giro = float(self.lim.get("vel_giro", 32))
        dir_max = float(self.lim.get("dir_max", 100))

        salto = geo.buscar_salto(e, cfg)
        if salto is not None:
            self.ultimo_salto = salto
        ds = float(cfg.get("mm_por_seg_a_100", 900.0)) * (self.ultimo.vel / 100.0) * dt
        self._seguir_esquina(salto, self.lado_interno, self._dyaw, ds, dt,
                             e.lateral(self.lado_interno) if self.lado_interno else None)

        # --- el desvio por una señal se MANTIENE hasta haberla rebasado ----
        # El detector suelta la señal cuando la tiene tan cerca que ya no la ve
        # bien (~220 mm). Si en ese momento el control de pared recupera el
        # mando, tira del carro de vuelta a su distancia objetivo... justo
        # encima del pilar que estaba esquivando. Hay que sostener el desvio
        # hasta que el pilar quede fisicamente atras.
        if objetivo_lateral is not None:
            self._obj_lat = objetivo_lateral
            self._obj_lat_z = objetivo_z
            self._obj_lat_mm = 0.0
        elif self._obj_lat is not None:
            self._obj_lat_mm += abs(ds)
            if self._obj_lat_mm >= float(cfg.get("senal_mantener_mm", 450.0)):
                self._obj_lat = None
            else:
                objetivo_lateral = self._obj_lat
                objetivo_z = None            # ya la tenemos encima
                if not motivo_extra:
                    motivo_extra = f"rebasando señal ({self._obj_lat_mm:.0f} mm)"
        semiancho = float(cfg.get("semiancho_carro_mm", 110.0))
        frente = self._frente_con_memoria(e, semiancho, dt)
        cobertura = e.cobertura()

        if self.estado == FINALIZADO:
            return self._salida(0, 0, e, yaw, "tres vueltas completadas", frente)

        # ---------- seguridad, por encima de cualquier estrategia ---------
        parar_mm = float(cfg.get("parar_mm", 190.0))
        if frente < parar_mm and self.estado not in (FINALIZADO,):
            self._cambiar(BLOQUEADO)
        elif self.estado == BLOQUEADO and frente > float(cfg.get("salir_bloqueo_mm", 340.0)):
            self.bloq_lado = 0                      # maniobra terminada
            self.bloq_intentos = 0
            self.pd_pared.reiniciar()
            self._olvidar_esquina()
            self._cambiar(RECTO if self.lado_interno else BUSCANDO)

        if self.estado == BLOQUEADO:
            return self._desatascar(e, yaw, v_giro, dir_max, frente)

        # ---------- sentido aun sin decidir -------------------------------
        if self.lado_interno == 0:
            self._actualizar_lado(salto)

        if self.estado == BUSCANDO:
            # Centrado prudente hasta ver la primera esquina convexa. Aqui no
            # se esquivan señales: sin saber el sentido no se sabe siquiera
            # donde esta el carril, y un desvio a ciegas impide ver la esquina
            # que hace falta para fijarlo.
            izq = e.lateral(-1)
            der = e.lateral(+1)
            izq = 0.0 if izq is None else izq
            der = 0.0 if der is None else der
            err = (der - izq) / max(1.0, izq + der)
            direccion = _lim(err * float(cfg.get("kp_centrado", 120.0)), -dir_max, dir_max)
            vel = min(v_giro, v_crucero * 0.7)
            vel = self._frenar_por_frente(vel, frente, cobertura)
            return self._salida(vel, direccion, e, yaw,
                                f"buscando sentido ({self.confianza_lado})", frente)

        lado = self.lado_interno

        # ---------- giro en curso -----------------------------------------
        if self.estado == GIRO:
            return self._paso_giro(e, yaw, lado, v_giro, dir_max, frente, salto=salto)

        # ---------- disparo del giro --------------------------------------
        # La referencia es SIEMPRE la esquina convexa del muro interno, que es
        # el pivote real de la maniobra. La distancia frontal solo actua como
        # red de seguridad por si la esquina no se llega a ver.
        # En FINALIZANDO ya no se toman esquinas: las tres vueltas estan hechas
        # y lo unico que queda es entrar en la seccion de salida y pararse. Sin
        # esto el carro encadenaba un giro numero 13 y seguia dando vueltas.
        # En FINALIZANDO ya no se toman esquinas: las tres vueltas estan hechas
        # y solo queda entrar en la seccion de salida y pararse.
        #
        # OJO, LIMITACION CONOCIDA DEL MODO DE OBSTACULOS: un pilar tapa el
        # muro que tiene detras y abre un hueco en el perfil; a los dos lados
        # de ese hueco los rangos no coinciden y se parece a la discontinuidad
        # que delata una esquina convexa. En el simulador con pilares eso
        # provoca algun giro de 90 grados en mitad de la recta. No se puede
        # arreglar simplemente bloqueando el giro mientras se ve una señal,
        # porque con varios pilares por recta casi siempre se ve alguno y el
        # carro no giraria nunca: hace falta validar la esquina candidata
        # contra la geometria del corredor. Pendiente.
        estable = (self.estado != FINALIZANDO
                   and (ahora - self.t_estado) * 1000
                   >= float(cfg.get("min_recto_ms", 500)))
        disparo_z = self._z_disparo()
        dispara = False
        razon = ""
        if estable and self.esq_z is not None and self.esq_z <= disparo_z:
            visto = "vista" if self.esq_medida else f"por estima {self.esq_edad_s:.1f}s"
            dispara, razon = True, f"esquina interna a {self.esq_z:.0f} mm ({visto})"
        elif estable and frente <= float(cfg.get("giro_frente_mm", 300.0)):
            # Red de seguridad: si por lo que sea no hay esquina que seguir, no
            # se empotra contra el muro de enfrente.
            dispara, razon = True, f"red de seguridad: frente a {frente:.0f} mm"

        if dispara:
            self._iniciar_giro(yaw, lado)
            return self._paso_giro(e, yaw, lado, v_giro, dir_max, frente, razon,
                                   salto=salto)

        # ---------- recta: seguir el muro interno --------------------------
        return self._paso_recto(e, yaw, lado, v_crucero, dir_max, frente,
                                cobertura, objetivo_lateral, motivo_extra, salto,
                                objetivo_z)

    # -- geometria del disparo del giro -----------------------------------
    def _z_disparo(self) -> float:
        """A que distancia de la esquina interna hay que empezar a girar.

        Sale de la geometria del giro, no de un numero a ojo. Con el carro
        apuntando a lo largo del corredor y girando con radio R, un giro de 90
        grados lo desplaza EXACTAMENTE R hacia el lado del giro. Si al terminar
        queremos quedar a `objetivo` del muro interno nuevo (que es la cara
        lateral de la misma esquina), entonces hay que arrancar el giro cuando

            z_esquina = R - objetivo

        Con R = 350 y objetivo = 250 salen 100 mm, no los 320 que puse a ojo:
        disparar a 320 termina el giro a solo 30 mm del muro interno.

        Y fijate que NO depende del ancho del corredor. Eso es justamente lo
        que hace falta, porque el ancho del corredor siguiente no se puede
        medir desde el actual.
        """
        cfg = self.cfg
        manual = float(cfg.get("giro_z_mm", 0.0))
        if manual > 0:
            return manual
        R = float(cfg.get("radio_giro_mm", 350.0))
        objetivo = float(cfg.get("pared_objetivo_mm", 250.0))
        return _lim(R - objetivo, 40.0, 600.0)

    # -- seguimiento de la esquina interna --------------------------------
    def _seguir_esquina(self, salto: Optional[geo.Salto], lado: int,
                        dyaw: float, ds: float, dt: float,
                        d_interno: Optional[float] = None) -> None:
        """Mantiene la esquina convexa en el marco del carro aunque no se vea.

        ====================================================================
        POR QUE HACE FALTA ESTIMA Y NO BASTA CON MIRAR
        ====================================================================
        La esquina interna es el pivote del giro, pero medida en la pista real
        resulta que se ve al PRINCIPIO de la recta y desaparece justo cuando te
        acercas a ella, que es cuando hace falta:

            carro en el corredor sur, esquina interna a 500 mm de lado
              a 900 mm de ella  -> rumbo 29 grados  -> se ve
              a 500 mm          -> rumbo 45 grados  -> justo en el borde
              a 320 mm          -> rumbo 57 grados  -> FUERA del encuadre
              a 100 mm          -> rumbo 79 grados  -> muy fuera

        Con HFOV de 100 grados el limite son 50. O sea que disparar el giro
        con "la esquina esta a 320 mm" era imposible: a esa distancia la
        camara ya no la ve. Y el otro camino, disparar por distancia frontal,
        tampoco vale: la distancia correcta depende del ancho del corredor
        SIGUIENTE, que desde aqui no se puede medir.

        La salida es acordarse. Se ve la esquina pronto, se anota donde esta, y
        despues se arrastra con el giroscopio y la velocidad. Solo hay que
        aguantar el ultimo metro, asi que la deriva no da tiempo a importar.
        Es el mismo principio que hace que la memoria sustituya al campo de
        vision en el punto ciego lateral.
        """
        self.esq_medida = False
        if self.estado == GIRO:
            # Durante el giro la escena barre 90 grados y cualquier medida es
            # ruido. La esquina que interesa es la SIGUIENTE, y se readquiere
            # sola en cuanto se vuelve a recta.
            self.esq_x = self.esq_z = None
            return
        if self.esq_x is not None:
            # Estima: el carro avanza ds y gira dyaw (positivo = a la derecha).
            z = self.esq_z - ds
            x = self.esq_x
            c, sn = math.cos(math.radians(dyaw)), math.sin(math.radians(dyaw))
            self.esq_x = x * c - z * sn
            self.esq_z = z * c + x * sn
            self.esq_edad_s += dt

        if salto is not None and (lado == 0 or salto.lado == lado):
            # Una medida fresca NO manda automaticamente: tiene que ser
            # COHERENTE con lo que ya se estaba siguiendo. Sin esta puerta,
            # cualquier discontinuidad que aparezca en otro sitio de la pista
            # secuestra el seguimiento, y como suele reaparecer en el mismo
            # sitio, la esquina se queda congelada a una distancia fija.
            aceptar = True
            motivo = ""

            # 1) La esquina interna ES el borde del muro interno, asi que su
            #    posicion lateral tiene que parecerse a la distancia a ese muro.
            if d_interno is not None:
                tol = float(self.cfg.get("esquina_tol_lateral_mm", 260.0))
                if abs(abs(salto.x) - d_interno) > tol:
                    aceptar, motivo = False, "lateral incoherente con el muro"

            # 2) Si ya se estaba siguiendo una y la estima es reciente, la
            #    nueva tiene que caer cerca de la prediccion.
            if aceptar and self.esq_x is not None and self.esq_edad_s < 2.0:
                salto_max = float(self.cfg.get("esquina_gate_mm", 400.0))
                d = math.hypot(salto.x - self.esq_x, salto.z - self.esq_z)
                if d > salto_max:
                    aceptar, motivo = False, "salta lejos de la prediccion"

            if aceptar:
                self.esq_x, self.esq_z = salto.x, salto.z
                self.esq_edad_s = 0.0
                self.esq_medida = True
                self.esq_rechazos = 0
                return
            self.esq_rechazos += 1
            # Si se rechazan muchas seguidas, puede que la buena sea la nueva:
            # se suelta la vieja y se readquiere limpio.
            if self.esq_rechazos > int(self.cfg.get("esquina_rechazos_max", 20)):
                self.esq_x = self.esq_z = None
                self.esq_rechazos = 0

        if self.esq_x is None:
            return
        # Caducidad: si lleva demasiado sin verse, o ya quedo claramente atras,
        # se olvida. Arrastrar una esquina fantasma es peor que no tener ninguna.
        if (self.esq_edad_s > float(self.cfg.get("esquina_memoria_s", 6.0))
                or self.esq_z < -float(self.cfg.get("esquina_atras_mm", 400.0))):
            self.esq_x = self.esq_z = None

    def _olvidar_esquina(self) -> None:
        self.esq_x = self.esq_z = None
        self.esq_edad_s = 0.0

    # -- referencia lateral -----------------------------------------------
    def _referencia(self, e: geo.Escaneo, lado: int,
                    salto: Optional[geo.Salto]) -> Tuple[float, float, str]:
        """(error_lateral, angulo, motivo) usando el muro que de verdad se vea.

        ====================================================================
        POR QUE NO SE PUEDE SEGUIR SIEMPRE EL MURO INTERNO
        ====================================================================
        Parecia lo natural: el reglamento prohibe tocar el exterior, luego
        sigamos el interior. Pero con una camara frontal la geometria no lo
        permite, y se ve midiendo en la pista real:

          Carro en el corredor sur mirando al este, HFOV 100 grados
            x=1500  ->  muro interno  32 columnas  (esquina aun en el encuadre)
            x=1610  ->  muro interno   0 columnas  (la esquina ya salio)
            cualquiera -> muro externo ~140 columnas SIEMPRE

        El muro interno TERMINA en la esquina, asi que subtiende un angulo
        diminuto y desaparece del encuadre en cuanto la esquina se sale del
        campo de vision: aproximadamente en la primera mitad de cada recta. El
        externo, en cambio, es continuo y siempre esta ahi.

        Asi que la referencia es:
          1. el muro INTERNO mientras se vea (primera parte de la recta, justo
             despues de cada giro), porque impone el sesgo directamente;
          2. si no, el muro EXTERNO, con el objetivo trasladado usando el ancho
             del corredor estimado. Seguir el externo NO significa arrimarse a
             el: significa medirlo para colocarse a `ancho - objetivo`.

        El ancho no se puede dar por supuesto (1000 o 600 mm, sorteado por
        seccion), asi que se mide: cuando se ven los dos muros, o con la
        posicion lateral de la esquina convexa, que es el borde del interno.
        """
        cfg = self.cfg
        z0 = float(cfg.get("recta_z_desde_mm", 120.0))
        z1 = float(cfg.get("recta_z_hasta_mm", 900.0))
        objetivo = float(cfg.get("pared_objetivo_mm", 250.0))

        r_int = e.recta(lado, z0, z1)
        r_ext = e.recta(-lado, z0, z1)
        d_int = r_int[0] if r_int else e.lateral(lado, z0, z1)
        d_ext = r_ext[0] if r_ext else e.lateral(-lado, z0, z1)

        # --- estimacion del ancho del corredor ---------------------------
        medida = None
        if d_int is not None and d_ext is not None:
            medida = d_int + d_ext
        elif salto is not None and salto.lado == lado and d_ext is not None:
            medida = abs(salto.x) + d_ext
        if medida is not None and 400.0 <= medida <= 1400.0:
            a = float(cfg.get("ancho_mezcla", 0.15))
            self.ancho_corredor_mm = (1 - a) * self.ancho_corredor_mm + a * medida
            self.ancho_visto_t = self._t

        # --- 1) muro interno, si se ve -----------------------------------
        if r_int is not None:
            self.ref_lateral = "int"
            return ((r_int[0] - objetivo) * lado, r_int[1],
                    f"int {r_int[0]:.0f}/{objetivo:.0f} ang{r_int[1]:+.1f}")

        # --- 2) muro externo -----------------------------------------------
        # Aqui hay que separar lo fiable de lo incierto. El muro externo se
        # ajusta con 140 columnas, asi que su ANGULO es una medida excelente:
        # sirve de referencia de rumbo sin ninguna duda. Pero traducir su
        # DISTANCIA a "donde deberia estar el carro" exige saber el ancho del
        # corredor, que es 1000 o 600 segun el sorteo.
        #
        # Y equivocarse ahi es peligroso: con el ancho estimado en 1000 cuando
        # en realidad son 600, el objetivo sale 750 mm desde el exterior, que
        # en ese corredor esta DENTRO del muro interno. El carro se mete solo.
        # Le paso al simulador con deriva y tumbaba 30 de 32 vueltas.
        #
        # Asi que si la medida del ancho no es reciente, se usa el muro externo
        # solo para mantenerse PARALELO (error lateral cero) y se deja que el
        # guardia de proximidad y el muro interno, cuando reaparezca, hagan el
        # posicionamiento.
        if r_ext is not None:
            fresca = (self._t - self.ancho_visto_t) < float(cfg.get("ancho_fiable_s", 4.0))
            if fresca:
                margen = float(cfg.get("min_externo_mm", 150.0)) + 60.0
                obj_ext = _lim(self.ancho_corredor_mm - objetivo, margen,
                               self.ancho_corredor_mm - 150.0)
                self.ref_lateral = "ext"
                return ((r_ext[0] - obj_ext) * (-lado), r_ext[1],
                        f"ext {r_ext[0]:.0f}/{obj_ext:.0f} (ancho~{self.ancho_corredor_mm:.0f})")
            self.ref_lateral = "ext~"
            return 0.0, r_ext[1], f"ext {r_ext[0]:.0f} solo rumbo (ancho sin medir)"

        # --- 3) nada fiable: mantener rumbo ------------------------------
        self.ref_lateral = "-"
        if d_int is not None:
            return (d_int - objetivo) * lado, 0.0, f"int {d_int:.0f} mm (sin recta)"
        return 0.0, 0.0, "sin muros a la vista"

    # -- estados ----------------------------------------------------------
    def _iniciar_giro(self, yaw: Optional[float], lado: int) -> None:
        self._cambiar(GIRO)
        self._obj_lat = None
        self.pd_pared.reiniciar()
        paso = float(self.cfg.get("giro_grados", 90.0)) * lado
        if yaw is not None:
            # RELATIVO AL YAW ACTUAL, nunca encadenado al objetivo anterior.
            #
            # Encadenar (objetivo += 90 sobre el objetivo previo) solo es
            # correcto con un giroscopio sin deriva. Un MPU6050 sin calibrar se
            # va 1-3 grados por segundo y el error se acumula vuelta tras
            # vuelta hasta que el rumbo objetivo no significa nada. Tomando el
            # yaw actual, la deriva solo tiene que aguantar los ~2 segundos que
            # dura UN giro, no los 60 de la carrera entera.
            self.rumbo_objetivo = _norm_angulo(yaw + paso)
        self.dist_desde_giro_mm = 0.0

    def _paso_giro(self, e: geo.Escaneo, yaw: Optional[float], lado: int,
                   v_giro: float, dir_max: float, frente: float,
                   razon: str = "", salto: Optional[geo.Salto] = None) -> Decision:
        """Los 90 grados de la esquina.

        DOS CRITERIOS DE SALIDA REDUNDANTES, y gana el que llegue antes:

          * el GIROSCOPIO, que es preciso pero deriva;
          * la VISION, que no deriva: el muro interno vuelve a estar paralelo
            y la esquina ya quedo atras.

        Depender solo del giroscopio es fragil de un modo que se mide: en el
        simulador, con 2 grados/s de deriva -lo que da un MPU6050 SIN
        calibrar- el giro no llegaba nunca a su objetivo y el carro terminaba
        atravesado. Con las dos fuentes, la que primero diga "ya esta" cierra
        el giro, y ninguna averia de una sola de ellas tumba la vuelta.
        """
        cfg = self.cfg
        transcurrido = (self._t - self.t_estado) * 1000
        venc = transcurrido > float(cfg.get("giro_max_ms", 4000))
        min_ms = max(250.0, float(cfg.get("giro_min_ms", 700)))

        # --- criterio de VISION (no deriva) -------------------------------
        r = e.recta(lado, z_desde=120.0, z_hasta=800.0)
        z_esq = self._z_disparo() * 1.5
        sin_esquina = (salto is None or salto.lado != lado or salto.z > z_esq)
        paralelo = (r is not None
                    and abs(r[1]) < float(cfg.get("giro_paralelo_grados", 12.0)))
        vision_lista = paralelo and sin_esquina and transcurrido > min_ms

        # --- criterio de GIROSCOPIO ---------------------------------------
        err = None
        if yaw is not None and self.rumbo_objetivo is not None:
            err = _dif_angulo(self.rumbo_objetivo, yaw)
        yaw_listo = err is not None and abs(err) <= float(cfg.get("giro_tolerancia", 5.0))

        if venc or yaw_listo or vision_lista:
            fuente = ("tiempo agotado" if venc else
                      "yaw" if yaw_listo else "vision: muro paralelo")
            self._terminar_giro(yaw)
            return self._salida(v_giro, 0, e, yaw, f"giro cerrado por {fuente}", frente)

        if err is not None:
            # Salida PROPORCIONAL: a tope hasta ~35 grados de error y luego
            # aflojando. Mantener el volante clavado hasta la tolerancia es lo
            # que producia los sobregiros que se vieron en pista.
            direccion = _lim(err * float(cfg.get("giro_kp", 2.8)), -dir_max, dir_max)
            direccion, aviso = self._abrir_si_roza(e, lado, direccion)
            return self._salida(
                v_giro, direccion, e, yaw,
                razon or f"giro {abs(err):5.1f} grados restantes{aviso}", frente)

        # Sin giroscopio: volante fijo y salida por vision.
        direccion, aviso = self._abrir_si_roza(
            e, lado, lado * float(cfg.get("dir_giro", 85.0)))
        return self._salida(v_giro, direccion, e, yaw,
                            razon or f"giro (sin giroscopio){aviso}", frente)

    def _abrir_si_roza(self, e: geo.Escaneo, lado: int,
                       direccion: float) -> Tuple[float, str]:
        """Afloja el volante si la esquina interna se esta acercando demasiado.

        La holgura con la que se sale de un giro es `radio_REAL - z_disparo`,
        y `z_disparo` se calcula con el radio CONFIGURADO. Si el configurado
        es MAYOR que el real, se dispara pronto y se sale pegado a la esquina.
        Medido en el simulador, con el carro dando R = 346 mm de verdad:

            radio_cfg 350  ->  holgura 138 mm     (bien)
            radio_cfg 450  ->  holgura  43 mm     (roza)
            radio_cfg 550  ->  holgura   4 mm     (choca, y toca el exterior)

        Lo correcto es MEDIR el radio con el asistente. Este guardia es la
        red: mira lo que hay de verdad por el lado de dentro y abre el arco si
        se acerca. Con radio_cfg 550 sube la holgura de 4 a 29 mm y evita
        tocar el muro exterior.

        Se mide contra la CARROCERIA, no contra el centro del carro: rodeando
        una esquina el vertice puede estar a 40 mm del morro y a 220 mm del
        centro, y midiendo desde el centro el guardia no salta nunca.
        """
        cfg = self.cfg
        umbral = float(cfg.get("giro_min_interno_mm", 130.0))
        d = e.mas_cerca(lado, float(cfg.get("giro_mira_z_mm", 500.0)),
                        float(cfg.get("semi_largo_carro_mm", 150.0)),
                        float(cfg.get("semiancho_carro_mm", 110.0)))
        if d is None or d >= umbral:
            return direccion, ""
        factor = max(float(cfg.get("giro_abrir_min", 0.35)), d / umbral)
        return direccion * factor, f" | ABRO ({d:.0f} mm)"

    def _terminar_giro(self, yaw: Optional[float]) -> None:
        self.giros += 1
        self._olvidar_esquina()
        self.dist_desde_giro_mm = 0.0
        if self.giros % GIROS_POR_VUELTA == 0:
            self.vueltas = self.giros // GIROS_POR_VUELTA
        self.pd_pared.reiniciar()
        if self.vueltas >= VUELTAS_OBJETIVO:
            self._cambiar(FINALIZANDO)
        else:
            self._cambiar(RECTO)

    def _paso_recto(self, e: geo.Escaneo, yaw: Optional[float], lado: int,
                    v_crucero: float, dir_max: float, frente: float,
                    cobertura: float, objetivo_lateral: Optional[float],
                    motivo_extra: str,
                    salto: Optional[geo.Salto] = None,
                    objetivo_z: Optional[float] = None) -> Decision:
        cfg = self.cfg
        ahora = self._t

        if objetivo_lateral is not None:
            # Modo obstaculos: la señal manda sobre la pared. `objetivo_lateral`
            # es DONDE DEBE ESTAR el carro respecto a donde esta ahora, en mm,
            # + = a la derecha. Como el carro esta en x=0 por definicion, el
            # error es directamente ese numero.
            #
            # Pero ACOTADO CONTRA LOS MUROS. El detector de señales no sabe
            # nada de paredes: con un pilar verde pegado al muro interno pedia
            # "vete 380 mm a la izquierda" estando el muro a 250, y el carro se
            # metia en el. Esquivar una señal nunca puede justificar salirse
            # del corredor: el pilar se puede tocar sin perder la ronda si no
            # se mueve, el muro no.
            hueco = float(cfg.get("hueco_muro_senal_mm", 150.0))
            d_izq = e.lateral(-1)
            d_der = e.lateral(+1)
            # Si un muro no se ve, NO se deja el lado sin tope: se deduce del
            # ancho estimado del corredor y del muro que si se ve. Dejarlo
            # abierto era peligroso de verdad -el carro se iba contra el muro
            # exterior esquivando un pilar- porque justamente el muro que menos
            # se ve es el que esta mas cerca del borde del encuadre.
            ancho = self.ancho_corredor_mm
            if d_izq is None and d_der is not None:
                d_izq = max(0.0, ancho - d_der)
            if d_der is None and d_izq is not None:
                d_der = max(0.0, ancho - d_izq)
            tope_izq = -(d_izq - hueco) if d_izq is not None else -hueco
            tope_der = (d_der - hueco) if d_der is not None else hueco
            if tope_izq > tope_der:                 # corredor mas estrecho que
                tope_izq = tope_der = 0.0           # el hueco: quedarse quieto
            err_lat = _lim(objetivo_lateral, tope_izq, tope_der)
            _r = e.recta(lado, float(cfg.get("recta_z_desde_mm", 120.0)),
                         float(cfg.get("recta_z_hasta_mm", 900.0)))
            ang = _r[1] if _r is not None else 0.0
            usa_senal = True
            motivo = motivo_extra or "esquivando señal"
            if err_lat != objetivo_lateral:
                motivo += " (acotado por muro)"
        else:
            err_lat, ang, motivo = self._referencia(e, lado, salto)
            usa_senal = False

        # ------------------------------------------------------------------
        # CONTROL EN CASCADA: error lateral -> angulo deseado -> direccion.
        #
        # La version directa (direccion = kp * error_lateral) parece mas simple
        # pero se lanza contra la pared: con el carro en el centro de un
        # corredor de 1000 mm y un objetivo de 250, el error inicial es de
        # 250 mm y cualquier kp razonable satura el volante. El carro cruza el
        # corredor en diagonal, se pasa de largo, pierde el muro de vista al
        # salirse del campo de vision, y a partir de ahi navega a ciegas.
        #
        # En cascada el error lateral NO manda sobre el volante: manda sobre el
        # ANGULO DE APROXIMACION, y ese angulo esta acotado a `aprox_max_grados`.
        # Da igual lo lejos que estes de la pared, nunca te acercas a ella con
        # mas de ~20 grados. El volante solo persigue ese angulo.
        #
        # Y sale gratis una ventaja: el angulo del muro es, en la practica, la
        # derivada del error lateral respecto al avance. O sea que el lazo
        # interior ya aporta la amortiguacion que antes se buscaba con un
        # termino derivativo sobre una medida ruidosa.
        # ------------------------------------------------------------------
        if usa_senal and objetivo_z is not None and objetivo_z > 1.0:
            # ---------------- PERSECUCION PURA hacia la señal --------------
            # Aqui NO vale el esquema en cascada de la pared, y confundirlos
            # cuesta caro: `atan2(err_lat, z)` es el rumbo del punto objetivo
            # medido DESDE EL CARRO, mientras que `ang` es el angulo del MURO.
            # Son dos marcos distintos. Igualandolos, el mando salia casi cero
            # justo cuando mas falta hacia -en el simulador el carro pasaba por
            # encima de los pilares a 7 mm de su centro creyendo que los estaba
            # esquivando-.
            #
            # Con la señal si se sabe donde esta el punto al que hay que llegar
            # y cuanto queda para llegar, asi que se persigue directamente ese
            # punto: se gira en proporcion a su rumbo. Es persecucion pura, y
            # se aprieta sola segun se acerca el pilar.
            #
            # Un pilar se puede rozar sin perder la ronda si no se sale de su
            # circulo (regla 9.20); un muro no. De ahi que aqui se permita mas
            # angulo que contra una pared.
            aprox_max = float(cfg.get("aprox_max_grados_senal", 38.0))
            err_ang = _lim(math.degrees(math.atan2(err_lat, objetivo_z)),
                           -aprox_max, aprox_max)
        else:
            # ---------------- CASCADA contra la pared ----------------------
            # Contra un muro no hay "punto de llegada": la referencia es un
            # angulo de aproximacion acotado, para no atacar la pared en
            # diagonal y pasarse de largo.
            k_aprox = float(cfg.get("aprox_grados_por_mm", 0.06))
            aprox_max = float(cfg.get("aprox_max_grados", 22.0))
            ang_deseado = _lim(-k_aprox * err_lat, -aprox_max, aprox_max)
            err_ang = ang - ang_deseado

        kp_r = float(cfg.get("kp_rumbo", 2.5))        # % de direccion por grado
        kd_r = float(cfg.get("kd_rumbo", 0.15))
        pd = self.pd_senal if usa_senal else self.pd_pared
        (self.pd_pared if usa_senal else self.pd_senal).reiniciar()
        direccion = pd.paso(err_ang, kp_r, kd_r, ahora)

        # --- guardia duro contra el muro externo (regla 9.18) -------------
        # El reglamento prohibe TOCAR el muro exterior en la prueba abierta, y
        # los jueces resuelven las dudas hacia el peor resultado. Asi que si
        # el externo se acerca demasiado, se ignora todo lo demas.
        d_ext = e.lateral(-lado)
        min_ext = float(cfg.get("min_externo_mm", 150.0))
        if d_ext is not None and d_ext < min_ext:
            empuje = (min_ext - d_ext) * float(cfg.get("kp_externo", 0.5))
            direccion += lado * empuje
            motivo += f" | EXTERNO a {d_ext:.0f} mm"

        # --- el giroscopio solo manda cuando la vision no tiene nada -------
        # El ANGULO del muro ya es una referencia de rumbo relativa al
        # corredor y ademas no deriva, asi que mientras se vea una pared el
        # giroscopio no aporta nada y encima estorba: sostener un rumbo
        # absoluto con un sensor que deriva es pelearse con la vision.
        #
        # Y esquivando una señal es peor todavia, porque la maniobra
        # consiste justamente en desviarse del rumbo del corredor:
        # sosteniendo el rumbo a la vez, el giroscopio CANCELABA el esquive
        # y el carro pasaba por encima del pilar creyendo que lo evitaba.
        #
        # Asi que mientras haya referencia se refresca el objetivo con el
        # yaw actual (sin corregir nada), y solo cuando no queda ninguna se
        # usa para mantener la recta.
        if yaw is not None and bool(cfg.get("usar_yaw", True)):
            if usa_senal or self.ref_lateral in ("int", "ext"):
                self.rumbo_objetivo = yaw
            elif self.rumbo_objetivo is not None:
                err_yaw = _dif_angulo(self.rumbo_objetivo, yaw)
                direccion += _lim(err_yaw * float(cfg.get("yaw_kp", 1.2)),
                                  -float(cfg.get("yaw_max", 30.0)),
                                  float(cfg.get("yaw_max", 30.0)))
                motivo += " (rumbo por yaw)"

        vel = self._frenar_por_frente(v_crucero, frente, cobertura)
        # Correr y girar fuerte a la vez es como se sale de la pista.
        vel *= 1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max))

        # --- final de carrera ---------------------------------------------
        if self.estado == FINALIZANDO:
            tope = float(cfg.get("parada_tras_giro_mm", 700.0))
            if self.dist_desde_giro_mm >= tope:
                self._cambiar(FINALIZADO)
                return self._salida(0, 0, e, yaw, "parado en la seccion de salida", frente)
            motivo = f"ultima recta {self.dist_desde_giro_mm:.0f}/{tope:.0f} mm"
            vel = min(vel, float(self.lim.get("vel_giro", 32)))

        return self._salida(vel, direccion, e, yaw, motivo, frente)

    # -- utilidades -------------------------------------------------------
    def _frente_con_memoria(self, e: geo.Escaneo, semiancho: float,
                            dt: float) -> float:
        """Distancia al frente, pero sin creerse que un muro se ha esfumado.

        La camara tiene un suelo de medida (unos 200 mm con el recorte del
        chasis): mas cerca que eso, el muro sale del encuadre y sus columnas
        pasan a invalidas. `Escaneo.frente()` devolveria entonces "despejado",
        que es exactamente el fallo que hace que el carro acelere contra la
        pared en el ultimo palmo.

        Asi que cuando el pasillo se queda sin puntos validos NO se asume via
        libre: se conserva la ultima medida buena y se le va restando lo que
        el carro ha avanzado. Un muro que desaparece por abajo se comporta
        como lo que es, un muro que sigue acercandose.
        """
        cfg = self.cfg
        crudo = e.frente(semiancho)
        hay = bool((e.valido & (np.abs(e.x) <= semiancho)).any())
        umbral = float(cfg.get("memoria_frente_mm", 700.0))

        if hay:
            self.frente_mm = crudo
            self._frente_visto = self._t
            return crudo

        # Sin puntos en el pasillo. Si lo ultimo que vimos estaba lejos, es que
        # de verdad no hay nada: via libre. Si estaba cerca, es que se nos ha
        # metido debajo del encuadre.
        if self.frente_mm >= umbral:
            return crudo
        # CON SIGNO: si el carro esta retrocediendo (estado BLOQUEADO) el muro
        # se ALEJA. Con valor absoluto se acercaba tambien marcha atras, y el
        # carro se quedaba encerrado en BLOQUEADO retrocediendo sin fin.
        avance = float(cfg.get("mm_por_seg_a_100", 900.0)) * (self.ultimo.vel / 100.0) * dt
        self.frente_mm = _lim(self.frente_mm - avance, 0.0, geo.Z_MAX_MM)
        if self._t - self._frente_visto > float(cfg.get("memoria_frente_s", 2.0)):
            self.frente_mm = geo.Z_MAX_MM
        return self.frente_mm

    def _frenar_por_frente(self, vel: float, frente: float, cobertura: float) -> float:
        cfg = self.cfg
        frenar = float(cfg.get("frenar_mm", 620.0))
        parar = float(cfg.get("parar_mm", 190.0))
        v_min = float(self.lim.get("vel_giro", 32))
        if frente < frenar:
            t = (frente - parar) / max(1.0, frenar - parar)
            vel = v_min + (vel - v_min) * _lim(t, 0.0, 1.0)
        # Poca cobertura = la mascara del muro esta fallando. No acelerar
        # hacia lo desconocido: es justo lo que provocaba los choques.
        cob_min = float(cfg.get("cobertura_min", 0.25))
        if cobertura < cob_min:
            vel = min(vel, v_min * 0.7)
        return vel

    def _lado_mas_libre(self, e: geo.Escaneo) -> int:
        """Por que semiplano DELANTERO hay mas sitio. 0 = empate o sin datos.

        Se mira el hueco por delante, no la banda lateral: encajonado contra
        una esquina las bandas laterales se quedan sin puntos y no dicen nada,
        mientras que el reparto de rango entre el semiplano izquierdo y el
        derecho sigue indicando por donde se sale.
        """
        sel = e.valido & (e.z > 0)
        izq = e.z[sel & (e.x < 0)]
        der = e.z[sel & (e.x > 0)]
        mi = float(np.percentile(izq, 30)) if izq.size >= 5 else geo.Z_MAX_MM
        md = float(np.percentile(der, 30)) if der.size >= 5 else geo.Z_MAX_MM
        if abs(mi - md) < 40.0:
            return 0
        return 1 if md > mi else -1

    def _desatascar(self, e: geo.Escaneo, yaw: Optional[float], v_giro: float,
                    dir_max: float, frente: float) -> Decision:
        """Salir de un atasco. Maniobra en varios tiempos, con memoria.

        ====================================================================
        POR QUE LA VERSION ANTERIOR NO SALIA NUNCA
        ====================================================================
        Antes esto era una sola linea: retroceder girando hacia el lado mas
        despejado, recalculado en CADA frame. Y ahi esta el fallo: al
        retroceder cambia lo que la camara ve, el lado "mas despejado" se da la
        vuelta, el volante se invierte, y el carro se queda restregandose
        adelante y atras contra la pared sin avanzar en ninguna direccion. En
        el simulador se quedaba clavado los tres minutos.

        La maniobra correcta con direccion Ackermann es la de toda la vida para
        salir de un aparcamiento: atras con el volante a un lado, adelante con
        el volante al otro, y repetir. Aqui:

          * el lado se decide UNA vez al entrar y se mantiene;
          * se alternan tiempos de marcha atras y de marcha adelante;
          * si tras varios intentos sigue atascado, se invierte el lado, por si
            la primera eleccion era la mala;
          * la marcha adelante solo se permite si hay hueco de verdad delante,
            para no rematar el golpe.
        """
        cfg = self.cfg
        alterna = float(cfg.get("bloqueo_alterna_s", 1.4))
        t_fase = self._t - self.bloq_t0

        if self.bloq_lado == 0:                     # primera vez en el atasco
            self.bloq_lado = self._lado_mas_libre(e) or (self.lado_interno or 1)
            self.bloq_t0 = self._t
            self.bloq_fase = 0
            self.bloq_intentos = 0
            t_fase = 0.0

        if t_fase > alterna:
            self.bloq_fase = 1 - self.bloq_fase
            self.bloq_t0 = self._t
            t_fase = 0.0
            if self.bloq_fase == 0:                 # ciclo completo sin salir
                self.bloq_intentos += 1
                if self.bloq_intentos % 2 == 0:
                    # Puede que el lado elegido fuera el malo: se prueba el otro.
                    self.bloq_lado = -self.bloq_lado

        hueco_delante = frente > float(cfg.get("parar_mm", 280.0)) * 0.8
        if self.bloq_fase == 1 and hueco_delante:
            # Adelante despacio con el volante al lado contrario del retroceso.
            return self._salida(v_giro * 0.5, self.bloq_lado * dir_max * 0.8,
                                e, yaw,
                                f"desatasco: adelante ({self.bloq_intentos})", frente)

        # Marcha atras. Retrocediendo con el volante a un lado, el MORRO gira
        # hacia el contrario: por eso el signo va invertido respecto a adelante.
        return self._salida(-v_giro * 0.6, -self.bloq_lado * dir_max * 0.8,
                            e, yaw,
                            f"desatasco: atras hacia "
                            f"{'der' if self.bloq_lado > 0 else 'izq'} "
                            f"({self.bloq_intentos})", frente)

    def _salida(self, vel: float, direccion: float, e: geo.Escaneo,
                yaw: Optional[float], motivo: str, frente: float) -> Decision:
        dir_max = float(self.lim.get("dir_max", 100))
        d = Decision(
            vel=int(round(_lim(vel, -100, 100))),
            direccion=int(round(_lim(direccion, -dir_max, dir_max))),
            estado=self.estado,
            motivo=motivo,
            metricas=self.metricas(e, yaw, frente),
        )
        self.ultimo = d
        return d

    def metricas(self, e: Optional[geo.Escaneo] = None, yaw: Optional[float] = None,
                 frente: Optional[float] = None) -> Dict[str, float]:
        m: Dict[str, float] = {
            "lado_interno": self.lado_interno,
            "giros": self.giros,
            "vueltas": self.vueltas,
            "yaw_acum": round(self.yaw_acumulado, 1),
            "ancho_mm": round(self.ancho_corredor_mm),
        }
        if e is not None:
            izq = e.lateral(-1)
            der = e.lateral(+1)
            m["izq_mm"] = round(izq, 0) if izq is not None else -1
            m["der_mm"] = round(der, 0) if der is not None else -1
            m["frente_mm"] = round(frente if frente is not None else e.frente(110.0), 0)
            m["cobertura"] = round(e.cobertura(), 2)
        if self.esq_z is not None:
            m["esquina_z"] = round(self.esq_z)
            m["esquina_x"] = round(self.esq_x)
            m["esquina_vista"] = 1 if self.esq_medida else 0
        m["ref"] = self.ref_lateral
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["yaw_obj"] = round(self.rumbo_objetivo, 1)
        return m


# ---------------------------------------------------------------------------
def dibujar_navegacion(frame: np.ndarray, e: geo.Escaneo, d: Decision,
                       nav: Navegador, cfg: Dict[str, Any]) -> np.ndarray:
    """Superpone lo que el carro esta viendo. Es lo que se ve en carrito.local."""
    H, W = frame.shape[:2]

    # --- perfil de contacto, coloreado por validez -----------------------
    for x in range(0, W, 3):
        if not e.valido[x]:
            continue
        y = int(e.y_contacto[x])
        if y <= 0:
            continue
        cv2.circle(frame, (x, y), 1, (0, 255, 255), -1)

    # --- esquina interna detectada (el pivote del giro) ------------------
    s = nav.ultimo_salto
    if s is not None and 0 <= s.columna < W:
        y = int(e.y_contacto[s.columna])
        cv2.drawMarker(frame, (s.columna, max(8, y)), (0, 128, 255),
                       cv2.MARKER_TRIANGLE_DOWN, 18, 2)
        cv2.putText(frame, f"esq {s.z:.0f}mm", (max(0, s.columna - 30), max(20, y - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 128, 255), 1, cv2.LINE_AA)

    # --- lineas por donde pasan las ruedas -------------------------------
    xi = int(W * float(cfg.get("ruedas_izq", 0.32)))
    xd = int(W * float(cfg.get("ruedas_der", 0.68)))
    for x in (xi, xd):
        cv2.line(frame, (x, int(H * 0.45)), (x, H), (255, 255, 255), 1, cv2.LINE_AA)

    # --- volante ---------------------------------------------------------
    cx, cy = W // 2, 26
    x2 = int(cx + int(W * 0.22) * (d.direccion / 100.0))
    cv2.line(frame, (cx, cy), (x2, cy), (0, 165, 255), 4)
    cv2.circle(frame, (cx, cy), 4, (255, 255, 255), -1)

    color = {RECTO: (0, 255, 0), GIRO: (0, 200, 255), BLOQUEADO: (0, 0, 255),
             BUSCANDO: (255, 255, 0), FINALIZANDO: (255, 0, 255),
             FINALIZADO: (255, 255, 255)}.get(d.estado, (255, 255, 255))
    lado = {-1: "izq", 0: "?", 1: "der"}[int(d.metricas.get("lado_interno", 0))]
    cv2.putText(frame, f"{d.estado.upper()} int={lado} v{d.vel:+d} d{d.direccion:+d}"
                       f"  vuelta {int(d.metricas.get('vueltas', 0))}/3",
                (8, H - 94), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if d.motivo:
        cv2.putText(frame, d.motivo[:62], (8, H - 78),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
    m = d.metricas
    cv2.putText(frame, f"izq {m.get('izq_mm', -1):.0f}  frente {m.get('frente_mm', -1):.0f}"
                       f"  der {m.get('der_mm', -1):.0f}  cob {m.get('cobertura', 0):.2f}",
                (8, H - 62), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 200, 255), 1, cv2.LINE_AA)
    return frame
