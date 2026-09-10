"""
navegacion.py — Decidir velocidad y direccion a partir del perfil del muro.

Maquina de estados:

    RECTO -> PRE_GIRO -> GIRO | GIRO_2T -> RECTO   y ESCAPE por encima de todo
      |_________________________________________|

  * RECTO      la estrategia manda ('centrado' o 'pared'), el giroscopio
               corrige el rumbo (acotado: ayuda, no manda) y el esquive de
               pilares aporta su sesgo si esta activo.
  * PRE_GIRO   la esquina ya se disparo pero: (1) se frena ANTES de doblar,
               (2) se espera retardo_giro_ms para que las ruedas TRASERAS
               pasen el canto del muro interno, y (3) si hay sitio, se abre
               hacia el lado contrario (giro abierto, como un camion en un
               cruce de 90) para no cortar la esquina con la cola.
  * GIRO       con giroscopio: rumbo objetivo +-90 y a clavarlo. Sin el:
               direccion fija hasta que el pasillo abre.
  * GIRO_2T    giro de 90 en DOS TIEMPOS (opcional, giro2t.activo): avanza en
               diagonal y luego retrocede con la direccion invertida. Con
               direccion Ackermann eso mantiene el mismo sentido de rotacion,
               asi que el carro gira practicamente sobre el sitio y termina
               ALINEADO con el tramo nuevo, viendo el pasillo entero. Es lo
               que evita que se le escape un pilar por no haberlo visto.
  * ESCAPE     el pasillo se cerro de verdad: marcha atras COMPROMETIDA
               (minimo + extra segun el deficit). Ir y venir cada 500 ms
               frente a un muro es exactamente como se choca; el compromiso
               es la cura. El volante va HACIA el muro para que el morro se
               separe, como al salir de un estacionamiento; pero DENTRO de una
               esquina no se pregunta a la vision: se mete -sentido, que con
               marcha atras rota el morro hacia adentro de la curva
               (_lado_escape).
  * GIRO_COLOR (esquina_color.activo) giro hacia ADENTRO de la pista con
               velocidad variable segun el pasillo, que se SUELTA en el acto
               si aparece un pilar (manda el esquive, en RECTO) y se retoma
               cuando el pilar deja de mandar. Sustituye a GIRO y GIRO_2T
               mientras el modo esta encendido. Solo va hacia adelante; el
               ESCAPE sigue por encima.

EL BUCLE DE LAS ESQUINAS, Y POR QUE HACE FALTA `en_esquina`
Cuando el muro interno se termina queda un hueco de piso blanco muy grande.
Para cualquier navegacion por espacio libre ese hueco ES el camino: el carro
se mete, desde la posicion nueva vuelve a ver otro hueco, se vuelve a meter,
y da vueltas dentro de la curva sin salir. No es un problema de umbrales:
la vision esta contestando bien a la pregunta equivocada.

La solucion es no preguntarle a la vision mientras se esta en la curva. Las
lineas del piso dicen exactamente cuando se entra (lineas.py lleva la zona),
y con `bloqueo_esquina` el giro de 90 se ejecuta COMPROMETIDO: se termina por
angulo de giroscopio o por timeout, nunca porque la camara vea un hueco
tentador. La vision sigue mandando en lo unico que no admite discusion: la
seguridad (ESCAPE si de verdad hay un muro encima).

Sentido de la ronda: geometria pura. HORARIO = el centro de la pista queda a
la DERECHA del carro = muro interno a la derecha = las esquinas doblan a la
derecha. ANTIHORARIO, todo al reves. Si el sentido aun no se conoce, la
esquina se decide por el lado que tenga mas espacio (como el programa viejo).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from .muro import PerfilMuro, DetectorEsquinaInterna

RECTO = "recto"
PRE_GIRO = "pre_giro"
GIRO = "giro"
GIRO_2T = "giro_2t"
ESCAPE = "escape"
GIRO_COLOR = "giro_color"


@dataclass
class Decision:
    vel: int = 0                 # % de vmax, con signo
    direccion: int = 0           # % -100 izquierda .. +100 derecha
    estado: str = RECTO
    motivo: str = ""
    metricas: Dict[str, Any] = field(default_factory=dict)


class _PD:
    def __init__(self):
        self.prev: Optional[float] = None
        self.t_prev = 0.0

    def paso(self, err: float, kp: float, kd: float, ahora: float) -> float:
        d = 0.0
        if self.prev is not None:
            dt = max(1e-3, ahora - self.t_prev)
            d = (err - self.prev) / dt
        self.prev = err
        self.t_prev = ahora
        return kp * err + kd * d * 0.1

    def reiniciar(self):
        self.prev = None


def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _norm_ang(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class Navegador:
    def __init__(self, cfg_nav: Dict[str, Any], cfg_lim: Dict[str, Any],
                 cfg_esc: Dict[str, Any], cfg_2t: Optional[Dict[str, Any]] = None,
                 al_completar_giro: Optional[Callable[[int], None]] = None,
                 cfg_obst: Optional[Dict[str, Any]] = None,
                 cfg_color: Optional[Dict[str, Any]] = None):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.esc = cfg_esc
        self.g2t = cfg_2t if cfg_2t is not None else {}
        self.obst = cfg_obst if cfg_obst is not None else {}
        self.gc = cfg_color if cfg_color is not None else {}
        self.al_completar_giro = al_completar_giro or (lambda lado: None)

        self.pd = _PD()
        self.pd_pared = _PD()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.lado_giro = 0
        self.rumbo_objetivo: Optional[float] = None
        # Rumbo de la RECTA por la que se va. Solo cambia en las curvas de
        # verdad (+-90). Un escape NO puede tocarlo: si el rumbo de referencia
        # se toma del yaw despues de una maniobra de rescate, el carro adopta
        # como "bueno" el rumbo en el que se quedo mirando, y si se quedo
        # mirando hacia atras se va en sentido contrario. El reglamento
        # termina la ronda por eso.
        self.rumbo_recta: Optional[float] = None
        self.ultimo = Decision()

        self._t_fin_escape = 0.0
        self._escape_intentos = 0
        self._pasillo_prev: Optional[float] = None
        self._t_pasillo = 0.0
        self._vel_cierre = 0.0        # mm/s, positivo = el muro se acerca
        # Una esquina, UN giro: sin esto el carro giraria 90 grados otra vez
        # mientras siga dentro de la zona de la curva.
        self._esquina_atendida = False
        # Un giro de RESCATE (salir de un atasco, recuperar el rumbo) usa la
        # misma maquinaria que una curva, pero NO es una esquina: si contara,
        # el marcador de vueltas se dispararia con cada choque.
        self._giro_es_rescate = False

        # --- giro de dos tiempos ---
        self._2t_fase = "avance"      # "avance" | "reversa"
        self._2t_ciclo = 0
        self._2t_t_fase = 0.0
        self._2t_t_inicio = 0.0
        self._2t_acum = 0.0           # grados girados ACUMULADOS (con signo)
        self._2t_yaw_prev: Optional[float] = None

        # --- giro por color ---
        ### pendiente = el giro se solto por un pilar (o lo corto un escape)
        ### y aun no se ha dado por terminado; se retoma cuando el pilar
        ### lleva reanudar_tras_ms fuera de juego
        self._color_pendiente = False
        self._color_apuntado = False  # ya se avanzo el rumbo +-90 para esta esquina
        self._color_t_inicio = 0.0    # para el max_ms total (con reanudaciones)
        self._t_pilar_fuera = 0.0     # desde cuando no hay pilar en juego

        # --- re-anclaje del rumbo ---
        ### _apuntado: el giro en curso ya tiene su objetivo puesto CON yaw.
        ### Si el yaw faltaba justo al entrar en la esquina (enlace.yaw()
        ### devuelve None con 0,4 s sin telemetria) el apuntado se saltaba en
        ### silencio y el giro terminaba nada mas volver el yaw, con la
        ### referencia vieja: el carro se iba al lado contrario.
        self._apuntado = True
        self._t_reanclar = 0.0        # desde cuando se ve la esquina sin registrar
        self._t_ultimo_reanclaje = 0.0
        self.reanclajes = 0

    # ------------------------------------------------------------------
    def reiniciar(self):
        self.pd.reiniciar()
        self.pd_pared.reiniciar()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.rumbo_objetivo = None
        self._escape_intentos = 0
        self._pasillo_prev = None
        self._2t_yaw_prev = None
        self._esquina_atendida = False
        self.rumbo_recta = None
        self._color_pendiente = False
        self._color_apuntado = False
        self._t_pilar_fuera = 0.0
        self._apuntado = True
        self._t_reanclar = 0.0

    @property
    def modo_color(self) -> bool:
        return bool(self.gc.get("activo", False))

    def _lado_escape(self, p: PerfilMuro, sentido: int,
                     en_esquina: bool) -> Tuple[int, str]:
        """Hacia que lado va el volante durante la REVERSA del escape.

        Con direccion Ackermann, reversa + volante a un lado hace rotar el
        MORRO hacia el lado contrario (es la misma geometria que usa el giro
        de dos tiempos: "avance con el volante hacia el giro y reversa con el
        volante al contrario rotan el carro igual"). Asi que para que el morro
        acabe apuntando hacia ADENTRO de la curva hay que meter volante
        -sentido: en antihorario, volante a la DERECHA.

        DENTRO DE UNA ESQUINA eso se sabe sin mirar nada, porque la ronda
        dobla siempre hacia el mismo lado. Antes se decidia tambien aqui por
        "donde se ve mas hueco", y a menos de parar_bajo_mm de la pared, con
        el muro llenando el cuadro, esa diferencia es ruido: salia al reves
        muy a menudo, el carro retrocedia girando hacia el lado equivocado y
        se quedaba encajado en la curva. Es exactamente la leccion que ya
        estaba escrita unas lineas mas arriba para el giro de rescate ("elegir
        donde haya mas hueco es lo que podia dejar al carro encarado hacia
        atras") y que a la reversa no se le habia aplicado.

        FUERA de la esquina sigue mandando la geometria: ahi el muro de
        delante suele ser una pared lateral porque el carro va cruzado, y
        girar "hacia la ronda" lo meteria mas contra ella.
        """
        if sentido != 0 and en_esquina:
            return -sentido, "hacia la curva"
        return (-1 if p.izq < p.der else 1), "separandose del muro"

    def rearmar_esquina(self) -> None:
        """(modo color) Acaba de pisarse una linea buena. Si el carro esta en
        recta y sin giro a medias, la esquina se atiende aunque la zona de la
        anterior siguiera abierta. Si esta girando o con el giro suelto por
        un pilar, ese giro YA es el de esta esquina: no se apila otro."""
        if self.estado == RECTO and not self._color_pendiente:
            self._esquina_atendida = False

    def _cambiar(self, estado: str):
        if estado != self.estado:
            self.estado = estado
            self.t_estado = time.time()

    def error_de_rumbo(self, yaw: Optional[float]) -> Optional[float]:
        """Grados que el carro se desvia del rumbo de la recta actual
        (positivo = apunta mas a la derecha que la recta). Es lo que permite
        saber que pared es cual aunque el carro venga cruzado esquivando."""
        if yaw is None or self.rumbo_objetivo is None:
            return None
        return _norm_ang(yaw - self.rumbo_objetivo)

    # ------------------------------------------------------------------
    def paso(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
             linea_reciente: bool = False,
             bias_obstaculo: Tuple[float, float] = (0.0, 0.0),
             en_esquina: bool = False,
             esquina_confirmada: bool = False,
             pilar_en_juego: bool = False,
             freno_linea: float = 1.0) -> Decision:
        """sentido: +1 horario, -1 antihorario, 0 desconocido.
        pilar_en_juego: (modo color) hay un pilar visto ahora mismo o en el
                    punto ciego adelantandolo. Es lo que SUELTA el giro por
                    color; fuera de ese modo no se usa.
        freno_linea: factor 0..1 sobre la velocidad en recta cuando la camara
                    ve una linea del piso cerca (lineas.frenar_ante_linea).
        bias_obstaculo: (direccion_deseada_pct, peso 0..1) del esquive.
        en_esquina: el carro esta en una curva, por lineas del piso O porque
                    la vision decidio girar. Con bloqueo_esquina, mientras
                    dure eso el giro no se abandona (anti-bucle).
        esquina_confirmada: el carro esta en la curva segun las LINEAS DEL
                    PISO, que es la unica prueba fisica de que hay curva.
                    Es el permiso para el giro de dos tiempos: esa maniobra
                    retrocede, y retroceder en mitad de una recta (porque la
                    vision creyo ver una esquina donde no la hay) es meterse
                    contra lo que venga detras. Sin esta confirmacion se hace
                    el giro normal, que solo va hacia adelante."""
        ahora = time.time()
        cfg, lim, esc = self.cfg, self.lim, self.esc
        vel_crucero = float(lim.get("vel_crucero", 55))
        vel_giro = float(lim.get("vel_giro", 38))
        dir_max = float(lim.get("dir_max", 100))
        pasillo = p.pasillo_mm

        usar_yaw = bool(cfg.get("usar_yaw", True)) and yaw is not None
        if usar_yaw and self.rumbo_objetivo is None:
            self.rumbo_objetivo = yaw
        if usar_yaw and self.rumbo_recta is None:
            self.rumbo_recta = yaw

        # --- velocidad de cierre del pasillo (para el freno por tiempo) ---
        if self._pasillo_prev is not None:
            dt = max(1e-3, ahora - self._t_pasillo)
            inst = (self._pasillo_prev - pasillo) / dt
            self._vel_cierre = 0.7 * self._vel_cierre + 0.3 * inst
        self._pasillo_prev = pasillo
        self._t_pasillo = ahora

        aviso_interna = self.detector_interna.paso(p, cfg)

        # Al salir de la zona de curva se rearma el disparo por linea. Mientras
        # se sigue dentro, la esquina ya atendida no vuelve a disparar: el giro
        # acaba antes de que la zona expire, y sin esto el carro encadenaria
        # giros de 90 hasta meterse en la pared.
        if not en_esquina:
            self._esquina_atendida = False

        # =================== ESCAPE (prioridad maxima) ====================
        # GIRO_2T queda fuera: esa maniobra se acerca al muro a proposito y
        # lleva su propia marcha atras (corta sola en min_pasillo_mm, que va
        # por encima de parar_bajo_mm). Si el escape se metiera en medio, los
        # dos estarian dando ordenes de reversa distintas.
        parar_bajo = float(cfg.get("parar_bajo_mm", 300.0))
        if self.estado not in (ESCAPE, GIRO_2T) and pasillo < parar_bajo:
            if self.estado == GIRO_COLOR:
                ### el giro por color solo va hacia adelante: si el muro se
                ### le echa encima manda el escape, y al volver se retoma
                self._color_pendiente = True
            self._cambiar(ESCAPE)
            deficit = parar_bajo - pasillo
            comp = float(esc.get("escape_min_ms", 750)) + \
                float(esc.get("escape_k_ms_por_mm", 3.0)) * deficit
            self._t_fin_escape = ahora + comp / 1000.0
            self._escape_intentos += 1

        if self.estado == ESCAPE:
            if ahora >= self._t_fin_escape:
                if pasillo > float(cfg.get("girar_bajo_mm", 650.0)) * 0.8:
                    self._escape_intentos = 0
                    self._cambiar(RECTO)
                    self.pd.reiniciar()
                    # Se vuelve al rumbo de LA RECTA, no al que haya quedado
                    # despues de maniobrar. Adoptar el yaw de aqui es como el
                    # carro terminaba yendose en sentido contrario.
                    if usar_yaw:
                        self.rumbo_objetivo = self.rumbo_recta
                elif self._escape_intentos > int(esc.get("escape_max_intentos", 4)):
                    # La reversa no gana espacio (algo detras): giro adelante.
                    # Se gira hacia el lado que ACERCA al rumbo de la recta;
                    # elegir "donde haya mas hueco" es lo que podia dejar al
                    # carro encarado hacia atras.
                    self._escape_intentos = 0
                    if usar_yaw and self.rumbo_recta is not None:
                        err = _norm_ang(self.rumbo_recta - yaw)
                        self.lado_giro = 1 if err >= 0 else -1
                        self.rumbo_objetivo = self.rumbo_recta
                    else:
                        self.lado_giro = 1 if p.der > p.izq else -1
                    self._giro_es_rescate = True
                    self._cambiar(GIRO)
                else:
                    deficit = max(0.0, parar_bajo - pasillo)
                    comp = float(esc.get("escape_min_ms", 750)) + \
                        float(esc.get("escape_k_ms_por_mm", 3.0)) * deficit
                    self._t_fin_escape = ahora + comp / 1000.0
                    self._escape_intentos += 1
            if self.estado == ESCAPE:
                lado, regla = self._lado_escape(p, sentido, en_esquina)
                return self._salida(
                    -float(lim.get("vel_reversa", 35)),
                    lado * float(esc.get("escape_dir", 80.0)),
                    p, yaw, sentido,
                    f"escape #{self._escape_intentos} pasillo={pasillo:.0f}mm "
                    f"({regla})")

        # =================== PRE_GIRO =====================================
        if self.estado == PRE_GIRO:
            if (self.modo_color and pilar_en_juego
                    and bool(self.gc.get("ceder_al_pilar", True))):
                ### pilar a la vista ya en el pre-giro: ni empezar. Se suelta
                ### y cae al bloque RECTO de este mismo tick, que es donde
                ### el esquive tiene el mando.
                self._soltar_color(yaw, usar_yaw)
            elif (ahora - self.t_estado) * 1000 >= float(cfg.get("retardo_giro_ms", 220)):
                if self.modo_color:
                    self._iniciar_color(yaw, usar_yaw)
                # El giro de dos tiempos SOLO se permite en una esquina
                # confirmada por el par de lineas del piso: es la unica
                # maniobra que retrocede, y nunca debe retroceder en recta.
                elif (bool(self.g2t.get("activo", False)) and usar_yaw
                        and esquina_confirmada):
                    self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
                    self._iniciar_2t(yaw)
                else:
                    self._cambiar(GIRO)
                    self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
                    self._apuntado = usar_yaw     # sin yaw: apuntar en cuanto vuelva
            else:
                # giro abierto: contra-direccion si el lado contrario tiene sitio
                apertura = float(cfg.get("apertura_pct", 25.0))
                libre_contrario = (p.izq if self.lado_giro > 0 else p.der) * p.alcance_mm
                d = 0.0
                if apertura > 0 and libre_contrario > float(
                        cfg.get("apertura_min_libre_mm", 400.0)):
                    d = -self.lado_giro * apertura
                return self._salida(vel_giro, d, p, yaw, sentido,
                                    f"pre-giro {'der' if self.lado_giro > 0 else 'izq'}")

        # =================== GIRO_COLOR (hacia adentro, cede al pilar) ====
        if self.estado == GIRO_COLOR:
            d = self._paso_color(p, yaw, usar_yaw, sentido, ahora, pilar_en_juego)
            if d is not None:
                return d
            # None: termino, o se solto por un pilar. En los dos casos el
            # estado ya es RECTO y se sigue abajo en este mismo tick.

        # =================== GIRO_2T (dos tiempos) ========================
        if self.estado == GIRO_2T:
            d = self._paso_2t(p, yaw, sentido, ahora, esquina_confirmada)
            if d is not None:
                return d
            # _paso_2t devuelve None cuando la maniobra termino: cae a RECTO

        # =================== GIRO =========================================
        if self.estado == GIRO:
            venc = (ahora - self.t_estado) * 1000 > float(cfg.get("giro_max_ms", 3000))
            bloqueado = en_esquina and bool(cfg.get("bloqueo_esquina", True))
            if usar_yaw and not self._apuntado:
                ### el yaw faltaba al entrar; ahora que esta, se apunta (el
                ### re-anclaje de _apuntar_a_la_recta_siguiente descuenta lo
                ### que ya se haya girado a ciegas)
                self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
                self._apuntado = True
            if usar_yaw and self.rumbo_objetivo is not None:
                err = _norm_ang(self.rumbo_objetivo - yaw)
                if abs(err) < float(cfg.get("giro_tolerancia_deg", 8.0)) or venc:
                    if venc:
                        self.rumbo_objetivo = yaw
                    self._terminar_giro()
                else:
                    d = _lim(err * float(cfg.get("yaw_kp", 1.6)) * 3.0,
                             -dir_max, dir_max)
                    return self._salida(vel_giro, d, p, yaw, sentido,
                                        f"giro yaw err={err:+.0f}"
                                        + (" [en esquina]" if bloqueado else ""))
            else:
                # Sin giroscopio solo queda la vision para saber cuando acabo
                # el giro; pero DENTRO de la curva el hueco blanco del muro
                # interno abre el pasillo antes de tiempo y ahi nace el bucle.
                # Con bloqueo, se exige ademas haber girado un tiempo minimo.
                abrio = pasillo > float(cfg.get("salir_giro_mm", 950.0))
                if bloqueado:
                    minimo = float(cfg.get("giro_max_ms", 3000)) * 0.45
                    abrio = abrio and (ahora - self.t_estado) * 1000 > minimo
                if abrio or venc:
                    self._terminar_giro()
                else:
                    d = self.lado_giro * float(cfg.get("dir_giro", 85.0))
                    return self._salida(vel_giro, d, p, yaw, sentido,
                                        f"giro vision pasillo={pasillo:.0f}"
                                        + (" [en esquina]" if bloqueado else ""))

        # =================== RECTO ========================================
        # Guardia de sentido: si el rumbo se ha ido mas de desvio_max_deg de la
        # recta, el carro quedo mirando hacia atras (tipico tras varios
        # escapes). Circular en sentido contrario TERMINA LA RONDA, asi que se
        # fuerza un giro de rescate para volver, y ese giro no cuenta esquina.
        if usar_yaw and self.rumbo_recta is not None:
            desvio = _norm_ang(yaw - self.rumbo_recta)
            ### ESQUINA SIN REGISTRAR. Girado ~90 EN EL SENTIDO DE LA RONDA
            ### respecto a la recta, sostenido: el carro doblo una esquina que
            ### el codigo no vio (TCS perdido + centrado por el hueco). Antes
            ### el yaw tiraba hacia la recta vieja = al lado contrario = pared.
            ### Con el sentido desconocido no se toca: no se distingue de una
            ### vuelta sobre si mismo, y de eso se encarga desvio_max_deg.
            if (bool(cfg.get("reanclar_rumbo", True)) and sentido != 0
                    and float(cfg.get("reanclar_desde_deg", 65.0))
                    <= sentido * desvio
                    <= float(cfg.get("reanclar_max_deg", 135.0))):
                if self._t_reanclar == 0.0:
                    self._t_reanclar = ahora
                if (ahora - self._t_reanclar) * 1000 >= float(cfg.get("reanclar_ms", 400)):
                    self.rumbo_recta = _norm_ang(
                        self.rumbo_recta + sentido * float(cfg.get("giro_grados", 90.0)))
                    self.rumbo_objetivo = self.rumbo_recta
                    self.pd.reiniciar()
                    self.reanclajes += 1
                    self._t_ultimo_reanclaje = ahora
                    self._t_reanclar = 0.0
                    desvio = _norm_ang(yaw - self.rumbo_recta)
            else:
                self._t_reanclar = 0.0
            if abs(desvio) > float(cfg.get("desvio_max_deg", 110.0)):
                self.lado_giro = -1 if desvio > 0 else 1
                self.rumbo_objetivo = self.rumbo_recta
                self._giro_es_rescate = True
                self._cambiar(GIRO)
                return self._salida(vel_giro, 0.0, p, yaw, sentido,
                                    f"RESCATE de rumbo: {desvio:+.0f} grados "
                                    f"fuera de la recta")

        # --- giro por color suelto: retomarlo cuando el pilar ya no manda --
        if self.modo_color and self._color_pendiente:
            d = self._reanudar_color(p, yaw, usar_yaw, sentido, ahora,
                                     en_esquina, pilar_en_juego)
            if d is not None:
                return d

        recto_estable = (ahora - self.t_estado) * 1000 >= float(
            cfg.get("min_recto_ms", 700))
        ### en modo color la vision puede quedarse sin voto para DISPARAR
        ### (esquina_color.vision_dispara); frenar y escapar siguen igual
        vision_dispara = (not self.modo_color
                          or bool(self.gc.get("vision_dispara", True)))

        disparo = ""
        # Las lineas del piso marcan fisicamente donde esta la curva: si el
        # carro acaba de cruzar la primera del par, ESTA en la esquina y no
        # hay nada que deliberar. Es el disparo mas fiable de todos, pero solo
        # una vez por curva (_esquina_atendida).
        if (en_esquina and not self._esquina_atendida
                and bool(cfg.get("linea_dispara_esquina", True))):
            disparo = "linea del piso: dentro de la esquina"
        elif en_esquina and self._esquina_atendida:
            pass                     # curva ya girada: a esperar la salida
        elif recto_estable and vision_dispara:
            frontal = p.frontal_mm if bool(cfg.get("usar_rectas", True)) else None
            if frontal is not None and frontal < float(cfg.get("girar_bajo_mm", 650.0)):
                # pared cruzada delante: esto es una esquina identificada, no
                # una pared lateral que parece cercana por ir torcido
                disparo = f"pared de frente a {frontal:.0f}mm"
            elif pasillo < float(cfg.get("girar_bajo_mm", 650.0)):
                disparo = f"pasillo {pasillo:.0f}mm"
            elif aviso_interna is not None:
                lado_aviso = 1 if aviso_interna == "der" else -1
                # si conocemos el sentido, solo cuenta si desaparecio el lado
                # INTERNO (el externo casi nunca desaparece; si lo hace, es ruido)
                if sentido == 0 or lado_aviso == sentido:
                    disparo = f"muro interno ({aviso_interna}) desaparecio"
                    self.lado_giro = lado_aviso
            elif linea_reciente and bool(cfg.get("giro_por_linea", True)) and \
                    pasillo < float(cfg.get("girar_bajo_mm", 650.0)) * 1.6:
                disparo = "linea de esquina + pasillo cerrando"

        if disparo:
            if not self.lado_giro or "pasillo" in disparo or "linea" in disparo:
                self.lado_giro = sentido if sentido != 0 else (
                    1 if p.der > p.izq else -1)
            self._cambiar(PRE_GIRO)
            self._esquina_atendida = True
            return self._salida(vel_giro, 0.0, p, yaw, sentido,
                                f"esquina: {disparo}")

        # --- estrategia ----------------------------------------------------
        if str(cfg.get("estrategia", "centrado")).startswith("par"):
            direccion, motivo = self._dir_pared(p, sentido, ahora)
        else:
            direccion, motivo = self._dir_centrado(p, ahora)

        # --- esquive de pilares (sesgo ponderado) --------------------------
        bias_dir, peso = bias_obstaculo
        if peso > 0.0 and bool(self.obst.get("ceder_ante_muro", True)):
            # Con el pasillo cerrandose, el pilar CEDE el mando a la evitacion
            # de muros. Sin esto, un pilar pegado a la pared interior se lleva
            # al carro de frente contra la esquina: el esquive pesaba mas que
            # el muro justo cuando el muro era el problema.
            holgura = (pasillo - parar_bajo) / max(
                1.0, float(cfg.get("frenar_bajo_mm", 1000.0)) - parar_bajo)
            peso *= _lim(holgura, 0.0, 1.0)
        if peso > 0.0:
            direccion = (1.0 - peso) * direccion + peso * bias_dir
            motivo += f" esq({bias_dir:+.0f}x{peso:.2f})"
        if self._color_pendiente:
            motivo += " [giro color suelto]"
        if ahora - self._t_ultimo_reanclaje < 1.5:
            motivo += " [rumbo re-anclado: esquina sin registrar]"

        # --- rumbo por giroscopio -----------------------------------------
        # OJO CON EL SIGNO DE SUMA: la correccion de rumbo se añade DESPUES de
        # mezclar el esquive. Mientras se esquiva a proposito, el carro se sale
        # del rumbo de la recta y esta correccion tira justo al reves, o sea
        # pelea contra el esquive y ademas SUMA: con el tope del esquive en
        # 55 % y yaw_max en 45 %, el volante llegaba al 100 % y el carro se
        # cruzaba de golpe (la rueda trasera se llevaba el pilar). Mientras el
        # pilar manda, el mantenimiento de rumbo cede en la misma proporcion.
        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            corr = _lim(err * float(cfg.get("yaw_kp", 1.6)),
                        -float(cfg.get("yaw_max", 45.0)),
                        float(cfg.get("yaw_max", 45.0)))
            if peso > 0.0 and bool(cfg.get("yaw_cede_al_esquivar", True)):
                corr *= (1.0 - peso)
            direccion += corr
            motivo += f" yaw{err:+.0f}"

        # --- velocidad -----------------------------------------------------
        frenar = float(cfg.get("frenar_bajo_mm", 1000.0))
        if pasillo >= frenar:
            vel = vel_crucero
        else:
            t = (pasillo - parar_bajo) / max(1.0, frenar - parar_bajo)
            vel = vel_giro + (vel_crucero - vel_giro) * _lim(t, 0.0, 1.0)
            motivo += " frenando"

        # freno por tiempo-hasta-el-muro: la inercia no espera a la distancia
        ttc_min = float(cfg.get("ttc_min_s", 0.7))
        if self._vel_cierre > 60.0:
            ttc = pasillo / self._vel_cierre
            if ttc < ttc_min:
                vel = min(vel, vel_giro * _lim(ttc / ttc_min, 0.35, 1.0))
                motivo += f" ttc={ttc:.1f}s"

        # girar fuerte y correr a la vez es como se sale de la pista
        vel *= 1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max))

        # linea del piso a la vista (o recien cruzada): despacio, que el TCS
        # tenga muestras de sobra al pasar por encima
        if freno_linea < 1.0:
            vel *= _lim(freno_linea, 0.0, 1.0)
            motivo += f" linea x{freno_linea:.2f}"

        return self._salida(vel, direccion, p, yaw, sentido, motivo)

    # ---------------- giro de dos tiempos -------------------------------
    def _iniciar_2t(self, yaw: Optional[float]) -> None:
        self._cambiar(GIRO_2T)
        self._2t_fase = "avance"
        self._2t_ciclo = 0
        self._2t_t_fase = time.time()
        self._2t_t_inicio = time.time()
        self._2t_acum = 0.0
        self._2t_yaw_prev = yaw

    def _paso_2t(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
                 ahora: float, esquina_confirmada: bool = True
                 ) -> Optional[Decision]:
        """Un tick del giro en dos tiempos. Devuelve None si ya termino.

        Avance con el volante hacia el lado del giro y reversa con el volante
        al lado CONTRARIO hacen rotar el carro en el MISMO sentido: es la
        maniobra de dar la vuelta en una calle estrecha. Asi se consiguen los
        90 grados en un espacio en el que un giro de un solo tiempo no cabe,
        y el carro acaba encarado al tramo nuevo en vez de entrar de lado.
        """
        g = self.g2t
        cfg = self.cfg
        objetivo = float(cfg.get("giro_grados", 90.0))
        lado = self.lado_giro or 1

        # Angulo ACUMULADO paso a paso (nunca por diferencia contra el inicio:
        # esa se envuelve y la maniobra no terminaria nunca).
        if yaw is not None:
            if self._2t_yaw_prev is not None:
                self._2t_acum += _norm_ang(yaw - self._2t_yaw_prev)
            self._2t_yaw_prev = yaw
        girado = self._2t_acum * lado          # positivo = va en la buena direccion
        falta = objetivo - girado

        vencido = (ahora - self._2t_t_inicio) * 1000 > float(g.get("max_ms", 7000))
        if falta <= float(cfg.get("giro_tolerancia_deg", 8.0)) or vencido:
            self._terminar_giro(yaw, reanclar=True)
            return None

        if not esquina_confirmada:
            # Se perdio la prueba de que esto es una curva (caduco la zona de
            # lineas). No se retrocede fuera de una esquina: se pasa al giro
            # normal y se completan hacia adelante los grados que falten.
            self._cambiar(GIRO)
            if yaw is not None:
                self.rumbo_objetivo = _norm_ang(yaw + lado * max(0.0, falta))
            return None

        t_fase = (ahora - self._2t_t_fase) * 1000
        vel_av = float(g.get("vel_avance", 32))
        vel_rev = float(g.get("vel_reversa", 30))

        if self._2t_fase == "avance":
            meta = objetivo * float(g.get("frac_avance", 0.6))
            sin_sitio = p.pasillo_mm < float(g.get("min_pasillo_mm", 260.0))
            if girado >= meta or sin_sitio or t_fase > float(g.get("avance_max_ms", 2000)):
                self._2t_fase = "reversa"
                self._2t_t_fase = ahora
                motivo = "sin sitio delante" if sin_sitio else "fraccion hecha"
                return self._salida(0, lado * float(g.get("dir_avance", 100.0)),
                                    p, yaw, sentido,
                                    f"2T: paso a reversa ({motivo})")
            return self._salida(vel_av, lado * float(g.get("dir_avance", 100.0)),
                                p, yaw, sentido,
                                f"2T avance {girado:.0f}/{objetivo:.0f} deg")

        # ---- reversa: volante al lado contrario, el carro sigue rotando ----
        if t_fase > float(g.get("reversa_max_ms", 1100)):
            self._2t_ciclo += 1
            if self._2t_ciclo >= int(g.get("max_ciclos", 3)):
                self._terminar_giro(yaw, reanclar=True)
                return None
            self._2t_fase = "avance"
            self._2t_t_fase = ahora
            return self._salida(0, lado * float(g.get("dir_avance", 100.0)),
                                p, yaw, sentido,
                                f"2T: ciclo {self._2t_ciclo + 1}, vuelve a avanzar")
        return self._salida(-vel_rev, -lado * float(g.get("dir_reversa", 100.0)),
                            p, yaw, sentido,
                            f"2T reversa {girado:.0f}/{objetivo:.0f} deg")

    # ---------------- giro por color (hacia adentro, cede al pilar) ------
    def _iniciar_color(self, yaw: Optional[float], usar_yaw: bool,
                       reanudar: bool = False) -> None:
        """Arranca (o retoma) el giro hacia adentro. El rumbo de la recta
        nueva se avanza UNA sola vez por esquina, aunque el giro se suelte
        y se retome varias veces: si se avanzara en cada reanudacion el
        carro acabaria apuntando 180 o 270 grados."""
        self._cambiar(GIRO_COLOR)
        if not reanudar:
            self._color_t_inicio = time.time()
            self._color_apuntado = False
        if not self._color_apuntado and usar_yaw:
            ### solo cuenta como apuntado si habia yaw; si no, _paso_color
            ### lo hace en cuanto vuelva
            self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
            self._color_apuntado = True
        self._color_pendiente = False
        self._t_pilar_fuera = 0.0

    def _soltar_color(self, yaw: Optional[float], usar_yaw: bool) -> None:
        """Hay un pilar: el volante es del esquive desde YA. Se pasa a RECTO
        con el giro marcado como pendiente. El rumbo de referencia se deja
        apuntando a la recta NUEVA aunque se suelte desde el pre-giro: en
        recta el giroscopio sigue tirando hacia adentro (acotado por yaw_max
        y cediendo al pilar), que es hacia donde hay que ir. Probe a dejar
        el rumbo viejo y el carro se iba derecho al muro de enfrente."""
        if not self._color_apuntado:
            self._color_t_inicio = time.time()
            if usar_yaw:
                self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
                self._color_apuntado = True
        self._cambiar(RECTO)
        self.pd.reiniciar()
        self._color_pendiente = True
        self._t_pilar_fuera = 0.0

    def _paso_color(self, p: PerfilMuro, yaw: Optional[float], usar_yaw: bool,
                    sentido: int, ahora: float,
                    pilar_en_juego: bool) -> Optional[Decision]:
        """Un tick del giro por color. Devuelve None si termino o se solto
        (en ambos casos el estado ya es RECTO)."""
        g, cfg = self.gc, self.cfg
        lado = self.lado_giro or 1
        pasillo = p.pasillo_mm

        if pilar_en_juego and bool(g.get("ceder_al_pilar", True)):
            self._soltar_color(yaw, usar_yaw)
            return None

        if usar_yaw and not self._color_apuntado:
            # el yaw faltaba al arrancar: apuntar ahora (con re-anclaje)
            self._apuntar_a_la_recta_siguiente(yaw, usar_yaw)
            self._color_apuntado = True

        vencido = (ahora - self._color_t_inicio) * 1000 > float(g.get("max_ms", 3500))
        tope = min(float(g.get("dir_pct", 75.0)), float(self.lim.get("dir_max", 100)))
        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            if abs(err) < float(cfg.get("giro_tolerancia_deg", 8.0)) or vencido:
                if vencido:
                    self.rumbo_objetivo = yaw
                self._terminar_giro()
                return None
            ### misma ley que el giro normal (P sobre el error de rumbo),
            ### pero topada en dir_pct: asi va soltando volante al final
            d = _lim(err * float(cfg.get("yaw_kp", 1.6)) * 3.0, -tope, tope)
            txt = f"giro color yaw err={err:+.0f}"
        else:
            # Sin giroscopio: el pasillo abre cuando el giro encaro la recta.
            # Con anti-bucle se exige ademas un tiempo minimo (el hueco del
            # muro interno abre el pasillo antes de tiempo).
            abrio = pasillo > float(cfg.get("salir_giro_mm", 950.0))
            if bool(cfg.get("bloqueo_esquina", True)):
                abrio = abrio and (ahora - self._color_t_inicio) * 1000 > \
                    float(g.get("max_ms", 3500)) * 0.4
            if abrio or vencido:
                self._terminar_giro()
                return None
            d = lado * tope
            txt = f"giro color vision pasillo={pasillo:.0f}"

        # --- velocidad VARIABLE: con el pasillo ---------------------------
        # despejado = vel_max; el muro encima (parar_bajo) = vel_min
        parar_bajo = float(cfg.get("parar_bajo_mm", 300.0))
        frenar = float(cfg.get("frenar_bajo_mm", 1000.0))
        t = (pasillo - parar_bajo) / max(1.0, frenar - parar_bajo)
        v_min = float(g.get("vel_min_pct", 22))
        v_max = float(g.get("vel_max_pct", 40))
        vel = v_min + (v_max - v_min) * _lim(t, 0.0, 1.0)
        if bool(g.get("ceder_al_pilar", True)):
            txt += " [cede al pilar]"
        return self._salida(vel, d, p, yaw, sentido, txt)

    def _reanudar_color(self, p: PerfilMuro, yaw: Optional[float],
                        usar_yaw: bool, sentido: int, ahora: float,
                        en_esquina: bool,
                        pilar_en_juego: bool) -> Optional[Decision]:
        """En RECTO con un giro por color suelto. Decide si retomarlo, darlo
        por hecho (el esquive ya encaro la recta) o seguir esperando.
        Devuelve la Decision del giro retomado, o None para seguir en recto."""
        g, cfg = self.gc, self.cfg
        if not en_esquina:
            ### la zona caduco: lo que falte de rumbo lo pone el yaw en recta
            self._color_pendiente = False
            return None
        if pilar_en_juego:
            self._t_pilar_fuera = 0.0
            return None
        if self._t_pilar_fuera == 0.0:
            self._t_pilar_fuera = ahora
        if (ahora - self._t_pilar_fuera) * 1000 < float(g.get("reanudar_tras_ms", 400)):
            return None

        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            hecho = abs(err) < float(cfg.get("giro_tolerancia_deg", 8.0))
        else:
            hecho = p.pasillo_mm > float(cfg.get("salir_giro_mm", 950.0))
        if hecho:
            # esquivando ya quedo encarado: la esquina esta hecha
            self._color_pendiente = False
            self._terminar_giro()
            return None
        if not bool(g.get("reanudar", True)):
            return None
        self._iniciar_color(yaw, usar_yaw, reanudar=True)
        return self._paso_color(p, yaw, usar_yaw, sentido, ahora, pilar_en_juego)

    # ------------------------------------------------------------------
    def _apuntar_a_la_recta_siguiente(self, yaw: Optional[float],
                                      usar_yaw: bool) -> None:
        """Avanza el rumbo de referencia 90 grados: la recta de despues de la
        curva. Se calcula sobre la recta ANTERIOR, no sobre el yaw actual, asi
        que el error con el que se entre en la curva no se hereda.

        RE-ANCLAJE: si el yaw dice que el carro ya lleva ~90 grados girado en
        el sentido del giro respecto a esa recta, la recta acumulada se quedo
        una esquina atras (se perdio una linea y el centrado doblo solo). Se
        parte de la recta REAL; si no, el objetivo sale 90 grados desfasado y
        el carro dobla al lado contrario. Torcido en contra (esquivando un
        pilar hacia el muro exterior) no cambia nada: la base sigue siendo la
        recta acumulada, que ahi es la buena."""
        if not usar_yaw:
            return
        base = self.rumbo_recta if self.rumbo_recta is not None else yaw
        if base is None:
            return
        lado = self.lado_giro or 1
        grados = float(self.cfg.get("giro_grados", 90.0))
        if bool(self.cfg.get("reanclar_rumbo", True)) and yaw is not None:
            girado = lado * _norm_ang(yaw - base)
            if (float(self.cfg.get("reanclar_desde_deg", 65.0)) <= girado
                    <= float(self.cfg.get("reanclar_max_deg", 135.0))):
                base = _norm_ang(base + lado * grados)
                self.reanclajes += 1
                self._t_ultimo_reanclaje = time.time()
        self.rumbo_recta = _norm_ang(base + lado * grados)
        self.rumbo_objetivo = self.rumbo_recta

    def _terminar_giro(self, yaw: Optional[float] = None,
                       reanclar: bool = False):
        """reanclar: fijar el rumbo de la recta nueva al yaw actual. Se usa al
        salir del giro de dos tiempos (que mide angulo acumulado y no lleva
        rumbo objetivo). En el giro normal por yaw NO se reancla: ese rumbo
        objetivo ya es exactamente 'el de antes mas 90', y sustituirlo por el
        yaw real meteria el error de cada giro en la referencia siguiente."""
        lado = self.lado_giro
        rescate = self._giro_es_rescate
        self._giro_es_rescate = False
        self._apuntado = True
        self._cambiar(RECTO)
        self.pd.reiniciar()
        if reanclar:
            # El 2T mide su propio angulo acumulado; la referencia buena sigue
            # siendo la recta, no el yaw con el que quedo la maniobra.
            self.rumbo_objetivo = (self.rumbo_recta if self.rumbo_recta is not None
                                   else yaw)
        if rescate:
            return          # un rescate no es una esquina: no cuenta vuelta
        try:
            self.al_completar_giro(lado)
        except Exception:
            pass

    def _dir_centrado(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        err = p.der - p.izq          # + = mas espacio a la derecha
        d = self.pd.paso(err, float(self.cfg.get("kp", 95.0)),
                         float(self.cfg.get("kd", 22.0)), ahora)
        return d, f"centrado {err:+.2f}"

    def _dir_pared(self, p: PerfilMuro, sentido: int,
                   ahora: float) -> Tuple[float, str]:
        """Sigue el muro INTERNO (horario: derecha; antihorario: izquierda)
        a distancia fija. Si el sentido no se conoce aun, cae al centrado.

        Prefiere la pared que se ha IDENTIFICADO como lateral interna (ver
        muro.clasificar_recta): la media de la banda lateral mezcla lo que
        haya en ese lado de la imagen, y en una curva eso incluye la pared de
        enfrente, que no es la que hay que seguir.
        """
        if sentido == 0:
            return self._dir_centrado(p, ahora)
        lado_int = sentido               # +1 = interno a la derecha
        if bool(self.cfg.get("usar_rectas", True)) and p.interna_mm is not None:
            d_actual = p.interna_mm
            fuente = "recta"
        else:
            d_actual = (p.der if lado_int > 0 else p.izq) * p.alcance_mm
            fuente = "banda"
        objetivo = float(self.cfg.get("pared_objetivo_mm", 320.0))
        err = d_actual - objetivo        # >0 = estoy lejos del muro interno
        salida = self.pd_pared.paso(err, float(self.cfg.get("kp_pared", 0.22)),
                                    float(self.cfg.get("kd_pared", 0.05)), ahora)
        # interno a la derecha y lejos -> acercarse girando a la derecha
        return lado_int * salida, f"pared int({fuente}) d={d_actual:.0f} err={err:+.0f}"

    # ------------------------------------------------------------------
    def _salida(self, vel: float, direccion: float, p: PerfilMuro,
                yaw: Optional[float], sentido: int, motivo: str) -> Decision:
        dir_max = float(self.lim.get("dir_max", 100))
        d = Decision(
            vel=int(round(_lim(vel, -100, 100))),
            direccion=int(round(_lim(direccion, -dir_max, dir_max))),
            estado=self.estado,
            motivo=motivo,
            metricas=self._metricas(p, yaw, sentido),
        )
        self.ultimo = d
        return d

    def _metricas(self, p: PerfilMuro, yaw: Optional[float],
                  sentido: int) -> Dict[str, Any]:
        m = {
            "izq": round(p.izq, 2),
            "der": round(p.der, 2),
            "pasillo_mm": round(p.pasillo_mm, 0),
            "cob_izq": round(p.cobertura_izq, 2),
            "cob_der": round(p.cobertura_der, 2),
            "cierre_mms": round(self._vel_cierre, 0),
            "sentido": sentido,
            "interna_mm": None if p.interna_mm is None else round(p.interna_mm),
            "externa_mm": None if p.externa_mm is None else round(p.externa_mm),
            "frontal_mm": None if p.frontal_mm is None else round(p.frontal_mm),
            "desvio_recta": None if p.error_rumbo is None else round(p.error_rumbo, 1),
        }
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["rumbo_obj"] = round(self.rumbo_objetivo, 1)
            if self.rumbo_recta is not None:
                m["rumbo_recta"] = round(self.rumbo_recta, 1)
        if self.reanclajes:
            m["reanclajes"] = self.reanclajes
        return m
