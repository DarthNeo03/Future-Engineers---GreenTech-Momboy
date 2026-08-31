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
               es la cura. La direccion va HACIA el muro para que el morro
               se separe, como al salir de un estacionamiento.

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
                 al_completar_giro: Optional[Callable[[int], None]] = None):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.esc = cfg_esc
        self.g2t = cfg_2t if cfg_2t is not None else {}
        self.al_completar_giro = al_completar_giro or (lambda lado: None)

        self.pd = _PD()
        self.pd_pared = _PD()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.lado_giro = 0
        self.rumbo_objetivo: Optional[float] = None
        self.ultimo = Decision()

        self._t_fin_escape = 0.0
        self._escape_intentos = 0
        self._pasillo_prev: Optional[float] = None
        self._t_pasillo = 0.0
        self._vel_cierre = 0.0        # mm/s, positivo = el muro se acerca
        # Una esquina, UN giro: sin esto el carro giraria 90 grados otra vez
        # mientras siga dentro de la zona de la curva.
        self._esquina_atendida = False

        # --- giro de dos tiempos ---
        self._2t_fase = "avance"      # "avance" | "reversa"
        self._2t_ciclo = 0
        self._2t_t_fase = 0.0
        self._2t_t_inicio = 0.0
        self._2t_acum = 0.0           # grados girados ACUMULADOS (con signo)
        self._2t_yaw_prev: Optional[float] = None

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

    def _cambiar(self, estado: str):
        if estado != self.estado:
            self.estado = estado
            self.t_estado = time.time()

    # ------------------------------------------------------------------
    def paso(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
             linea_reciente: bool = False,
             bias_obstaculo: Tuple[float, float] = (0.0, 0.0),
             en_esquina: bool = False) -> Decision:
        """sentido: +1 horario, -1 antihorario, 0 desconocido.
        bias_obstaculo: (direccion_deseada_pct, peso 0..1) del esquive.
        en_esquina: el carro esta DENTRO de la curva segun las lineas del
                    piso. Con bloqueo_esquina, mientras dure eso el giro no
                    se abandona por lo que vea la camara (anti-bucle)."""
        ahora = time.time()
        cfg, lim, esc = self.cfg, self.lim, self.esc
        vel_crucero = float(lim.get("vel_crucero", 55))
        vel_giro = float(lim.get("vel_giro", 38))
        dir_max = float(lim.get("dir_max", 100))
        pasillo = p.pasillo_mm

        usar_yaw = bool(cfg.get("usar_yaw", True)) and yaw is not None
        if usar_yaw and self.rumbo_objetivo is None:
            self.rumbo_objetivo = yaw

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
                    if usar_yaw:
                        self.rumbo_objetivo = yaw   # el rumbo viejo ya no vale
                elif self._escape_intentos > int(esc.get("escape_max_intentos", 4)):
                    # la reversa no gana espacio (algo detras): giro adelante
                    self._escape_intentos = 0
                    self.lado_giro = 1 if p.der > p.izq else -1
                    self._cambiar(GIRO)
                    if usar_yaw:
                        self.rumbo_objetivo = _norm_ang(
                            yaw + self.lado_giro * float(cfg.get("giro_grados", 90.0)))
                else:
                    deficit = max(0.0, parar_bajo - pasillo)
                    comp = float(esc.get("escape_min_ms", 750)) + \
                        float(esc.get("escape_k_ms_por_mm", 3.0)) * deficit
                    self._t_fin_escape = ahora + comp / 1000.0
                    self._escape_intentos += 1
            if self.estado == ESCAPE:
                lado_muro = -1 if p.izq < p.der else 1   # donde esta el muro
                return self._salida(
                    -float(lim.get("vel_reversa", 35)),
                    lado_muro * float(esc.get("escape_dir", 80.0)),
                    p, yaw, sentido,
                    f"escape #{self._escape_intentos} pasillo={pasillo:.0f}mm")

        # =================== PRE_GIRO =====================================
        if self.estado == PRE_GIRO:
            if (ahora - self.t_estado) * 1000 >= float(cfg.get("retardo_giro_ms", 220)):
                if bool(self.g2t.get("activo", False)) and usar_yaw:
                    self._iniciar_2t(yaw)
                else:
                    self._cambiar(GIRO)
                    if usar_yaw:
                        base = (self.rumbo_objetivo if self.rumbo_objetivo is not None
                                else yaw)
                        self.rumbo_objetivo = _norm_ang(
                            base + self.lado_giro * float(cfg.get("giro_grados", 90.0)))
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

        # =================== GIRO_2T (dos tiempos) ========================
        if self.estado == GIRO_2T:
            d = self._paso_2t(p, yaw, sentido, ahora)
            if d is not None:
                return d
            # _paso_2t devuelve None cuando la maniobra termino: cae a RECTO

        # =================== GIRO =========================================
        if self.estado == GIRO:
            venc = (ahora - self.t_estado) * 1000 > float(cfg.get("giro_max_ms", 3000))
            bloqueado = en_esquina and bool(cfg.get("bloqueo_esquina", True))
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
        recto_estable = (ahora - self.t_estado) * 1000 >= float(
            cfg.get("min_recto_ms", 700))

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
        elif recto_estable:
            if pasillo < float(cfg.get("girar_bajo_mm", 650.0)):
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
        if peso > 0.0:
            direccion = (1.0 - peso) * direccion + peso * bias_dir
            motivo += f" esq({bias_dir:+.0f}x{peso:.2f})"

        # --- rumbo por giroscopio -----------------------------------------
        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            corr = _lim(err * float(cfg.get("yaw_kp", 1.6)),
                        -float(cfg.get("yaw_max", 45.0)),
                        float(cfg.get("yaw_max", 45.0)))
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
                 ahora: float) -> Optional[Decision]:
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

    # ------------------------------------------------------------------
    def _terminar_giro(self, yaw: Optional[float] = None,
                       reanclar: bool = False):
        """reanclar: fijar el rumbo de la recta nueva al yaw actual. Se usa al
        salir del giro de dos tiempos (que mide angulo acumulado y no lleva
        rumbo objetivo). En el giro normal por yaw NO se reancla: ese rumbo
        objetivo ya es exactamente 'el de antes mas 90', y sustituirlo por el
        yaw real meteria el error de cada giro en la referencia siguiente."""
        lado = self.lado_giro
        self._cambiar(RECTO)
        self.pd.reiniciar()
        if reanclar:
            self.rumbo_objetivo = yaw
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
        a distancia fija. Si el sentido no se conoce aun, cae al centrado."""
        if sentido == 0:
            return self._dir_centrado(p, ahora)
        lado_int = sentido               # +1 = interno a la derecha
        d_actual = (p.der if lado_int > 0 else p.izq) * p.alcance_mm
        objetivo = float(self.cfg.get("pared_objetivo_mm", 320.0))
        err = d_actual - objetivo        # >0 = estoy lejos del muro interno
        salida = self.pd_pared.paso(err, float(self.cfg.get("kp_pared", 0.22)),
                                    float(self.cfg.get("kd_pared", 0.05)), ahora)
        # interno a la derecha y lejos -> acercarse girando a la derecha
        return lado_int * salida, f"pared int d={d_actual:.0f} err={err:+.0f}"

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
        }
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["rumbo_obj"] = round(self.rumbo_objetivo, 1)
        return m
