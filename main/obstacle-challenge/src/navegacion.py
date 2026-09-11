"""
navegacion.py — Decidir velocidad y direccion a partir del perfil del muro,
el giroscopio y lo que pide el esquivador de pilares.

Maquina de estados:

    RECTO -> PRE_GIRO -> GIRO -> RECTO        y ESCAPE por encima de todo
      |________________________________|

  * RECTO      el giroscopio mantiene el rumbo de la recta y el centrado
               (espacio libre izquierda/derecha) corrige la posicion. Si hay
               un PILAR en juego, manda el servo visual del esquivador: el
               centrado cede segun el peso del pilar y del rumbo solo
               sobrevive una fraccion (obstaculos.yaw_al_esquivar). Si se
               esta ADELANTANDO un pilar que ya no se ve, se mantiene el
               rumbo CONGELADO que trae el esquivador y el centrado se apaga
               del todo: es lo que impide que la cola barra el pilar.
  * PRE_GIRO   la esquina ya se disparo pero: (1) se frena ANTES de doblar,
               (2) se espera retardo_giro_ms para que las ruedas TRASERAS
               pasen el canto del muro interno, y (3) si hay sitio, se abre
               hacia el lado contrario (giro abierto, como un camion en un
               cruce de 90) para no cortar la esquina con la cola.
  * GIRO       con giroscopio: rumbo objetivo +-90 y a clavarlo. Sin el:
               direccion fija hasta que el pasillo abre. Si aparece un pilar
               el giro se SUELTA (vuelve a RECTO con el rumbo ya apuntando a
               la recta nueva) y se REANUDA cuando el pilar deja de mandar;
               el rumbo objetivo se avanza UNA sola vez por esquina, no en
               cada reintento.
  * RESCATE    la esquina se cerro y el piso quedo en TRIANGULO: reversa
               corta y giro comprometido hacia adentro. Va por encima de todo,
               incluido el escape, porque ahi el escape no sirve (retrocede,
               abre un palmo y el centrado se vuelve a meter en el mismo
               hueco). Ver muro.DetectorAtrapado.
  * ESCAPE     el pasillo se cerro de verdad: marcha atras COMPROMETIDA
               (minimo + extra segun el deficit). Ir y venir cada 500 ms
               frente a un muro es exactamente como se choca; el compromiso
               es la cura. La direccion va HACIA el muro para que el morro
               se separe, como al salir de un estacionamiento.

Tras cualquier maniobra que deje el carro torcido (escape, rescate, giro
vencido) el rumbo de referencia se ANCLA a una recta valida (la de antes o
sus vecinas de +-90), nunca al rumbo en el que quedo mirando: adoptar ese
rumbo fue lo que hizo que el carro se fuera en sentido contrario tras varios
escapes.

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
from .obstaculos import Maniobra, LIBRE

RECTO = "recto"
PRE_GIRO = "pre_giro"
GIRO = "giro"
ESCAPE = "escape"
RESCATE = "rescate"


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
                 cfg_esc: Dict[str, Any], cfg_res: Dict[str, Any],
                 al_completar_giro: Optional[Callable[[int], None]] = None):
        self.cfg = cfg_nav
        self.lim = cfg_lim
        self.esc = cfg_esc
        self.res = cfg_res
        self.al_completar_giro = al_completar_giro or (lambda lado: None)

        self.pd = _PD()
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
        self._rumbo_rescate: Optional[float] = None
        self._lado_rescate = 0
        self._fin_reversa = 0.0
        self._motivo_rescate = ""
        self._giro_suelto = False     # el giro de esquina cedio ante un pilar

    # ------------------------------------------------------------------
    def reiniciar(self):
        self.pd.reiniciar()
        self.detector_interna = DetectorEsquinaInterna()
        self.estado = RECTO
        self.t_estado = time.time()
        self.rumbo_objetivo = None
        self._escape_intentos = 0
        self._pasillo_prev = None
        self._rumbo_rescate = None
        self._lado_rescate = 0
        self._giro_suelto = False

    def _cambiar(self, estado: str):
        if estado != self.estado:
            self.estado = estado
            self.t_estado = time.time()

    # ------------------------------------------------------------------
    def paso(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
             linea_reciente: bool = False,
             maniobra: Optional[Maniobra] = None,
             atrapado: bool = False, lado_pilar: int = 0) -> Decision:
        """sentido: +1 horario, -1 antihorario, 0 desconocido.
        maniobra: lo que pide el esquivador (direccion, peso, rumbo fijo...).
        atrapado: el detector del triangulo dice que la esquina se cerro.
        lado_pilar: +1/-1 si hay un pilar mandando ahora mismo, 0 si no."""
        ahora = time.time()
        cfg, lim, esc = self.cfg, self.lim, self.esc
        man = maniobra if maniobra is not None else Maniobra()
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

        # =================== RESCATE DE ESQUINA (lo primero) ==============
        # Va POR ENCIMA del escape: cuando el piso queda en triangulo, el
        # escape no sirve de nada (retrocede, abre un palmo, devuelve el mando
        # al centrado y el centrado se vuelve a meter en el mismo hueco).
        if (self.estado != RESCATE and atrapado and
                bool(self.res.get("activo", True))):
            self._iniciar_rescate(yaw, sentido, lado_pilar, p, usar_yaw)

        if self.estado == RESCATE:
            d = self._paso_rescate(p, yaw, sentido, ahora, usar_yaw)
            if d is not None:
                return d

        # =================== ESCAPE =======================================
        parar_bajo = float(cfg.get("parar_bajo_mm", 300.0))
        if self.estado != ESCAPE and pasillo < parar_bajo:
            self._cambiar(ESCAPE)
            self._giro_suelto = False
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
                    # A una recta VALIDA, no al rumbo en que quedo mirando.
                    self._anclar_recta_mas_cercana(yaw, usar_yaw)
                elif self._escape_intentos > int(esc.get("escape_max_intentos", 4)):
                    # La reversa no gana espacio (algo detras): giro adelante
                    # HACIA DONDE HAY MAS PISO. Aqui no vale mirar distancias:
                    # metido en un rincon el muro esta igual de cerca por los
                    # dos lados y la comparacion no dice nada. Los pixeles de
                    # piso si: la salida esta por donde se ve mas blanco.
                    self._escape_intentos = 0
                    self.lado_giro = 1 if p.piso_der > p.piso_izq else -1
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
                # el muro esta del lado donde se ve MENOS piso
                lado_muro = -1 if p.piso_izq < p.piso_der else 1
                return self._salida(
                    -float(lim.get("vel_reversa", 35)),
                    lado_muro * float(esc.get("escape_dir", 80.0)),
                    p, yaw, sentido,
                    f"escape #{self._escape_intentos} pasillo={pasillo:.0f}mm")

        # =================== PRE_GIRO =====================================
        if self.estado == PRE_GIRO:
            if (ahora - self.t_estado) * 1000 >= float(cfg.get("retardo_giro_ms", 220)):
                self._cambiar(GIRO)
                if usar_yaw:
                    base = self.rumbo_objetivo if self.rumbo_objetivo is not None else yaw
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

        # =================== GIRO =========================================
        if self.estado == GIRO:
            if lado_pilar != 0 and bool(cfg.get("giro_cede_ante_pilar", True)):
                # El giro de esquina CEDE si aparece un pilar. Un giro
                # comprometido de 90 grados con un pilar delante se lo lleva
                # por delante o lo pasa por el lado prohibido, y eso termina
                # la ronda. El rumbo objetivo ya apunta a la recta nueva, asi
                # que mientras esquiva el giroscopio sigue tirando hacia
                # adentro; el giro se reanuda cuando el pilar deje de mandar,
                # SIN volver a sumar 90.
                self._soltar_giro()
            else:
                d = self._paso_giro(p, yaw, sentido, ahora, usar_yaw)
                if d is not None:
                    return d

        # =================== RECTO ========================================
        desvio = None
        if usar_yaw and self.rumbo_objetivo is not None:
            desvio = abs(_norm_ang(self.rumbo_objetivo - yaw))

        # --- giro suelto: reanudarlo cuando el pilar ya no manda ----------
        # _giro_suelto se queda puesto hasta que el giro termine: asi el giro
        # reanudado sabe que NO tiene que contar la esquina.
        if self._giro_suelto and lado_pilar == 0 and man.estado == LIBRE:
            if desvio is not None and desvio < float(cfg.get("giro_tolerancia_deg", 8.0)):
                self._terminar_giro(contar=False)
            else:
                self._cambiar(GIRO)
                d = self._paso_giro(p, yaw, sentido, ahora, usar_yaw,
                                    etiqueta="giro reanudado")
                if d is not None:
                    return d

        recto_estable = (ahora - self.t_estado) * 1000 >= float(
            cfg.get("min_recto_ms", 700))

        # Con el carro torcido respecto a la recta (esquivando), un pasillo
        # que se cierra o un muro lateral que desaparece casi nunca son una
        # esquina: son el muro de la propia recta visto de lado. Girar 90 ahi
        # es meterse contra el. La linea del piso si sigue valiendo.
        desvio_ok = desvio is None or desvio <= float(
            cfg.get("esquina_max_desvio_deg", 25.0))

        disparo = ""
        if recto_estable and not self._giro_suelto:
            if pasillo < float(cfg.get("girar_bajo_mm", 650.0)) and desvio_ok:
                disparo = f"pasillo {pasillo:.0f}mm"
            elif aviso_interna is not None and desvio_ok:
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
            return self._salida(vel_giro, 0.0, p, yaw, sentido,
                                f"esquina: {disparo}")

        # --- centrado -------------------------------------------------------
        direccion, motivo = self._dir_centrado(p, ahora)
        if self._giro_suelto:
            motivo = "giro suelto por pilar; " + motivo

        yaw_kp = float(cfg.get("yaw_kp", 1.6))
        yaw_max = float(cfg.get("yaw_max", 45.0))

        if man.rumbo_fijo is not None and usar_yaw:
            # --- ADELANTANDO: rumbo congelado, sin centrado ----------------
            # El pilar ya no se ve (esta en el punto ciego de delante del
            # carro). Se mantiene EXACTAMENTE el rumbo con el que se perdio
            # hasta que la cola lo haya pasado. Ni el centrado ni la recta
            # tiran del carro hacia el pilar mientras tanto.
            err = _norm_ang(man.rumbo_fijo - yaw)
            direccion = _lim(err * yaw_kp, -yaw_max, yaw_max)
            motivo = f"adelantando {man.color}: rumbo fijo err={err:+.0f}"
        else:
            # --- esquive de pilares: el servo visual manda ------------------
            if man.peso > 0.0:
                direccion = man.direccion + (1.0 - man.peso) * direccion
                motivo += f" pilar({man.direccion:+.0f}x{man.peso:.2f})"

            # --- rumbo por giroscopio -----------------------------------
            if usar_yaw and self.rumbo_objetivo is not None:
                err = _norm_ang(self.rumbo_objetivo - yaw)
                corr = _lim(err * yaw_kp, -yaw_max, yaw_max)
                # Al esquivar el carro se sale del rumbo A PROPOSITO. La
                # correccion se desvanece con el peso del pilar hasta la
                # fraccion que pide el esquivador (ANTi conserva mas o menos
                # un tercio): bastante para no cruzarse, poco para no volver
                # hacia el pilar.
                if man.peso > 0.0:
                    corr *= 1.0 - man.peso * (1.0 - man.yaw_factor)
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

        # con un pilar en juego se va a la velocidad del esquive
        if man.vel_pct is not None:
            vel = min(vel, float(man.vel_pct))

        # girar fuerte y correr a la vez es como se sale de la pista
        vel *= 1.0 - 0.45 * min(1.0, abs(direccion) / max(1.0, dir_max))

        return self._salida(vel, direccion, p, yaw, sentido, motivo)

    # ------------------------------------------------------------------
    def _paso_giro(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
                   ahora: float, usar_yaw: bool,
                   etiqueta: str = "giro") -> Optional[Decision]:
        """La salida del GIRO mientras dure, o None cuando termina (y el
        resto del paso() sigue en RECTO)."""
        cfg, lim = self.cfg, self.lim
        vel_giro = float(lim.get("vel_giro", 38))
        dir_max = float(lim.get("dir_max", 100))
        venc = (ahora - self.t_estado) * 1000 > float(cfg.get("giro_max_ms", 3000))
        if usar_yaw and self.rumbo_objetivo is not None:
            err = _norm_ang(self.rumbo_objetivo - yaw)
            if abs(err) < float(cfg.get("giro_tolerancia_deg", 8.0)) or venc:
                if venc:
                    self._anclar_recta_mas_cercana(yaw, usar_yaw)
                self._terminar_giro(contar=not self._giro_suelto)
                return None
            d = _lim(err * float(cfg.get("yaw_kp", 1.6)) * 3.0, -dir_max, dir_max)
            return self._salida(vel_giro, d, p, yaw, sentido,
                                f"{etiqueta} yaw err={err:+.0f}")
        if p.pasillo_mm > float(cfg.get("salir_giro_mm", 950.0)) or venc:
            self._terminar_giro(contar=not self._giro_suelto)
            return None
        d = self.lado_giro * float(cfg.get("dir_giro", 85.0))
        return self._salida(vel_giro, d, p, yaw, sentido,
                            f"{etiqueta} vision pasillo={p.pasillo_mm:.0f}")

    def _soltar_giro(self) -> None:
        """El giro cede ante un pilar: a RECTO sin contar la esquina y sin
        tocar el rumbo objetivo (ya apunta a la recta nueva)."""
        self._giro_suelto = True
        self._cambiar(RECTO)
        self.pd.reiniciar()

    # ------------------------------------------------------------------
    def _iniciar_rescate(self, yaw: Optional[float], sentido: int,
                         lado_pilar: int, p: PerfilMuro,
                         usar_yaw: bool) -> None:
        """Elegir hacia donde se sale y comprometerse.

        Orden de preferencia del lado, y el orden importa:
          1. EL PILAR. Si hay uno en juego manda el, porque pasarlo por el
             lado incorrecto termina la ronda y quedarse atascado solo cuesta
             tiempo. Es lo que pidio el usuario: "si se ve el siguiente
             obstaculo debe girar de sentido correcto".
          2. EL SENTIDO de la ronda: hacia adentro (horario = derecha).
          3. EL PISO. Sin sentido conocido, hacia donde se ve mas blanco. Esta
             es la medida que NO empata en un rincon, al reves que comparar
             distancias, que ahi estan las dos igual de cortas.
        """
        if lado_pilar != 0:
            self._lado_rescate = lado_pilar
            motivo = "pilar"
        elif sentido != 0:
            self._lado_rescate = sentido
            motivo = "sentido"
        else:
            self._lado_rescate = 1 if p.piso_der > p.piso_izq else -1
            motivo = "piso"
        self._cambiar(RESCATE)
        self._giro_suelto = False
        self._fin_reversa = time.time() + \
            float(self.res.get("reversa_ms", 400)) / 1000.0
        if usar_yaw and yaw is not None:
            self._rumbo_rescate = _norm_ang(
                yaw + self._lado_rescate * float(self.res.get("grados", 100.0)))
        else:
            self._rumbo_rescate = None
        self._motivo_rescate = motivo

    def _paso_rescate(self, p: PerfilMuro, yaw: Optional[float], sentido: int,
                      ahora: float, usar_yaw: bool) -> Optional[Decision]:
        """Devuelve la decision mientras dure el rescate, o None cuando
        termina (y entonces el resto del paso() sigue como siempre)."""
        res = self.res
        venc = (ahora - self.t_estado) * 1000 > float(res.get("max_ms", 3000))
        vel = float(res.get("vel_pct", 35))

        # Reversa corta con el volante al reves: separa el morro de la pared
        # para que quepa el giro. Sin esto, con el morro pegado, girar no
        # mueve el carro: lo raspa.
        if ahora < self._fin_reversa and not venc:
            return self._salida(-vel, -self._lado_rescate * 80.0, p, yaw,
                                sentido, f"rescate: reversa ({self._motivo_rescate})")

        if usar_yaw and self._rumbo_rescate is not None and yaw is not None:
            err = _norm_ang(self._rumbo_rescate - yaw)
            if abs(err) < float(self.cfg.get("giro_tolerancia_deg", 8.0)) or venc:
                self._terminar_rescate(yaw, usar_yaw)
                return None
            d = _lim(err * float(self.cfg.get("yaw_kp", 1.6)) * 3.0,
                     -float(self.lim.get("dir_max", 100)),
                     float(self.lim.get("dir_max", 100)))
            return self._salida(vel, d, p, yaw, sentido,
                                f"rescate {err:+.0f} ({self._motivo_rescate})")

        # sin giroscopio: girar hasta que se abra el pasillo, o hasta el tope
        if venc or p.pasillo_mm > float(self.cfg.get("salir_giro_mm", 950.0)):
            self._terminar_rescate(yaw, usar_yaw)
            return None
        return self._salida(vel, self._lado_rescate * float(res.get("dir_pct", 90.0)),
                            p, yaw, sentido,
                            f"rescate vision ({self._motivo_rescate})")

    def _terminar_rescate(self, yaw: Optional[float], usar_yaw: bool) -> None:
        self._cambiar(RECTO)
        self.pd.reiniciar()
        self._rumbo_rescate = None
        self._anclar_recta_mas_cercana(yaw, usar_yaw)
        # El rescate NO cuenta esquina: el carro atrapado casi siempre ya cruzo
        # esa linea, y cada choque sumando vuelta fue un fallo real de pista.

    def _anclar_recta_mas_cercana(self, yaw: Optional[float],
                                  usar_yaw: bool) -> None:
        """Tras maniobrar, volver a una recta VALIDA, no al rumbo en el que se
        quedo mirando. Adoptar el rumbo de salida fue exactamente lo que hizo
        que el carro se fuera en sentido contrario tras varios escapes: se
        escoge entre la recta de antes y sus vecinas de +-90, la que quede mas
        cerca del yaw real, y nada mas."""
        if not usar_yaw or yaw is None or self.rumbo_objetivo is None:
            if usar_yaw and yaw is not None:
                self.rumbo_objetivo = yaw
            return
        candidatas = [self.rumbo_objetivo,
                      _norm_ang(self.rumbo_objetivo + 90.0),
                      _norm_ang(self.rumbo_objetivo - 90.0)]
        self.rumbo_objetivo = min(candidatas,
                                  key=lambda c: abs(_norm_ang(c - yaw)))

    def _terminar_giro(self, contar: bool = True):
        """contar=False: el giro se habia soltado por un pilar y se reanuda
        mas tarde; para entonces las lineas ya contaron esa esquina y volver
        a sumarla seria contar una de mentira. Mejor perder una que sumar."""
        lado = self.lado_giro
        self._cambiar(RECTO)
        self.pd.reiniciar()
        self._giro_suelto = False
        if not contar:
            return
        try:
            self.al_completar_giro(lado)
        except Exception:
            pass

    def _dir_centrado(self, p: PerfilMuro, ahora: float) -> Tuple[float, str]:
        err = p.der - p.izq          # + = mas espacio a la derecha
        d = self.pd.paso(err, float(self.cfg.get("kp", 95.0)),
                         float(self.cfg.get("kd", 22.0)), ahora)
        return d, f"centrado {err:+.2f}"

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
            "piso_izq": round(p.piso_izq, 2),
            "piso_der": round(p.piso_der, 2),
            "pasillo_mm": round(p.pasillo_mm, 0),
            "cob_izq": round(p.cobertura_izq, 2),
            "cob_der": round(p.cobertura_der, 2),
            "cierre_mms": round(self._vel_cierre, 0),
            "sentido": sentido,
            "giro_suelto": self._giro_suelto,
        }
        if yaw is not None:
            m["yaw"] = round(yaw, 1)
            if self.rumbo_objetivo is not None:
                m["rumbo_obj"] = round(self.rumbo_objetivo, 1)
        return m
