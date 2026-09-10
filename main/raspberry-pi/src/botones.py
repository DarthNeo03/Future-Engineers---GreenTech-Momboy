"""
botones.py — El pulsador de competencia, visto desde la Pi.

El reglamento WRO Future Engineers exige que la ronda EMPIECE con una sola
accion sobre el robot ya colocado en la pista: nada de teclado, ni pantalla,
ni web, ni cable. El boton ARMAR de la interfaz no sirve para eso.

UN SOLO PULSADOR hace las dos cosas, como el start/stop de un cronometro:

    pulsacion con el carro parado    ->  ARMA y arranca la ronda
    pulsacion con el carro armado    ->  DESARMA y para (emergencia)
    pulsacion LARGA (opcional)       ->  apaga la Pi, si botones.apagar_con_larga

CUELGA DEL ESP32, NO DEL GPIO DE LA PI. El firmware
(code/esp32_carro/botones.h) lo lee a 100 Hz con antirrebote y manda su NIVEL
en la trama de sensores, 40 veces por segundo. Este modulo solo decide que
significa, porque es el lado que sabe si el carro estaba armado.

Y el ESP32 ademas CORTA SOLO: si el carro esta armado, una pulsacion no puede
querer decir otra cosa que parar, asi que enclava el corte y para el motor en
el tick siguiente, sin esperar a la Pi. Lo que se hace aqui es lo demas:
desarmar, parar la carrera y dejarlo escrito en el log.

NIVEL Y NO CONTADOR (al reves que los cruces de linea, que si son contadores)
Un dedo aguanta el pulsador 100-300 ms = 4-12 tramas: el nivel llega de
sobra. Y un contador guardaria pulsaciones pendientes, asi que un corte del
serial con una pulsacion sin entregar podria arrancar el carro AL RECONECTAR,
solo. Con nivel, lo que se pierde no se ejecuta.

DOS REDES DE SEGURIDAD, las dos por la misma razon (que el carro no salga
corriendo cuando nadie lo ha pedido):
  1. Un pulsador que ya esta pisado cuando este modulo empieza a mirarlo
     queda MUDO hasta que se le ve suelto una vez. Cubre el dedo apoyado, el
     cable al reves y el boton pegado.
  2. Si el enlace con el ESP32 se cae, se OLVIDA el estado del pulsador. Sin
     esto, un nivel viejo congelado en "pisado" seria una pulsacion larga
     fantasma, y al reaparecer el enlace, una corta fantasma.

Toda la logica fina (antirrebote y pulsacion larga) vive en Pulsador, que no
toca ni el enlace ni el hardware: se le dan niveles y tiempos y devuelve el
evento. Por eso el selftest lo prueba entero sin carro y sin ESP32.
"""

from __future__ import annotations

import subprocess
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

# Solo hay un boton; el nombre es el de la orden de la web (?boton=corta).
BOTON = "boton"

CORTA = "corta"
LARGA = "larga"
TIPOS = (CORTA, LARGA)


# ===========================================================================
# La logica: antirrebote y pulsacion larga, sin enlace y sin hardware
# ===========================================================================
class Pulsador:
    """El pulsador. Se le da el nivel leido (True = pisado) y el reloj, y
    devuelve "" | CORTA | LARGA.

    La pulsacion CORTA se avisa al SOLTAR (hasta entonces no se sabe si iba a
    ser larga) y la LARGA en cuanto se cumple el tiempo con el boton todavia
    pisado, para que el operador note el efecto sin tener que adivinar cuando
    soltar.

    El antirrebote de aqui se suma al del ESP32 (30 ms). No sobra: filtra la
    trama suelta rara, y sobre todo permite subirlo desde la web si el
    pulsador viene ruidoso, sin volver a compilar el firmware.
    """

    def __init__(self, antirrebote_ms: float = 50.0,
                 largo_ms: float = 3000.0):
        self.antirrebote_ms = float(antirrebote_ms)
        self.largo_ms = float(largo_ms)
        self.pulsado = False
        self._crudo = False
        self._t_crudo = 0.0
        self._t_cambio = 0.0
        self._largo_lanzado = False
        # Hasta ver el boton SUELTO una vez no se admite ninguna pulsacion.
        self._listo = False

    def reiniciar(self) -> None:
        self.pulsado = False
        self._crudo = False
        self._t_crudo = 0.0
        self._t_cambio = 0.0
        self._largo_lanzado = False
        self._listo = False

    def paso(self, crudo: bool, ahora: Optional[float] = None) -> str:
        ahora = time.monotonic() if ahora is None else float(ahora)
        crudo = bool(crudo)
        evento = ""

        if crudo != self._crudo:          # el nivel se movio: empieza a contar
            self._crudo = crudo
            self._t_crudo = ahora
        estable = (ahora - self._t_crudo) * 1000.0 >= self.antirrebote_ms

        if estable and crudo != self.pulsado:
            self.pulsado = crudo
            self._t_cambio = ahora
            if crudo:
                self._largo_lanzado = False
            elif not self._largo_lanzado:
                evento = CORTA

        if (self.pulsado and not self._largo_lanzado and self.largo_ms > 0
                and (ahora - self._t_cambio) * 1000.0 >= self.largo_ms):
            self._largo_lanzado = True
            evento = LARGA

        if not self._listo:
            # Se mira el nivel CRUDO ya estable, no el pulsado: al empezar,
            # 'pulsado' todavia vale False aunque el boton este pisado, y
            # darlo por bueno ahi seria dejar pasar justo la pulsacion que
            # esto tiene que tragarse.
            if estable and not crudo:
                self._listo = True
            return ""
        return evento


# ===========================================================================
# El gestor: sondea el enlace, decide y llama a las acciones
# ===========================================================================
class GestorBotones:
    """Lee el nivel del pulsador del ESP32 y llama a las acciones.

    enlace:   objeto con boton() -> (pisado, corte_esp32, frescos). Puede ser
              None para probar solo con pulsaciones virtuales.
    acciones: {"corta": fn, "larga": fn}

    El hilo corre SIEMPRE, tambien con botones.activo apagado y tambien en
    --simulado: las pulsaciones VIRTUALES de la web (/api/cmd?boton=corta)
    entran por el mismo camino que las de verdad, asi que la secuencia de
    competencia se ensaya entera en el PC.
    """

    def __init__(self, cfg: Dict[str, Any], enlace: Any = None,
                 acciones: Optional[Dict[str, Callable[[], None]]] = None,
                 al_log: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self.enlace = enlace
        self.acciones = dict(acciones or {})
        self._log = al_log or (lambda t: None)

        self.pulsador = Pulsador(float(cfg.get("antirrebote_ms", 50)),
                                 float(cfg.get("largo_ms", 3000)))
        self.ultimo = ""                 # "corta" | "larga", para la web
        self.t_ultimo = 0.0
        self.frescos = False             # hay tramas de sensores al dia
        self.corte_esp32 = False         # el ESP32 tiene cortado por el boton

        self._hilo: Optional[threading.Thread] = None
        self._parar = threading.Event()
        self._virtual = 0.0              # hasta cuando dura la pulsacion web
        self._t_accion = 0.0
        self._aviso_corte = False

    # -- ciclo de vida ----------------------------------------------------
    def iniciar(self) -> None:
        if self._hilo is not None:
            return
        if bool(self.cfg.get("activo", True)):
            self._log("[botones] pulsador del ESP32: arma y desarma")
        else:
            self._log("[botones] pulsador del ESP32 IGNORADO "
                      "(botones.activo): solo quedan las pulsaciones de la web")
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, daemon=True,
                                      name="botones")
        self._hilo.start()

    def cerrar(self) -> None:
        self._parar.set()
        if self._hilo:
            self._hilo.join(timeout=1.0)
            self._hilo = None

    # -- pulsacion desde la web (sin dedo y sin cable) ---------------------
    def pulsar_virtual(self, tipo: str = CORTA) -> None:
        if tipo not in TIPOS:
            raise ValueError(f"pulsacion '{tipo}': solo hay {list(TIPOS)}")
        anti = float(self.cfg.get("antirrebote_ms", 50)) / 1000.0
        if tipo == LARGA:
            dura = float(self.cfg.get("largo_ms", 3000)) / 1000.0 + anti + 0.15
        else:
            dura = anti * 2.0 + 0.10
        self._virtual = time.monotonic() + dura

    # -- bucle -------------------------------------------------------------
    def _leer_enlace(self) -> Tuple[bool, bool, bool]:
        if self.enlace is None or not bool(self.cfg.get("activo", True)):
            return (False, False, False)
        try:
            return self.enlace.boton()
        except Exception as e:
            self._log(f"[botones] no se puede leer el enlace ({e})")
            return (False, False, False)

    def _bucle(self) -> None:
        while not self._parar.is_set():
            hz = max(20, min(200, int(self.cfg.get("hz", 50))))
            ahora = time.monotonic()
            pisado, corte, frescos = self._leer_enlace()

            if frescos != self.frescos:
                self.frescos = frescos
                # Se olvida el estado en los DOS sentidos. Al caerse el
                # enlace, para no leer un nivel viejo congelado. Y al VOLVER,
                # para no tomar por pulsacion nueva un boton que ya estaba
                # pisado cuando el enlace regreso: tiene que verse suelto otra
                # vez antes de que se le haga caso.
                self.pulsador.reiniciar()
            if not frescos:
                pisado = False

            if corte != self.corte_esp32:
                self.corte_esp32 = corte
                if corte and not self._aviso_corte:
                    self._aviso_corte = True
                    self._log("[botones] el ESP32 corto la traccion: se pulso "
                              "con el carro armado")
                elif not corte:
                    self._aviso_corte = False

            # se releen en caliente: la web puede afinar el antirrebote y los
            # tiempos con el carro encendido, sin reiniciar
            self.pulsador.antirrebote_ms = float(self.cfg.get("antirrebote_ms", 50))
            self.pulsador.largo_ms = float(self.cfg.get("largo_ms", 3000))
            nivel = pisado or (ahora < self._virtual)
            evento = self.pulsador.paso(nivel, ahora)
            if evento:
                self._disparar(evento, ahora)
            time.sleep(1.0 / hz)

    def _disparar(self, tipo: str, ahora: float) -> None:
        muerto = float(self.cfg.get("repeticion_ms", 800)) / 1000.0
        if ahora - self._t_accion < muerto:
            # Un doble toque involuntario justo despues del start pararia la
            # ronda recien empezada: para eso existe este tiempo muerto.
            self._log(f"[botones] pulsacion {tipo} ignorada (tiempo muerto)")
            return
        self._t_accion = ahora
        self.ultimo = tipo
        self.t_ultimo = time.time()
        self._log(f"[boton] pulsacion {tipo}")
        fn = self.acciones.get(tipo)
        if fn is None:
            return
        try:
            fn()
        except Exception as e:
            self._log(f"[botones] la accion {tipo} fallo: "
                      f"{type(e).__name__}: {e}")

    # -- lectura para la web ----------------------------------------------
    def estado(self) -> Dict[str, Any]:
        return {
            "activo": bool(self.cfg.get("activo", True)),
            "fuente": "esp32",
            "frescos": self.frescos,
            "corte_esp32": self.corte_esp32,
            "pulsado": self.pulsador.pulsado,
            "ultimo": self.ultimo,
            "hace_s": (round(time.time() - self.t_ultimo, 1)
                       if self.t_ultimo else None),
        }


# ===========================================================================
# Apagado de la Pi (pulsacion larga, si esta permitido)
# ===========================================================================
def apagar_pi(al_log: Optional[Callable[[str], None]] = None) -> bool:
    """Apaga la Pi ordenadamente. Quitarle la corriente con la tarjeta SD
    montada es como se corrompen las tarjetas la noche antes de competir."""
    log = al_log or (lambda t: None)
    ordenes = (["systemctl", "poweroff"],
               ["sudo", "-n", "systemctl", "poweroff"],
               ["sudo", "-n", "shutdown", "-h", "now"])
    for orden in ordenes:
        try:
            r = subprocess.run(orden, timeout=5.0, capture_output=True)
        except Exception:
            continue
        if r.returncode == 0:
            log(f"[botones] apagando la Pi ({' '.join(orden)})")
            return True
    log("[botones] no se pudo apagar la Pi: falta permiso (sudo sin "
        "contrasena para 'systemctl poweroff')")
    return False
