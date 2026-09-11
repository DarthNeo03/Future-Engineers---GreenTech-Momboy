"""
enlace.py — El cable entre la Raspberry Pi y el ESP32.

DECISION CENTRAL: EL MANDO SE ENVIA SOLO, A RITMO FIJO.

La camara entrega 30 fps, pero no relojeados: un frame puede tardar 25 ms y el
siguiente 60 porque el sistema decidio hacer otra cosa. Si el mando se enviara
"cuando termine de pensar", cada pausa larga de vision seria un silencio en el
serial, y el ESP32 no distingue un silencio de una Pi muerta: cortaria a los
300 ms. El carro se pondria a dar tirones sin que nada este roto.

Por eso hay DOS hilos:

    hilo TX  (50 Hz, fijo)   reenvia SIEMPRE el ultimo mando decidido
    hilo RX  (bloqueante)    lee telemetria y sensores en cuanto llegan

y el lazo de vision solo deja el mando nuevo en una variable. Asi el enlace
late a 50 Hz pase lo que pase arriba, el failsafe mide lo que debe medir
(la Pi viva o muerta) y la latencia de una decision nunca pasa de 20 ms.

El hilo RX es el unico que toca el Lector; el resto lee copias bajo lock.
"""

from __future__ import annotations

import glob
import platform
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import serial  # pyserial

from . import protocolo as proto

BAUDIOS = 115200
PERIODO_TX = 0.020        # 50 Hz
TIMEOUT_RX = 0.050


def puertos_probables() -> List[str]:
    """Candidatos ordenados por lo probable que son EN COMPETENCIA.

    En el carro el ESP32 va por GPIO (ttyAMA0/serial0). El USB queda para el
    banco de pruebas, asi que se prueba despues: si estan los dos conectados,
    gana el que de verdad se usa en pista.
    """
    if platform.system().lower().startswith("win"):
        return [f"COM{i}" for i in range(3, 13)]
    fijos = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyAMA10"]
    usb = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return fijos + usb


@dataclass
class EstadoEnlace:
    """Foto del otro lado del cable. Se copia bajo lock antes de usarse."""
    conectado: bool = False
    puerto: str = ""
    tele: proto.Telemetria = field(default_factory=proto.Telemetria)
    sens: proto.Sensores = field(default_factory=proto.Sensores)
    t_tele: float = 0.0
    t_sens: float = 0.0
    tramas_malas: int = 0
    logs: List[str] = field(default_factory=list)

    @property
    def sensores_frescos(self) -> bool:
        return (time.time() - self.t_sens) < 0.5


class Enlace:
    def __init__(self, puerto: Optional[str] = None, baudios: int = BAUDIOS,
                 verbose: bool = True) -> None:
        self.verbose = verbose
        self._baudios = baudios
        self._puerto_pedido = puerto
        self._ser: Optional[serial.Serial] = None
        self._lector = proto.Lector()
        self._lock = threading.Lock()
        self._estado = EstadoEnlace()
        self._parar = threading.Event()
        self._hilos: List[threading.Thread] = []

        # Ultimo mando decidido por el piloto. Arranca en "parado y armado a
        # falso": si el piloto muere antes de decidir nada, esto es lo que el
        # hilo TX repite, y el carro no se mueve.
        self._seq = 0
        self._mando = dict(vel=0, direccion=0, armado=False, parada=True,
                           centrar=True, limpiar=False, vmax=255, aux=0)

    # ------------------------------------------------------------ apertura
    def abrir(self) -> bool:
        candidatos = [self._puerto_pedido] if self._puerto_pedido else puertos_probables()
        for p in candidatos:
            if not p:
                continue
            try:
                ser = serial.Serial(p, self._baudios, timeout=TIMEOUT_RX,
                                    write_timeout=0.2)
            except (OSError, serial.SerialException):
                continue
            # Un ESP32 que acaba de recibir DTR se reinicia: hay que darle su
            # medio segundo antes de creerse el primer silencio.
            time.sleep(0.3)
            ser.reset_input_buffer()
            self._ser = ser
            with self._lock:
                self._estado.conectado = True
                self._estado.puerto = p
            self._arrancar_hilos()
            if self._confirmar(timeout=1.5):
                if self.verbose:
                    print(f"[enlace] ESP32 en {p}")
                return True
            # Contesto el puerto pero no era el ESP32 (o esta mudo): siguiente.
            self.cerrar()
        if self.verbose:
            print("[enlace] no se encontro el ESP32")
        return False

    def _confirmar(self, timeout: float = 1.5) -> bool:
        """Ping/pong. Sin esto, un adaptador USB vacio pasa por ESP32 y el
        fallo aparece treinta segundos despues, en pista."""
        fin = time.time() + timeout
        while time.time() < fin:
            self.enviar_crudo(proto.trama_ping(0xA5))
            time.sleep(0.1)
            with self._lock:
                if self._estado.t_tele > 0 or self._estado.t_sens > 0:
                    return True
        return False

    def _arrancar_hilos(self) -> None:
        self._parar.clear()
        self._hilos = [
            threading.Thread(target=self._bucle_rx, name="enlace-rx", daemon=True),
            threading.Thread(target=self._bucle_tx, name="enlace-tx", daemon=True),
        ]
        for h in self._hilos:
            h.start()

    # ------------------------------------------------------------- lectura
    def _bucle_rx(self) -> None:
        while not self._parar.is_set():
            ser = self._ser
            if ser is None:
                time.sleep(0.05)
                continue
            try:
                datos = ser.read(max(1, ser.in_waiting))
            except (OSError, serial.SerialException):
                with self._lock:
                    self._estado.conectado = False
                return
            if not datos:
                continue
            ahora = time.time()
            for tipo, cuerpo in self._lector.alimentar(datos):
                with self._lock:
                    if tipo == proto.TIPO_TELE:
                        t = proto.decodificar_telemetria(cuerpo)
                        if t:
                            self._estado.tele = t
                            self._estado.t_tele = ahora
                    elif tipo == proto.TIPO_SENS:
                        s = proto.decodificar_sensores(cuerpo)
                        if s:
                            self._estado.sens = s
                            self._estado.t_sens = ahora
                    elif tipo == proto.TIPO_PONG:
                        self._estado.t_tele = max(self._estado.t_tele, ahora)
                    elif tipo == proto.TIPO_LOG:
                        texto = cuerpo.decode("ascii", "replace")
                        self._estado.logs.append(f"{ahora:.3f} {texto}")
                        del self._estado.logs[:-40]
                    self._estado.tramas_malas = self._lector.tramas_malas

    # -------------------------------------------------------------- envio
    def _bucle_tx(self) -> None:
        proximo = time.time()
        while not self._parar.is_set():
            proximo += PERIODO_TX
            with self._lock:
                self._seq = (self._seq + 1) & 0xFF
                m = dict(self._mando)
                seq = self._seq
            self.enviar_crudo(proto.trama_mando(seq, **m))
            # Dormir lo que falte, nunca un periodo entero: si un ciclo se
            # retrasa, el siguiente se acorta y el ritmo medio sigue a 50 Hz.
            espera = proximo - time.time()
            if espera > 0:
                time.sleep(espera)
            else:
                proximo = time.time()

    def enviar_crudo(self, trama: bytes) -> bool:
        ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(trama)
            return True
        except (OSError, serial.SerialException):
            with self._lock:
                self._estado.conectado = False
            return False

    # ------------------------------------------------------------- mandos
    def mandar(self, vel: float, direccion: float, *, armado: bool = True,
               parada: bool = False, centrar: bool = False,
               limpiar: bool = False, vmax: int = 255, aux: int = 0) -> None:
        """Deja el mando nuevo. NO escribe en el puerto: de eso se encarga el
        hilo TX a 50 Hz. Llamar a esto mas rapido que 50 Hz no hace daño;
        llamarlo mas lento tampoco, porque el ultimo se repite."""
        with self._lock:
            self._mando = dict(vel=vel, direccion=direccion, armado=armado,
                               parada=parada, centrar=centrar, limpiar=limpiar,
                               vmax=vmax, aux=aux)

    def parar(self) -> None:
        """Parada inmediata y desarme. Idempotente: se puede llamar mil veces."""
        self.mandar(0, 0, armado=False, parada=True, centrar=True)

    def configurar_servo(self, centro: int, izquierda: int, derecha: int,
                         grados_por_seg: int = 320, rampa: int = 10,
                         ms_freno_inversion: int = 150) -> None:
        self.enviar_crudo(proto.trama_config(centro, izquierda, derecha,
                                             grados_por_seg, rampa,
                                             ms_freno_inversion))

    def calibrar(self, comando: int) -> None:
        self.enviar_crudo(proto.trama_cal(comando))

    # -------------------------------------------------------------- estado
    def estado(self) -> EstadoEnlace:
        with self._lock:
            e = EstadoEnlace(
                conectado=self._estado.conectado,
                puerto=self._estado.puerto,
                tele=self._estado.tele,
                sens=self._estado.sens,
                t_tele=self._estado.t_tele,
                t_sens=self._estado.t_sens,
                tramas_malas=self._estado.tramas_malas,
                logs=list(self._estado.logs))
        return e

    # -------------------------------------------------------------- cierre
    def cerrar(self) -> None:
        # Orden importante: primero parar el carro con el hilo TX aun vivo,
        # despues bajar los hilos. Al reves, el ultimo mando en volar podria
        # ser uno con velocidad.
        try:
            self.parar()
            time.sleep(3 * PERIODO_TX)
        except Exception:
            pass
        self._parar.set()
        for h in self._hilos:
            h.join(timeout=0.5)
        self._hilos = []
        if self._ser is not None:
            try:
                self._ser.write(proto.trama_mando(0, 0, 0, armado=False,
                                                  parada=True, centrar=True))
                self._ser.flush()
            except Exception:
                pass
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        with self._lock:
            self._estado.conectado = False

    def __enter__(self) -> "Enlace":
        self.abrir()
        return self

    def __exit__(self, *_) -> None:
        self.cerrar()
