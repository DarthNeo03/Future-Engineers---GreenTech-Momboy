#!/usr/bin/env python3
"""
asistente.py — Calibracion guiada del carro, de principio a fin.

    python3 tools/asistente.py              # todo, en orden
    python3 tools/asistente.py --solo 4 5   # solo unos pasos
    python3 tools/asistente.py --revisar    # solo comprobar, sin mover nada

QUE HACE SOLO Y QUE NO
----------------------
Automatico del todo:
  * comprobaciones de hardware (I2C, camara, ESP32, giroscopio);
  * el SIGNO del yaw, que si esta al reves hace que el carro gire para el
    lado contrario y es dificil de ver a ojo;
  * el RADIO DE GIRO, midiendolo con el giroscopio en vez de con tiza;
  * la coherencia entre `parar_mm` y la distancia minima que la camara puede
    medir, que si no cuadra deja la parada de seguridad sin poder dispararse.

Necesita que midas tu, porque no hay forma de saberlo desde dentro:
  * la distancia que recorre el carro en la prueba de velocidad (cinta
    metrica). Una camara sola no puede recuperar la escala del mundo: hace
    falta UNA medida real de referencia, y esta es la del carro.

Tiene su propia herramienta, porque necesitan ver por la camara:
  * la homografia al suelo   -> tools/calibrar_suelo.py --tablero
  * los colores HSV          -> tools/calibrador.py
  * el sensor de color       -> tools/calibrar_piso.py

SEGURIDAD
---------
Los pasos 4, 5 y 6 MUEVEN EL CARRO. Antes de cada uno se dice exactamente que
va a hacer y hay que confirmar. Las velocidades son bajas, cada movimiento
tiene un tope de tiempo y el `finally` manda parada de emergencia pase lo que
pase, tambien con Ctrl+C.

Ponlo en el suelo con AL MENOS 1,5 m despejados por delante y 1 m a los
lados. Levantar las ruedas no vale: el paso 5 mide distancia real y el 6
necesita que el carro gire de verdad.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import camera, color_config as cc, color_piso as cp  # noqa: E402
from src import enlace as enl, geometria as geo, imu as imu_mod  # noqa: E402
from src import robot_config  # noqa: E402

VERDE, ROJO, AMAR, GRIS, FIN = "\033[92m", "\033[91m", "\033[93m", "\033[90m", "\033[0m"


def ok(t: str) -> None:
    print(f"  {VERDE}OK{FIN}   {t}")


def mal(t: str) -> None:
    print(f"  {ROJO}MAL{FIN}  {t}")


def avi(t: str) -> None:
    print(f"  {AMAR}!{FIN}    {t}")


def titulo(n: int, t: str) -> None:
    print(f"\n{'='*66}\n  PASO {n}. {t}\n{'='*66}")


def preguntar(t: str, por_defecto: bool = False) -> bool:
    suf = "[S/n]" if por_defecto else "[s/N]"
    r = input(f"\n  {t} {suf} ").strip().lower()
    return por_defecto if not r else r.startswith("s")


def numero(t: str, minimo: float = 0.0, maximo: float = 1e9) -> Optional[float]:
    while True:
        r = input(f"  {t} (vacio = saltar): ").strip().replace(",", ".")
        if not r:
            return None
        try:
            v = float(r)
        except ValueError:
            print("    no es un numero")
            continue
        if not (minimo <= v <= maximo):
            print(f"    tiene que estar entre {minimo:.0f} y {maximo:.0f}")
            continue
        return v


# ===========================================================================
class Banco:
    """Enlace con el ESP32 y giroscopio, con parada garantizada al salir."""

    def __init__(self, cfg: Dict[str, Any], simulado: bool = False):
        self.cfg = cfg
        self.enlace = enl.Enlace(cfg["enlace"], simulado=simulado,
                                 al_log=lambda s: print(f"    {GRIS}{s}{FIN}"))
        self.imu = imu_mod.IMU(cfg["imu"])
        self.simulado = simulado

    def __enter__(self) -> "Banco":
        # Tope de PWM bajo durante toda la calibracion: aqui no interesa
        # correr, y un carro lento se recupera de un error tonto.
        self.enlace.fijar_vmax(int(self.cfg["calibracion"]["vmax"]))
        self.enlace.iniciar()
        time.sleep(2.5)                    # que le de tiempo a encontrar el puerto
        self.imu.iniciar()
        return self

    def __exit__(self, *_exc) -> None:
        try:
            self.enlace.parar(emergencia=True)
            time.sleep(0.2)
        finally:
            self.enlace.cerrar()
            self.imu.parar()

    def mover(self, vel: int, direccion: int, segundos: float,
              muestrear: bool = False) -> List[Tuple[float, float]]:
        """Mueve el carro un tiempo acotado y devuelve (t, yaw) si se pide.

        El enlace manda a 50 Hz por su cuenta, pero pone la velocidad a cero
        si nadie refresca la orden en 250 ms. O sea que este bucle no es solo
        para muestrear: mientras no gire, el carro no se mueve. Es la misma
        red que impide que un cuelgue deje el carro a fondo.
        """
        muestras: List[Tuple[float, float]] = []
        t0 = time.monotonic()
        self.enlace.rearmar()
        try:
            while True:
                t = time.monotonic() - t0
                if t >= segundos:
                    break
                self.enlace.mandar(vel, direccion, armado=True)
                if muestrear and self.imu.disponible:
                    muestras.append((t, self.imu.yaw))
                time.sleep(0.02)
        finally:
            self.enlace.mandar(0, 0, armado=False)
            self.enlace.parar()
            time.sleep(0.6)                # que frene del todo antes de seguir
        return muestras


def _desenrollar(muestras: List[Tuple[float, float]]) -> float:
    """Yaw total recorrido, sin envolverse en +-180."""
    total = 0.0
    for (_t0, a), (_t1, b) in zip(muestras, muestras[1:]):
        total += (b - a + 180.0) % 360.0 - 180.0
    return total


# ===========================================================================
def _sensores_por_esp32(cfg: Dict[str, Any], est: Dict[str, bool]) -> None:
    """Pregunta por los sensores AL ESP32, que es donde cuelgan ahora.

    Escanear el I2C de la Raspberry aqui no sirve de nada: desde que los dos
    sensores se movieron al ESP32, ese bus esta vacio A PROPOSITO. Preguntar
    en el sitio equivocado daba un "no se ve el MPU6050" que asustaba y que
    era mentira.
    """
    e = enl.Enlace(cfg["enlace"], al_log=lambda t: None)
    e.iniciar()
    print("  preguntando al ESP32...")
    time.sleep(2.5)
    try:
        if not e.conectado:
            mal(f"sin ESP32: {e.motivo}")
            print("       Los sensores cuelgan de el, asi que sin enlace no se")
            print("       puede saber si responden. Revisa el cruce TX/RX.")
            est["imu"] = est["piso"] = False
            return
        ok(f"ESP32: {e.motivo}")
        t = e.telemetria
        if t.version < 3:
            avi(f"firmware version {t.version}: es anterior a los sensores en el "
                f"ESP32.\n       Sube firmware/esp32_carro/esp32_carro.ino "
                f"(los cuatro archivos juntos)")
            est["imu"] = est["piso"] = False
            return
        est["imu"] = bool(t.imu_ok)
        est["piso"] = bool(t.piso_ok)
        if t.imu_ok:
            ok(f"MPU6050 en el ESP32   (yaw {t.yaw:+.1f} grados"
               f"{', calibrando' if t.imu_calibrando else ''})")
        else:
            avi("el ESP32 NO ve el MPU6050. Revisa SDA=GPIO21, SCL=GPIO22 y 3V3")
        if t.piso_ok:
            ok(f"TCS34725 en el ESP32  ({t.lineas} lineas cruzadas)")
        else:
            avi("el ESP32 NO ve el TCS34725. Mismo bus, misma comprobacion")
    finally:
        e.cerrar()


def _sensores_por_i2c(cfg: Dict[str, Any], est: Dict[str, bool]) -> None:
    """Montaje anterior: los sensores en el I2C de la Raspberry."""
    try:
        from smbus2 import SMBus
        with SMBus(int(cfg["imu"]["bus"])) as bus:
            vistos = []
            for dirn in (0x29, 0x68, 0x69):
                try:
                    bus.read_byte(dirn)
                    vistos.append(dirn)
                except Exception:
                    pass
        est["imu"] = any(d in vistos for d in (0x68, 0x69))
        est["piso"] = 0x29 in vistos
        if est["imu"]:
            ok(f"MPU6050 en 0x{[d for d in vistos if d in (0x68, 0x69)][0]:02X} "
               f"(I2C de la Pi)")
        else:
            avi("no se ve el MPU6050 (0x68/0x69) en el I2C de la Pi")
        if est["piso"]:
            ok("TCS34725 en 0x29 (I2C de la Pi)")
        else:
            avi("no se ve el TCS34725 (0x29) en el I2C de la Pi")
    except Exception as e:
        avi(f"sin I2C ({e}). En Windows es normal")
        est["imu"] = est["piso"] = False


def paso1_hardware(cfg: Dict[str, Any]) -> Dict[str, bool]:
    titulo(1, "Comprobaciones de hardware (no mueve nada)")
    est: Dict[str, bool] = {}

    # --- sensores, donde toque segun la configuracion --------------------
    fuente = str(cfg["imu"].get("fuente", "esp32"))
    if fuente == "esp32":
        _sensores_por_esp32(cfg, est)
    else:
        _sensores_por_i2c(cfg, est)

    # --- camara ----------------------------------------------------------
    c = cfg["camara"]
    cap = camera.abrir(indice=c["indice"], ancho=c["ancho"], alto=c["alto"],
                       fps=c["fps"], fourcc=c["fourcc"], verbose=False)
    if cap is None:
        mal("no se pudo abrir la camara")
        est["camara"] = False
    else:
        leidos = sum(1 for _ in range(10) if cap.read()[0])
        cap.release()
        if leidos >= 8:
            ok(f"camara: {leidos}/10 fotogramas")
            est["camara"] = True
        else:
            mal(f"camara inestable: solo {leidos}/10 fotogramas")
            est["camara"] = False

    # --- calibraciones ya hechas -----------------------------------------
    suelo = geo.Suelo(cfg["camara"]).cargar()
    est["suelo"] = suelo.calibrado
    if suelo.calibrado:
        ok(f"suelo calibrado: {suelo.origen}")
    else:
        avi("suelo SIN CALIBRAR: usa tools/calibrar_suelo.py --tablero")

    perfil = cc.obtener(cc.cargar())
    ok(f"perfil de color activo: '{perfil.get('nombre','?')}'")

    piso_cfg = cfg.get("piso", {})
    nar = [q for q in piso_cfg.get("perfiles", []) if q["nombre"] == "naranja"]
    if nar and abs(nar[0]["r"] - 0.55) < 1e-9:
        avi("el sensor de piso tiene los perfiles DE FABRICA: calibra con "
            "tools/calibrar_piso.py")
        if str(piso_cfg.get("fuente", "esp32")) == "esp32":
            print("       y acuerdate de copiarlos luego al array perfiles[] "
                  "del .ino:\n       con el sensor en el ESP32, quien clasifica "
                  "es el firmware")
    elif nar:
        ok("sensor de piso con perfiles medidos")
    return est


def paso2_geometria(cfg: Dict[str, Any]) -> None:
    titulo(2, "Coherencia de la geometria (no mueve nada)")
    suelo = geo.Suelo(cfg["camara"]).cargar()
    nav = cfg["navegacion"]
    alto = int(cfg["camara"]["alto"])

    print(f"  {suelo.diagnostico_fov(float(nav['pared_objetivo_mm']))}")
    hfov = float(cfg["camara"].get("hfov_deg", 70.0))
    if hfov < 90.0:
        avi(f"HFOV de {hfov:.0f} grados es estrecho. El muro externo no entra "
            f"en el encuadre en campo cercano,\n       que es justo donde el "
            f"guardia de la regla 9.18 tendria que actuar")
    else:
        ok(f"campo de vision de {hfov:.0f} grados: suficiente")

    z_min = suelo.z_minimo_medible(alto, float(nav.get("ignorar_abajo", 0.0)))
    parar = float(nav["parar_mm"])
    print(f"  distancia minima medible: {z_min:.0f} mm   |   parar_mm: {parar:.0f} mm")
    if z_min == z_min and parar < z_min:
        mal(f"parar_mm ({parar:.0f}) esta POR DEBAJO del minimo medible "
            f"({z_min:.0f}).\n       La parada de seguridad no podria "
            f"dispararse nunca: sube parar_mm")
    else:
        ok("la parada de seguridad puede dispararse")

    z_disp = float(nav["radio_giro_mm"]) - float(nav["pared_objetivo_mm"])
    print(f"  disparo del giro: radio {nav['radio_giro_mm']:.0f} - objetivo "
          f"{nav['pared_objetivo_mm']:.0f} = {z_disp:.0f} mm")
    if z_disp < 40:
        avi("sale muy corto. Si el radio real es menor que el objetivo, "
            "el carro girara tarde")


def paso3_herramientas(cfg: Dict[str, Any]) -> None:
    titulo(3, "Calibraciones con camara (cada una tiene su herramienta)")
    suelo = geo.Suelo(cfg["camara"]).cargar()
    print("  En este orden, porque cada una depende de la anterior:\n")
    print(f"  {'[hecho]' if suelo.calibrado else '[FALTA]'} 1) Suelo    "
          f"python3 tools/calibrar_suelo.py --tablero")
    print("            Sin esto todo lo metrico se apoya en una estimacion.")
    print("  [manual]  2) Colores  python3 tools/calibrador.py")
    print("            Sobre el tapete y con la luz de la pista.")
    print("  [manual]  3) Piso     python3 tools/calibrar_piso.py")
    print("            Solo si montaste el TCS34725.")
    print("\n  Vuelve aqui cuando las tengas y sigue con el paso 4.")


def paso4_signo_yaw(cfg: Dict[str, Any], banco: Banco) -> Optional[bool]:
    titulo(4, "Signo del giroscopio  (MUEVE EL CARRO)")
    if not banco.imu.disponible:
        avi(f"sin giroscopio ({banco.imu.motivo}); me lo salto")
        return None
    print("  El navegador espera yaw en convenio de BRUJULA: que AUMENTE al")
    print("  girar a la derecha. Si esta al reves, el carro toma las esquinas")
    print("  hacia el lado contrario y cuesta mucho verlo mirando el carro.\n")
    print("  Voy a girar a la DERECHA, despacio, poco mas de un segundo.")
    if not preguntar("Hay sitio despejado y puedo mover el carro?"):
        return None

    print("  calibrando el giroscopio, NO MUEVAS EL CARRO...")
    banco.imu.calibrar()
    time.sleep(0.3)
    m = banco.mover(vel=int(cfg["calibracion"]["vel"]), direccion=+80,
                    segundos=1.3, muestrear=True)
    if len(m) < 5:
        mal("no llegaron muestras del giroscopio")
        return None
    d = _desenrollar(m)
    print(f"  el yaw se movio {d:+.1f} grados girando a la DERECHA")
    if abs(d) < 8.0:
        mal("apenas se movio. El carro no giro, o el giroscopio no responde")
        return None
    if d > 0:
        ok("signo correcto: no hay que tocar nada")
        return False
    avi("signo INVERTIDO respecto a lo que espera el navegador")
    return preguntar("Pongo imu.invertir_yaw = true?", True)


def paso5_velocidad(cfg: Dict[str, Any], banco: Banco) -> Optional[float]:
    titulo(5, "Velocidad real  (MUEVE EL CARRO)")
    vel = int(cfg["calibracion"]["vel"])
    seg = float(cfg["calibracion"]["segundos_recto"])
    print(f"  Voy a avanzar RECTO al {vel}% durante {seg:.1f} s.")
    print("  Marca donde empieza, deja {:.1f} m libres y mide donde para."
          .format(seg * 1.2))
    print("\n  Es la unica medida que no puedo tomar yo: una camara sola no")
    print("  recupera la escala del mundo, hace falta una referencia real.")
    if not preguntar("Listo para arrancar?"):
        return None

    for n in (3, 2, 1):
        print(f"    {n}...")
        time.sleep(1.0)
    banco.mover(vel=vel, direccion=0, segundos=seg)
    print("  parado.")

    d = numero("Distancia recorrida en MILIMETROS", 50, 5000)
    if d is None:
        return None
    mm_s = d / seg / (vel / 100.0)
    print(f"  {d:.0f} mm en {seg:.1f} s al {vel}%  ->  "
          f"{d/seg:.0f} mm/s medidos  ->  {mm_s:.0f} mm/s al 100%")
    if not (200 <= mm_s <= 3000):
        avi("el numero sale raro. Comprueba que midieras bien y que el carro "
            "no patinara")
    return mm_s


def paso6_radio(cfg: Dict[str, Any], banco: Banco, mm_s: float) -> Optional[float]:
    titulo(6, "Radio de giro  (MUEVE EL CARRO)")
    if not banco.imu.disponible:
        avi("sin giroscopio no puedo medirlo. Hazlo con tiza: volante a tope, "
            "una vuelta,\n       y mide el circulo")
        return None
    vel = int(cfg["calibracion"]["vel"])
    seg = float(cfg["calibracion"]["segundos_circulo"])
    print("  Volante a tope y un arco, midiendo con el giroscopio cuanto gira.")
    print("  De la velocidad de giro y la velocidad de avance sale el radio:\n")
    print("      R = v / w        (v en mm/s, w en radianes/s)\n")
    print("  Es mas exacto que la tiza y no hace falta medir nada a mano.")
    print(f"  Necesita un circulo libre de {2.2:.1f} m de diametro aprox.")
    if not preguntar("Hay sitio?"):
        return None

    banco.imu.calibrar()
    time.sleep(0.3)
    for n in (3, 2, 1):
        print(f"    {n}...")
        time.sleep(1.0)
    m = banco.mover(vel=vel, direccion=+100, segundos=seg, muestrear=True)
    if len(m) < 10:
        mal("no llegaron muestras suficientes")
        return None

    giro = _desenrollar(m)
    dt = m[-1][0] - m[0][0]
    if abs(giro) < 25.0 or dt <= 0:
        mal(f"solo giro {giro:.0f} grados en {dt:.1f} s: no puedo calcular el radio")
        return None

    w = math.radians(abs(giro) / dt)          # rad/s
    v = mm_s * (vel / 100.0)                  # mm/s
    R = v / w
    print(f"  giro {abs(giro):.0f} grados en {dt:.1f} s  ->  {abs(giro)/dt:.0f} grados/s")
    print(f"  avance {v:.0f} mm/s  ->  radio = {R:.0f} mm")
    if not (80 <= R <= 900):
        avi("el radio sale fuera de lo esperable. Comprueba que el volante "
            "llegaba al tope\n       y que las ruedas no patinaban")
    else:
        ok(f"radio de giro {R:.0f} mm  ->  el giro se disparara a "
           f"{R - float(cfg['navegacion']['pared_objetivo_mm']):.0f} mm de la esquina")
    return R


# ===========================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description="Calibracion guiada del carro")
    ap.add_argument("--solo", type=int, nargs="+", default=None,
                    help="ejecutar solo estos pasos (1..6)")
    ap.add_argument("--revisar", action="store_true",
                    help="solo comprobar: no mueve el carro")
    ap.add_argument("--simulado", action="store_true",
                    help="sin ESP32, para probar el asistente")
    args = ap.parse_args()

    cfg = robot_config.cargar()
    cfg.setdefault("calibracion", {})
    cal = cfg["calibracion"]
    cal.setdefault("vmax", 90)
    cal.setdefault("vel", 45)
    cal.setdefault("segundos_recto", 2.0)
    cal.setdefault("segundos_circulo", 3.0)

    pasos = args.solo or ([1, 2, 3] if args.revisar else [1, 2, 3, 4, 5, 6])
    cambios: Dict[str, Any] = {}

    print(f"{'='*66}\n  ASISTENTE DE CALIBRACION\n{'='*66}")
    if any(p >= 4 for p in pasos):
        print(f"  {AMAR}Los pasos 4, 5 y 6 MUEVEN EL CARRO.{FIN} Ponlo en el "
              f"suelo con 1,5 m\n  despejados delante y 1 m a los lados. "
              f"Ctrl+C para abortar.")

    if 1 in pasos:
        paso1_hardware(cfg)
    if 2 in pasos:
        paso2_geometria(cfg)
    if 3 in pasos:
        paso3_herramientas(cfg)

    if any(p in pasos for p in (4, 5, 6)):
        with Banco(cfg, simulado=args.simulado) as banco:
            if not banco.enlace.conectado and not args.simulado:
                mal(f"sin ESP32: {banco.enlace.motivo}")
                print("  Revisa el cruce TX/RX y que el firmware este subido.")
                return 2
            ok(f"ESP32: {banco.enlace.motivo}")

            if 4 in pasos:
                inv = paso4_signo_yaw(cfg, banco)
                if inv is not None:
                    cambios[("imu", "invertir_yaw")] = inv

            mm_s = float(cfg["navegacion"]["mm_por_seg_a_100"])
            if 5 in pasos:
                medido = paso5_velocidad(cfg, banco)
                if medido:
                    cambios[("navegacion", "mm_por_seg_a_100")] = round(medido, 1)
                    mm_s = medido

            if 6 in pasos:
                R = paso6_radio(cfg, banco, mm_s)
                if R:
                    cambios[("navegacion", "radio_giro_mm")] = round(R, 1)

    # --- guardar ---------------------------------------------------------
    print(f"\n{'='*66}\n  RESUMEN\n{'='*66}")
    if not cambios:
        print("  Nada que cambiar.")
        return 0
    for (sec, clave), valor in cambios.items():
        print(f"  {sec}.{clave}: {cfg[sec].get(clave)}  ->  {VERDE}{valor}{FIN}")
    if preguntar("Guardar en config/robot.json?", True):
        for (sec, clave), valor in cambios.items():
            cfg[sec][clave] = valor
        cfg["calibracion"] = cal
        ruta = robot_config.guardar(cfg)
        print(f"  guardado en {ruta}")
        print("\n  Siguiente: python3 tools/simulador.py --todas   (32/32 esperado)")
        print("             python3 main.py --sin-panel")
    else:
        print("  sin guardar")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n\nabortado. El carro queda parado.")
        raise SystemExit(1)
