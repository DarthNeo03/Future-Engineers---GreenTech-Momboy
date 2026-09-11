#!/usr/bin/env python3
"""
selftest.py — Pruebas sin carro, sin camara y sin ESP32.

QUE SE PRUEBA AQUI Y POR QUE ESTAS COSAS Y NO OTRAS

Se prueba lo que, si se rompe, rompe la ronda entera y NO se nota mirando el
carro rodar: el empaquetado del protocolo, el lado por el que se rebasa cada
color, la geometria pixel-milimetro y el contador de vueltas. Todo eso puede
estar mal y el carro seguir pareciendo que funciona durante media pista.

No se prueba el ajuste de las ganancias ni los rangos HSV: eso no es correcto
o incorrecto, es mejor o peor, y se mide en la pista con el cronometro.

    python3 tools/selftest.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src import protocolo as proto              # noqa: E402
from src.geometria import Geometria             # noqa: E402
from src.senales import LADO_OBLIGADO           # noqa: E402
from src.vueltas import Contador                # noqa: E402

fallos = 0


def check(nombre: str, condicion: bool, detalle: str = "") -> None:
    global fallos
    if condicion:
        print(f"  ok   {nombre}")
    else:
        fallos += 1
        print(f"  FALLA {nombre}  {detalle}")


# ======================================================== protocolo
def prueba_protocolo() -> None:
    print("protocolo")

    # CRC conocido: si este cambia, el firmware y la Pi ya no se entienden.
    check("crc8 vacio", proto.crc8(b"") == 0x00)
    check("crc8 '123456789'", proto.crc8(b"123456789") == 0xF4,
          hex(proto.crc8(b"123456789")))

    # Ida y vuelta completa por el Lector.
    lec = proto.Lector()
    t = proto.trama_mando(7, 42, -31, armado=True, vmax=200, aux=1)
    salida = lec.alimentar(t)
    check("mando ida y vuelta", len(salida) == 1 and salida[0][0] == proto.TIPO_MANDO)
    p = salida[0][1]
    check("mando seq/vel/dir", p[0] == 7 and p[2] == 42 and (p[3] - 256) == -31,
          str(list(p)))
    check("mando bandera armado", bool(p[1] & proto.F_ARMADO))

    # Recorte a +-100: una orden absurda no puede llegar al servo.
    t = proto.trama_mando(0, 999, -999)
    p = lec.alimentar(t)[0][1]
    check("recorte de vel/dir", p[2] == 100 and (p[3] - 256) == -100)

    # Un CRC roto se descarta y se cuenta.
    malo = bytearray(proto.trama_mando(1, 10, 10))
    malo[-1] ^= 0xFF
    antes = lec.tramas_malas
    check("CRC roto descartado",
          lec.alimentar(bytes(malo)) == [] and lec.tramas_malas == antes + 1)

    # Basura delante no impide encontrar la trama siguiente (resincronismo).
    lec2 = proto.Lector()
    flujo = b"\xA5\xA5\x00\xFF" + proto.trama_ping(9)
    check("resincroniza tras basura", len(lec2.alimentar(flujo)) == 1)

    # Trama partida en dos lecturas del puerto: caso normal a 115200.
    lec3 = proto.Lector()
    t = proto.trama_mando(3, 5, 5)
    check("trama partida", lec3.alimentar(t[:4]) == [] and
          len(lec3.alimentar(t[4:])) == 1)

    # Sensores: ida y vuelta con valores negativos y contadores.
    s_bytes = bytes([proto.S_MPU_OK | proto.S_TCS_OK |
                     (proto.LINEA_AZUL << 6)]) + \
        (-1234).to_bytes(2, "little", signed=True) + \
        (-77).to_bytes(2, "little", signed=True) + \
        (3000).to_bytes(2, "little") + bytes([5, 9, proto.B_NIVEL, 11, 1])
    s = proto.decodificar_sensores(s_bytes)
    check("sensores yaw negativo", s is not None and abs(s.yaw + 123.4) < 1e-6,
          str(s))
    check("sensores clase linea", s.clase_linea == proto.LINEA_AZUL)
    check("sensores contadores", s.cruces_naranja == 5 and s.cruces_azul == 9)
    check("sensores boton", s.boton_pisado and not s.cortado_por_boton)


# ======================================================== reglas del juego
def prueba_reglas() -> None:
    print("reglas del juego (9.19)")
    # Esta es LA regla del reto. Si se invierte, la ronda se termina en el
    # primer pilar y no hay forma de darse cuenta mirando el codigo de control.
    check("rojo se pasa por su DERECHA", LADO_OBLIGADO["rojo"] == +1)
    check("verde se pasa por su IZQUIERDA", LADO_OBLIGADO["verde"] == -1)

    from src.senales import MITAD_PILAR_MM
    check("pilar de 50 mm -> mitad 25", MITAD_PILAR_MM == 25.0)


# ======================================================== geometria
def prueba_geometria() -> None:
    print("geometria")
    cfg = {"alto_cam_mm": 125.0, "inclinacion_deg": 7.5,
           "fy_px": 460.0, "fx_px": 460.0,
           "ancho_carro_mm": 200.0, "margen_ruedas_mm": 30.0}
    g = Geometria(cfg, 640, 480)

    hy = g.fila_horizonte()
    check("horizonte dentro del frame", 0 < hy < 480, f"fila {hy}")

    # Ida y vuelta: una distancia -> fila -> distancia.
    for d in (300.0, 800.0, 1500.0, 2500.0):
        v = g.distancia_a_fila(d)
        d2 = float(g.fila_a_distancia(v))
        check(f"ida y vuelta {d:.0f} mm", abs(d2 - d) / d < 0.05,
              f"-> fila {v} -> {d2:.0f} mm")

    # Monotonia: bajar en la imagen es acercarse. Si esto se rompe, el signo
    # de la inclinacion esta invertido y el carro cree que el muro se aleja.
    d_arriba = float(g.fila_a_distancia(hy + 40))
    d_abajo = float(g.fila_a_distancia(479))
    check("mas abajo = mas cerca", d_abajo < d_arriba,
          f"{d_abajo:.0f} vs {d_arriba:.0f}")

    # Lateral: a la derecha del centro, positivo.
    x, _ = g.punto_suelo(500, 400)
    check("lateral positivo a la derecha", x > 0, f"{x:.0f} mm")

    # Distancia por altura aparente de un pilar de 100 mm.
    # A 1000 mm con fy=460 deberian ser 46 px de alto.
    d = g.distancia_por_altura(46.0)
    check("distancia por altura", abs(d - 1000.0) < 40.0, f"{d:.0f} mm")

    # Coherencia: dos medidas parecidas pasan, dos dispares no.
    check("coherencia acepta parecidas", g.coherente(1000.0, 1150.0))
    check("coherencia rechaza dispares", not g.coherente(400.0, 1800.0))


# ======================================================== vueltas
def prueba_vueltas() -> None:
    print("contador de vueltas")
    c = Contador({"color_entrada_horario": "naranja", "margen_meta_mm": 120.0})

    def sens(nar: int, azu: int) -> proto.Sensores:
        return proto.Sensores(estado=proto.S_TCS_OK, cruces_naranja=nar,
                              cruces_azul=azu)

    # Primera trama: se toma como cero, no cuenta cruces.
    c.actualizar(sens(3, 2), 0.0, 0.02)
    check("primera trama no cuenta", c.e.cruces_totales == 0)

    # Rodar 900 mm y pisar la primera naranja: fija sentido horario y la
    # distancia de referencia hasta la meta.
    c.actualizar(sens(3, 2), 900.0, 1.0)
    c.actualizar(sens(4, 2), 0.0, 0.02)
    check("sentido horario por naranja", c.e.sentido == +1)
    check("referencia de meta guardada",
          c.e.dist_arranque_a_primer_cruce_mm is not None and
          abs(c.e.dist_arranque_a_primer_cruce_mm - 900.0) < 1.0,
          str(c.e.dist_arranque_a_primer_cruce_mm))

    # Tres vueltas: 4 naranjas y 4 azules por vuelta.
    nar, azu = 4, 2
    for _ in range(3):
        for _ in range(4):
            azu += 1
            c.actualizar(sens(nar, azu), 0.0, 0.02)
            nar += 1
            c.actualizar(sens(nar, azu), 0.0, 0.02)
    # Se han metido 12 naranjas mas la primera = 13 -> 3 vueltas completas.
    check("tres vueltas contadas", c.e.vueltas == 3, str(c.e.vueltas))
    check("24 secciones", c.e.secciones == 25, str(c.e.secciones))

    # Aun no se ha recorrido la distancia hasta la meta.
    c.evaluar_parada(3)
    check("no para antes de la meta", not c.e.listo_para_parar)

    # Rodar los 900 mm de referencia: ahora si.
    c.actualizar(sens(nar, azu), 950.0, 1.0)
    c.evaluar_parada(3)
    check("para al llegar a la meta", c.e.listo_para_parar, c.e.motivo)

    # El contador del ESP32 es de 8 bits: tiene que envolver sin perder cruces.
    c2 = Contador({})
    c2.actualizar(proto.Sensores(estado=proto.S_TCS_OK, cruces_naranja=254), 0, 0.02)
    c2.actualizar(proto.Sensores(estado=proto.S_TCS_OK, cruces_naranja=1), 0, 0.02)
    check("contador de 8 bits envuelve", c2.e.cruces_totales == 3,
          str(c2.e.cruces_totales))


# ======================================================== maquina de estados
def prueba_fsm() -> None:
    print("maquina de estados")
    import time as _t

    from src.config import DEFECTOS
    from src.carril import SalidaCarril
    from src.fsm import Contexto, Estado, MaquinaEstados
    from src.senales import FASE_APROXIMACION, FASE_COMPROMISO, Maniobra

    cfg = dict(DEFECTOS["fsm"])
    cfg["arranque_s"] = 0.01
    fsm = MaquinaEstados(cfg)

    def ctx(**kw):
        base = dict(enlace_ok=True, hay_frame=True,
                    carril=SalidaCarril(dist_frente_mm=2000.0),
                    velocidad_sugerida=40.0)
        base.update(kw)
        return Contexto(**base)

    # Estado de espera: quieto y DESARMADO, pase lo que pase (regla 9.11).
    o = fsm.paso(ctx())
    check("arranca en espera", fsm.estado == Estado.ESPERA)
    check("en espera no se arma", not o.armado and o.vel == 0.0)

    # Un boton pulsado mientras el giroscopio calibra NO arranca la ronda.
    o = fsm.paso(ctx(arranque_pedido=True,
                     sens=proto.Sensores(estado=proto.S_CALIBRANDO)))
    check("no arranca calibrando", fsm.estado == Estado.ESPERA)

    # Ahora si.
    fsm.paso(ctx(arranque_pedido=True))
    check("boton -> arranque", fsm.estado == Estado.ARRANQUE)
    _t.sleep(0.02)
    o = fsm.paso(ctx())
    check("arranque -> pista", fsm.estado == Estado.PISTA)
    check("en pista se arma", o.armado and o.vel > 0)

    # Pilar rojo a media distancia: entra en SENAL y rebasa por la derecha.
    m = Maniobra(direccion=30.0, peso=0.6, fase=FASE_APROXIMACION,
                 color="rojo", lado=+1, dist_mm=900.0)
    o = fsm.paso(ctx(maniobra=m, direccion_mezclada=30.0))
    check("pilar -> senal", fsm.estado == Estado.SENAL)
    check("nota dice el lado correcto", "derecha" in o.nota, o.nota)

    # El pilar entra en zona ciega: compromiso, y manda SOLO el esquive.
    m2 = Maniobra(direccion=44.0, peso=1.0, fase=FASE_COMPROMISO, color="rojo")
    o = fsm.paso(ctx(maniobra=m2, direccion_mezclada=-10.0))
    check("compromiso -> esquive", fsm.estado == Estado.ESQUIVE)
    check("en compromiso ignora el carril", abs(o.direccion - 44.0) < 1e-6,
          str(o.direccion))

    # Se cumple el compromiso: vuelve a pista.
    fsm.paso(ctx())
    check("esquive -> pista", fsm.estado == Estado.PISTA)

    # Perder el enlace corta de inmediato, desde cualquier estado.
    o = fsm.paso(ctx(enlace_ok=False))
    check("sin enlace -> fallo", fsm.estado == Estado.FALLO)
    check("en fallo no se arma", not o.armado and o.vel == 0.0)
    fsm.paso(ctx())
    check("fallo -> pista al recuperar", fsm.estado == Estado.PISTA)

    # Tres vueltas y distancia cumplida: meta y parada desarmada.
    from src.vueltas import EstadoVueltas
    ev = EstadoVueltas(vueltas=3, listo_para_parar=False)
    fsm.paso(ctx(vueltas=ev))
    check("tres vueltas -> meta", fsm.estado == Estado.META)
    ev2 = EstadoVueltas(vueltas=3, listo_para_parar=True, motivo="prueba")
    o = fsm.paso(ctx(vueltas=ev2))
    check("meta -> fin", fsm.estado == Estado.FIN)
    check("al terminar, desarmado y quieto",
          not o.armado and o.parada and o.vel == 0.0)


def main() -> int:
    prueba_protocolo()
    prueba_reglas()
    prueba_geometria()
    prueba_vueltas()
    prueba_fsm()
    print()
    if fallos:
        print(f"{fallos} prueba(s) FALLARON")
        return 1
    print("todo correcto")
    return 0


if __name__ == "__main__":
    sys.exit(main())
