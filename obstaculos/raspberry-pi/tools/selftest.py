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
import struct
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

    # La trama de SENSORES y sus cuatro bytes de diagnostico, que son
    # opcionales: una Pi nueva con un ESP32 de firmware viejo tiene que
    # seguir funcionando, porque reflashear pide cable y se hace despues.
    p16 = struct.pack("<BhhHBBBBBHh", proto.S_TCS_OK, 123, -45, 700,
                      3, 2, 0, 1, 1, 950, -146)
    d = proto.decodificar_sensores(p16)
    check("sensores: 16 bytes traen el diagnostico de lineas",
          d is not None and d.blanco == 950 and d.separacion == -146 and
          d.lineas_diag, str(d))
    check("sensores: y lo de siempre sigue en su sitio",
          abs(d.yaw - 12.3) < 0.01 and d.claro == 700 and
          d.cruces_naranja == 3, str(d))
    d12 = proto.decodificar_sensores(p16[:12])
    check("sensores: con firmware viejo (12 B) se sigue leyendo",
          d12 is not None and d12.claro == 700 and not d12.lineas_diag,
          str(d12))
    check("la trama de sensores cabe en el payload maximo",
          16 <= proto.MAX_PAYLOAD, str(proto.MAX_PAYLOAD))

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
    # Con kilometraje realista: una vuelta son ~8 m, o sea ~1 m entre cruces.
    # Sin el, el suelo de distancia de evaluar_parada lo rechaza con razon —
    # tres vueltas no caben en dos metros.
    nar, azu = 4, 2
    for _ in range(3):
        for _ in range(4):
            azu += 1
            c.actualizar(sens(nar, azu), 1000.0, 1.0)
            nar += 1
            c.actualizar(sens(nar, azu), 1000.0, 1.0)
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

    # ---------------- el sentido por PAR ORDENADO de la esquina -----------
    # Una esquina tiene una linea naranja y una azul. El ORDEN en que se pisan
    # solo depende del sentido de la marcha, asi que confirma —o corrige— lo
    # que dijo la primera linea suelta.
    cfg_v = {"color_entrada_horario": "naranja", "ventana_par_mm": 1500.0}

    def rodar(cont, nar, azu, mm=600.0):
        """Un cruce nuevo despues de rodar mm milimetros."""
        cont.actualizar(sens_v(nar, azu), mm, 1.0)

    def sens_v(nar, azu):
        return proto.Sensores(estado=proto.S_TCS_OK, cruces_naranja=nar,
                              cruces_azul=azu)

    cp = Contador(cfg_v)
    cp.actualizar(sens_v(0, 0), 0.0, 0.02)
    rodar(cp, 1, 0, 800.0)                       # naranja: entrada
    check("con una sola linea el sentido es provisional",
          cp.e.sentido == +1 and not cp.e.sentido_firme, cp.e.origen_sentido)
    rodar(cp, 1, 1, 700.0)                       # azul: cierra el par
    check("el par naranja+azul confirma horario",
          cp.e.sentido == +1 and cp.e.sentido_firme, cp.e.origen_sentido)

    # EL CASO QUE JUSTIFICA TODO ESTO. Si el TCS se salta la linea de ENTRADA,
    # la de salida se toma por la primera y el sentido sale INVERTIDO: el carro
    # cree que las curvas van al otro lado y se estampa en la primera esquina
    # de frente. El par lo corrige en cuanto se ve una esquina entera.
    cm = Contador(cfg_v)
    cm.actualizar(sens_v(0, 0), 0.0, 0.02)
    rodar(cm, 0, 1, 800.0)          # se perdio la naranja: solo se ve la azul
    check("perdida la linea de entrada, el sentido sale al reves",
          cm.e.sentido == -1 and not cm.e.sentido_firme)
    rodar(cm, 1, 1, 2500.0)         # esquina siguiente, ya completa: naranja...
    rodar(cm, 1, 2, 700.0)          # ...y azul
    check("el par de la esquina siguiente lo corrige",
          cm.e.sentido == +1 and cm.e.sentido_firme, str(cm.e.sentido))

    # Una linea suelta NO se empareja con la de la esquina de al lado: estan a
    # metros de distancia, y un par inventado contesta al reves.
    cv_ = Contador(cfg_v)
    cv_.actualizar(sens_v(0, 0), 0.0, 0.02)
    rodar(cv_, 0, 1, 800.0)                      # azul suelta
    rodar(cv_, 1, 1, 3000.0)                     # naranja, media pista despues
    check("una linea suelta caduca en vez de emparejarse",
          not cv_.e.sentido_firme, cv_.e.origen_sentido)

    # Ya firme, un par al reves NO invierte el sentido: en esta pista no hay
    # media vuelta, asi que eso es un cruce mal leido y se anota como tal.
    ci = Contador(cfg_v)
    ci.actualizar(sens_v(0, 0), 0.0, 0.02)
    rodar(ci, 1, 0, 800.0)
    rodar(ci, 1, 1, 700.0)                       # firme: horario
    rodar(ci, 1, 2, 2500.0)                      # azul...
    rodar(ci, 2, 2, 700.0)                       # ...y naranja: par invertido
    check("un par invertido no cambia el sentido ya firme",
          ci.e.sentido == +1, str(ci.e.sentido))
    check("pero queda anotado como incoherente",
          ci.e.pares_incoherentes == 1, str(ci.e.pares_incoherentes))

    # El EVENTO de esquina: lo dispara la PRIMERA linea del par, no la
    # segunda. Es lo que la FSM usa para comprometer el giro de la curva.
    ce = Contador(cfg_v)
    ce.actualizar(sens_v(0, 0), 0.0, 0.02)
    e = ce.actualizar(sens_v(1, 0), 800.0, 1.0)
    check("la primera linea de la esquina abre esquina", e.esquina_abierta)
    check("y dice que linea fue", e.linea_nueva == "naranja", e.linea_nueva)
    e = ce.actualizar(sens_v(1, 1), 700.0, 1.0)
    check("la segunda linea NO abre otra esquina", not e.esquina_abierta)
    # Los eventos duran un ciclo: si se quedaran pegados, la FSM dispararia
    # un giro por cada ciclo de camara mientras siguiera puesto.
    e = ce.actualizar(sens_v(1, 1), 100.0, 0.1)
    check("el evento de esquina dura un solo ciclo",
          not e.esquina_abierta and e.linea_nueva == "")

    # Si los dos colores suben en el MISMO ciclo, el orden no se sabe: los
    # cruces cuentan, pero no forman par.
    cs = Contador(cfg_v)
    cs.actualizar(sens_v(0, 0), 0.0, 0.02)
    cs.actualizar(sens_v(1, 1), 800.0, 1.0)
    check("dos colores en el mismo ciclo no forman par",
          not cs.e.sentido_firme and cs.e.cruces_totales == 2,
          f"firme={cs.e.sentido_firme} cruces={cs.e.cruces_totales}")

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

    # Se cumple el compromiso: NO se vuelve derecho a pista.
    #
    # EL FALLO DE PISTA. Lo que sale de un rebase es un carro pegado a un muro
    # y apuntandolo, asi que su camara ve el frente cerrado. Devolviendolo a
    # PISTA, el seguidor de carril leia eso como "esquina", bajaba el centrado
    # al 25 % y empujaba hacia el lado de las curvas: giro brusco contra la
    # pared justo despues de un esquive limpio.
    o = fsm.paso(ctx(carril=SalidaCarril(dist_frente_mm=600.0,
                                         err_centrado=-0.8,
                                         rumbo_hueco_deg=-22.0,
                                         muro_encima=True),
                     sens=proto.Sensores(estado=proto.S_MPU_OK, yaw=0.0),
                     direccion_mezclada=-60.0, guardia_muro=-30.0))
    check("esquive -> reincorporacion", fsm.estado == Estado.REINCORPORACION)

    # RECTO NO ES CENTRADO, y confundirlos era la desviacion que se seguia
    # viendo. El carro sale del rebase desplazado; si se le manda centrarse
    # hace una ese (cruza al centro, se pasa, vuelve). Aqui se le manda
    # sostener el RUMBO: con el yaw en su sitio, lo unico que queda es la
    # guardia anti-muro, aunque el carril este pidiendo -60 % de centrado.
    check("con el rumbo en su sitio, no se cruza al centro",
          abs(o.direccion - (-30.0)) < 1.0,
          f"dir={o.direccion} (carril pedia -60, guardia -30)")

    # Y si el carro SI se sale del rumbo, vuelve a el.
    o = fsm.paso(ctx(carril=SalidaCarril(dist_frente_mm=600.0,
                                         err_centrado=-0.8,
                                         rumbo_hueco_deg=-22.0,
                                         muro_encima=True),
                     sens=proto.Sensores(estado=proto.S_MPU_OK, yaw=-10.0),
                     direccion_mezclada=-60.0))
    check("desviado 10 grados del rumbo, corrige hacia el",
          o.direccion > 15.0, f"dir={o.direccion}")

    # Mientras no este enfilado, se queda reincorporandose.
    fsm.paso(ctx(carril=SalidaCarril(dist_frente_mm=600.0, err_centrado=-0.8,
                                     rumbo_hueco_deg=-22.0, muro_encima=True),
                 sens=proto.Sensores(estado=proto.S_MPU_OK)))
    check("sigue reincorporandose si aun va cruzado",
          fsm.estado == Estado.REINCORPORACION, fsm.estado.value)

    # Un pilar nuevo manda sobre la reincorporacion: recolocarse no puede ser
    # motivo para pasar de largo la señal siguiente.
    m3 = Maniobra(direccion=-20.0, peso=0.7, fase=FASE_APROXIMACION,
                  color="verde", lado=-1, dist_mm=800.0)
    fsm.paso(ctx(maniobra=m3, direccion_mezclada=-20.0,
                 carril=SalidaCarril(dist_frente_mm=600.0, err_centrado=-0.8,
                                     rumbo_hueco_deg=-22.0, muro_encima=True)))
    check("reincorporacion -> senal con otro pilar",
          fsm.estado == Estado.SENAL, fsm.estado.value)

    # Y centrado y enfilado, ahora si: pista.
    fsm.paso(ctx(maniobra=Maniobra(direccion=44.0, peso=1.0,
                                   fase=FASE_COMPROMISO, color="rojo")))
    fsm.paso(ctx(carril=SalidaCarril(dist_frente_mm=1900.0, err_centrado=0.05,
                                     rumbo_hueco_deg=2.0)))
    check("reincorporado -> pista", fsm.estado == Estado.PISTA,
          fsm.estado.value)

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


# ================================ la curva la dispara la linea del piso
def prueba_esquina() -> None:
    print("curva disparada por la linea")
    import time as _t

    from src.config import DEFECTOS
    from src.carril import SalidaCarril
    from src.fsm import Contexto, Estado, MaquinaEstados
    from src.senales import FASE_APROXIMACION, Maniobra
    from src.vueltas import EstadoVueltas

    cfg = dict(DEFECTOS["fsm"])
    cfg["arranque_s"] = 0.01

    def arrancado():
        """Una FSM ya rodando en PISTA."""
        f = MaquinaEstados(cfg)
        f.paso(ctx())
        f.paso(ctx(arranque_pedido=True))
        _t.sleep(0.02)
        f.paso(ctx())
        return f

    def ctx(frente=2000.0, sentido=+1, yaw=0.0, esquina=False, dist=5000.0,
            maniobra=None, mpu=True, arranque_pedido=False):
        return Contexto(
            enlace_ok=True, hay_frame=True, velocidad_sugerida=40.0,
            arranque_pedido=arranque_pedido,
            sens=proto.Sensores(estado=proto.S_MPU_OK if mpu else 0, yaw=yaw),
            carril=SalidaCarril(dist_frente_mm=frente, sentido_curva=sentido),
            vueltas=EstadoVueltas(esquina_abierta=esquina, dist_mm=dist,
                                  sentido=sentido),
            maniobra=maniobra or Maniobra())

    # --- no se gira hasta que llega la linea ---------------------------
    # EL FALLO QUE SE VEIA: el carro empezaba a girar en cuanto la camara veia
    # el muro de la esquina, o sea un metro antes de la curva, y llegaba a la
    # linea ya torcido. Con el frente cerrado pero SIN linea, aqui no pasa
    # nada: sigue en PISTA y recto.
    f = arrancado()
    f.paso(ctx(frente=900.0))
    check("frente cerrado sin linea: sigue recto en pista",
          f.estado == Estado.PISTA, f.estado.value)

    # --- y con la linea, se compromete ---------------------------------
    o = f.paso(ctx(frente=900.0, esquina=True))
    check("la linea dispara la curva", f.estado == Estado.ESQUINA,
          f.estado.value)
    check("gira fuerte y hacia el sentido de la ronda",
          o.direccion >= float(cfg["dir_esquina"]) * 0.95,
          f"dir={o.direccion} (dir_esquina={cfg['dir_esquina']})")

    # Sigue girando aunque el hueco se abra: eso es el compromiso. El carril
    # aflojaria aqui, y por eso el giro se quedaba a medias.
    o = f.paso(ctx(frente=3000.0, yaw=20.0))
    check("con 20 grados girados todavia no suelta",
          f.estado == Estado.ESQUINA and
          o.direccion >= float(cfg["dir_esquina"]) * 0.95,
          f"{f.estado.value} dir={o.direccion}")

    # Girado lo que pide esquina_grados: hecho. El umbral se lee del config
    # para que cerrar mas el giro no rompa la prueba.
    hecho = float(cfg["esquina_grados"]) + 2.0
    f.paso(ctx(frente=3000.0, yaw=hecho))
    check(f"girados {cfg['esquina_grados']:.0f} grados, vuelve a pista",
          f.estado == Estado.PISTA, f.estado.value)

    # --- una esquina, un giro ------------------------------------------
    # Las dos lineas de una curva caen dentro de los mismos 1000 mm. Si la
    # segunda encadenara otro giro de 90, serian 180 y la pared de enfrente.
    f2 = arrancado()
    f2.paso(ctx(frente=900.0, esquina=True, dist=5000.0))
    check("primera linea: gira", f2.estado == Estado.ESQUINA)
    f2.paso(ctx(frente=900.0, yaw=hecho, dist=5400.0))         # curva hecha
    f2.paso(ctx(frente=900.0, esquina=True, dist=5600.0))      # 2a linea
    check("la segunda linea de la misma curva no encadena otro giro",
          f2.estado == Estado.PISTA, f2.estado.value)
    # Pero la curva SIGUIENTE, a metros de distancia, si.
    f2.paso(ctx(frente=900.0, esquina=True, dist=8000.0))
    check("la curva siguiente si vuelve a disparar",
          f2.estado == Estado.ESQUINA, f2.estado.value)

    # --- la camara confirma que la esquina esta ahi ---------------------
    # Si el TCS se perdio la linea de ENTRADA, la de SALIDA abre un par nuevo
    # y dispararia un giro justo al salir de la curva: contra el muro de la
    # recta. Al salir, el frente esta despejado, y eso basta para descartarlo.
    f3 = arrancado()
    f3.paso(ctx(frente=3000.0, esquina=True))
    check("linea con el frente despejado: no es una esquina, no gira",
          f3.estado == Estado.PISTA, f3.estado.value)

    # --- sin sentido no hay giro que comprometer ------------------------
    f4 = arrancado()
    f4.paso(ctx(frente=900.0, esquina=True, sentido=0))
    check("sin saber el sentido, no se inventa un giro",
          f4.estado == Estado.PISTA, f4.estado.value)

    # --- antihorario gira al otro lado ----------------------------------
    f5 = arrancado()
    o = f5.paso(ctx(frente=900.0, esquina=True, sentido=-1))
    check("en antihorario la curva va a la izquierda",
          o.direccion <= -float(cfg["dir_esquina"]) * 0.95,
          f"dir={o.direccion}")

    # --- un pilar manda sobre la curva -----------------------------------
    # Rebasar por el lado que toca vale 8 o 10 puntos y por el malo termina la
    # ronda; trazar bien la esquina no vale ninguno.
    f6 = arrancado()
    f6.paso(ctx(frente=900.0, esquina=True))
    pilar = Maniobra(direccion=-30.0, peso=0.8, fase=FASE_APROXIMACION,
                     color="verde", lado=-1, dist_mm=700.0)
    f6.paso(ctx(frente=900.0, yaw=15.0, maniobra=pilar))
    check("un pilar en plena curva interrumpe el giro",
          f6.estado == Estado.SENAL, f6.estado.value)

    # --- sin MPU se sale por tiempo, no se queda girando para siempre ----
    cfg_corto = dict(cfg)
    cfg_corto["esquina_max_s"] = 0.05
    f7 = MaquinaEstados(cfg_corto)
    f7.paso(ctx())
    f7.paso(ctx(arranque_pedido=True))
    _t.sleep(0.02)
    f7.paso(ctx())
    f7.paso(ctx(frente=900.0, esquina=True, mpu=False))
    check("sin MPU tambien entra en la curva", f7.estado == Estado.ESQUINA)
    _t.sleep(0.07)
    f7.paso(ctx(frente=900.0, mpu=False))
    check("sin MPU la curva termina por tiempo", f7.estado == Estado.PISTA,
          f7.estado.value)


# ============================================ esquive de extremo a extremo
def _frame_con_pilar(dist_mm, lat_mm, color_bgr, fy_real=460.0,
                     tilt_real=7.5, h_cam=125.0):
    """Fotograma sintetico: tapete blanco, muro negro al fondo y un pilar de
    50x100 mm proyectado con la geometria REAL (que no tiene por que ser la
    configurada). Sirve para probar el pipeline entero sin carro."""
    import math
    import cv2
    import numpy as np
    img = np.full((480, 640, 3), 235, np.uint8)
    cy, cx = 240.0, 320.0
    tilt = math.radians(tilt_real)
    theta = math.atan2(h_cam, dist_mm)
    v_base = cy + fy_real * math.tan(theta - tilt)
    rango = h_cam / math.sin(theta)
    alto_px = 100.0 * fy_real / dist_mm
    ancho_px = 50.0 * fy_real / dist_mm
    u = cx + lat_mm * fy_real / rango
    cv2.rectangle(img, (0, 0), (640, max(0, int(v_base - alto_px) - 30)),
                  (30, 30, 30), -1)
    cv2.rectangle(img, (int(u - ancho_px / 2), int(v_base - alto_px)),
                  (int(u + ancho_px / 2), int(v_base)), color_bgr, -1)
    return img


def prueba_esquive() -> None:
    print("esquive de extremo a extremo (regla 9.19)")
    import copy
    from src.config import DEFECTOS
    from src.geometria import Geometria
    from src.senales import Esquivador
    from src.vision import Detector

    ROJO, VERDE = (45, 35, 235), (50, 210, 70)

    def correr(color_bgr, dist=900.0, lat=0.0, **kw):
        geo = Geometria(dict(DEFECTOS["geometria"]), 640, 480)
        det = Detector(DEFECTOS["colores"], geo)
        esq = Esquivador(DEFECTOS["senales"], geo.semiancho_mm())
        esc = det.procesar(_frame_con_pilar(dist, lat, color_bgr, **kw))
        m = esq.paso(esc, {}, False, 800.0)
        return esc, m

    # --- caso nominal -------------------------------------------------
    esc, m = correr(ROJO)
    check("detecta el pilar rojo", len(esc.pilares) == 1,
          f"descartes={esc.descartes}")
    check("rojo: punto de paso a la DERECHA del pilar", m.objetivo_mm > 0,
          f"objetivo={m.objetivo_mm:.0f} mm")
    check("rojo: gira a la derecha", m.direccion > 0, f"dir={m.direccion:.1f}")
    check("rojo: el esquive toma autoridad", m.peso > 0.5, f"peso={m.peso}")

    esc, m = correr(VERDE)
    check("detecta el pilar verde", len(esc.pilares) == 1,
          f"descartes={esc.descartes}")
    check("verde: punto de paso a la IZQUIERDA del pilar", m.objetivo_mm < 0,
          f"objetivo={m.objetivo_mm:.0f} mm")
    check("verde: gira a la izquierda", m.direccion < 0, f"dir={m.direccion:.1f}")

    # --- ESTE es el caso que fallaba en pista --------------------------
    # Mastil torcido: la inclinacion real no es la configurada. Antes, el
    # filtro de coherencia borraba TODOS los pilares y el carro pasaba de
    # largo sin mirar el color.
    for tilt in (10.0, 12.0, 14.0):
        esc, m = correr(ROJO, tilt_real=tilt)
        check(f"rojo sigue viendose con inclinacion real {tilt} deg",
              len(esc.pilares) == 1, f"descartes={esc.descartes}")
        check(f"rojo sigue yendo por la derecha con {tilt} deg",
              m.objetivo_mm > 0 and m.direccion > 0,
              f"objetivo={m.objetivo_mm:.0f} dir={m.direccion:.1f}")
        esc, m = correr(VERDE, tilt_real=tilt)
        check(f"verde sigue yendo por la izquierda con {tilt} deg",
              m.objetivo_mm < 0 and m.direccion < 0,
              f"objetivo={m.objetivo_mm:.0f} dir={m.direccion:.1f}")

    # --- fy mal calibrada ---------------------------------------------
    esc, m = correr(ROJO, fy_real=554.0)
    check("rojo sobrevive a fy mal calibrada",
          len(esc.pilares) == 1 and m.direccion > 0, f"descartes={esc.descartes}")

    # --- el lado no depende de donde este el pilar en el cuadro --------
    # Un pilar rojo YA a la izquierda del carro se rebasa igual por su
    # derecha, y eso puede significar NO girar a la derecha del todo.
    esc, m = correr(ROJO, lat=-220.0)
    check("rojo a la izquierda: el lateral sale negativo",
          esc.pilares and esc.pilares[0].lat_mm < 0,
          str(esc.pilares[0].lat_mm if esc.pilares else None))
    check("rojo a la izquierda: el punto de paso sigue a SU derecha",
          m.objetivo_mm > esc.pilares[0].lat_mm,
          f"objetivo={m.objetivo_mm:.0f} pilar={esc.pilares[0].lat_mm:.0f}")

    # --- una aproximacion normal no es "lado incorrecto" ---------------
    # Un pilar de frente a 1.2 m es una aproximacion, no un error. Si se
    # marcaba como lado incorrecto, la FSM bajaba a 22 % en cada pilar.
    _, m = correr(ROJO, dist=1200.0)
    # --- ALCANCE: hasta donde se ven de verdad los pilares -------------
    # `activar_desde_mm` no manda aqui: el tope real lo pone `area_min`, y
    # los dos se habian quedado descoordinados. Un pilar de 50x100 mm ocupa
    # del orden de 8.5e8/d^2 px, asi que con area_min 320 el carro era ciego
    # mas alla de ~1.6 m aunque el JSON dijera otra cosa — y no se puede
    # tener la trayectoria decidida para un pilar que aun no se ve.
    def alcanza(dist_mm, area_min):
        cfg = copy.deepcopy(DEFECTOS["colores"])
        cfg["rojo"]["area_min"] = area_min
        geo = Geometria(dict(DEFECTOS["geometria"]), 640, 480)
        det = Detector(cfg, geo)
        esc = det.procesar(_frame_con_pilar(dist_mm, 0.0, (60, 60, 230)))
        return esc.pilar_mas_cercano() is not None

    check("con area_min 320 el carro era ciego mas alla de 1.6 m",
          alcanza(1600, 320) and not alcanza(2000, 320))
    area = int(DEFECTOS["colores"]["rojo"]["area_min"])
    check(f"con area_min {area} se ve a 2.4 m", alcanza(2400, area),
          f"area_min={area}")
    activar = float(DEFECTOS["senales"]["activar_desde_mm"])
    check("y el alcance de la vision cubre activar_desde_mm",
          alcanza(activar, area), f"activar={activar} area_min={area}")

    check("pilar lejano no dispara CORRECCION", not m.lado_incorrecto,
          f"fase={m.fase}")


# ================================================ seguimiento de carril
def _escena_pista(muro_izq_px=None, muro_der_px=None, fondo_px=150):
    """Fotograma sintetico de pista: tapete blanco y muros negros.

    muro_izq_px / muro_der_px: hasta que columna llega cada muro lateral.
    None = ese lado no se ve (caso normal dentro de una curva).
    """
    import cv2
    import numpy as np
    img = np.full((480, 640, 3), 235, np.uint8)
    cv2.rectangle(img, (0, 0), (640, fondo_px), (30, 30, 30), -1)
    if muro_izq_px:
        cv2.rectangle(img, (0, fondo_px), (muro_izq_px, 430), (30, 30, 30), -1)
    if muro_der_px:
        cv2.rectangle(img, (muro_der_px, fondo_px), (640, 430), (30, 30, 30), -1)
    return img


def _escena_corredor(lat_izq_mm=500.0, lat_der_mm=500.0, frente_mm=6000.0,
                     geo=None):
    """Fotograma sintetico con la geometria DE VERDAD, en milimetros.

    POR QUE HACE FALTA OTRO GENERADOR TENIENDO _escena_pista. Porque el otro
    dibuja los muros en pixeles, como rectangulos que llegan hasta la fila
    430, y eso corresponde a un muro cuya base pasa a ~150 mm del carro: no es
    un carril de 1000 mm, es un pasillo por el que el carro no cabria. Sirve
    para preguntar "de que lado esta el hueco", que es para lo que se
    escribio, pero no para nada que dependa de la separacion lateral en
    milimetros — y el centrado, la guardia anti-muro y el aviso de muro encima
    dependen exactamente de eso.

    Aqui cada muro lateral es la recta x = +-L proyectada punto a punto con la
    misma geometria que usa el piloto, asi que "500 mm" significa 500 mm.
    """
    import cv2
    import numpy as np
    from src.config import DEFECTOS
    from src.geometria import Geometria
    geo = geo or Geometria(dict(DEFECTOS["geometria"]), 640, 480)
    img = np.full((480, 640, 3), 235, np.uint8)
    hz = geo.fila_horizonte()
    for lat, signo in ((lat_izq_mm, -1), (lat_der_mm, +1)):
        if lat is None:
            continue
        for y in np.arange(80.0, min(frente_mm, 6000.0), 4.0):
            u, v = geo.suelo_a_pixel(signo * lat, float(y))
            if 0 <= u < 640 and hz < v < 480:
                cv2.line(img, (u, v), (u, max(hz, v - 400)), (30, 30, 30), 2)
    if frente_mm < 6000.0:
        v = geo.distancia_a_fila(frente_mm)
        cv2.rectangle(img, (0, max(hz, v - 400)), (640, v), (30, 30, 30), -1)
    return img


def prueba_carril() -> None:
    print("seguimiento de carril")
    from src.config import DEFECTOS
    from src.carril import SeguidorCarril
    from src.geometria import Geometria
    from src.vision import Detector

    def conducir(img, repeticiones=6, **kw):
        """Varios ciclos: el mando lleva suavizado, un solo frame no asienta."""
        geo = Geometria(dict(DEFECTOS["geometria"]), 640, 480)
        det = Detector(DEFECTOS["colores"], geo)
        seg = SeguidorCarril(dict(DEFECTOS["carril"]))
        esc = det.procesar(img)
        for _ in range(repeticiones):
            s = seg.paso(esc, **kw)
        return s

    # --- EL FALLO DE PISTA: curva con la salida a un lado --------------
    # El muro cierra el frente y el hueco queda claramente a un lado. Antes
    # esto daba +-7 % de volante y el carro se iba recto contra la pared.
    s = conducir(_escena_pista(muro_der_px=300))
    check("curva a la izquierda: la detecta como curva", s.en_curva,
          f"frente={s.dist_frente_mm:.0f}")
    check("curva a la izquierda: el hueco cae a la izquierda",
          s.rumbo_hueco_deg < -5, f"rumbo={s.rumbo_hueco_deg:.1f} deg")
    check("curva a la izquierda: gira fuerte a la izquierda",
          s.direccion < -40, f"dir={s.direccion:.1f} %")

    s = conducir(_escena_pista(muro_izq_px=340))
    check("curva a la derecha: el hueco cae a la derecha",
          s.rumbo_hueco_deg > 5, f"rumbo={s.rumbo_hueco_deg:.1f} deg")
    check("curva a la derecha: gira fuerte a la derecha",
          s.direccion > 40, f"dir={s.direccion:.1f} %")

    # --- recta centrada: no deberia pedir casi nada --------------------
    s = conducir(_escena_pista(muro_izq_px=90, muro_der_px=550, fondo_px=120))
    check("recta centrada: apenas toca el volante", abs(s.direccion) < 22,
          f"dir={s.direccion:.1f} %")

    # --- recta descentrada: corrige hacia el lado libre ----------------
    # Pegado al muro izquierdo (ocupa mucho mas cuadro) -> debe irse a la
    # derecha. Esto es lo que el centrado viejo medía mal: comparaba
    # distancias HACIA DELANTE en vez de separacion lateral.
    s = conducir(_escena_pista(muro_izq_px=250, muro_der_px=600, fondo_px=120))
    check("pegado al muro izquierdo: corrige a la derecha", s.direccion > 8,
          f"dir={s.direccion:.1f} %  izq={s.lat_izq_mm}  der={s.lat_der_mm}")
    s = conducir(_escena_pista(muro_izq_px=40, muro_der_px=390, fondo_px=120))
    check("pegado al muro derecho: corrige a la izquierda", s.direccion < -8,
          f"dir={s.direccion:.1f} %  izq={s.lat_izq_mm}  der={s.lat_der_mm}")

    # --- un muro DE FRENTE no es un muro al costado --------------------
    # Carro centrado en un carril de 1000 mm con una esquina a 700 mm. Antes
    # el muro frontal se colaba en la medida lateral —ocupa todos los rumbos,
    # y su sector a 4 grados del morro da x = 700*sen(4) = 49 mm— y el carro
    # creia tener las dos paredes a 46 mm en CADA esquina. Con eso el centrado
    # respondia a la esquina en vez de a los costados y la guardia anti-muro
    # empujaba sin tener nada al lado.
    esquina = _escena_corredor(500.0, 500.0, 700.0)
    s = conducir(esquina)
    check("esquina de frente: no se inventa muros al costado",
          s.lat_izq_mm is None and s.lat_der_mm is None,
          f"izq={s.lat_izq_mm} der={s.lat_der_mm}")
    check("esquina de frente: no cree tener el muro encima",
          not s.muro_encima)
    recta_mm = conducir(_escena_corredor(500.0, 500.0))
    check("recta de 1000 mm: mide ~400 mm a cada lado",
          recta_mm.lat_izq_mm is not None and 330 < recta_mm.lat_izq_mm < 470,
          f"izq={recta_mm.lat_izq_mm} der={recta_mm.lat_der_mm}")

    # --- la pista del sentido sirve cuando el hueco no decide ----------
    s0 = conducir(esquina)
    s_h = conducir(esquina, sentido_pista=+1)     # horario -> curvas derecha
    s_a = conducir(esquina, sentido_pista=-1)     # antihorario -> izquierda
    check("hueco indeciso sin sentido: no se inventa un giro",
          abs(s0.direccion) < abs(s_h.direccion),
          f"sin={s0.direccion:.1f} con={s_h.direccion:.1f}")
    check("horario: la esquina indecisa se resuelve a la derecha",
          s_h.direccion > 25, f"dir={s_h.direccion:.1f} %")
    check("antihorario: se resuelve a la izquierda",
          s_a.direccion < -25, f"dir={s_a.direccion:.1f} %")

    # --- EL FALLO DE PISTA: el giro sin sentido justo tras rebasar -------
    # El carro acaba de rebasar un pilar por la DERECHA y queda a 150 mm del
    # muro derecho, apuntandolo. El frente esta cerrado, pero no por una
    # esquina. En una ronda horaria el sesgo de curva empujaba ademas hacia la
    # derecha, o sea HACIA la pared: medido, el volante se quedaba en -12 %
    # —un empujon simbolico— y el carro terminaba raspando el muro.
    tras = _escena_corredor(850.0, 150.0, 600.0)
    s_mal = conducir(tras, sentido_pista=+1)
    check("tras rebasar: se da cuenta de que tiene el muro encima",
          s_mal.muro_encima and s_mal.lat_der_mm is not None and
          s_mal.lat_der_mm < 240, f"der={s_mal.lat_der_mm}")
    check("tras rebasar: el sesgo NO empuja hacia el muro",
          s_mal.sesgo == 0.0, f"sesgo={s_mal.sesgo}")
    check("tras rebasar: se aparta de verdad del muro derecho",
          s_mal.direccion < -40, f"dir={s_mal.direccion:.1f} %")

    # Y con tras_pilar el sesgo se calla aunque hubiera sitio: ahi el frente
    # cerrado es el muro al que se apunta, no una curva que anticipar.
    s_tp = conducir(tras, sentido_pista=+1, tras_pilar=True)
    check("tras_pilar deja mudo al sesgo de curva", s_tp.sesgo == 0.0,
          f"sesgo={s_tp.sesgo}")

    # --- con TCS vivo, el sesgo ESPERA a la linea ----------------------
    # La linea del piso cae donde de verdad empieza la curva; la camara ve el
    # muro un metro antes. Girar con la camara es recortar, y el carro llegaba
    # a la linea ya torcido. Con el sensor vivo el sesgo es solo la red.
    s_espera = conducir(esquina, sentido_pista=+1, espera_linea=True)
    check("con TCS, a 700 mm de la esquina todavia va recto",
          s_espera.sesgo == 0.0 and "linea" in s_espera.motivo,
          f"sesgo={s_espera.sesgo} [{s_espera.motivo}]")
    # Pero si la linea no llega y el frente se cierra de verdad, habla igual:
    # mas vale girar tarde que empotrarse por esperar un sensor que fallo.
    apurado = _escena_corredor(500.0, 500.0, 420.0)
    s_apurado = conducir(apurado, sentido_pista=+1, espera_linea=True)
    check("si la linea no llega y el frente se cierra, el sesgo salta igual",
          s_apurado.sesgo > 0.0, f"sesgo={s_apurado.sesgo}")
    # Y sin TCS no hay nada que esperar: manda desde dist_curva_mm, como antes.
    s_sin_tcs = conducir(esquina, sentido_pista=+1, espera_linea=False)
    check("sin TCS el sesgo vuelve a mandar desde lejos",
          s_sin_tcs.sesgo > 0.0, f"sesgo={s_sin_tcs.sesgo}")

    # --- el sentido de las curvas se aprende solo ----------------------
    from src.geometria import Geometria as G
    geo = G(dict(DEFECTOS["geometria"]), 640, 480)
    det = Detector(DEFECTOS["colores"], geo)
    seg = SeguidorCarril(dict(DEFECTOS["carril"]))
    curva = det.procesar(_escena_pista(muro_izq_px=340))       # hueco derecha
    recta_esc = det.procesar(_escena_pista(muro_izq_px=90, muro_der_px=550,
                                           fondo_px=120))
    check("antes de la primera curva no sabe el sentido",
          seg.sentido_curva == 0)
    for _ in range(10):
        seg.paso(curva)
    seg.paso(recta_esc)                            # salir de la curva
    check("tras una curva a la derecha, aprende que giran a la derecha",
          seg.sentido_curva == 1, str(seg.sentido_curva))

    # PERO NO APRENDE DE UN ESQUIVE. Si los ciclos de despues de rebasar
    # entraran en el promedio, el sentido "aprendido" seria el del ultimo
    # esquive, y a partir de ahi el sesgo empujaria hacia ese lado en TODAS
    # las esquinas de la ronda: una vuelta perdida por un dato sucio.
    seg2 = SeguidorCarril(dict(DEFECTOS["carril"]))
    for _ in range(10):
        seg2.paso(curva, tras_pilar=True)
    seg2.paso(recta_esc, tras_pilar=True)
    check("un esquive no ensena el sentido de las curvas",
          seg2.sentido_curva == 0, str(seg2.sentido_curva))
    # Y despues, con datos limpios, sigue aprendiendo igual.
    for _ in range(10):
        seg2.paso(curva)
    seg2.paso(recta_esc)
    check("con datos limpios vuelve a aprender", seg2.sentido_curva == 1,
          str(seg2.sentido_curva))

    # --- el gyro amortigua, no manda ----------------------------------
    quieto = conducir(_escena_pista(muro_der_px=300), gz=0.0)
    girando = conducir(_escena_pista(muro_der_px=300), gz=-60.0)
    check("girando ya a la izquierda, pide menos volante",
          abs(girando.direccion) < abs(quieto.direccion),
          f"{girando.direccion:.1f} vs {quieto.direccion:.1f}")


# ================================ compromiso: mantener rumbo, no volante
def prueba_compromiso() -> None:
    print("compromiso de adelantamiento")
    from src.config import DEFECTOS
    from src.carril import SalidaCarril, guardia_muro
    from src.senales import FASE_COMPROMISO, Esquivador
    from src.vision import Deteccion, Escena

    def escena(color, dist, lat=0.0):
        e = Escena()
        e.pilares = [Deteccion(color=color, x=300, y=200, w=40, h=90,
                               area=3600, dist_mm=dist, lat_mm=lat,
                               confianza=1.0)]
        return e

    def armar(yaw=0.0):
        """Aproxima a un pilar verde hasta que arranca el compromiso."""
        esq = Esquivador(dict(DEFECTOS["senales"]), 130.0)
        for d in (900.0, 700.0, 500.0, 380.0):
            m = esq.paso(escena("verde", d), {}, False, 900.0, yaw=yaw)
        return esq, m

    esq, m = armar()
    check("al acercarse, gira a la izquierda por el verde", m.direccion < -10,
          f"dir={m.direccion:.1f}")
    volante_al_armar = m.direccion

    # Ya no se ve el pilar: empieza el compromiso. MISMO rumbo que al armar.
    m0 = esq.paso(Escena(), {}, False, 900.0, yaw=0.0)
    check("sin pilar visible entra en compromiso", m0.fase == FASE_COMPROMISO,
          m0.fase)
    # ESTE es el fallo que se vio en pista: antes se congelaba el volante y el
    # carro seguia trazando el arco hacia el lado del pilar.
    check("en rumbo, el volante se suelta (no sigue el arco)",
          abs(m0.direccion) < abs(volante_al_armar) * 0.35,
          f"al armar {volante_al_armar:.1f} -> ahora {m0.direccion:.1f}")

    # Si el carro se ha desviado a la izquierda del rumbo guardado, tiene que
    # corregir a la DERECHA para volver a el.
    esq, _ = armar()
    esq.paso(Escena(), {}, False, 900.0, yaw=0.0)
    m_izq = esq.paso(Escena(), {}, False, 900.0, yaw=-12.0)
    check("desviado a la izquierda: corrige a la derecha", m_izq.direccion > 0,
          f"dir={m_izq.direccion:.1f}  err={m_izq.err_rumbo_deg:.1f} deg")

    esq, _ = armar()
    esq.paso(Escena(), {}, False, 900.0, yaw=0.0)
    m_der = esq.paso(Escena(), {}, False, 900.0, yaw=+12.0)
    check("desviado a la derecha: corrige a la izquierda", m_der.direccion < 0,
          f"dir={m_der.direccion:.1f}")

    # El envoltorio de +-180: cruzar el limite no puede dar un volantazo.
    esq, _ = armar(yaw=179.0)
    esq.paso(Escena(), {}, False, 900.0, yaw=179.0)
    m_env = esq.paso(Escena(), {}, False, 900.0, yaw=-179.0)
    check("cruzar +-180 no dispara un volantazo", abs(m_env.direccion) < 15,
          f"dir={m_env.direccion:.1f}  err={m_env.err_rumbo_deg:.1f} deg")

    # --- salida del compromiso: devolver el carro ALINEADO --------------
    # Sostener el rumbo congelado hasta el ultimo instante resuelve el
    # adelantamiento y crea el problema siguiente: ese rumbo apunta hacia el
    # lado por el que se rebaso, asi que el carril recibe un carro cruzado y
    # con el muro delante, y lo que hace entonces es el giro brusco que se
    # veia en pista. Pasada la mitad del compromiso, el objetivo se lleva al
    # rumbo del PASILLO, que aqui esta 20 grados a la izquierda.
    import time as _tt
    t0 = _tt.time()
    esq = Esquivador(dict(DEFECTOS["senales"]), 130.0)
    for i, d in enumerate((900.0, 700.0, 500.0, 380.0)):
        esq.paso(escena("verde", d), {}, False, 900.0, yaw=0.0,
                 ahora=t0 + i * 0.03)
    # dur = (380 + 200 + 120) / 900 = 0.78 s. A 0.1 s (12 %) aun se sostiene
    # el rumbo congelado; a 0.7 s (90 %) ya se mira al pasillo.
    pronto = esq.paso(Escena(), {}, False, 900.0, yaw=0.0,
                      rumbo_hueco_deg=-20.0, ahora=t0 + 0.12 + 0.1)
    tarde = esq.paso(Escena(), {}, False, 900.0, yaw=0.0,
                     rumbo_hueco_deg=-20.0, ahora=t0 + 0.12 + 0.70)
    check("al principio del compromiso manda el rumbo congelado",
          not pronto.saliendo and abs(pronto.err_rumbo_deg) < 2.0,
          f"err={pronto.err_rumbo_deg:.1f} saliendo={pronto.saliendo}")
    check("al final del compromiso se reenfila hacia el pasillo",
          tarde.saliendo and tarde.err_rumbo_deg < -12.0,
          f"err={tarde.err_rumbo_deg:.1f} saliendo={tarde.saliendo}")
    check("y eso se traduce en volante hacia el pasillo, no hacia el muro",
          tarde.direccion < -20.0, f"dir={tarde.direccion:.1f}")

    # Sin MPU no hay rumbo que sostener: se ENDEREZA, y rapido.
    #
    # ESTA PRUEBA ESTABA ROTA y tapaba el fallo. Llamaba tres veces seguidas
    # al esquivador y comparaba la primera con la ultima; como entre las tres
    # llamadas pasan microsegundos y el desvanecido iba por fraccion del
    # compromiso, las tres daban 69.3 y la prueba fallaba sin explicar por
    # que. Con el reloj inyectado se mide lo que de verdad importa: cuanto
    # tarda el volante en volver a cero. Un volante fijo no traza una recta,
    # traza un arco — y ese arco es el giro contra la pared.
    esq = Esquivador(dict(DEFECTOS["senales"]), 130.0)
    for i, d in enumerate((900.0, 700.0, 500.0, 380.0)):
        m = esq.paso(escena("verde", d), {}, False, 900.0, yaw=None,
                     ahora=t0 + i * 0.03)
    al_armar = abs(m.direccion)
    sin_mpu = [abs(esq.paso(Escena(), {}, False, 900.0, yaw=None,
                            ahora=t0 + 0.12 + t).direccion)
               for t in (0.05, 0.20, 0.40)]
    check("sin MPU el volante decae en vez de congelarse",
          al_armar > sin_mpu[0] > sin_mpu[1] > sin_mpu[2],
          f"al armar {al_armar:.1f} -> {[round(v, 1) for v in sin_mpu]}")
    check("sin MPU las ruedas quedan rectas antes de medio segundo",
          sin_mpu[-1] < 1e-6, f"{sin_mpu[-1]:.1f}")

    # --- guardia anti-muro --------------------------------------------
    cfg = DEFECTOS["carril"]
    libre = SalidaCarril(lat_izq_mm=500.0, lat_der_mm=500.0)
    check("con sitio de sobra, la guardia calla",
          abs(guardia_muro(libre, cfg)) < 1e-6)
    pegado_izq = SalidaCarril(lat_izq_mm=90.0, lat_der_mm=700.0)
    check("muro izquierdo encima: empuja a la derecha",
          guardia_muro(pegado_izq, cfg) > 20,
          f"{guardia_muro(pegado_izq, cfg):.1f}")
    pegado_der = SalidaCarril(lat_izq_mm=700.0, lat_der_mm=90.0)
    check("muro derecho encima: empuja a la izquierda",
          guardia_muro(pegado_der, cfg) < -20,
          f"{guardia_muro(pegado_der, cfg):.1f}")
    check("un solo muro visible y lejos: tampoco molesta",
          abs(guardia_muro(SalidaCarril(lat_der_mm=600.0), cfg)) < 1e-6)


# ============================== el muestreo del sensor de color ==========
def prueba_muestreo_tcs() -> None:
    """El hueco de 48 ms en el que cabe una linea entera.

    ESTO NO PRUEBA EL FIRMWARE —esta en C++ y aqui no hay compilador— sino la
    ARITMETICA DE LA PLANIFICACION, que es donde estaba el fallo y que es la
    misma se escriba en el lenguaje que se escriba. El chip integra durante
    T y el codigo preguntaba cada T. Dos relojes a la misma frecuencia sin
    sincronizar derivan, y en cuanto derivan la pregunta cae siempre un pelo
    antes de que el dato exista; como el hueco hasta la pregunta siguiente ya
    estaba reservado, el periodo real pasa a ser 2T.
    """
    print("muestreo del sensor de color")

    def simular(t_integracion_ms, periodo_sondeo_ms, reservar_hueco,
                deriva_ms=0.7, ms_total=4000.0):
        """Devuelve los instantes en que el codigo consigue una muestra nueva.

        deriva_ms: lo que el reloj del chip se adelanta cada integracion
        respecto al del micro. Con relojes RC de verdad esto es inevitable.
        """
        listo_en = t_integracion_ms          # cuando el chip tendra el dato
        prox = 0.0                           # cuando el codigo preguntara
        muestras = []
        t = 0.0
        while t < ms_total:
            t = prox
            if t >= listo_en:                # hay dato: se coge
                muestras.append(t)
                listo_en = t + t_integracion_ms + deriva_ms
                prox = t + periodo_sondeo_ms
            else:                            # todavia no
                # AQUI ESTA EL FALLO: reservar el hueco entero aunque no
                # hubiera dato es lo que dobla el periodo.
                prox = t + (periodo_sondeo_ms if reservar_hueco else 1.0)
        return muestras

    def peor_hueco(m):
        return max(b - a for a, b in zip(m, m[1:])) if len(m) > 1 else 1e9

    # El caso de antes: integra 24 ms y se pregunta cada 24 ms.
    antes = simular(24.0, 24.0, reservar_hueco=True)
    # El de ahora: integra 12 ms y se sondea a 500 Hz (cada 2 ms).
    ahora = simular(12.0, 2.0, reservar_hueco=False)

    check("antes: el hueco entre muestras llegaba a doblarse",
          peor_hueco(antes) > 40.0, f"peor hueco {peor_hueco(antes):.0f} ms")
    check("ahora: el hueco no pasa del periodo de integracion + margen",
          peor_hueco(ahora) < 20.0, f"peor hueco {peor_hueco(ahora):.0f} ms")

    # Lo que importa de verdad: cuantas lineas se pierden. Una linea de 20 mm
    # a 1 m/s dura 20 ms (reglamento 13.9); se cruza una cada 2 m de pista.
    def lineas_vistas(muestras, dura_ms=20.0, cada_ms=2000.0, n=8):
        vistas = 0
        for i in range(n):
            t0 = 120.0 + i * cada_ms
            if any(t0 <= m <= t0 + dura_ms for m in muestras):
                vistas += 1
        return vistas

    largo_antes = simular(24.0, 24.0, True, ms_total=18000.0)
    largo_ahora = simular(12.0, 2.0, False, ms_total=18000.0)
    v_antes = lineas_vistas(largo_antes)
    v_ahora = lineas_vistas(largo_ahora)
    check("antes se perdian lineas enteras", v_antes < 8,
          f"{v_antes} de 8 lineas vistas")
    check("ahora no se pierde ninguna", v_ahora == 8,
          f"{v_ahora} de 8 lineas vistas")
    # Y con dos muestras por linea el filtro de permanencia puede confirmarla.
    def muestras_por_linea(m, dura_ms=20.0, cada_ms=2000.0, n=8):
        return min(sum(1 for x in m if t0 <= x <= t0 + dura_ms)
                   for t0 in (120.0 + i * cada_ms for i in range(n)))
    # UNA muestra dentro de la linea tiene que bastar para contarla: el
    # clasificador cuenta milisegundos de permanencia, no lecturas, y una
    # lectura de 12 ms ES luz recogida durante 12 ms. Antes hacian falta dos
    # seguidas, y el dia que solo caia una la linea no se contaba.
    check("en la peor linea cae al menos una muestra",
          muestras_por_linea(largo_ahora) >= 1,
          f"{muestras_por_linea(largo_ahora)} muestras en la peor linea")
    check("y esa muestra cubre de sobra la permanencia minima (8 ms)",
          peor_hueco(largo_ahora) >= 8.0,
          f"periodo real {peor_hueco(largo_ahora):.0f} ms")


# ====================== vueltas fantasma (el fallo del boton)
def prueba_vueltas_fantasma() -> None:
    print("vueltas fantasma")
    from src.config import DEFECTOS
    cfg = DEFECTOS["vueltas"]

    def S(nar, azu=0):
        return proto.Sensores(estado=proto.S_TCS_OK, cruces_naranja=nar,
                              cruces_azul=azu)

    # EL FALLO DE PISTA. La Pi toma su linea base con el contador del ESP32 en
    # 200 y un instante despues llega el CAL_CERO_LINEAS: 200 -> 0. La resta
    # de 8 bits da 56, y el codigo se creia 56 cruces = 14 vueltas, con el
    # carro quieto. La ronda terminaba nada mas pulsar el boton.
    c = Contador(cfg)
    c.actualizar(S(200), 0.0, 0.03)
    c.actualizar(S(0), 0.0, 0.03)
    check("un reinicio del contador no inventa cruces",
          c.e.cruces_totales == 0, f"cruces={c.e.cruces_totales}")
    check("un reinicio del contador no inventa vueltas",
          c.e.vueltas == 0, f"vueltas={c.e.vueltas}")
    check("el reinicio queda anotado como resync",
          c.e.info.get("resync", 0) >= 1, str(c.e.info))
    # Y despues del salto sigue contando normal desde la nueva referencia.
    c.actualizar(S(1), 900.0, 1.0)
    check("tras resincronizar vuelve a contar bien", c.e.cruces_totales == 1,
          f"cruces={c.e.cruces_totales}")

    # Un envoltorio LEGITIMO de 8 bits (254 -> 1 son 3 cruces) debe pasar.
    c2 = Contador(cfg)
    c2.actualizar(S(254), 0.0, 0.03)
    c2.actualizar(S(1), 0.0, 0.03)
    check("el envoltorio legitimo de 8 bits si cuenta",
          c2.e.cruces_totales == 3, f"cruces={c2.e.cruces_totales}")

    # En ESPERA no se cuenta nada, aunque llegue ruido.
    c3 = Contador(cfg)
    c3.actualizar(S(0), 0.0, 0.03, contar=False)
    for n in range(1, 6):
        c3.actualizar(S(n), 500.0, 0.5, contar=False)
    check("antes de arrancar no se cuentan cruces", c3.e.cruces_totales == 0,
          f"cruces={c3.e.cruces_totales}")
    check("antes de arrancar el odometro no avanza", c3.e.dist_mm == 0.0,
          f"dist={c3.e.dist_mm}")
    # Y al empezar a contar, el primer cruce real si entra.
    c3.actualizar(S(6), 900.0, 1.0, contar=True)
    check("al arrancar, el primer cruce real si cuenta",
          c3.e.cruces_totales == 1, f"cruces={c3.e.cruces_totales}")

    # Suelo de distancia: tres vueltas no caben en 0.3 m.
    c4 = Contador(cfg)
    c4.actualizar(S(0), 0.0, 0.03)
    n = 0
    for _ in range(3):
        for _ in range(4):
            n += 1
            c4.actualizar(S(n), 100.0, 0.1)
    check("con 3 vueltas contadas pero sin kilometraje, NO para",
          not c4.evaluar_parada(3).listo_para_parar,
          f"{c4.e.vueltas} vueltas, {c4.e.dist_mm/1000:.1f} m: {c4.e.motivo}")


def main() -> int:
    prueba_protocolo()
    prueba_reglas()
    prueba_geometria()
    prueba_vueltas()
    prueba_fsm()
    prueba_esquina()
    prueba_esquive()
    prueba_carril()
    prueba_compromiso()
    prueba_muestreo_tcs()
    prueba_vueltas_fantasma()
    print()
    if fallos:
        print(f"{fallos} prueba(s) FALLARON")
        return 1
    print("todo correcto")
    return 0


if __name__ == "__main__":
    sys.exit(main())
