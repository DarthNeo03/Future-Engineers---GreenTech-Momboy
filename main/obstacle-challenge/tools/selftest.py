#!/usr/bin/env python3
"""
selftest.py — Pruebas del piloto SIN carro, SIN camara y SIN ESP32.

    python tools/selftest.py

Cubre lo que puede contar mal o romper hardware: el protocolo (con vectores
fijos que el firmware C++ debe reproducir), el lector con ruido, la geometria
(ida y vuelta), el perfil del muro sobre imagenes sinteticas (incluido el caso
"pared con brillo" que rompia el programa viejo), el conteo de lineas/esquinas
/sentido, la maquina de navegacion y los perfiles de parametros.
"""

from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

from src import protocolo as P          # noqa: E402
from src import params as params_mod    # noqa: E402
from src.geometria import Geometria     # noqa: E402
from src import muro                    # noqa: E402
from src.lineas import (GestorLineas, HORARIO, ANTIHORARIO,          # noqa: E402
                        clase_tcs, umbrales_desde_muestra)
from src.botones import Pulsador, CORTA, LARGA                     # noqa: E402
from src.navegacion import (Navegador, RECTO, PRE_GIRO, GIRO,      # noqa: E402
                            ESCAPE, RESCATE)
from src.obstaculos import (Esquivador, Maniobra, LADO,            # noqa: E402
                            LIBRE, SIGUIENDO, ADELANTANDO)
from src.vision import Deteccion                                   # noqa: E402

FALLOS = []
TOTAL = 0


def prueba(nombre: str, cond: bool, extra: str = ""):
    global TOTAL
    TOTAL += 1
    if cond:
        print(f"  ok  {nombre}")
    else:
        print(f"FALLA  {nombre}  {extra}")
        FALLOS.append(nombre)


# ===========================================================================
print("== protocolo ==")
m = P.Mando(seq=7, vel=-42, direccion=88, vmax=200, armado=True)
tr = m.a_bytes()
prueba("mando: 11 bytes", len(tr) == 11, f"{len(tr)}")
# Vector fijo: si cambia, el firmware y esto se separaron.
prueba("mando: bytes exactos",
       tr.hex() == "a55a06010701d658c80081",
       tr.hex())
m2 = P.Mando.desde_payload(tr[4:10])
prueba("mando: ida y vuelta", (m2.vel, m2.direccion, m2.vmax, m2.armado) ==
       (-42, 88, 200, True))

s = P.Sensores(yaw_deci=-1234, gz_deci=567, c=1000, r=400, g=300, b=250,
               estado=P.S_MPU_OK | P.S_TCS_OK | (P.LINEA_AZUL << 6),
               cnt_lineas=0x53)
tr = s.a_bytes()
prueba("sensores: 20 bytes (version 3, con el byte de botones)",
       len(tr) == 20, f"{len(tr)}")
s2 = P.Sensores.desde_payload(tr[4:19])
prueba("sensores: ida y vuelta",
       (s2.yaw_deci, s2.gz_deci, s2.c, s2.cnt_naranja, s2.cnt_azul) ==
       (-1234, 567, 1000, 3, 5))
prueba("sensores: clase linea", s2.clase_linea == P.LINEA_AZUL)
prueba("sensores: yaw en grados", abs(s2.yaw + 123.4) < 1e-6)

cfg = P.empaquetar_cfg_tcs(40, 110, 90, 95, 95, 1, 3, 251, 2, 30, 18, 55)
prueba("cfg_tcs: 18 bytes (con los discriminadores por diferencia)",
       len(cfg) == 18, f"{len(cfg)}")

lector = P.Lector()
basura = b"\x00\xa5" + tr + b"\xff\xa5\x5a" + m.a_bytes()
resultado = lector.alimentar(basura)
tipos = [t for t, _ in resultado]
prueba("lector: recupera tramas entre basura",
       P.TIPO_SENSORES in tipos and P.TIPO_MANDO in tipos, str(tipos))

trunca = m.a_bytes()[:6] + m.a_bytes()
lector2 = P.Lector()
res2 = lector2.alimentar(trunca)
prueba("lector: trama truncada no se traga la siguiente",
       len(res2) == 1 and res2[0][0] == P.TIPO_MANDO)

# contadores mod 16: perder tramas no pierde cruces
prueba("contador de lineas envuelve", ((3 - 14) & 0x0F) == 5)

# ===========================================================================
print("== geometria ==")
gcfg = {"alto_cam_mm": 125.0, "inclinacion_deg": 7.5, "fy_px": 460.0,
        "fx_px": 460.0, "ancho_carro_mm": 200.0, "margen_ruedas_mm": 30.0,
        "morro_mm": 60.0}
geo = Geometria(gcfg, 640, 480)
y_h = geo.fila_horizonte()
prueba("horizonte por encima del centro", 150 < y_h < 240, str(y_h))
for d in (300.0, 800.0, 2000.0):
    v = geo.distancia_a_fila(d)
    d2 = float(geo.fila_a_distancia(v))
    prueba(f"fila<->distancia {d:.0f}mm", abs(d - d2) < d * 0.08, f"{d2:.0f}")
prueba("mas abajo = mas cerca",
       geo.fila_a_distancia(470) < geo.fila_a_distancia(300))
# calibracion de fy: un objeto a 500mm clicado donde el modelo lo pondria
v500 = geo.distancia_a_fila(500.0)
fy = geo.calibrar_fy(v500, 500.0)
prueba("calibrar_fy recupera fy", abs(fy - 460.0) < 12.0, f"{fy:.0f}")
x_lat, y_ade = geo.punto_suelo(500, 400)
prueba("lateral positivo a la derecha", x_lat > 0)
u, v = geo.suelo_a_pixel(x_lat, y_ade)
prueba("suelo<->pixel", abs(u - 500) <= 2 and abs(v - 400) <= 2, f"{u},{v}")

# ===========================================================================
print("== perfil del muro ==")
H, W = 480, 640
mcfg = {"metodo": "piso", "alcance_mm": 2500.0, "k_transicion": 6,
        "margen_horizonte_px": 4, "ignorar_abajo": 0.05, "suavizado": 7,
        "banda_lateral": 0.28, "salto_borde_mm": 400.0,
        "seg_tolerancia_mm": 45.0, "seg_gap_max_mm": 350.0,
        "seg_angulo_fusion_deg": 12.0, "px_min_columna": 4}

def escena(fila_muro: int, con_brillo: bool = False,
           basura_arriba: bool = True):
    """Piso blanco hasta fila_muro; muro (no piso) encima; opcionalmente
    'brillo' (trozos de muro que parecen claros PERO no blancos => siguen sin
    ser piso) y basura oscura por encima del horizonte (sillas del publico)."""
    blanco = np.zeros((H, W), np.uint8)
    blanco[fila_muro:, :] = 255                    # piso debajo del contacto
    negro = np.zeros((H, W), np.uint8)
    negro[max(0, fila_muro - 60):fila_muro, :] = 255
    if con_brillo:
        # el brillo rompe la mascara NEGRA (el metodo viejo se pierde)...
        negro[:, 200:360] = 0
        # ...pero la zona sigue sin entrar en la mascara de PISO
    if basura_arriba:
        negro[0:100, :] = 255                      # sillas oscuras del fondo
    return {"blanco": blanco, "negro": negro}

p = muro.perfil(escena(300), geo, mcfg)
prueba("contacto en la fila correcta",
       abs(int(np.median(p.y_contacto[p.valido])) - 300) <= 2,
       str(int(np.median(p.y_contacto[p.valido]))))
prueba("todas las columnas ven muro", p.valido.all())
d_esperada = float(geo.fila_a_distancia(300)) - 60.0
prueba("distancia coherente",
       abs(p.pasillo_mm - d_esperada) < d_esperada * 0.15,
       f"{p.pasillo_mm:.0f} vs {d_esperada:.0f}")

p_brillo = muro.perfil(escena(300, con_brillo=True), geo, mcfg)
filas_brillo = p_brillo.y_contacto[220:340]
prueba("CON BRILLO el contacto no salta al fondo (metodo piso)",
       abs(int(np.median(filas_brillo)) - 300) <= 2,
       str(int(np.median(filas_brillo))))

### PISO POR MITADES: la medida que dice por donde se sale de un rincon.
### Muro pegado por la izquierda (poco piso) y lejos por la derecha (mucho).
esc_lado = escena(300)
esc_lado["blanco"][:, :W // 2] = 0
esc_lado["blanco"][440:, :W // 2] = 255          # solo un palmo de piso
p_lado = muro.perfil(esc_lado, geo, mcfg)
prueba("piso por mitades: gana el lado despejado",
       p_lado.piso_der > p_lado.piso_izq * 1.5,
       f"izq={p_lado.piso_izq:.2f} der={p_lado.piso_der:.2f}")

libre = {"blanco": np.full((H, W), 255, np.uint8),
         "negro": np.zeros((H, W), np.uint8)}
p_libre = muro.perfil(libre, geo, mcfg)
prueba("sin muro: pasillo = alcance", p_libre.pasillo_mm >= 2400)

# muro solo a la izquierda -> izq < der
mitad = escena(300)
mitad["blanco"][:, W // 2:] = 255                  # derecha: todo piso
p_mitad = muro.perfil(mitad, geo, mcfg)
prueba("muro a la izquierda: izq < der", p_mitad.izq < p_mitad.der - 0.2,
       f"{p_mitad.izq:.2f} {p_mitad.der:.2f}")
prueba("borde detectado en el cambio",
       any(abs(b[0] - W // 2) < 30 for b in p_mitad.bordes),
       str(p_mitad.bordes[:3]))

# segmentos: una pared frontal recta debe dar >=1 segmento casi horizontal
prueba("segmentos ajustados", len(p.segmentos) >= 1, str(len(p.segmentos)))

# ===========================================================================
print("== lineas / sentido / vueltas ==")
lcfg = {"naranja_es_horario": True, "usar_tcs": True, "usar_camara": True,
        "umbral_cruce_mm": 260.0, "cierre_max_ms": 2000,
        "refractario_esquina_ms": 400}
gl = GestorLineas(lcfg)
gl.evento_tcs("naranja")
prueba("primera linea naranja => horario", gl.sentido == HORARIO)
prueba("cuenta 1 esquina", gl.esquinas == 1)
gl.evento_tcs("azul")            # segunda linea del par: no cuenta otra
prueba("el par no cuenta doble", gl.esquinas == 1)
time.sleep(0.45)
gl.evento_tcs("naranja")
prueba("esquina siguiente tras refractario", gl.esquinas == 2)
prueba("vueltas", gl.vueltas(4) == 0)
for _ in range(2):
    time.sleep(0.45)
    gl.evento_tcs("naranja")
prueba("4 esquinas = 1 vuelta", gl.vueltas(4) == 1, str(gl.esquinas))

gl2 = GestorLineas(lcfg)
gl2.evento_tcs("azul")
prueba("primera azul => antihorario", gl2.sentido == ANTIHORARIO)

gl3 = GestorLineas(lcfg)
gl3.giro_completado(1)
prueba("giro derecha sin lineas => horario y 1 esquina",
       gl3.sentido == HORARIO and gl3.esquinas == 1)
gl3.evento_tcs("naranja")        # la linea llega justo despues del giro
prueba("giro+linea de la misma esquina no duplica", gl3.esquinas == 1)

prueba("sentido forzado gana", gl2.sentido_efectivo("horario") == HORARIO)

### EL GIRO FANTASMA DE LA AZUL TARDIA. La azul se detecta peor que la
### naranja y llegaba con el giro ya hecho; el codigo viejo la tomaba por la
### primera linea de una esquina NUEVA y sumaba una esquina de mentira.
gl4 = GestorLineas(lcfg)
gl4.evento_tcs("naranja")
time.sleep(0.6)                  # mas que el refractario, menos que el cierre
gl4.evento_tcs("azul")
prueba("azul tardia CIERRA la esquina, no abre otra", gl4.esquinas == 1,
       str(gl4.esquinas))

### ...y el reves: si la azul NO se ve nunca, dos naranjas seguidas tienen
### que ser dos esquinas. Si la espera de cierre callara tambien al mismo
### color, se contaria media pista.
gl5 = GestorLineas(lcfg)
gl5.evento_tcs("naranja")
time.sleep(0.45)
gl5.evento_tcs("naranja")
prueba("perdiendo siempre la azul, cada naranja es una esquina",
       gl5.esquinas == 2, str(gl5.esquinas))

### rebote del borde: la misma linea parpadea al pisarla
gl6 = GestorLineas(lcfg)
gl6.evento_tcs("naranja")
gl6.evento_tcs("naranja")
prueba("rebote de la misma linea no cuenta", gl6.esquinas == 1,
       str(gl6.esquinas))

# ===========================================================================
print("== clasificador del TCS (gemelo del firmware) ==")
### Lecturas REALES medidas en la pista del equipo. La azul es el caso que
### costo caro: solo saca 22 puntos al blanco en su propio canal (b=107 contra
### ~85), asi que con un umbral absoluto de 110 no se detectaba nunca; en la
### DIFERENCIA b-r saca 37 contra ~0. Y su canal claro es un 20 % del blanco,
### asi que un c_min sacado del 25 % del blanco la descartaba antes de mirar
### siquiera el color.
tcfg = {k: v["def"] for k, v in params_mod.ESQUEMA["tcs"].items()}
prueba("azul real de pista (C=678 R=186 B=284)",
       clase_tcs(678, 186, 200, 284, tcfg) == "azul",
       clase_tcs(678, 186, 200, 284, tcfg))
prueba("piso blanco no es linea",
       clase_tcs(3400, 1130, 1140, 1130, tcfg) == "-",
       clase_tcs(3400, 1130, 1140, 1130, tcfg))
prueba("naranja", clase_tcs(1800, 1100, 500, 200, tcfg) == "naranja",
       clase_tcs(1800, 1100, 500, 200, tcfg))
prueba("sombra (poco claro) no clasifica",
       clase_tcs(10, 4, 3, 3, tcfg) == "-")

### muestrear una linea NO puede subir el suelo de luz (si no, muestrear el
### naranja deja fuera a la azul, que es mas oscura)
u_az = umbrales_desde_muestra("azul", 70.0, 107.0, 678.0)
prueba("muestrear azul baja el discriminador, no lo inventa",
       8 <= u_az["azul_dif_min"] <= 25, str(u_az))

# ===========================================================================
print("== boton de competencia ==")
### Red de seguridad: un pulsador que YA esta pisado cuando el programa
### arranca (dedo apoyado, cable al reves, boton pegado) queda mudo hasta que
### se le ve suelto una vez. Si no, el carro saldria corriendo solo.
b = Pulsador(antirrebote_ms=50, largo_ms=300)
t0 = 100.0
prueba("un boton ya pisado al empezar queda mudo",
       b.paso(True, t0) == "" and b.paso(True, t0 + 1.0) == "" and
       b.paso(True, t0 + 1.4) == "")      # pasado el tiempo de 'largo'
b.paso(False, t0 + 1.5)
b.paso(False, t0 + 1.6)                    # se le ve suelto: ya vale
prueba("pulsacion corta al soltar",
       b.paso(True, t0 + 2.0) == "" and b.paso(True, t0 + 2.1) == "" and
       b.paso(False, t0 + 2.2) == "" and
       b.paso(False, t0 + 2.3) == CORTA)
b2 = Pulsador(antirrebote_ms=50, largo_ms=300)
b2.paso(False, t0)
b2.paso(True, t0 + 1.0)
b2.paso(True, t0 + 1.1)
prueba("pulsacion larga avisa sin soltar", b2.paso(True, t0 + 1.5) == LARGA)

# ===========================================================================
print("== navegacion ==")
ncfg = params_mod.valores_por_defecto()
nav = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], ncfg["rescate"])
p_ok = muro.perfil(escena(300), geo, mcfg)          # muro lejos-medio
d = nav.paso(p_libre, None, 0)
prueba("libre: avanza", d.vel > 0 and d.estado == RECTO, f"{d.vel} {d.estado}")

# muro encima -> escape en reversa
cerca = escena(int(H * 0.93))
p_cerca = muro.perfil(cerca, geo, mcfg)
nav2 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], ncfg["rescate"])
d2 = nav2.paso(p_cerca, None, 0)
prueba("muro encima: reversa", d2.vel < 0 and d2.estado == ESCAPE,
       f"{d2.vel} {d2.estado} pasillo={p_cerca.pasillo_mm:.0f}")

# muro a media distancia: sigue en recto pero mas despacio que en libre
nav3 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], ncfg["rescate"])
p_medio = muro.perfil(escena(260), geo, mcfg)
d3a = nav3.paso(p_libre, None, 0)
d3b = nav3.paso(p_medio, None, 0)
prueba("frena al acercarse", d3b.vel <= d3a.vel, f"{d3b.vel} vs {d3a.vel}")

# giro hacia el lado con mas espacio si el pasillo cae (sin sentido conocido)
nav4 = Navegador(dict(ncfg["navegacion"], min_recto_ms=0),
                 ncfg["limites"], ncfg["escape"], ncfg["rescate"])
d4 = nav4.paso(p_mitad, None, 0)   # pared a la izquierda
girado = None
if nav4.estado in ("pre_giro", "giro"):
    girado = nav4.lado_giro
    prueba("esquina: gira hacia el lado libre (derecha)", girado == 1,
           str(girado))
else:
    prueba("con a pared a un lado navega o gira", d4.vel != 0, d4.estado)

# ===========================================================================
print("== esquivar pilares (servo visual, como ANTi) ==")
MORRO = 60.0


def pilar(color: str, dist_mm: float, lat_mm: float):
    """Una deteccion puesta donde la geometria pondria un pilar de 50 mm a
    esa distancia y ese desplazamiento lateral, con el ancho en pixeles que
    tendria a esa distancia."""
    u, v = geo.suelo_a_pixel(lat_mm, dist_mm + MORRO)
    w = max(6, int(round(50.0 * geo._fx / (dist_mm + MORRO))))
    h = 2 * w
    return Deteccion(color=color, x=int(u) - w // 2, y=int(v) - h, w=w, h=h,
                     area=w * h, llenado=0.9, aspecto=2.0, cx=float(u),
                     cy=float(v) - h / 2.0)


def pilar_px(color: str, x: int, base_y: int, w: int = 20, h: int = 40):
    """Una deteccion puesta directamente en pixeles."""
    return Deteccion(color=color, x=x, y=base_y - h, w=w, h=h, area=w * h,
                     llenado=0.9, aspecto=h / w, cx=x + w / 2.0,
                     cy=base_y - h / 2.0)


ocfg = params_mod.valores_por_defecto()


def esquivador(**cambios):
    return Esquivador(dict(ocfg["obstaculos"], **cambios), ocfg["geometria"],
                      ocfg["velocidad"], ocfg["limites"])


### LA REGLA. Rojo -> se pasa por SU derecha: se le empuja al canto IZQUIERDO
### de la imagen y la direccion que pide es a la DERECHA (+). Verde, todo al
### reves. Y no se invierte con el sentido: son la derecha y la izquierda DEL
### VEHICULO, y trabajar en pixeles de la camara es trabajar en ese marco.
e1 = esquivador(rampa_dir_pct_s=0.0)
m1 = e1.paso({"rojo": [pilar("rojo", 800.0, 0.0)]}, geo, yaw=0.0, vel_pct=50,
             ahora=1000.0)
prueba("rojo de frente: pide DERECHA", m1.direccion > 0 and m1.peso > 0,
       f"dir={m1.direccion:.0f} peso={m1.peso:.2f}")
prueba("rojo: se le empuja al canto IZQUIERDO",
       e1.info["obj_px"] < geo.W / 2 and e1.info["borde_px"] > e1.info["obj_px"],
       str(e1.info))
prueba("rojo: hay un pilar en juego y su lado es +1 (paso por su derecha)",
       e1.en_juego and e1.lado_en_juego == 1 and m1.estado == SIGUIENDO)
e2 = esquivador(rampa_dir_pct_s=0.0)
m2 = e2.paso({"verde": [pilar("verde", 800.0, 0.0)]}, geo, yaw=0.0, vel_pct=50,
             ahora=1000.0)
prueba("verde de frente: pide IZQUIERDA", m2.direccion < 0, f"{m2.direccion:.0f}")
prueba("verde: se le empuja al canto DERECHO", e2.info["obj_px"] > geo.W / 2)
prueba("los lados estan declarados una sola vez",
       LADO["rojo"] == 1 and LADO["verde"] == -1)

### ASIMETRICO. Un pilar que ya esta mas afuera que la columna objetivo no
### pide nada. Un servo simetrico giraria HACIA el para "mantenerlo" en el
### canto, y eso es un rumbo de colision con angulo fijo.
e3 = esquivador(rampa_dir_pct_s=0.0)
m3 = e3.paso({"rojo": [pilar_px("rojo", 0, 280)]}, geo, yaw=0.0, vel_pct=50,
             ahora=1000.0)
prueba("rojo ya fuera del camino (pegado al canto izquierdo): no pide volver hacia el",
       m3.direccion == 0.0 and m3.estado == SIGUIENDO and e3.info["falta_pct"] == 0,
       f"dir={m3.direccion:.0f} {e3.info}")

### EL EMPUJON DE CERCA. Con el pilar encima y todavia en cuadro, aunque ya
### este en el canto se pide un poco mas hacia el lado de paso: el borde de
### la imagen no da holgura para un carro de 20 cm.
e4 = esquivador(rampa_dir_pct_s=0.0)
m4 = e4.paso({"rojo": [pilar_px("rojo", 0, 400)]}, geo, yaw=0.0, vel_pct=50,
             ahora=1000.0)
prueba("pilar cerca y en el canto: empujon hacia el lado de paso",
       m4.direccion > 0 and e4.info["empuje"] and
       e4.info["dist_mm"] < ocfg["obstaculos"]["empuje_bajo_mm"],
       str(e4.info))

### MAS CERCA, MAS PESO: de lejos el pilar solo insinua; de cerca manda.
e5 = esquivador(rampa_dir_pct_s=0.0)
m_lejos = e5.paso({"rojo": [pilar("rojo", 1400.0, 0.0)]}, geo, ahora=1000.0)
m_cerca = e5.paso({"rojo": [pilar("rojo", 700.0, 0.0)]}, geo, ahora=1000.1)
prueba("mas cerca, mas peso", m_cerca.peso > m_lejos.peso,
       f"{m_lejos.peso:.2f} -> {m_cerca.peso:.2f}")
prueba("dentro de mandar_desde_mm el peso es 1", abs(m_cerca.peso - 1.0) < 1e-6)
prueba("con un pilar en juego se limita la velocidad",
       m_cerca.vel_pct == ocfg["obstaculos"]["vel_pct"])

### EL MAS CERCANO MANDA, con ventaja para el que ya se venia siguiendo.
e6 = esquivador(rampa_dir_pct_s=0.0)
e6.paso({"rojo": [pilar("rojo", 1200.0, -100.0)],
         "verde": [pilar("verde", 600.0, 100.0)]}, geo, ahora=1000.0)
prueba("dos pilares: manda el mas cercano", e6.info["color"] == "verde",
       str(e6.info))
e7 = esquivador(rampa_dir_pct_s=0.0)
e7.paso({"rojo": [pilar("rojo", 900.0, -100.0)]}, geo, ahora=1000.0)
e7.paso({"rojo": [pilar("rojo", 900.0, -100.0)],
         "verde": [pilar("verde", 800.0, 100.0)]}, geo, ahora=1000.1)
prueba("no se cambia de pilar por 10 cm de diferencia",
       e7.info["color"] == "rojo", str(e7.info))

### EL COMPROMISO DE ADELANTAMIENTO. Perdido de CERCA, el pilar esta en el
### punto ciego de delante del carro: se congela el rumbo con el que se
### perdio, sin centrado, hasta que la cola lo haya pasado.
e8 = esquivador()
e8.paso({"rojo": [pilar("rojo", 300.0, -120.0)]}, geo, yaw=10.0, vel_pct=50,
        ahora=1000.0)
m8 = e8.paso({}, geo, yaw=12.0, vel_pct=50, ahora=1000.05)
prueba("perdido de cerca: se congela el rumbo en el que quedo",
       m8.estado == ADELANTANDO and m8.rumbo_fijo is not None and
       abs(m8.rumbo_fijo - 12.0) < 1e-6 and m8.peso == 1.0 and m8.direccion == 0.0,
       str(m8))
prueba("y se sabe que hay un pilar en juego (lado +1)",
       e8.en_juego and e8.lado_en_juego == 1)
seg = e8._fin_compromiso - 1000.05
v_real = ocfg["velocidad"]["vel_max_mm_s"] * ocfg["limites"]["vmax"] / 255.0 * 0.5
esperado = (300.0 + ocfg["geometria"]["largo_carro_mm"]) / v_real
prueba("dura lo que tarda el carro ENTERO en pasarlo, a la velocidad REAL (con vmax)",
       abs(seg - min(esperado, ocfg["obstaculos"]["compromiso_max_ms"] / 1000.0)) < 0.05,
       f"{seg:.2f}s vs {esperado:.2f}s")
m8b = e8.paso({}, geo, yaw=12.0, vel_pct=50, ahora=1000.5)
prueba("sigue congelado mientras dure", m8b.estado == ADELANTANDO and
       m8b.rumbo_fijo == m8.rumbo_fijo)
m8c = e8.paso({}, geo, yaw=12.0, vel_pct=50, ahora=1010.0)
prueba("el compromiso caduca y se suelta el rumbo",
       m8c.estado == LIBRE and m8c.rumbo_fijo is None and m8c.peso == 0.0 and
       not e8.en_juego, str(m8c))

### Perdido de LEJOS no hay compromiso: salio por el canto porque el carro ya
### giro de sobra, y el rumbo lo volvera a traer a la vista.
e9 = esquivador()
e9.paso({"rojo": [pilar("rojo", 900.0, -120.0)]}, geo, yaw=0.0, ahora=1000.0)
m9 = e9.paso({}, geo, yaw=0.0, ahora=1000.05)
prueba("perdido de lejos: libre, sin compromiso",
       m9.estado == LIBRE and m9.rumbo_fijo is None and not e9.en_juego, str(m9))

### Sin giroscopio el compromiso es "recto": direccion 0 y centrado apagado.
e10 = esquivador()
e10.paso({"verde": [pilar("verde", 300.0, 120.0)]}, geo, yaw=None, ahora=1000.0)
m10 = e10.paso({}, geo, yaw=None, ahora=1000.05)
prueba("sin giroscopio: compromiso recto (peso 1, direccion 0, sin rumbo)",
       m10.estado == ADELANTANDO and m10.rumbo_fijo is None and
       m10.peso == 1.0 and m10.direccion == 0.0 and e10.lado_en_juego == -1)

### Lo VISIBLE manda: otro pilar cerca interrumpe el compromiso; uno lejos no.
e11 = esquivador(rampa_dir_pct_s=0.0)
e11.paso({"rojo": [pilar("rojo", 300.0, -120.0)]}, geo, yaw=0.0, ahora=1000.0)
e11.paso({}, geo, yaw=0.0, ahora=1000.05)
m11 = e11.paso({"verde": [pilar("verde", 1300.0, 0.0)]}, geo, yaw=0.0, ahora=1000.1)
prueba("un pilar lejano NO interrumpe el compromiso", m11.estado == ADELANTANDO)
m11b = e11.paso({"verde": [pilar("verde", 500.0, 0.0)]}, geo, yaw=0.0, ahora=1000.15)
prueba("un pilar cercano SI lo interrumpe y se le sigue",
       m11b.estado == SIGUIENDO and m11b.color == "verde" and m11b.direccion < 0,
       str(m11b))

### El MURO manda: si el pasillo se cierra durante el compromiso, se suelta.
e12 = esquivador()
e12.paso({"rojo": [pilar("rojo", 300.0, -120.0)]}, geo, yaw=0.0, ahora=1000.0)
e12.paso({}, geo, yaw=0.0, pasillo_mm=1500.0, ahora=1000.05)
m12 = e12.paso({}, geo, yaw=0.0, pasillo_mm=400.0, ahora=1000.1)
prueba("pasillo cerrado durante el compromiso: se suelta",
       m12.estado == LIBRE and e12.info.get("fin") == "pasillo cerrado", str(e12.info))

### LOS PILARES NO SON MURO. Un pilar en el pasillo NO lo cierra: de el se
### ocupa el esquivador. Si lo cerrara, dispararia frenada, giro de esquina
### de 90 (y su conteo) o escape en reversa, que son solo para muros.
p_base = muro.perfil(escena(300), geo, mcfg)
esc_pilar = escena(300)
esc_pilar["blanco"][380:440, 300:340] = 0            # el pilar no es blanco
esc_pilar["rojo"] = np.zeros((H, W), np.uint8)
esc_pilar["rojo"][380:440, 300:340] = 255
p_pilar = muro.perfil(esc_pilar, geo, mcfg)
prueba("un pilar en el pasillo NO lo cierra",
       abs(p_pilar.pasillo_mm - p_base.pasillo_mm) < 30,
       f"{p_pilar.pasillo_mm:.0f} vs {p_base.pasillo_mm:.0f}")
sin_rojo = dict(esc_pilar)
del sin_rojo["rojo"]
p_sin = muro.perfil(sin_rojo, geo, mcfg)
prueba("(sin la mascara del pilar si lo cerraria: esa era la trampa)",
       p_sin.pasillo_mm < p_base.pasillo_mm - 100,
       f"{p_sin.pasillo_mm:.0f} vs {p_base.pasillo_mm:.0f}")

### LA NAVEGACION CON UN PILAR EN JUEGO
ncfg2 = params_mod.valores_por_defecto()


def navegador(**cambios):
    return Navegador(dict(ncfg2["navegacion"], **cambios), ncfg2["limites"],
                     ncfg2["escape"], ncfg2["rescate"])


# pared LEJANA a la izquierda (1.3 m): el centrado pide derecha, no hay esquina
lado220 = escena(220)
lado220["blanco"][:, W // 2:] = 255
p_lado = muro.perfil(lado220, geo, mcfg)
nv0 = navegador()
d_sin = nv0.paso(p_lado, None, 0)
prueba("sin pilar, el centrado se aparta de la pared (derecha)",
       d_sin.direccion > 0 and d_sin.estado == RECTO,
       f"{d_sin.direccion} {d_sin.estado} {d_sin.motivo}")
nv1 = navegador()
d_con = nv1.paso(p_lado, None, 0,
                 maniobra=Maniobra(direccion=-40.0, peso=1.0,
                                   estado=SIGUIENDO, color="verde"))
prueba("con peso 1 manda el servo del pilar y el centrado calla",
       d_con.direccion < 0 and d_con.estado == RECTO,
       f"{d_con.direccion} {d_con.motivo}")

### Adelantando: se mantiene el rumbo congelado; ni el centrado ni la recta
### tiran del carro mientras la cola pasa el pilar.
nv2 = navegador()
nv2.paso(p_lado, 0.0, 1)                          # rumbo de la recta = 0
d_ad = nv2.paso(p_lado, 0.0, 1,
                maniobra=Maniobra(direccion=0.0, peso=1.0, rumbo_fijo=-20.0,
                                  estado=ADELANTANDO, color="rojo"))
prueba("adelantando: se sigue el rumbo congelado aunque el centrado diga otra cosa",
       d_ad.direccion < 0 and "rumbo fijo" in d_ad.motivo,
       f"{d_ad.direccion} {d_ad.motivo}")

### Mientras el pilar manda, del rumbo solo sobrevive yaw_al_esquivar.
nv3 = navegador()
nv3.paso(p_libre, 0.0, 1)                         # rumbo 0
d_y1 = nv3.paso(p_libre, 30.0, 1,                 # el carro va 30 torcido
                maniobra=Maniobra(direccion=40.0, peso=1.0, yaw_factor=0.3,
                                  estado=SIGUIENDO, color="rojo"))
d_y2 = nv3.paso(p_libre, 30.0, 1,
                maniobra=Maniobra(direccion=40.0, peso=1.0, yaw_factor=1.0,
                                  estado=SIGUIENDO, color="rojo"))
prueba("el rumbo cede al pilar segun yaw_al_esquivar",
       d_y1.direccion > 0 and d_y1.direccion > d_y2.direccion,
       f"{d_y1.direccion} vs {d_y2.direccion}")
d_v = nv3.paso(p_libre, 0.0, 1,
               maniobra=Maniobra(direccion=0.0, peso=1.0, vel_pct=45.0,
                                 estado=SIGUIENDO, color="rojo"))
prueba("con pilar se va a la velocidad del esquive", d_v.vel <= 45, str(d_v.vel))

### EL GIRO DE ESQUINA CEDE ANTE EL PILAR Y SE REANUDA, sin sumar 90 en cada
### reintento y sin contar la esquina dos veces.
contados = []
nv4 = navegador(min_recto_ms=0)
nv4.al_completar_giro = lambda lado: contados.append(lado)
nv4.paso(p_libre, 0.0, 1)                         # rumbo 0
nv4.estado = GIRO
nv4.lado_giro = 1
nv4.rumbo_objetivo = 90.0
nv4.t_estado = time.time()
man_v = Maniobra(direccion=-30.0, peso=1.0, estado=SIGUIENDO, color="verde")
nv4.paso(p_libre, 0.0, 1, maniobra=man_v, lado_pilar=-1)
prueba("el giro cede ante el pilar: RECTO sin contar",
       nv4.estado == RECTO and not contados and nv4._giro_suelto, nv4.estado)
nv4.paso(p_libre, 0.0, 1, maniobra=man_v, lado_pilar=-1)
prueba("mientras el pilar manda no se vuelve a sumar 90",
       nv4.rumbo_objetivo == 90.0 and nv4.estado == RECTO,
       str(nv4.rumbo_objetivo))
d_re = nv4.paso(p_libre, 10.0, 1)                 # el pilar se fue, faltan 80
prueba("sin pilar el giro se reanuda hacia la recta nueva",
       nv4.estado == GIRO and d_re.direccion > 0, f"{nv4.estado} {d_re.direccion}")
nv4.paso(p_libre, 88.0, 1)                        # ya clavado
prueba("el giro reanudado termina SIN contar la esquina (ya la contaron las lineas)",
       nv4.estado == RECTO and not contados and not nv4._giro_suelto,
       f"{nv4.estado} {contados}")

### Tras el ESCAPE el rumbo se ancla a una recta valida, no al yaw en el que
### quedo mirando (asi se fue el carro en sentido contrario tras varios escapes).
nv5 = navegador()
nv5.paso(p_libre, 0.0, 1)                         # rumbo 0
nv5.estado = ESCAPE
nv5._t_fin_escape = 0.0
nv5.paso(p_libre, 80.0, 1)
prueba("tras el escape se ancla a la recta mas cercana (90), no al yaw (80)",
       nv5.estado == RECTO and abs(nv5.rumbo_objetivo - 90.0) < 1e-6,
       f"{nv5.estado} {nv5.rumbo_objetivo}")

### Torcido respecto a la recta (esquivando), un pasillo cerrado NO es una
### esquina: es el muro de la propia recta visto de lado.
nv6 = navegador(min_recto_ms=0)
nv6.paso(p_libre, 0.0, 1)
d6 = nv6.paso(p_mitad, 40.0, 1)                   # pared a 40 cm, carro 40 torcido
prueba("torcido 40 grados, el pasillo cerrado NO dispara la esquina",
       nv6.estado == RECTO, f"{nv6.estado} {d6.motivo}")
nv7 = navegador(min_recto_ms=0)
nv7.paso(p_libre, 0.0, 1)
nv7.paso(p_mitad, 5.0, 1)
prueba("derecho, el mismo pasillo SI dispara la esquina",
       nv7.estado == PRE_GIRO, nv7.estado)

# ===========================================================================
print("== rescate de esquina (el triangulo) ==")


def rincon(vertice_fila: int = 250, ala: int = 120):
    """Piso en TRIANGULO: las dos paredes del rincon cerrando por los dos
    lados, con el vertice arriba en el centro."""
    blanco = np.zeros((H, W), np.uint8)
    for x in range(W):
        # el contacto sube hacia el centro: triangulo de piso
        fila = vertice_fila + int(ala * abs(x - W / 2) / (W / 2))
        blanco[min(H - 1, fila):, x] = 255
    return {"blanco": blanco, "negro": np.zeros((H, W), np.uint8)}


rcfg = ncfg["rescate"]
det = muro.DetectorAtrapado()
p_rincon = muro.perfil(rincon(), geo, mcfg)
det.paso(p_rincon, rcfg, ahora=100.0)
atrapado = det.paso(p_rincon, rcfg, ahora=100.0 + rcfg["confirmar_ms"] / 1000.0 + 0.1)
prueba("el rincon en triangulo se reconoce", atrapado, str(det.info))

### Una pared PLANA de frente no es una esquina: da 0 % de relieve y se
### resuelve con el escape en reversa, que ya existia. Si el rescate saltara
### aqui, el carro giraria 100 grados contra una pared.
det2 = muro.DetectorAtrapado()
p_plano = muro.perfil(escena(int(H * 0.93)), geo, mcfg)
det2.paso(p_plano, rcfg, ahora=100.0)
prueba("una pared plana de frente NO dispara el rescate",
       not det2.paso(p_plano, rcfg, ahora=105.0),
       det2.info.get("fallo", "") + " " + str(det2.info.get("relieve_pct")))

### Si el vertice se ALEJA, el carro si esta saliendo: no hay nada que
### rescatar y la cuenta se reinicia.
det3 = muro.DetectorAtrapado()
det3.paso(muro.perfil(rincon(250), geo, mcfg), rcfg, ahora=100.0)
lejos = muro.perfil(rincon(180), geo, mcfg)      # el vertice se va arriba
prueba("si el vertice se aleja, no hay rescate",
       not det3.paso(lejos, rcfg, ahora=100.0 + rcfg["confirmar_ms"] / 1000.0 + 0.1),
       str(det3.info.get("fallo")))

### El giro: hacia adentro segun el sentido, y hacia EL LADO DEL PILAR si hay
### uno en juego (pasarlo por el lado incorrecto termina la ronda; quedarse
### atascado solo cuesta tiempo).
nav5 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], rcfg)
nav5.paso(p_rincon, None, 1, atrapado=True)
prueba("atrapado: entra en RESCATE", nav5.estado == RESCATE, nav5.estado)
prueba("horario: gira a la derecha", nav5._lado_rescate == 1,
       str(nav5._lado_rescate))
nav6 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], rcfg)
nav6.paso(p_rincon, None, 1, atrapado=True, lado_pilar=-1)
prueba("con un pilar en juego manda el pilar, no el sentido",
       nav6._lado_rescate == -1, str(nav6._lado_rescate))

### Al terminar NO se adopta el rumbo en el que se quedo mirando: eso fue lo
### que hizo que el carro se fuera en sentido contrario tras varios escapes.
nav7 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"], rcfg)
nav7.rumbo_objetivo = 0.0
nav7._anclar_recta_mas_cercana(84.0, True)
prueba("tras maniobrar se ancla a una recta valida, no al yaw",
       abs(nav7.rumbo_objetivo - 90.0) < 1e-6, str(nav7.rumbo_objetivo))

### El giro de esquina CEDE ante un pilar (el modo que el usuario quiere
### mantener): un giro comprometido de 90 con un pilar delante lo atropella.
nav8 = Navegador(dict(ncfg["navegacion"], min_recto_ms=0), ncfg["limites"],
                 ncfg["escape"], rcfg)
nav8.estado = "giro"
nav8.lado_giro = 1
nav8.paso(p_libre, None, 1, lado_pilar=-1)
prueba("el giro de esquina cede ante un pilar", nav8.estado == RECTO,
       nav8.estado)

# ===========================================================================
print("== parametros ==")
vals = params_mod.valores_por_defecto()
prueba("validar recorta", params_mod.validar("limites", "vmax", 999) == 255)
prueba("validar bool", params_mod.validar("navegacion", "usar_yaw", "0") is False)
try:
    params_mod.validar("carrera", "sentido", "volar")
    prueba("opcion invalida lanza", False)
except ValueError:
    prueba("opcion invalida lanza", True)

with tempfile.TemporaryDirectory() as tmp:
    ruta = Path(tmp) / "params.json"
    datos = params_mod.cargar(ruta)
    vals["limites"]["vmax"] = 99
    params_mod.guardar_perfil(datos, "prueba", vals)
    params_mod.guardar_archivo(datos, ruta)
    datos2 = params_mod.cargar(ruta)
    perfil = params_mod.obtener(datos2, "prueba")
    prueba("perfil guardado y releido",
           perfil["valores"]["limites"]["vmax"] == 99)
    for i in range(22):
        params_mod.guardar_perfil(datos2, f"p{i}", vals)
    prueba(f"solo quedan {params_mod.MAX_PERFILES} perfiles",
           len(datos2["perfiles"]) == params_mod.MAX_PERFILES,
           str(len(datos2["perfiles"])))

# ===========================================================================
print()
if FALLOS:
    print(f"{len(FALLOS)}/{TOTAL} PRUEBAS FALLARON: {FALLOS}")
    sys.exit(1)
print(f"las {TOTAL} pruebas pasaron")
