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

import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

from src import protocolo as P          # noqa: E402
from src import params as params_mod    # noqa: E402
from src import botones as bot_mod      # noqa: E402
from src.geometria import Geometria     # noqa: E402
from src import muro                    # noqa: E402
from src.lineas import (GestorLineas, HORARIO, ANTIHORARIO,   # noqa: E402
                        ZONA_RECTA, ZONA_ESQUINA)
from src.navegacion import (Navegador, RECTO, PRE_GIRO,      # noqa: E402
                            GIRO, GIRO_2T, ESCAPE)

FALLOS = []
TOTAL = 0


def _norm(a):
    return (a + 180.0) % 360.0 - 180.0


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
               cnt_lineas=0x53, botones=P.B_BOTON | P.B_CORTE)
tr = s.a_bytes()
prueba("sensores: 20 bytes (15 de payload)", len(tr) == 20, f"{len(tr)}")
s2 = P.Sensores.desde_payload(tr[4:19])
prueba("sensores: ida y vuelta",
       (s2.yaw_deci, s2.gz_deci, s2.c, s2.cnt_naranja, s2.cnt_azul) ==
       (-1234, 567, 1000, 3, 5))
prueba("sensores: clase linea", s2.clase_linea == P.LINEA_AZUL)
prueba("sensores: yaw en grados", abs(s2.yaw + 123.4) < 1e-6)
prueba("sensores: nivel del pulsador y corte del ESP32",
       s2.boton and s2.corte_por_boton, f"{s2.botones:#04x}")
# Un ESP32 con firmware viejo manda 14 bytes: tiene que seguir hablando, solo
# que sin botones. Si esto falla, actualizar el firmware deja de ser opcional
# y pasa a ser obligatorio, que es justo lo que se quiere evitar.
s3 = P.Sensores.desde_payload(tr[4:18])
prueba("sensores: payload viejo de 14 bytes sigue valiendo",
       s3.botones == 0 and s3.cnt_azul == 5 and s3.yaw_deci == -1234)

cfg = P.empaquetar_cfg_tcs(80, 120, 60, 110, 70, 1, 3, 246, 2)
prueba("cfg_tcs: 18 bytes (13 de payload)", len(cfg) == 18, f"{len(cfg)}")

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

print("== patas INT de los sensores ==")
# El estado de los sensores lleva dos bits nuevos para saber, desde la web, si
# cada pata INT esta cableada y funcionando. Los bits 0x40/0x80 siguen siendo
# la clase de linea: no se pueden pisar.
prueba("los bits INT no chocan con la clase de linea",
       (P.S_MPU_INT | P.S_TCS_INT) & 0xC0 == 0,
       hex(P.S_MPU_INT | P.S_TCS_INT))
s_int = P.Sensores(estado=P.S_MPU_OK | P.S_TCS_OK | P.S_MPU_INT | P.S_TCS_INT
                   | (P.LINEA_AZUL << 6))
prueba("se leen las dos patas", s_int.mpu_int and s_int.tcs_int)
prueba("y la clase de linea sigue intacta", s_int.clase_linea == P.LINEA_AZUL)
s_sin = P.Sensores(estado=P.S_MPU_OK | P.S_TCS_OK)
prueba("sin patas cableadas los bits estan a cero",
       not s_sin.mpu_int and not s_sin.tcs_int)
prueba("y eso no impide que los sensores esten OK",
       s_sin.mpu_ok and s_sin.tcs_ok)

# El umbral de la INT viaja en la trama de configuracion del TCS.
cfg13 = P.empaquetar_cfg_tcs(80, 110, 90, 95, 95, 1, 3, 246, 2, 30, 18, 40)
prueba("cfg_tcs con umbral de INT: 18 bytes", len(cfg13) == 18, str(len(cfg13)))
prueba("el umbral va en el ultimo byte del payload", cfg13[4 + 12] == 40,
       str(cfg13[4 + 12]))
# el firmware lo recorta a 5..95 y usa 55 si viene fuera; aqui se recorta ya
cfg_alto = P.empaquetar_cfg_tcs(80, 110, 90, 95, 95, 1, 3, 246, 2, 30, 18, 200)
prueba("un umbral absurdo se recorta antes de salir", cfg_alto[4 + 12] == 95,
       str(cfg_alto[4 + 12]))


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

mcfg_viejo = dict(mcfg, metodo="negro")
p_viejo = muro.perfil(escena(300, con_brillo=True), geo, mcfg_viejo)
filas_viejo = p_viejo.y_contacto[220:340]
prueba("(control) el metodo viejo SI se pierde con el brillo",
       int(np.median(filas_viejo)) < 290 or not p_viejo.valido[250:300].any(),
       str(int(np.median(filas_viejo))))

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
print("== que recta es cada pared (giroscopio) ==")
from src.muro import clasificar_recta, Segmento   # noqa: E402

# Carro derecho: la pared de la izquierda sale a +90, la de la derecha a -90
# y la de enfrente a 0 (ver el docstring de Segmento.angulo).
prueba("derecho: pared izquierda", clasificar_recta(90, -300, 0.0) == "lateral_izq")
prueba("derecho: pared derecha", clasificar_recta(-90, 300, 0.0) == "lateral_der")
prueba("derecho: pared de frente", clasificar_recta(0, 0, 0.0) == "frontal")

# Carro cruzado 40 grados a la derecha (viene de esquivar un pilar y se ha
# ido hacia la esquina interna). Todas las paredes aparecen giradas 40 grados.
prueba("cruzado 40: la pared de enfrente se reconoce igual",
       clasificar_recta(40, 0, 40.0) == "frontal",
       clasificar_recta(40, 0, 40.0))
prueba("cruzado 40: la lateral izquierda tambien",
       clasificar_recta(130, -300, 40.0) == "lateral_izq",
       clasificar_recta(130, -300, 40.0))
# Sin giroscopio, con ese desvio, ninguna se reconoce: queda "otro". Es el
# fallo seguro (no saber) en vez del peligroso (confundir frente con lateral).
prueba("cruzado 40 SIN giroscopio: no confunde, se declara ignorante",
       clasificar_recta(40, 0, None) == "otro" and
       clasificar_recta(130, -300, None) == "otro")

# El caso que rompia al carro: cruzado 50 grados, la pared de ENFRENTE
# aparece a 50 y sin correccion se tomaria por la pared del carril.
prueba("cruzado 50 SIN giroscopio la de enfrente parece lateral",
       clasificar_recta(50, 0, None, tolerancia_deg=45.0) == "lateral_der",
       clasificar_recta(50, 0, None, tolerancia_deg=45.0))
prueba("cruzado 50 CON giroscopio sigue siendo la de enfrente",
       clasificar_recta(50, 0, 50.0, tolerancia_deg=45.0) == "frontal")

# --- sobre el perfil completo, con una escena sintetica -------------------
def escena_pasillo(x_izq_mm=-450.0, x_der_mm=450.0, y_frente_mm=None):
    """Piso blanco con dos paredes laterales rectas y, opcionalmente, una
    pared cruzada delante.

    El muestreo va en pasos de 0.5 mm a proposito: con pasos gruesos quedan
    columnas sin pintar (la perspectiva comprime la distancia cerca del
    horizonte), las rectas salen troceadas y la prueba mediria el ruido del
    generador de escenas en vez del clasificador.
    """
    blanco = np.full((H, W), 255, np.uint8)
    for y_mm in np.arange(150.0, 2600.0, 0.5):
        for xm in (x_izq_mm, x_der_mm):
            u, v = geo.suelo_a_pixel(xm, y_mm)
            if 0 <= u < W and 0 <= v < H:
                blanco[:v + 1, u] = 0          # encima del contacto: no es piso
    if y_frente_mm is not None:
        for xm in np.arange(-900.0, 900.0, 0.5):
            u, v = geo.suelo_a_pixel(xm, y_frente_mm)
            if 0 <= u < W and 0 <= v < H:
                blanco[:v + 1, u] = 0
    return {"blanco": blanco, "negro": np.zeros((H, W), np.uint8)}

pr = muro.perfil(escena_pasillo(), geo, mcfg, error_rumbo=0.0, sentido=1)
clases = [s.clase for s in pr.segmentos]
prueba("pasillo recto: reconoce las dos paredes laterales",
       "lateral_izq" in clases and "lateral_der" in clases, str(clases))
prueba("y ninguna de frente", "frontal" not in clases, str(clases))
prueba("distancias laterales del orden esperado (450mm)",
       pr.lateral_izq_mm is not None and pr.lateral_der_mm is not None and
       abs(pr.lateral_izq_mm - 450) < 150 and abs(pr.lateral_der_mm - 450) < 150,
       f"izq={pr.lateral_izq_mm} der={pr.lateral_der_mm}")
prueba("en horario la interna es la DERECHA",
       pr.interna_mm == pr.lateral_der_mm and pr.externa_mm == pr.lateral_izq_mm)
pr_anti = muro.perfil(escena_pasillo(), geo, mcfg, error_rumbo=0.0, sentido=-1)
prueba("en antihorario la interna es la IZQUIERDA",
       pr_anti.interna_mm == pr_anti.lateral_izq_mm)

pf = muro.perfil(escena_pasillo(y_frente_mm=900.0), geo, mcfg,
                 error_rumbo=0.0, sentido=1)
prueba("con pared cruzada delante, la reconoce",
       pf.frontal_mm is not None, str([s.clase for s in pf.segmentos]))
prueba("y la situa a la distancia correcta (900mm)",
       pf.frontal_mm is not None and abs(pf.frontal_mm - 900) < 250,
       str(pf.frontal_mm))

# --- la navegacion usa la pared interna identificada ----------------------
vn = params_mod.valores_por_defecto()
vn["navegacion"]["estrategia"] = "pared"
nav_p = Navegador(vn["navegacion"], vn["limites"], vn["escape"], vn["giro2t"])
d = nav_p.paso(pr, 0.0, 1)
prueba("la estrategia 'pared' sigue la recta identificada",
       "pared int(recta)" in d.motivo, d.motivo)

# sin clasificacion (sin rectas) cae a la banda, sin romperse
vn2 = params_mod.valores_por_defecto()
vn2["navegacion"]["estrategia"] = "pared"
vn2["navegacion"]["usar_rectas"] = False
nav_b = Navegador(vn2["navegacion"], vn2["limites"], vn2["escape"], vn2["giro2t"])
d2 = nav_b.paso(pr, 0.0, 1)
prueba("con usar_rectas apagado vuelve a la banda", "pared int(banda)" in d2.motivo,
       d2.motivo)

# error_de_rumbo: lo que alimenta toda la clasificacion
nav_r = Navegador(vn["navegacion"], vn["limites"], vn["escape"], vn["giro2t"])
prueba("sin rumbo de referencia no hay desvio", nav_r.error_de_rumbo(10.0) is None)
nav_r.rumbo_objetivo = 90.0
prueba("desvio = yaw - rumbo de la recta", abs(nav_r.error_de_rumbo(120.0) - 30.0) < 1e-6,
       str(nav_r.error_de_rumbo(120.0)))
prueba("y no se envuelve mal cerca de 180",
       abs(nav_r.error_de_rumbo(-170.0) - 100.0) < 1e-6,
       str(nav_r.error_de_rumbo(-170.0)))

# El caso que describe el equipo: el carro cruza la esquina, esquiva un pilar
# que lo empuja hacia la esquina interna y llega TORCIDO viendo las dos rectas
# a la vez. Sin referencia de rumbo no sabe cual es cual.
def escena_esquina(desvio_deg, x_pared_mm=-500.0, y_frente_mm=1100.0):
    blanco = np.full((H, W), 255, np.uint8)
    th = math.radians(desvio_deg)
    def pintar(px, py):
        # del marco de la PISTA al del CARRO, girado 'desvio' a la derecha
        xc = px * math.cos(th) - py * math.sin(th)
        yc = px * math.sin(th) + py * math.cos(th)
        if yc < 120:
            return
        u, v = geo.suelo_a_pixel(xc, yc)
        if 0 <= u < W and 0 <= v < H:
            blanco[:v + 1, u] = 0
    for t in np.arange(120.0, 2600.0, 0.5):
        pintar(x_pared_mm, t)              # pared lateral de la recta actual
    for t in np.arange(-900.0, 900.0, 0.5):
        pintar(t, y_frente_mm)             # pared de enfrente (fondo de curva)
    return {"blanco": blanco, "negro": np.zeros((H, W), np.uint8)}

pe = muro.perfil(escena_esquina(0), geo, mcfg, error_rumbo=0.0, sentido=1)
clases_e = [s.clase for s in pe.segmentos if s.largo > 120]
prueba("derecho en la curva: separa su carril de la pared de enfrente",
       "lateral_izq" in clases_e and "frontal" in clases_e, str(clases_e))
prueba("y da las dos distancias",
       pe.externa_mm is not None and pe.frontal_mm is not None,
       f"ext={pe.externa_mm} frente={pe.frontal_mm}")

esc45 = escena_esquina(45)
sin_giro = muro.perfil(esc45, geo, mcfg, error_rumbo=None, sentido=1)
con_giro = muro.perfil(esc45, geo, mcfg, error_rumbo=45.0, sentido=1)
prueba("cruzado 45 SIN giroscopio: no identifica nada (no se inventa)",
       sin_giro.frontal_mm is None and sin_giro.interna_mm is None and
       sin_giro.externa_mm is None,
       f"frente={sin_giro.frontal_mm} int={sin_giro.interna_mm}")
prueba("cruzado 45 CON giroscopio: reconoce la pared de enfrente",
       con_giro.frontal_mm is not None, str(con_giro.frontal_mm))
prueba("y a una distancia razonable (1100mm)",
       con_giro.frontal_mm is not None and abs(con_giro.frontal_mm - 1100) < 300,
       str(con_giro.frontal_mm))

# esa pared de frente identificada dispara la esquina por si sola
vf = params_mod.valores_por_defecto()
vf["navegacion"].update(min_recto_ms=0, girar_bajo_mm=1400.0)
nav_f = Navegador(vf["navegacion"], vf["limites"], vf["escape"], vf["giro2t"])
df = nav_f.paso(con_giro, 45.0, 1)
prueba("la pared de frente identificada dispara la esquina",
       "pared de frente" in df.motivo, df.motivo)

# ===========================================================================
print("== lineas / sentido / vueltas ==")
lcfg = {"naranja_es_horario": True, "usar_tcs": True, "usar_camara": True,
        "umbral_cruce_mm": 260.0, "ventana_par_ms": 300,
        "refractario_esquina_ms": 400, "pares_para_invertir": 2}

def par(g, a, b):
    """Cruza las dos lineas de una esquina, en ese orden."""
    g.evento_tcs(a)
    g.evento_tcs(b)

gl = GestorLineas(lcfg)
gl.evento_tcs("naranja")
prueba("primera linea naranja => horario", gl.sentido == HORARIO)
prueba("una linea SOLA no cuenta esquina (falta el par)",
       gl.esquinas == 0, str(gl.esquinas))
prueba("pero ya entra en la zona de esquina", gl.en_esquina)
gl.evento_tcs("azul")
prueba("el par completo cuenta UNA esquina", gl.esquinas == 1, str(gl.esquinas))
prueba("el primer par fija el orden de referencia",
       gl.orden_esperado == ["naranja", "azul"], str(gl.orden_esperado))
time.sleep(0.45)
par(gl, "naranja", "azul")
prueba("segundo par cuenta", gl.esquinas == 2, str(gl.esquinas))
prueba("vueltas", gl.vueltas(4) == 0)
for _ in range(2):
    time.sleep(0.45)
    par(gl, "naranja", "azul")
prueba("4 pares = 1 vuelta", gl.vueltas(4) == 1, str(gl.esquinas))
prueba("sin incoherencias en una vuelta limpia", gl.incoherencias == 0)

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

print("== coherencia del par de lineas ==")
# Las cuatro esquinas se cruzan siempre en el mismo orden. Un par al reves o
# es basura o el carro se dio la vuelta; contarlo estropea el fin de carrera.
gi = GestorLineas(lcfg)
par(gi, "naranja", "azul")                    # referencia: naranja+azul
time.sleep(0.45)
par(gi, "azul", "naranja")                    # <-- al reves
prueba("un par al reves NO cuenta esquina", gi.esquinas == 1, str(gi.esquinas))
prueba("y queda registrado como incoherencia", gi.incoherencias == 1)
prueba("el sentido no cambia por una lectura suelta", gi.sentido == HORARIO)
time.sleep(0.45)
par(gi, "azul", "naranja")                    # <-- otra vez: ya no es ruido
prueba("dos pares al reves seguidos SI cuentan (vuelta de regreso)",
       gi.esquinas == 2, str(gi.esquinas))
prueba("y el sentido se invierte", gi.sentido == ANTIHORARIO,
       str(gi.sentido))
prueba("la nueva referencia es el orden invertido",
       gi.orden_esperado == ["azul", "naranja"], str(gi.orden_esperado))
time.sleep(0.45)
par(gi, "azul", "naranja")
prueba("ya en el sentido nuevo, los pares cuentan normal",
       gi.esquinas == 3 and gi.incoherencias == 2, str(gi.esquinas))

# Una linea perdida no debe emparejarse con la esquina siguiente
gp = GestorLineas(dict(lcfg, cierre_max_ms=400))
par(gp, "naranja", "azul")
time.sleep(0.45)
gp.evento_tcs("naranja")          # se pierde el azul de esta esquina
time.sleep(0.45)                  # caduca la espera de la segunda linea
gp.paso_zona()
prueba("la esquina sin su segunda linea caduca", gp.pares_incompletos == 1,
       str(gp.pares_incompletos))
par(gp, "naranja", "azul")        # esquina siguiente, entera
# La del medio no cuenta porque no completo par NI hubo giro; en pista el
# giro siempre ocurre y la cuenta. Lo que importa aqui es que la naranja de
# la esquina siguiente NO se empareja con la azul huerfana de la anterior.
prueba("la esquina siguiente cuenta bien y sin incoherencia",
       gp.esquinas == 2 and gp.incoherencias == 0,
       f"esq={gp.esquinas} incoh={gp.incoherencias}")

print("== el giro fantasma de la azul que llega tarde ==")
# El fallo en pista: naranja abre la esquina y dispara el giro; la azul se
# detecta mal y llega cuando el giro ya termino. Antes se tomaba por la
# primera linea de una esquina NUEVA: otro giro de 90, encima entre pilares,
# y al emparejarse desfasada acababa invirtiendo el sentido de la ronda.
gf = GestorLineas(dict(lcfg, refractario_esquina_ms=200, cierre_max_ms=6000))
gf.evento_tcs("naranja")
prueba("la naranja abre la esquina y entra en la zona", gf.en_esquina)
prueba("todavia no cuenta (falta la segunda linea)", gf.esquinas == 0,
       str(gf.esquinas))
gf.giro_completado(1)             # el giro de 90 se hace y termina
prueba("el giro da la esquina por contada", gf.esquinas == 1, str(gf.esquinas))
prueba("y saca de la zona de esquina", not gf.en_esquina)
prueba("pero SIGUE esperando la segunda linea", gf._esquina_abierta)

time.sleep(0.25)                  # pasa el refractario: antes bastaba para
gf.evento_tcs("azul")             # que la azul abriera OTRA esquina
prueba("la azul tardia CIERRA la esquina, no abre otra",
       not gf._esquina_abierta and not gf.en_esquina,
       f"abierta={gf._esquina_abierta} zona={gf.zona}")
prueba("y no dispara un giro de mas: sigue habiendo UNA esquina",
       gf.esquinas == 1, str(gf.esquinas))
prueba("el sentido no se invierte", gf.sentido == HORARIO, str(gf.sentido))
prueba("ni se apunta una incoherencia", gf.incoherencias == 0,
       str(gf.incoherencias))

# La misma linea repetida (rebote en el borde) tampoco abre nada
gr = GestorLineas(dict(lcfg, refractario_esquina_ms=200))
gr.evento_tcs("naranja")
gr.evento_tcs("naranja")
prueba("la misma linea repetida se ignora",
       gr.esquinas == 0 and gr._color_apertura == "naranja",
       gr.ultimo_evento)

# Cuatro esquinas seguidas perdiendo SIEMPRE la azul: la cuenta sigue bien
gs = GestorLineas(dict(lcfg, refractario_esquina_ms=200, cierre_max_ms=300))
for i in range(4):
    gs.evento_tcs("naranja")
    gs.giro_completado(1)
    time.sleep(0.35)              # caduca la espera y pasa el refractario
    gs.paso_zona()
prueba("sin ver NUNCA la azul, 4 esquinas siguen siendo 4",
       gs.esquinas == 4, str(gs.esquinas))
prueba("y se contabilizan como pares incompletos",
       gs.pares_incompletos == 4, str(gs.pares_incompletos))

print("== zona de esquina (anti-bucle) ==")
zcfg = dict(lcfg, esquina_max_ms=400)
gz = GestorLineas(zcfg)
prueba("arranca en recta", gz.zona == ZONA_RECTA and not gz.en_esquina)
gz.evento_tcs("naranja")
prueba("primera linea => ENTRA en la esquina", gz.en_esquina)
gz.evento_tcs("azul")
prueba("la segunda linea del par NO saca de la esquina", gz.en_esquina)
gz.giro_completado(1)
prueba("giro de 90 completado => SALE de la esquina",
       gz.zona == ZONA_RECTA, gz.motivo_zona)

gz2 = GestorLineas(zcfg)
gz2.evento_tcs("naranja")
gz2.paso_zona()
prueba("sin timeout sigue en la esquina", gz2.en_esquina)
time.sleep(0.45)
gz2.paso_zona()
prueba("timeout de seguridad devuelve a recta (giroscopio caido)",
       gz2.zona == ZONA_RECTA, gz2.motivo_zona)

# ===========================================================================
print("== navegacion ==")
ncfg = params_mod.valores_por_defecto()
nav = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"])
p_ok = muro.perfil(escena(300), geo, mcfg)          # muro lejos-medio
d = nav.paso(p_libre, None, 0)
prueba("libre: avanza", d.vel > 0 and d.estado == RECTO, f"{d.vel} {d.estado}")

# muro encima -> escape en reversa
cerca = escena(int(H * 0.93))
p_cerca = muro.perfil(cerca, geo, mcfg)
nav2 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"])
d2 = nav2.paso(p_cerca, None, 0)
prueba("muro encima: reversa", d2.vel < 0 and d2.estado == ESCAPE,
       f"{d2.vel} {d2.estado} pasillo={p_cerca.pasillo_mm:.0f}")

# muro a media distancia: sigue en recto pero mas despacio que en libre
nav3 = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"])
p_medio = muro.perfil(escena(260), geo, mcfg)
d3a = nav3.paso(p_libre, None, 0)
d3b = nav3.paso(p_medio, None, 0)
prueba("frena al acercarse", d3b.vel <= d3a.vel, f"{d3b.vel} vs {d3a.vel}")

# giro hacia el lado con mas espacio si el pasillo cae (sin sentido conocido)
nav4 = Navegador(dict(ncfg["navegacion"], min_recto_ms=0),
                 ncfg["limites"], ncfg["escape"])
d4 = nav4.paso(p_mitad, None, 0)   # pared a la izquierda
girado = None
if nav4.estado in ("pre_giro", "giro"):
    girado = nav4.lado_giro
    prueba("esquina: gira hacia el lado libre (derecha)", girado == 1,
           str(girado))
else:
    prueba("con a pared a un lado navega o gira", d4.vel != 0, d4.estado)

# ===========================================================================
print("== esquinas: bucle y giro de dos tiempos ==")

def nav_esquina(**cambios_nav):
    """Navegador listo para entrar en esquina de inmediato."""
    v = params_mod.valores_por_defecto()
    v["navegacion"].update(dict(min_recto_ms=0, retardo_giro_ms=0), **cambios_nav)
    hechos = []
    n = Navegador(v["navegacion"], v["limites"], v["escape"], v["giro2t"],
                  al_completar_giro=lambda lado: hechos.append(lado))
    return n, hechos, v

# El escenario del bucle: DENTRO de la curva, sin giroscopio, con el hueco de
# piso blanco enorme que deja el muro interno al acabarse. Para el perfil todo
# esta despejado, asi que la vision dice "sigue recto" justo cuando hay que
# girar. Ese hueco es el falso camino que hacia dar vueltas al carro.
sin_bloqueo, hechos_sb, _ = nav_esquina(bloqueo_esquina=False)
estados_sb = []
for _ in range(8):
    d = sin_bloqueo.paso(p_libre, None, 1, en_esquina=True)
    estados_sb.append(d.estado)
giro_sb = estados_sb.count(GIRO)
prueba("SIN anti-bucle el hueco blanco aborta el giro nada mas empezarlo",
       giro_sb == 0 and sin_bloqueo.estado == RECTO,
       f"ticks en giro={giro_sb} estado={sin_bloqueo.estado} {estados_sb}")
prueba("SIN anti-bucle el carro se va recto hacia el hueco (no dobla)",
       abs(sin_bloqueo.ultimo.direccion) < 30 and sin_bloqueo.ultimo.vel > 0,
       f"dir={sin_bloqueo.ultimo.direccion} vel={sin_bloqueo.ultimo.vel}")

con_bloqueo, hechos_cb, _ = nav_esquina(bloqueo_esquina=True)
estados_cb = []
for _ in range(8):
    d = con_bloqueo.paso(p_libre, None, 1, en_esquina=True)
    estados_cb.append(d.estado)
prueba("CON anti-bucle el giro se mantiene aunque vea hueco libre",
       len(hechos_cb) == 0 and con_bloqueo.estado == GIRO,
       f"giros={len(hechos_cb)} estado={con_bloqueo.estado} {estados_cb}")
prueba("CON anti-bucle el carro esta doblando de verdad",
       abs(con_bloqueo.ultimo.direccion) > 50,
       f"dir={con_bloqueo.ultimo.direccion}")
prueba("el motivo avisa de que esta en esquina",
       "en esquina" in con_bloqueo.ultimo.motivo, con_bloqueo.ultimo.motivo)

# Una esquina, UN giro: aunque la zona de curva siga activa mucho rato (el
# TCS no vio la segunda linea, el timeout es largo), no se encadenan giros.
una_vez, hechos_uv, _ = nav_esquina()
for _ in range(60):
    una_vez.paso(p_libre, 0.0, 1, en_esquina=True)   # yaw quieto: no completa
prueba("la misma curva no vuelve a disparar el giro",
       len(hechos_uv) == 0 and una_vez.estado in (GIRO, PRE_GIRO),
       f"giros={len(hechos_uv)} estado={una_vez.estado}")

# La linea del piso entra en la esquina por si sola, sin esperar al pasillo
solo_linea, _h, _v = nav_esquina()
d = solo_linea.paso(p_libre, None, 1, en_esquina=True)
prueba("la linea del piso dispara la esquina con el frente despejado",
       solo_linea.estado in (PRE_GIRO, GIRO), solo_linea.estado)

# --- giro de dos tiempos, cableado como en el robot de verdad -------------
def curva_completa(activo_2t: bool, pasos: int = 400, k: float = 9.0):
    """Esquina entera: cruzar la linea -> girar -> salir de la esquina.

    El gestor de lineas y el navegador van conectados igual que en robot.py,
    asi que esto prueba el lazo completo, no las piezas por separado.
    Modelo Ackermann: la rotacion va con signo(vel)*signo(direccion), asi que
    retroceder con el volante al reves sigue girando hacia el mismo lado.
    """
    v = params_mod.valores_por_defecto()
    v["navegacion"].update(min_recto_ms=0, retardo_giro_ms=0)
    v["giro2t"]["activo"] = activo_2t
    gl = GestorLineas(dict(lcfg, esquina_max_ms=60000))
    nav = Navegador(v["navegacion"], v["limites"], v["escape"], v["giro2t"],
                    al_completar_giro=gl.giro_completado)
    gl.evento_tcs("naranja")               # cruza la primera linea del par
    yaw = 0.0
    traza = []
    for _ in range(pasos):
        gl.paso_zona()
        d = nav.paso(p_libre, yaw, 1, en_esquina=gl.en_esquina,
                     esquina_confirmada=gl.en_esquina)
        traza.append((nav.estado, d.vel, d.direccion, yaw))
        yaw = ((yaw + k * (d.vel / 100.0) * (d.direccion / 100.0)) + 180) % 360 - 180
        if not gl.en_esquina and nav.estado == RECTO:
            break
    return traza, yaw, gl, nav

traza, yaw_fin, gl2t, n2t = curva_completa(True)
estados = [t[0] for t in traza]
prueba("entra en el giro de dos tiempos", GIRO_2T in estados, str(set(estados)))
avances = [t for t in traza if t[0] == GIRO_2T and t[1] > 0]
reversas = [t for t in traza if t[0] == GIRO_2T and t[1] < 0]
prueba("tiene tramo de avance", len(avances) > 0, str(len(avances)))
prueba("tiene tramo de reversa", len(reversas) > 0, str(len(reversas)))
prueba("en reversa la direccion va al lado CONTRARIO del avance",
       avances[0][2] > 0 and reversas[0][2] < 0,
       f"avance dir={avances[0][2]} reversa dir={reversas[0][2]}")
prueba("la maniobra termina y vuelve a recto", n2t.estado == RECTO, n2t.estado)
prueba("giro de 90 grados completado", 80 <= yaw_fin <= 105, f"{yaw_fin:.1f}")
prueba("al terminar el giro SALE de la esquina", not gl2t.en_esquina,
       gl2t.zona)
prueba("la esquina se conto una sola vez", gl2t.esquinas == 1,
       str(gl2t.esquinas))

# la misma curva con el giro normal de un tiempo: tambien sale y cuenta una
traza1, yaw1, gl1, n1 = curva_completa(False)
prueba("giro normal: tambien completa los 90", 80 <= yaw1 <= 105, f"{yaw1:.1f}")
prueba("giro normal: sale de la esquina y cuenta una",
       not gl1.en_esquina and gl1.esquinas == 1,
       f"{gl1.zona} {gl1.esquinas}")
prueba("el de un tiempo no retrocede", all(t[1] >= 0 for t in traza1),
       "hay reversa en el giro normal")

# sin giroscopio el 2T no se puede medir: se usa el giro normal
n2t_sin, _h, _v = nav_esquina()
n2t_sin.g2t["activo"] = True
for _ in range(3):
    n2t_sin.paso(p_libre, None, 1, en_esquina=True)
prueba("sin giroscopio cae al giro normal", n2t_sin.estado == GIRO,
       n2t_sin.estado)

# el escape no puede secuestrar la maniobra de dos tiempos
n2t_cerca, _h, _v = nav_esquina()
n2t_cerca.g2t["activo"] = True
for _ in range(2):
    n2t_cerca.paso(p_libre, 0.0, 1, en_esquina=True,
                   esquina_confirmada=True)              # entrar en 2T
est_antes = n2t_cerca.estado
d = n2t_cerca.paso(p_cerca, 1.0, 1, en_esquina=True,
                   esquina_confirmada=True)              # muro encima
prueba("con muro delante el 2T corta a reversa, no lo secuestra el escape",
       est_antes == GIRO_2T and n2t_cerca.estado == GIRO_2T and d.estado != ESCAPE,
       f"{est_antes} -> {n2t_cerca.estado} ({d.motivo})")

print("== la reversa del 2T solo dentro de una esquina ==")
# El giro de dos tiempos es lo unico que retrocede. Retroceder en mitad de una
# recta (porque la vision creyo ver una esquina donde no la hay) es meterse
# contra lo que venga detras, asi que exige la prueba fisica: el par de lineas.

def rodar(nav, pasos, confirmada, yaw0=0.0, k=9.0, perfil=None):
    """Simula 'pasos' ticks. 'confirmada' puede ser un bool o una funcion del
    numero de tick, para poder quitar la confirmacion a mitad."""
    perfil = perfil if perfil is not None else p_libre
    yaw = yaw0
    traza = []
    for i in range(pasos):
        conf = confirmada(i) if callable(confirmada) else confirmada
        d = nav.paso(perfil, yaw, 1, en_esquina=True, esquina_confirmada=conf)
        traza.append((nav.estado, d.vel, d.direccion))
        yaw = ((yaw + k * (d.vel / 100.0) * (d.direccion / 100.0)) + 180) % 360 - 180
    return traza, yaw

# --- sin confirmacion por lineas: NI UN SOLO frame de reversa -------------
sin_conf, _h, _v = nav_esquina()
sin_conf.g2t["activo"] = True
traza_sc, yaw_sc = rodar(sin_conf, 120, confirmada=False)
prueba("sin lineas NO entra en el giro de dos tiempos",
       all(t[0] != GIRO_2T for t in traza_sc), str({t[0] for t in traza_sc}))
prueba("sin lineas NO retrocede en ningun momento",
       all(t[1] >= 0 for t in traza_sc),
       str([t for t in traza_sc if t[1] < 0][:3]))
prueba("hace el giro normal, hacia adelante", GIRO in [t[0] for t in traza_sc])
prueba("y aun asi completa los 90 grados", 80 <= yaw_sc <= 105, f"{yaw_sc:.1f}")

# --- con confirmacion: la maniobra se hace entera -------------------------
con_conf, _h, _v = nav_esquina()
con_conf.g2t["activo"] = True
traza_cc, yaw_cc = rodar(con_conf, 400, confirmada=True)
reversas = [t for t in traza_cc if t[0] == GIRO_2T and t[1] < 0]
prueba("con lineas SI entra en el giro de dos tiempos",
       any(t[0] == GIRO_2T for t in traza_cc))
prueba("y retrocede (esa es la maniobra)", len(reversas) > 0, str(len(reversas)))
prueba("toda la reversa ocurre en la maniobra, no suelta",
       all(t[0] == GIRO_2T for t in traza_cc if t[1] < 0))

# --- se pierde la confirmacion a mitad: termina hacia adelante ------------
mitad, _h, _v = nav_esquina()
mitad.g2t["activo"] = True
# confirmada solo los primeros 6 ticks: justo despues caduca la zona
traza_m, yaw_m = rodar(mitad, 200, confirmada=lambda i: i < 6)
tras_perder = traza_m[6:]
prueba("al caducar la zona deja de retroceder de inmediato",
       all(t[1] >= 0 for t in tras_perder),
       str([t for t in tras_perder if t[1] < 0][:3]))
prueba("y pasa al giro normal para acabar",
       any(t[0] == GIRO for t in tras_perder), str({t[0] for t in tras_perder}))
prueba("terminando igualmente el giro de 90", 80 <= yaw_m <= 110, f"{yaw_m:.1f}")

# --- el escape de seguridad NO queda capado -------------------------------
# Es la ultima red contra un choque: tiene que poder retroceder este donde
# este, tambien en mitad de una recta.
esc_nav, _h, _v = nav_esquina()
d_esc = esc_nav.paso(p_cerca, None, 1, en_esquina=False, esquina_confirmada=False)
prueba("el ESCAPE sigue pudiendo retroceder fuera de una esquina",
       d_esc.estado == ESCAPE and d_esc.vel < 0,
       f"{d_esc.estado} vel={d_esc.vel}")

# ===========================================================================
print("== esquina por color: conteo ==")
### El modo nuevo. El TCS fija el sentido con la PRIMERA linea, desde ahi
### solo cuenta ese color, y el giro hacia adentro se suelta al ver un pilar.
import dataclasses                                   # noqa: E402
from src.navegacion import GIRO_COLOR                # noqa: E402
from src.carrera import Carrera, PARANDO, TERMINADO, CORRIENDO  # noqa: E402

ccfg = dict(params_mod.valores_por_defecto()["esquina_color"],
            activo=True, refractario_ms=300, reanudar_tras_ms=0)

gc = GestorLineas(dict(lcfg, esquina_max_ms=60000), cfg_color=ccfg)
prueba("color: sin lineas no hay color objetivo", gc.color_objetivo() == "")
ok_n = gc.evento_tcs("naranja")
prueba("color: la primera naranja fija horario y cuenta 1",
       ok_n and gc.sentido == HORARIO and gc.esquinas == 1,
       f"{gc.sentido} {gc.esquinas}")
prueba("color: y entra en la zona de esquina", gc.en_esquina)
prueba("color: deja UN disparo para el navegador",
       gc.tomar_disparo() and not gc.tomar_disparo())
ok_a = gc.evento_tcs("azul")
prueba("color: la azul se ignora del todo (ni cuenta ni es 'reciente')",
       not ok_a and gc.esquinas == 1 and gc.ignoradas == 1, gc.ultimo_evento)
prueba("color: la azul no deja disparo", not gc.tomar_disparo())
gc.evento_tcs("naranja")                    # dentro del refractario
prueba("color: la misma naranja pisada otra vez no cuenta",
       gc.esquinas == 1 and gc.rebotes == 1, gc.ultimo_evento)
time.sleep(0.35)
gc.evento_tcs("naranja")
prueba("color: pasado el refractario, la naranja siguiente cuenta",
       gc.esquinas == 2, str(gc.esquinas))
gc.giro_completado(1)
prueba("color: el giro completado NO suma esquina", gc.esquinas == 2,
       str(gc.esquinas))
prueba("color: pero si saca de la zona", not gc.en_esquina, gc.zona)
gc.evento_tcs("azul")
gc.evento_tcs("azul")
prueba("color: mas azules, mismo marcador",
       gc.esquinas == 2 and gc.ignoradas == 3, str(gc.ignoradas))

ga = GestorLineas(lcfg, cfg_color=ccfg)
ga.evento_tcs("azul")
prueba("color: primera azul => antihorario y cuenta las azules",
       ga.sentido == ANTIHORARIO and ga.esquinas == 1
       and ga.color_objetivo() == "azul", ga.color_objetivo())
ga.evento_tcs("naranja")
prueba("color: en antihorario la naranja se ignora", ga.esquinas == 1)

# sentido forzado desde la web: manda sobre lo que diga la primera linea
gf2 = GestorLineas(lcfg, cfg_color=ccfg)
gf2.sentido_forzado = "horario"
gf2.evento_tcs("azul")
prueba("color: forzado horario, la azul no cuenta ni siendo la primera",
       gf2.esquinas == 0 and gf2.color_objetivo() == "naranja",
       gf2.ultimo_evento)
gf2.evento_tcs("naranja")
prueba("color: ...y la naranja si", gf2.esquinas == 1)

gg = GestorLineas(lcfg, cfg_color=dict(ccfg, contar_giro_sin_linea=True))
gg.giro_completado(1)
prueba("color: con contar_giro_sin_linea el giro suelto cuenta",
       gg.esquinas == 1, str(gg.esquinas))

gpar = GestorLineas(lcfg)
prueba("modo par: evento_tcs sigue devolviendo True (linea reciente)",
       gpar.evento_tcs("azul") is True and gpar.evento_tcs("azul") is True)

print("== esquina por color: el giro hacia adentro ==")

def nav_color(**cambios):
    """Gestor + navegador en modo color, cableados como en robot.py."""
    v = params_mod.valores_por_defecto()
    v["navegacion"].update(min_recto_ms=0, retardo_giro_ms=0)
    v["esquina_color"].update(dict(activo=True, reanudar_tras_ms=0), **cambios)
    gl = GestorLineas(dict(lcfg, esquina_max_ms=60000),
                      cfg_color=v["esquina_color"])
    n = Navegador(v["navegacion"], v["limites"], v["escape"], v["giro2t"],
                  al_completar_giro=gl.giro_completado,
                  cfg_color=v["esquina_color"])
    return n, gl, v

def curva_color(nav, gl, pilar=False, pasos=400, k=9.0, perfil=None):
    """Esquina entera en modo color. 'pilar' es un bool o una funcion del
    tick que dice si hay pilar en juego. Mismo modelo Ackermann que
    curva_completa. Traza: (estado, vel, dir, yaw, pilar)."""
    perfil = perfil if perfil is not None else p_libre
    yaw = 0.0
    traza = []
    for i in range(pasos):
        gl.paso_zona()
        if gl.tomar_disparo():
            nav.rearmar_esquina()
        pe = bool(pilar(i)) if callable(pilar) else bool(pilar)
        d = nav.paso(perfil, yaw, gl.sentido_efectivo(),
                     en_esquina=gl.en_esquina, esquina_confirmada=gl.en_esquina,
                     pilar_en_juego=pe)
        traza.append((nav.estado, d.vel, d.direccion, yaw, pe, d.motivo))
        yaw = ((yaw + k * (d.vel / 100.0) * (d.direccion / 100.0)) + 180) % 360 - 180
        if not gl.en_esquina and nav.estado == RECTO and not nav._color_pendiente:
            break
    return traza, yaw

# --- sin pilares: cruzar la naranja, girar a la derecha, salir ------------
n1, g1, _ = nav_color()
g1.evento_tcs("naranja")
tr1, yaw1 = curva_color(n1, g1)
est1 = [t[0] for t in tr1]
prueba("color: la naranja dispara el giro por color", GIRO_COLOR in est1,
       str(set(est1)))
prueba("color: horario => gira a la DERECHA",
       all(t[2] > 0 for t in tr1 if t[0] == GIRO_COLOR),
       str([t[2] for t in tr1 if t[0] == GIRO_COLOR][:5]))
prueba("color: completa los 90 y vuelve a recto",
       80 <= yaw1 <= 105 and n1.estado == RECTO, f"{yaw1:.1f} {n1.estado}")
prueba("color: al terminar sale de la zona", not g1.en_esquina, g1.zona)
prueba("color: la esquina se conto UNA vez", g1.esquinas == 1, str(g1.esquinas))
prueba("color: nunca retrocede", all(t[1] >= 0 for t in tr1))
prueba("color: no usa el giro normal ni el 2T",
       GIRO not in est1 and GIRO_2T not in est1, str(set(est1)))

# aunque el 2T este encendido, en modo color no se usa
n2t, g2tc, _ = nav_color()
n2t.g2t["activo"] = True
g2tc.evento_tcs("naranja")
tr2t, _y = curva_color(n2t, g2tc)
prueba("color: con giro2t.activo encendido sigue mandando el giro por color",
       GIRO_2T not in [t[0] for t in tr2t] and all(t[1] >= 0 for t in tr2t))

# antihorario: primera azul => giro a la izquierda
n3, g3, _ = nav_color()
g3.evento_tcs("azul")
tr3, yaw3 = curva_color(n3, g3)
prueba("color: antihorario => gira a la IZQUIERDA",
       all(t[2] < 0 for t in tr3 if t[0] == GIRO_COLOR) and -105 <= yaw3 <= -80,
       f"{yaw3:.1f}")

# --- velocidad variable con el pasillo ------------------------------------
n4, g4, _ = nav_color()
g4.evento_tcs("naranja")
for _ in range(2):
    n4.paso(p_libre, 0.0, 1, en_esquina=True)      # entrar en GIRO_COLOR
p_450 = dataclasses.replace(p_libre, pasillo_mm=450.0)
d_libre = n4.paso(p_libre, 0.0, 1, en_esquina=True)
d_450 = n4.paso(p_450, 0.0, 1, en_esquina=True)
prueba("color: en el giro (estado correcto)", n4.estado == GIRO_COLOR, n4.estado)
prueba("color: velocidad variable: con el pasillo cerrandose va mas despacio",
       0 < d_450.vel < d_libre.vel, f"{d_450.vel} < {d_libre.vel}")

# --- el pilar SUELTA el giro en el acto y luego se retoma ------------------
n5, g5, _ = nav_color()
g5.evento_tcs("naranja")
tr5, yaw5 = curva_color(n5, g5, pilar=lambda i: 6 <= i < 40)
est5 = [t[0] for t in tr5]
prueba("color: empieza girando", GIRO_COLOR in est5[:6], str(est5[:6]))
prueba("color: el pilar suelta el giro EN EL MISMO tick",
       tr5[6][0] == RECTO, f"tick 6 -> {tr5[6][0]}")
prueba("color: mientras hay pilar nunca esta girando por color",
       all(t[0] != GIRO_COLOR for t in tr5 if t[4]),
       str([t[0] for t in tr5 if t[4]][:5]))
prueba("color: el motivo avisa de que el giro quedo suelto",
       all("suelto" in t[5] for t in tr5 if t[0] == RECTO and t[4]),
       str([t[5] for t in tr5 if t[0] == RECTO and t[4]][:2]))
tras = [t[0] for t in tr5[40:]]
prueba("color: sin pilar RETOMA el giro hacia adentro", GIRO_COLOR in tras,
       str(set(tras)))
prueba("color: y aun asi completa los 90", 80 <= yaw5 <= 105, f"{yaw5:.1f}")
prueba("color: una esquina, una cuenta, aunque se soltara",
       g5.esquinas == 1 and not g5.en_esquina, f"{g5.esquinas} {g5.zona}")
prueba("color: con pilar tampoco retrocede", all(t[1] >= 0 for t in tr5))
prueba("color: el rumbo objetivo se avanzo UNA sola vez (no 180)",
       n5.rumbo_recta is not None and 80 <= _norm(n5.rumbo_recta) <= 100,
       str(n5.rumbo_recta))

# reanudar apagado: tras el pilar no vuelve a GIRO_COLOR; el yaw en recta
# acaba encarando la recta y la zona se cierra sola
n6, g6, _ = nav_color(reanudar=False)
g6.evento_tcs("naranja")
tr6, yaw6 = curva_color(n6, g6, pilar=lambda i: 6 <= i < 40, pasos=600)
prueba("color: reanudar=False no vuelve al giro por color",
       GIRO_COLOR not in [t[0] for t in tr6[6:]], str(set(t[0] for t in tr6[6:])))
prueba("color: reanudar=False: el giroscopio en recta termina de encarar",
       80 <= yaw6 <= 105 and not g6.en_esquina and g6.esquinas == 1,
       f"{yaw6:.1f} {g6.zona}")

# ceder_al_pilar apagado: el giro va comprometido como el normal
n7, g7, _ = nav_color(ceder_al_pilar=False)
g7.evento_tcs("naranja")
tr7, yaw7 = curva_color(n7, g7, pilar=True)
prueba("color: ceder_al_pilar=False ignora al pilar y gira igual",
       GIRO_COLOR in [t[0] for t in tr7] and 80 <= yaw7 <= 105, f"{yaw7:.1f}")

# pilar ya en el pre-giro: ni empieza
n8, g8, v8 = nav_color()
v8["navegacion"]["retardo_giro_ms"] = 500
g8.evento_tcs("naranja")
n8.rearmar_esquina()
n8.paso(p_libre, 0.0, 1, en_esquina=True)                  # -> PRE_GIRO
prueba("color: entra en pre-giro", n8.estado == PRE_GIRO, n8.estado)
d8 = n8.paso(p_libre, 0.0, 1, en_esquina=True, pilar_en_juego=True)
prueba("color: un pilar en el pre-giro lo suelta en el acto",
       n8.estado == RECTO and d8.estado == RECTO and n8._color_pendiente,
       f"{n8.estado} {d8.motivo}")

# el escape sigue por encima del giro por color, y al volver se retoma
n9, g9, _ = nav_color()
g9.evento_tcs("naranja")
for _ in range(2):
    n9.paso(p_libre, 0.0, 1, en_esquina=True)
prueba("color: (preparacion) esta girando", n9.estado == GIRO_COLOR, n9.estado)
d9 = n9.paso(p_cerca, 0.0, 1, en_esquina=True)
prueba("color: con el muro encima manda el ESCAPE (retrocede)",
       d9.estado == ESCAPE and d9.vel < 0, f"{d9.estado} {d9.vel}")
prueba("color: y el giro queda pendiente para retomarlo",
       n9._color_pendiente)

# la azul tardia, ya con el giro hecho, no dispara nada
n10, g10, _ = nav_color()
g10.evento_tcs("naranja")
curva_color(n10, g10)
est_antes = n10.estado
g10.evento_tcs("azul")
prueba("color: la azul tardia no deja disparo ni abre zona",
       not g10.tomar_disparo() and not g10.en_esquina and g10.esquinas == 1)
for _ in range(5):
    n10.paso(p_libre, 90.0, 1, en_esquina=g10.en_esquina)
prueba("color: ...y el carro sigue recto", n10.estado == RECTO == est_antes,
       n10.estado)

# vision_dispara apagado: el pasillo cerrandose ya no dispara giros
n11, g11, _ = nav_color(vision_dispara=False)
d11 = n11.paso(p_mitad, None, 0)
prueba("color: vision_dispara=False: el pasillo no dispara la esquina",
       n11.estado == RECTO and d11.estado == RECTO, n11.estado)

print("== esquina por color: parada en meta ==")
# La esquina se cuenta al PISAR la linea, antes de girar: la carrera tiene
# que esperar a que el giro termine antes de arrancar la parada final.
gcar = GestorLineas(dict(lcfg, esquina_max_ms=60000), cfg_color=ccfg)
car = Carrera({"vueltas": 1, "esquinas_por_vuelta": 1, "parada_ms": 0,
               "sentido": "auto", "autostop": True, "tiempo_max_s": 180}, gcar)
car.arrancar()
gcar.evento_tcs("naranja")
prueba("meta: la linea ya suma la ultima esquina", gcar.esquinas == 1)
prueba("meta: pero con el giro a medias NO se para",
       car.paso() is False and car.estado == CORRIENDO, car.estado)
gcar.giro_completado(1)
prueba("meta: al salir de la esquina arranca la parada",
       car.paso() is True and car.estado == TERMINADO, car.estado)
prueba("meta: el sentido forzado llega al gestor de lineas",
       gcar.sentido_forzado == "auto")

# ===========================================================================
print("== la linea que el TCS se perdio ==")
### Los dos fallos de pista de septiembre, que son EL MISMO fallo visto por
### dos lados: el TCS no veia la linea azul (c_min por encima de su canal
### claro), y en modo color el conteo y el sentido colgaban solo del TCS.
###   - en horario: cada naranja perdida es una esquina que no suma, la
###     carrera no llega nunca a la meta de esquinas y el carro no se para;
###   - en antihorario: perdida la azul de ENTRADA, la naranja de SALIDA
###     declaraba "horario" y el carro doblaba a la derecha toda la ronda.
cfg_red = dict(ccfg, contar_giro_sin_linea=True, refractario_ms=300)
lin_red = dict(lcfg, esquina_max_ms=60000)

# --- el giro como red: cuenta la esquina que la linea no conto -------------
gr = GestorLineas(lin_red, cfg_color=cfg_red)
gr.evento_tcs("naranja")                       # esquina 1: la linea la cuenta
time.sleep(0.35)                               # el giro acaba DESPUES del
gr.giro_completado(1)                          # refractario, como en pista
prueba("red: la esquina que ya conto la linea no se cuenta dos veces",
       gr.esquinas == 1, f"{gr.esquinas} {gr.ultimo_evento}")
time.sleep(0.35)
gr.giro_completado(1)                          # esquina 2: SIN linea (perdida)
prueba("red: la esquina sin linea la cuenta el giro",
       gr.esquinas == 2, f"{gr.esquinas} {gr.ultimo_evento}")
time.sleep(0.35)
gr.giro_completado(-1)                         # giro contra el sentido
prueba("red: un giro hacia el lado contrario no cuenta esquina",
       gr.esquinas == 2, gr.ultimo_evento)
gapag = GestorLineas(lin_red, cfg_color=dict(cfg_red,
                                             contar_giro_sin_linea=False))
gapag.giro_completado(1)
prueba("red: apagada, el giro sigue sin contar", gapag.esquinas == 0)

# --- y con la red, la carrera SI llega a la meta y para --------------------
gm = GestorLineas(lin_red, cfg_color=cfg_red)
carm = Carrera({"vueltas": 1, "esquinas_por_vuelta": 4, "parada_ms": 0,
                "sentido": "auto", "autostop": True, "tiempo_max_s": 180}, gm)
carm.arrancar()
for i in range(4):
    if i != 2:                                 # en la 3a el TCS se pierde la linea
        gm.evento_tcs("naranja")
    time.sleep(0.35)
    gm.giro_completado(1)
    carm.paso()
prueba("meta: con una linea perdida la carrera igual llega a las 4 esquinas",
       gm.esquinas == 4, str(gm.esquinas))
prueba("meta: y el carro se para", carm.paso() is True and carm.estado == TERMINADO,
       carm.estado)

# --- el sentido: la camara vota antes de pisar nada ------------------------
from src.vision import Deteccion as _DetL                    # noqa: E402

def _linea_suelo(dist_mm, color):
    """Deteccion sintetica de una linea del piso a esa distancia del morro."""
    fila = int(geo.distancia_a_fila(dist_mm + gcfg["morro_mm"]))
    return _DetL(color=color, x=220, y=fila - 8, w=200, h=8, area=1600,
                 llenado=1.0, aspecto=0.04, cx=320.0, cy=float(fila - 4))

# antihorario: mirando de frente, la AZUL esta mas cerca (se cruza primero)
gs = GestorLineas(dict(lin_red, usar_camara=False, sugerencia_votos=3),
                  cfg_color=cfg_red)
dets_anti = {"azul": [_linea_suelo(700.0, "azul")],
             "naranja": [_linea_suelo(1100.0, "naranja")]}
gs.paso_camara(dets_anti, None, geo)
prueba("sentido: un solo cuadro no basta para sugerir",
       gs.sugerencia == 0, str(gs.sugerencia))
gs.paso_camara(dets_anti, None, geo)
gs.paso_camara(dets_anti, None, geo)
prueba("sentido: tres cuadros de acuerdo => la camara sugiere antihorario",
       gs.sugerencia == ANTIHORARIO, str(gs.sugerencia))
prueba("sentido: sugerir no es contar (usar_camara apagado)",
       gs.esquinas == 0 and gs.dist_lineas == {} and not gs.en_esquina)
gs.evento_tcs("naranja")          # el TCS se perdio la azul y ve la de salida
prueba("sentido: la sugerencia manda sobre la linea pisada",
       gs.sentido == ANTIHORARIO and gs.color_objetivo() == "azul",
       f"{gs.sentido} {gs.color_objetivo()}")
prueba("sentido: y esa naranja de salida no cuenta esquina",
       gs.esquinas == 0 and gs.ignoradas == 1, gs.ultimo_evento)
gs.evento_tcs("azul")
prueba("sentido: la azul siguiente si cuenta", gs.esquinas == 1)

# sin camara (o sin ver el par) todo sigue como antes: manda la linea pisada
gsin = GestorLineas(dict(lin_red, usar_camara=False), cfg_color=cfg_red)
gsin.evento_tcs("naranja")
prueba("sentido: sin sugerencia sigue mandando la primera linea pisada",
       gsin.sentido == HORARIO and gsin.esquinas == 1)

# --- el cronometro del reglamento no depende del autostop ------------------
gt = GestorLineas(lin_red, cfg_color=cfg_red)
cart = Carrera({"vueltas": 3, "esquinas_por_vuelta": 4, "parada_ms": 0,
                "sentido": "auto", "autostop": False, "tiempo_max_s": 180}, gt)
cart.arrancar()
prueba("tiempo: con autostop apagado no para por vueltas",
       cart.paso() is False, cart.estado)
cart.t_inicio -= 200.0                          # como si llevara 200 s
prueba("tiempo: pero el tope de 3 minutos si corta igual",
       cart.paso() is True and cart.estado == TERMINADO, cart.estado)

# --- el perfil GUARDADO tiene que dejar pasar la linea azul real -----------
### Esta es la comprobacion que nos faltaba: el codigo estaba bien y el perfil
### que corria en el carro llevaba c_min=842, tres veces mas alto que el canal
### claro de la linea azul (678). Se descartaba antes de mirar el color.
import json as _json                                        # noqa: E402
from src.lineas import clase_tcs as _clase_tcs               # noqa: E402

# C, R, G, B leidos por el equipo con el sensor sobre la linea azul de la
# pista (ratios r=70, b=107; el blanco da ~85 en los dos).
_AZUL_PISTA = (678, 186, 231, 284)
_BLANCO_PISTA = (3368, 1120, 1120, 1128)
_pj = _json.loads((RAIZ / "config" / "params.json").read_text(encoding="utf-8"))
_act = [x for x in _pj["perfiles"] if x["nombre"] == _pj["activo"]][0]
_tcs_activo = _act["valores"]["tcs"]
prueba("perfil activo: la linea azul medida en pista se clasifica azul",
       _clase_tcs(*_AZUL_PISTA, _tcs_activo) == "azul",
       f"c_min={_tcs_activo['c_min']} -> {_clase_tcs(*_AZUL_PISTA, _tcs_activo)}")
prueba("perfil activo: y el piso blanco sigue sin clasificarse",
       _clase_tcs(*_BLANCO_PISTA, _tcs_activo) == "-",
       _clase_tcs(*_BLANCO_PISTA, _tcs_activo))
# CON EL CARRO EN MARCHA casi ninguna muestra cae entera sobre la linea: el
# sensor integra 24 ms y en ese rato el borde se mueve, asi que la ventana
# mezcla linea y piso. Y la mezcla no es mitad y mitad en los RATIOS: el piso
# blanco devuelve cinco veces mas luz, asi que domina la lectura y aplasta la
# diferencia b-r. Aqui, con la ventana un 75 % sobre la linea, la diferencia
# cae de 36 a 14: con azul_dif_min en 18 esa muestra se perdia aunque el
# sensor estuviera justo encima de la linea. Ese es el "no ve el azul en
# movimiento" que se veia en pista.
_AZUL_MEZCLA = (1350, 419, 453, 495)      # 75 % linea azul, 25 % piso
prueba("perfil activo: muestra de azul a medio borde (carro en marcha) entra",
       _clase_tcs(*_AZUL_MEZCLA, _tcs_activo) == "azul",
       str(_clase_tcs(*_AZUL_MEZCLA, _tcs_activo)))
prueba("(control) con los umbrales del perfil viejo esa muestra se perdia",
       _clase_tcs(*_AZUL_MEZCLA, {"c_min": 842, "azul_dif_min": 18,
                                  "azul_b_min": 90, "azul_r_max": 121,
                                  "naranja_dif_min": 30, "naranja_r_min": 130,
                                  "naranja_b_max": 85}) == "-")

# ===========================================================================
print("== el rumbo que se quedo atras (giro al lado incorrecto) ==")
### Visto en pista: el TCS se pierde la linea, el centrado toma la curva solo
### por el hueco blanco, y el codigo sigue creyendo que la recta es la vieja.
### El yaw tira al 45 % hacia ella = al lado contrario, hasta la pared. Y en
### la linea siguiente el objetivo sale 90 grados desfasado.

def nav_recta(**cambios_nav):
    v = params_mod.valores_por_defecto()
    v["navegacion"].update(dict(min_recto_ms=0, retardo_giro_ms=0, reanclar_ms=0),
                           **cambios_nav)
    n = Navegador(v["navegacion"], v["limites"], v["escape"], v["giro2t"])
    n.paso(p_libre, 0.0, 1)             # fija la recta de referencia en 0
    return n

# --- control: asi se comportaba (re-anclaje apagado) ------------------------
viejo = nav_recta(reanclar_rumbo=False)
d_viejo = None
for _ in range(10):
    d_viejo = viejo.paso(p_libre, 90.0, 1)      # doblo solo a la derecha
prueba("(control) sin re-anclar, tras doblar solo el yaw tira a la IZQUIERDA",
       d_viejo.direccion <= -30 and viejo.estado == RECTO,
       f"dir={d_viejo.direccion} {d_viejo.motivo}")
prueba("(control) la recta de referencia se quedo en la vieja",
       viejo.rumbo_recta == 0.0, str(viejo.rumbo_recta))

# --- arreglado: adopta la recta nueva y deja de tirar -----------------------
nuevo = nav_recta()
d_nuevo = None
for _ in range(3):
    d_nuevo = nuevo.paso(p_libre, 90.0, 1)
prueba("re-anclado: la recta pasa a ser la real (90)",
       nuevo.rumbo_recta == 90.0 and nuevo.reanclajes == 1,
       f"recta={nuevo.rumbo_recta} reanclajes={nuevo.reanclajes}")
prueba("re-anclado: el volante queda centrado, no tira al otro lado",
       abs(d_nuevo.direccion) < 8 and nuevo.estado == RECTO,
       f"dir={d_nuevo.direccion}")
prueba("re-anclado: el motivo lo cuenta",
       "re-anclado" in d_nuevo.motivo, d_nuevo.motivo)

# el tiempo de sostenimiento (un bandazo de un frame no re-ancla)
tard = nav_recta(reanclar_ms=400)
tard.paso(p_libre, 90.0, 1)
prueba("un frame girado no re-ancla todavia", tard.rumbo_recta == 0.0)
time.sleep(0.45)
tard.paso(p_libre, 90.0, 1)
prueba("sostenido reanclar_ms, si", tard.rumbo_recta == 90.0, str(tard.rumbo_recta))

# torcido EN CONTRA del sentido (esquivando hacia el muro exterior) NO es
# una esquina: la referencia no se toca y el yaw corrige hacia la recta
contra = nav_recta()
d_contra = None
for _ in range(3):
    d_contra = contra.paso(p_libre, -70.0, 1)
prueba("girado en contra del sentido no re-ancla",
       contra.rumbo_recta == 0.0 and contra.reanclajes == 0 and d_contra.direccion > 0,
       f"recta={contra.rumbo_recta} dir={d_contra.direccion}")
# ...y mas alla de desvio_max sigue mandando el rescate de siempre
resc = nav_recta()
d_resc = resc.paso(p_libre, -120.0, 1)
prueba("en contra y pasado desvio_max: rescate, como antes",
       resc.estado == GIRO and "RESCATE" in d_resc.motivo, d_resc.motivo)
# con el sentido DESCONOCIDO no se re-ancla (no se distingue de una vuelta)
sinsent = nav_recta()
for _ in range(3):
    sinsent.paso(p_libre, 90.0, 0)
prueba("sin sentido conocido no re-ancla", sinsent.rumbo_recta == 0.0)
# dos esquinas sin registrar (180): ya no es distinguible de una vuelta
# sobre si mismo -> rescate, como antes
dos = nav_recta()
d_dos = dos.paso(p_libre, 175.0, 1)
prueba("180 girado: no se re-ancla, manda el rescate",
       dos.reanclajes == 0 and dos.estado == GIRO, f"{dos.estado} {d_dos.motivo}")

# --- al apuntar la esquina: el objetivo sale de la recta REAL ----------------
# El carro ya doblo 88 grados solo cuando por fin se registra la linea.
# Antes: objetivo = 0 + 90 = 90 = donde ya esta -> "giro hecho" al instante y
# el yaw tirando a la recta de referencia (que era la vieja).
n_snap, g_snap, _ = nav_color()
n_snap.paso(p_libre, 0.0, 1)                # recta 0
g_snap.evento_tcs("naranja")
n_snap.rearmar_esquina()
n_snap.paso(p_libre, 88.0, 1, en_esquina=True)     # PRE_GIRO
d_snap = n_snap.paso(p_libre, 88.0, 1, en_esquina=True)   # GIRO_COLOR
prueba("al apuntar con 88 girados, la recta base se re-ancla a 90",
       abs(_norm(n_snap.rumbo_recta - 180.0)) < 1e-6 and n_snap.reanclajes == 1,
       f"recta={n_snap.rumbo_recta}")
prueba("y el giro va a la DERECHA (90 mas), no se da por hecho",
       n_snap.estado == GIRO_COLOR and d_snap.direccion > 40,
       f"{n_snap.estado} dir={d_snap.direccion}")
# entrar torcido 30 en contra (esquivando): la base sigue siendo la recta
n_tor, g_tor, _ = nav_color()
n_tor.paso(p_libre, 0.0, 1)
g_tor.evento_tcs("naranja")
n_tor.rearmar_esquina()
n_tor.paso(p_libre, -30.0, 1, en_esquina=True)
n_tor.paso(p_libre, -30.0, 1, en_esquina=True)
prueba("entrando torcido en contra el objetivo sigue siendo 90",
       n_tor.rumbo_recta == 90.0 and n_tor.reanclajes == 0,
       str(n_tor.rumbo_recta))
# el giro normal (modo par) tambien se beneficia
n_par, _h, _v = nav_esquina()
n_par.paso(p_libre, 0.0, 1)
n_par.paso(p_libre, 88.0, 1, en_esquina=True)      # PRE_GIRO
n_par.paso(p_libre, 88.0, 1, en_esquina=True)      # GIRO
prueba("giro normal: tambien apunta desde la recta real",
       abs(_norm(n_par.rumbo_recta - 180.0)) < 1e-6, str(n_par.rumbo_recta))

# --- el yaw falta justo al entrar en la esquina ----------------------------
# enlace.yaw() devuelve None con 0,4 s sin telemetria. Antes el apuntado se
# saltaba y al volver el yaw el giro "terminaba" con la referencia vieja.
n_sin, g_sin, _ = nav_color()
n_sin.paso(p_libre, 0.0, 1)
g_sin.evento_tcs("naranja")
yaw_s = 0.0
traza_s = []
for i in range(400):
    g_sin.paso_zona()
    if g_sin.tomar_disparo():
        n_sin.rearmar_esquina()
    y_in = None if i < 4 else yaw_s                 # 4 ticks sin yaw
    d = n_sin.paso(p_libre, y_in, 1, en_esquina=g_sin.en_esquina,
                   esquina_confirmada=g_sin.en_esquina)
    traza_s.append((n_sin.estado, d.vel, d.direccion))
    yaw_s = ((yaw_s + 9.0 * (d.vel / 100.0) * (d.direccion / 100.0)) + 180) % 360 - 180
    if not g_sin.en_esquina and n_sin.estado == RECTO:
        break
prueba("sin yaw al entrar: al volver el yaw se apunta y se completan los 90",
       80 <= yaw_s <= 105 and n_sin.rumbo_recta == 90.0,
       f"yaw={yaw_s:.1f} recta={n_sin.rumbo_recta}")
n_sin2, _h, _v = nav_esquina()
n_sin2.paso(p_libre, 0.0, 1)
n_sin2.paso(p_libre, None, 1, en_esquina=True)       # PRE_GIRO sin yaw
n_sin2.paso(p_libre, None, 1, en_esquina=True)       # GIRO sin yaw
prueba("giro normal sin yaw: queda pendiente de apuntar",
       n_sin2.estado == GIRO and not n_sin2._apuntado)
n_sin2.paso(p_libre, 5.0, 1, en_esquina=True)
prueba("giro normal: al volver el yaw apunta a 90",
       n_sin2.rumbo_recta == 90.0 and n_sin2._apuntado, str(n_sin2.rumbo_recta))

# ===========================================================================
print("== freno ante linea (camara) ==")
### El TCS integra 24 ms por muestra: a crucero una linea de 2 cm deja 1 o 2
### lecturas y a veces ninguna. La camara la ve antes: se frena desde una
### distancia y se sigue lento un rato despues de que pase bajo el carro.
from src.vision import Deteccion as _Det          # noqa: E402

def linea_vista(dist_mm, color="naranja"):
    """Deteccion sintetica de una linea cuya base cae a esa distancia."""
    v = int(geo.distancia_a_fila(dist_mm + gcfg["morro_mm"]))
    return _Det(color=color, x=220, y=v - 8, w=200, h=8, area=1600,
                llenado=1.0, aspecto=0.04, cx=320.0, cy=float(v - 4))

fcfg = dict(lcfg, usar_camara=False, frenar_ante_linea=True,
            frenar_desde_mm=500.0, vel_linea_pct=60, frenar_tras_ms=300)
gfr = GestorLineas(fcfg)
gfr.paso_camara({"naranja": [linea_vista(900.0)]}, None, geo)
prueba("linea lejos (900): se mide pero no frena",
       gfr.dist_linea_vista is not None and 800 < gfr.dist_linea_vista < 1000
       and gfr.freno_linea() == 1.0,
       f"{gfr.dist_linea_vista} {gfr.freno_linea()}")
gfr.paso_camara({"naranja": [linea_vista(400.0)]}, None, geo)
prueba("linea a 400: frena a 0.6", abs(gfr.freno_linea() - 0.6) < 1e-6,
       str(gfr.freno_linea()))
prueba("con usar_camara apagado la camara NO cuenta ni cruza",
       gfr.esquinas == 0 and gfr.dist_lineas == {} and not gfr.en_esquina)
gfr.paso_camara({}, None, geo)                 # salio por debajo del cuadro
prueba("desaparecida del cuadro: sigue frenado (esta pasando bajo el carro)",
       abs(gfr.freno_linea() - 0.6) < 1e-6 and gfr.dist_linea_vista is None)
time.sleep(0.35)
prueba("pasado frenar_tras_ms vuelve a velocidad normal", gfr.freno_linea() == 1.0)
gfr.evento_tcs("azul")                         # el TCS avisa del cruce
prueba("un cruce del TCS tambien frena un rato",
       abs(gfr.freno_linea() - 0.6) < 1e-6)
goff = GestorLineas(dict(fcfg, frenar_ante_linea=False))
goff.paso_camara({"naranja": [linea_vista(300.0)]}, None, geo)
prueba("frenar_ante_linea apagado: nunca frena", goff.freno_linea() == 1.0)
# una linea vista por encima del muro (no esta en el piso) no cuenta
gmur = GestorLineas(fcfg)
gmur.paso_camara({"naranja": [linea_vista(400.0)]}, p_mitad, geo)
gmur2 = GestorLineas(fcfg)
gmur2.paso_camara({"naranja": [linea_vista(400.0)]}, p_libre, geo)
prueba("con el perfil libre la linea del piso sigue contando",
       gmur2.dist_linea_vista is not None)

# el navegador aplica el factor en recta
nf = Navegador(ncfg["navegacion"], ncfg["limites"], ncfg["escape"])
v_normal = nf.paso(p_libre, None, 0).vel
v_freno = nf.paso(p_libre, None, 0, freno_linea=0.6).vel
prueba("en recta el freno por linea baja la velocidad",
       0 < v_freno < v_normal and abs(v_freno - v_normal * 0.6) <= 1,
       f"{v_freno} vs {v_normal}")

# ===========================================================================
print("== clasificador del TCS (gemelo del firmware) ==")
from src.lineas import clase_tcs, umbrales_desde_muestra   # noqa: E402

# Lectura REAL tomada por el equipo con el sensor sobre la linea azul.
AZUL_REAL = (678, 186, 231, 284)          # C, R, G, B  -> ratios r=70 b=107
prueba("la linea azul real se clasifica como azul",
       clase_tcs(*AZUL_REAL) == "azul", clase_tcs(*AZUL_REAL))
# Con los umbrales viejos (absolutos) fallaba por 3 unidades: 107 < 110.
viejos = {"azul_b_min": 110, "azul_r_max": 70, "azul_dif_min": 0,
          "naranja_dif_min": 0, "naranja_r_min": 120, "naranja_b_max": 60}
prueba("(control) con los umbrales absolutos viejos NO se veia",
       clase_tcs(*AZUL_REAL, viejos) == "-", clase_tcs(*AZUL_REAL, viejos))

prueba("el piso blanco no se clasifica", clase_tcs(900, 300, 300, 300) == "-")
prueba("una naranja tipica se clasifica",
       clase_tcs(700, 380, 180, 140) == "naranja",
       clase_tcs(700, 380, 180, 140))
prueba("sin luz (canal claro bajo) no se clasifica",
       clase_tcs(10, 3, 3, 4) == "-")
prueba("c=0 no revienta", clase_tcs(0, 0, 0, 0) == "-")

# La diferencia no depende de la luz: al doblar la iluminacion, misma clase.
doble = tuple(v * 2 for v in AZUL_REAL)
prueba("con el doble de luz sigue siendo azul", clase_tcs(*doble) == "azul")

# Muestrear sobre la linea deja umbrales que aciertan esa lectura y siguen
# descartando el blanco.
u = umbrales_desde_muestra("azul", 70.0, 107.0, 678.0)
cfg_u = dict(u); cfg_u["c_min"] = 80
prueba("muestrear azul deja umbrales que la reconocen",
       clase_tcs(*AZUL_REAL, cfg_u) == "azul", str(u))
prueba("y que siguen sin coger el blanco",
       clase_tcs(900, 300, 300, 300, cfg_u) == "-")

# El fallo de verdad por el que el TCS no veia la linea azul en pista: el
# perfil guardado tenia c_min=842 (sacado del 25 % de un blanco muy brillante)
# y la linea azul devuelve C=678, porque el color absorbe luz. Se descartaba
# antes de mirar el color siquiera.
PERFIL_EN_PISTA = {"c_min": 842, "naranja_r_min": 130, "naranja_b_max": 85,
                   "azul_b_min": 98, "azul_r_max": 133,
                   "naranja_dif_min": 30, "azul_dif_min": 18}
prueba("(control) con c_min demasiado alto la linea azul se descarta",
       clase_tcs(*AZUL_REAL, PERFIL_EN_PISTA) == "-")
arreglado = dict(PERFIL_EN_PISTA)
arreglado.update(umbrales_desde_muestra("azul", 70.0, 107.0, 678.0))
prueba("muestrear la linea BAJA el c_min y la recupera",
       clase_tcs(*AZUL_REAL, arreglado) == "azul",
       f"c_min={arreglado['c_min']}")
prueba("y el c_min queda por debajo del claro de la linea",
       arreglado["c_min"] < 678, str(arreglado["c_min"]))
blanco_fix = dict(PERFIL_EN_PISTA)
blanco_fix.update(umbrales_desde_muestra("blanco", 85.0, 85.0, 3368.0))
prueba("muestrear el blanco tambien deja sitio a las lineas",
       clase_tcs(*AZUL_REAL, blanco_fix) == "azul",
       f"c_min={blanco_fix['c_min']}")
prueba("y el blanco brillante sigue sin clasificarse",
       clase_tcs(3368, 1120, 1120, 1128, blanco_fix) == "-")

# ===========================================================================
print("== pilares mas alla de la linea del piso ==")
from src.obstaculos import Esquivador                       # noqa: E402
from src.vision import Deteccion                            # noqa: E402

vo = params_mod.valores_por_defecto()
vo["obstaculos"]["activo"] = True
geo_o = Geometria(vo["geometria"], 640, 480)

def pilar(dist_mm, lat_mm=0.0, color="rojo"):
    """Deteccion sintetica de un pilar cuya base cae a esa distancia."""
    u, v = geo_o.suelo_a_pixel(lat_mm, dist_mm + vo["geometria"]["morro_mm"])
    d = Deteccion(color=color, x=u - 15, y=v - 60, w=30, h=60,
                  area=1800, llenado=0.9, aspecto=2.0, cx=float(u), cy=float(v - 30))
    return d

esq = Esquivador(vo["obstaculos"])
cerca = {"rojo": [pilar(600.0, 120.0)]}       # pilar de ESTA recta
lejos = {"rojo": [pilar(1400.0, 120.0)]}      # pilar detras de la linea

def esquivar(dets, lineas=None, en_esq=False, sentido=0, vel=500.0):
    """Cada caso arranca limpio: el compromiso de adelantamiento es estado
    que sobrevive entre llamadas, y aqui se mide una situacion aislada."""
    esq.reiniciar()
    return esq.paso(dets, None, geo_o, lineas, en_esq, sentido, vel)

# sin lineas a la vista, todo cuenta (comportamiento de siempre)
_d, peso = esquivar(lejos)
prueba("sin lineas a la vista, el pilar lejano cuenta", peso > 0, str(peso))

# con la linea a 1100 mm, el pilar de 1400 queda detras: se descarta
_d, peso = esquivar(lejos, {"naranja": 1100.0})
prueba("con la linea delante, el pilar de detras NO cuenta", peso == 0.0,
       f"peso={peso} info={esq.info}")
prueba("y queda anotado en la telemetria", esq.info.get("tras_linea") == 1,
       str(esq.info))

# el pilar de esta recta sigue contando igual
_d, peso = esquivar(cerca, {"naranja": 1100.0})
prueba("el pilar de esta recta sigue contando", peso > 0, str(peso))

# justo antes de la linea, dentro del margen: cuenta
_d, peso = esquivar({"rojo": [pilar(1130.0, 120.0)]}, {"naranja": 1100.0})
prueba("un pilar justo antes de la linea (margen) cuenta", peso > 0, str(peso))

# YA EN LA ESQUINA: el limite se levanta y el pilar del tramo nuevo cuenta
_d, peso = esquivar(lejos, {"naranja": 1100.0}, en_esq=True)
prueba("una vez en la esquina, el pilar de la seccion nueva SI cuenta",
       peso > 0, f"peso={peso} info={esq.info}")

# se puede apagar
vo["obstaculos"]["limitar_por_lineas"] = False
_d, peso = esq.paso(lejos, None, geo_o, {"naranja": 1100.0}, False)
prueba("el filtro se puede apagar", peso > 0, str(peso))
vo["obstaculos"]["limitar_por_lineas"] = True

print("== como se pasa un pilar ==")
# --- el lado de paso NO depende del sentido -------------------------------
# Reglamento 9.19: rojo por la derecha, verde por la izquierda, y son la
# derecha y la izquierda DEL VEHICULO. El mismo pilar fisico se pasa por un
# lado distinto en cada sentido, y eso sale solo de trabajar en el marco del
# carro: no hay que invertir nada.
for sen, nombre in ((1, "horario"), (-1, "antihorario")):
    d_rojo, _ = esquivar({"rojo": [pilar(800.0, 0.0)]}, sentido=sen)
    d_verde, _ = esquivar({"verde": [pilar(800.0, 0.0)]}, sentido=sen)
    prueba(f"{nombre}: el rojo se pasa por su derecha (el carro va a la derecha)",
           d_rojo > 0, f"dir={d_rojo:.0f}")
    prueba(f"{nombre}: el verde se pasa por su izquierda",
           d_verde < 0, f"dir={d_verde:.0f}")

# el interruptor existe por si en su pista se interpreta al reves
vo["obstaculos"]["invertir_en_antihorario"] = True
d_inv, _ = esquivar({"rojo": [pilar(800.0, 0.0)]}, sentido=-1)
prueba("con el interruptor, en antihorario se invierte", d_inv < 0,
       f"dir={d_inv:.0f}")
d_hor, _ = esquivar({"rojo": [pilar(800.0, 0.0)]}, sentido=1)
prueba("y en horario no cambia nada", d_hor > 0, f"dir={d_hor:.0f}")
vo["obstaculos"]["invertir_en_antihorario"] = False

# --- ya no se va al tope de direccion -------------------------------------
# Con los valores que el equipo tenia en pista (k_dir 2.83, margen 261) el
# esquive pedia 96 % de volante a 600 mm y 100 % a 400.
vo["obstaculos"].update(k_dir=2.8324, margen_mm=261.0, semi_pilar_mm=40.7,
                        activar_desde_mm=4000.0, mandar_desde_mm=506.6)
picos = []
for dist in (2000.0, 1500.0, 1000.0, 800.0, 600.0, 400.0, 250.0):
    d, _pe = esquivar({"rojo": [pilar(dist, 0.0)]})
    picos.append(abs(d))
prueba("con los valores reales del equipo ya no pide el volante a tope",
       max(picos) <= vo["obstaculos"]["dir_max_pct"] + 0.5,
       f"maximo pedido {max(picos):.0f}% (tope {vo['obstaculos']['dir_max_pct']})")
prueba("y de cerca no pide mas que de lejos por el angulo",
       picos[-1] <= max(picos) + 0.5, str([round(x) for x in picos]))

# la rampa impide el volantazo en un solo frame
esq.reiniciar()
d1, _ = esq.paso({"rojo": [pilar(500.0, -300.0)]}, None, geo_o, None, False, 1, 500.0)
prueba("el primer frame no da un volantazo", abs(d1) < 30, f"{d1:.0f}%")

# --- compromiso: no volverse hacia el pilar mientras se le adelanta -------
vo["obstaculos"].update(k_dir=1.4, margen_mm=70.0, semi_pilar_mm=25.0,
                        activar_desde_mm=1600.0, mandar_desde_mm=700.0)
esq.reiniciar()
# se ve el pilar ya encima (dentro de mandar_desde_mm)
esq.paso({"rojo": [pilar(300.0, -150.0)]}, None, geo_o, None, False, 1, 500.0)
prueba("con el pilar encima se arma el compromiso",
       esq.info.get("compromiso_s", 0) > 0, str(esq.info))
# ahora desaparece de la vista (esta demasiado cerca para la camara)
d_c, peso_c = esq.paso({}, None, geo_o, None, False, 1, 500.0)
prueba("aunque ya no se vea, el esquive sigue mandando", peso_c > 0,
       f"peso={peso_c}")
prueba("y lo que pide es ir RECTO, no volver hacia el pilar",
       abs(d_c) < 20, f"dir={d_c:.0f}")
prueba("la telemetria dice que esta adelantando",
       "adelantando_s" in esq.info, str(esq.info))

# sin pilar y sin compromiso, no manda nada
esq.reiniciar()
_d, peso0 = esq.paso({}, None, geo_o, None, False, 1, 500.0)
prueba("sin pilar ni compromiso, el esquive no manda", peso0 == 0.0)

# --- hueco mas estrecho que el carro: al centro, no a un lado al azar -----
class PerfilEstrecho:
    """Pasillo con las dos paredes a +-x_mm: si no cabe el carro, el punto de
    paso no puede elegirse 'a un lado', hay que ir por el centro."""
    hay_muro = True
    ancho = 640

    def __init__(self, geo, x_mm, dist_mm=500.0):
        fila = geo.distancia_a_fila(dist_mm)
        self.valido = np.zeros(640, bool)
        self.dist_mm = np.full(640, 9999.0, np.float32)
        self.y_contacto = np.zeros(640, np.int32)
        for signo in (-1, 1):
            u, v = geo.suelo_a_pixel(signo * x_mm, dist_mm)
            for c in range(max(0, u - 40), min(640, u + 40)):
                self.valido[c] = True
                self.dist_mm[c] = dist_mm
                self.y_contacto[c] = v

esq.reiniciar()
estrecho = PerfilEstrecho(geo_o, 130.0)      # 260 mm de hueco, el carro mide 200
d_e, _pe = esq.paso({"rojo": [pilar(600.0, 0.0)]}, estrecho, geo_o,
                    None, False, 1, 500.0)
prueba("con el hueco mas estrecho que el carro, lo detecta",
       "hueco_estrecho_mm" in esq.info, str(esq.info))
prueba("y apunta al centro del hueco, no a un lado",
       abs(esq.info.get("objetivo_mm", 999)) < 60,
       str(esq.info.get("objetivo_mm")))

# ===========================================================================
print("== el carro nunca circula en sentido contrario ==")
# Tras varios escapes el carro podia quedar mirando hacia atras y adoptar ese
# rumbo como bueno. El reglamento termina la ronda por circular al reves.
vg = params_mod.valores_por_defecto()
vg["navegacion"].update(min_recto_ms=0, retardo_giro_ms=0)
esquinas_contadas = []
nav_g = Navegador(vg["navegacion"], vg["limites"], vg["escape"], vg["giro2t"],
                  al_completar_giro=lambda lado: esquinas_contadas.append(lado),
                  cfg_obst=vg["obstaculos"])
nav_g.paso(p_libre, 0.0, 1)                  # ancla el rumbo de la recta en 0
prueba("el rumbo de la recta se ancla al arrancar", nav_g.rumbo_recta == 0.0,
       str(nav_g.rumbo_recta))
d_mal = nav_g.paso(p_libre, 175.0, 1)        # el carro quedo mirando atras
prueba("mirando hacia atras, dispara un rescate de rumbo",
       "RESCATE de rumbo" in d_mal.motivo, d_mal.motivo)
prueba("y gira hacia el lado que lo devuelve a la recta",
       nav_g.rumbo_objetivo == 0.0, str(nav_g.rumbo_objetivo))
# el rescate NO es una esquina: no puede sumar vuelta
for _ in range(30):
    nav_g.paso(p_libre, 2.0, 1)              # ya alineado: el giro termina
prueba("un rescate de rumbo NO cuenta como esquina",
       len(esquinas_contadas) == 0, str(esquinas_contadas))

# el escape ya no se queda con el rumbo en el que acabo la maniobra
nav_e = Navegador(vg["navegacion"], vg["limites"], vg["escape"], vg["giro2t"],
                  cfg_obst=vg["obstaculos"])
nav_e.paso(p_libre, 0.0, 1)
recta = nav_e.rumbo_recta
nav_e.paso(p_cerca, 40.0, 1)                 # muro encima -> ESCAPE
prueba("el escape no toca el rumbo de la recta", nav_e.rumbo_recta == recta,
       f"{nav_e.rumbo_recta} vs {recta}")

# una curva de verdad SI avanza el rumbo de la recta 90 grados
nav_c = Navegador(vg["navegacion"], vg["limites"], vg["escape"], vg["giro2t"],
                  cfg_obst=vg["obstaculos"])
nav_c.paso(p_libre, 0.0, 1, en_esquina=True)      # entra en PRE_GIRO
nav_c.paso(p_libre, 0.0, 1, en_esquina=True)      # PRE_GIRO -> GIRO
prueba("una curva avanza el rumbo de la recta 90 grados",
       abs(_norm(nav_c.rumbo_recta - 90.0)) < 1e-6, str(nav_c.rumbo_recta))

# --- el muro puede vetar al pilar ----------------------------------------
vv = params_mod.valores_por_defecto()
vv["navegacion"].update(min_recto_ms=0)
vv["obstaculos"]["peso_max"] = 1.0
nav_v = Navegador(vv["navegacion"], vv["limites"], vv["escape"], vv["giro2t"],
                  cfg_obst=vv["obstaculos"])
d_lejos = nav_v.paso(p_libre, None, 1, bias_obstaculo=(50.0, 1.0))
prueba("con el frente despejado el pilar manda",
       abs(d_lejos.direccion - 50) < 15, f"dir={d_lejos.direccion}")
p_medio = muro.perfil(escena(285), geo, mcfg)     # muro cerca, sin ser escape
nav_v2 = Navegador(vv["navegacion"], vv["limites"], vv["escape"], vv["giro2t"],
                   cfg_obst=vv["obstaculos"])
d_cerca = nav_v2.paso(p_medio, None, 1, bias_obstaculo=(50.0, 1.0))
prueba("con el muro encima el pilar CEDE el mando",
       abs(d_cerca.direccion) < abs(d_lejos.direccion),
       f"lejos={d_lejos.direccion} cerca={d_cerca.direccion} "
       f"pasillo={p_medio.pasillo_mm:.0f}")




# ===========================================================================
print("== el lado de paso no se puede invertir ==")
from src.obstaculos import Esquivador                       # noqa: E402
from src.vision import Deteccion                            # noqa: E402


class _PerfilPared:
    """Perfil con dos paredes rectas, para probar el recorte sin renderizar."""
    def __init__(self, x_izq, x_der, ancho=640):
        self.ancho = ancho
        self.hay_muro = True
        self.valido = np.ones(ancho, bool)
        self.dist_mm = np.full(ancho, 600.0, np.float32)
        self.y_contacto = np.full(ancho, 300, np.int32)
        self._xi, self._xd = x_izq, x_der

    def lateral_de(self, c):
        return self._xi if c < self.ancho / 2 else self._xd


class _GeoPared(Geometria):
    def __init__(self, base, perfil):
        super().__init__(base.cfg, base.W, base.H)
        self._p = perfil

    def lateral_mm(self, u, v):
        try:
            return self._p.lateral_de(int(u))
        except Exception:
            return super().lateral_mm(u, v)


vg = params_mod.valores_por_defecto()
vg["obstaculos"]["margen_mm"] = 261.0        # el valor que usa el equipo
geo_o = Geometria(vg["geometria"], 640, 480)
pf = _PerfilPared(-500.0, 420.0)
gf = _GeoPared(geo_o, pf)
esq = Esquivador(vg["obstaculos"])
semi = vg["geometria"]["ancho_carro_mm"] / 2.0
semi_p = vg["obstaculos"]["semi_pilar_mm"]
minimo = semi + semi_p + 15.0
pedido = semi + vg["obstaculos"]["margen_mm"] + semi_p

invertidos = 0
for lat in (0.0, 150.0, 250.0, 300.0, 350.0, 400.0):
    esq.info = {}
    obj = esq._recortar(lat + pedido, 600.0, pf, gf, semi, lat, +1, minimo)
    if obj <= lat:
        invertidos += 1
prueba("un pilar ROJO nunca acaba pasandose por la izquierda",
       invertidos == 0, f"{invertidos} casos invertidos")

invertidos = 0
for lat in (0.0, -150.0, -250.0, -300.0, -400.0):
    esq.info = {}
    obj = esq._recortar(lat - pedido, 600.0, pf, gf, semi, lat, -1, minimo)
    if obj >= lat:
        invertidos += 1
prueba("un pilar VERDE nunca acaba pasandose por la derecha",
       invertidos == 0, f"{invertidos} casos invertidos")

esq.info = {}
esq._recortar(350.0 + pedido, 600.0, pf, gf, semi, 350.0, +1, minimo)
prueba("y cuando no cabe, lo dice en vez de callarselo",
       "sin_sitio" in esq.info, str(esq.info))

# El lado sale del COLOR, no del sentido de la ronda.
def lado_elegido(color, sentido):
    v2 = params_mod.valores_por_defecto()
    v2["obstaculos"].update(activo=True, limitar_por_lineas=False)
    e = Esquivador(v2["obstaculos"])
    u, vv = geo_o.suelo_a_pixel(0.0, 800.0 + v2["geometria"]["morro_mm"])
    d = Deteccion(color=color, x=u - 18, y=vv - 70, w=36, h=70, area=2400,
                  llenado=0.9, aspecto=2.0, cx=float(u), cy=float(vv - 35))
    e.paso({color: [d]}, None, geo_o, None, False, sentido, 500.0)
    return e.info.get("lado")

prueba("rojo por la DERECHA en horario y en antihorario",
       lado_elegido("rojo", 1) == "derecha" and lado_elegido("rojo", -1) == "derecha",
       f"{lado_elegido('rojo', 1)} / {lado_elegido('rojo', -1)}")
prueba("verde por la IZQUIERDA en horario y en antihorario",
       lado_elegido("verde", 1) == "izquierda" and lado_elegido("verde", -1) == "izquierda",
       f"{lado_elegido('verde', 1)} / {lado_elegido('verde', -1)}")

# ===========================================================================
print("== la correccion de rumbo cede mientras se esquiva ==")
libre_o = {"blanco": np.full((480, 640), 255, np.uint8),
           "negro": np.zeros((480, 640), np.uint8)}
p_libre_o = muro.perfil(libre_o, geo_o, params_mod.valores_por_defecto()["muro"])


def dir_con_yaw(yaw, cede):
    vv = params_mod.valores_por_defecto()
    vv["navegacion"]["yaw_cede_al_esquivar"] = cede
    n = Navegador(vv["navegacion"], vv["limites"], vv["escape"], vv["giro2t"],
                  vv["obstaculos"])
    n.rumbo_objetivo = 0.0
    return n.paso(p_libre_o, yaw, 1, bias_obstaculo=(55.0, 1.0)).direccion


prueba("(control) sin ceder, el yaw SUMA y satura el volante",
       dir_con_yaw(-20.0, False) >= 85, str(dir_con_yaw(-20.0, False)))
prueba("(control) sin ceder, hacia el otro lado RESTA y se queda corto",
       dir_con_yaw(20.0, False) <= 30, str(dir_con_yaw(20.0, False)))
prueba("cediendo, el esquive se respeta venga como venga el rumbo",
       dir_con_yaw(-20.0, True) == 55 and dir_con_yaw(20.0, True) == 55
       and dir_con_yaw(0.0, True) == 55,
       f"{dir_con_yaw(-20.0, True)} / {dir_con_yaw(20.0, True)}")

# ===========================================================================
print("== modo borde y memoria del ultimo pilar ==")
vb = params_mod.valores_por_defecto()
vb["obstaculos"].update(activo=True, modo="borde", limitar_por_lineas=False)
eb = Esquivador(vb["obstaculos"])


def _pil(dist_mm, lat_mm, color="rojo"):
    u, vv = geo_o.suelo_a_pixel(lat_mm, dist_mm + vb["geometria"]["morro_mm"])
    return Deteccion(color=color, x=u - 18, y=vv - 70, w=36, h=70, area=2400,
                     llenado=0.9, aspecto=2.0, cx=float(u), cy=float(vv - 35))


def estable(dets, ticks=60):
    d = 0.0
    for _ in range(ticks):
        eb._t_prev = time.time() - 0.033      # 30 fps de verdad, no el bucle
        d, _p = eb.paso(dets, None, geo_o, None, False, 0, 500.0)
    return d


eb.reiniciar()
d_izq = estable({"rojo": [_pil(900.0, -200.0)]})
eb.reiniciar()
d_der = estable({"rojo": [_pil(900.0, 250.0)]})
prueba("modo borde: el rojo se empuja al canto izquierdo (gira a la derecha)",
       d_izq > 0 and d_der > d_izq, f"{d_izq:.0f} / {d_der:.0f}")
eb.reiniciar()
v_izq = estable({"verde": [_pil(900.0, -250.0)]})
prueba("y el verde al reves, simetrico", v_izq < 0, f"{v_izq:.0f}")

eb.reiniciar()
lejos = estable({"rojo": [_pil(900.0, 0.0)]})
eb.reiniciar()
cerca = estable({"rojo": [_pil(300.0, 0.0)]})
prueba("al tenerlo encima mete el giro grande",
       cerca > lejos and eb.info.get("giro_final") is True,
       f"lejos {lejos:.0f} / cerca {cerca:.0f}")

# memoria + busqueda
eb.reiniciar()
eb._t_prev = time.time() - 0.033
eb.paso({"rojo": [_pil(1200.0, 150.0)]}, None, geo_o, None, False, 0, 500.0)
prueba("recuerda el ultimo pilar", eb.memoria_viva(time.time())
       and eb.memoria()["lado"] == "derecha", str(eb.memoria()))
d_busca = 0.0
for _ in range(30):
    eb._t_prev = time.time() - 0.033
    d_busca, peso_b = eb.paso({}, None, geo_o, None, False, 0, 500.0)
prueba("si se pierde LEJOS, lo busca para volver a encuadrarlo",
       eb.info.get("buscando") is True and d_busca < 0,
       f"dir {d_busca:.0f} info {eb.info}")

vb["obstaculos"]["recordar_lado"] = False
prueba("la memoria se puede apagar", not eb.memoria_viva(time.time()))
vb["obstaculos"]["recordar_lado"] = True

# perdido de CERCA: manda el compromiso de adelantamiento, no la busqueda
eb.reiniciar()
for _ in range(20):
    eb._t_prev = time.time() - 0.033
    eb.paso({"rojo": [_pil(400.0, 100.0)]}, None, geo_o, None, False, 0, 500.0)
eb._t_prev = time.time() - 0.033
d_cerca, _pc = eb.paso({}, None, geo_o, None, False, 0, 500.0)
prueba("si se pierde CERCA no lo busca: adelanta comprometido",
       eb.info.get("buscando") is None and "adelantando_s" in eb.info,
       str(eb.info))

# ===========================================================================
print("== boton de competencia ==")
# La logica del pulsador no toca hardware ni enlace: se le dan niveles y
# relojes. Lo que se prueba aqui es justo lo que no se puede probar con un
# dedo en la pista: rebotes de contacto y el boton pisado al encender.
b = bot_mod.Pulsador(antirrebote_ms=40.0, largo_ms=1000.0)
t = 100.0
prueba("empieza suelto", b.paso(False, t) == "" and not b.pulsado)

# rebote de contacto: sube y baja varias veces en 10 ms y NO cuenta
ev = ""
for i in range(6):
    t += 0.005
    ev = ev or b.paso(i % 2 == 0, t)
prueba("un rebote de 5 ms no dispara nada", ev == "" and not b.pulsado)

# ahora se mantiene pisado: se admite pasado el antirrebote. Cada cambio de
# nivel reinicia la cuenta, asi que hacen falta dos lecturas separadas.
t += 0.05
b.paso(True, t)                          # sube el nivel: empieza la cuenta
t += 0.05
prueba("40 ms pisado = pulsado", b.paso(True, t) == "" and b.pulsado)
t += 0.10
b.paso(False, t)                         # baja: empieza la cuenta de la soltada
t += 0.05
prueba("suelto antes del tiempo largo = pulsacion CORTA",
       b.paso(False, t) == bot_mod.CORTA)

# pisado largo: avisa SIN soltar y al soltar ya no da la corta
t += 0.10
b.paso(True, t)
t += 0.06
b.paso(True, t)
t += 1.0
prueba("pisado 1 s = pulsacion LARGA", b.paso(True, t) == bot_mod.LARGA)
t += 0.5
prueba("la larga no se repite sola", b.paso(True, t) == "")
t += 0.10
prueba("al soltar tras una larga no sale una corta", b.paso(False, t) == "")

# el boton pisado al encender el carro no puede lanzar la ronda
b2 = bot_mod.Pulsador(antirrebote_ms=40.0, largo_ms=300.0)
t = 500.0
eventos = []
for _ in range(20):                      # 2 s con el boton pisado desde el 0
    t += 0.1
    eventos.append(b2.paso(True, t))
for _ in range(2):                       # y al fin se suelta (y se confirma)
    t += 0.1
    eventos.append(b2.paso(False, t))
prueba("boton pisado al arrancar: mudo hasta verlo suelto",
       not any(eventos), str([e for e in eventos if e]))
for pisado in (True, True, False, False):
    t += 0.1
    ultimo_ev = b2.paso(pisado, t)
prueba("y despues ya funciona", ultimo_ev == bot_mod.CORTA, ultimo_ev)

# El gestor completo contra un ESP32 de mentira. Es el camino entero: nivel
# en la trama de sensores -> antirrebote -> corta/larga -> accion.
class EnlaceFalso:
    """Lo unico que el gestor le pide al enlace: el nivel del pulsador."""

    def __init__(self):
        self.pisado = False
        self.corte = False
        self.frescos = True

    def boton(self):
        return (self.pisado, self.corte, self.frescos)


enl = EnlaceFalso()
cfg_bot = params_mod.valores_por_defecto()["botones"]
cfg_bot.update({"activo": True, "largo_ms": 250, "antirrebote_ms": 10,
                "repeticion_ms": 100, "hz": 200})
hechas = []
g = bot_mod.GestorBotones(
    cfg_bot, enl,
    acciones={"corta": lambda: hechas.append("armar/desarmar"),
              "larga": lambda: hechas.append("apagar")})
g.iniciar()
time.sleep(0.1)                          # verlo suelto una vez

enl.pisado = True
time.sleep(0.12)
enl.pisado = False
time.sleep(0.12)
prueba("el nivel que manda el ESP32 dispara la accion",
       hechas == ["armar/desarmar"], str(hechas))

del hechas[:]
enl.pisado = True
time.sleep(0.35)                         # mas que largo_ms
prueba("mantenerlo pisado = pulsacion larga", hechas == ["apagar"], str(hechas))
enl.pisado = False
time.sleep(0.15)
prueba("al soltar tras la larga no sale ademas la corta", hechas == ["apagar"],
       str(hechas))

# El caso peligroso: el enlace se cae con el dedo encima y vuelve con el dedo
# todavia encima. Si eso contara como pulsacion, el carro saldria corriendo
# solo al reconectar el serial.
del hechas[:]
enl.pisado = True
enl.frescos = False                      # se cae con el boton pisado
time.sleep(0.15)
prueba("con el enlace caido no pasa nada", hechas == [], str(hechas))
enl.frescos = True                       # vuelve, y sigue pisado
time.sleep(0.4)
prueba("y al volver con el boton pisado tampoco arranca", hechas == [],
       str(hechas))
enl.pisado = False
time.sleep(0.15)
prueba("soltarlo solo lo desbloquea, no cuenta como pulsacion", hechas == [],
       str(hechas))
enl.pisado = True
time.sleep(0.12)
enl.pisado = False
time.sleep(0.12)
prueba("despues ya vuelve a funcionar", hechas == ["armar/desarmar"],
       str(hechas))

# El corte local del ESP32 se ve desde la web
enl.corte = True
time.sleep(0.1)
prueba("el estado dice que el ESP32 corto por el boton",
       g.estado()["corte_esp32"] is True, str(g.estado()))
enl.corte = False

# Pulsaciones virtuales: el mismo camino, sin ESP32 (asi se ensaya en el PC)
del hechas[:]
g.pulsar_virtual()
time.sleep(0.3)
prueba("pulsacion virtual = la misma accion", hechas == ["armar/desarmar"],
       str(hechas))
prueba("el estado dice cual fue la ultima",
       g.estado()["ultimo"] == bot_mod.CORTA, str(g.estado()))
try:
    g.pulsar_virtual("mediana")
    prueba("una pulsacion inventada lanza", False)
except ValueError:
    prueba("una pulsacion inventada lanza", True)

# Apagado, el pulsador del ESP32 se ignora; el virtual de la web, no (ese lo
# manda quien ya podia armar el carro desde la interfaz).
del hechas[:]
cfg_bot["activo"] = False
time.sleep(0.1)
enl.pisado = True
time.sleep(0.15)
enl.pisado = False
time.sleep(0.15)
prueba("con botones.activo apagado el ESP32 se ignora", hechas == [],
       str(hechas))
g.pulsar_virtual()
time.sleep(0.3)
prueba("pero la pulsacion de la web sigue valiendo",
       hechas == ["armar/desarmar"], str(hechas))
g.cerrar()

# ===========================================================================
print("== parametros ==")
vals = params_mod.valores_por_defecto()
prueba("el boton tiene su grupo de parametros",
       set(("activo", "largo_ms", "repeticion_ms")) <= set(vals["botones"]),
       str(sorted(vals["botones"])))
prueba("el pulsador viene encendido (es el start del reglamento)",
       vals["botones"]["activo"] is True)
# El pin NO se configura desde aqui: es del firmware del ESP32, como los del
# motor. Si algun dia reaparece en este grupo es que alguien volvio a
# cablearlo a la Pi sin actualizar el resto.
prueba("el pin no vive en los parametros de la Pi",
       not [k for k in vals["botones"] if k.startswith("pin")],
       str(sorted(vals["botones"])))
prueba("el apagado por pulsacion larga viene desactivado",
       vals["botones"]["apagar_con_larga"] is False)
prueba("validar recorta", params_mod.validar("limites", "vmax", 999) == 255)
prueba("validar bool", params_mod.validar("navegacion", "usar_yaw", "0") is False)
try:
    params_mod.validar("navegacion", "estrategia", "volar")
    prueba("opcion invalida lanza", False)
except ValueError:
    prueba("opcion invalida lanza", True)

# Valores "automaticos": el centinela vive fuera del rango util a proposito
# (-1 no es una temperatura de color ni un indice de camara). Si validar lo
# recortara contra el minimo, el boton AUTO de la web dejaria de funcionar.
for grupo, clave in (("camara", "exposicion"), ("camara", "balance_blancos"),
                     ("camara", "indice_trasera")):
    e = params_mod.ESQUEMA[grupo][clave]
    prueba(f"{clave} declara valor automatico", "auto" in e)
    prueba(f"{clave}: el centinela pasa sin recortar",
           params_mod.validar(grupo, clave, e["auto"]) == e["auto"],
           str(params_mod.validar(grupo, clave, e["auto"])))
    prueba(f"{clave}: al apagar AUTO cae en un valor valido",
           params_mod.validar(grupo, clave, e["auto_off"]) == e["auto_off"],
           str(e["auto_off"]))
prueba("fuera de rango si se recorta",
       params_mod.validar("camara", "balance_blancos", 10) == 2000.0)
prueba("los valores por defecto sobreviven a normalizar",
       params_mod.normalizar(params_mod.valores_por_defecto())["camara"]["exposicion"] == -1.0)

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
    for i in range(25):
        params_mod.guardar_perfil(datos2, f"p{i}", vals, "obstaculos")
    prueba("cada linea guarda hasta 20",
           len(params_mod.listar(datos2, "obstaculos")) == 20,
           str(len(params_mod.listar(datos2, "obstaculos"))))
    prueba("y no toca los de las otras lineas",
           len(params_mod.listar(datos2, "open")) >= 1,
           str(params_mod.listar(datos2, "open")))
    params_mod.guardar_perfil(datos2, "solo_estacionar", vals, "estacionar")
    prueba("las tres lineas conviven",
           set(params_mod.CATEGORIAS) == {"open", "obstaculos", "estacionar"}
           and params_mod.listar(datos2, "estacionar") == ["solo_estacionar"],
           str(params_mod.listar(datos2, "estacionar")))
    prueba("una categoria inventada cae en open",
           params_mod.categoria_valida("loquesea") == "open")

# ===========================================================================
print()
if FALLOS:
    print(f"{len(FALLOS)}/{TOTAL} PRUEBAS FALLARON: {FALLOS}")
    sys.exit(1)
print(f"las {TOTAL} pruebas pasaron")
