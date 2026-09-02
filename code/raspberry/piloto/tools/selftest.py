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
from src.geometria import Geometria     # noqa: E402
from src import muro                    # noqa: E402
from src.lineas import (GestorLineas, HORARIO, ANTIHORARIO,   # noqa: E402
                        ZONA_RECTA, ZONA_ESQUINA)
from src.navegacion import (Navegador, RECTO, PRE_GIRO,      # noqa: E402
                            GIRO, GIRO_2T, ESCAPE)

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
prueba("sensores: 19 bytes", len(tr) == 19, f"{len(tr)}")
s2 = P.Sensores.desde_payload(tr[4:18])
prueba("sensores: ida y vuelta",
       (s2.yaw_deci, s2.gz_deci, s2.c, s2.cnt_naranja, s2.cnt_azul) ==
       (-1234, 567, 1000, 3, 5))
prueba("sensores: clase linea", s2.clase_linea == P.LINEA_AZUL)
prueba("sensores: yaw en grados", abs(s2.yaw + 123.4) < 1e-6)

cfg = P.empaquetar_cfg_tcs(80, 120, 60, 110, 70, 1, 3, 246, 2)
prueba("cfg_tcs: 17 bytes (12 de payload)", len(cfg) == 17, f"{len(cfg)}")

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
gp = GestorLineas(lcfg)
par(gp, "naranja", "azul")
time.sleep(0.45)
gp.evento_tcs("naranja")          # se pierde el azul de esta esquina
time.sleep(0.45)                  # caduca la ventana del par
gp.paso_zona()
prueba("el par a medias caduca", gp.pares_incompletos == 1,
       str(gp.pares_incompletos))
par(gp, "naranja", "azul")        # esquina siguiente, entera
prueba("la esquina siguiente cuenta bien y sin incoherencia",
       gp.esquinas == 2 and gp.incoherencias == 0,
       f"esq={gp.esquinas} incoh={gp.incoherencias}")

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

# sin lineas a la vista, todo cuenta (comportamiento de siempre)
_d, peso = esq.paso(lejos, None, geo_o, None, False)
prueba("sin lineas a la vista, el pilar lejano cuenta", peso > 0, str(peso))

# con la linea a 1100 mm, el pilar de 1400 queda detras: se descarta
_d, peso = esq.paso(lejos, None, geo_o, {"naranja": 1100.0}, False)
prueba("con la linea delante, el pilar de detras NO cuenta", peso == 0.0,
       f"peso={peso} info={esq.info}")
prueba("y queda anotado en la telemetria", esq.info.get("tras_linea") == 1,
       str(esq.info))

# el pilar de esta recta sigue contando igual
_d, peso = esq.paso(cerca, None, geo_o, {"naranja": 1100.0}, False)
prueba("el pilar de esta recta sigue contando", peso > 0, str(peso))

# justo antes de la linea, dentro del margen: cuenta
_d, peso = esq.paso({"rojo": [pilar(1130.0, 120.0)]}, None, geo_o,
                    {"naranja": 1100.0}, False)
prueba("un pilar justo antes de la linea (margen) cuenta", peso > 0, str(peso))

# YA EN LA ESQUINA: el limite se levanta y el pilar del tramo nuevo cuenta
_d, peso = esq.paso(lejos, None, geo_o, {"naranja": 1100.0}, True)
prueba("una vez en la esquina, el pilar de la seccion nueva SI cuenta",
       peso > 0, f"peso={peso} info={esq.info}")

# se puede apagar
vo["obstaculos"]["limitar_por_lineas"] = False
_d, peso = esq.paso(lejos, None, geo_o, {"naranja": 1100.0}, False)
prueba("el filtro se puede apagar", peso > 0, str(peso))
vo["obstaculos"]["limitar_por_lineas"] = True

# ===========================================================================
print("== parametros ==")
vals = params_mod.valores_por_defecto()
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
    for i in range(7):
        params_mod.guardar_perfil(datos2, f"p{i}", vals)
    prueba("solo quedan 5 perfiles", len(datos2["perfiles"]) == 5)

# ===========================================================================
print()
if FALLOS:
    print(f"{len(FALLOS)}/{TOTAL} PRUEBAS FALLARON: {FALLOS}")
    sys.exit(1)
print(f"las {TOTAL} pruebas pasaron")
