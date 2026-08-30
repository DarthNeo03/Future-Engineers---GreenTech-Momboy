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
from src.lineas import GestorLineas, HORARIO, ANTIHORARIO  # noqa: E402
from src.navegacion import Navegador, RECTO, ESCAPE        # noqa: E402

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
prueba("cfg_tcs: 15 bytes", len(cfg) == 15, f"{len(cfg)}")

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
print("== lineas / sentido / vueltas ==")
lcfg = {"naranja_es_horario": True, "usar_tcs": True, "usar_camara": True,
        "umbral_cruce_mm": 260.0, "ventana_par_ms": 300,
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
print("== parametros ==")
vals = params_mod.valores_por_defecto()
prueba("validar recorta", params_mod.validar("limites", "vmax", 999) == 255)
prueba("validar bool", params_mod.validar("navegacion", "usar_yaw", "0") is False)
try:
    params_mod.validar("navegacion", "estrategia", "volar")
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
    for i in range(7):
        params_mod.guardar_perfil(datos2, f"p{i}", vals)
    prueba("solo quedan 5 perfiles", len(datos2["perfiles"]) == 5)

# ===========================================================================
print()
if FALLOS:
    print(f"{len(FALLOS)}/{TOTAL} PRUEBAS FALLARON: {FALLOS}")
    sys.exit(1)
print(f"las {TOTAL} pruebas pasaron")
