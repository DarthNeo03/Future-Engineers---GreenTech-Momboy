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
        d = nav.paso(p_libre, yaw, 1, en_esquina=gl.en_esquina)
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
    n2t_cerca.paso(p_libre, 0.0, 1, en_esquina=True)      # entrar en 2T
est_antes = n2t_cerca.estado
d = n2t_cerca.paso(p_cerca, 1.0, 1, en_esquina=True)      # muro encima
prueba("con muro delante el 2T corta a reversa, no lo secuestra el escape",
       est_antes == GIRO_2T and n2t_cerca.estado == GIRO_2T and d.estado != ESCAPE,
       f"{est_antes} -> {n2t_cerca.estado} ({d.motivo})")

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
