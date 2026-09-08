"""
color_config.py — Gestion de perfiles de calibracion de color (WRO Future Engineers).

Guarda hasta MAX_PERFILES calibraciones en un unico archivo JSON
(config/colors.json). El perfil mas reciente queda de primero; al guardar el
sexto se descarta el mas viejo. El programa principal solo necesita:

    from src import color_config as cc
    perfil = cc.perfil_activo()          # dict con los colores ya validados
    rojo   = perfil["colores"]["rojo"]

Compatible Windows 11 / Debian-Raspbian: solo usa pathlib + json y escribe de
forma atomica (archivo temporal + os.replace) para no corromper el JSON si se
corta la corriente a mitad de guardado.
"""

from __future__ import annotations

import copy
import datetime as _dt
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

# --------------------------------------------------------------------------
# Rutas
# --------------------------------------------------------------------------
RAIZ_PROYECTO = Path(__file__).resolve().parent.parent
RUTA_CONFIG_POR_DEFECTO = RAIZ_PROYECTO / "config" / "colors.json"

MAX_PERFILES = 5
VERSION_ESQUEMA = 2

# --------------------------------------------------------------------------
# Parametros por defecto de cada color
# --------------------------------------------------------------------------
# "rangos" es una lista de pares [[Hmin,Smin,Vmin],[Hmax,Smax,Vmax]].
# El rojo necesita DOS porque el tono (Hue) se envuelve en 0/179.
#
# Parametros de deteccion:
#   desenfoque   : tamano del medianBlur previo (0 = desactivado, impar)
#   abrir        : MORPH_OPEN, borra motas sueltas (0 = off, impar)
#   cerrar       : MORPH_CLOSE, tapa huecos internos por brillo (0 = off, impar)
#   unir_huecos  : radio en px para considerar "un mismo objeto" dos manchas
#                  separadas. Sustituye al viejo criterio de "si se tocan".
#   area_min/max : area real en pixeles del objeto (sin contar la dilatacion)
#   llenado_min  : area / (ancho*alto) del bounding box. Un pilar solido da
#                  0.7-0.95; el ruido disperso da valores bajos.
#   aspecto_*    : alto/ancho. Un pilar 5x5x10 cm de pie es > 1.
#   usar_aspecto : permite desactivar ese filtro (util para las paredes).
#   ancho_min / alto_min : descarta astillas de 1-2 px.
#   roi_arriba / roi_abajo : fraccion vertical de la imagen donde SI se busca.
#                  Subir roi_arriba evita detectar camisetas rojas del publico.
#   max_objetos  : cuantas detecciones devolver (las mas grandes primero).
#   color_dibujo : BGR con el que se dibuja en pantalla.

PARAMS_BASE: Dict[str, Any] = {
    "rangos": [[[0, 0, 0], [179, 255, 255]]],
    "desenfoque": 3,
    "abrir": 3,
    "cerrar": 5,
    "unir_huecos": 5,
    "area_min": 400,
    "area_max": 250000,
    "llenado_min": 0.45,
    "aspecto_min": 0.5,
    "aspecto_max": 5.0,
    "usar_aspecto": True,
    "ancho_min": 6,
    "alto_min": 6,
    "roi_arriba": 0.0,
    "roi_abajo": 1.0,
    "max_objetos": 4,
    "color_dibujo": [255, 255, 255],
}


def _color(**cambios: Any) -> Dict[str, Any]:
    d = copy.deepcopy(PARAMS_BASE)
    d.update(cambios)
    return d


def colores_por_defecto() -> Dict[str, Dict[str, Any]]:
    """Punto de partida razonable; se afina con el calibrador."""
    return {
        "rojo": _color(
            rangos=[
                [[0, 110, 70], [8, 255, 255]],
                [[168, 110, 70], [179, 255, 255]],
            ],
            area_min=350,
            llenado_min=0.55,
            aspecto_min=0.6,
            aspecto_max=4.0,
            usar_aspecto=True,
            roi_arriba=0.15,
            color_dibujo=[0, 0, 255],
        ),
        "verde": _color(
            rangos=[[[45, 70, 50], [85, 255, 255]]],
            area_min=350,
            llenado_min=0.55,
            aspecto_min=0.6,
            aspecto_max=4.0,
            usar_aspecto=True,
            roi_arriba=0.15,
            color_dibujo=[0, 200, 0],
        ),
        # Paredes: negro = saturacion y valor bajos, el tono no importa.
        # Filtros de forma relajados porque una pared no es un rectangulo.
        "negro": _color(
            rangos=[[[0, 0, 0], [179, 90, 65]]],
            desenfoque=3,
            abrir=3,
            cerrar=7,
            unir_huecos=7,
            area_min=1200,
            area_max=300000,
            llenado_min=0.20,
            usar_aspecto=False,
            aspecto_min=0.0,
            aspecto_max=50.0,
            ancho_min=4,
            alto_min=4,
            roi_arriba=0.0,
            roi_abajo=1.0,
            max_objetos=3,
            color_dibujo=[255, 0, 255],
        ),
        # ---- Todavia sin usar, listos para cuando toquen ------------------
        # Magenta: muros del estacionamiento del reto de obstaculos.
        "magenta": _color(
            rangos=[[[140, 80, 60], [168, 255, 255]]],
            area_min=400,
            llenado_min=0.35,
            usar_aspecto=False,
            roi_arriba=0.10,
            max_objetos=3,
            color_dibujo=[200, 0, 200],
        ),
        # Naranja: una de las dos lineas del piso (cuenta de vueltas y esquinas).
        # Va pegada al suelo, por eso la ROI mira solo la mitad de abajo y el
        # filtro de aspecto esta apagado (es una franja ancha y fina).
        "naranja": _color(
            rangos=[[[8, 120, 90], [24, 255, 255]]],
            desenfoque=3,
            abrir=3,
            cerrar=5,
            unir_huecos=8,
            area_min=500,
            llenado_min=0.25,
            usar_aspecto=False,
            roi_arriba=0.45,
            roi_abajo=1.0,
            max_objetos=2,
            color_dibujo=[0, 140, 255],
        ),
        # Azul: la otra linea del piso.
        "azul": _color(
            rangos=[[[95, 90, 60], [125, 255, 255]]],
            desenfoque=3,
            abrir=3,
            cerrar=5,
            unir_huecos=8,
            area_min=500,
            llenado_min=0.25,
            usar_aspecto=False,
            roi_arriba=0.45,
            roi_abajo=1.0,
            max_objetos=2,
            color_dibujo=[255, 120, 0],
        ),
    }


# --------------------------------------------------------------------------
# Validacion / normalizacion
# --------------------------------------------------------------------------
def _lim(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def normalizar_rangos(rangos: Any) -> List[List[List[int]]]:
    """Deja los rangos como enteros validos y ordenados (min <= max)."""
    salida: List[List[List[int]]] = []
    if not isinstance(rangos, (list, tuple)):
        rangos = []
    for par in rangos:
        try:
            bajo, alto = par[0], par[1]
            b = [int(_lim(int(bajo[i]), 0, 179 if i == 0 else 255)) for i in range(3)]
            a = [int(_lim(int(alto[i]), 0, 179 if i == 0 else 255)) for i in range(3)]
        except Exception:
            continue
        for i in range(3):
            if b[i] > a[i]:
                b[i], a[i] = a[i], b[i]
        salida.append([b, a])
    if not salida:
        salida = [[[0, 0, 0], [179, 255, 255]]]
    return salida[:4]  # 4 rangos por color es mas que suficiente


def normalizar_color(params: Any) -> Dict[str, Any]:
    """Completa claves faltantes con PARAMS_BASE y corrige tipos/limites."""
    d = copy.deepcopy(PARAMS_BASE)
    if isinstance(params, dict):
        d.update(params)

    d["rangos"] = normalizar_rangos(d.get("rangos"))

    for clave in ("desenfoque", "abrir", "cerrar"):
        v = int(_lim(int(d.get(clave, 0) or 0), 0, 31))
        if v > 0 and v % 2 == 0:  # los kernels deben ser impares
            v += 1
        d[clave] = v

    d["unir_huecos"] = int(_lim(int(d.get("unir_huecos", 0) or 0), 0, 40))
    d["area_min"] = int(_lim(int(d.get("area_min", 0) or 0), 0, 10_000_000))
    d["area_max"] = int(_lim(int(d.get("area_max", 0) or 0), 1, 10_000_000))
    if d["area_max"] <= d["area_min"]:
        d["area_max"] = d["area_min"] + 1
    d["llenado_min"] = float(_lim(float(d.get("llenado_min", 0.0)), 0.0, 1.0))
    d["aspecto_min"] = float(_lim(float(d.get("aspecto_min", 0.0)), 0.0, 50.0))
    d["aspecto_max"] = float(_lim(float(d.get("aspecto_max", 50.0)), 0.0, 50.0))
    if d["aspecto_max"] < d["aspecto_min"]:
        d["aspecto_min"], d["aspecto_max"] = d["aspecto_max"], d["aspecto_min"]
    d["usar_aspecto"] = bool(d.get("usar_aspecto", True))
    d["ancho_min"] = int(_lim(int(d.get("ancho_min", 0) or 0), 0, 4000))
    d["alto_min"] = int(_lim(int(d.get("alto_min", 0) or 0), 0, 4000))
    d["roi_arriba"] = float(_lim(float(d.get("roi_arriba", 0.0)), 0.0, 0.99))
    d["roi_abajo"] = float(_lim(float(d.get("roi_abajo", 1.0)), 0.01, 1.0))
    if d["roi_abajo"] <= d["roi_arriba"]:
        d["roi_abajo"] = min(1.0, d["roi_arriba"] + 0.01)
    d["max_objetos"] = int(_lim(int(d.get("max_objetos", 1) or 1), 1, 50))

    col = d.get("color_dibujo") or [255, 255, 255]
    try:
        d["color_dibujo"] = [int(_lim(int(c), 0, 255)) for c in col][:3]
        while len(d["color_dibujo"]) < 3:
            d["color_dibujo"].append(255)
    except Exception:
        d["color_dibujo"] = [255, 255, 255]

    return d


def normalizar_perfil(perfil: Any, nombre_alt: str = "sin_nombre") -> Dict[str, Any]:
    if not isinstance(perfil, dict):
        perfil = {}
    colores_in = perfil.get("colores")
    if not isinstance(colores_in, dict) or not colores_in:
        colores_in = colores_por_defecto()
    # Migracion: un perfil guardado antes de que existiera un color nuevo hereda
    # los valores por defecto de ese color, sin perder lo que ya tenia calibrado.
    colores = colores_por_defecto()
    colores.update({str(k): v for k, v in colores_in.items()})
    colores = {k: normalizar_color(v) for k, v in colores.items()}

    cam = perfil.get("camara") if isinstance(perfil.get("camara"), dict) else {}
    camara = {
        "indice": cam.get("indice", 0),
        "ancho": int(cam.get("ancho", 640) or 640),
        "alto": int(cam.get("alto", 480) or 480),
        "fps": int(cam.get("fps", 30) or 30),
        "fourcc": str(cam.get("fourcc", "MJPG") or "MJPG"),
        "voltear": bool(cam.get("voltear", False)),
    }
    return {
        "nombre": str(perfil.get("nombre") or nombre_alt),
        "fecha": str(perfil.get("fecha") or _ahora()),
        "notas": str(perfil.get("notas") or ""),
        "camara": camara,
        "colores": colores,
    }


def _ahora() -> str:
    return _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def perfil_nuevo(nombre: str = "base", notas: str = "") -> Dict[str, Any]:
    return normalizar_perfil({"nombre": nombre, "notas": notas,
                              "colores": colores_por_defecto()})


def datos_por_defecto() -> Dict[str, Any]:
    p = perfil_nuevo("base", "Valores iniciales sin calibrar")
    return {"version": VERSION_ESQUEMA, "activo": p["nombre"], "perfiles": [p]}


# --------------------------------------------------------------------------
# Lectura / escritura
# --------------------------------------------------------------------------
def cargar(ruta: Optional[Union[str, Path]] = None,
           crear_si_falta: bool = True) -> Dict[str, Any]:
    """Devuelve el archivo completo, ya validado. Nunca lanza por JSON dañado:
    si el archivo esta corrupto lo respalda como .bak y arranca de cero."""
    ruta = Path(ruta) if ruta else RUTA_CONFIG_POR_DEFECTO
    if not ruta.exists():
        datos = datos_por_defecto()
        if crear_si_falta:
            guardar(datos, ruta)
        return datos
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            datos = json.load(f)
        if not isinstance(datos, dict):
            raise ValueError("raiz del JSON no es un objeto")
    except Exception as e:
        respaldo = ruta.with_suffix(ruta.suffix + ".bak")
        try:
            os.replace(ruta, respaldo)
            print(f"[color_config] JSON invalido ({e}). Respaldado en {respaldo}")
        except Exception:
            pass
        datos = datos_por_defecto()
        if crear_si_falta:
            guardar(datos, ruta)
        return datos

    perfiles = datos.get("perfiles")
    if not isinstance(perfiles, list) or not perfiles:
        perfiles = [perfil_nuevo("base")]
    perfiles = [normalizar_perfil(p, f"perfil_{i}") for i, p in enumerate(perfiles)]
    perfiles = perfiles[:MAX_PERFILES]

    activo = datos.get("activo")
    nombres = [p["nombre"] for p in perfiles]
    if activo not in nombres:
        activo = nombres[0]

    return {"version": VERSION_ESQUEMA, "activo": activo, "perfiles": perfiles}


def guardar(datos: Dict[str, Any],
            ruta: Optional[Union[str, Path]] = None) -> Path:
    """Escritura atomica: escribe un .tmp en la misma carpeta y lo renombra."""
    ruta = Path(ruta) if ruta else RUTA_CONFIG_POR_DEFECTO
    ruta.parent.mkdir(parents=True, exist_ok=True)
    datos = {
        "version": VERSION_ESQUEMA,
        "activo": datos.get("activo"),
        "perfiles": [normalizar_perfil(p) for p in datos.get("perfiles", [])][:MAX_PERFILES],
    }
    nombres = [p["nombre"] for p in datos["perfiles"]]
    if datos["activo"] not in nombres:
        datos["activo"] = nombres[0] if nombres else None

    fd, tmp = tempfile.mkstemp(dir=str(ruta.parent), prefix=".colors_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(datos, f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, ruta)  # atomico en Windows y en Linux
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return ruta


# --------------------------------------------------------------------------
# Operaciones sobre perfiles
# --------------------------------------------------------------------------
def listar(datos: Dict[str, Any]) -> List[str]:
    return [p["nombre"] for p in datos.get("perfiles", [])]


def obtener(datos: Dict[str, Any],
            clave: Union[str, int, None] = None) -> Dict[str, Any]:
    """Acepta nombre, indice (0..4) o None (=> el activo)."""
    perfiles = datos.get("perfiles", [])
    if not perfiles:
        return perfil_nuevo("base")
    if clave is None:
        clave = datos.get("activo")
    if isinstance(clave, int):
        return perfiles[clave % len(perfiles)]
    for p in perfiles:
        if p["nombre"] == clave:
            return p
    return perfiles[0]


def fijar_activo(datos: Dict[str, Any], nombre: str) -> Dict[str, Any]:
    if nombre in listar(datos):
        datos["activo"] = nombre
    return datos


def guardar_perfil(datos: Dict[str, Any],
                   nombre: str,
                   colores: Dict[str, Any],
                   camara: Optional[Dict[str, Any]] = None,
                   notas: str = "",
                   hacer_activo: bool = True) -> Dict[str, Any]:
    """Inserta el perfil de primero. Si el nombre ya existe lo reemplaza en el
    sitio (no gasta cupo). Si no, empuja y descarta el mas viejo pasado de 5."""
    nombre = (nombre or "").strip() or _dt.datetime.now().strftime("calib_%m%d_%H%M")
    nuevo = normalizar_perfil({
        "nombre": nombre,
        "fecha": _ahora(),
        "notas": notas,
        "camara": camara or {},
        "colores": colores,
    })
    perfiles = [p for p in datos.get("perfiles", []) if p["nombre"] != nombre]
    perfiles.insert(0, nuevo)
    datos["perfiles"] = perfiles[:MAX_PERFILES]
    if hacer_activo:
        datos["activo"] = nombre
    elif datos.get("activo") not in listar(datos):
        datos["activo"] = datos["perfiles"][0]["nombre"]
    return datos


def borrar_perfil(datos: Dict[str, Any], nombre: str) -> Dict[str, Any]:
    perfiles = [p for p in datos.get("perfiles", []) if p["nombre"] != nombre]
    if not perfiles:
        perfiles = [perfil_nuevo("base")]
    datos["perfiles"] = perfiles
    if datos.get("activo") not in listar(datos):
        datos["activo"] = perfiles[0]["nombre"]
    return datos


# --------------------------------------------------------------------------
# Atajo para el programa principal
# --------------------------------------------------------------------------
def perfil_activo(ruta: Optional[Union[str, Path]] = None) -> Dict[str, Any]:
    """Uso tipico desde main.py:  perfil = color_config.perfil_activo()"""
    return obtener(cargar(ruta))


if __name__ == "__main__":
    d = cargar()
    print(f"Archivo: {RUTA_CONFIG_POR_DEFECTO}")
    print(f"Activo : {d['activo']}")
    for i, p in enumerate(d["perfiles"]):
        print(f"  [{i}] {p['nombre']:<20} {p['fecha']}  colores={list(p['colores'])}")
