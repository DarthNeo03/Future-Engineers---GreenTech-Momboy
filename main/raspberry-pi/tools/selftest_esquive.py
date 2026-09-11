#!/usr/bin/env python3
"""
selftest_esquive.py — El carro PASA el pilar, en simulacion.

    python3 tools/selftest_esquive.py            # todas las pruebas
    python3 tools/selftest_esquive.py --traza    # y la trayectoria de cada caso

selftest_obstaculos.py comprueba que el LADO sale bien. Esto comprueba lo
otro: que con ese lado el carro de verdad se aparta y no se lleva el pilar
por delante, que es lo que pasaba en pista. Para eso hace falta un carro que
se mueva, asi que aqui hay:

  * un modelo cinematico de BICICLETA con direccion Ackermann: el radio de
    giro a tope es geometria.radio_giro_mm (o uno distinto, para probar que
    pasa si se midio mal), el servo tarda en llegar y el motor tambien;
  * una CAMARA que proyecta cada pilar con la MISMA geometria que usa el carro
    y lo pierde igual que la de verdad: por el canto de la imagen cuando queda
    muy de lado y por abajo cuando esta encima;
  * un PERFIL DE MURO sintetico con las dos paredes del carril (y una de
    frente, lejos), calculado por trazado de rayos sobre la geometria;
  * el ESQUIVADOR y el NAVEGADOR cableados igual que en robot.py, con los
    parametros de config/obstaculos/ (los que corre el carro) y tambien con
    los valores por defecto del esquema.

Cada caso mide, tick a tick, donde queda cada pilar respecto al carro:
  * COLISION si el pilar entra en el rectangulo del carro;
  * LADO: al pasar el morro, el pilar tiene que quedar del lado que manda el
    color (rojo a la izquierda del carro, verde a la derecha);
  * HOLGURA: lo mas cerca que el costado del carro paso del pilar;
  * PARED: si alguna esquina del carro toca una pared del carril.
"""

from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
if str(RAIZ) not in sys.path:
    sys.path.insert(0, str(RAIZ))

from src import muro                        # noqa: E402
from src import navegacion as nav_mod       # noqa: E402
from src import obstaculos as obst_mod      # noqa: E402
from src import params as params_mod        # noqa: E402
from src import vision                      # noqa: E402
from src.geometria import Geometria, DIST_MAX_MM   # noqa: E402
from src.muro import PerfilMuro             # noqa: E402
from src.navegacion import Navegador, ESCAPE       # noqa: E402
from src.obstaculos import Esquivador       # noqa: E402

W, H = 640, 480
DT = 1.0 / 30.0

# --- el carro (mm). Las medidas de largo salen de geometria.largo_carro_mm y
# morro_mm; estas dos son del modelo: donde esta el eje trasero y cuanto mide
# entre ejes. No las usa el codigo del carro, solo el simulador.
EJE_A_COLA = 40.0          # del eje trasero a la cola
ENTRE_EJES = 165.0         # distancia entre ejes
SERVO_PCT_S = 900.0        # cuanto % de volante por segundo alcanza el servo
MOTOR_TAU_S = 0.12         # constante de tiempo del motor


class Reloj:
    """Reloj falso: el esquivador y el navegador usan time.time() para sus
    plazos, y una simulacion tiene que ser determinista y rapida."""
    def __init__(self, t0: float = 1000.0):
        self.t = t0

    def time(self) -> float:
        return self.t

    def avanzar(self, dt: float) -> None:
        self.t += dt


@dataclass
class Carro:
    x: float = 0.0          # eje trasero, mm, a lo largo del carril
    y: float = 0.0          # eje trasero, mm, + hacia la derecha
    th: float = 0.0         # rumbo, rad, + = girado a la derecha
    v: float = 0.0          # mm/s (con signo)
    dir_real: float = 0.0   # % de volante que el servo tiene puesto de verdad

    def relativo(self, px: float, py: float, adelanto: float = 0.0):
        """(adelante, derecha) de un punto del mundo respecto a un punto del
        carro situado 'adelanto' mm por delante del eje trasero."""
        c, s = math.cos(self.th), math.sin(self.th)
        ox, oy = self.x + adelanto * c, self.y + adelanto * s
        dx, dy = px - ox, py - oy
        return dx * c + dy * s, -dx * s + dy * c


@dataclass
class Obstaculo:
    color: str
    x: float                # centro, mundo
    y: float
    semi_lat: float = 25.0  # medio ancho de la huella, de lado
    semi_x: float = 25.0    # medio ancho de la huella, a lo largo
    ancho_img: float = 50.0 # ancho que ve la camara
    alto: float = 100.0
    # --- lo que se mide ---
    lado_visto: int = 0     # signo de 'derecha' al pasar el morro
    holgura: float = 1e9    # minimo de |derecha| - semi - semi_lat al costado
    colision: bool = False


@dataclass
class Resultado:
    obstaculos: List[Obstaculo]
    pared: bool = False
    escapes: int = 0
    vel_max_mandando: float = 0.0
    giro_hacia_costado: int = 0
    y_final: float = 0.0
    th_final_deg: float = 0.0
    x_final: float = 0.0
    traza: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
def config_obstaculos() -> Dict[str, Dict[str, Any]]:
    """Los parametros con los que corre el carro en el reto (config/obstaculos).
    Si no existen, los del esquema con el esquive encendido."""
    ruta = params_mod.ruta_de_reto("obstaculos")
    if ruta.exists():
        datos = params_mod.cargar(ruta)
        vals = params_mod.obtener(datos)["valores"]
    else:
        vals = params_mod.valores_por_defecto()
    vals["obstaculos"]["activo"] = True
    return vals


def config_defecto() -> Dict[str, Dict[str, Any]]:
    vals = params_mod.valores_por_defecto()
    vals["obstaculos"]["activo"] = True
    return vals


# ---------------------------------------------------------------------------
def ver(geo: Geometria, fwd_cam: float, right: float, ob: Obstaculo
        ) -> Optional[vision.Deteccion]:
    """Lo que el detector de color sacaria de este obstaculo, o None si la
    camara no lo ve (detras, fuera del cuadro por un canto, o por abajo)."""
    if fwd_cam < 60.0:
        return None
    u_izq, v_base = geo.suelo_a_pixel(right - ob.ancho_img / 2.0, fwd_cam)
    u_der, _ = geo.suelo_a_pixel(right + ob.ancho_img / 2.0, fwd_cam)
    v_cima = geo.fila_de_altura(fwd_cam, ob.alto)
    x0, x1 = min(u_izq, u_der), max(u_izq, u_der)
    y0, y1 = v_cima, v_base
    if x1 <= 0 or x0 >= W or y1 <= 0 or y0 >= H:
        return None
    x0c, x1c = max(0, x0), min(W - 1, x1)
    y0c, y1c = max(0, y0), min(H - 1, y1)
    w, h = x1c - x0c, y1c - y0c
    if w < 4 or h < 4:
        return None
    return vision.Deteccion(color=ob.color, x=x0c, y=y0c, w=w, h=h,
                            area=int(w * h * 0.9), llenado=0.9,
                            aspecto=h / float(w),
                            cx=(x0c + x1c) / 2.0, cy=(y0c + y1c) / 2.0)


def perfil_sintetico(geo: Geometria, carro: Carro, y_izq: float, y_der: float,
                     x_frente: float, cfg_muro: Dict[str, Any],
                     adelanto_cam: float, error_rumbo: Optional[float] = None,
                     sentido: int = 0) -> PerfilMuro:
    """Perfil del muro por trazado de rayos: las dos paredes del carril y una
    de frente, proyectadas al cuadro y reducidas a la fila de contacto por
    columna. Despues, la MISMA aritmetica que muro.perfil(), rectas y
    clasificacion incluidas (el navegador distingue la pared de frente de la
    de al lado con ellas, y esquivando el carro va cruzado)."""
    h = float(geo.cfg.get("alto_cam_mm", 125.0))
    tilt = math.radians(float(geo.cfg.get("inclinacion_deg", 7.5)))
    fy = float(geo.cfg.get("fy_px", 460.0)) * (H / 480.0)
    fx = float(geo.cfg.get("fx_px", 460.0)) * (W / 640.0)
    cx, cy = W / 2.0, H / 2.0

    # muestreo fino: de cerca cada milimetro de pared cae en una columna
    # distinta, y con pasos gruesos quedan columnas sin contacto (agujeros
    # que la camara de verdad no tiene)
    xs = np.arange(carro.x - 400.0, carro.x + 3600.0, 0.5)
    ys_f = np.arange(y_izq, y_der, 2.0)
    px = np.concatenate([xs, xs, np.full(ys_f.shape, x_frente)])
    py = np.concatenate([np.full(xs.shape, y_izq), np.full(xs.shape, y_der), ys_f])

    c, s = math.cos(carro.th), math.sin(carro.th)
    ox, oy = carro.x + adelanto_cam * c, carro.y + adelanto_cam * s
    dx, dy = px - ox, py - oy
    fwd = dx * c + dy * s
    right = -dx * s + dy * c
    ok = fwd > 30.0
    fwd, right = fwd[ok], right[ok]

    theta = np.arctan2(h, fwd)
    v = cy + fy * np.tan(theta - tilt)
    rango = np.sqrt(h * h + fwd * fwd)
    u = cx + right * fx / rango
    y_hor = max(0, geo.fila_horizonte() + int(cfg_muro.get("margen_horizonte_px", 4)))
    dentro = (u >= 0) & (u < W) & (v > y_hor)
    u = u[dentro].astype(np.int32)
    v = np.minimum(v[dentro], H - 1).astype(np.int32)

    y_cont = np.zeros(W, np.int32)
    np.maximum.at(y_cont, u, v)
    valido = y_cont > 0

    p = PerfilMuro(y_contacto=y_cont, valido=valido,
                   dist_mm=np.zeros(W, np.float32), libre=np.zeros(W, np.float32),
                   alto=H, ancho=W, y_horizonte=y_hor,
                   alcance_mm=float(cfg_muro.get("alcance_mm", 2500.0)))
    morro = float(geo.cfg.get("morro_mm", 60.0))
    d = geo.fila_a_distancia(y_cont.astype(np.float32)) - morro
    d = np.clip(d, 0.0, DIST_MAX_MM)
    d = np.where(valido, d, DIST_MAX_MM)
    d_suave = muro._media_movil(d.astype(np.float32), int(cfg_muro.get("suavizado", 7)))
    p.dist_mm = np.where(valido, d_suave, DIST_MAX_MM).astype(np.float32)
    p.libre = np.clip(p.dist_mm / p.alcance_mm, 0.0, 1.0).astype(np.float32)

    banda = float(cfg_muro.get("banda_lateral", 0.28))
    n_lat = max(1, int(W * banda))
    p.izq = float(p.libre[:n_lat].mean())
    p.der = float(p.libre[W - n_lat:].mean())
    a_i = valido[:n_lat] & (p.dist_mm[:n_lat] < p.alcance_mm * 0.98)
    a_d = valido[W - n_lat:] & (p.dist_mm[W - n_lat:] < p.alcance_mm * 0.98)
    p.cobertura_izq = float(np.count_nonzero(a_i)) / n_lat
    p.cobertura_der = float(np.count_nonzero(a_d)) / n_lat

    semi = (float(geo.cfg.get("ancho_carro_mm", 200.0)) / 2.0 +
            float(geo.cfg.get("margen_ruedas_mm", 30.0)))
    cols = np.arange(W, dtype=np.float32)
    lat = geo.lateral_mm(cols, np.maximum(y_cont, 1).astype(np.float32))
    en_camino = valido & (np.abs(lat) <= semi)
    p.pasillo_mm = (float(np.percentile(p.dist_mm[en_camino], 15))
                    if en_camino.any() else p.alcance_mm)
    p.pasillo = min(1.0, p.pasillo_mm / p.alcance_mm)
    p.min_mm = float(p.dist_mm.min()) if valido.any() else DIST_MAX_MM
    p.hay_muro = bool(valido.any())

    salto = float(cfg_muro.get("salto_borde_mm", 400.0))
    dd = np.abs(np.diff(p.dist_mm))
    ambos = valido[1:] | valido[:-1]
    ultimo = -10
    for i in np.nonzero((dd > salto) & ambos)[0]:
        if i - ultimo > 4:
            p.bordes.append((int(i), float(p.dist_mm[i]), float(p.dist_mm[i + 1])))
        ultimo = i
    try:
        p.segmentos, p.esquinas = muro._segmentos(p, geo, cfg_muro, lat)
        muro._clasificar_segmentos(p, cfg_muro, error_rumbo, sentido)
    except Exception:
        pass
    return p


# ---------------------------------------------------------------------------
def simular(obstaculos: List[Obstaculo], vals: Optional[Dict] = None,
            y_izq: float = -350.0, y_der: float = 350.0, x_frente: float = 6000.0,
            sentido: int = 1, ticks: int = 270, y0: float = 0.0, th0_deg: float = 0.0,
            radio_real: Optional[float] = None, p_fallo_det: float = 0.0,
            con_yaw: bool = True, semilla: int = 1, traza: bool = False) -> Resultado:
    vals = vals if vals is not None else config_obstaculos()
    geo = Geometria(vals["geometria"], W, H)
    largo = float(vals["geometria"]["largo_carro_mm"])
    morro = float(vals["geometria"]["morro_mm"])
    semi = float(vals["geometria"]["ancho_carro_mm"]) / 2.0
    eje_a_morro = largo - EJE_A_COLA
    adelanto_cam = eje_a_morro - morro
    radio_cfg = float(vals["geometria"].get("radio_giro_mm", 550.0))
    radio_real = radio_cfg if radio_real is None else radio_real
    delta_max = math.atan(ENTRE_EJES / radio_real)
    vmax = int(vals["limites"]["vmax"])
    vel_max_mm_s = float(vals["velocidad"]["vel_max_mm_s"])
    mandar = float(vals["obstaculos"]["mandar_desde_mm"])

    reloj = Reloj()
    t_obst, t_nav = obst_mod.time, nav_mod.time
    obst_mod.time = reloj          # type: ignore[assignment]
    nav_mod.time = reloj           # type: ignore[assignment]
    rng = random.Random(semilla)
    res = Resultado(obstaculos=obstaculos)
    try:
        esq = Esquivador(vals["obstaculos"])
        nav = Navegador(vals["navegacion"], vals["limites"], vals["escape"],
                        vals["giro2t"], cfg_obst=vals["obstaculos"],
                        cfg_color=vals["esquina_color"])
        if con_yaw:
            # la recta es el eje x del mundo: si el carro arranca cruzado, el
            # navegador tiene que saberlo (en pista, la referencia la pone la
            # salida alineada y cada curva la avanza 90 grados)
            nav.rumbo_recta = nav.rumbo_objetivo = 0.0
        carro = Carro(x=0.0, y=y0, th=math.radians(th0_deg))
        vel_pct, dir_pct = 0.0, 0.0
        estado_prev = ""
        for i in range(ticks):
            # --- camara ------------------------------------------------
            dets: Dict[str, List[vision.Deteccion]] = {"rojo": [], "verde": [],
                                                       "magenta": []}
            for ob in obstaculos:
                fwd_cam, right = carro.relativo(ob.x, ob.y, adelanto_cam)
                if p_fallo_det and rng.random() < p_fallo_det:
                    continue
                d = ver(geo, fwd_cam, right, ob)
                if d is not None:
                    dets[ob.color].append(d)
            yaw = math.degrees(carro.th) if con_yaw else None
            err = nav.error_de_rumbo(yaw)
            perfil = perfil_sintetico(geo, carro, y_izq, y_der, x_frente,
                                      vals["muro"], adelanto_cam, err, sentido)

            # --- decidir (igual que robot.py) ---------------------------
            vel_mm_s = vel_pct / 100.0 * vmax / 255.0 * vel_max_mm_s
            bias = esq.paso(dets, perfil, geo, None, False, sentido, vel_mm_s,
                            yaw=yaw, error_rumbo=err)
            info = esq.info
            en_juego = ("dist_mm" in info or "al_costado_mm" in info
                        or "adelantando_s" in info)
            d = nav.paso(perfil, yaw, sentido, False, bias, False, False,
                         pilar_en_juego=en_juego, freno_linea=1.0,
                         restriccion=esq.restriccion)
            vel_pct, dir_pct = float(d.vel), float(d.direccion)
            if d.estado == ESCAPE and estado_prev != ESCAPE:
                res.escapes += 1
            estado_prev = d.estado
            if "dist_mm" in info and info["dist_mm"] <= mandar and vel_pct > 0:
                res.vel_max_mandando = max(res.vel_max_mandando, vel_pct)
            if "al_costado_mm" in info and info.get("no_girar"):
                prohibido = 1 if info["no_girar"] == "derecha" else -1
                if dir_pct * prohibido > float(info.get("tope_hacia_pct", 0.0)) + 0.5:
                    res.giro_hacia_costado += 1

            # --- fisica --------------------------------------------------
            v_cmd = vel_pct / 100.0 * vmax / 255.0 * vel_max_mm_s
            carro.v += (v_cmd - carro.v) * min(1.0, DT / MOTOR_TAU_S)
            paso_max = SERVO_PCT_S * DT
            carro.dir_real += max(-paso_max, min(paso_max, dir_pct - carro.dir_real))
            delta = delta_max * carro.dir_real / 100.0
            carro.th += carro.v / ENTRE_EJES * math.tan(delta) * DT
            carro.x += carro.v * math.cos(carro.th) * DT
            carro.y += carro.v * math.sin(carro.th) * DT
            reloj.avanzar(DT)

            # --- medir ---------------------------------------------------
            for ob in obstaculos:
                fwd, right = carro.relativo(ob.x, ob.y, 0.0)
                if (-EJE_A_COLA - ob.semi_x < fwd < eje_a_morro + ob.semi_x
                        and abs(right) < semi + ob.semi_lat):
                    ob.colision = True
                if -EJE_A_COLA < fwd < eje_a_morro + ob.semi_x:
                    ob.holgura = min(ob.holgura, abs(right) - semi - ob.semi_lat)
                if ob.lado_visto == 0 and fwd < eje_a_morro:
                    ob.lado_visto = 1 if right > 0 else -1
            c_, s_ = math.cos(carro.th), math.sin(carro.th)
            for fx_, fy_ in ((eje_a_morro, semi), (eje_a_morro, -semi),
                             (-EJE_A_COLA, semi), (-EJE_A_COLA, -semi)):
                ey = carro.y + fx_ * s_ + fy_ * c_
                if ey <= y_izq or ey >= y_der:
                    res.pared = True
            if traza:
                pil = obstaculos[0]
                fwd, right = carro.relativo(pil.x, pil.y, eje_a_morro)
                res.traza.append(
                    f"{i:3d} t={reloj.t - 1000:4.2f}s x={carro.x:5.0f} y={carro.y:+5.0f} "
                    f"th={math.degrees(carro.th):+5.1f} vel={vel_pct:3.0f} dir={dir_pct:+4.0f} "
                    f"| pilar {fwd:+5.0f}/{right:+5.0f} | {d.estado} "
                    f"{('ARCO' if 'arco_x' in info else '')}"
                    f"{(' ciego' if info.get('ciego') else '')}"
                    f"{(' costado' if 'al_costado_mm' in info else '')}"
                    f"{(' bloq' if 'bloqueo_mm' in info else '')} "
                    f"pas={perfil.pasillo_mm:4.0f}/fr={perfil.frontal_mm if perfil.frontal_mm is None else round(perfil.frontal_mm)} "
                    f"esq={info.get('dir', '-')}x{info.get('peso', '-')} {d.motivo[:60]}")
        res.y_final, res.th_final_deg, res.x_final = carro.y, math.degrees(carro.th), carro.x
    finally:
        obst_mod.time = t_obst      # type: ignore[assignment]
        nav_mod.time = t_nav        # type: ignore[assignment]
    return res


# ---------------------------------------------------------------------------
LADO_OK = {"rojo": -1, "verde": +1}   # donde queda el pilar respecto al carro


def fallos_de(res: Resultado, holgura_min: float = 15.0,
              exigir_lado: bool = True, pared: bool = True) -> List[str]:
    f = []
    for ob in res.obstaculos:
        etq = f"{ob.color}@({ob.x:.0f},{ob.y:+.0f})"
        # Pilar pegado a la pared del lado de paso: entre el y la pared hay 7 cm
        # para repartir entre el pilar y la pared, asi que ahi basta con no
        # tocarlo (el reglamento no penaliza rozar la pared; mover el pilar si).
        apretado = (ob.color == "rojo" and ob.y >= 200.0) or \
                   (ob.color == "verde" and ob.y <= -200.0)
        minimo = min(holgura_min, 5.0) if apretado else holgura_min
        if ob.colision:
            f.append(f"{etq}: COLISION (holgura {ob.holgura:.0f} mm)")
        elif ob.holgura < minimo:
            f.append(f"{etq}: paso demasiado justo ({ob.holgura:.0f} mm)")
        if exigir_lado and ob.color in LADO_OK:
            if ob.lado_visto == 0:
                f.append(f"{etq}: nunca llego a pasarlo (x final {res.x_final:.0f})")
            elif ob.lado_visto != LADO_OK[ob.color]:
                f.append(f"{etq}: lo paso por el lado INCORRECTO")
    if pared and res.pared:
        f.append("toco una pared del carril")
    return f


PRUEBAS = []
TRAZAR = False


def prueba(nombre):
    def deco(fn):
        PRUEBAS.append((nombre, fn))
        return fn
    return deco


def _correr(obs, **kw) -> Resultado:
    r = simular(obs, traza=TRAZAR, **kw)
    if TRAZAR:
        print("\n".join(r.traza))
        print(f"   -> holgura {[round(o.holgura) for o in obs]} lado {[o.lado_visto for o in obs]} "
              f"pared={r.pared} escapes={r.escapes} vmax={r.vel_max_mandando:.0f} "
              f"y_fin={r.y_final:+.0f} th_fin={r.th_final_deg:+.0f}")
    return r


def pilar(color: str, x: float, y: float) -> Obstaculo:
    return Obstaculo(color=color, x=x, y=y)


def delimitador(x: float, y_pared: float) -> Obstaculo:
    """Delimitador del cajon (200x20x100) pegado a la pared exterior, visto
    de canto: 20 mm en la imagen, 200 de huella lateral."""
    lado = 1 if y_pared < 0 else -1
    return Obstaculo(color="magenta", x=x, y=y_pared + lado * 100.0,
                     semi_lat=100.0, semi_x=10.0, ancho_img=20.0)


# --- las pruebas -----------------------------------------------------------
# El carril del reto mide entre 600 y 1000 mm. Los pilares descentrados hacia
# el lado de paso solo caben en el carril ancho: un rojo a 25 cm a la derecha
# del centro de un carril de 700 deja 75 mm entre el y la pared, y por ahi no
# pasa un carro de 200. Esos casos no existen en la pista (el reglamento coloca
# las señales donde se pueden pasar), asi que se prueban donde caben.
ANCHO = dict(y_izq=-500.0, y_der=500.0)


@prueba("rojo: lo pasa por su derecha este donde este en el carril")
def t_rojo():
    f = []
    for lat in (-250.0, -120.0, 0.0, 120.0, 200.0):
        r = _correr([pilar("rojo", 1500.0, lat)], **ANCHO)
        f += [f"lat {lat:+.0f}: {x}" for x in fallos_de(r)]
    r = _correr([pilar("rojo", 1500.0, 0.0)])
    f += [f"carril 700, centrado: {x}" for x in fallos_de(r)]
    r = _correr([pilar("rojo", 1500.0, -150.0)])
    f += [f"carril 700, lat -150: {x}" for x in fallos_de(r)]
    return f


@prueba("verde: lo pasa por su izquierda este donde este")
def t_verde():
    f = []
    for lat in (-200.0, -120.0, 0.0, 120.0, 250.0):
        r = _correr([pilar("verde", 1500.0, lat)], **ANCHO)
        f += [f"lat {lat:+.0f}: {x}" for x in fallos_de(r)]
    r = _correr([pilar("verde", 1500.0, 0.0)])
    f += [f"carril 700, centrado: {x}" for x in fallos_de(r)]
    r = _correr([pilar("verde", 1500.0, 150.0)])
    f += [f"carril 700, lat +150: {x}" for x in fallos_de(r)]
    return f


@prueba("da igual el sentido de la ronda: en antihorario el rojo sigue por su derecha")
def t_antihorario():
    f = []
    for color, lat in (("rojo", 0.0), ("verde", 0.0), ("rojo", 200.0), ("verde", -200.0)):
        r = _correr([pilar(color, 1500.0, lat)], sentido=-1, **ANCHO)
        f += [f"{color} lat {lat:+.0f}: {x}" for x in fallos_de(r)]
    return f


@prueba("ver un pilar es frenar: mientras manda no va a velocidad de crucero")
def t_frena():
    vals = config_obstaculos()
    r = _correr([pilar("rojo", 1500.0, 0.0)], vals=vals)
    tope = float(vals["obstaculos"]["vel_esquive"])
    crucero = float(vals["limites"]["vel_crucero"])
    f = fallos_de(r)
    if r.vel_max_mandando > tope + 3.0:
        f.append(f"con el pilar dentro de mandar_desde_mm fue a {r.vel_max_mandando:.0f} % "
                 f"(vel_esquive {tope:.0f}, crucero {crucero:.0f})")
    return f


@prueba("dos pilares seguidos de colores distintos: una S, los dos por su lado")
def t_dos_en_s():
    """Entre el primero y el segundo hay 1,3 m: al pasar el primero el carro
    esta pegado a un lado y tiene que cruzar el carril entero antes del
    segundo. Con radio de giro de 55 cm, cambiar 35 cm de carril necesita
    unos 90 cm de recta (dos arcos a tope) y la cola tarda otros 25 en
    librar el primero: con menos recta que eso la S no cabe fisicamente, y
    el reglamento no pone dos señales de lado contrario tan seguidas."""
    f = []
    r = _correr([pilar("rojo", 1400.0, 0.0), pilar("verde", 2700.0, 0.0)], ticks=390)
    f += fallos_de(r, holgura_min=10.0)
    r = _correr([pilar("verde", 1400.0, 100.0), pilar("rojo", 2700.0, -100.0)], ticks=390)
    f += ["(al reves) " + x for x in fallos_de(r, holgura_min=10.0)]
    return f


@prueba("el pilar aparece cerca y de frente (saliendo de la curva): no lo choca")
def t_aparece_cerca():
    """Lo mas apretado que hay: pilar en el medio del carril de 700 a 65 cm
    del morro. Apartarse 20 cm en 65 cm es un arco al limite del radio de
    giro, asi que se llega al pilar todavia cruzado; lo que se exige es no
    tocarlo y pasarlo por su lado, aunque despues se roce la pared."""
    f = []
    for dist_morro in (650.0, 500.0):
        r = _correr([pilar("rojo", 200.0 + dist_morro, 0.0)], ticks=400)
        f += [f"a {dist_morro:.0f} mm: {x}"
              for x in fallos_de(r, holgura_min=5.0, pared=False)]
    return f


@prueba("sale de la curva torcido y con el pilar delante: lo pasa igual")
def t_torcido():
    f = []
    # apuntando 30 grados a la izquierda del carril (asi se sale de una curva
    # antihoraria a medio hacer), desplazado a la derecha, pilar en el medio
    r = _correr([pilar("rojo", 1100.0, 0.0)], th0_deg=-30.0, y0=120.0, ticks=330)
    f += ["rojo: " + x for x in fallos_de(r, holgura_min=5.0)]
    r = _correr([pilar("verde", 1100.0, 0.0)], th0_deg=30.0, y0=-120.0, ticks=330)
    f += ["verde: " + x for x in fallos_de(r, holgura_min=5.0)]
    return f


@prueba("el delimitador MAGENTA del cajon se rodea por el interior de la pista")
def t_magenta():
    f = []
    # horario: la pared exterior es la IZQUIERDA; el cajon esta pegado a ella
    r = _correr([delimitador(1500.0, -350.0)], sentido=1)
    f += ["horario: " + x for x in fallos_de(r, exigir_lado=False)]
    if r.obstaculos[0].lado_visto != -1:
        f.append("horario: no lo dejo a su izquierda (no paso por el interior)")
    # antihorario: la exterior es la DERECHA
    r = _correr([delimitador(1500.0, 350.0)], sentido=-1)
    f += ["antihorario: " + x for x in fallos_de(r, exigir_lado=False)]
    if r.obstaculos[0].lado_visto != 1:
        f.append("antihorario: no lo dejo a su derecha")
    return f


@prueba("el detector parpadea (30 % de frames perdidos) y lo pasa igual")
def t_parpadeo():
    f = []
    for semilla in (1, 2, 3):
        r = _correr([pilar("rojo", 1500.0, -100.0)], p_fallo_det=0.3, semilla=semilla)
        f += [f"semilla {semilla}: {x}" for x in fallos_de(r)]
    return f


@prueba("sin giroscopio (yaw=None) manda el compromiso por tiempo y no lo choca")
def t_sin_yaw():
    f = []
    for color, lat in (("rojo", 0.0), ("verde", 150.0)):
        r = _correr([pilar(color, 1500.0, lat)], con_yaw=False)
        f += [f"{color}: {x}" for x in fallos_de(r, holgura_min=5.0, pared=False)]
    return f


@prueba("radio de giro mal medido (el carro gira peor de lo que cree): lo pasa igual")
def t_radio_mal():
    f = []
    for radio in (750.0, 420.0):
        r = _correr([pilar("rojo", 1500.0, 0.0)], radio_real=radio)
        f += [f"radio real {radio:.0f}: {x}" for x in fallos_de(r, holgura_min=5.0)]
    return f


@prueba("carril estrecho (600 mm) con el pilar en el medio: cabe y no toca la pared")
def t_estrecho():
    f = []
    for color in ("rojo", "verde"):
        r = _correr([pilar(color, 1500.0, 0.0)], y_izq=-300.0, y_der=300.0)
        f += [f"{color}: {x}" for x in fallos_de(r, holgura_min=5.0)]
    return f


@prueba("al costado del pilar nunca gira hacia el, y despues vuelve al centro")
def t_costado_y_vuelta():
    f = []
    r = _correr([pilar("rojo", 1500.0, 0.0)], ticks=330)
    f += fallos_de(r)
    if r.giro_hacia_costado:
        f.append(f"giro hacia el pilar {r.giro_hacia_costado} ticks con el al costado")
    if abs(r.y_final) > 200.0 or abs(r.th_final_deg) > 15.0:
        f.append(f"no volvio al carril: y={r.y_final:+.0f} th={r.th_final_deg:+.0f}")
    return f


@prueba("los valores por defecto del esquema tambien pasan el pilar")
def t_defectos():
    f = []
    vals = config_defecto()
    for color, lat in (("rojo", 0.0), ("verde", -150.0), ("rojo", 200.0)):
        r = _correr([pilar(color, 1500.0, lat)], vals=vals, **ANCHO)
        f += [f"{color} lat {lat:+.0f}: {x}" for x in fallos_de(r, holgura_min=10.0)]
    r = _correr([pilar("verde", 1500.0, 0.0)], vals=vals)
    f += [f"carril 700 verde: {x}" for x in fallos_de(r, holgura_min=10.0)]
    return f


@prueba("(control) con el modo 'punto' de antes y el peso 0.8 el carro chocaba")
def t_control_antes():
    """No es una prueba del carro: es la prueba de que el simulador reproduce
    el fallo de pista. Si esto deja de fallar, o el simulador se ha vuelto
    demasiado benevolo o alguien arreglo 'punto'."""
    vals = config_obstaculos()
    # lo de antes: modo punto, el centrado con el 20 %, sin frenar, sin la
    # regla del costado, sin la reversa por pilar en el corredor
    vals["obstaculos"].update(modo="punto", peso_max=0.8, k_dir=1.4,
                              dir_max_pct=55.0, vel_esquive=100,
                              activar_desde_mm=1600.0, mandar_desde_mm=700.0,
                              no_volver_al_costado=False, pilar_parar_mm=60.0,
                              recuperar_ms=0, costado_desde_mm=0.0)
    chocados = 0
    for lat in (-120.0, 0.0, 120.0):
        r = _correr([pilar("rojo", 1500.0, lat)], vals=vals)
        if r.obstaculos[0].colision or r.obstaculos[0].holgura < 0:
            chocados += 1
    return [] if chocados >= 1 else ["el simulador ya no reproduce el choque de antes"]


def main() -> int:
    global TRAZAR
    TRAZAR = "--traza" in sys.argv
    solo = [a for a in sys.argv[1:] if not a.startswith("--")]
    print("El carro pasa el pilar (simulacion cinematica)\n")
    mal = 0
    for nombre, fn in PRUEBAS:
        if solo and not any(s in nombre for s in solo):
            continue
        try:
            fallos = fn()
        except Exception as e:                          # noqa: BLE001
            import traceback
            traceback.print_exc()
            fallos = [f"excepcion: {type(e).__name__}: {e}"]
        if fallos:
            mal += 1
            print(f"  FALLA  {nombre}")
            for d in fallos:
                print(f"           - {d}")
        else:
            print(f"  ok     {nombre}")
    print()
    if mal:
        print(f"{mal} de {len(PRUEBAS)} pruebas FALLAN: no lo subas al carro.")
    else:
        print(f"Las {len(PRUEBAS)} pruebas pasan.")
    return 1 if mal else 0


if __name__ == "__main__":
    raise SystemExit(main())
