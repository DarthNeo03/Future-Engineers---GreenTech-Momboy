"""
config.py — Un solo sitio con todos los numeros ajustables.

POR QUE TODO AQUI Y NO REPARTIDO POR EL CODIGO
En la pista de practica hay veinte minutos y dos personas. Buscar una ganancia
dentro de un modulo, editarla, y acordarse de deshacerlo si sale peor, es como
se pierden las tardes. Con un unico JSON: se edita, se relanza, y si sale mal
se recupera del git. Ademas, los valores por defecto viven en el codigo, asi
que un JSON corrupto o incompleto NO deja el carro sin arrancar: se rellena
con lo que falte y se avisa por consola.

QUE NO VA AQUI: nada que el reglamento fije. El ancho del carril (1000 mm), el
tamaño del pilar (50x100) y el lado por el que se rebasa cada color son
constantes del juego, no parametros: viven en el modulo que los usa, con la
cita de la regla al lado. Si un dia el reglamento cambia, se busca la cita.
"""

from __future__ import annotations

import copy
import json
import os
from typing import Any, Dict

RUTA_POR_DEFECTO = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "pista.json")


# ---------------------------------------------------------------------------
# Valores de partida. Los de vision estan calibrados para el tapete WRO bajo
# luz de taller; los de control, para nuestro carro de 200 x 180 mm con motor
# de 500 rpm y servo MG996R.
# ---------------------------------------------------------------------------
DEFECTOS: Dict[str, Any] = {

    "camara": {
        "indice": 0,
        "ancho": 640,
        "alto": 480,
        "fps": 30,
        "fourcc": "MJPG",
        "voltear": False,
    },

    # Servo: lo unico que la Pi le dice al firmware sobre el hardware. Los
    # topes de compilacion del ESP32 siempre ganan; esto solo puede estrechar.
    "servo": {
        "centro": 100,
        "izquierda": 65,
        "derecha": 135,
        "grados_por_seg": 320,
        "rampa_motor": 10,
        "ms_freno_inversion": 150,
        # PWM MINIMO CON EL QUE EL MOTOR DE VERDAD ARRANCA.
        # Un motor de 500 rpm cargado con el carro y la reduccion del
        # diferencial no se mueve con PWM 40: zumba, calienta el IBT-2 y desde
        # fuera parece que el carro "se traba". Por debajo de este valor el
        # firmware empuja hasta el minimo util. Subelo de 5 en 5 hasta que el
        # carro arranque limpio desde parado en el tapete, y no mas.
        "pwm_min_motor": 55,
    },

    # Geometria de la camara: altura del centro optico, inclinacion y focales.
    # fy/fx se calibran con tools/calibrar_camara.py y NO se tocan a ojo.
    "geometria": {
        "alto_cam_mm": 125.0,
        "inclinacion_deg": 7.5,
        "fy_px": 460.0,
        "fx_px": 460.0,
        "ancho_carro_mm": 200.0,
        "largo_carro_mm": 200.0,
        "margen_ruedas_mm": 30.0,
    },

    # Conversion de % de mando a velocidad real. Se mide una vez: se manda un
    # 40 % durante 3 s en recta y se divide la distancia entre el tiempo.
    "traccion": {
        "mm_s_por_pct": 22.0,      # 40 % -> ~880 mm/s
        "vmax_pwm": 240,           # techo duro de PWM que manda la Pi
    },

    "carril": {
        "ancho_carril_mm": 1000.0,
        "dist_curva_mm": 820.0,    # al ir mas rapido hay que frenar antes
        "dist_recta_mm": 1800.0,
        "mirada_min_mm": 500.0,
        # Cono angular que cuenta como "lo que hay delante". En grados, no en
        # fraccion de imagen: asi no cambia de significado al tocar la
        # resolucion o la lente.
        "cono_frente_deg": 12.0,
        "ventana_centrado_mm": 1500.0,  # hasta donde se miran los muros para
                                        #   medir la separacion lateral
        # Y hasta que fraccion de lo que hay DELANTE. Un muro frontal aparece
        # en todos los rumbos, asi que sus sectores centrales dan un lateral
        # ridiculo y el carro se cree con la pared encima en cada esquina. Solo
        # es costado lo que esta mas cerca que el frente.
        "frac_frente_lateral": 0.85,
        "rumbo_max_deg": 30.0,          # rumbo que satura el termino a 1.0
        # SESGO DE CURVA. De frente a una esquina el hueco puede quedar casi
        # centrado y el carro se iba recto contra el muro. Si ya se sabe hacia
        # donde giran las curvas de esta ronda, se empuja hacia ese lado.
        "sesgo_curva": 45.0,
        "hueco_indeciso_deg": 10.0,     # por debajo, el hueco no decide nada
        "aprender_curva_deg": 6.0,      # rumbo medio minimo para dar por
                                        #   aprendido el sentido de las curvas
        "kp_centrado": 55.0,
        "kp_rumbo": 70.0,
        "kd_giro": 0.28,
        # GUARDIA ANTI-MURO. Solo actua mientras se adelanta un pilar, que es
        # cuando el centrado esta callado. Por debajo de esta separacion
        # lateral empuja hacia el otro lado.
        "guardia_muro_mm": 240.0,
        "k_guardia": 70.0,
        "suavizado": 0.45,
        # VELOCIDADES. Subidas respecto a la primera version porque el carro
        # iba sobrado de margen. Si al subirlas se empieza a comer pilares en
        # las curvas, lo primero que hay que bajar es vel_curva, no vel_recta.
        "vel_recta": 58.0,
        "vel_curva": 34.0,
        "freno_por_volante": 0.28,
    },

    "senales": {
        "activar_desde_mm": 1600.0,   # se empieza a tener en cuenta
        "mandar_desde_mm": 850.0,     # manda del todo sobre el carril
        "juzgar_lado_desde_mm": 700.0,  # desde aqui se juzga si vamos por el
                                        #   lado malo; mas lejos aun hay sitio
        "usar_limite_seccion": True,  # descartar pilares de la seccion
                                      #   siguiente. Si en pista se ve que
                                      #   ignora pilares buenos, ponlo en false
        "linea_valida_desde_mm": 450.0,  # una linea mas cerca es la que se
                                         #   esta pisando, no una frontera
        "ciego_desde_mm": 400.0,      # deja de verse: arranca el compromiso
        "dist_reversa_mm": 230.0,     # sin radio para corregir: reversa
        "margen_mm": 70.0,            # holgura al costado del pilar
        "morro_mm": 60.0,             # del eje de la camara al morro
        "mirada_min_mm": 420.0,
        "ganancia": 2.1,
        # Proporcional del rumbo durante el adelantamiento a ciegas. Manda
        # sobre el RUMBO (yaw del MPU), no sobre el volante.
        "kp_rumbo_compromiso": 2.6,
        "kd_rumbo_compromiso": 0.22,
        # SALIDA DEL COMPROMISO. Fraccion del compromiso a partir de la cual
        # el rumbo objetivo deja de ser el congelado y se lleva hacia el del
        # pasillo libre. Antes de esto el pilar sigue al costado y volver
        # hacia el lo barreria con la cola; despues ya quedo atras y lo que
        # importa es no salir del rebase apuntando al muro. Subirlo = salir
        # mas tarde y mas cruzado; bajarlo = arriesgarse a rozar el pilar.
        "salida_desde": 0.55,
        # Sin MPU no hay rumbo que sostener: en cuantos segundos se sueltan
        # las ruedas. No es fraccion del compromiso a proposito — un volante
        # fijo traza un arco, y el arco no espera a que acabe la maniobra.
        "soltar_sin_mpu_s": 0.35,
        "suavizado": 0.5,
        "holgura_linea_mm": 80.0,     # margen al descartar pilares de la
                                      #   seccion siguiente
        "largo_carro_mm": 200.0,
        "extra_mm": 120.0,
        "compromiso_max_s": 1.6,
        "memoria_s": 3.0,
    },

    "vueltas": {
        # CUAL DE LOS DOS COLORES SE PISA AL ENTRAR EN CURVA EN SENTIDO
        # HORARIO. Se comprueba UNA VEZ en la pista de practica empujando el
        # carro a mano y mirando la telemetria. No se adivina.
        "color_entrada_horario": "naranja",
        # Pista que puede haber entre las DOS lineas de una misma esquina. La
        # seccion de curva mide 1000 mm; con 1500 sobra margen para el 15 % de
        # error del odometro y sigue sin llegar a la esquina siguiente, que
        # esta a varios metros. Si se pasara, una linea suelta se emparejaria
        # con la de la curva de al lado y el orden saldria invertido.
        "ventana_par_mm": 1500.0,
        "margen_meta_mm": 120.0,
        # Cruces que caben de verdad entre dos ciclos de la Pi. Por encima no
        # son lineas: es el contador del ESP32 reiniciado o un salto de
        # sincronismo, y contarlos regala vueltas que no ocurrieron.
        "max_cruces_por_ciclo": 3,
        # Ultima red: tres vueltas no caben en tres metros. El recorrido real
        # de una vuelta ronda los 8 m.
        "min_dist_por_vuelta_mm": 4000.0,
    },

    "fsm": {
        "vueltas": 3,
        "arranque_s": 0.4,
        "vel_arranque": 34.0,
        "vel_senal": 44.0,
        "vel_esquive": 42.0,
        "vel_correccion": 28.0,
        # REINCORPORACION: el tramo entre "ya rebase" y "ya estoy en carril".
        # Se va despacio a proposito: es donde hay que recolocarse y donde
        # aparece el pilar siguiente, y las dos cosas piden frames por metro.
        "vel_reincorporacion": 34.0,
        "reincorporacion_max_s": 1.2,   # red por si no se llega a centrar
        "reincorporado_err": 0.30,      # error de centrado que ya vale
        "reincorporado_deg": 14.0,      # rumbo al hueco que ya vale
        "vel_meta": 30.0,
        "vel_reversa": 28.0,
        "dir_reversa": 70.0,
        "reversa_s": 0.9,
        "atasco_dist_mm": 260.0,
        "atasco_movimiento_mm": 45.0,
        "atasco_s": 1.2,
    },

    # -------------------------------------------------------- vision (HSV)
    # H va de 0 a 179 en OpenCV (no 0-359). S y V, de 0 a 255.
    # El ROJO necesita DOS rangos porque el tono es circular y el rojo esta a
    # caballo del cero.
    "colores": {
        "rojo": {
            "rangos": [[[0, 110, 70], [10, 255, 255]],
                       [[168, 110, 70], [179, 255, 255]]],
            "abrir": 3, "cerrar": 5, "area_min": 320,
            "aspecto_min": 0.6, "aspecto_max": 4.5, "llenado_min": 0.5,
            # Tolerancia al error de inclinacion del mastil, en pixeles.
            # 70 px aguantan ~8 grados de montaje torcido sin perder pilares.
            "margen_horizonte_px": 70,
            "max_objetos": 4,
        },
        "verde": {
            "rangos": [[[42, 80, 55], [88, 255, 255]]],
            "abrir": 3, "cerrar": 5, "area_min": 320,
            "aspecto_min": 0.6, "aspecto_max": 4.5, "llenado_min": 0.5,
            # Tolerancia al error de inclinacion del mastil, en pixeles.
            # 70 px aguantan ~8 grados de montaje torcido sin perder pilares.
            "margen_horizonte_px": 70,
            "max_objetos": 4,
        },
        # Delimitadores del cajon. NO son objetivo: son muro intocable.
        "magenta": {
            "rangos": [[[140, 90, 70], [166, 255, 255]]],
            "abrir": 3, "cerrar": 5, "area_min": 400,
            "margen_horizonte_px": 70,
            "max_objetos": 3,
        },
        # Muros: negro es poca V, sin importar el tono.
        "negro": {
            "rangos": [[[0, 0, 0], [179, 255, 88]]],
            "abrir": 3, "cerrar": 7, "area_min": 1200,
            "max_objetos": 3,
        },
        "naranja": {
            "rangos": [[[8, 120, 90], [24, 255, 255]]],
            "abrir": 3, "cerrar": 5, "area_min": 500,
            "ancho_min_frac": 0.18, "max_objetos": 2,
        },
        "azul": {
            "rangos": [[[95, 90, 60], [125, 255, 255]]],
            "abrir": 3, "cerrar": 5, "area_min": 500,
            "ancho_min_frac": 0.18, "max_objetos": 2,
        },
    },
}


def _fundir(base: Dict[str, Any], encima: Dict[str, Any]) -> Dict[str, Any]:
    """Mezcla recursiva. Lo que falte en 'encima' se queda del defecto, para
    que un JSON a medio escribir no deje el carro sin arrancar."""
    salida = copy.deepcopy(base)
    for k, v in (encima or {}).items():
        if isinstance(v, dict) and isinstance(salida.get(k), dict):
            salida[k] = _fundir(salida[k], v)
        else:
            salida[k] = v
    return salida


def cargar(ruta: str = RUTA_POR_DEFECTO, verbose: bool = True) -> Dict[str, Any]:
    if not os.path.exists(ruta):
        if verbose:
            print(f"[config] {ruta} no existe: se usan los valores por defecto")
        return copy.deepcopy(DEFECTOS)
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            datos = json.load(f)
    except (OSError, ValueError) as err:
        # Aqui NO se lanza la excepcion a proposito: un JSON roto en la mesa de
        # jueces no puede ser el motivo de que el carro no salga.
        print(f"[config] {ruta} ilegible ({err}); se usan los defectos")
        return copy.deepcopy(DEFECTOS)
    if verbose:
        print(f"[config] {ruta}")
    return _fundir(DEFECTOS, datos)


def guardar(cfg: Dict[str, Any], ruta: str = RUTA_POR_DEFECTO) -> None:
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
