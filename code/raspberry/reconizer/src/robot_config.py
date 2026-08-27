"""
robot_config.py — Ajustes del robot (config/robot.json).

Separado de colors.json a proposito: los colores se recalibran en cada pista y
rotan entre 5 perfiles; esto otro (puerto serie, ganancias, limites) cambia poco
y no quieres perderlo al cambiar de perfil de color.

Todo se puede tocar en caliente desde el panel o desde la web y guardarse.
"""

from __future__ import annotations

import copy
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, Optional, Union

RAIZ_PROYECTO = Path(__file__).resolve().parent.parent
RUTA_ROBOT = RAIZ_PROYECTO / "config" / "robot.json"

POR_DEFECTO: Dict[str, Any] = {
    "version": 1,

    "enlace": {
        # vacio = autodeteccion. Orden de busqueda en Linux:
        #   /dev/serial0, /dev/ttyAMA0, /dev/ttyAMA1, /dev/ttyUSB*, /dev/ttyACM*
        # y en Windows todos los COM. Se prueba cada uno y se queda con el
        # primero que conteste una trama de telemetria valida.
        "puerto": "",
        "baudios": 115200,
        "hz_envio": 50,          # tramas de mando por segundo
        "timeout_tele_ms": 500,  # sin telemetria = enlace caido
        "reintento_s": 2.0,
    },

    "camara": {
        "indice": 0, "ancho": 640, "alto": 480, "fps": 30,
        "fourcc": "MJPG", "voltear": False,

        # Geometria del montaje. Solo se usa mientras NO exista
        # config/suelo.json; en cuanto se calibra con tools/calibrar_suelo.py
        # manda la homografia medida y esto se ignora.
        # Subir la camara mejora directamente la precision en distancia: el
        # limite del reglamento son 300 mm de alto total.
        "altura_mm": 200.0,
        "cabeceo_deg": 20.0,     # cuanto mira hacia abajo respecto a la horizontal
        "hfov_deg": 70.0,        # campo horizontal REAL de la lente
        "vfov_deg": 0.0,         # 0 = deducirlo del horizontal
    },

    "limites": {
        # SIN CALIBRAR EL SUELO, estos tres se recortan solos a los valores de
        # `sin_calibrar`. Toda la navegacion trabaja en milimetros, y sin
        # homografia esos milimetros son inventados: correr con distancias
        # inventadas es como se choca. Ver Robot._revisar_calibracion.
        "vmax": 130,             # tope de PWM 0-255 que el ESP32 nunca supera
        "vel_crucero": 45,       # % de vmax en recta
        "vel_giro": 32,          # % de vmax girando
        "dir_max": 100,          # % de direccion que se permite pedir

        # Techo mientras el suelo no este calibrado. No es una preferencia:
        # es lo unico que hace que un error de escala se traduzca en un roce
        # en vez de en un golpe.
        "sin_calibrar": {"vmax": 110, "vel_crucero": 30, "vel_giro": 25},
    },

    # "abierto" = Open Challenge, "obstaculos" = Obstacle Challenge
    "reto": "abierto",

    "navegacion": {
        "color_muro": "negro",

        # --- lectura del muro (geometria.py) ---
        "alto_min_muro_px": 12,  # racha vertical continua minima para creerse
                                 # que hay muro. Es lo que separa un muro de
                                 # 100 mm de una sombra en el piso blanco.
        "ignorar_abajo": 0.06,   # franja inferior tapada por el chasis
        "suavizado_mm": 5,       # media movil del escaneo, en columnas

        # --- deteccion de la esquina convexa (el pivote del giro) ---
        "salto_min_mm": 260.0,   # salto de rango que delata al muro interno
        "salto_corrida": 6,      # columnas validas pegadas exigidas al lado
                                 # cercano del salto (mata los saltos de ruido)
        "salto_z_max_mm": 1600.0,
        # Columnas de los bordes donde NO se admite una esquina: ahi el perfil
        # se sale del encuadre y el rango salta igual que en una esquina de
        # verdad. Es una falsa esquina que ademas reaparece en cada frame.
        "salto_borde_px": 25,
        # La esquina interna es el borde del muro interno: su posicion lateral
        # tiene que parecerse a la distancia medida a ese muro.
        "esquina_tol_lateral_mm": 260.0,
        # Cuanto puede saltar una medida nueva respecto a la prediccion antes
        # de considerarla otra cosa y no la esquina que se venia siguiendo.
        "esquina_gate_mm": 400.0,
        "esquina_rechazos_max": 20,
        "frames_para_fijar_lado": 4,
        # La esquina se ve al principio de la recta y desaparece del encuadre
        # justo cuando te acercas. Se sigue por estima con yaw + velocidad.
        "esquina_memoria_s": 6.0,
        "esquina_atras_mm": 400.0,
        "ancho_corredor_inicial_mm": 800.0,
        "ancho_fiable_s": 4.0,
        "ancho_mezcla": 0.15,

        # --- seguimiento del muro interno ---
        # El sesgo hacia dentro es intencionado: la regla 9.18 prohibe TOCAR
        # el muro exterior en la prueba abierta, y rozar el interior sin
        # moverlo no penaliza. En un pasillo de 600 mm esto deja ~150 mm de
        # holgura interior y ~250 mm exterior con un carro de 200 mm.
        "pared_objetivo_mm": 250.0,
        "recta_z_desde_mm": 120.0,
        "recta_z_hasta_mm": 900.0,
        # Control en cascada. El error lateral manda sobre el ANGULO de
        # aproximacion (acotado), y el volante persigue ese angulo. Ver el
        # comentario largo en navegacion._paso_recto: la version directa
        # satura el volante y cruza el corredor en diagonal.
        "aprox_grados_por_mm": 0.06,  # 250 mm de error -> 15 grados
        "aprox_max_grados": 22.0,     # nunca se ataca la pared mas inclinado
        "aprox_max_grados_senal": 30.0,  # esquivar un pilar si admite mas prisa
        "kp_rumbo": 2.5,              # % de direccion por grado
        "kd_rumbo": 0.15,
        "aprox_max_grados_buscando": 14.0,
        "kp_centrado": 60.0,     # solo en BUSCANDO, antes de saber el sentido

        # --- guardia contra el muro externo (regla 9.18) ---
        "min_externo_mm": 150.0,
        "kp_externo": 0.5,

        # --- disparo y ejecucion del giro ---
        # RADIO DE GIRO REAL del carro, medido con tiza en el suelo. De aqui
        # sale solo la distancia de disparo: z_disparo = radio - objetivo.
        # Es el numero mas importante de todo el archivo.
        "radio_giro_mm": 350.0,
        "giro_z_mm": 0.0,        # 0 = automatico desde radio_giro_mm.
                                 # Un valor > 0 lo fuerza a mano.
        "giro_frente_mm": 300.0, # red de seguridad si no se ve la esquina
        "giro_grados": 90.0,     # la pista es cuadrada
        "giro_kp": 2.8,          # salida proporcional: evita el sobregiro
        "giro_tolerancia": 5.0,
        "giro_max_ms": 4000,
        "giro_min_ms": 700,      # solo para el cierre sin giroscopio
        "giro_paralelo_grados": 12.0,
        "dir_giro": 85.0,        # direccion fija si no hay giroscopio
        # Guardia durante el giro: si la esquina interna se acerca mas que
        # esto (holgura de CARROCERIA, no del centro) se afloja el volante y
        # el arco se abre. Es la red que evita que un `radio_giro_mm` mal
        # medido acabe en un golpe. Ver navegacion._abrir_si_roza.
        "giro_min_interno_mm": 130.0,
        "giro_mira_z_mm": 500.0,
        "giro_abrir_min": 0.35,  # nunca se afloja mas: hay que seguir girando
        "min_recto_ms": 500,     # espera minima entre dos esquinas seguidas

        # --- velocidad y seguridad ---
        "semiancho_carro_mm": 110.0,
        "semi_largo_carro_mm": 150.0,
        "frenar_mm": 650.0,
        # OJO: `parar_mm` tiene que estar POR ENCIMA de la distancia minima
        # medible (unos 200 mm con ignorar_abajo=0.06 y la camara a 200 mm).
        # Si se pone por debajo, la parada no puede dispararse nunca porque el
        # muro desaparece del encuadre antes de llegar al umbral. El robot
        # avisa al arrancar si estos numeros no cuadran.
        "parar_mm": 280.0,
        "salir_bloqueo_mm": 430.0,
        # Segundos de cada tiempo de la maniobra de desatasco
        # (atras / adelante / atras...). Ver navegacion._desatascar.
        "bloqueo_alterna_s": 1.4,
        "memoria_frente_mm": 700.0,   # por debajo de esto, un muro que sale del
        "memoria_frente_s": 2.0,      # encuadre se recuerda en vez de olvidarse
        "cobertura_min": 0.25,   # si se ve menos muro que esto, no acelerar:
                                 # desconocido no es lo mismo que despejado

        # --- giroscopio ---
        "usar_yaw": True,
        "yaw_kp": 1.2,
        "yaw_max": 30.0,

        # --- final de carrera ---
        # Sin encoder la distancia se integra de la velocidad pedida por esta
        # constante, medida a mano una vez. Con encoder, sustituir por el
        # contador real de la telemetria.
        "mm_por_seg_a_100": 900.0,
        "parada_tras_giro_mm": 700.0,

        # --- solo para dibujar ---
        "ruedas_izq": 0.32,
        "ruedas_der": 0.68,
    },

    "obstaculos": {
        # ROJO  -> el carro pasa por la DERECHA del pilar
        # VERDE -> el carro pasa por la IZQUIERDA del pilar
        "senal_z_min_mm": 120.0,
        "senal_z_max_mm": 1600.0,
        "senal_z_soltar_mm": 220.0,   # mas cerca que esto ya no se corrige
        "senal_alto_min_px": 10,
        "senal_aspecto_min": 1.1,     # un pilar 50x50x100 siempre es mas alto
        # 100 (medio carro) + 25 (medio pilar) + holgura. Con 190 la holgura
        # real quedaba en 65 mm y el simulador rozaba pilares: 240 deja 115.
        "senal_margen_mm": 245.0,
        # Distancia minima a un muro que un esquive NO puede invadir.
        "hueco_muro_senal_mm": 220.0,
        # mm que se sostiene el desvio despues de soltar la señal, para
        # rebasarla de verdad antes de volver a la pared.
        "senal_mantener_mm": 450.0,
        # Tope del desvio lateral. Puede ser generoso porque el navegador lo
        # acota ademas contra los muros medidos, que es el limite que de
        # verdad importa: con 380 el carro no llegaba a apartarse lo
        # suficiente de un pilar centrado y lo rozaba.
        "senal_desvio_max_mm": 600.0,
        "senal_salto_mm": 260.0,
        # Mas lateral que esto la señal esta en OTRO corredor, vista de reojo
        # por encima de la esquina: no es nuestra.
        "senal_x_max_mm": 650.0,
    },

    "imu": {
        "activo": True,          # si no aparece el chip, se sigue sin el
        # "esp32" = el MPU6050 cuelga del ESP32 y el yaw llega en la
        # telemetria. "pi" = colgado del I2C de la Raspberry (montaje
        # anterior). Ver la cabecera de firmware/esp32_carro/sensores.h.
        "fuente": "esp32",
        "bus": 1,                # solo si fuente = "pi"
        "direcciones": [104, 105],   # 0x68 y 0x69
        "hz": 100,
        "alfa_complementario": 0.98,
        "muestras_calibracion": 400,
        # El navegador espera yaw en convenio de BRUJULA: aumenta al girar a la
        # DERECHA. Comprueba con el carro en la mano: girandolo a la derecha, el
        # yaw del panel tiene que subir. Si baja, pon esto en true. Con el signo
        # al reves los giros salen hacia el lado contrario.
        "invertir_yaw": False,
    },

    # ---- TCS34725 mirando al suelo: cruces de linea naranja y azul ----
    "piso": {
        "activo": True,          # si no aparece el chip, se sigue sin el
        "fuente": "esp32",       # "esp32" o "pi", igual que el giroscopio
        "bus": 1,                # solo si fuente = "pi"
        # EL PARAMETRO QUE DECIDE TODO. A 0,4 m/s una linea de 20 mm pasa por
        # debajo del sensor en 50 ms: con los 154 o 700 ms que traen por
        # defecto muchas librerias, la linea se promedia con el piso blanco y
        # no se ve NUNCA. Con 24 ms caben dos muestras dentro de la linea.
        "integracion_ms": 24.0,
        "ganancia": 4,           # 1, 4, 16 o 60
        "clear_min": 60,         # por debajo, a oscuras: no se decide
        "clear_max": 65000,      # saturado: la medida no vale
        "muestras_min": 2,       # muestras seguidas para creerse una linea
        "separacion_min_s": 0.35,  # antirrebote entre dos cruces
        # Medidos con tools/calibrar_piso.py SOBRE EL TAPETE DE COMPETENCIA.
        # Los de aqui son un punto de partida, no una calibracion.
        # OJO: con fuente = "esp32" la clasificacion la hace el FIRMWARE, asi
        # que estos numeros hay que copiarlos tambien al array `perfiles` de
        # esp32_carro.ino y volver a subirlo. El calibrador los imprime en el
        # formato de C++ justo para eso.
        "perfiles": [
            {"nombre": "blanco", "r": 0.33, "g": 0.34, "b": 0.33, "tol": 0.05},
            {"nombre": "naranja", "r": 0.55, "g": 0.30, "b": 0.15, "tol": 0.09},
            {"nombre": "azul", "r": 0.20, "g": 0.32, "b": 0.48, "tol": 0.09},
        ],
        # Cuantas lineas se cruzan por vuelta. SIN VERIFICAR: el reglamento
        # 2026 dice que las lineas existen, su grosor y su color, pero no
        # donde estan. Cuentalas sobre el tapete antes de fiarte.
        "lineas_por_vuelta": 4,
        # Deducir el sentido del ORDEN de las dos primeras lineas. Desactivado
        # a proposito hasta confirmar el orden real: el sentido se deduce de la
        # primera esquina, que es geometrico y no depende de ningun plano.
        "usar_para_sentido": False,
        "orden_horario": ["naranja", "azul"],
    },

    "red": {
        "puerto_http": 8080,
        "calidad_jpeg": 70,
        "fps_stream": 15,
        "ancho_stream": 640,
        "hostname": "carrito",
    },
}


def _fusionar(base: Dict[str, Any], encima: Any) -> Dict[str, Any]:
    salida = copy.deepcopy(base)
    if isinstance(encima, dict):
        for k, v in encima.items():
            if k in salida and isinstance(salida[k], dict) and isinstance(v, dict):
                salida[k] = _fusionar(salida[k], v)
            else:
                salida[k] = v
    return salida


def cargar(ruta: Optional[Union[str, Path]] = None) -> Dict[str, Any]:
    """Nunca falla: si el archivo no existe o esta roto, devuelve los valores
    por defecto. Las claves nuevas de una version futura aparecen solas."""
    ruta = Path(ruta) if ruta else RUTA_ROBOT
    if not ruta.exists():
        cfg = copy.deepcopy(POR_DEFECTO)
        guardar(cfg, ruta)
        return cfg
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            datos = json.load(f)
    except Exception as e:
        print(f"[robot_config] {ruta} ilegible ({e}); uso valores por defecto")
        return copy.deepcopy(POR_DEFECTO)
    return _fusionar(POR_DEFECTO, datos)


def guardar(cfg: Dict[str, Any], ruta: Optional[Union[str, Path]] = None) -> Path:
    ruta = Path(ruta) if ruta else RUTA_ROBOT
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(ruta.parent), prefix=".robot_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(_fusionar(POR_DEFECTO, cfg), f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, ruta)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return ruta
