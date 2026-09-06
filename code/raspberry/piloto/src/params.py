"""
params.py — Todos los parametros del robot, con esquema autodocumentado.

Cada parametro declara tipo, limites y UNA descripcion en castellano. La web
construye sus sliders/campos leyendo este esquema, asi que agregar un
parametro aqui lo hace aparecer solo en la interfaz, ya validado.

Igual que los colores, los parametros se guardan en PERFILES rotativos (los 5
mas recientes) en config/params.json: puedes calibrar "pista de casa" y
"pabellon" y cambiar entre ellos con un toque durante las pruebas.
"""

from __future__ import annotations

import copy
import datetime as _dt
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional

RAIZ_PROYECTO = Path(__file__).resolve().parent.parent
RUTA_PARAMS = RAIZ_PROYECTO / "config" / "params.json"
MAX_PERFILES = 5


def _p(tipo: str, defecto, desc: str, minimo=None, maximo=None,
       opciones: Optional[List[str]] = None, paso=None,
       auto=None, auto_label: str = "AUTO", auto_off=None) -> Dict[str, Any]:
    """Declara un parametro.

    auto:       valor centinela que significa "deja que lo decida el sistema"
                (por ejemplo -1 = exposicion automatica). La web le pone un
                boton AUTO en vez de obligar a clavar ese numero con el slider,
                que en un rango de -14 a 1000 es sencillamente imposible.
    auto_off:   valor que se aplica al APAGAR el modo automatico, para que el
                boton sirva en los dos sentidos y caiga en algo razonable.
    """
    d: Dict[str, Any] = {"tipo": tipo, "def": defecto, "desc": desc}
    if minimo is not None:
        d["min"] = minimo
    if maximo is not None:
        d["max"] = maximo
    if opciones:
        d["opciones"] = opciones
    if paso is not None:
        d["paso"] = paso
    if auto is not None:
        d["auto"] = auto
        d["auto_label"] = auto_label
        d["auto_off"] = auto_off if auto_off is not None else defecto
    return d


# ===========================================================================
# EL ESQUEMA. Grupo -> clave -> {tipo, def, min, max, desc}
# ===========================================================================
ESQUEMA: Dict[str, Dict[str, Dict[str, Any]]] = {

    "limites": {
        "vmax": _p("int", 130, "Tope absoluto de PWM (0-255) que el ESP32 nunca supera. Es el freno de mano de todas las pruebas.", 0, 255),
        "vel_crucero": _p("int", 55, "Velocidad en recta, en % de vmax.", 0, 100),
        "vel_giro": _p("int", 38, "Velocidad durante los giros de esquina, en % de vmax.", 0, 100),
        "vel_reversa": _p("int", 35, "Velocidad de la marcha atras del escape, en % de vmax.", 0, 100),
        "dir_max": _p("int", 100, "Tope de direccion que se permite pedir, en %.", 0, 100),
    },

    "camara": {
        "indice": _p("int", 0, "Indice de la camara frontal (0 = primera USB).", 0, 8),
        "ancho": _p("int", 640, "Ancho de captura en pixeles.", 160, 1920),
        "alto": _p("int", 480, "Alto de captura en pixeles.", 120, 1080),
        "fps": _p("int", 30, "FPS pedidos a la camara.", 5, 60),
        "voltear": _p("bool", False, "Girar la imagen 180 grados (camara montada al reves)."),
        "exposicion": _p("float", -1.0, "Exposicion de la camara. Pulsa AUTO para dejarsela a la camara; apagalo para fijarla a mano. CONGELALA antes de calibrar colores: en automatico el HSV cambia solo al girar hacia una pared clara. Ojo: en Windows/DSHOW los valores utiles son negativos (-14 a -1) y en Linux/V4L2 positivos (1 a 1000+), asi que prueba y mira la imagen.", -14.0, 1000.0, auto=-1.0, auto_label="AUTO", auto_off=-6.0),
        "balance_blancos": _p("float", -1.0, "Temperatura del balance de blancos, en kelvin. Con AUTO lo decide la camara; fijalo a mano (unos 4000-5000 con luz de pabellon) para que el HSV no cambie solo.", 2000.0, 10000.0, auto=-1.0, auto_label="AUTO", auto_off=4500.0),
        "indice_trasera": _p("int", -1, "Indice de la camara trasera para el estacionamiento. Con NINGUNA no se usa. Reservado para el futuro; el codigo ya no se opone.", 0, 8, auto=-1, auto_label="NINGUNA", auto_off=1),
    },

    "geometria": {
        "alto_cam_mm": _p("float", 125.0, "Altura del centro de la lente sobre el suelo, en mm. Midela con regla.", 50.0, 400.0),
        "inclinacion_deg": _p("float", 7.5, "Inclinacion de la camara hacia abajo desde la horizontal, en grados.", 0.0, 45.0),
        "fy_px": _p("float", 460.0, "Focal vertical en pixeles (referida a 480 de alto). Se calibra con un clic sobre un objeto a distancia conocida en la pestaña Calibracion.", 100.0, 2000.0),
        "fx_px": _p("float", 460.0, "Focal horizontal en pixeles (referida a 640 de ancho). Se calibra con un clic sobre un objeto desplazado a un lado.", 100.0, 2000.0),
        "ancho_carro_mm": _p("float", 200.0, "Ancho total del carro con ruedas, en mm. Define el corredor dibujado por donde van a pasar las ruedas.", 100.0, 300.0),
        "margen_ruedas_mm": _p("float", 30.0, "Margen de seguridad extra a cada lado del carro para el corredor, en mm.", 0.0, 150.0),
        "morro_mm": _p("float", 60.0, "Distancia de la lente al frente del carro, en mm. Las distancias mostradas se miden desde el morro.", 0.0, 300.0),
    },

    "muro": {
        "metodo": _p("str", "piso", "Como encontrar la base del muro. 'piso': primera transicion piso->no-piso subiendo desde abajo (robusto al brillo de la pared). 'negro': pixel negro mas bajo (el metodo viejo).", opciones=["piso", "negro"]),
        "alcance_mm": _p("float", 2500.0, "Distancia a partir de la cual una columna se considera despejada (define el 1.0 del espacio libre normalizado).", 500.0, 6000.0),
        "k_transicion": _p("int", 6, "Filas seguidas de no-piso para dar por buena la transicion piso->muro. Subelo si sombras o rayas del tapete crean muros fantasma; bajalo si el muro cercano se detecta tarde.", 2, 30),
        "margen_horizonte_px": _p("int", 4, "Pixeles de margen que se añaden bajo la fila del horizonte antes de empezar a buscar muro. Todo lo de arriba se ignora por geometria (ahi no puede haber pista).", 0, 60),
        "ignorar_abajo": _p("float", 0.05, "Fraccion inferior de la imagen tapada por el chasis del carro, que se ignora.", 0.0, 0.4),
        "px_min_columna": _p("int", 4, "(metodo 'negro') pixeles negros minimos en una columna para creer que hay muro.", 1, 60),
        "suavizado": _p("int", 7, "Promedio movil del perfil, en columnas. Alisa el ruido sin comerse las esquinas.", 0, 41),
        "banda_lateral": _p("float", 0.28, "Ancho (fraccion de imagen) de las bandas izquierda/derecha con las que se compara espacio libre.", 0.1, 0.45),
        "salto_borde_mm": _p("float", 400.0, "Discontinuidad de distancia entre columnas vecinas que cuenta como borde de muro (fin del muro interno, esquina).", 100.0, 2000.0),
        "seg_tolerancia_mm": _p("float", 45.0, "Tolerancia del ajuste de rectas al contorno del muro, en mm. Los tramos que se desvian menos que esto se consideran la misma recta.", 10.0, 200.0),
        "seg_gap_max_mm": _p("float", 350.0, "Hueco maximo entre dos rectas casi colineales que se fusionan (puentea los cortes por brillo en la pared).", 50.0, 1200.0),
        "seg_angulo_fusion_deg": _p("float", 12.0, "Diferencia angular maxima para fusionar dos rectas en una.", 2.0, 40.0),
        "tol_recta_deg": _p("float", 32.0, "Margen para decidir si una pared es LATERAL (paralela a la recta) o de FRENTE. Se aplica sobre el angulo ya corregido con el giroscopio, asi que puede ser generoso: subelo si en las curvas no reconoce ninguna pared, bajalo si confunde la de enfrente con la de al lado.", 10.0, 44.0),
        "recta_largo_min_mm": _p("float", 120.0, "Largo minimo (mm) para que un tramo cuente como pared con orientacion. Los trocitos cortos no dicen nada fiable y se marcan como 'otro'.", 30.0, 600.0),
    },

    "navegacion": {
        "estrategia": _p("str", "centrado", "'centrado': compara espacio libre izq/der y va por el medio (lo que mejor funciona). 'pared': sigue el muro interno a distancia fija.", opciones=["centrado", "pared"]),
        "kp": _p("float", 95.0, "Ganancia proporcional del centrado. Subela si corrige lento; bajala si serpentea.", 0.0, 400.0),
        "kd": _p("float", 22.0, "Ganancia derivativa del centrado. Subela si oscila al corregir.", 0.0, 200.0),
        "pared_objetivo_mm": _p("float", 320.0, "(estrategia 'pared') distancia objetivo al muro interno, en mm.", 100.0, 900.0),
        "kp_pared": _p("float", 0.22, "PD del seguimiento de pared: % de direccion por mm de error.", 0.0, 2.0),
        "kd_pared": _p("float", 0.05, "Derivativa del seguimiento de pared.", 0.0, 1.0),
        "girar_bajo_mm": _p("float", 650.0, "Pasillo por debajo de esto (mm) = hay esquina delante: entrar en giro.", 150.0, 2000.0),
        "salir_giro_mm": _p("float", 950.0, "(sin giroscopio) pasillo por encima de esto = el giro ya abrio la via: volver a recto.", 200.0, 3000.0),
        "frenar_bajo_mm": _p("float", 1000.0, "Empezar a bajar la velocidad cuando el pasillo baja de esto (mm).", 200.0, 3000.0),
        "parar_bajo_mm": _p("float", 300.0, "Pasillo por debajo de esto (mm) = muro encima: parar y escapar en reversa.", 80.0, 800.0),
        "ttc_min_s": _p("float", 0.7, "Segundos-hasta-el-muro minimos. Si el pasillo se cierra mas rapido que esto, frena aunque la distancia parezca comoda. Es el slider anti-inercia.", 0.1, 3.0),
        "dir_giro": _p("float", 85.0, "Cuanta direccion se mete en la esquina, en % (no siempre conviene el 100).", 20.0, 100.0),
        "retardo_giro_ms": _p("int", 220, "PRE-GIRO: espera frenando antes de volcar la direccion, para que las ruedas TRASERAS pasen la esquina interna antes de cortar.", 0, 1500),
        "apertura_pct": _p("float", 25.0, "Giro abierto (como los camiones): % de contra-direccion hacia el lado contrario durante el pre-giro para abrirse antes de cortar la esquina. 0 = desactivado.", 0.0, 80.0),
        "apertura_min_libre_mm": _p("float", 400.0, "Solo abrirse si el lado contrario tiene al menos este espacio (mm); si no, el remedio choca antes que la enfermedad.", 0.0, 1500.0),
        "giro_grados": _p("float", 90.0, "Grados de cada giro de esquina (la pista es cuadrada).", 30.0, 120.0),
        "giro_tolerancia_deg": _p("float", 8.0, "(con giroscopio) error de rumbo con el que se da el giro por terminado.", 2.0, 30.0),
        "giro_max_ms": _p("int", 3000, "Tiempo maximo dentro de un giro antes de rendirse y seguir.", 500, 8000),
        "min_recto_ms": _p("int", 700, "Tiempo minimo en recto antes de admitir OTRA esquina (evita encadenar giros sobre si mismo).", 0, 4000),
        "cobertura_esquina": _p("float", 0.22, "Si la banda del lado interno pasa de ver muro a ver menos que esta fraccion, se dispara la esquina (el muro interno desaparece en cada esquina).", 0.05, 0.8),
        "giro_por_linea": _p("bool", True, "Permitir que el cruce de linea del piso (TCS/camara) tambien dispare el giro cuando el pasillo ya se esta cerrando."),
        "linea_dispara_esquina": _p("bool", True, "MUY RECOMENDADO. Cruzar la primera linea del piso ENTRA en la esquina por si sola, sin esperar a que el pasillo se cierre. Las lineas marcan fisicamente donde esta la curva: es el disparo mas fiable que existe."),
        "bloqueo_esquina": _p("bool", True, "ANTI-BUCLE. Mientras el carro esta DENTRO de la esquina (entre las lineas del piso), el giro de 90 se ejecuta comprometido y la vision NO puede redirigirlo. Sin esto, el hueco blanco que deja el muro interno al terminar parece 'camino libre', el centrado se mete ahi, vuelve a ver otro hueco y el carro entra en bucle sin salir nunca de la curva."),
        "usar_rectas": _p("bool", True, "Usar las paredes ya IDENTIFICADAS (lateral interna / externa / de frente) en vez de la media de las bandas de la imagen. Con el giroscopio, esto es lo que evita que el carro confunda la pared de enfrente con la de su carril cuando llega torcido a la esquina esquivando un pilar."),
        "usar_yaw": _p("bool", True, "Usar el giroscopio: la camara decide CUANDO girar, el giroscopio decide CUANTO (90 grados clavados)."),
        "yaw_kp": _p("float", 1.6, "Correccion de rumbo en recta: % de direccion por grado de error.", 0.0, 10.0),
        "yaw_max": _p("float", 45.0, "Tope de la correccion por rumbo (que el giroscopio ayude, no que mande).", 0.0, 100.0),
    },

    "giro2t": {
        "activo": _p("bool", False, "SOLO se ejecuta en esquinas confirmadas por el PAR DE LINEAS del piso (TCS o camara): es la unica maniobra que retrocede y no debe retroceder nunca en mitad de una recta. Sin lineas detectadas se hace el giro normal, que solo va hacia adelante. GIRO DE DOS TIEMPOS en las esquinas: avanzar en diagonal y luego retroceder con la direccion invertida, logrando un 90 limpio en el sitio. Al terminar el carro queda ALINEADO con el tramo nuevo y ve el pasillo entero, asi no se le escapa ningun obstaculo. Mas lento pero mucho mas seguro para el reto con obstaculos. Necesita giroscopio."),
        "frac_avance": _p("float", 0.6, "Que fraccion de los 90 grados se hace en el tramo de AVANCE. El resto se completa en reversa. 0.6 = 54 grados adelante y 36 atras.", 0.2, 0.9),
        "vel_avance": _p("int", 32, "Velocidad del tramo de avance, en % de vmax. Despacio: el giro es cerrado.", 10, 100),
        "vel_reversa": _p("int", 30, "Velocidad del tramo de reversa, en % de vmax.", 10, 100),
        "dir_avance": _p("float", 100.0, "Direccion en el avance, en %. A tope para que el radio sea el minimo posible.", 40.0, 100.0),
        "dir_reversa": _p("float", 100.0, "Direccion en la reversa, en %. Va al lado CONTRARIO del avance: con Ackermann eso hace que el carro siga rotando en el mismo sentido mientras recupera sitio.", 40.0, 100.0),
        "min_pasillo_mm": _p("float", 340.0, "Si durante el avance el pasillo baja de esto, cortar ya y pasar a la reversa (sin esperar a completar la fraccion). Mantenlo POR ENCIMA de navegacion.parar_bajo_mm: asi la maniobra corta sola antes de que salte el escape.", 100.0, 900.0),
        "reversa_max_ms": _p("int", 1100, "Tope de cada tramo de reversa. Importante: el reglamento solo permite ir marcha atras dentro de la seccion; con esto no se cruza el limite hacia atras.", 200, 4000),
        "avance_max_ms": _p("int", 2000, "Tope de cada tramo de avance.", 200, 6000),
        "max_ciclos": _p("int", 3, "Cuantas veces puede repetir avance+reversa si aun le faltan grados.", 1, 6),
        "max_ms": _p("int", 7000, "Tiempo total maximo de la maniobra antes de rendirse y seguir de frente. Mantenlo POR DEBAJO de lineas.esquina_max_ms: si la zona de esquina caduca antes, la maniobra deja de tener permiso para retroceder y termina el giro hacia adelante.", 1000, 20000),
    },

    "escape": {
        "escape_min_ms": _p("int", 750, "Compromiso minimo de la marcha atras. Retrocesos cortos frente a un muro no ganan espacio: ir y venir cada 500 ms es como se choca.", 200, 3000),
        "escape_k_ms_por_mm": _p("float", 3.0, "ms extra de reversa por cada mm que falte de espacio (escala el compromiso segun el deficit).", 0.0, 20.0),
        "escape_dir": _p("float", 80.0, "Direccion durante la reversa, en %. Se gira HACIA el muro para que el morro se separe (como al salir de un estacionamiento).", 0.0, 100.0),
        "escape_max_intentos": _p("int", 4, "Reversas seguidas antes de rendirse y probar giro hacia adelante.", 1, 10),
    },

    "lineas": {
        "naranja_es_horario": _p("bool", True, "Convencion del tapete 2026: en sentido horario la PRIMERA linea que se cruza en cada esquina es la naranja. Si un tapete viniera al reves, apaga esto."),
        "usar_tcs": _p("bool", True, "Contar lineas con el TCS34725 del ESP32 (el metodo casi infalible)."),
        "usar_camara": _p("bool", True, "Contar/anticipar lineas tambien con la camara (redundancia y deteccion de sentido antes de cruzar)."),
        "umbral_cruce_mm": _p("float", 260.0, "(camara) una linea del piso a menos de esta distancia del morro cuenta como cruzada.", 50.0, 800.0),
        "ventana_par_ms": _p("int", 2500, "Las dos lineas de una misma esquina llegan dentro de esta ventana; lo que caiga dentro es LA MISMA esquina, no dos.", 500, 6000),
        "refractario_esquina_ms": _p("int", 3000, "Tras contar una esquina no se admite otra (venga del sensor que venga) durante este tiempo.", 500, 10000),
        "pares_para_invertir": _p("int", 2, "Cuantos pares de lineas seguidos en el orden CONTRARIO hacen falta para aceptar que el carro va de verdad al reves (y no que fue una lectura suelta). Con 2, un par raro se descarta sin contar; dos seguidos invierten el sentido.", 1, 5),
        "esquina_max_ms": _p("int", 8000, "Red de seguridad: tiempo maximo que el carro puede considerarse DENTRO de una esquina. Si lo supera vuelve a modo recta aunque el giro no se haya confirmado, para no quedarse bloqueado si el giroscopio falla.", 1000, 20000),
    },

    "tcs": {
        "c_min": _p("int", 80, "Canal claro minimo del TCS para clasificar (por debajo es sombra o sensor tapado).", 0, 65535),
        "naranja_dif_min": _p("int", 30, "DISCRIMINADOR del naranja: cuanto tiene que superar el ratio rojo al azul (r-b). Sobre el piso blanco esa diferencia es ~0 y sobre una linea es grande, asi que separa mucho mejor que un umbral absoluto y no depende de la luz. Es el que hay que tocar.", 0, 255),
        "azul_dif_min": _p("int", 18, "DISCRIMINADOR del azul: cuanto tiene que superar el ratio azul al rojo (b-r). Medido en la pista del equipo, la linea azul da b-r = +37 contra ~0 del blanco.", 0, 255),
        "naranja_r_min": _p("int", 110, "Reja de seguridad del naranja (ratio rojo minimo), no discriminador: dejalo holgado y decide con naranja_dif_min.", 0, 255),
        "naranja_b_max": _p("int", 90, "Reja de seguridad del naranja (ratio azul maximo).", 0, 255),
        "azul_b_min": _p("int", 95, "Reja de seguridad del azul (ratio azul minimo). OJO: con 110 la linea azul del equipo NO se detectaba, porque medía 107.", 0, 255),
        "azul_r_max": _p("int", 95, "Reja de seguridad del azul (ratio rojo maximo).", 0, 255),
        "muestras_min": _p("int", 1, "Lecturas seguidas iguales antes de confirmar el cruce.", 1, 5),
        "refractario_ds": _p("int", 3, "Decimas de segundo sin admitir otro cruce del MISMO color (que una linea no cuente doble).", 1, 30),
        "atime": _p("int", 246, "Registro ATIME del TCS: 255=2.4ms, 246=24ms, 235=50ms de integracion. Es lo que fija CUANTAS muestras se toman por linea: a 24 ms una linea de 2 cm cruzada a 0.5 m/s solo da 1 o 2 lecturas, y a 2.4 ms da unas 16. OJO: al bajarlo entra menos luz y TODOS los valores absolutos cambian, asi que hay que repetir la calibracion del TCS (y quiza subir la ganancia).", 0, 255),
        "int_umbral_pct": _p("int", 55, "Pata INT del TCS: salta cuando el claro cae por debajo de este % del nivel del PISO, que el ESP32 aprende solo. Al ser relativo no se estropea si cambias la integracion o la ganancia. Bajalo si la interrupcion salta con sombras; subelo si no engancha las lineas.", 5, 95),
        "gain": _p("int", 2, "Ganancia del TCS: 0=x1, 1=x4, 2=x16, 3=x60.", 0, 3),
    },

    "carrera": {
        "vueltas": _p("int", 3, "Vueltas a completar (el reglamento pide 3).", 1, 10),
        "esquinas_por_vuelta": _p("int", 4, "Esquinas que cierran una vuelta (la pista es cuadrada).", 3, 8),
        "sentido": _p("str", "auto", "Sentido de la ronda. 'auto': el carro lo deduce solo (lineas del piso / geometria). Forzarlo sirve para probar.", opciones=["auto", "horario", "antihorario"]),
        "parada_ms": _p("int", 1400, "Tras la ultima esquina, avanzar este tiempo para quedar BIEN DENTRO de la seccion de meta y detenerse (la proyeccion completa del carro debe quedar dentro).", 0, 6000),
        "autostop": _p("bool", True, "Detenerse solo al completar las vueltas. Apagalo para pruebas de resistencia."),
        "tiempo_max_s": _p("int", 180, "Duracion maxima de la ronda (el reglamento da 3 minutos).", 10, 600),
    },

    "obstaculos": {
        "activo": _p("bool", False, "Esquivar pilares rojos (pasar por su derecha) y verdes (por su izquierda). Version basica: enciendelo solo para probar el reto de obstaculos."),
        "activar_desde_mm": _p("float", 1600.0, "Distancia a la que un pilar empieza a influir en la direccion.", 300.0, 4000.0),
        "mandar_desde_mm": _p("float", 700.0, "Distancia a la que el pilar ya manda al maximo sobre la direccion.", 100.0, 2000.0),
        "margen_mm": _p("float", 70.0, "Holgura extra entre el costado del carro y el pilar al pasarlo.", 0.0, 300.0),
        "semi_pilar_mm": _p("float", 25.0, "Medio ancho del pilar (son de 50x50 mm).", 10.0, 60.0),
        "k_dir": _p("float", 1.4, "Ganancia de la correccion hacia el punto de paso: % de direccion por grado de desvio.", 0.1, 10.0),
        "peso_max": _p("float", 0.8, "Peso maximo del esquive frente al centrado (1 = el pilar manda del todo).", 0.0, 1.0),
        "limitar_por_lineas": _p("bool", True, "No hacer caso a los pilares que quedan MAS ALLA de la linea del piso: esos son de la seccion siguiente. Si se les hace caso desde la recta, el esquive pega el carro a la esquina interna justo antes de la curva y engancha el canto al girar. En cuanto se cruza la linea el filtro se levanta y esos pilares cuentan."),
        "margen_linea_mm": _p("float", 60.0, "Holgura sobre la distancia a la linea: un pilar justo antes de ella sigue contando. Subelo si descarta pilares que si son de esta recta.", 0.0, 400.0),
    },

    "manual": {
        "timeout_ms": _p("int", 400, "Si el joystick deja de refrescar durante esto, el carro se para solo (hombre muerto).", 100, 3000),
        "manual_seguro": _p("bool", False, "En manual, no dejar avanzar contra un muro a menos de parar_bajo_mm. Apagado por defecto: en un rescate tras choque suele estorbar."),
        "vel_max_manual": _p("int", 60, "Tope de velocidad del joystick, en % de vmax.", 10, 100),
    },

    "enlace": {
        "puerto": _p("str", "", "Puerto serie del ESP32 (vacio = autodeteccion probando todos)."),
        "baudios": _p("int", 115200, "Baudios del enlace.", 9600, 921600),
        "hz_envio": _p("int", 50, "Tramas de mando por segundo.", 10, 100),
        "timeout_tele_ms": _p("int", 500, "Sin telemetria durante esto = enlace caido, se reabre el puerto.", 100, 5000),
        "reintento_s": _p("float", 2.0, "Cada cuanto reintentar la conexion.", 0.5, 30.0),
    },

    "servo": {
        "centro": _p("int", 100, "Angulo del servo con las ruedas rectas. OJO: el firmware recorta contra sus topes de compilacion (50-145).", 50, 145),
        "izquierda": _p("int", 65, "Tope util a la izquierda (dir = -100).", 50, 145),
        "derecha": _p("int", 135, "Tope util a la derecha (dir = +100).", 50, 145),
        "grados_s": _p("int", 320, "Velocidad maxima de barrido del servo, grados/s (protege la cremallera).", 20, 2000),
        "rampa_pwm": _p("int", 10, "Cuentas de PWM que el motor puede cambiar por tick de 10 ms (rampa de aceleracion).", 1, 255),
    },

    "red": {
        "puerto_http": _p("int", 8080, "Puerto del servidor web.", 1024, 65535),
        "calidad_jpeg": _p("int", 70, "Calidad JPEG del stream (mas = mas nitido y mas ancho de banda).", 20, 95),
        "fps_stream": _p("int", 15, "FPS del stream MJPEG.", 2, 30),
        "ancho_stream": _p("int", 640, "Ancho al que se reescala el stream.", 320, 1280),
    },

    "velocidad": {
        "vel_max_mm_s": _p("float", 850.0, "Velocidad real del carro en mm/s con PWM a 255 (motor 500rpm, diferencial 2:1, rueda 65.2mm => ~850). Se usa para estimar cuanto avanza en la parada final.", 100.0, 3000.0),
    },
}


# ===========================================================================
# Valores: {grupo: {clave: valor}}
# ===========================================================================
def valores_por_defecto() -> Dict[str, Dict[str, Any]]:
    return {g: {k: copy.deepcopy(e["def"]) for k, e in claves.items()}
            for g, claves in ESQUEMA.items()}


def validar(grupo: str, clave: str, valor: Any) -> Any:
    """Devuelve el valor convertido y recortado segun el esquema, o lanza."""
    e = ESQUEMA[grupo][clave]
    t = e["tipo"]
    if t == "bool":
        if isinstance(valor, str):
            return valor.strip().lower() not in ("0", "false", "no", "")
        return bool(valor)
    if t == "int":
        v = int(float(valor))
    elif t == "float":
        v = float(valor)
    else:  # str
        v = str(valor)
        if "opciones" in e and v not in e["opciones"]:
            raise ValueError(f"'{v}' no esta en {e['opciones']}")
        return v
    # El centinela de "automatico" vive FUERA del rango util a proposito
    # (-1 no es una temperatura de color ni un indice de camara), asi que se
    # deja pasar tal cual en vez de recortarlo contra el minimo.
    if "auto" in e and v == type(v)(e["auto"]):
        return v
    if "min" in e:
        v = max(e["min"], v)
    if "max" in e:
        v = min(e["max"], v)
    return v


def normalizar(valores: Any) -> Dict[str, Dict[str, Any]]:
    """Completa claves faltantes y valida todo. Nunca lanza."""
    base = valores_por_defecto()
    if isinstance(valores, dict):
        for g, claves in valores.items():
            if g not in base or not isinstance(claves, dict):
                continue
            for k, v in claves.items():
                if k not in base[g]:
                    continue
                try:
                    base[g][k] = validar(g, k, v)
                except Exception:
                    pass
    return base


# ===========================================================================
# Perfiles rotativos (igual que los de color)
# ===========================================================================
def _ahora() -> str:
    return _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def datos_por_defecto() -> Dict[str, Any]:
    p = {"nombre": "base", "fecha": _ahora(), "valores": valores_por_defecto()}
    return {"version": 1, "activo": "base", "perfiles": [p]}


def cargar(ruta: Optional[Path] = None) -> Dict[str, Any]:
    ruta = Path(ruta) if ruta else RUTA_PARAMS
    if not ruta.exists():
        datos = datos_por_defecto()
        guardar_archivo(datos, ruta)
        return datos
    try:
        with open(ruta, "r", encoding="utf-8") as f:
            datos = json.load(f)
        if not isinstance(datos, dict):
            raise ValueError("raiz invalida")
    except Exception as e:
        respaldo = ruta.with_suffix(ruta.suffix + ".bak")
        try:
            os.replace(ruta, respaldo)
            print(f"[params] {ruta} ilegible ({e}); respaldado en {respaldo}")
        except Exception:
            pass
        datos = datos_por_defecto()
        guardar_archivo(datos, ruta)
        return datos

    perfiles = datos.get("perfiles")
    if not isinstance(perfiles, list) or not perfiles:
        perfiles = [datos_por_defecto()["perfiles"][0]]
    limpios = []
    for i, p in enumerate(perfiles[:MAX_PERFILES]):
        if not isinstance(p, dict):
            continue
        limpios.append({
            "nombre": str(p.get("nombre") or f"perfil_{i}"),
            "fecha": str(p.get("fecha") or _ahora()),
            "valores": normalizar(p.get("valores")),
        })
    if not limpios:
        limpios = [datos_por_defecto()["perfiles"][0]]
    activo = datos.get("activo")
    nombres = [p["nombre"] for p in limpios]
    if activo not in nombres:
        activo = nombres[0]
    return {"version": 1, "activo": activo, "perfiles": limpios}


def guardar_archivo(datos: Dict[str, Any], ruta: Optional[Path] = None) -> Path:
    ruta = Path(ruta) if ruta else RUTA_PARAMS
    ruta.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(ruta.parent), prefix=".params_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(datos, f, indent=1, ensure_ascii=False)
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


def obtener(datos: Dict[str, Any], nombre: Optional[str] = None) -> Dict[str, Any]:
    perfiles = datos.get("perfiles", [])
    if nombre is None:
        nombre = datos.get("activo")
    for p in perfiles:
        if p["nombre"] == nombre:
            return p
    return perfiles[0]


def guardar_perfil(datos: Dict[str, Any], nombre: str,
                   valores: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Mete el perfil de primero; si el nombre existe lo reemplaza en el sitio."""
    nombre = (nombre or "").strip() or _dt.datetime.now().strftime("params_%m%d_%H%M")
    nuevo = {"nombre": nombre, "fecha": _ahora(), "valores": normalizar(valores)}
    perfiles = [p for p in datos.get("perfiles", []) if p["nombre"] != nombre]
    perfiles.insert(0, nuevo)
    datos["perfiles"] = perfiles[:MAX_PERFILES]
    datos["activo"] = nombre
    return datos


def borrar_perfil(datos: Dict[str, Any], nombre: str) -> Dict[str, Any]:
    perfiles = [p for p in datos.get("perfiles", []) if p["nombre"] != nombre]
    if not perfiles:
        perfiles = [datos_por_defecto()["perfiles"][0]]
    datos["perfiles"] = perfiles
    if datos.get("activo") not in [p["nombre"] for p in perfiles]:
        datos["activo"] = perfiles[0]["nombre"]
    return datos


def esquema_para_web() -> Dict[str, Any]:
    """El esquema tal cual, para que la web arme la interfaz sola."""
    return ESQUEMA
