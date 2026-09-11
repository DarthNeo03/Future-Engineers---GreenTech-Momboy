"""
params.py — Todos los parametros del robot, con esquema autodocumentado.

Cada parametro declara tipo, limites y UNA descripcion en castellano. La web
construye sus sliders/campos leyendo este esquema, asi que agregar un
parametro aqui lo hace aparecer solo en la interfaz, ya validado.

Igual que los colores, los parametros se guardan en PERFILES rotativos (los 20
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
MAX_PERFILES = 20


def _p(tipo: str, defecto, desc: str, minimo=None, maximo=None,
       opciones: Optional[List[str]] = None, paso=None) -> Dict[str, Any]:
    d: Dict[str, Any] = {"tipo": tipo, "def": defecto, "desc": desc}
    if minimo is not None:
        d["min"] = minimo
    if maximo is not None:
        d["max"] = maximo
    if opciones:
        d["opciones"] = opciones
    if paso is not None:
        d["paso"] = paso
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
        "exposicion": _p("float", -1.0, "Exposicion manual (-1 = automatica). CONGELALA antes de calibrar colores: en automatico el HSV cambia solo al girar hacia una pared clara.", -14.0, 1000.0),
        "balance_blancos": _p("float", -1.0, "Temperatura del balance de blancos manual (-1 = automatico).", -1.0, 10000.0),
    },

    "geometria": {
        "alto_cam_mm": _p("float", 125.0, "Altura del centro de la lente sobre el suelo, en mm. Midela con regla.", 50.0, 400.0),
        "inclinacion_deg": _p("float", 7.5, "Inclinacion de la camara hacia abajo desde la horizontal, en grados.", 0.0, 45.0),
        "fy_px": _p("float", 460.0, "Focal vertical en pixeles (referida a 480 de alto). Se calibra con un clic sobre un objeto a distancia conocida en la pestaña Calibracion.", 100.0, 2000.0),
        "fx_px": _p("float", 460.0, "Focal horizontal en pixeles (referida a 640 de ancho). Se calibra con un clic sobre un objeto desplazado a un lado.", 100.0, 2000.0),
        "ancho_carro_mm": _p("float", 200.0, "Ancho total del carro con ruedas, en mm. Define el corredor dibujado por donde van a pasar las ruedas.", 100.0, 300.0),
        "margen_ruedas_mm": _p("float", 30.0, "Margen de seguridad extra a cada lado del carro para el corredor, en mm.", 0.0, 150.0),
        "largo_carro_mm": _p("float", 250.0, "Largo total del carro, en mm. Se usa para saber cuanto hay que seguir recto despues de adelantar un pilar hasta que la RUEDA TRASERA lo haya pasado: con direccion Ackermann la cola corta por dentro y es lo que barre el pilar.", 100.0, 400.0),
        "morro_mm": _p("float", 60.0, "Distancia de la lente al frente del carro, en mm. Las distancias mostradas se miden desde el morro.", 0.0, 300.0),
    },

    "muro": {
        "alcance_mm": _p("float", 2500.0, "Distancia a partir de la cual una columna se considera despejada (define el 1.0 del espacio libre normalizado).", 500.0, 6000.0),
        "k_transicion": _p("int", 6, "Filas seguidas de no-piso para dar por buena la transicion piso->muro. Subelo si sombras o rayas del tapete crean muros fantasma; bajalo si el muro cercano se detecta tarde.", 2, 30),
        "margen_horizonte_px": _p("int", 4, "Pixeles de margen bajo la fila del horizonte antes de empezar a buscar muro: lo de arriba no se mapea, porque por geometria ahi no puede haber pista (es publico, mesas o techo). OJO: esto recorta solo la BUSQUEDA DEL MURO. La medida de cuanto piso se ve por cada mitad (piso_izq/piso_der), que es la que dice por donde se sale de un rincon, se hace sobre toda la franja y este recorte no la toca.", 0, 60),
        "ignorar_abajo": _p("float", 0.05, "Fraccion inferior de la imagen tapada por el chasis del carro, que se ignora.", 0.0, 0.4),
        "suavizado": _p("int", 7, "Promedio movil del perfil, en columnas. Alisa el ruido sin comerse las esquinas.", 0, 41),
        "banda_lateral": _p("float", 0.28, "Ancho (fraccion de imagen) de las bandas izquierda/derecha con las que se compara espacio libre.", 0.1, 0.45),
        "salto_borde_mm": _p("float", 400.0, "Discontinuidad de distancia entre columnas vecinas que cuenta como borde de muro (fin del muro interno, esquina).", 100.0, 2000.0),
        "seg_tolerancia_mm": _p("float", 45.0, "Tolerancia del ajuste de rectas al contorno del muro, en mm. Los tramos que se desvian menos que esto se consideran la misma recta.", 10.0, 200.0),
        "seg_gap_max_mm": _p("float", 350.0, "Hueco maximo entre dos rectas casi colineales que se fusionan (puentea los cortes por brillo en la pared).", 50.0, 1200.0),
        "seg_angulo_fusion_deg": _p("float", 12.0, "Diferencia angular maxima para fusionar dos rectas en una.", 2.0, 40.0),
    },

    "navegacion": {
        "kp": _p("float", 95.0, "Ganancia proporcional del centrado. Subela si corrige lento; bajala si serpentea.", 0.0, 400.0),
        "kd": _p("float", 22.0, "Ganancia derivativa del centrado. Subela si oscila al corregir.", 0.0, 200.0),
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
        "usar_yaw": _p("bool", True, "Usar el giroscopio: la camara decide CUANDO girar, el giroscopio decide CUANTO (90 grados clavados)."),
        "yaw_kp": _p("float", 1.6, "Correccion de rumbo en recta: % de direccion por grado de error.", 0.0, 10.0),
        "yaw_max": _p("float", 45.0, "Tope de la correccion por rumbo (que el giroscopio ayude, no que mande).", 0.0, 100.0),
        "yaw_cede_al_esquivar": _p("bool", True, "Desvanecer la correccion de rumbo mientras manda un pilar. La correccion se SUMA despues de mezclar el esquive, y al esquivar el carro se sale del rumbo a proposito: sin esto, o se suma (55 % de esquive + 45 % de yaw = volante al tope, y la cola se lleva el pilar) o se resta (y no esquiva bastante)."),
        "giro_cede_ante_pilar": _p("bool", True, "Soltar el giro de 90 de la esquina en cuanto aparece un pilar. Un giro comprometido con un pilar delante lo atropella o lo pasa por el lado prohibido, y eso termina la ronda; perder la esquina solo cuesta que la cuente el TCS un poco mas tarde."),
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
        "refractario_esquina_ms": _p("int", 3000, "Tras contar una esquina no se admite otra (venga del sensor que venga) durante este tiempo.", 500, 10000),
        "cierre_max_ms": _p("int", 6000, "Cuanto se espera la SEGUNDA linea de una esquina ya abierta. Mientras dure, esa linea CIERRA la esquina en vez de abrir otra: es lo que impide que una azul que llega tarde sume una esquina de mentira y el carro se crea una vuelta por delante. Debe cubrir toda la curva; solo bajalo si dos esquinas de verdad quedan muy seguidas.", 1000, 15000),
    },

    "tcs": {
        "c_min": _p("int", 40, "Canal claro minimo del TCS para clasificar (por debajo es sombra o sensor tapado). CUIDADO: una linea de color absorbe luz y devuelve MUCHO menos claro que el piso blanco; con un c_min sacado del 25 % del blanco, la linea azul se descartaba antes de mirar su color y el TCS no la vio nunca. Ademas escala con atime: al pasar de 24 a 12 ms todas las cuentas se parten por dos y este umbral tambien.", 0, 65535),
        "naranja_dif_min": _p("int", 30, "DISCRIMINADOR del naranja: cuanto tiene que superar el ratio rojo al azul (r-b). Sobre el piso blanco esa diferencia es ~0 y sobre una linea es grande, asi que separa mucho mejor que un umbral absoluto Y NO CAMBIA con el tiempo de integracion, porque es un cociente. Es el que hay que tocar.", 0, 255),
        "azul_dif_min": _p("int", 18, "DISCRIMINADOR del azul: cuanto tiene que superar el ratio azul al rojo (b-r). Medido en la pista del equipo, la linea azul da b-r = +37 contra ~0 del blanco.", 0, 255),
        "naranja_r_min": _p("int", 110, "Reja de seguridad del naranja (ratio rojo minimo), no discriminador: dejalo holgado y decide con naranja_dif_min.", 0, 255),
        "naranja_b_max": _p("int", 90, "Reja de seguridad del naranja (ratio azul maximo).", 0, 255),
        "azul_b_min": _p("int", 95, "Reja de seguridad del azul (ratio azul minimo). OJO: con 110 la linea azul del equipo NO se detectaba, porque medía 107.", 0, 255),
        "azul_r_max": _p("int", 95, "Reja de seguridad del azul (ratio rojo maximo).", 0, 255),
        "muestras_min": _p("int", 1, "Lecturas seguidas iguales antes de confirmar el cruce.", 1, 5),
        "refractario_ds": _p("int", 3, "Decimas de segundo sin admitir otro cruce del MISMO color (que una linea no cuente doble).", 1, 30),
        "atime": _p("int", 251, "Registro ATIME del TCS: 255=2.4ms, 251=12ms, 246=24ms, 235=50ms de integracion. Es lo que fija CUANTAS muestras se toman por linea: a 12 ms una linea de 2 cm cruzada a 0.5 m/s da 3 lecturas, y a 24 ms solo 1 o 2. El ESP32 sondea a 200 Hz y coge la muestra en cuanto el chip la da por buena, asi que el periodo real es este y no el doble. OJO: al bajarlo entra menos luz y TODOS los valores absolutos se parten (c_min); los ratios y las diferencias NO cambian, que por eso son los discriminadores.", 0, 255),
        "int_umbral_pct": _p("int", 55, "Pata INT del TCS: salta cuando el claro cae por debajo de este % del nivel del PISO, que el ESP32 aprende solo. Al ser relativo no se estropea si cambias la integracion o la ganancia. Bajalo si la interrupcion salta con sombras; subelo si no engancha las lineas.", 5, 95),
        "gain": _p("int", 2, "Ganancia del TCS: 0=x1, 1=x4, 2=x16, 3=x60. Dejala en x16 al bajar atime: el claro satura en 1024*(256-atime), asi que subirla a x60 con 12 ms quema el piso blanco y los ratios dejan de valer.", 0, 3),
    },

    "carrera": {
        "vueltas": _p("int", 3, "Vueltas a completar (el reglamento pide 3).", 1, 10),
        "esquinas_por_vuelta": _p("int", 4, "Esquinas que cierran una vuelta (la pista es cuadrada).", 3, 8),
        "sentido": _p("str", "auto", "Sentido de la ronda. 'auto': el carro lo deduce solo (lineas del piso / geometria). Forzarlo sirve para probar.", opciones=["auto", "horario", "antihorario"]),
        "parada_ms": _p("int", 1400, "Tras la ultima esquina, avanzar este tiempo para quedar BIEN DENTRO de la seccion de meta y detenerse (la proyeccion completa del carro debe quedar dentro).", 0, 6000),
        "autostop": _p("bool", True, "Detenerse solo al completar las vueltas. Apagalo para pruebas de resistencia."),
        "tiempo_max_s": _p("int", 180, "Duracion maxima de la ronda (el reglamento da 3 minutos).", 10, 600),
    },

    # =======================================================================
    # ESQUIVAR PILARES. La regla es una sola y no se invierte con el sentido:
    # el pilar ROJO se pasa por SU DERECHA y el VERDE por SU IZQUIERDA, con la
    # derecha y la izquierda DEL VEHICULO. Ver src/obstaculos.py.
    # =======================================================================
    "obstaculos": {
        "activo": _p("bool", True, "Esquivar los pilares de colores. Apagalo para probar solo la conduccion, como en el Open Challenge."),
        "activar_desde_mm": _p("float", 1600.0, "Distancia a la que un pilar empieza a influir en la direccion.", 300.0, 4000.0),
        "mandar_desde_mm": _p("float", 700.0, "Distancia a la que el pilar ya manda al maximo sobre la direccion.", 100.0, 2000.0),
        "lateral_max_mm": _p("float", 900.0, "Un pilar desplazado mas que esto hacia un lado no es de este carril: se ignora.", 200.0, 2000.0),
        "margen_mm": _p("float", 70.0, "Holgura que se PIDE entre el costado del carro y el pilar al pasarlo. Si no cabe se aprieta hasta rozar, pero NUNCA se cruza al otro lado del pilar.", 0.0, 300.0),
        "semi_pilar_mm": _p("float", 25.0, "Medio ancho del pilar (son de 50x50 mm).", 10.0, 60.0),
        "k_dir": _p("float", 1.4, "Ganancia hacia el punto de paso: % de direccion por grado de desvio.", 0.1, 10.0),
        "dir_max_pct": _p("float", 55.0, "TOPE de direccion que puede pedir el esquive. El pilar nunca deberia mandar el volante a fondo: eso es lo que hace que el carro gire de golpe y se cruce. Si necesitas mas, acercate mas tarde (mandar_desde_mm) en vez de subir esto.", 10.0, 100.0),
        "mirada_min_mm": _p("float", 350.0, "Mirada minima al calcular el angulo hacia el punto de paso. Es lo que impide el volantazo al acercarse: sin ella, el mismo desvio lateral pide cada vez mas angulo hasta llegar al tope. Subela si el carro esquiva demasiado brusco.", 150.0, 1200.0),
        "rampa_dir_pct_s": _p("float", 220.0, "Cuanto puede cambiar la direccion del esquive por segundo. Evita el volantazo en un solo frame.", 40.0, 1000.0),
        "peso_max": _p("float", 0.8, "Peso maximo del esquive frente al centrado. Con 1 el muro deja de contar: no lo pongas en 1.", 0.0, 1.0),
        "ceder_ante_muro": _p("bool", True, "Cuando el pasillo se cierra, el esquive cede el mando a la evitacion de muros. Sin esto, un pilar pegado a la pared interior puede llevarse el carro de frente contra la esquina."),
        "ceder_bajo_mm": _p("float", 700.0, "Pasillo por debajo del cual el esquive empieza a ceder ante el muro.", 200.0, 2000.0),
        "compromiso_bajo_mm": _p("float", 500.0, "A partir de esta cercania se arma el COMPROMISO de adelantamiento: el pilar esta a punto de salir del cuadro (la camara no llega tan abajo) y si el esquive desapareciera de golpe, el centrado tiraria del carro al medio y la rueda trasera barreria el pilar.", 150.0, 1200.0),
        "peso_compromiso": _p("float", 0.7, "Cuanto manda ese 'ir recto' mientras se adelanta un pilar que ya no se ve.", 0.0, 1.0),
        "compromiso_max_ms": _p("int", 2500, "Tope del compromiso. Es una red: si la velocidad estimada fuera absurda, el carro no se queda yendo recto para siempre.", 300, 8000),
    },

    # =======================================================================
    # RESCATE DE ESQUINA: el triangulo. Ver muro.DetectorAtrapado.
    # Si esto salta en curvas buenas, SUBE confirmar_ms antes de tocar nada mas.
    # =======================================================================
    "rescate": {
        "activo": _p("bool", True, "Rescatar al carro cuando la esquina se cierra y el piso queda en TRIANGULO. Ahi el escape en reversa no sirve: retrocede, abre un palmo de pasillo y devuelve el mando al centrado, que se vuelve a meter en el mismo hueco."),
        "grados": _p("float", 100.0, "Grados del giro de rescate, hacia adentro. Un poco mas de 90 para salir apuntando a la recta, pero sin pasarse: un giro exagerado deja el carro mirando a la pared de enfrente.", 45.0, 180.0),
        "confirmar_ms": _p("int", 1200, "Cuanto tiene que verse el patron del triangulo, seguido y sin que el vertice se aleje, antes de dar el rescate por necesario. ES EL PARAMETRO: si el rescate salta en curvas buenas, subelo.", 200, 5000),
        "reversa_ms": _p("int", 400, "Reversa corta con el volante al reves antes de girar, para separar el morro de la pared. Con el morro pegado, girar no mueve el carro: lo raspa.", 0, 1500),
        "vel_pct": _p("int", 35, "Velocidad durante el rescate, en % de vmax.", 10, 80),
        "dir_pct": _p("float", 90.0, "(sin giroscopio) direccion durante el giro de rescate, en %.", 30.0, 100.0),
        "max_ms": _p("int", 3000, "Tope del rescate. Pasado esto se vuelve a conducir normal, salga bien o mal.", 500, 8000),
        "dist_max_mm": _p("float", 800.0, "Para estar atrapado, incluso lo MAS LEJOS que se ve (percentil 80) tiene que estar a menos de esto. Si queda algo lejos es que hay salida.", 200.0, 2000.0),
        "cobertura_min": _p("float", 0.90, "Fraccion minima de columnas que tienen que ver muro. Un hueco sin muro es una salida.", 0.5, 1.0),
        "relieve_min_pct": _p("float", 20.0, "Relieve minimo del perfil, en PORCENTAJE, para creer que es una esquina y no una pared plana de frente (que da 0 % y se resuelve con el escape normal). En porcentaje y no en mm porque la misma esquina de 90 grados deja el mismo relieve relativo desde 40 o desde 70 cm, pero en mm el numero encoge segun te acercas y el umbral se caeria justo cuando mas atrapado estas.", 5.0, 60.0),
        "piso_max_pct": _p("float", 75.0, "Piso visible maximo, en % del cuadro util. Con mas que esto todavia hay sitio.", 20.0, 95.0),
        "borde_frac": _p("float", 0.25, "Ancho de las bandas de los cantos con las que se compara el centro para medir el relieve.", 0.1, 0.4),
        "avance_max_mm": _p("float", 250.0, "Si el vertice de la esquina se ALEJA mas que esto, el carro si esta saliendo: la cuenta se reinicia y no hay rescate.", 50.0, 1000.0),
    },

    "botones": {
        "activo": _p("bool", True, "Hacer caso al pulsador de competencia que cuelga del ESP32. Apagalo solo para depurar: las pulsaciones VIRTUALES de la web siguen funcionando igual."),
        "antirrebote_ms": _p("int", 50, "Tiempo que el nivel tiene que aguantar quieto antes de creerselo. Se suma al antirrebote del ESP32 (30 ms); subelo si el pulsador viene ruidoso, sin recompilar el firmware.", 0, 400),
        "largo_ms": _p("int", 3000, "A partir de cuanto una pulsacion es LARGA. Solo hace algo si apagar_con_larga esta encendido: por eso es tan generosa.", 500, 8000),
        "repeticion_ms": _p("int", 800, "Tiempo muerto tras atender una pulsacion. Un doble toque involuntario justo despues del start desarmaria la ronda recien empezada.", 0, 5000),
        "hz": _p("int", 50, "Veces por segundo que la Pi mira el estado del pulsador. Las tramas de sensores llegan a 40 Hz, asi que mas de 50 no aporta nada.", 20, 200),
        "apagar_con_larga": _p("bool", False, "Dejar que la pulsacion LARGA apague la Pi. Necesita sudo sin contraseña para 'systemctl poweroff'."),
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
