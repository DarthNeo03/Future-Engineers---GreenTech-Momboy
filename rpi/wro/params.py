# -*- coding: utf-8 -*-
"""
Registro central de parametros calibrables.

Cada parametro se declara una sola vez aqui, con su valor por defecto, sus
limites y un texto de ayuda. De esa declaracion salen automaticamente:

  * el fichero config.json que se guarda en disco,
  * el panel de calibracion del servidor web (con la explicacion de cada campo),
  * los parametros que se empujan al ESP32 (los que llevan target="esp32").

Sistema de coordenadas del robot (usado en todo el proyecto):

      X (+) hacia adelante  [mm]
      Y (+) hacia la IZQUIERDA [mm]
      angulo / giro (+) hacia la IZQUIERDA [grados]

El origen esta en el punto de referencia del robot (aprox. el centro del eje
trasero); la camara esta adelantada `cam_offset_x_mm` respecto a ese origen.
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Spec:
    key: str
    default: Any
    group: str
    label: str
    help: str
    kind: str = "float"          # float | int | bool | choice | text
    lo: Optional[float] = None
    hi: Optional[float] = None
    step: Optional[float] = None
    choices: Optional[List[str]] = None
    target: str = "pi"           # pi | esp32
    advanced: bool = False


def F(key, default, group, label, help, lo=None, hi=None, step=None, **kw):
    return Spec(key, float(default), group, label, help, "float", lo, hi, step, **kw)


def I(key, default, group, label, help, lo=None, hi=None, step=1, **kw):
    return Spec(key, int(default), group, label, help, "int", lo, hi, step, **kw)


def B(key, default, group, label, help, **kw):
    return Spec(key, bool(default), group, label, help, "bool", **kw)


def C(key, default, group, label, help, choices, **kw):
    return Spec(key, default, group, label, help, "choice", choices=choices, **kw)


def T(key, default, group, label, help, **kw):
    return Spec(key, default, group, label, help, "text", **kw)


# ===========================================================================
#  G1 - Camara
# ===========================================================================
G_CAM = "1. Camara"
G_IPM = "2. Geometria / vista de pajaro"
G_SEG = "3. Deteccion de muros (imagen)"
G_WAL = "4. Geometria de muros (suelo)"
G_DRV = "5. Conduccion"
G_TRN = "6. Giros y vueltas"
G_REC = "7. Recuperacion / seguridad"
G_OBS = "8. Obstaculos (pilares)"
G_ESP = "9. Hardware (ESP32)"
G_SYS = "10. Sistema"


PARAMS: List[Spec] = [
    # ----------------------------------------------------------------- camara
    I("cam_index", 0, G_CAM, "Indice de camara",
      "Numero de /dev/videoN. Si tienes varias camaras UVC prueba 0, 2, 4...",
      lo=0, hi=20),
    I("cam_width", 640, G_CAM, "Ancho de captura (px)",
      "Resolucion horizontal. 640 es el mejor compromiso: mas resolucion no "
      "mejora la deteccion del muro (la base del muro ya se ve bien) y si "
      "baja los FPS.", lo=160, hi=1920, step=32),
    I("cam_height", 480, G_CAM, "Alto de captura (px)",
      "Resolucion vertical. Manten la relacion de aspecto nativa de la camara "
      "(640x480 o 1280x720) para no cambiar el campo de vision.",
      lo=120, hi=1080, step=24),
    I("cam_fps", 60, G_CAM, "FPS solicitados",
      "FPS que se le piden al driver. La IMX179 en MJPG suele dar 30-60 a "
      "640x480. Si la camara no lo soporta, cae al valor mas cercano.",
      lo=5, hi=120),
    B("cam_mjpg", True, G_CAM, "Usar MJPG",
      "Pide el formato MJPG al driver. Casi siempre necesario para pasar de "
      "10 FPS por USB 2.0. Si la imagen sale corrupta, desactivalo (YUYV)."),
    B("cam_auto_exposure", False, G_CAM, "Exposicion automatica",
      "DESACTIVALO para competir. Con auto-exposicion el brillo del tapete "
      "cambia solo y el umbral de muro deja de valer. Con exposicion fija la "
      "segmentacion es estable."),
    I("cam_exposure", 120, G_CAM, "Exposicion manual",
      "Valor V4L2 de exposicion (solo si la automatica esta apagada). Subelo "
      "si la imagen sale oscura, bajalo si el tapete blanco se satura. Ajusta "
      "mirando la vista 'mascara': el tapete debe quedar todo negro y los "
      "muros todo blanco.", lo=1, hi=2000),
    I("cam_gain", 20, G_CAM, "Ganancia",
      "Ganancia analogica del sensor. Prefiere subir exposicion antes que "
      "ganancia: la ganancia mete ruido y ensucia la mascara.", lo=0, hi=255),
    B("cam_flip", False, G_CAM, "Rotar 180 grados",
      "Actívalo si montaste la camara boca abajo."),

    # -------------------------------------------------------------- geometria
    F("cam_height_mm", 125.0, G_IPM, "Altura de la camara (mm)",
      "Altura del centro optico sobre el tapete. Medida real: 125 mm. Es el "
      "factor de escala de toda la proyeccion: si esta mal, TODAS las "
      "distancias salen mal en la misma proporcion.", lo=40, hi=300, step=0.5),
    F("cam_pitch_deg", 15.0, G_IPM, "Inclinacion hacia abajo (grados)",
      "Cuanto mira la camara hacia el suelo. ES EL PARAMETRO MAS CRITICO. "
      "Si esta mal, las paredes rectas se ven curvadas en la vista de pajaro "
      "y las distancias se van con el rango. Usa el boton 'Calibrar pitch' "
      "(pon el robot mirando de frente a un muro, mide la distancia real de "
      "la camara a la base del muro y escribela).", lo=-10, hi=60, step=0.1),
    F("cam_roll_deg", 0.0, G_IPM, "Alabeo / roll (grados)",
      "Giro de la camara sobre su eje optico. Si el horizonte sale torcido en "
      "la imagen, corrige aqui. Positivo = la imagen gira a la izquierda.",
      lo=-15, hi=15, step=0.1),
    F("cam_hfov_deg", 78.0, G_IPM, "Campo de vision horizontal (grados)",
      "Angulo horizontal que abarca la camara. Determina la escala lateral "
      "(Y). Calibralo asi: pon el robot centrado en un pasillo de ancho "
      "conocido y ajusta hasta que 'ancho pasillo' coincida con la cinta "
      "metrica. Un objetivo de 90-120 grados ayuda mucho en esta prueba.",
      lo=40, hi=160, step=0.5),
    F("cam_cx_off", 0.0, G_IPM, "Offset del centro optico X (px)",
      "Desplazamiento del punto principal respecto al centro de la imagen. "
      "Dejalo en 0 salvo que veas un sesgo constante izquierda/derecha en las "
      "distancias con el robot bien centrado.", lo=-200, hi=200, step=1),
    F("cam_cy_off", 0.0, G_IPM, "Offset del centro optico Y (px)",
      "Igual que el anterior pero vertical. Mueve el horizonte; normalmente es "
      "mas comodo tocar la inclinacion.", lo=-200, hi=200, step=1,
      advanced=True),
    F("lens_k1", 0.0, G_IPM, "Distorsion radial k1",
      "Correccion de barril del objetivo. Negativo corrige barril (lo tipico "
      "en gran angular). Si en la vista de pajaro un muro recto sale curvado "
      "aunque el pitch este bien, prueba entre -0.35 y -0.05.",
      lo=-0.6, hi=0.6, step=0.005),
    F("lens_k2", 0.0, G_IPM, "Distorsion radial k2",
      "Termino de segundo orden. Casi siempre 0 basta.",
      lo=-0.4, hi=0.4, step=0.005, advanced=True),
    F("cam_offset_x_mm", 60.0, G_IPM, "Camara por delante del origen (mm)",
      "Distancia horizontal desde el punto de referencia del robot (centro del "
      "eje trasero) hasta la camara. Se resta para que X=0 sea el robot y no "
      "la camara.", lo=-200, hi=300, step=1),
    F("cam_offset_y_mm", 0.0, G_IPM, "Camara desplazada a la izquierda (mm)",
      "Si la camara no esta en el eje central del robot, ponlo aqui "
      "(positivo = a la izquierda).", lo=-120, hi=120, step=1),
    F("wheelbase_mm", 150.0, G_IPM, "Distancia entre ejes (mm)",
      "Batalla del vehiculo. Se usa en la conversion de curvatura a angulo de "
      "direccion.", lo=60, hi=280, step=1),

    # -------------------------------------------------- segmentacion en imagen
    I("wall_v_max", 85, G_SEG, "Umbral de oscuridad (V max)",
      "Un pixel se considera MURO si su brillo (canal V de HSV) es menor que "
      "este valor. Los muros son negros (V ~ 20-60) y el tapete blanco "
      "(V ~ 180-250). Subelo si el muro se te escapa en zonas oscuras; bajalo "
      "si el tapete en sombra se toma por muro.", lo=10, hi=200),
    I("wall_s_max", 120, G_SEG, "Saturacion maxima (S max)",
      "Un pixel muy saturado NO es muro. Sirve para que las lineas naranja y "
      "azul del tapete y los pilares rojo/verde no se confundan con el muro "
      "negro, que es acromatico. Bajalo si algun color se cuela en la mascara.",
      lo=20, hi=255),
    B("wall_auto_thresh", False, G_SEG, "Umbral automatico (Otsu)",
      "Calcula el umbral de oscuridad solo, dentro de la region de interes. "
      "Util si la luz de la sede es muy distinta a la de tus pruebas, pero es "
      "menos predecible. Recomendado: apagado + exposicion fija."),
    I("wall_min_run_px", 6, G_SEG, "Pixeles oscuros consecutivos",
      "Para aceptar la base de un muro se exigen N pixeles oscuros seguidos "
      "hacia arriba. Filtra sombras finas, juntas del tapete y ruido. Subelo "
      "si aparecen detecciones sueltas; bajalo si pierdes muros lejanos "
      "(que ocupan pocos pixeles de alto).", lo=1, hi=40),
    I("roi_bottom_crop_px", 6, G_SEG, "Recorte inferior (px)",
      "Filas de abajo que se ignoran. Sube el valor si en el borde inferior se "
      "ve el propio chasis, un cable o la sombra del robot.", lo=0, hi=200),
    F("roi_x_min_mm", 120.0, G_SEG, "Rango minimo (mm)",
      "Puntos del suelo mas cercanos que esto se descartan. Evita usar la "
      "franja mas baja de la imagen, donde un error de pitch se amplifica.",
      lo=0, hi=600, step=5),
    F("roi_x_max_mm", 1700.0, G_SEG, "Rango maximo (mm)",
      "Distancia maxima que se considera. TODO lo que en la imagen quede por "
      "encima de la fila correspondiente a este rango se recorta antes de "
      "procesar. ESTE ES EL FILTRO QUE ELIMINA EL FONDO (mesas, gente, techo): "
      "como la camara (125 mm) esta MAS ALTA que los muros (100 mm), todo el "
      "muro cae por debajo del horizonte y el fondo queda fuera. "
      "NO LO SUBAS de 1800. La perspectiva comprime muchisimo el fondo: con la "
      "camara a 125 mm y 20 grados de inclinacion, un pixel vale 11 mm a 900 "
      "mm, 37 mm a 1600 mm y 72 mm a 2200 mm. Esa ultima franja son unas pocas "
      "filas de imagen que solo aportan ruido, y es justo por donde se cuelan "
      "los objetos de la sala que quedan por debajo del horizonte. Nada del "
      "control necesita ver tan lejos: la curva se dispara a 700 mm y el "
      "ajuste de muros llega a 1100.",
      lo=500, hi=3200, step=25),
    I("col_step", 3, G_SEG, "Paso de columnas",
      "Se analiza 1 de cada N columnas. 3 da ~210 puntos a 640 px, suficiente "
      "y rapido. Sube a 4-6 si necesitas mas FPS.", lo=1, hi=12),
    I("boundary_median", 5, G_SEG, "Filtro de mediana (columnas)",
      "Mediana movil sobre la fila detectada a lo largo de las columnas. "
      "Elimina columnas sueltas mal detectadas. Debe ser impar.", lo=1, hi=21,
      step=2),
    I("morph_open", 3, G_SEG, "Apertura morfologica (px)",
      "Borra motas de la mascara antes de buscar la base del muro. 0 = "
      "desactivado.", lo=0, hi=9),

    # --------------------------------------------------- geometria de los muros
    F("seg_split_tol_mm", 45.0, G_WAL, "Tolerancia de segmentacion (mm)",
      "El contorno del suelo se parte en tramos rectos (split-and-merge). Un "
      "punto que se separa mas que esto de la recta provoca un corte. Bajalo "
      "para partir mas fino, subelo si un muro recto se parte en trozos.",
      lo=10, hi=200, step=1),
    F("seg_gap_mm", 220.0, G_WAL, "Salto que separa tramos (mm)",
      "Si dos puntos consecutivos del contorno estan a mas de esta distancia, "
      "se consideran muros distintos. Es lo que detecta la ESQUINA CONVEXA "
      "donde termina el muro interior.", lo=60, hi=700, step=5),
    I("seg_min_points", 5, G_WAL, "Puntos minimos por tramo",
      "Tramos con menos puntos se descartan por ruido.", lo=3, hi=40),
    F("seg_range_ref_mm", 700.0, G_WAL, "Distancia de referencia de tolerancias",
      "A esta distancia las tolerancias de corte y hueco valen lo que dicen sus "
      "campos; mas lejos crecen con el CUADRADO de la distancia, porque asi se "
      "degrada la resolucion (1 pixel vale 4 mm a 600 mm y 73 mm a 2200). Sin "
      "esto, un solo pixel de ruido trocea un muro lejano y cada trozo se "
      "clasifica al azar: es el parpadeo de colores con el robot parado. "
      "Bajarlo = mas permisivo lejos.", lo=300, hi=1500, step=25,
      advanced=True),
    F("side_angle_band_deg", 8.0, G_WAL, "Banda muerta de clasificacion",
      "Franja alrededor del angulo limite en la que un tramo NO se clasifica ni "
      "como lateral ni como frontal (se dibuja en gris). Evita que un tramo "
      "justo en el umbral cambie de color en cada fotograma. En esta pista los "
      "muros laterales estan cerca de 0 grados y los frontales cerca de 90, "
      "asi que descartar la franja intermedia no cuesta nada.",
      lo=0, hi=25, step=1),
    F("side_min_y_mm", 90.0, G_WAL, "Separacion minima para ser lateral",
      "Un tramo cuyo centro esta casi sobre el eje del robot no puede llamarse "
      "izquierdo ni derecho: el signo lo decidiria el ruido. Por debajo de esta "
      "separacion se marca como no clasificado.", lo=0, hi=400, step=10),
    F("side_min_len_mm", 90.0, G_WAL, "Longitud minima para ser lateral",
      "Un trozo demasiado corto tiene una orientacion sin sentido. Este minimo "
      "se aplica a la distancia de referencia y CRECE con el cuadrado de la "
      "distancia, igual que las tolerancias: si fuera fijo se descartaria el "
      "muro interior justo al llegar a una esquina, que es cuando su tramo "
      "visible es mas corto (para ver un muro a 340 mm de lado hay que estar a "
      "mas de 400 mm de el, asi que a veces solo se ven 200 mm). Subelo si ves "
      "fragmentos sueltos etiquetados como muro lateral.",
      lo=0, hi=600, step=10),
    F("side_max_angle_deg", 42.0, G_WAL, "Angulo max. de un muro lateral",
      "Un tramo se considera muro LATERAL si su direccion se desvia menos que "
      "esto del eje del robot; si no, se considera muro FRONTAL. 42 grados "
      "separa bien recta y curva.", lo=15, hi=70, step=1),
    F("front_band_mm", 150.0, G_WAL, "Semiancho de la banda frontal (mm)",
      "Para medir la distancia al frente solo se miran puntos con |Y| menor "
      "que esto. Ponlo un poco mayor que el semiancho del robot (100 mm).",
      lo=60, hi=400, step=5),
    F("wall_max_y_mm", 1300.0, G_WAL, "|Y| maximo de un muro (mm)",
      "Puntos mas laterales que esto se ignoran: en un pasillo de 1000 mm no "
      "hay nada valido tan lejos.", lo=400, hi=2500, step=25),
    F("wall_eval_x_mm", 260.0, G_WAL, "Distancia de evaluacion (mm)",
      "La recta ajustada a cada muro se evalua a esta distancia por delante "
      "para dar la distancia lateral. Usar un punto adelantado (en vez de "
      "X=0) es mas estable porque no extrapola.", lo=0, hi=800, step=10),
    F("fit_x_lo_mm", 150.0, G_WAL, "Ajuste: X minimo (mm)",
      "Ventana longitudinal usada para ajustar la recta del muro lateral.",
      lo=0, hi=800, step=10),
    F("fit_x_hi_mm", 1100.0, G_WAL, "Ajuste: X maximo (mm)",
      "Limite superior de la ventana de ajuste. Si lo subes mucho entran "
      "puntos lejanos y ruidosos; si lo bajas demasiado te quedas sin puntos.",
      lo=200, hi=2500, step=25),
    F("ransac_tol_mm", 35.0, G_WAL, "Tolerancia RANSAC (mm)",
      "Distancia maxima de un punto a la recta para contar como inlier al "
      "ajustar el muro.", lo=5, hi=150, step=1),
    F("wall_end_max_mm", 1250.0, G_WAL, "Fin de muro creible (mm)",
      "Si el muro lateral termina (con salto convexo) antes de esta distancia, "
      "se interpreta como 'aqui viene la esquina'. Los tramos rectos miden "
      "1000 mm, asi que el muro interior nunca termina mas alla de ~1100 mm.",
      lo=400, hi=2500, step=25),
    F("smooth_alpha", 0.45, G_WAL, "Suavizado temporal (0-1)",
      "Filtro exponencial sobre distancias y angulos. 1 = sin filtro (rapido "
      "pero nervioso), 0.2 = muy suave (estable pero con retraso).",
      lo=0.05, hi=1.0, step=0.05),

    # ------------------------------------------------------------- conduccion
    C("wall_mode", "inner", G_DRV, "Modo de seguimiento",
      "inner = pega el robot al muro interior (el reglamento prohibe tocar el "
      "muro exterior en el Reto Abierto, asi que ir por dentro es lo seguro y "
      "ademas recorre menos distancia). center = va centrado. adaptive = "
      "centrado en pasillos de 600 mm y pegado al interior en los de 1000 mm.",
      ["inner", "center", "adaptive"]),
    F("target_inner_mm", 340.0, G_DRV, "Distancia objetivo al muro interior",
      "Separacion que se intenta mantener respecto al muro interior, medida "
      "desde el eje del robot. Con un robot de 200 mm de ancho quedan 240 mm "
      "de holgura. Bajalo para trazar mas cerrado (mas rapido, mas riesgo).",
      lo=150, hi=700, step=5),
    F("outer_min_mm", 220.0, G_DRV, "Holgura minima al muro exterior",
      "El objetivo lateral nunca acerca el robot al muro exterior mas de esto. "
      "Es la red de seguridad contra la regla 9.18 (no tocar el muro exterior "
      "en el Reto Abierto).", lo=120, hi=600, step=5),
    F("k_heading", 2.4, G_DRV, "Ganancia de rumbo",
      "Cuanto giro se aplica por cada grado de error de rumbo (giroscopio). "
      "Es la ganancia principal. Subela si el robot tarda en enderezarse; "
      "bajala si oscila (zigzag).", lo=0.2, hi=12.0, step=0.1),
    F("k_lateral", 0.16, G_DRV, "Ganancia lateral",
      "Cuanto se corrige el rumbo objetivo por cada mm de error de distancia "
      "al muro. Muy alta = zigzag; muy baja = el robot se queda descentrado.",
      lo=0.0, hi=1.2, step=0.01),
    F("lat_err_max_mm", 260.0, G_DRV, "Error lateral maximo (mm)",
      "Satura el error lateral para que un fallo puntual de vision no provoque "
      "un volantazo.", lo=50, hi=800, step=10),
    F("seek_inner_deg", 9.0, G_DRV, "Busqueda del muro interior (grados)",
      "Rumbo que se pide hacia el lado interior cuando ya se sabe el sentido "
      "pero el muro interior TODAVIA NO SE VE. Ocurre al arrancar lejos del "
      "interior: para ver un muro a Y mm de lado hace falta estar a mas de "
      "Y/tan(fov/2) de el, asi que con poco campo de vision el muro interior "
      "cae fuera de cuadro. Este sesgo acerca el robot hasta que aparece. "
      "Solo actua si hay sitio de sobra respecto al muro exterior.",
      lo=0, hi=25, step=0.5),
    F("lat_head_max_deg", 26.0, G_DRV, "Correccion de rumbo maxima (grados)",
      "Limite del rumbo extra que puede pedir el control lateral. Evita que "
      "el robot se ponga de lado intentando corregir.", lo=5, hi=60, step=1),
    F("steer_limit", 100.0, G_DRV, "Direccion maxima (%)",
      "Recorrido maximo del servo usado por el control (100 = tope fisico "
      "definido en los pulsos del servo).", lo=20, hi=100, step=1),
    F("steer_slew", 420.0, G_DRV, "Velocidad de giro del volante (%/s)",
      "Limita cuan rapido cambia la direccion. Suaviza la conduccion y evita "
      "que el servo tire de la bateria a golpes.", lo=60, hi=2000, step=10),
    F("base_speed", 45.0, G_DRV, "Velocidad en recta (%)",
      "Velocidad nominal. Empieza en 35-45 y sube cuando la trayectoria sea "
      "estable. 3 vueltas caben de sobra en 3 minutos: prioriza terminar.",
      lo=10, hi=100, step=1),
    F("turn_speed", 32.0, G_DRV, "Velocidad en curva (%)",
      "Velocidad mientras se ejecuta un giro de 90 grados.",
      lo=8, hi=100, step=1),
    F("min_speed", 22.0, G_DRV, "Velocidad minima (%)",
      "Suelo de velocidad: por debajo el motor no mueve el robot.",
      lo=5, hi=60, step=1),
    F("slow_front_mm", 750.0, G_DRV, "Distancia para frenar (mm)",
      "Cuando el muro de enfrente esta mas cerca que esto, la velocidad baja "
      "proporcionalmente hasta la velocidad de curva.", lo=200, hi=2000, step=25),
    F("speed_steer_gain", 0.45, G_DRV, "Freno por direccion (0-1)",
      "Cuanto se reduce la velocidad cuando el volante esta a tope. 0.45 = "
      "hasta un 45% menos con direccion maxima.", lo=0, hi=0.9, step=0.05),
    F("yaw_vision_gain", 0.35, G_DRV, "Fusion rumbo vision-giroscopio",
      "Corrige lentamente la deriva del giroscopio usando el angulo del muro "
      "medido por vision (solo en rectas y con ajuste fiable). 0 = solo "
      "giroscopio. 0.3-0.5 mejora mucho la consistencia en 3 vueltas.",
      lo=0.0, hi=1.5, step=0.05),

    # -------------------------------------------------------- giros y vueltas
    F("turn_trigger_front_mm", 720.0, G_TRN, "Disparo del giro (frente, mm)",
      "PARAMETRO CLAVE DE LAS CURVAS. Se empieza el giro cuando el muro de "
      "enfrente esta a esta distancia.\n"
      "Por que funciona: el muro que tienes delante SERA el muro exterior del "
      "siguiente pasillo, y un giro de 90 grados con radio R adelanta al robot "
      "exactamente R. Girar a (R + holgura) te deja a esa holgura del nuevo "
      "muro exterior, dé igual que el siguiente carril mida 600 o 1000 mm y dé "
      "igual por donde del carril vinieras: CADA CURVA RECOLOCA el robot.\n"
      "Valor tipico = radio de giro minimo (~300 mm en un coche de 20 cm) + "
      "holgura deseada al nuevo muro exterior (~380 mm). El radio real es mayor que el teorico porque el servo tarda en llegar al tope, asi que en la practica salen unos 720. Mide el tuyo: mira en la vista de pajaro a que distancia del muro exterior sales de cada curva y corrige.\n"
      "Si el robot sale de la curva demasiado pegado al muro exterior, SUBELO. "
      "Si sale demasiado abierto (o corta hacia el muro interior), BAJALO.",
      lo=250, hi=1400, step=10),
    F("corner_arm_mm", 650.0, G_TRN, "Armado de la curva (fin de muro, mm)",
      "Cuando la esquina del muro interior queda a menos de esto, la curva se "
      "considera 'armada' y la correccion lateral se atenua al 25 %. Evita que "
      "el robot siga tirando hacia un muro que se esta acabando (y cuyo ajuste "
      "se apoya en cada vez menos puntos).\n"
      "OJO: esto NO dispara el giro. La geometria dice que para bordear la "
      "esquina hay que empezar a girar cuando la esquina esta a "
      "(radio - distancia objetivo) por delante, que con radio ~300 y objetivo "
      "340 sale NEGATIVO: hay que pasarla antes de girar. Usar el fin de muro "
      "como disparo hace que el robot corte contra el muro interior.",
      lo=200, hi=1400, step=10),
    F("turn_hard_front_mm", 300.0, G_TRN, "Disparo de emergencia (mm)",
      "Si el frente se acerca mas que esto se gira si o si, sin importar el "
      "resto de condiciones.", lo=120, hi=700, step=10),
    F("turn_angle_deg", 90.0, G_TRN, "Angulo de giro (grados)",
      "Cuanto se suma al rumbo objetivo en cada esquina. La pista es "
      "ortogonal, asi que 90. Toca solo para compensar un giroscopio con "
      "factor de escala mal (ej. 88 o 92).", lo=70, hi=110, step=0.5),
    F("turn_exit_tol_deg", 10.0, G_TRN, "Tolerancia de salida (grados)",
      "El estado de giro termina cuando falta menos de esto para el rumbo "
      "objetivo. Luego vuelve el seguimiento de muro.", lo=3, hi=45, step=1),
    F("turn_min_time_s", 0.45, G_TRN, "Duracion minima del giro (s)",
      "Impide que el giro se de por terminado en el primer ciclo por ruido.",
      lo=0.05, hi=3.0, step=0.05),
    F("turn_max_time_s", 4.5, G_TRN, "Duracion maxima del giro (s)",
      "Si se supera, se sale del giro igualmente (proteccion anti-bloqueo).",
      lo=1.0, hi=12.0, step=0.1),
    F("corner_cooldown_s", 1.6, G_TRN, "Tiempo muerto tras una esquina (s)",
      "Tras contar una esquina no se admite otra hasta pasado este tiempo. "
      "Evita contar dos veces la misma curva.", lo=0.3, hi=6.0, step=0.1),
    I("laps_target", 3, G_TRN, "Vueltas objetivo",
      "3 segun reglamento. Ponlo a 1 para probar rapido.", lo=1, hi=10),
    F("finish_end_mm", 600.0, G_TRN, "Parada: fin de muro interior (mm)",
      "Tras la ultima curva el robot avanza hasta que la esquina del muro "
      "interior quede a esta distancia. Como los tramos rectos miden 1000 mm, "
      "600 deja al robot hacia la mitad del tramo, bien dentro de la seccion "
      "de meta (regla 9.25.2: la proyeccion entera debe quedar dentro).",
      lo=200, hi=900, step=10),
    F("finish_max_time_s", 4.5, G_TRN, "Parada: tiempo maximo (s)",
      "Si la vision no da la senal de parada, se para de todas formas tras "
      "este tiempo desde la ultima esquina.", lo=0.5, hi=15.0, step=0.1),
    C("direction_source", "auto", G_TRN, "Origen del sentido de marcha",
      "auto = lo deduce solo (sensor de color: azul primero -> antihorario, "
      "naranja primero -> horario; y ademas por vision, viendo que muro se "
      "acaba). cw/ccw = forzado, SOLO PARA PRUEBAS: el reglamento no permite "
      "introducir datos antes de la ronda.",
      ["auto", "cw", "ccw"]),
    B("use_color_lines", True, G_TRN, "Usar el sensor de color",
      "Usa el TCS34725 para deducir el sentido y contar cruces de linea como "
      "verificacion del conteo de vueltas. Si el sensor te da problemas, "
      "desactivalo: la vision sola tambien cuenta esquinas."),

    # -------------------------------------------------------- recuperacion
    F("stuck_time_s", 1.3, G_REC, "Tiempo para declarar atasco (s)",
      "Si se ordena avanzar pero ni la distancia frontal ni el rumbo cambian "
      "durante este tiempo, se declara atasco.", lo=0.3, hi=6.0, step=0.1),
    F("stuck_front_delta_mm", 25.0, G_REC, "Cambio minimo de distancia (mm)",
      "Variacion de la distancia frontal por debajo de la cual se considera "
      "que el robot no avanza.", lo=5, hi=200, step=5),
    F("reverse_time_s", 0.9, G_REC, "Duracion de la marcha atras (s)",
      "Cuanto retrocede al desatascarse.", lo=0.2, hi=3.0, step=0.1),
    F("reverse_speed", 38.0, G_REC, "Velocidad marcha atras (%)",
      "Modulo de la velocidad al retroceder.", lo=10, hi=100, step=1),
    F("emergency_front_mm", 170.0, G_REC, "Frenada de emergencia (mm)",
      "Si aparece algo mas cerca que esto justo delante, se corta la traccion "
      "y se pasa a recuperacion.", lo=80, hi=500, step=5),
    F("round_time_limit_s", 175.0, G_REC, "Limite de la ronda (s)",
      "El reglamento da 180 s. A los 175 el robot se detiene solo para no "
      "quedarse chocando contra un muro al acabar el tiempo.",
      lo=30, hi=200, step=5),

    # ------------------------------------------------------------- obstaculos
    B("obstacles_enabled", False, G_OBS, "Detectar pilares",
      "Activa la deteccion de pilares rojos/verdes. En el Reto Abierto no hay "
      "pilares: dejalo apagado para ahorrar CPU."),
    I("red_h1_lo", 0, G_OBS, "Rojo: H min (banda baja)",
      "El rojo en HSV esta partido en dos bandas (cerca de 0 y cerca de 180). "
      "Esta es la banda baja.", lo=0, hi=180),
    I("red_h1_hi", 8, G_OBS, "Rojo: H max (banda baja)", "Ver arriba.", lo=0, hi=180),
    I("red_h2_lo", 170, G_OBS, "Rojo: H min (banda alta)", "Ver arriba.", lo=0, hi=180),
    I("red_h2_hi", 180, G_OBS, "Rojo: H max (banda alta)", "Ver arriba.", lo=0, hi=180),
    I("red_s_min", 110, G_OBS, "Rojo: saturacion minima",
      "Sube si el suelo o reflejos entran como rojo.", lo=0, hi=255),
    I("red_v_min", 60, G_OBS, "Rojo: brillo minimo",
      "Sube si las sombras oscuras entran como rojo.", lo=0, hi=255),
    I("green_h_lo", 40, G_OBS, "Verde: H min",
      "Rango de tono del verde RGB(68,214,44).", lo=0, hi=180),
    I("green_h_hi", 85, G_OBS, "Verde: H max", "Ver arriba.", lo=0, hi=180),
    I("green_s_min", 90, G_OBS, "Verde: saturacion minima", "Ver arriba.", lo=0, hi=255),
    I("green_v_min", 55, G_OBS, "Verde: brillo minimo", "Ver arriba.", lo=0, hi=255),
    I("magenta_h_lo", 140, G_OBS, "Magenta: H min",
      "Delimitadores del cajon de estacionamiento RGB(255,0,255). Solo se "
      "detectan y se muestran; el estacionamiento no esta implementado.",
      lo=0, hi=180),
    I("magenta_h_hi", 168, G_OBS, "Magenta: H max", "Ver arriba.", lo=0, hi=180),
    I("magenta_s_min", 90, G_OBS, "Magenta: saturacion minima", "Ver arriba.", lo=0, hi=255),
    I("pillar_min_area", 90, G_OBS, "Area minima del pilar (px)",
      "Manchas de color mas pequenas se descartan. Un pilar de 50x100 mm a "
      "1.5 m ocupa unos 100-200 px a 640x480.", lo=20, hi=4000),
    F("pillar_min_fill", 0.45, G_OBS, "Relleno minimo del contorno",
      "Area del contorno dividida entre el area de su caja. Un pilar es un "
      "rectangulo lleno (~0.8); los reflejos y bordes tienen relleno bajo.",
      lo=0.1, hi=1.0, step=0.05),
    F("pillar_max_range_mm", 1500.0, G_OBS, "Alcance maximo (mm)",
      "Pilares mas lejos de esto se ignoran para el control (se siguen "
      "dibujando).", lo=300, hi=3000, step=50),
    F("pillar_pass_offset_mm", 260.0, G_OBS, "Separacion al esquivar (mm)",
      "A que distancia lateral del centro del pilar debe pasar el robot. "
      "25 mm (medio pilar) + 100 mm (medio robot) + margen. Rojo se rebasa por "
      "su derecha, verde por su izquierda (regla 9.19).",
      lo=120, hi=500, step=10),
    F("pillar_react_mm", 950.0, G_OBS, "Distancia de reaccion (mm)",
      "A partir de aqui el pilar empieza a desviar la trayectoria. Mas alto = "
      "maniobra mas suave y temprana.", lo=300, hi=2000, step=25),

    # ------------------------------------------------------------- ESP32
    I("servo_center_us", 1500, G_ESP, "Servo: pulso centro (us)",
      "Pulso con el que las ruedas quedan RECTAS. Ajustalo primero, en modo "
      "manual y con direccion a 0, hasta que el robot ruede recto.",
      lo=800, hi=2200, target="esp32"),
    I("servo_left_us", 2000, G_ESP, "Servo: pulso izquierda (us)",
      "Pulso para direccion +100% (izquierda). Reducelo si el servo hace tope "
      "mecanico y zumba: forzarlo quema el MG996R y descalibra la direccion.",
      lo=800, hi=2400, target="esp32"),
    I("servo_right_us", 1000, G_ESP, "Servo: pulso derecha (us)",
      "Pulso para direccion -100% (derecha). Mismo cuidado con los topes.",
      lo=500, hi=2200, target="esp32"),
    I("servo_slew_us", 4000, G_ESP, "Servo: velocidad maxima (us/s)",
      "Limite de velocidad del servo. Bajalo si los golpes de direccion "
      "provocan reinicios por caida de tension.", lo=500, hi=40000,
      target="esp32", advanced=True),
    B("steer_invert", False, G_ESP, "Invertir direccion",
      "Actívalo si al pedir izquierda el robot gira a la derecha.",
      target="esp32"),
    B("motor_invert", False, G_ESP, "Invertir motor",
      "Actívalo si al pedir avance el robot retrocede (equivale a intercambiar "
      "RPWM y LPWM).", target="esp32"),
    I("motor_min_pwm", 40, G_ESP, "PWM minimo",
      "PWM (0-255) a partir del cual el motor realmente mueve el robot. "
      "Buscalo en modo manual subiendo el acelerador muy despacio. Vencer la "
      "friccion estatica aqui hace que las velocidades bajas sean utiles.",
      lo=0, hi=200, target="esp32"),
    I("motor_max_pwm", 255, G_ESP, "PWM maximo",
      "Tope de PWM. Bajalo para limitar la velocidad punta del robot.",
      lo=40, hi=255, target="esp32"),
    I("motor_slew", 900, G_ESP, "Rampa del motor (PWM/s)",
      "Limita la aceleracion. Evita que el arranque brusco haga patinar las "
      "ruedas o reinicie la Pi por caida de tension.", lo=50, hi=20000,
      target="esp32", advanced=True),
    B("yaw_invert", False, G_ESP, "Invertir giroscopio",
      "Actívalo si al girar el robot a la IZQUIERDA el rumbo (yaw) baja en vez "
      "de subir. En este proyecto el rumbo positivo es siempre hacia la "
      "izquierda; si el MPU6050 va montado boca abajo hay que invertirlo."),
    B("brake_active", True, G_ESP, "Frenado activo",
      "Con el motor parado deja ambos habilitadores activos, cortocircuitando "
      "el motor: frena mucho mejor que dejarlo en rueda libre.",
      target="esp32"),
    F("th_orange_r", 0.44, G_ESP, "Color: r min naranja",
      "Componente roja normalizada minima para dar una linea por naranja. "
      "Mira los valores r/g/b en vivo en el panel pasando el sensor por "
      "encima de la linea.", lo=0.2, hi=0.9, step=0.01, target="esp32"),
    F("th_orange_b", 0.22, G_ESP, "Color: b max naranja",
      "Componente azul normalizada maxima para naranja.",
      lo=0.02, hi=0.5, step=0.01, target="esp32"),
    F("th_blue_b", 0.42, G_ESP, "Color: b min azul",
      "Componente azul normalizada minima para dar una linea por azul.",
      lo=0.2, hi=0.9, step=0.01, target="esp32"),
    F("th_blue_r", 0.26, G_ESP, "Color: r max azul",
      "Componente roja normalizada maxima para azul.",
      lo=0.02, hi=0.5, step=0.01, target="esp32"),
    I("th_clear_min", 60, G_ESP, "Color: canal C minimo",
      "Por debajo de este nivel de luz la lectura se descarta. Sube si el "
      "sensor dispara en la oscuridad bajo el chasis.", lo=5, hi=2000,
      target="esp32"),
    I("confirm_n", 2, G_ESP, "Color: muestras para confirmar",
      "Lecturas consecutivas del mismo color necesarias para dar un evento.",
      lo=1, hi=15, target="esp32"),
    I("refractory_ms", 220, G_ESP, "Color: tiempo entre eventos (ms)",
      "Tiempo muerto tras detectar una linea. Evita contar la misma linea "
      "varias veces al cruzarla en diagonal.", lo=50, hi=1500, target="esp32"),

    # ----------------------------------------------------------------- sistema
    T("serial_port", "auto", G_SYS, "Puerto serie del ESP32",
      "'auto' busca /dev/ttyUSB* y /dev/ttyACM*. Para fijarlo usa una regla "
      "udev y pon aqui la ruta estable."),
    I("serial_baud", 115200, G_SYS, "Baudios", "Debe coincidir con el firmware.",
      lo=9600, hi=1000000),
    I("control_hz", 40, G_SYS, "Frecuencia del lazo de control (Hz)",
      "Ritmo del lazo principal. 30-40 Hz va sobrado en una Pi 5.",
      lo=10, hi=120),
    I("web_port", 8000, G_SYS, "Puerto del servidor web",
      "Puerto del panel de depuracion.", lo=1024, hi=65535),
    I("start_button_pin", -1, G_SYS, "GPIO del boton de arranque (Pi)",
      "Numero BCM del pin del boton fisico de arranque (a GND, pull-up "
      "interno). -1 lo desactiva. El reglamento (9.11) exige un unico boton de "
      "arranque; tambien puedes usar el del ESP32 (GPIO 4).", lo=-1, hi=27),
    B("record_run", False, G_SYS, "Grabar la ronda (caja negra)",
      "Mientras el robot este armado, guarda fotogramas con la decision de "
      "control de cada instante en rpi/grabaciones/. Sirve para reconstruir "
      "despues por que hizo lo que hizo: con tools/reproducir_ronda.py sale un "
      "video con la superposicion y una linea de tiempo de los disparos de "
      "curva. Se graba en JPEG (no PNG) porque una vuelta entera son miles de "
      "fotogramas; para medir ruido con precision usa las capturas PNG."),
    I("record_fps", 10, G_SYS, "Fotogramas por segundo a grabar",
      "Ritmo de grabacion de la caja negra. 10 basta para entender las "
      "decisiones y deja una ronda de 100 s en unos 40 MB. Subelo solo si "
      "necesitas ver un detalle rapido.", lo=2, hi=30),
    I("record_quality", 85, G_SYS, "Calidad JPEG de la grabacion",
      "Compromiso entre tamano y detalle. Por debajo de 75 los artefactos "
      "empiezan a notarse en el borde del muro.", lo=50, hi=100,
      advanced=True),
    B("log_enabled", True, G_SYS, "Guardar registro CSV",
      "Guarda telemetria en logs/ para analizar despues de cada intento."),
]


SPECS: Dict[str, Spec] = {s.key: s for s in PARAMS}
DEFAULTS: Dict[str, Any] = {s.key: s.default for s in PARAMS}
ESP_KEYS = [s.key for s in PARAMS if s.target == "esp32"]


class Config:
    """Diccionario de parametros con acceso por atributo y persistencia JSON."""

    def __init__(self, path: str):
        self.path = path
        self._lock = threading.RLock()
        self._d: Dict[str, Any] = dict(DEFAULTS)
        self.load()

    # -- acceso ------------------------------------------------------------
    def __getattr__(self, k):
        try:
            return object.__getattribute__(self, "_d")[k]
        except KeyError:
            raise AttributeError(k)

    def get(self, k, default=None):
        return self._d.get(k, default)

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._d)

    # -- escritura ---------------------------------------------------------
    def set_many(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        """Valida y aplica. Devuelve solo lo que realmente cambio."""
        changed = {}
        with self._lock:
            for k, v in updates.items():
                spec = SPECS.get(k)
                if spec is None:
                    continue
                try:
                    v = _coerce(spec, v)
                except (TypeError, ValueError):
                    continue
                if self._d.get(k) != v:
                    self._d[k] = v
                    changed[k] = v
        return changed

    def reset(self, keys=None):
        with self._lock:
            if keys is None:
                self._d = dict(DEFAULTS)
            else:
                for k in keys:
                    if k in DEFAULTS:
                        self._d[k] = DEFAULTS[k]

    # -- disco -------------------------------------------------------------
    def load(self):
        if not os.path.isfile(self.path):
            return
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            self.set_many(data)
        except Exception as exc:                       # pragma: no cover
            print("[config] no se pudo leer %s: %s" % (self.path, exc))

    def save(self):
        with self._lock:
            data = dict(self._d)
        tmp = self.path + ".tmp"
        os.makedirs(os.path.dirname(os.path.abspath(self.path)) or ".", exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, sort_keys=True, ensure_ascii=False)
        os.replace(tmp, self.path)

    # -- descripcion para la interfaz --------------------------------------
    @staticmethod
    def schema() -> List[Dict[str, Any]]:
        out = []
        for s in PARAMS:
            out.append({
                "key": s.key, "group": s.group, "label": s.label,
                "help": s.help, "kind": s.kind, "lo": s.lo, "hi": s.hi,
                "step": s.step, "choices": s.choices, "target": s.target,
                "advanced": s.advanced, "default": s.default,
            })
        return out


def _coerce(spec: Spec, v: Any) -> Any:
    if spec.kind == "bool":
        if isinstance(v, str):
            return v.strip().lower() in ("1", "true", "on", "yes", "si")
        return bool(v)
    if spec.kind == "int":
        v = int(round(float(v)))
    elif spec.kind == "float":
        v = float(v)
    elif spec.kind == "choice":
        v = str(v)
        if spec.choices and v not in spec.choices:
            raise ValueError(v)
        return v
    else:
        return str(v)
    if spec.lo is not None:
        v = max(v, type(v)(spec.lo))
    if spec.hi is not None:
        v = min(v, type(v)(spec.hi))
    return v
