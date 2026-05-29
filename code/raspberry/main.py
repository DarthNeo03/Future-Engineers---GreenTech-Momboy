import cv2
import numpy as np
import sys # Añade esta librería nativa al principio del archivo

#modulo internos

import visual.processColor

def iniciar():
    if sys.platform.startswith('win'):
        # Si es Windows, usa DirectShow
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    else:
        # Si es Linux (Raspberry), usa V4L2
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    kernel = np.ones((1,1), np.uint8) # este es el tamaño de cada nucleo/grupo de pixel para evitar pequeños destellos de la camara


    # iniciar el bucle de la camara
    while True:
        ret , frame = cap.read()
        if not ret:
            print("ERROR: No se puede leer datos de la cámara. Verifica la conexión o el índice.")
            break
    


if __name__ == "__main__":
    iniciar()