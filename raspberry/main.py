import cv2
import numpy as np
import sys # Añade esta librería nativa al principio del archivo

def iniciar():
    if sys.platform.startswith('win'):
        # Si es Windows, usa DirectShow
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    else:
        # Si es Linux (Raspberry), usa V4L2
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)




if __name__ == "__main__":
    iniciar()