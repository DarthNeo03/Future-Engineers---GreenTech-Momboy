import cv2
import numpy as np
import sys

# modules for color processing

from visual.processColor import ColorProcessor

kernel = np.ones((15,15), np.uint8) # this is the size of each kernel/group of pixels to avoid small flashes from the camera
height = 480
width = 640


color_processor = ColorProcessor(kernel=kernel, height=height, width=width) 
def iniciar():

    ############################################# init camera diferent for windows and linux (raspberry)
    if sys.platform.startswith('win'):
        # if the platform is Windows, use DirectShow
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    else:
        # if the platform is Linux (Raspberry), use V4L2
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    #############################################


    # init bucle camera
    while True:
        ret , frame = cap.read()
        if not ret:
            print("ERROR: No se puede leer datos de la cámara. Verifica la conexión o el índice.")
            break

        hsv_frame , rgb_frame = color_processor.process(frame)
        

        # show the result
        cv2.imshow("Robot Vision", rgb_frame)


        # close the program when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    


if __name__ == "__main__":
    iniciar()