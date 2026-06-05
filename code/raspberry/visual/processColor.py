import numpy as np
import cv2

# integrate the color processing modules into a single class
from .processRed import RedProcessor
from .processGreen import GreenProcessor


class ColorProcessor:
    def __init__(self, kernel = None, height=480, width=640):

        if kernel is None:
            self.kernel = np.ones((15,15), np.uint8) #  this is the size of each kernel/group of pixels to avoid small flashes from the camera
        else:
            self.kernel = kernel
        self.height = height
        self.width = width
        self.processRed = RedProcessor(kernel=self.kernel)
        self.processGreen = GreenProcessor(kernel=self.kernel)

        ## calculo de alcutra y ancho del frame para calcular la distancia a los objetos

        camino_izq_base = (140, self.height)
        camino_izq_tope = (260, self.height - 480)
        camino_der_base = (500, self.height)
        camino_der_tope = (380, self.height - 480)
        linea_choque_y = self.height - 100

        self.dibujar_interfaz = {
            "camino_izq_base": camino_izq_base,
            "camino_izq_tope": camino_izq_tope,
            "camino_der_base": camino_der_base,
            "camino_der_tope": camino_der_tope,
            "linea_choque_y": linea_choque_y
        }

        pass
    def process(self, frame, draw=True):
    
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert the image to HSV color space

        red_mask = self.processRed.process(hsv_frame)
        green_mask = self.processGreen.process(hsv_frame)

        hsv_frame, centro_X, area, frame = self.identifyColor(hsv_frame, red_mask, "ROJO", "TAG_ROJO", draw , frame)
        hsv_frame, centro_X, area, frame = self.identifyColor(hsv_frame, green_mask, "VERDE", "TAG_VERDE", draw ,frame)

                # 3. Dibujar la Interfaz de Navegación (Túnel y Línea de Choque)
        # Línea izquierda (Azul)
        cv2.line(frame, self.dibujar_interfaz["camino_izq_base"], self.dibujar_interfaz["camino_izq_tope"], (255, 0, 0), 2)
        # Línea derecha (Azul)
        cv2.line(frame, self.dibujar_interfaz["camino_der_base"], self.dibujar_interfaz["camino_der_tope"], (255, 0, 0), 2)
        # Línea de Choque Inminente (Roja, a lo ancho de toda la pantalla)
        cv2.line(frame, (0, self.dibujar_interfaz["linea_choque_y"]), (640, self.dibujar_interfaz["linea_choque_y"]), (0, 0, 255), 3)
        cv2.putText(frame, "ZONA DE REVERSA", (10, self.dibujar_interfaz["linea_choque_y"] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


        return hsv_frame , frame
    
    def identifyColor(self, hsv_frame, mask, color_name, tag, draw, frame):
        # Implementation for identifying color in the frame

        # Se aplica DESPUÉS del MORPH_OPEN
        mascara_limpia = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mascara_perfecta = cv2.morphologyEx(mascara_limpia, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mascara_perfecta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        best_center_x = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000 and area > max_area:
                max_area = area
                x, y, w, h = cv2.boundingRect(contour)
                best_center_x = x + (w // 2)
                center_y = y + (h // 2)

                if color_name == "ROJO":
                    color_dibujo = (0, 0, 255)  # Red color in BGR
                elif color_name == "VERDE":
                    color_dibujo = (0, 255, 0)  # Green color in BGR

                if draw:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_dibujo, 2)
                    cv2.circle(frame, (best_center_x, center_y), 5, color_dibujo, -1)
                    cv2.putText(frame, f"{color_name} | {tag}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_dibujo, 2)
        return hsv_frame, best_center_x, max_area, frame