import numpy as np
import cv2

# integrate the color processing modules into a single class
from .processRed import RedProcessor
from .processGreen import GreenProcessor


class ColorProcessor:
    def __init__(self):

        kernel = np.ones((5,5), np.uint8) #  this is the size of each kernel/group of pixels to avoid small flashes from the camera
        self.processRed = RedProcessor(kernel=kernel)
        self.processGreen = GreenProcessor(kernel=kernel)
        pass
    def process(self, frame, draw=True):
    
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Convert the image to HSV color space

        red_mask = self.processRed.process(hsv_frame)
        green_mask = self.processGreen.process(hsv_frame)

        hsv_frame, centro_X, area, frame = self.identifyColor(hsv_frame, red_mask, "ROJO", "TAG_ROJO", draw , frame)
        hsv_frame, centro_X, area, frame = self.identifyColor(hsv_frame, green_mask, "VERDE", "TAG_VERDE", draw ,frame)


        return hsv_frame , frame
    
    def identifyColor(self, hsv_frame, mask, color_name, tag, draw, frame):
        # Implementation for identifying color in the frame
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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