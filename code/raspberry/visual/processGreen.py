import cv2
import numpy as np

class GreenProcessor:
    def __init__(self,kernel=np.ones((5,5), np.uint8)):

        # Define the lower and upper bounds for green color in HSV color space
        self.kernel = kernel   
        self.lower_green = np.array([35, 50, 50])
        self.upper_green = np.array([85, 255, 255])


        pass

    def process(self, frame):
        # the frame is already in HSV color space, so we can directly apply the mask

        # Create masks for green color, two ranges are needed to cover the hue values for green
        mask1 = cv2.inRange(frame, self.lower_green, self.upper_green)

        # Optional: Apply morphological operations to remove noise from the mask
        green_mask = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, self.kernel)

        
        return green_mask