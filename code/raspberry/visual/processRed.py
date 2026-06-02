import cv2
import numpy as np

class RedProcessor:
    def __init__(self,kernel=np.ones((5,5), np.uint8)):

        # Define the lower and upper bounds for red color in HSV color space
        self.kernel = kernel   
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])

        pass

    def process(self, frame):
        # the frame is already in HSV color space, so we can directly apply the mask

        # Create masks for red color, two ranges are needed to cover the hue values for red
        mask1 = cv2.inRange(frame, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(frame, self.lower_red2, self.upper_red2)

        # Combine the masks to get the final mask for red color
        red_mask = cv2.add(mask1, mask2)
        # Optional: Apply morphological operations to remove noise from the mask
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel)

       
        return red_mask