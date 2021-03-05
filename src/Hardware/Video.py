import numpy as np
import cv2

class HardwareVideo:
    def __init__(self, **params):
        self._params = params
        self._cap = cv2.VideoCapture(self._params["file"])
    def set(self, servo: float, motor: float):
        # cv2.imshow("")
        # cv2.waitKey(1)
        print(f"                                            {servo} {motor}")
    def get(self):
        ret, frame = self._cap.read()
        if ret == False:
            return "Error", None
        else:
            return "OK", frame
        
    