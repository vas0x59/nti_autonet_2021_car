from Hardware.Interface import IHardware
import numpy as np
import cv2
import time

class HardwareVideo(IHardware):
    def __init__(self, frame_wh, **params):
        super().__init__(frame_wh, **params)
        self._cap = cv2.VideoCapture(self._params["file"])

    def set(self, servo: float, motor: float):
        pass
        # cv2.imshow("")
        # cv2.waitKey(1)
        # print(f"[HardwareVideo]                                     {servo} {motor}")

    def get(self):
        ret, frame = self._cap.read()
        # time.sleep(0.05)
        if not ret:
            print("THE END")
            return "Error", None
        else:
            return "OK", cv2.resize(frame, self.frame_wh)

