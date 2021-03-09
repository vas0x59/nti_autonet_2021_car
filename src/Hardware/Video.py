from Hardware.Interface import IHardware
import numpy as np
import cv2


class HardwareVideo(IHardware):
    def __init__(self, frame_wh, **params):
        super().__init__(frame_wh, **params)
        self._cap = cv2.VideoCapture(self._params["file"])

    def set(self, servo: float, motor: float):
        # cv2.imshow("")
        # cv2.waitKey(1)
        print(f"[HardwareVideo]                                     {servo} {motor}")

    def get(self):
        ret, frame = self._cap.read()
        if not ret:
            return "Error", None
        else:
            return "OK", cv2.resize(frame, self.frame_wh)

