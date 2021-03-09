import numpy as np


class IHardware:
    def __init__(self, frame_wh, **params):
        self._params = params
        self.frame_wh = frame_wh

    def set(self, servo: float, motor: float):
        pass

    def get(self):
        return "Error", np.zeros((480, 640, 3))
