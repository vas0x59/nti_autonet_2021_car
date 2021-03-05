import numpy as np


class IHardware:
    def __init__(self, **params):
        pass
    def set(self, servo: float, motor: float):
        pass
    def get(self) -> np.array:
        pass
    