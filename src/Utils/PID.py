import time
import numpy as np


class PID:
    def __init__(self, kP: float, kI: float, kD: float, clip_integral: float = None):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.prev_time = time.time()
        self.dt = 0
        self.prev_error = 0
        self.first = True
        self.integral = 0
        self.clip_integral = clip_integral

    def calc(self, err):

        self.dt = (time.time() - self.prev_time)
        self.integral += err * self.dt
        res = 0
        if not self.first:
            res = self.kP * err + self.kD * \
                       ((err - self.prev_error) / self.dt) + self.kI * self.integral
        else:
            res = self.kP * err
            self.first = False
        if self.clip_integral is not None:
            self.integral = np.clip(self.integral, -self.clip_integral, self.clip_integral)
        self.prev_error = err
        self.prev_time = time.time()
        return res

