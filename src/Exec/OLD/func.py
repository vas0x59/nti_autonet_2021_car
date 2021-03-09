import cv2
import numpy as np
import time
import threading
from functools import wraps
timeout_detect_stop = 0
KP = 0.32  # 0.22
KD = 0.16
last = 0
SIZE = (400, 300)

RECT = np.float32([[0, 299],
                   [399, 299],
                   [399, 0],
                   [0, 0]])

TRAP = np.float32([[0, 299],    
                   [399, 299],
                   [320, 200],
                   [80, 200]])

timeout = 0
l = 1
r = 0

povor = 0
totl = 1
pid = 0

ESCAPE = 27
SPASE = 32

i = 1
j = 0

IPadress = "192.168.1.104"

flag = 1
key = 1
fn = 1
speed = 1548
stop_speed = 1500
def delay(delay=0.):
    """
    Decorator delaying the execution of a function for a while.
    """
    def wrap(f):
        @wraps(f)
        def delayed(*args, **kwargs):
            timer = threading.Timer(delay, f, args=args, kwargs=kwargs)
            timer.start()
        return delayed
    return wrap

# def detect_stop(perspective):
#     if (perspective[150][180] > 200) and (perspective[150][200] > 200) and (perspective[150][160] > 200):
#         return True
#     return False


def binarize(img, d=0):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    binary_h = cv2.inRange(hls, (0, 0, 30), (255, 255, 255))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_g = cv2.inRange(gray, 150, 255)  # 130

    binary = cv2.bitwise_and(binary_g, binary_h)

    if d:
        cv2.imshow('hls', hls)
        cv2.imshow('hlsRange', binary_h)
        cv2.imshow('grayRange', binary_g)
        cv2.imshow('gray', gray)
        cv2.imshow('bin', binary)

    # return binary
    return binary_g


def trans_perspective(binary, trap, rect, size, d=0):
    matrix_trans = cv2.getPerspectiveTransform(trap, rect)
    perspective = cv2.warpPerspective(binary, matrix_trans, size, flags=cv2.INTER_LINEAR)
    if d:
        cv2.imshow('perspective', perspective)
    return perspective


def find_left_right(perspective, d=0):
    hist = np.sum(perspective[perspective.shape[0] // 3:, :], axis=0)
    mid = hist.shape[0] // 2
    left = np.argmax(hist[:mid])
    right = np.argmax(hist[mid:]) + mid
    if left <= 10 and right - mid <= 10:
        right = 399

    if d:
        cv2.line(perspective, (left, 0), (left, 300), 50, 2)
        cv2.line(perspective, (right, 0), (right, 300), 50, 2)
        cv2.line(perspective, ((left + right) // 2, 0), ((left + right) // 2, 300), 110, 3)
        cv2.imshow('lines', perspective)

    return left, right


def centre_mass(perspective, d=0):
    hist = np.sum(perspective, axis=0)
    if d:
        cv2.imshow("Perspektiv2in", perspective)

    mid = hist.shape[0] // 2
    i = 0
    centre = 0
    sum_mass = 0
    while (i <= mid):
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass > 0:
        mid_mass_left = centre / sum_mass
    else:
        mid_mass_left = mid - 1

    centre = 0
    sum_mass = 0
    i = mid
    while (i < hist.shape[0]):
        centre += hist[i] * (i + 1)
        sum_mass += hist[i]
        i += 1
    if sum_mass > 0:
        mid_mass_right = centre / sum_mass
    else:
        mid_mass_right = mid + 1

    # print(mid_mass_left)
    # print(mid_mass_right)
    mid_mass_left = int(mid_mass_left)
    mid_mass_right = int(mid_mass_right)
    if d:
        cv2.line(perspective, (mid_mass_left, 0), (mid_mass_left, 300), 50, 2)
        cv2.line(perspective, (mid_mass_right, 0), (mid_mass_right, 300), 50, 2)
        # cv2.line(perspective, ((mid_mass_right + mid_mass_left) // 2, 0), ((mid_mass_right + mid_mass_left) // 2, 300), 110, 3)
        cv2.imshow('CentrMass', perspective)

    return mid_mass_left, mid_mass_right

def wait_time(time_wait: int):
    last_time = time.time()
    now_time = time.time()
#     print(last_time)
    while now_time - last_time < time_wait:
        now_time = time.time()

class PD:
    def __init__(self, kP, kD):
        self.kP = kP
        self.kD = kD
        self.prev_error = 0
        self.res = 0
        self.err = 0

    def calc(self, left, right):
        self.err = 0 - ((left + right) // 2 - 200)
        if abs(right - left) < 100:
            self.err = self.prev_error
        self.res = int(87 + self.kP * self.err + self.kD * (self.err - self.prev_error))
        self.prev_error = self.err
        return self.res