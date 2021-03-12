import cv2
import numpy as np
import time
import threading
from functools import wraps
timeout_detect_stop = 0
KP = 0.3  # 0.22
KD = 0.1
last = 0
SIZE = (400, 300)
SERVO_0 = 87
speed = 1549
stop_speed = 1500

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
    binary_h = cv2.inRange(hls, (245, 245, 245), (255, 255, 255))

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    binary_g = cv2.inRange(gray, 200, 255)  # 130

    binary = cv2.bitwise_and(binary_g, binary_h)

    # if d:
    #     cv2.imshow('hls', hls)
    #     cv2.imshow('hlsRange', binary_h)
    #     cv2.imshow('grayRange', binary_g)
    #     cv2.imshow('gray', gray)
    #     cv2.imshow('bin', binary)

    # return binary
    return binary_g


def trans_perspective(binary, trap, rect, size, d=0):
    matrix_trans = cv2.getPerspectiveTransform(trap, rect)
    perspective = cv2.warpPerspective(binary, matrix_trans, size, flags=cv2.INTER_LINEAR)
    # if d:
        # cv2.imshow('perspective', perspective)
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
        # cv2.imshow('lines', perspective)

    return left, right


# def centre_mass(perspective, d=0):
#     hist = np.sum(perspective, axis=0)
#     if d:
#         cv2.imshow("Perspektiv2in", perspective)
#
#     mid = hist.shape[0] // 2
#     i = 0
#     centre = 0
#     sum_mass = 0
#     while (i <= mid):
#         centre += hist[i] * (i + 1)
#         sum_mass += hist[i]
#         i += 1
#     if sum_mass > 0:
#         mid_mass_left = centre / sum_mass
#     else:
#         mid_mass_left = mid - 1
#
#     centre = 0
#     sum_mass = 0
#     i = mid
#     while (i < hist.shape[0]):
#         centre += hist[i] * (i + 1)
#         sum_mass += hist[i]
#         i += 1
#     if sum_mass > 0:
#         mid_mass_right = centre / sum_mass
#     else:
#         mid_mass_right = mid + 1
#
#     # print(mid_mass_left)
#     # print(mid_mass_right)
#     mid_mass_left = int(mid_mass_left)
#     mid_mass_right = int(mid_mass_right)
#     if True:
#         cv2.line(perspective, (mid_mass_left, 0), (mid_mass_left, 300), 50, 2)
#         cv2.line(perspective, (mid_mass_right, 0), (mid_mass_right, 300), 50, 2)
#         # cv2.line(perspective, ((mid_mass_right + mid_mass_left) // 2, 0), ((mid_mass_right + mid_mass_left) // 2, 300), 110, 3)
#         cv2.imshow('CentrMass', perspective)
#
#     return mid_mass_left, mid_mass_right
windows_count = 10
window_width = 80
window_thresh = 50
window_thresh_max = 600

KoL = 0.7
KoD = 0.3
def centre_mass(warped):
    warped_v = warped.copy()
    points = [[[None, None, False] for _ in range(windows_count)],
              [[None, None, False] for _ in range(windows_count)]]

    window_height = np.int(warped.shape[0] / windows_count)
    histogram = np.sum(warped[warped.shape[0] // 2:, :], axis=0)

    midpoint = histogram.shape[0] // 2
    IndWhitestColumnL = np.argmax(histogram[:midpoint])
    IndWhitestColumnR = np.argmax(histogram[midpoint:]) + midpoint

    x_center_window = [IndWhitestColumnL, IndWhitestColumnR]

    for i in range(2):
        for window in range(windows_count):
            win_y1 = int(warped.shape[0] - (window + 1) * window_height)
            win_y2 = int(warped.shape[0] - (window) * window_height)

            win_x1 = int(np.clip(x_center_window[i] - window_width / 2, 0, warped.shape[1]))
            win_x2 = int(np.clip(x_center_window[i] + window_width / 2, 0, warped.shape[1]))
            # if show == True:

            # cv2.rectangle(out_img, (right_win_x1, win_y1),
            #               (right_win_x2, win_y2), (0, 0, 50 + window * 21), 2)

            # right_win_x1 = np.clip(XCenterRightWindow - self.lane_sep/2, 0, warped.shape[1])
            # right_win_x2 = np.clip(XCenterRightWindow + self.lane_sep/2, 0, warped.shape[1])

            point = [None, None, True]
            croped = warped[win_y1:win_y2, win_x1:win_x2]
            # cv2.imshow("croped", croped)
            # for i, croped in enumerate([, warped[win_y1:win_y2, right_win_x1:right_win_x2, :]]):
            ssss = np.sum(croped) / 255
            if ssss >= window_thresh and ssss <= window_thresh_max:
                m = cv2.moments(croped, True)
                point[0] = m['m10'] / (m["m00"] + 1e-7)
                point[1] = m['m01'] / (m["m00"] + 1e-7)
                if point[0] == float("nan") or point[1] == float("nan") or point[0] == -float("nan") or point[
                    1] == -float("nan"):
                    point[0] = x_center_window[i]
                    point[1] = win_y1
                    point[2] = False
                else:
                    point[0] += win_x1
                    point[1] += win_y1
                    cv2.rectangle(warped_v, (win_x1, win_y1),
                                  (win_x2, win_y2), (50 + window * 21, i * 255, 0), 2)
            else:
                point[0] = x_center_window[i]
                point[1] = win_y1
                point[2] = False

            # if sum(map(lambda x: not x, points[np.clip(i-self._params.windows_count//3, 0, self._params.windows_count-1):i])) >= 2:
            #     point[2] = False

            # if self._prev_wr_points is not None:
            #     # # print("self._prev_wr_points[window]", self._prev_wr_points[window])
            #     # print("L", len(self._prev_wr_points[i]))
            #     if self._prev_wr_points[i][window][2] == True:
            #         if abs(self._prev_wr_points[i][window][0] - point[0]) > self._params.window_max_d:
            #             point[0] = (self._prev_wr_points[i][window][0] + point[0]) / 2
            # print("point_point_point_point", point)
            points[i][window] = point
            x_center_window[i] = points[i][window][0]
    # points = [[i[:2] for i in ps if i[2] == True] for ps in points]
    # print("left", points[0])
    # print("right", points[1])

    # self._prev_wr_points = points
    po = [np.array([i[:2] for i in ps if i[2] == True]) for ps in points]
    for i, p in enumerate(po):
        for pp in p:
            cv2.circle(warped_v, (int(
                pp[0]), int(pp[1])), 5, (50 + 21, i * 255, 0), 1)
    # po = np.array(po)
    # cv2.imshow("warped_v", warped_v)
    # print(p[0].shape)
    if len(po[0]) == 0:
        po[0] = np.array([[warped.shape[1]//2, 0]])
    if len(po[1]) == 0:
        po[1] = np.array([[warped.shape[1]//2, 0]])
    left, right = int(po[0][:, 0].mean()), int(po[1][:, 0].mean())
    left_d = warped.shape[1]//2+int(po[0][:, 0][po[0][:, 1].argmin()] - po[0][:, 0][po[0][:, 1].argmax()])
    right_d = warped.shape[1]//2+int(po[1][:, 0][po[1][:, 1].argmin()] - po[1][:, 0][po[1][:, 1].argmax()])
    # print(one_d, one)
    if True:
        cv2.line(warped_v, (left, 0), (left, 300), 100, 2)
        cv2.line(warped_v, (right, 0), (right, 300), 50, 2)
        cv2.line(warped_v, ((right + left) // 2, 0), ((right + left) // 2, 300), 255, 3)
    cv2.imshow('warped_v', warped_v)
    return int(left*KoL+left_d*KoD), int(right*KoL+right_d*KoD)

# windows_count = 10
window_width_2 = 115
window_thresh_2 = 20
window_thresh_max_2 = 800


def centre_mass1(warped):
    warped_v = warped.copy()
    points = [[[None, None, False] for _ in range(windows_count)],
              [[None, None, False] for _ in range(windows_count)]]

    window_height = np.int(warped.shape[0] / windows_count)
    histogram = np.sum(warped[warped.shape[0] // 2:, :], axis=0)

    # midpoint = histogram.shape[0] // 2
    # IndWhitestColumnL = np.argmax(histogram[:midpoint])
    IndWhitestColumnR = np.argmax(histogram)

    x_center_window = [IndWhitestColumnR]

    for i in range(1):
        for window in range(windows_count):
            win_y1 = int(warped.shape[0] - (window + 1) * window_height)
            win_y2 = int(warped.shape[0] - (window) * window_height)

            win_x1 = int(np.clip(x_center_window[i] - window_width_2 / 2, 0, warped.shape[1]))
            win_x2 = int(np.clip(x_center_window[i] + window_width_2 / 2, 0, warped.shape[1]))
            # if show == True:

            # cv2.rectangle(out_img, (right_win_x1, win_y1),
            #               (right_win_x2, win_y2), (0, 0, 50 + window * 21), 2)

            # right_win_x1 = np.clip(XCenterRightWindow - self.lane_sep/2, 0, warped.shape[1])
            # right_win_x2 = np.clip(XCenterRightWindow + self.lane_sep/2, 0, warped.shape[1])

            point = [None, None, True]
            croped = warped[win_y1:win_y2, win_x1:win_x2]
            # cv2.imshow("croped", croped)
            # for i, croped in enumerate([, warped[win_y1:win_y2, right_win_x1:right_win_x2, :]]):
            ssss = np.sum(croped) / 255
            if ssss >= window_thresh_2 and ssss <= window_thresh_max_2:
                m = cv2.moments(croped, True)
                point[0] = m['m10'] / (m["m00"] + 1e-7)
                point[1] = m['m01'] / (m["m00"] + 1e-7)
                if point[0] == float("nan") or point[1] == float("nan") or point[0] == -float("nan") or point[
                    1] == -float("nan"):
                    point[0] = x_center_window[i]
                    point[1] = win_y1
                    point[2] = False
                else:
                    point[0] += win_x1
                    point[1] += win_y1
                    cv2.rectangle(warped_v, (win_x1, win_y1),
                                  (win_x2, win_y2), (50 + window * 21, i * 255, 0), 2)
            else:
                point[0] = x_center_window[i]
                point[1] = win_y1
                point[2] = False

            # if sum(map(lambda x: not x, points[np.clip(i-self._params.windows_count//3, 0, self._params.windows_count-1):i])) >= 2:
            #     point[2] = False

            # if self._prev_wr_points is not None:
            #     # # print("self._prev_wr_points[window]", self._prev_wr_points[window])
            #     # print("L", len(self._prev_wr_points[i]))
            #     if self._prev_wr_points[i][window][2] == True:
            #         if abs(self._prev_wr_points[i][window][0] - point[0]) > self._params.window_max_d:
            #             point[0] = (self._prev_wr_points[i][window][0] + point[0]) / 2
            # print("point_point_point_point", point)
            points[i][window] = point
            x_center_window[i] = points[i][window][0]
    # points = [[i[:2] for i in ps if i[2] == True] for ps in points]
    # print("left", points[0])
    # print("right", points[1])

    # self._prev_wr_points = points
    po = [np.array([i[:2] for i in ps if i[2] == True]) for ps in points]
    for i, p in enumerate(po):
        for pp in p:
            cv2.circle(warped_v, (int(
                pp[0]), int(pp[1])), 5, (50 + 21, i * 255, 0), 1)
    # po = np.array(po)
    # cv2.imshow("warped_v", warped_v)
    # print(p[0].shape)
    if len(po[0]) == 0:
        po[0] = np.array([[warped.shape[1]//2, 0]])
    # if len(po[1]) == 0:
    #     po[1] = np.array([[warped.shape[1]//2, 0]])
    one = int(po[0][:, 0].mean())
    one_d = warped.shape[1]//2+int(po[0][:, 0][po[0][:, 1].argmin()] - po[0][:, 0][po[0][:, 1].argmax()])

    if True:
        cv2.line(warped_v, (one, 0), (one, 300), 100, 2)
        # cv2.line(warped_v, (right, 0), (right, 300), 50, 2)
        # cv2.line(warped_v, ((right + left) // 2, 0), ((right + left) // 2, 300), 255, 3)
    cv2.imshow('warped1_v', warped_v)
    print(one_d, one)

    return int(one*KoL+one_d*KoD)


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
        self.res = int(SERVO_0 + self.kP * self.err + self.kD * (self.err - self.prev_error))
        self.prev_error = self.err
        return self.res