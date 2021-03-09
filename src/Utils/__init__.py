import time
import threading
from functools import wraps
from dataclasses import dataclass
import numpy as np
import math


def delay(t=0.):
    """
    Decorator delaying the execution of a function for a while.
    """
    def wrap(f):
        @wraps(f)
        def delayed(*args, **kwargs):
            timer = threading.Timer(t, f, args=args, kwargs=kwargs)
            timer.start()
        return delayed
    return wrap


class Rate:
    def __init__(self, freq: float):
        self._freq = freq
        self._prev_t = None

    def sleep(self):
        now = time.time()
        if self._prev_t is None:
            self._prev_t = now
            return 0
        if (now - self._prev_t) < (1/self._freq):
            time.sleep((1/self._freq) - (now - self._prev_t))

        pt = self._prev_t
        self._prev_t = now
        return 1/(now - pt)


# @dataclass
# class CameraIntrinsic:

def euclidean_to_homogeneous(p_e: np.ndarray) -> np.ndarray:
    if len(p_e.shape) > 1:
        if p_e.shape[1] == 1:
            return np.expand_dims(np.append(p_e, np.array([1])), 1)
        else:
            return np.array([np.append(p_e[0], 1)]).transpose()
    else:
        return np.append(p_e, 1)


def homogeneous_to_euclidean(p_h: np.ndarray) -> np.ndarray:
    return np.array(p_h[:-1]/p_h[-1][0])


def get_rotation_matrix_2(angle: float) -> np.array:
    b = [[] for _ in range(2)]
    b[0] = [math.cos(angle), -math.sin(angle)]
    b[1] = [math.sin(angle), math.cos(angle)]
    return np.array(b)


def get_rotation_matrix_3_x(angle: float) -> np.array:
    b = np.zeros((3, 3))
    b[0, 0] = 1
    b[1:3, 1:3] = get_rotation_matrix_2(angle)
    return b
def get_rotation_matrix_3_y(angle: float) -> np.array:
    b = np.zeros((3, 3))
    b[1, 1] = 1
    b[::-2, ::-2] = get_rotation_matrix_2(angle)
    return b
def get_rotation_matrix_3_z(angle: float) -> np.array:
    b = np.zeros((3, 3))
    b[2, 2] = 1
    b[0:2, 0:2] = get_rotation_matrix_2(angle)
    return b
def get_rotation_matrix_3_xyz(x: float, y:  float, z: float) -> np.array:
    return np.dot(np.dot(get_rotation_matrix_3_x(x), get_rotation_matrix_3_y(y)), get_rotation_matrix_3_z(z))


def move_3d_xyz(x: float, y: float, z: float) -> np.array:
    a = np.zeros((4, 4))
    a[0, 3] = x
    a[1, 3] = y
    a[2, 3] = z
    a[list(range(0, 4)), list(range(0, 4))] = 1
    return a


def rotate_3d_xyz(x: float, y:  float, z: float):
    a = np.zeros((4, 4))
    a[0:3, 0:3] = get_rotation_matrix_3_xyz(x, y, z)
    a[3, 3] = 1
    return a


class TF:
    # @dataclass()
    class Transform:

        tr_mat = None
        def __init__(self, tr_mat):
            tr_mat = tr_mat
            # pass

        @classmethod
        def from_xyzypr(cls, x: float,
            y: float,
            z: float,
            yaw: float,
            pitch: float,
            roll: float):
            tr_mat = np.linalg.inv(rotate_3d_xyz(yaw, pitch, roll) @ move_3d_xyz(x, y, z))
            return cls(tr_mat)

        @classmethod
        def inv(cls, o):
            return cls(np.linalg.inv(o.tr_mat))

        def transform(self, pnts):
            pnts = np.array(pnts)
            h_pnts = euclidean_to_homogeneous(pnts)
            h_pnts_tr = h_pnts @ self.tr_mat
            return homogeneous_to_euclidean(h_pnts_tr)
        def transform_rot(self, pnts):
            pnts = np.array(pnts)
            h_pnts = euclidean_to_homogeneous(pnts)
            t = np.array(self.tr_mat).copy()
            t[0:2, 3] = 0
            h_pnts_tr = h_pnts @ t
            return homogeneous_to_euclidean(h_pnts_tr)
        # @classmethod
        # def from_tr_mat(cls, tr_mat):
        #     tr_ma

    def __init__(self):
        # pass
        self.tree = dict()
    def _add_tr(self, parent, child, tr:Transform):
        p = self.tree.get(parent, None)
        if p is None:
            self.tree[parent] = dict()
        self.tree[parent][child]  = tr
    def set_transform(self, parent, child, tr:Transform):
        self._add_tr(parent, child, tr)
        tr_inv = self.Transform.inv(tr)
        self._add_tr(child, parent, tr_inv)
    def get_transform(self, parent, child) -> Transform:
        p = self.tree.get(parent, None)
        if p is None:
            return None
        c = p.get(child, None)
        return c

    def transform(self, parent, child, points):
        points = np.array(points)
        tr = self.get_transform(parent, child)
        if tr is not None:
            return tr.transform(points)
        else:
            return None

    def transform_r(self, parent, child, points):
        points = np.array(points)
        tr = self.get_transform(parent, child)
        if tr is not None:
            return tr.transform_rot(points)
        else:
            return None
    # def _find_transform

