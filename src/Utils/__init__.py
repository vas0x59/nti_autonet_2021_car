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

def euclidean_to_homogeneous(p_e: np.ndarray) -> np.ndarray:
    p_e = np.array(p_e)
    if len(p_e.shape) > 1:
        a = np.zeros((p_e.shape[0], p_e.shape[1]+1))
        a[:, :p_e.shape[1]] = p_e
        a[:, p_e.shape[1]] = 1
        # if p_e.shape[1] == 1:
        #     return np.expand_dims(np.append(p_e, np.array([1])), 1)
        # else:
        #     return np.array([np.append(p_e[0], 1)]).transpose()
        return a
    else:
        return np.append(p_e, 1)


def homogeneous_to_euclidean(p_h: np.ndarray) -> np.ndarray:
    p_h = np.array(p_h)
    if len(p_h.shape) > 1:
        return np.array(p_h[:, :-1] / p_h[:, -1][0])
    else:
        return np.array(p_h[:-1]/p_h[-1][0])


def get_rotation_matrix_2(angle: float) -> np.array:
    b = [[] for _ in range(2)]
    b[0] = [math.cos(angle), math.sin(angle)]
    b[1] = [-math.sin(angle), math.cos(angle)]
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

class Transformator:

    def __init__(self, parent, child):
        self.child = child
        self.basis = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype="float64")
        self.translation = np.array([0, 0, 0], dtype="float64")
        self.parent = parent
#         self.name = None
    def _tr_xyz(self, x, y, z):
        self.translation += np.array([x, y, z])
    def _tr_rot(self, x, y, z):
        self.basis = get_rotation_matrix_3_xyz(x, y, z) @ self.basis
    def set(self, x, y, z, rx, ry, rz):
        self.basis = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype="float64")
        self.translation = np.array([0, 0, 0], dtype="float64")
#         self.parent = None
        self._tr_xyz(x, y, z)
        self._tr_rot(rx, ry, rz)
#         self.parent = p
    def transform_pnts_to_parent(self, pnt):
        ppnt = np.array(pnt)
        for i in range(ppnt.shape[0]):
            p = ppnt[i]
            p_par = (np.linalg.inv(self.basis) @ p.T).T
            p_par += self.translation
            ppnt[i] = p_par
        return ppnt
    def transform_pnts_from_parent(self, pnt):
#         ppnt = np.array(pnt)
# #         if ppnt.shape
#         p_s = ppnt - self.translation
#         p_s = (self.basis @ p_s.transpose()).transpose()
#         return p_s
        ppnt = np.array(pnt)
        for i in range(ppnt.shape[0]):
            p = ppnt[i]
            p_par = p - self.translation
            p_par = (self.basis @ p_par.T).T
            ppnt[i] = p_par
        return ppnt
    def transform_r_pnts_to_parent(self, pnt):
        ppnt = np.array(pnt)
        for i in range(ppnt.shape[0]):
            p = ppnt[i]
            p_par = (np.linalg.inv(self.basis) @ p.T).T
#             p_par += self.translation
            ppnt[i] = p_par
        return ppnt
    def transform_r_pnts_from_parent(self, pnt):
#         ppnt = np.array(pnt)
# #         if ppnt.shape
#         p_s = ppnt - self.translation
#         p_s = (self.basis @ p_s.transpose()).transpose()
#         return p_s
        ppnt = np.array(pnt)
        for i in range(ppnt.shape[0]):
            p = ppnt[i]
            p_par = p
            p_par = (self.basis @ p_par.T).T
            ppnt[i] = p_par
        return ppnt
    def get_tr_mat_from_parent(self):
        a = move_3d_xyz(-self.translation[0], -self.translation[1], -self.translation[2])
        b = np.zeros((4, 4))
        b[3, 3] = 1
        b[0:3, 0:3] = self.basis
        a = b @ a
#         a[0:3, 3] = -self.translation
#         a[0:3, 0:3] = self.basis
#         a[3, 3] = 1
#         a[1, 3] = y
#         a[2, 3] = z
        print(a, self.basis)
        return a
    def get_tr_mat_to_parent(self):
        a = np.zeros((4, 4))
        a[0:3, 0:3] = np.linalg.inv(self.basis)
        a[0:3, 3] = self.translation
        a[3, 3] = 1
#         a[1, 3] = y
#         a[2, 3] = z
        return a
    @classmethod
    def concat(cls, old_med, med_new):
        if old_med.child == med_new.parent:
            old_new = cls(old_med.parent, med_new.child)
            trans = old_med.translation + old_med.basis @ med_new.translation
            old_new.translation = trans
            old_new.basis = med_new.basis @ old_med.basis
            return old_new
        else:
            return None