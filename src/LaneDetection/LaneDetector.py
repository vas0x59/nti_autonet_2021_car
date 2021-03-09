import numpy as np
# import numpy.typing as npt
import cv2
from dataclasses import dataclass
import math
from typing import *
import Utils

class LaneDetector:
    """
    Input:
        camera matrix
        dist coeffs

        camera pose

        region pose XY


    """
    @dataclass
    class DetectorParams:
        blur_ksize: int = 5
        thresh_val: int = 10
        windows_count: int = 10
        window_width: int = 120
        window_thresh: int = 20
        window_max_d: float = 100
        proc_img_size: Tuple[int, int] = (None, None)

    def __init__(self, region: np.ndarray,
                 camera_mat: np.ndarray,
                 # camera_distc: np.ndarray,
                 params: DetectorParams,
                 lane_sep: float,
                 lane_width: float,
                 tf: Utils.TF):
        """

        :param region: Region of interest XY in footprint frame
        :param camera_mat: Camera matrix
        # :param camera_distc: Camera distortion coefficients
        :param camera_position: Camera position in footprint frame
        :param camera_roll: Roll angle of camera
        :param params: DetectorParams
        :param lane_sep: lane separation distance
        :param lane_width: lane width
        """
        self.lane_width = lane_width
        self.lane_sep = lane_sep
        self._params: LaneDetector.DetectorParams = params
        # self.camera_roll: float = camera_roll
        # self.camera_position: np.ndarray = camera_position
        self.region: np.ndarray = region
        self.camera_mat: np.ndarray = camera_mat
        # self.camera_distc: np.ndarray = camera_distc
        self._point_count = 30
        self._prev_road_center = None
        self._prev_wr_points = None
        self._p_transform = None
        self._p_transform_inv = None
        # self._camera_rot_mat = np.array([
        #     [math.cos(self.camera_roll), 0, math.sin(self.camera_roll)],
        #     [0, 1, 0],
        #     [-math.sin(self.camera_roll), 0, math.cos(self.camera_roll)]
        # ])
        self._region_img = self._sort_region_img(self._proj_from_ground_to_img(self.region))
        self._p_transform = cv2.getPerspectiveTransform(self._region_img, [[0, self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], 0],
                                                                           [self._params.proc_img_size[0], self._params.proc_img_size[1]]
                                                                           ])
        self._p_transform_inv = np.linalg.inv(self._p_transform)
        self._cam_t_mat = None
        self._cam_t_mat_inv = None
        self.tf = tf
        # self._region_n = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 1]))
        # self._region_p0 = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 0]))
        self._region_p0 = self.tf.transform("footprint", "camera_optical", [np.append(self.region.mean(axis=0), 0)])
        self._region_n  = self.tf.transform_r("footprint", "camera_optical", [np.array([0, 0, 1])])
        # self.filter_params
    # def _camera_transform(self):
    #     # self._cam_t_mat = Utils.rotate_3d_xyz() @ Utils.move_3d_xyz(self.camera_position[0], self.camera_position[1], self.camera_position[2])
    #     # self._cam_t_mat_inv = np.linalg.inv(self._cam_t_mat)
    # def _transform_from_footprint_to_camera(self, pnts):
    #     # return Utils.homogeneous_to_euclidean(self._cam_t_mat @ Utils.euclidean_to_homogeneous(pnts))
    # def _transform_from_camera_to_footprint(self, pnts):
    #     # return Utils.homogeneous_to_euclidean(self._cam_t_mat_inv @ Utils.euclidean_to_homogeneous(pnts))

    def _proj_from_ground_to_img(self, pnts):
        return self.camera_mat @ self.tf.transform("footprint", "camera_optical", pnts)

    def _sort_region_img(self, reg_img):
        max_x, max_y = np.max(reg_img[:, 0]), np.max(reg_img[:, 1])
        min_x, min_y = np.min(reg_img[:, 0]), np.min(reg_img[:, 1])
        # np.h
        lb = reg_img[np.argmin(np.sum((reg_img - np.array([min_x, max_y])) ** 2) ** 0.5)]
        lt = reg_img[np.argmin(np.sum((reg_img - np.array([min_x, min_y])) ** 2) ** 0.5)]
        rb = reg_img[np.argmin(np.sum((reg_img - np.array([max_x, max_y])) ** 2) ** 0.5)]
        rt = reg_img[np.argmin(np.sum((reg_img - np.array([max_x, min_y])) ** 2) ** 0.5)]
        return np.array([lb, rb, rt, lt])

    def _img_filter(self, img: np.ndarray) -> np.ndarray:
        out = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        out = np.clip(out, self._params.thresh_val, 255)
        out = cv2.medianBlur(out, self._params.blur_ksize)
        out = cv2.adaptiveThreshold(out, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 5, 2)
        out = cv2.dilate(out, np.ones((2, 2), dtype="uint8"))
        return out

    # def _check_point(self, p):
    #     pass

    def _proj_from_warped_to_ground(self, pnts):
        img_pnts = self._proj_from_warped_to_img(pnts)
        ground_pnts = self._proj_from_img_to_ground(img_pnts)
        return ground_pnts

    def _detect_lanes_on_warped(self, warped: np.ndarray) -> (list, list):
        # prev_road_center_on_warped = self._proj_from_ground_to_warped(self._prev_road_center)

        points = ([[None, None, False] for _ in range(self._params.windows_count)], [[None, None, False] for _ in range(self._params.windows_count)])

        window_height = np.int(warped.shape[0] / self._params.windows_count)

        x_center_window = [0, 0]

        for i in range(2):
            for window in range(self._params.windows_count):
                win_y1 = warped.shape[0] - (window + 1) * window_height
                win_y2 = warped.shape[0] - (window) * window_height

                win_x1 = np.clip(x_center_window[i] - self.lane_sep/2, 0, warped.shape[1])
                win_x2 = np.clip(x_center_window[i] + self.lane_sep/2, 0, warped.shape[1])
                # right_win_x1 = np.clip(XCenterRightWindow - self.lane_sep/2, 0, warped.shape[1])
                # right_win_x2 = np.clip(XCenterRightWindow + self.lane_sep/2, 0, warped.shape[1])

                point = [None, None, True]
                croped = warped[win_y1:win_y2, win_x1:win_x2]

                # for i, croped in enumerate([, warped[win_y1:win_y2, right_win_x1:right_win_x2, :]]):
                if np.sum(croped) / 255 >= self._params.window_thresh:
                    m = cv2.moments(croped, True)
                    point = [m['m10'] / (m["m00"] + 1e-7), m['m01'] / (m["m00"] + 1e-7)]
                    if point[0] == float("nan") or point[1] == float("nan") or point[0] == -float("nan") or point[1] == -float("nan"):
                        point[0] = win_x1 + (win_x2 - win_x1)
                        point[1] = win_y1 + (win_y2 - win_y1)
                    else:
                        point[0] += win_x1
                        point[1] += win_y1
                else:
                    point[0] = win_x1 + (win_x2 - win_x1)
                    point[1] = win_y1 + (win_y2 - win_y1)
                    point[2] = False
                if self._prev_wr_points is not None:
                    if abs(self._prev_wr_points[window][0] - point[0]) > self._params.window_max_d:
                        point[0] = (self._prev_wr_points[window][0] + point[0] + self._prev_wr_points[0 if (window - 1) < 0 else (window - 1)][0]) / 3
                points[i][window] = point
                x_center_window[i] = points[i][window][0]

        self._prev_wr_points = points
        return points

    def _proj_from_warped_to_img(self, points):
        # if self._p_transform_inv is None:

        return self._p_transform_inv @ np.array(points)

    def _warp(self, frame):
        # warped =
        average = frame.mean(axis=0).mean(axis=0)
        warped = cv2.warpPerspective(
            frame, self._p_transform, (self._params.proc_img_size[1], self._params.proc_img_size[0]), flags=cv2.INTER_LINEAR, borderValue=(average))
        return warped

    def _get_lanes_img_points(self, frame: np.ndarray) -> (list, list):
        """
        Get img points of lanes
        :param frame:
        :return:
        """
        # self._p_transform = cv2.getPerspectiveTransform()
        warped = self._warp(frame)
        wr_th = self._img_filter(warped)
        wr_points = self._detect_lanes_on_warped(wr_th)
        img_points = [self._proj_from_warped_to_img(p) for p in wr_points]
        # warped = ()(th)

        return img_points

    # def _ray_plane_inte
    def _proj_pnt_from_img_to_ground(self, point: np.ndarray) -> np.ndarray:
        ray_dir = np.linalg.inv(self.camera_mat) @ point
        ray_origin = np.array([0, 0, 0])
        # ray_origin =
        t: float = (self._region_p0 - ray_origin).dot(self._region_n) / (ray_dir.dot(self._region_n))
        return self.tf.transform("camera_optical", "footprint", [ray_dir * t + ray_origin])[0]

    def _proj_from_img_to_ground(self, points):
        out = list()
        for p in points:
            out.append(self._proj_pnt_from_img_to_ground(p))
        return np.array(out)

    # def _get_lane_sep(self,):

    def _get_extra_lane(self, lane_fit: np.ndarray, lane_pnts: np.ndarray, offset: float) -> np.ndarray:
        # np.polyfit()
        p = np.poly1d(lane_fit)

        new_pnts = []
        pnts = np.array(list(map(lambda x: [x, p(x)], np.linspace(np.min(self.region[:, 0]), np.max(self.region[:, 0], 20)))))
        for p1, p2 in zip(pnts[0:], pnts[1:-1]):
            origin = (p1 + p2) / 2
            if (offset > 0) == (-(p2[1] - p1[1]) > 0):
                normal = np.array([-(p2[1] - p1[1]), (p2[0] - p1[0])])
            else:
                normal = np.array([(p2[1] - p1[1]), -(p2[0] - p1[0])])
            normal /= np.linalg.norm(normal)
            new_pnts.append(origin+(normal*offset))

        new_pnts = np.array(new_pnts)

        new_fit = np.polyfit(new_pnts[:,0], new_pnts[:, 1], 2)
        return new_fit

    def _get_real_road(self, img_points: (list, list)) -> (int, np.ndarray, np.ndarray, np.ndarray):
        # lines_pnts = [self._proj_img_points_to_ground(img_points[i]) for i in range(2)]
        # lines_fits = [np.polyfit(pnts) for pnts in lines_pnts]
        left = np.array(self._proj_from_img_to_ground(img_points[0]))
        right = np.array(self._proj_from_img_to_ground(img_points[1]))

        if len(right) >= 3 and len(left) >= 3:
            left_fit_xy  = np.polyfit(left[:, 0], left[:, 1], 2)
            left_p = np.poly1d(left_fit_xy)
            right_fit_xy = np.polyfit(right[:, 0], right[:, 1], 2)
            right_p = np.poly1d(right_fit_xy)
            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)

            road_xs = np.linspace(0, np.max(self.region[:0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 2, road_pnts, left_pnts, right_pnts
        elif len(right) >= 3 and len(left) < 3:
            right_fit_xy = np.polyfit(right[:, 0], right[:, 1], 2)
            right_p = np.poly1d(right_fit_xy)
            left_fit_xy = self._get_extra_lane(right_fit_xy, right, self.lane_width)
            left_p = np.poly1d(left_fit_xy)

            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)

            road_xs = np.linspace(0, np.max(self.region[:0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 1, road_pnts, left_pnts, right_pnts
        elif len(left) >= 3 and len(right) < 3:
            left_fit_xy = np.polyfit(left[:, 0], left[:, 1], 2)
            left_p = np.poly1d(left_fit_xy)
            right_fit_xy = self._get_extra_lane(left_fit_xy, left, -self.lane_width)
            right_p = np.poly1d(right_fit_xy)

            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)


            road_xs = np.linspace(0, np.max(self.region[:0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 1, road_pnts, left_pnts, right_pnts
        else:
            return 0, None, None, None

    def run(self, frame: np.ndarray):
        img_points = self._get_lanes_img_points(frame)
        ret, road_pnts, left_pnts, right_pnts = self._get_real_road(img_points)
        if not ret:
            return
        return road_pnts, left_pnts, right_pnts

