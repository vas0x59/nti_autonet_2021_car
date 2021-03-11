import numpy as np
# import numpy.typing as npt
import cv2
from dataclasses import dataclass
import math
from typing import *
import Utils
import time

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
        thresh_val: int = 200
        windows_count: int = 12
        window_width: int = 100
        window_thresh: int = 150
        window_thresh_max: int = 250
        window_max_d: float = 20
        proc_img_size: Tuple[int, int] = (400, 300)

    def __init__(self, region: np.ndarray,
                 camera_mat: np.ndarray,
                 # camera_distc: np.ndarray,
                 params: DetectorParams,
                 lane_sep: float,
                 # lane_width: float,
                 foot_print_to_camopt: Utils.Transformator):
        """

        :param region: Region of interest XY in footprint frame
        :param camera_mat: Camera matrix
        # :param camera_distc: Camera distortion coefficients
        # :param camera_position: Camera position in footprint frame
        # :param camera_roll: Roll angle of camera
        :param params: DetectorParams
        :param lane_sep: lane separation distance
        # :param lane_width: lane width
        """
        # self.lane_width = lane_width
        self.foot_print_to_camopt = foot_print_to_camopt

        self.lane_sep = lane_sep
        self._params: LaneDetector.DetectorParams = params
        # self.camera_roll: float = camera_roll
        # self.camera_position: np.ndarray = camera_position
        self.region: np.ndarray = region
        # print(self.region)
        self.camera_mat: np.ndarray = camera_mat
        # self.camera_distc: np.ndarray = camera_distc
        self._point_count = 30
        self._prev_road_center = None
        self._prev_wr_points = None
        self._p_transform = None
        self._p_transform_inv = None
        self._lk_params = dict( winSize  = (15,15),
                        maxLevel = 2,
                        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self._feature_params = dict( maxCorners = 75,
                       qualityLevel = 0.01,
                       minDistance = 5,
                       blockSize = 5 )
        self._prev_frame = None
        self._p0 = []
        self._pr_t = None
        # self._camera_rot_mat = np.array([
        #     [math.cos(self.camera_roll), 0, math.sin(self.camera_roll)],
        #     [0, 1, 0],
        #     [-math.sin(self.camera_roll), 0, math.cos(self.camera_roll)]
        # ])
        self._region_img = np.float32(self._sort_region_img(np.array(self._proj_from_ground_to_img(self.region))))
        # print("self._region_img", self._region_img)
        # image_out = np.zeros((720, 1280, 3), dtype="uint8")
        # # print(np.array(self._region_img, dtype="int32").reshape((-1, 1, 2)))
        # cv2.polylines(image_out,np.array(self._region_img, dtype="int32").reshape((-1, 1, 2)), True, (0, 100, 255), 4)
        # cv2.imshow("image_out", image_out)
        # cv2.waitKey(0)
        self._p_transform = cv2.getPerspectiveTransform(self._region_img.reshape(-1,1,2), np.float32([[0, self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], 0],
                                                                           [0, 0]
                                                                           ]))
        self._p_transform_inv = cv2.getPerspectiveTransform(np.float32([[0, self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], self._params.proc_img_size[1]],
                                                                           [self._params.proc_img_size[0], 0],
                                                                           [0, 0]
                                                                           ]), self._region_img.reshape(-1,1,2))
        self._cam_t_mat = None
        self._cam_t_mat_inv = None

        # self._region_n = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 1]))
        # self._region_p0 = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 0]))
        self._region_p0 = self.foot_print_to_camopt.transform_pnts_from_parent([self.region.mean(axis=0)])
        self._region_n  = self.foot_print_to_camopt.transform_r_pnts_from_parent([np.array([0, 0, 1])])
        # cv2.

        # self.filter_params
    # def _camera_transform(self):
    #     # self._cam_t_mat = Utils.rotate_3d_xyz() @ Utils.move_3d_xyz(self.camera_position[0], self.camera_position[1], self.camera_position[2])
    #     # self._cam_t_mat_inv = np.linalg.inv(self._cam_t_mat)
    # def _transform_from_footprint_to_camera(self, pnts):
    #     # return Utils.homogeneous_to_euclidean(self._cam_t_mat @ Utils.euclidean_to_homogeneous(pnts))
    # def _transform_from_camera_to_foot# print(self, pnts):
    #     # return Utils.homogeneous_to_euclidean(self._cam_t_mat_inv @ Utils.euclidean_to_homogeneous(pnts))

    def _proj_from_ground_to_img(self, pnts):
        # print("camopt", self.foot_print_to_camopt.transform_pnts_from_parent(pnts))
        # r = Utils.homogeneous_to_euclidean((self.camera_mat @ Utils.euclidean_to_homogeneous(self.tf.transform("footprint", "camera_optical", pnts)).transpose()).transpose())
        pppp = self.foot_print_to_camopt.transform_pnts_from_parent(pnts)
        ppr = cv2.projectPoints(pppp, np.array([[0, 0, 0]], dtype="float"), np.array([[0, 0, 0]], dtype="float"), np.array(self.camera_mat[:, :3], dtype="float"), None)[0].reshape(-1, 2)
        # print("image", ppr)
        return ppr

    def _sort_region_img(self, reg_img):
        max_x, max_y = np.max(reg_img[:, 0]), np.max(reg_img[:, 1])
        min_x, min_y = np.min(reg_img[:, 0]), np.min(reg_img[:, 1])
        # # print("qweqwe", max_x, max_y, min_x, min_y)
        # np.h
        # # print(reg_img - np.array([min_x, max_y]))
        # # print(np.sum((reg_img - np.array([min_x, max_y])) ** 2, axis=1) ** 0.5)
        # # print(np.sum((reg_img - np.array([min_x, min_y])) ** 2, axis=1) ** 0.5)
        lb = reg_img[np.argmin(np.sum((reg_img - np.array([min_x, max_y])) ** 2, axis=1) ** 0.5)]
        lt = reg_img[np.argmin(np.sum((reg_img - np.array([min_x, min_y])) ** 2, axis=1) ** 0.5)]
        rb = reg_img[np.argmin(np.sum((reg_img - np.array([max_x, max_y])) ** 2, axis=1) ** 0.5)]
        rt = reg_img[np.argmin(np.sum((reg_img - np.array([max_x, min_y])) ** 2, axis=1) ** 0.5)]
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

    # def _proj_from_warped_to_ground(self, pnts):
    #     img_pnts = self._proj_from_warped_to_img(pnts)
    #     ground_pnts = self._proj_from_img_to_ground(img_pnts)
    #     return ground_pnts
    def _proj_pnt_from_warped_to_ground(self, point):
        point = np.array([self._params.proc_img_size[0] - point[0], self._params.proc_img_size[1] - point[1]])
        point = np.array([point[1], point[0]])
        # print(point)
        # print("ORIG", point)
        point = point * [(np.max(self.region[:, 0]) - np.min(self.region[:, 0])) / self._params.proc_img_size[1],
                         (np.max(self.region[:, 1]) - np.min(self.region[:, 1])) / self._params.proc_img_size[0]]
        # point +=
        # point = np.array([-point[0], -point[1]])
        point += [np.min(self.region[:, 0]), np.min(self.region[:, 1])]
        # point = np.array([point[1], point[0]])
        # print("gorund", point)
        return point
    def _proj_from_warped_to_ground(self, pnts):
        # img_pnts = self._proj_from_warped_to_img(pnts)
        # ground_pnts = self._proj_from_img_to_ground(img_pnts)
        out = list()
        for p in pnts:
            out.append(self._proj_pnt_from_warped_to_ground(p))
        return np.array(out)

    def _detect_lanes_on_warped(self, warped: np.ndarray, warped_v) -> (list, list):
        # prev_road_center_on_warped = self._proj_from_ground_to_warped(self._prev_road_center)

        points = [[[None, None, False] for _ in range(self._params.windows_count)], [[None, None, False] for _ in range(self._params.windows_count)]]

        window_height = np.int(warped.shape[0] / self._params.windows_count)
        histogram = np.sum(warped[warped.shape[0] // 2:, :], axis=0)

        midpoint = histogram.shape[0] // 2
        IndWhitestColumnL = np.argmax(histogram[:midpoint])
        IndWhitestColumnR = np.argmax(histogram[midpoint:]) + midpoint

        x_center_window = [IndWhitestColumnL, IndWhitestColumnR]

        for i in range(2):
            for window in range(self._params.windows_count):
                win_y1 = int(warped.shape[0] - (window + 1) * window_height)
                win_y2 = int(warped.shape[0] - (window) * window_height)

                win_x1 = int(np.clip(x_center_window[i] - self._params.window_width/2, 0, warped.shape[1]))
                win_x2 = int(np.clip(x_center_window[i] + self._params.window_width/2, 0, warped.shape[1]))
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
                if ssss >= self._params.window_thresh and ssss <= self._params.window_thresh_max:
                    m = cv2.moments(croped, True)
                    point[0] = m['m10'] / (m["m00"] + 1e-7)
                    point[1] = m['m01'] / (m["m00"] + 1e-7)
                    if point[0] == float("nan") or point[1] == float("nan") or point[0] == -float("nan") or point[1] == -float("nan"):
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

                if self._prev_wr_points is not None:
                    # # print("self._prev_wr_points[window]", self._prev_wr_points[window])
                    # print("L", len(self._prev_wr_points[i]))
                    if self._prev_wr_points[i][window][2] == True:
                        if abs(self._prev_wr_points[i][window][0] - point[0]) > self._params.window_max_d:
                            point[0] = (self._prev_wr_points[i][window][0] + point[0]) / 2
                # print("point_point_point_point", point)
                points[i][window] = point
                x_center_window[i] = points[i][window][0]
        # points = [[i[:2] for i in ps if i[2] == True] for ps in points]
        # print("left", points[0])
        # print("right", points[1])


        self._prev_wr_points = points
        po = [[i[:2] for i in ps if i[2] == True] for ps in points]
        for i, p in enumerate(po):
            for pp in p:
                cv2.circle(warped_v, (int(
                    pp[0]), int(pp[1])), 5, (50 + 21, i * 255, 0), 1)
        cv2.imshow("warped_v", warped_v)
        return po

    def _proj_from_warped_to_img(self, points):
        # if self._p_transform_inv is None:
        # print("asdasdsadsad", points)
        points = np.array(points, dtype="float32")
        return cv2.perspectiveTransform(points.reshape(-1,1,2), self._p_transform_inv).reshape(-1,1,2)

    def _warp(self, frame):
        # warped =
        average = frame.mean(axis=0).mean(axis=0)
        warped = cv2.warpPerspective(
            frame, self._p_transform, (self._params.proc_img_size[0], self._params.proc_img_size[1]), flags=cv2.INTER_LINEAR, borderValue=(average))
        return warped

    def _get_lanes_img_points(self, frame: np.ndarray) -> (list, list):
        """
        Get img points of lanes
        :param frame:
        :return:
        """
        # self._p_transform = cv2.getPerspectiveTransform()
        warped = self._warp(frame)
        # self._get_xy_speed(warped)
        # # print(("warped"))
        # cv2.imshow("warped", warped)
        # # print()
        # cv2.waitKey(1)
        wr_th = self._img_filter(warped)
        # cv2.imshow("wr_th", wr_th)
        # # print()
        # cv2.waitKey(1)
        # print(wr_th.shape)
        wr_points = self._detect_lanes_on_warped(wr_th, warped.copy())
        # vv = cv2.point(warped.copy(), np.array(wr_points[0], dtype="int32").reshape((-1, 1, 2)), True, (0, 100, 255), 4)
        # vv = cv2.polylines(vv, np.array(wr_points[1], dtype="int32").reshape((-1, 1, 2)), True, (0, 255, 255), 4)
        # cv2.imshow("warped_p", vv)
        img_points = [self._proj_from_warped_to_img(p) if len(p) > 0 else [] for p in wr_points ]
        # warped = ()(th)

        return img_points

    # def _ray_plane_inte
    def _proj_pnt_from_img_to_ground(self, point: np.ndarray) -> np.ndarray:
        point = cv2.perspectiveTransform(np.array(point).reshape(-1,1,2), self._p_transform).reshape(-1,2)[0]
        point = np.array([self._params.proc_img_size[0]-point[0], self._params.proc_img_size[1]-point[1]])
        point = np.array([point[1], point[0]])
        # print(point)
        # print("ORIG", point)
        point = point * [(np.max(self.region[:, 0])-np.min(self.region[:, 0]))/self._params.proc_img_size[1],
                         (np.max(self.region[:, 1])-np.min(self.region[:, 1]))/self._params.proc_img_size[0]]
        # point +=
        # point = np.array([-point[0], -point[1]])
        point += [np.min(self.region[:, 0]), np.min(self.region[:, 1])]
        # point = np.array([point[1], point[0]])
        # print("gorund", point)
        return point
        # # print(self.camera_mat, np.apoint.T)
        # K = np.array(K)
        # R = np.eye(3)
        # t = np.array([[0], [1.], [0]])
        # P = self.camera_mat.dot(np.hstack((R, t)))
        #
        # point = Utils.euclidean_to_homogeneous(np.array(point))
        # # print(np.linalg.pinv(P), point.T)
        # ray_dir = Utils.homogeneous_to_euclidean(np.linalg.pinv(P).dot(point.T).T).T
        # ray_dir /= np.linalg.norm(ray_dir)
        # # ray_dir = ray_dir[]
        # ray_origin = np.array([0, 0, 0]).T
        # # print(ray_dir)
        #
        # # ray_origin =
        # # print((self._region_p0 - ray_origin).T)
        # # t: float = (self._region_p0 - ray_origin).T.dot(self._region_n.T) / (ray_dir.T.dot(self._region_n.T))
        # return self.foot_print_to_camopt.transform_pnts_to_parent([ray_dir * t + ray_origin])[0]

    def _proj_from_img_to_ground(self, points):
        out = list()
        for p in points:
            out.append(self._proj_pnt_from_img_to_ground([p]))
        return np.array(out)

    # def _get_lane_sep(self,):

    def _get_extra_lane(self, lane_fit: np.ndarray, lane_pnts: np.ndarray, offset: float) -> np.ndarray:
        # np.polyfit()
        p = np.poly1d(lane_fit)

        new_pnts = []
        pnts = np.array(list(map(lambda x: [x, p(x)], np.linspace(np.min(self.region[:, 0]), np.max(self.region[:, 0]), 20))))
        for p1, p2 in zip(pnts[0:], pnts[1:-1]):
            origin = (p1 + p2) / 2
            # if (offset > 0) == (-(p2[1] - p1[1]) > 0):
            normal = np.array([-(p2[1] - p1[1]), (p2[0] - p1[0])])
            # else:
            # normal = np.array([(p2[1] - p1[1]), -(p2[0] - p1[0])])
            normal /= np.linalg.norm(normal)
            new_pnts.append(origin+(normal*offset))

        new_pnts = np.array(new_pnts)

        new_fit = np.polyfit(new_pnts[:,0], new_pnts[:, 1], 2)
        return new_fit

    # def _get_xy_speed(self, warped) -> (float, float):
    #     frame = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    #     if self._prev_frame is None or self._p0 is None or len(self._p0) <= 0:
    #         self._prev_frame = frame
    #         self._p0 = cv2.goodFeaturesToTrack(self._prev_frame, mask=None, **self._feature_params)
    #     else:
    #         _p1, st, err = cv2.calcOpticalFlowPyrLK(self._prev_frame, frame, self._p0, None, **self._lk_params)
    #         good_new = _p1[st == 1]
    #         good_old = self._p0[st == 1]
    #         # em = self.estimate_motion(good_old, good_new)
    #         # self.drive_data.set("opt_x", em[0])
    #         # self.drive_data.set("opt_y", em[1])
    #         self._prev_frame = frame.copy()
    #         if self._p0 is None or len(self._p0) < self._feature_params["maxCorners"] * 0.1:
    #             self._p0 = cv2.goodFeaturesToTrack(frame, mask=None, **self._feature_params)
    #         else:
    #             self._p0 = good_new.reshape(-1, 1, 2)
    #         # if not self.p0 is None:
    #         #     print(len(self.p0))
    #
    #         # img_out =
    #         for i, (new, old) in enumerate(zip(good_new, good_old)):
    #             a, b = new.ravel()
    #             c, d = old.ravel()
    #             img_out = cv2.circle(warped.copy(), (c, d), 5, (255, 255, 0), -1)
    #             img_out = cv2.circle(img_out, (a, b), 5, (0, 255, 0), -1)
    #             cv2.imshow("opt", img_out)
    #     self._pr_t = time.time()

    def _get_real_road(self, img_points: (list, list)) -> (int, np.ndarray, np.ndarray, np.ndarray):
        # lines_pnts = [self._proj_img_points_to_ground(img_points[i]) for i in range(2)]
        # lines_fits = [np.polyfit(pnts) for pnts in lines_pnts]
        # print(img_points)
        if len(img_points[0]) > 0:
            left = np.array(self._proj_from_img_to_ground(np.array(img_points[0])[:, :2]))
        else:
            left = []
        if len(img_points[1]) > 0:
            right = np.array(self._proj_from_img_to_ground(np.array(img_points[1])[:, :2]))
        else:
            right = []
        # return 1, left, right, 0
        if len(right) >= 4 and len(left) >= 4:
            left_fit_xy  = np.polyfit(left[:, 0], left[:, 1], 2)
            left_p = np.poly1d(left_fit_xy)
            right_fit_xy = np.polyfit(right[:, 0], right[:, 1], 2)
            right_p = np.poly1d(right_fit_xy)
            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)

            road_xs = np.linspace(0, np.max(self.region[:, 0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 2, road_pnts, left_pnts, right_pnts
        elif len(right) >= 4 and len(left) < 4:
            right_fit_xy = np.polyfit(right[:, 0], right[:, 1], 2)
            right_p = np.poly1d(right_fit_xy)
            left_fit_xy = self._get_extra_lane(right_fit_xy, right, self.lane_sep)
            left_p = np.poly1d(left_fit_xy)

            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)

            road_xs = np.linspace(0, np.max(self.region[:, 0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 1, road_pnts, left_pnts, right_pnts
        elif len(left) >= 4 and len(right) < 4:
            left_fit_xy = np.polyfit(left[:, 0], left[:, 1], 2)
            left_p = np.poly1d(left_fit_xy)
            right_fit_xy = self._get_extra_lane(left_fit_xy, left, -self.lane_sep)
            right_p = np.poly1d(right_fit_xy)

            road_fit_xy = (left_fit_xy + right_fit_xy) / 2.0
            road_p = np.poly1d(road_fit_xy)


            road_xs = np.linspace(0, np.max(self.region[:, 0]), self._point_count)
            road_pnts = np.array([[x, road_p(x)] for x in road_xs])
            left_pnts = np.array([[x, left_p(x)] for x in road_xs])
            right_pnts = np.array([[x, right_p(x)] for x in road_xs])

            return 1, road_pnts, left_pnts, right_pnts
        else:
            return 0, None, None, None
    def _viz_itog(self, frame, road_pnts, c):
        road_pnts_3d = np.zeros((road_pnts.shape[0], road_pnts.shape[1]+1))
        road_pnts_3d[:, :2] = road_pnts
        road_pnts_3d[:, 2] = 0
        image_pts = np.array(self._proj_from_ground_to_img(road_pnts_3d), dtype="int32").reshape((-1, 1, 2))
        # print("image_pts", image_pts)
        return cv2.polylines(frame,image_pts, True, c, 4)

    def run(self, frame: np.ndarray):
        ts = time.time()
        img_points = self._get_lanes_img_points(frame)
        print("Detect", (time.time()-ts)*1000)
        ts = time.time()
        ret, road_pnts, left_pnts, right_pnts = self._get_real_road(img_points)
        print("_get_real_road", (time.time() - ts) * 1000)
        if not ret:
            return 0, None, None, None
        # print("ROAD ret", ret)
        # print("PROC LANe", (time.time()-ts)*1000)
        v = frame.copy()
        if len(road_pnts) > 0:
            v = self._viz_itog(v, road_pnts, (0, 150, 250))
        if len(right_pnts) > 0:
            v = self._viz_itog(v, right_pnts, (150, 255, 0))
        if len(left_pnts) > 0:
            v = self._viz_itog(v, left_pnts, (255, 0, 0))
        # v = self._viz_itog(v, right_pnts, (0, 255, 150))
        cv2.imshow("LANE_VIZ", v)
        return ret, road_pnts, left_pnts, right_pnts



