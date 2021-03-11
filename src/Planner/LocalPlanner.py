import numpy as np
# import numpy.typing as npt
import cv2
from dataclasses import dataclass
import math
from typing import *
import Utils
import time

class LocalPlanner:
    """
    Input:
        camera matrix
        dist coeffs

        camera pose

        region pose XY


    """
    @dataclass
    class PlannerParams:
        # pass
        base_l: float = 0.162
        # blur_ksize: int = 5
        # thresh_val: int = 200
        # windows_count: int = 12
        # window_width: int = 100
        # window_thresh: int = 150
        # window_max_d: float = 20
        max_servo: int = 35
        proc_img_size: Tuple[int, int] = (400, 300)
        min_x_foot_print: float = 0.3
        std_speed: float = 0.2
        d_cost_k: float =  0.2
        l_cost_k: float = 1
        x_cost_k: float = 0.09
        da_cost_k: float = 0.1
        o_cost_k: float = 0.12
        # d_cost_k: float = 0.2
        # l_cost_k: float = 0.5
        # x_cost_k: float = 0.09
        # da_cost_k: float = 0.1
        # o_cost_k: float = 0.12
    @dataclass
    class PlannerState:
        pass
    def __init__(self,
                 region: np.ndarray,
                 camera_mat: np.ndarray,
                 # camera_distc: np.ndarray,
                 params: PlannerParams,
                 lane_sep: float,
                 # lane_width: float,
                 foot_print_to_camopt: Utils.Transformator,
                 foot_print_to_back_wheels: Utils.Transformator):
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
        self.foot_print_to_back_wheels = foot_print_to_back_wheels
        # self.lane_width = lane_width
        self.foot_print_to_camopt = foot_print_to_camopt
        self.lane_sep = lane_sep
        self._params: LocalPlanner.PlannerParams = params
        # self.camera_roll: float = camera_roll
        # self.camera_position: np.ndarray = camera_position
        self.region: np.ndarray = region
        # print(self.region)
        self.camera_mat: np.ndarray = camera_mat
        # self.camera_distc: np.ndarray = camera_distc
        # self._point_count = 30
        # self._prev_road_center = None
        # self._prev_wr_points = None
        self._p_transform = None
        self._p_transform_inv = None
        # self._lk_params = dict( winSize  = (15,15),
        #                 maxLevel = 2,
        #                 criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        # self._feature_params = dict( maxCorners = 75,
        #                qualityLevel = 0.01,
        #                minDistance = 5,
        #                blockSize = 5 )
        # self._prev_frame = None
        # self._p0 = []
        # self._pr_t = None
        # self._camera_rot_mat = np.array([
        #     [math.cos(self.camera_roll), 0, math.sin(self.camera_roll)],
        #     [0, 1, 0],
        #     [-math.sin(self.camera_roll), 0, math.cos(self.camera_roll)]
        # ])
        self._prev_ang = 0
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
        # self._cam_t_mat = None
        # self._cam_t_mat_inv = None

        # self._region_n = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 1]))
        # self._region_p0 = self.tf.transform("footprint", "camera_optical", np.array([0, 0, 0]))
        self._region_p0 = self.foot_print_to_camopt.transform_pnts_from_parent([self.region.mean(axis=0)])
        self._region_n  = self.foot_print_to_camopt.transform_r_pnts_from_parent([np.array([0, 0, 1])])

        self._trajs_back = []
        # self._poly_back = []
        for i in np.linspace(-self._params.max_servo, self._params.max_servo, 90):
            serv_ang = math.radians(i)
            w = math.tan(serv_ang)*self._params.std_speed/self._params.base_l
            traj = self._traj_gen(self._params.std_speed, w, 0.06, 2.2)[:, 1:3]
            traj_3d = np.zeros((traj.shape[0], traj.shape[1] + 1))
            traj_3d[:, :2] = traj
            traj_3d[:, 2] = 0
            # traj_n = self.foot_print_to_back_wheels.transform_pnts_to_parent(traj_3d)[:, :2]
            poly = np.polyfit(traj[:, 0], traj[:, 1], 2)
            self._trajs_back.append([serv_ang, traj, poly])
            # v = self._viz_itog(v, traj_n, (0, 255, 0))
            # self._poly_back.append/()
        # cv2.
        self._trajs_back = self._trajs_back
        # s
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

    def _traj_gen(self, vx, w, dt, tmax):
        ts = time.time()
        points = []
        q = np.array([0, 0, 0], dtype="float")

        t = 0
        while t < tmax:
            #         print(np.array([[vx, w]]).T)
            qq = (np.array([[0, 1],
                            [math.cos(q[0]), 0],
                            [math.sin(q[0]), 0]]) @ np.array([[vx, w]]).T).T[0] * dt
            #         print(qq, q)
            q = qq + q
            points.append(q)
            t += dt
        #     w_vars = np.linspace()
        # print("RUN T", time.time() - ts)
        return np.array(points)
    def _viz_itog(self, frame, road_pnts, c):
        road_pnts_3d = np.zeros((road_pnts.shape[0], road_pnts.shape[1]+1))
        road_pnts_3d[:, :2] = road_pnts
        road_pnts_3d[:, 2] = 0
        image_pts = np.array(self._proj_from_ground_to_img(road_pnts_3d), dtype="int32").reshape((-1, 1, 2))
        # print("image_pts", image_pts)
        return cv2.polylines(frame,image_pts, True, c, 4)

    def _cost_dist_b_dist_x_angle_d_dl_to_road(self, traj_poly, road_points, road_poly) -> (float, float, float, float):
        d_p = np.polysub(road_poly, traj_poly)
        # np.polyval(d_p, )
        xss = np.linspace(self._params.min_x_foot_print, np.max(road_points[:, 0]), 20)
        dss = np.abs(np.polyval(traj_poly, xss))
        i = np.argmin(dss)
        d = dss[i]
        x = xss[i]
        y = np.polyval(traj_poly, x)
        yr = np.polyval(road_poly, x)
        a1 = math.atan2((x + 0.001) - x, np.polyval(traj_poly, x + 0.001) - y)
        a2 = math.atan2((x + 0.001) - x, np.polyval(road_poly, x + 0.001) - yr)
        da = abs(a1-a2)
        da = math.asin(math.sin(da))
        dl = abs(np.polyval(traj_poly, self._params.min_x_foot_print+0.1)-np.polyval(road_poly, self._params.min_x_foot_print+0.1))
        return d, x, da, dl

        # pass


    def _cost_border_intersect(self, traj, left_lane_points, right_lane_points):
        pass
        # pass
    def _cost_oscillation(self, new_w, prev_w):
        return abs(new_w-prev_w)

    def _find_1(self, road_points):
        road_points_3d = np.zeros((road_points.shape[0], road_points.shape[1]+1))
        road_points_3d[:, :2] = road_points
        road_points_b = self.foot_print_to_back_wheels.transform_pnts_from_parent(road_points_3d)[:, :2]
        road_poly = np.polyfit(road_points_b[:, 0], road_points_b[:, 1], 3)
        costs = np.zeros((len(self._trajs_back)))
        for i, traj_ in enumerate(self._trajs_back):
            s_ang, traj, poly = traj_
            d, x, da, dl = self._cost_dist_b_dist_x_angle_d_dl_to_road(poly, road_points, road_poly)
            cost = self._params.d_cost_k*d + self._params.x_cost_k*x + \
                   self._params.da_cost_k*da + self._params.o_cost_k*self._cost_oscillation(s_ang, self._prev_ang) + self._params.l_cost_k*dl
            costs[i] = cost
        imin = np.argmin(costs)
        # print("costs", costs)
        self._prev_ang = self._trajs_back[imin][0]
        return self._trajs_back[imin][0], self._trajs_back[imin][1]



    def run(self, frame: np.ndarray, road_points: np.ndarray, left_lane_points: np.ndarray, right_lane_points: np.ndarray, vx: float, vy: float, std_vx: float)->float:
        # pass
        # serv_ang = math.pi/3
        ts = time.time()
        v = self._viz_itog(frame.copy(), road_points, (0, 0, 255))


        # ts = time.time()
        # img_points = self._get_lanes_img_points(frame)
        # ret, road_pnts, left_pnts, right_pnts = self._get_real_road(img_points)
        # if not ret:
        #     return 0, None, None, None
        # print("ROAD ret", ret)
        # print("PROC LANe", (time.time()-ts)*1000)
        # v = frame.copy()
        # if len(road_pnts) > 0:
        #     v = self._viz_itog(v, road_pnts, (0, 150, 250))
        # if len(right_pnts) > 0:
        #     v = self._viz_itog(v, right_pnts, (150, 255, 0))
        # if len(left_pnts) > 0:
        #     v = self._viz_itog(v, left_pnts, (255, 0, 0))
        # # v = self._viz_itog(v, right_pnts, (0, 255, 150))

        # return ret, road_pnts, left_pnts, right_pnts
        a, tr = self._find_1(road_points)
        # v = self._viz_itog(v, tr, (0, 255, 0))
        road_pnts_3d = np.zeros((tr.shape[0], tr.shape[1] + 1))
        road_pnts_3d[:, :2] = tr
        road_pnts_3d[:, 2] = 0
        road_pnts_3d = self.foot_print_to_back_wheels.transform_pnts_to_parent(road_pnts_3d)
        image_pts = np.array(self._proj_from_ground_to_img(road_pnts_3d), dtype="int32").reshape((-1, 1, 2))
        # print("image_pts", image_pts)
        v = cv2.polylines(v, image_pts, True, (0, 255, 0), 4)
        cv2.imshow("PLANER_VIZ", v)
        # print("GEN T", time.time() - ts)
        return a

