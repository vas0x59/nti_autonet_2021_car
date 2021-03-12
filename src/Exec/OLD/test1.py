# from Hardware import Hardware
from Utils.VideoRecorder import VideoRecorder
import Utils.colors
import traceback
import os
from LaneDetection.LaneDetector import LaneDetector
from LaneDetection.OpticalFlow import OpticalFlow
from LaneDetection.SpeedPredictor import SpeedPredictor
import Utils.PID
import math
import threading
d = True
from Exec.OLD.func import *

from ObjectDetection import OBJDetection
from Planner.LocalPlanner import LocalPlanner


class Main:
    def __init__(self, PROJECT_FOLDER, hard):
        self.SERVO_0 = SERVO_0
        self.MOTOR_STD = speed
        self.d = d
        self.last = 0
        self.angle_pd = PD(kP=KP, kD=KD)
        self.speed = speed
        self.exit = False

        self.PROJECT_FOLDER = PROJECT_FOLDER
        self.hard = hard
        self.objd = OBJDetection.OBJDetection(PROJECT_FOLDER)
        self.objd.svet_enable = True
        self.objd.load()
        self.frame = None
        self.ssnow, self.sign, self.svet_sign, self.person = None, None, None, None
        self.status = "ERRER"
        # self. = Utils.Transformator()
        f1 = Utils.Transformator("footprint", "camera")
        f1.set(0.04, 0, 0.21, 0, 0.68, 0)
        f2 = Utils.Transformator("camera", "camopt")
        f2.set(0, 0, 0, -math.pi/2, 0, -math.pi/2)
        f3 = Utils.Transformator("footprint", "back_wheels")
        f3.set(-0.162, 0, 0, 0, 0, 0)
        self.foot_print_to_camopt = Utils.Transformator.concat(f1, f2)
        # self.tf.set_transform("footprint", "camera_optical", Utils.TF.Transform(Utils.TF.Transform.from_xyzypr(0, 0, 0, 0, 0, 0).tr_mat @ Utils.TF.Transform.from_xyzypr(0, 0, 0, -math.pi/2, 0, -math.pi/2).tr_mat @ Utils.TF.Transform.from_xyzypr(0, 0, 0.1, 0, 0, 0).tr_mat ))
        self.ld = LaneDetector(region=np.array([[0.08, 0.17, 0], [0.08, -0.17, 0], [0.25, 0.17, 0], [0.25, -0.17, 0]]),
                               camera_mat=np.array([[686.9927350716015, 0.0, 640.5], [0.0, 686.9927350716015, 360.5], [0.0, 0.0, 1.0]]),
                               lane_sep=0.22, foot_print_to_camopt=self.foot_print_to_camopt, params=LaneDetector.DetectorParams())
        self.spred = SpeedPredictor(region=np.array([[0.08, 0.17, 0], [0.08, -0.17, 0], [0.5, 0.17, 0], [0.5, -0.17, 0]]),
                                camera_mat=np.array([[686.9927350716015, 0.0, 640.5], [0.0, 686.9927350716015, 360.5], [0.0, 0.0, 1.0]]),
                                foot_print_to_camopt=self.foot_print_to_camopt, params=SpeedPredictor.DetectorParams())
        self.pln = LocalPlanner(region=np.array([[0.08, 0.17, 0], [0.08, -0.17, 0], [0.3, 0.17, 0], [0.3, -0.17, 0]]),
                                camera_mat=np.array([[686.9927350716015, 0.0, 640.5], [0.0, 686.9927350716015, 360.5], [0.0, 0.0, 1.0]]),
                                lane_sep=0.22, foot_print_to_camopt=self.foot_print_to_camopt,
                                foot_print_to_back_wheels=f3,
                                params=LocalPlanner.PlannerParams())
        print(self.hard)
        for _ in range(4):
            self.hard.get()

        while self.hard.get()[0] != "OK":
            pass
        self.frame_shape = self.hard.get()[1].shape

        ### Video recorders
        FPS = 28
        ts = time.time()
        video_path_folder = PROJECT_FOLDER + f"/video_temp/video____time_{int(ts)}"
        os.makedirs(video_path_folder)
        video_o_path = f"{video_path_folder}/video_o_time_{int(ts)}.mkv"
        video_v_path = f"{video_path_folder}/video_v_time_{int(ts)}.mkv"

        self.record_original = VideoRecorder(video_o_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        self.record_vis = VideoRecorder(video_v_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        self.img_out = None
        self.gette_th = threading.Thread(target=self.img_getter)
        self.gette_th.daemon = True
        self.gette_th.start()
        self.sig_th = threading.Thread(target=self.signs)
        self.sig_th.daemon = True
        self.sig_th.start()
        try:
            self.main1()
        except KeyboardInterrupt:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} KeyboardInterrupt")
        except Exception as ex:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} Exception {type(ex).__name__} {ex.args}\
             in main1\n {traceback.format_exc()}")
        self.record_original.stop()
        self.record_vis.stop()
    def img_getter(self):
        r = Utils.Rate(30)
        while True:
            self.status, self.frame = self.hard.get()
            r.sleep()
    def main1(self):
        self._servo_pid = Utils.PID.PID(1, 0.00000, 0.00001)
        self._motor_pid = Utils.PID.PID(6, 0.2, 2)
        r = Utils.Rate(20)
        vel_setpoint = 0.1
        motor = self.MOTOR_STD
        print(f"{Utils.colors.YELLOW}[Main]{Utils.colors.ENDC} Started ")
        while cv2.waitKey(1) != ord("q"):

            if self.status == "OK":
                if self.img_out is not None:
                    cv2.imshow("self.img_out", self.img_out)
                # self.frame = frame
                # print()
                rc, road, ll, rl = self.ld.run(self.frame)
                vx, vy = self.spred.run(self.frame)

                # print(vx, vy)
                if rc > 0:
                    current_p = road[np.argmin((road - np.array([0, 0]))[0])]
                    # print("current_p", , )
                    q = road[1] - road[0]
                    aa = math.atan2(q[1], q[0])
                    if vx is not None:
                        # print("V", vel_setpoint-vx)
                        motor = self._motor_pid.calc(vel_setpoint-vx)*50+1500
                    else:
                        motor = (self.MOTOR_STD+motor)/2
                    # print("MOtor", motor)
                    aa = self.pln.run(self.frame, road, ll, rl, vx, vy, vel_setpoint)

                    ser = self.SERVO_0 + self._servo_pid.calc(math.degrees(aa))
                    # print(math.radians(ser-90))

                    self.hard.set(ser, motor)
                    # self.hard.set(ang, spd)
                # self.record_original.write(frame)
                # ang, spd = self.run(frame.copy())
                # vel_err = vel_setpoint-vx
                # spd = self._motor_pid.calc(vel_err)*50+1500
                # print("spd", "vel_err", vel_err)

                if self.exit:
                    break
            elif self.status == "ERROE":
                print(f"Error")
                break
            print("Loop rate", r.sleep())
            # r.sleep()

    def vision_func(self, frame):
        image = frame.copy()
        img = cv2.resize(image, (400, 300))
        binary = binarize(img, d=self.d)
        perspective = trans_perspective(binary, TRAP, RECT, SIZE)
        return perspective

    def detect_stop_line(self, frame):
        if (frame[100][180] > 200) and (frame[100][200] > 200) and (frame[100][160] > 200):
            return True
        else:
            return False

    def angleC(self, frame):
        image = frame.copy()
        left, right = centre_mass(image, d=self.d)
        angle = self.angle_pd.calc(left=left, right=right)
        if angle < 70:
            angle = 70
        elif angle > 106:
            angle = 104
        return angle

    @delay(delay=0.5)
    def stopeer_f(self):
        self.speed = stop_speed
        # exit()
        time.sleep(0.1)
        self.speed = self.MOTOR_STD
        # self.hard.set(90, self.speed)
        # self.exit = True

    def signs(self):
        r = Utils.Rate(20)
        while True:
            if self.frame is not None and self.status == "OK":
                self.img_out, self.ssnow, self.sign, self.svet_sign, self.person = self.objd.run(self.frame.copy())

                # cv2.imshow("Detected", img_out)
                # cv2.waitKey(1)
            r.sleep()

    def run(self, frame):
        img_out, ssnow, sign, svet_sign, person = self.objd.run(frame.copy())
        cv2.imshow("objd", img_out)
        print("[OBJD]", ssnow, sign, svet_sign, person)
        perspective = self.vision_func(frame=frame)
        self.angle = self.angleC(frame=perspective)
        cv2.imshow("perspective", perspective)
        stop_line = self.detect_stop_line(frame=perspective)
        if stop_line:
            print("STOP_LINE")
            self.speed = 1450
            self.stopeer_f()
            # send_cmd('H00/' + str(speed) + '/' + str(angle) + "E")
        # else:
        # send_cmd('H00/' + '1450' + '/' + str(angle) + "E")
        # time.sleep(0.5)
        # send_cmd('H00/' + str(stop_speed) + '/' + str(angle) + "E")
        return self.angle, self.speed
