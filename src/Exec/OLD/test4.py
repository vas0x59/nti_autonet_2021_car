# from Hardware import Hardware
from Utils.VideoRecorder import VideoRecorder
import Utils.colors
import traceback
import os
from LaneDetection.LaneDetector import LaneDetector
import math

d = True
from Exec.OLD.func import *

from ObjectDetection import OBJDetection


class Main:
    def __init__(self, PROJECT_FOLDER, hard):
        self.d = d
        self.last = 0
        self.angle_pd = PD(kP=KP, kD=KD)
        self.speedPovorot = 1550
        self.speed = speed
        self.exit = False

        self.PROJECT_FOLDER = PROJECT_FOLDER
        self.hard = hard
        self.objd = OBJDetection.OBJDetection(PROJECT_FOLDER)
        # self.objd.svet_enable = True
        # self.objd.load()
        self.sign = "none"
        self.sign_hist = []
        self.objd.load()
        self.timeLast = 0
        self.l = 0
        self.r = 0
        self.pov = 0
        self.go = 1
        self.next = 0
        self.timeNow = 0
        self.kyda = ['r', 'l',
                     'f']  # Здесь будут хранится куда ему надо будет поварачиваться, после каждого пройденого маршрута, этот элемент будет удалаться
        self.nKyda = len(self.kyda)  # Количество маршрутов
        self.objd.svet_enable = True
        self.objd.sign_enable = True
        self.signStop = 0
        self.angle = 87
        # self.tf = Utils.TF()
        # self.tf.set_transform("footprint", "camera_optical", Utils.TF.Transform.from_xyzypr(0, 0, 0.3, 0, 0, 0) @ Utils.TF.Transform.from_xyzypr(0, 0, 0, -math.pi/2, 0, -math.pi/2))
        # self.ld = LaneDetector(region=[[0.1, 0.1], [0.2]])

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
        try:
            self.main1()
        except KeyboardInterrupt:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} KeyboardInterrupt")
        except Exception as ex:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} Exception {type(ex).__name__} {ex.args}\
             in main1\n {traceback.format_exc()}")
        self.record_original.stop()
        self.record_vis.stop()

    def resetPeret(self):
        self.need_svet = False
        self.next = 0
        self.go = 0
        self.pov = 0
        self.timeNow = 0
        del self.kyda[0]
        self.nKyda = len(self.kyda)
        self.l = 0
        self.r = 0
        self.signStop = 0
        self.objd.svet_enable = False
        self.objd.sign_enable = True

    def main1(self):
        r = Utils.Rate(20)
        while cv2.waitKey(1) != ord("q"):
            status, frame = self.hard.get()
            if status == "OK":
                cv2.imshow("Frame", frame)
                self.record_original.write(frame)
                ang, spd = self.run(frame.copy())
                self.hard.set(ang, spd)
                if self.exit:
                    break
            elif status == "ERROE":
                print(f"Error")
                break
            # print("Loop rate", )
            r.sleep()

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

    def angele(self, frame):
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
        time.sleep(0.1)
        self.exit = True
        # exit()

    def run(self, frame):

        img_out, ssnow, self.sign, svet_sign, person = self.objd.run(frame.copy(), thresh=6, conf=0.15)
        print("COUNT", self.objd.person_calc)
        perspective = self.vision_func(frame=frame)
        self.angle = self.angele(frame=perspective)
        cv2.imshow("perspective", perspective)
        cv2.imshow("img_out", img_out)
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
        # if slen(elf.sign_hist
        # if self.sign == "stop":
        #     self.speed = 1450
        #     self.stopeer_f()
        # exit()
        if len(self.sign_hist) == 0 and self.sign != "none":
            self.sign_hist += [self.sign]
        elif self.sign != "none" and self.sign_hist[-1] != self.sign:
            self.sign_hist += [self.sign]
        print(self.sign_hist)
        return self.angle, self.speed