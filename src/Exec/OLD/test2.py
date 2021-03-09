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
        self.kyda = ['r', 'l', 'f']  # Здесь будут хранится куда ему надо будет поварачиваться, после каждого пройденого маршрута, этот элемент будет удалаться
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
        self.go = 1
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

    def angleC(self, frame):
        image = frame.copy()
        left, right = centre_mass(image, d=self.d)
        angle = self.angle_pd.calc(left=left, right=right)
        if angle < 70:
            angle = 70
        elif angle > 106:
            angle = 104
        return angle

    def angele(self, left, right):
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
        # self.exit = True

    # def run(self, frame):
    #     img_out, ssnow, sign, svet_sign, person = self.objd.run(frame.copy())
    #     cv2.imshow("objd", img_out)
    #     print("[OBJD]", ssnow, sign, svet_sign, person)
    #     perspective = self.vision_func(frame=frame)
    #     self.angle = self.angleC(frame=perspective)
    #     cv2.imshow("perspective", perspective)
    #     stop_line = self.detect_stop_line(frame=perspective)
    #     if stop_line:
    #         print("STOP_LINE")
    #         self.speed = 1450
    #         self.stopeer_f()
    #         # send_cmd('H00/' + str(speed) + '/' + str(angle) + "E")
    #     # else:
    #     # send_cmd('H00/' + '1450' + '/' + str(angle) + "E")
    #     # time.sleep(0.5)
    #     # send_cmd('H00/' + str(stop_speed) + '/' + str(angle) + "E")
    #     return self.angle, self.speed
    def run(self, frame):
        # if self.objd.sign_enable:
        img_out, ssnow, self.sign, svet_sign, person = self.objd.run(frame.copy(), conf=0.05)
        cv2.imshow("img_out", img_out)
        # if person:
        #     self.speed = 1500
        #     self.stopeer_f()
        #     return self.angle, self.speed
        # if self.signStop == 0:
        #     if self.sign == 'stop':
        #         self.speed = 1500
        #         if self.timeLast == 0:
        #             self.timeLast = time.time()
        #         elif time.time() - self.timeLast > 5:
        #             self.signStop = 1
        #             self.speed = speed
        #             self.timeLast = 0
        #         return self.angle, self.speed
        perspective = self.vision_func(frame=frame.copy())
        left, right = centre_mass(perspective.copy())
        cv2.imshow("perspective", perspective)
        if self.pov == 0:
            self.need_svet = False
            stop_line = self.detect_stop_line(frame=perspective)
            if stop_line:
                self.objd.svet_enable = True
                self.objd.sign_enable = False
                print("STOP_LINE")
                self.angle = 88
                self.speed = 1500
                self.stopeer_f()
                self.pov = 1
                self.go == 1
                if self.nKyda > 0:
                    print("asdasdsad")
                    self.l, self.r = (1, 0) if self.kyda[0] == 'l' else (0, 1) if self.kyda[0] == 'r' else (1, 1)
                else:
                    self.speed = 1500
            else:
                self.angle = self.angele(left=left, right=right)
        else:
            if self.go == 0:
                # img_out, ssnow, self.sign, svet_sign, person = self.objd.run(frame.copy(), thresh=15, conf=0.5)
                if self.need_svet:
                    if svet_sign == "green":
                        self.go = 1
                        self.angle = 88
                        self.speed = speed
                    else:
                        self.go = 0
                        self.speed = stop_speed
            else:
                print("234")
                if self.l == 1 and self.r == 1:  # ехать прямо
                    if left >= 150 and self.next == 0:
                        self.next += 1
                    elif left < 150 and self.next == 1:
                        self.next += 1
                    if self.next <= 1:
                        self.angle = 88
                    else:
                        self.resetPeret()
                    print("Forward = ", time.time() - self.timeLast,"angle = {} speed = {}".format(self.angle, self.speed))
                elif self.l == 1:  # Ехать на лево
                    self.speed = self.speedPovorot
                    print("lllllllll")
                    if self.timeLast == 0:  # По времени
                        self.timeLast = time.time()
                    else:
                        print("lllllllllllllllllllllllll")
                        if time.time() - self.timeLast >= 0.9 and self.next == 0:
                            self.next += 1
                            self.timeLast = 0
                        elif time.time() - self.timeLast >= 3.5 and self.next == 1:
                            self.next += 1
                        if self.next == 0:
                            self.angle = 88
                        elif self.next == 1:
                            self.angle = 88 + 25
                        else:
                            self.resetPeret()
                        print("Left = ", time.time() - self.timeLast,
                              "angle = {} speed = {}".format(self.angle, self.speed))
                elif self.r == 1:  # ехать на право
                    print("rrrrrrr")
                    self.speed = speed
                    if self.timeLast == 0:  # По времени
                        self.timeLast = time.time()
                    else:
                        print("rrrrrrrrrrrrrrrrrrrr")
                        if time.time() - self.timeLast >= 0.5 and self.next == 0:
                            self.next += 1
                        elif time.time() - self.timeLast >= 2.5 and self.next == 1:
                            self.next += 1
                        if self.next == 0:
                            self.angle = 87
                        elif self.next == 1:
                            self.angle = 87 - 30
                        else:
                            self.resetPeret()
                        print("Right = ", time.time() - self.timeLast,"angle = {} speed = {}".format(self.angle, self.speed))
        return self.angle, self.speed