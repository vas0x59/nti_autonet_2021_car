# from Hardware import Hardware
from Utils.VideoRecorder import VideoRecorder
import Utils.colors
import traceback
import os
from LaneDetection.LaneDetector import LaneDetector
import math
d = True
from Exec.OLD.func import *
import threading

from ObjectDetection import OBJDetection

SIGNS = False

class Main:
    def __init__(self, PROJECT_FOLDER, hard):
        self.d = d
        self.last = 0
        self.angle_pd = PD(kP=KP, kD=KD)
        self.speedPovorot = 1549
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
        self.stop_del = 0
        self.kyda = ['r', 'r', 'r']  # Здесь будут хранится куда ему надо будет поварачиваться, после каждого пройденого маршрута, этот элемент будет удалаться
        self.kyda_svet = [1, 0, 0]
        self.nKyda = 0  # Количество маршрутов
        self.objd.svet_enable = True
        self.objd.sign_enable = True
        self.signStop = 0
        self.angle = SERVO_0
        self.need_svet = False
        self.svet_sign = "nothing"
        self.person = 0
        self.ssnow = []
        self.stop_line = False
        self.img_out = None
        # self.tf = Utils.TF()
        # self.tf.set_transform("footprint", "camera_optical", Utils.TF.Transform.from_xyzypr(0, 0, 0.3, 0, 0, 0) @ Utils.TF.Transform.from_xyzypr(0, 0, 0, -math.pi/2, 0, -math.pi/2))
        # self.ld = LaneDetector(region=[[0.1, 0.1], [0.2]])

        print(self.hard)
        for _ in range(4):
            self.hard.get()

        while self.hard.get()[0] != "OK":
            pass
        self.frame_shape = self.hard.get()[1].shape
        self.frame = None
        ### Video recorders
        FPS = 28
        ts = time.time()
        video_path_folder = PROJECT_FOLDER + f"/video_temp/video____time_{int(ts)}"
        os.makedirs(video_path_folder)
        video_o_path = f"{video_path_folder}/video_o_time_{int(ts)}.mkv"
        video_v_path = f"{video_path_folder}/video_v_time_{int(ts)}.mkv"

        # self.record_original = VideoRecorder(video_o_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        # self.record_vis = VideoRecorder(video_v_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        th = threading.Thread(target=self.img_reg)
        # th.daemon = True
        th.start()
        try:
            self.main1()
        except KeyboardInterrupt:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} KeyboardInterrupt")
        except Exception as ex:
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} Exception {type(ex).__name__} {ex.args}\
             in main1\n {traceback.format_exc()}")
        # self.record_original.stop()
        # self.record_vis.stop()

    def resetPeret(self):
        self.need_svet = False
        self.next = 0
        self.go = 0
        self.pov = 0
        self.timeNow = 0
        # del self.kyda[0]
        self.nKyda += 1
        # self.nKyda = len(self.kyda)
        self.l = 0
        self.r = 0
        self.angle = SERVO_0
        self.signStop = 0
        self.objd.svet_enable = False
        self.objd.sign_enable = True
        self.timeLast = 0

    def main1(self):
        r = Utils.Rate(20)
        while True:
            status, frame = self.hard.get()
            if status == "OK":
                self.frame = frame
                self.img_out, self.ssnow, self.sign, self.svet_sign, self.person = self.objd.run(self.frame.copy(),
                                                                                                 conf=0.0001)
                # print("OK",  self.exit)
                # self.record_original.write(frame)
                ang, spd = self.run(frame.copy())
                self.hard.set(ang, spd)
                vf = frame.copy()
                cv2.putText(vf, "go:" + str(self.go), (10, 50), cv2.FONT_HERSHEY_DUPLEX,
                            1, (0, 200, 0), 1)
                cv2.putText(vf, "pov:" + str(self.pov), (10, 90), cv2.FONT_HERSHEY_DUPLEX,
                            1, (0, 200, 0), 1)
                cv2.putText(vf, "l:" + str(self.l) + " r:" + str(self.r), (10, 130), cv2.FONT_HERSHEY_DUPLEX,
                            1, (0, 200, 0), 1)
                cv2.putText(vf, "angle: " + str(self.angle) + " speed:" + str(self.speed), (10, 170), cv2.FONT_HERSHEY_DUPLEX,
                            1, (0, 200, 0), 1)
                cv2.putText(vf, "PERSON COUNT: " + str(self.objd.person_calc), (200, 50),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1, (255, 0, 100), 1)
                cv2.putText(vf, "SIGNS: " + str(self.sign_hist), (10, 250),
                            cv2.FONT_HERSHEY_DUPLEX,
                            1, (255, 0, 100), 1)
                color = (50, 50, 50)
                if self.svet_sign == "red":
                    color = (0, 0, 255)
                elif self.svet_sign == "green":
                    color = (0, 255, 0)
                elif self.svet_sign == "yellow":
                    color = (0, 255, 255)
                elif self.svet_sign == "red_yellow":
                    color = (0, 150, 255)
                elif self.svet_sign == "green_blink":
                    color = (0, 255, 110)
                cv2.putText(vf, "Svet: " + str(self.svet_sign), (500, 50), cv2.FONT_HERSHEY_DUPLEX,
                            1, color, 1)
                cv2.putText(vf, "Person: " + str(self.person), (500, 90), cv2.FONT_HERSHEY_DUPLEX,
                            1, (255, 0, 150), 1)
                cv2.putText(vf, "STOPLINE: " + str(self.stop_line), (500, 130), cv2.FONT_HERSHEY_DUPLEX,
                            1, (255, 0, 100), 1)
                cv2.imshow("Frame", vf)
                # cv2.waitKey(1)
                if self.exit:
                    break
            elif status == "ERROE":
                print(f"Error")
                break
            if cv2.waitKey(1) == 27:
                self.exit = True
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
        if (frame[90][180] > 200) and (frame[90][200] > 200) and (frame[90][160] > 200):
            return True
        else:
            return False

    # def detect_stop_line(self, frame):
    #     return frame[85:95][160:200].mean(axis=0).mean(axis=0) > 200

    def angleC(self, frame):
        image = frame.copy()
        left, right = centre_mass(image)
        angle = self.angle_pd.calc(left=left, right=right)
        if angle < SERVO_0 - 20:
            angle = SERVO_0 - 20
        elif angle > SERVO_0 + 20:
            angle = SERVO_0 + 20
        return angle

    def angele(self, left, right):
        angle = self.angle_pd.calc(left=left, right=right)
        if angle < SERVO_0-20:
            angle = SERVO_0-20
        elif angle > SERVO_0+20:
            angle = SERVO_0+20
        return angle

    @delay(delay=0.5)
    def stopeer_f(self):
        self.speed = 1470
        # exit()
        self.stop_del = 1
        time.sleep(0.3)
        self.speed = 1500
        self.stop_del = 0
        # self.exit = True
        # if self.need_svet != True:
        # self.go = 1

    def img_reg(self):
        r = Utils.Rate(20)
        while not self.exit:
            # if self.frame is not None:
            #     self.img_out, self.ssnow, self.sign, self.svet_sign, self.person = self.objd.run(self.frame.copy(), conf=0.0001)
            r.sleep()
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

        if len(self.sign_hist) == 0 and self.sign != "none":
            self.sign_hist += [self.sign]
        elif self.sign != "none" and self.sign_hist[-1] != self.sign:
            self.sign_hist += [self.sign]
        # print(self.sign_hist
        # )
        if self.img_out is not None:
            cv2.imshow("img_out", self.img_out)
        # print("COUNT", self.objd.person_calc)
        # if self.person:
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
        self.stop_line = self.detect_stop_line(frame=perspective)
        # cv2.imshow("perspective", perspective)
        if self.pov == 0:
            if self.stop_line:
                # self.resetPeret()

                self.go = 0
                self.objd.svet_enable = True
                self.objd.sign_enable = True
                # self.need_svet = True
                # print("STOP_LINE")
                self.angle = SERVO_0
                self.speed = 1500
                self.stopeer_f()
                self.pov = 1


                if SIGNS == True and (self.nKyda == 0):
                    if ("parking" in self.sign_hist) or ("stop" in self.sign_hist):
                        self.kyda = ["l", "r", "f", "r"]
                        # self.nKyda = 1
                        self.sign_hist = []
                    elif ("pedestrain" in self.sign_hist) or ("no_entery" in self.sign_hist) or ("no_drive" in self.sign_hist):
                        self.kyda = ["r", "r", "r"]
                        # self.nKyda = 1
                        self.sign_hist = []
                if len(self.kyda) > self.nKyda:
                    print(self.kyda)
                    self.need_svet = self.kyda_svet[self.nKyda]
                    self.l, self.r = (1, 0) if self.kyda[self.nKyda] == 'l' else (0, 1) if self.kyda[self.nKyda] == 'r' else (1, 1)
                else:
                    # self.speed = 1500
                    # self.speed = 1500
                    self.pov = 0
                    self.go = 0
            else:
                self.angle = self.angele(left=left, right=right)
        else:
            self.angle = SERVO_0
            if self.go == 0:
                #self.speed = 1500
                # img_out, ssnow, self.sign, svet_sign, self.person = self.objd.run(frame.copy(), thresh=15, conf=0.5)
                if self.need_svet:
                    if self.svet_sign == "green" and self.stop_del == 0:
                        self.go = 1
                        self.angle = SERVO_0
                        # self.speed = 1500

                    else:
                        self.go = 0
                        # self.speed = 1500
                else:
                    self.go = 1
                    self.angle = SERVO_0
                    # self.speed = 1500
            elif self.stop_del == 0:
                # print("234")

                if self.l == 1 and self.r == 1:  # ехать прямо
                    if left >= 150 and self.next == 0:
                        self.next += 1
                    elif left < 150 and self.next == 1:
                        self.next += 1
                    if self.next <= 1:
                        self.angle = SERVO_0
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
                        if time.time() - self.timeLast >= 2 and self.next == 0:
                            self.next += 1
                            self.timeLast = 0
                        elif time.time() - self.timeLast >= 1.8 and self.next == 1:
                            self.next += 1
                        if self.next == 0:
                            self.angle = SERVO_0
                        elif self.next == 1:
                            self.angle = SERVO_0 + 25
                        else:
                            self.resetPeret()
                        print("Left = ", time.time() - self.timeLast,
                              "angle = {} speed = {}".format(self.angle, self.speed))
                elif self.r == 1:  # ехать на право
                    # print("rrrrrrr")
                    self.speed = self.speedPovorot
                    if self.timeLast == 0:  # По времени
                        self.timeLast = time.time()
                    else:

                        one = centre_mass1(perspective[:, perspective.shape[1]//2:].copy())
                        print("rrrrrrrrrrrrrrrrrrrr", one, one)
                        # self.angle = self.angele(left=one, right=one)
                        self.angle = SERVO_0+(perspective.shape[1]//4-one)*0.35
                        if self.angle < SERVO_0 - 20:
                            self.angle = SERVO_0 - 20
                        elif self.angle > SERVO_0 + 20:
                            self.angle = SERVO_0 + 20
                        print(self.angle, "self.angle")
                        if (time.time() - self.timeLast >= 6) and self.next == 0:
                            self.next += 1
                            self.timeLast = time.time()
                        elif ((time.time() - self.timeLast >= 3) or (left < 150)) and self.next == 1:
                            self.next = 3
                        # if self.next == 0:
                        #     self.angle = SERVO_0
                        # elif self.next == 1:
                        #     self.angle = SERVO_0 - 45
                        if self.next == 3:
                            self.resetPeret()
                        print("Right = ", self.next,"angle = {} speed = {}".format(self.angle, self.speed))
        return self.angle, self.speed