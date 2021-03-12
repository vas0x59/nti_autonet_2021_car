# import Data
import numpy as np
from ObjectDetection.obj_detectors.Detectors.YoloOpencvDetector import YoloOpencvDetector
from ObjectDetection.obj_detectors.Detectors.SSDOpencvCaffeDetector import SSDOpencvDetetor
from ObjectDetection.obj_detectors.Detectors import Utils
import cv2
import time
import dlib
cv = cv2
def get_area(p):
    return p[2]*p[3]
def get_centre(p):
    # print( p[1] + (p[3] // 2))
    return (p[0] + (p[2] // 2), p[1] + (p[3] // 2))

class OBJDetection:
    def __init__(self, PR_DIR):
        # self.drive_data = drive_data
        self.detector = None
        self.detector_std = None
        self.PR_DIR = PR_DIR
        # self.model_w_path = "yolo_sign_model_v1/yolov3_signs_v1_12800.weights"
        # self.model_c_path = "yolo_sign_model_v1/yolov3_signs_v1.cfg"
        # self.model_n_path = "yolo_sign_model_v1/signs.names"
        # self.model_w_path = "yolo_sign_v2/yolov3_cfg_381000.weights" # signs200 all197 208 ok
        self.model_w_path = PR_DIR+ "/models/yolo_sign_v2_1/yolov3_cfg_524000.weights"
        self.model_c_path = PR_DIR+ "/models/yolo_sign_v2_1/yolov3_cfg.cfg"
        self.model_n_path = PR_DIR+ "/models/yolo_sign_v2_1/classes.txt"
        # self.model_w_path = PR_DIR + "/models/yolo_sign_v2_1/yolov3_cfg_524000.weights"
        self.model_2_w_path = PR_DIR + "/models/NTI_person-main/MobileNetSSD_deploy.caffemodel"
        self.model_2_p_path = PR_DIR + "/models/NTI_person-main/MobileNetSSD_deploy.prototxt.txt"
        self.model_2_n_path = PR_DIR + "/models/NTI_person-main/coco.names"
        self.model_res = 320
        self.sings_filter = ["pedestrian", "stop", "parking",
                             "a_unevenness", "road_works", "way_out", "no_drive", "no_entery"]
        self.filter_dict = dict().fromkeys(self.sings_filter, 0)
        self.frames_left = 0
        self.hist = []
        self.labels = ['red', 'yellow', 'green']
        self.svet_hist = []
        self.svet_enable = False
        self.sign_enable = True
        self.person_calc = 0
        self.last_person_time = 0
        """
        pedestrian
        no_drive
        a_unevenness
        no_entery
        road_works
        stop
        way_out
        parking
        """

    def load(self):
        self.detector = YoloOpencvDetector(
            self.model_c_path, self.model_w_path, CLASSESPath=self.model_n_path)
        self.detector2 = cv2.dnn.readNetFromCaffe(self.model_2_p_path, self.model_2_w_path)
        # self.detector_std = YoloOpencvDetector("obj_detectors/Detectors/YOLO/yolov3_tiny.cfg",
        #                                        "obj_detectors/Detectors/YOLO/yolov3_tiny.weights", CLASSESPath="obj_detectors/coco.names")
        # self.dlib_d = dlib.simple_object_detector(self.PR_DIR+"/models/tld/tld.svm")

    # def reg_svet_hog_dlib(self, fr):
    #     # img_size = [200, 360]
    #     # frame = cv2.resize(fr, (img_size[1], img_size[0]))
    #     # cv2.imshow("frame",frame)
    #     # cv2.waitKey(1)
    #     # frame = frame[0:img_size[0] // 10 * 3, 180:img_size[1] - 140]
    #     # cv2.imshow("frame", frame)
    #     # cv2.waitKey(1)
    #     cv2.imshow("fr", fr)
    #     image = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
    #     boxes = self.dlib_d(image)
    #
    #     label = "none"
    #     for box in boxes:
    #         # cv2.im
    #         (x, y, xb, yb) = [box.left(), box.top(), box.right(), box.bottom()]
    #         if x > 0 and y > 0 and xb > 0 and yb > 0:
    #             crop = frame[y:yb, x:xb].copy()
    #             cv2.imshow("Croped", crop)
    #             label = self.predict_svet(crop)
    #             # print (box)
    #             break
    #     # cv2.waitKey(1)
    #     return label
    def reg_svet_simple(self, fr):
        # cv2.imshow("fr", fr)
        up_r = np.array([166, 255, 255])
        down_r = np.array([80, 50, 229])
        hsv = cv2.cvtColor(np.array(np.clip(np.float32(fr)-2, 0, 255), dtype="uint8"), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, down_r, up_r)
        cv2.imshow("mask", mask)
        ss = np.sum(mask)
        print("sum", ss)
        return ss > 15000


    def predict_svet(self, inm):
        crop2 = inm.copy()
        crop2 = cv2.resize(crop2, (20, 60))
        est = False
        svet = [0, 0, 0]
        h, w = crop2.shape[:2]
        # q = crop.copy()
        # crop2 = cv.cvtColor(crop2, cv.COLOR_RGB2BGR)

        # svezt[3] = 3
        cv2.imshow("1", crop2)
        crop2 = cv.cvtColor(crop2, cv.COLOR_BGR2HLS)[:, :, 1]
        # crop2 = cv.inRange(crop2, (0, 80, 180), (255, 255, 255))
        # crop2 = crop2[:, :, 2]s
        crop2 = cv.medianBlur(crop2, 3)
        cv2.imshow("crop2", crop2)
        redCrop = crop2[int(0):h//15*4]
        yellowCrop = crop2[h//15*5:h//15*9]
        greenCrop = crop2[h//15*10:int(h// 15 * 15)]
        # cv2.imshow("cr", crop2)
        # cv2.waitKey(1)
        # svet[0] = np.sum(redCrop[:,:,2]) / (w*h)
        # redBlueSum = np.sum(redCrop[:,:,0]) / (w*h)
        svet[0] = np.mean(redCrop[:, :]) / float(255)

        # svet[1] = (np.sum(yellowCrop[:,:,1]) + np.sum(yellowCrop[:,:,2]))/2 / (w*h)
        # yellowBlueSum = np.sum(yellowCrop[:,:,0]) / (w*h)
        svet[1] = np.mean(yellowCrop[:, :]) / float(255)
        # svet[2] = np.sum(greenCrop[:,:,1]) / (w*h)
        svet[2] = np.mean(greenCrop[:, :]) / float(255)
        # if (abs(svet[0]-svet[2]) < 0.05):
        #     return "yellow"
        # print(svet)
        # return self.labels[svet.index(max(svet))]
        label = "green_blink"
        svet[2]  = svet[2] *1.0
        # print(svet)
        if max(svet) > 0.56:
            if svet[0] > svet[2] and svet[1] > svet[2] and svet[0] > 0.6 and svet[1] > 0.6 :
                label = "red_yellow"
            elif svet[2] > svet[0] and svet[2] > svet[1]:
                label = "green"
            elif svet[1] > svet[2] and svet[1] > svet[0]:
                label = "yellow"
            elif svet[0] > svet[2] and svet[0] > svet[1]:
                label = "red"
        self.svet_hist += [label]
        if len(self.svet_hist) > 20:
            self.svet_hist = self.svet_hist[-20:]
        if abs(self.svet_hist.count("green") -self.svet_hist.count("green_blink")) < 6 and  abs((self.svet_hist[-5:].count("green") + self.svet_hist[-5:].count("green_blink")) - len(self.svet_hist[-5:])) < 2:
            label="green_blink"
        return label
    def detect_persons2(self, img):
        # self.detector2
        # frame = rgb_image
        (h, w) = img.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)),
                                     0.007843, (300, 300), 127.5)

        self.detector2.setInput(blob)
        detections = self.detector2.forward()
        rects = []
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.1:
                idx = int(detections[0, 0, i, 1])
                if idx == 15:
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (xMin, yMin, xMax, yMax) = box.astype("int")
                    rects.append([xMin, yMin, xMax, yMax])

        return rects

    def run(self, frame, thresh=6, conf=0.001):
        frame = frame[:frame.shape[0]//4*3, frame.shape[1]//6*2:]
        # frame
        label_simple = self.reg_svet_simple(frame[20:90, 150:400])
        boxes, classIDs, confidences = self.detector.detect(
            np.array(np.clip(np.float32(frame)-5, 0, 255), dtype="uint8"), s=(self.model_res, self.model_res), conf=conf)
        img_out = Utils.draw_boxes(
            frame, boxes, classIDs, confidences, self.detector.CLASSES, COLORS=self.detector.COLORS)
        # boxes2, classIDs2, confidences2 = self.detector2.detect(
        #     frame, s=(self.model_res, self.model_res), conf=0.4)
        # self.detector2
        # rects = self.detect_persons2(frame)

        # img_out = Utils.draw_boxes(
        #     img_out, rects, [10 for _ in rects], [1 for _ in rects], self.detector.CLASSES, COLORS=self.detector.COLORS)
        if self.sign_enable ==True:
            signs_o = sorted([(self.detector.CLASSES[classIDs[i]], boxes[i]) for i in range(len(classIDs)) if get_area(boxes[i]) > frame.shape[0]
                            * frame.shape[1] * 0.01 and self.detector.CLASSES[classIDs[i]] in self.sings_filter and confidences[i] > 0.33], key=lambda x: get_area(x[1]), reverse=True)
        # print(signs_o)
        persons = [confidences[i] for i in range(len(classIDs)) if classIDs[i] == 10 and get_centre(boxes[i])[1] > (frame.shape[0] // 6 * 0.2)]
        # person = 10 in classIDs
        # print(persons)
        person = len(persons) > 0
        if person and time.time() - self.last_person_time > 3.6:
            self.person_calc += 1
            self.last_person_time = time.time()
        cv2.putText(img_out, str(person), (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 150), 2)
        signs = []
        if self.sign_enable ==True:
            signs = [i[0] for i in signs_o]
        # self.drive_data.set("signs", signs)
        svet_label = "nothing"
        # svet_label = label_hog
        if self.svet_enable == True:
            if "traffic_light" in [self.detector.CLASSES[i] for i in classIDs]:
                svet = max([(confidences[i], boxes[i]) for i in range(len(
                    classIDs)) if self.detector.CLASSES[classIDs[i]] == "traffic_light"], key=lambda x: x[0])[1]
                # svet = svet
                svet_label = self.predict_svet(frame[np.clip(svet[1], 0, frame.shape[0]):np.clip(
                    svet[1] + svet[3], 0, frame.shape[0]), np.clip(svet[0], 0, frame.shape[1]):np.clip(svet[0] + svet[2], 0, frame.shape[1])])
            color = (0, 0, 0)
            if svet_label == "red":
                color = (0, 0, 255)
            elif svet_label == "green":
                color = (0, 255, 0)
            elif svet_label == "yellow":
                color = (0, 255, 255)
            elif svet_label=="red_yellow":
                color = (0, 150, 255)
            elif svet_label == "green_blink":
                color = (0, 255, 110)
            cv2.putText(img_out, svet_label, (10, 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)
            if svet_label == "nothing" and len(self.svet_hist) > 0:
                del self.svet_hist[-1]
            # print(self.svet_hist)
        if svet_label == "nothing" and label_simple:
            svet_label = "green"
        # print(svet_label)
        # boxes, classIDs, confidences = self.detector_std.detect(
        #     frame, s=(self.model_res, self.model_res))
        # img_out = Utils.draw_boxes(img_out, boxes, classIDs, confidences,
        #                            self.detector_std.CLASSES, COLORS=self.detector_std.COLORS)
        # for i in signs:
        # self.filter_dict[i] += 1
        # if self.frames_left == thresh:
        # self.frames_left = 0
        # self.filter_dict = dict().fromkeys(self.sings_filter, 0)
        # else:
        # self.frames_left +=1

        mm = "none"
        if self.sign_enable ==True:
            self.hist += signs
            if len(self.hist) > thresh:
                self.hist = self.hist[thresh:]
            self.filter_dict = dict().fromkeys(self.sings_filter, 0)
            for i in self.hist:
                self.filter_dict[i] += 1
            if sum(map(lambda x: x[1], self.filter_dict.items())) > 0 and len(self.hist) > 4:
                mm = max(self.filter_dict.items(), key=lambda x: x[1])[0]
        return img_out, signs, mm, svet_label, person
