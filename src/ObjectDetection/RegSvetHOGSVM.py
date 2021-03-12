import numpy as np
import cv2
import cv2 as cv
# from keras.preprocessing.image import img_to_array
# from keras.models import load_model
# from keras import backend as K
# model = load_model('sv3.keras')
# from tensorflow.keras.models import model_from_json
import dlib
from threading import Thread
import time


class RegSvet:
    def __init__(self, cap):
        self.labels = ['red', 'yellow', 'green']
        self.label = "none"
        # self.vs = vs
        self.cap = cap
        self.reg_t = False
        self.stop_t = False
        self.prev_t = time.time()

    def load_model_nn(self, w, j):
        # f = open(j, 'r')
        # json_string = f.readline()
        # f.close()
        # self.model = model_from_json(json_string)
        # self.model.load_weights(w)
        pass

    def load_model_svm(self, p):
        self.model_detector = dlib.simple_object_detector("tld.svm")

    def predict_label(self, inm):
        crop2 = inm.copy()
        est = False
        svet = [0, 0, 0]
        h, w = crop2.shape[:2]
        # q = crop.copy()
        # crop2 = cv.cvtColor(crop2, cv.COLOR_RGB2BGR)

        # svezt[3] = 3
        # cv2.imshow("1", crop2)
        crop2 = cv.cvtColor(crop2, cv.COLOR_BGR2HLS)
        crop2 = cv.inRange(crop2, (0, 80, 180), (255, 255, 255))
        # crop2 = crop2[:, :, 2]s
        crop2 = cv.medianBlur(crop2, 3)
        # cv2.imshow("crop2", crop2)
        redCrop = crop2[0:h // 3]
        yellowCrop = crop2[h // 3:h // 3 * 2]
        greenCrop = crop2[h // 3 * 2:h]
        # cv2.imshow("cr", crop2)
        # cv2.waitKey(1)
        # svet[0] = np.sum(redCrop[:,:,2]) / (w*h)
        # redBlueSum = np.sum(redCrop[:,:,0]) / (w*h)
        svet[0] = np.sum(redCrop[:, :]) / (w * h // 3) / float(255)

        # svet[1] = (np.sum(yellowCrop[:,:,1]) + np.sum(yellowCrop[:,:,2]))/2 / (w*h)
        # yellowBlueSum = np.sum(yellowCrop[:,:,0]) / (w*h)
        svet[1] = np.sum(yellowCrop[:, :]) / (w * h // 3) / float(255)
        # svet[2] = np.sum(greenCrop[:,:,1]) / (w*h)
        svet[2] = np.sum(greenCrop[:, :]) / (w * h // 3) / float(255)
        if (abs(svet[0] - svet[2]) < 0.05):
            return "yellow"
        # print(svet)
        return self.labels[svet.index(max(svet))]

    # def predict_label(self, inm):
    #     image = inm.copy()
    #     image = cv2.resize(image, (38, 38))
    #     # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    #     # cv2.imshow("image", image)
    #     image = img_to_array(image)
    #     image = np.expand_dims(image, axis=0)
    #     image = np.array(image, dtype="float") / 255.0
    #     # print(image)
    #     pre = self.model.predict(image)[0]
    #     # K.clear_session()
    #     i_max = 0
    #     for i in range(len(pre)):
    #         if pre[i_max] < pre[i]:
    #             i_max = i
    #     return self.labels[i_max]

    def reg_svet(self, fr):
        img_size = [200, 360]
        frame = cv2.resize(fr, (img_size[1], img_size[0])).copy()
        # cv2.imshow("frame",frame)
        # cv2.waitKey(1)
        frame = frame[0:img_size[0] // 10 * 3, 180:img_size[1] - 140]
        cv2.imshow("frame", frame)
        # cv2.waitKey(1)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = self.model_detector(image)

        label = "none"
        for box in boxes:
            # cv2.im
            (x, y, xb, yb) = [box.left(), box.top(), box.right(), box.bottom()]
            if x > 0 and y > 0 and xb > 0 and yb > 0:
                crop = frame[y:yb, x:xb].copy()
                cv2.imshow("Croped", crop)
                label = self.predict_label(crop)
                # print (box)
                break
        return label

    # def reg_toggle(self, q):
    #     self.reg_t = q
    #
    # def stop(self):
    #     self.stop_t = True

    # def update(self):
    #
    #     # self.model = load_model("svv3.keras")
    #     while not self.stop_t:
    #         st = time.time()
    #         # frame = self.vs.read()
    #         r, frame = self.cap.read()
    #         # print(frame)
    #         # cv2.imshow("frame", frame)
    #         # cv2.waitKey(1)
    #         if self.stop_t:
    #             break
    #         self.label = self.reg_svet(frame)
    #         # print("tt:", time.time()-st)

    # def get_label(self):
    #     return self.label

    # def start(self):
    #
    #     t = Thread(target=self.update, args=())
    #     t.daemon = True
    #     t.start()