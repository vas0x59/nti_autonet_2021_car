# import os
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import cv2
import numpy as np 
from Detectors.YoloOpencvDetector import YoloOpencvDetector
from Detectors import Utils 
import time
# detector = YoloOpencvDetector("./Detectors/YOLO/yolov3.cfg", "./Detectors/YOLO/yolov3_320.weights")
# detector = YoloOpencvDetector("./Detectors/YOLO/yolov3.cfg", "./Detectors/YOLO/yolov3.weights")
# detector = YoloOpencvDetector("./Detectors/YOLO/yolov3.cfg", "./Detectors/YOLO/yolov3.weights")
# detector = YoloOpencvDetector("./Detectors/YOLO/yolov2-voc.cfg", "./Detectors/YOLO/yolov2-voc.weights")
detector = YoloOpencvDetector("./Detectors/YOLO/signs/yolov3_cfg.cfg", "./Detectors/YOLO/signs/yolov3_cfg_8800.weights", CLASSESPath="./signs.names")
# cap = cv2.VideoCapture("/home/vasily/Downloads/DJI_0002.MP4")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(640))
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(480))
# out = cv2.VideoWriter()
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_3.avi',fourcc, 28.0, (640,480))
frame_i = 0
time.sleep(1)
while True:
    ret, frame = cap.read()
    if ret == False:
        break
    if 0 == 0:
        
        # frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        # boxes, classIDs, confidences = detector.detect(frame, s=(320, 320))
        boxes, classIDs, confidences = detector.detect(frame, s=(320, 320))
        # boxes, classIDs, confidences = detector.detect(frame, s=(416, 416))
        # boxes, classIDs, confidences = detector.detect(frame, s=(608, 608))
        # boxes, classIDs, confidences = detector.detect(frame, s=(700, 700))
        frame = Utils.draw_boxes(frame, boxes, classIDs, confidences, detector.CLASSES, COLORS=detector.COLORS)
        out.write(frame)
        cv2.imshow("frame", frame)
        
    if cv2.waitKey(1) == ord('q'):
        break
    frame_i +=1 
cap.release()
out.release()