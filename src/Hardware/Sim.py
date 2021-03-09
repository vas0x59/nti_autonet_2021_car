from Hardware.Interface import IHardware
import numpy as np
import Utils.colors
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
import math


class HardwareSim(IHardware):
    def __init__(self, frame_wh, **params):
        super().__init__(frame_wh, **params)
        rospy.init_node('YASK_HardwareSim', anonymous=True)
        print(f"{Utils.colors.GREEN}[HardwareSim]{Utils.colors.ENDC} Node initialized successfully ")
        self._bridge = CvBridge()
        self._sub_img = rospy.Subscriber(params["img_topic"], Image, self._img_h)
        self._img = None
        self._pub = rospy.Publisher("/asd", Twist)

    def _img_h(self, msg):
        self._img = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    def get(self):
        if self._img is None:
            return "Empty", None
        else:
            return "OK", cv2.resize(self._img, self.frame_wh)

    def set(self, servo: float, motor: float):
        tw = Twist()
        tw.linear.x = (motor-1500)*(0.15/50)
        tw.angular.z = math.radians(np.clip(-(90-servo), -90, 90))
        self._pub.publish(tw)
        print(tw)
        # return None

