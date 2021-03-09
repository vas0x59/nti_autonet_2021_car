from Hardware.Interface import IHardware
import numpy as np
import Utils.colors
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float64


class AckermannSimCmd1:
    """
    w - distance beetwen left and right wheels
    l - distance beetwen front and back wheel pairs
    r - radius of wheel
    minR - max value of the turning radius
    ma - maximum rotation angle of the servo
    servo_a > 0 - right
    """

    def __init__(self, w, l, r, ma):
        self.w = w
        self.l = l
        self.r = r
        # self.minR = minR
        self.ma = ma
        # self.k =

    def calcCmdServo(self, motor, servo_a):
        a = np.clip(servo_a, -self.ma, self.ma)
        # R = self.l * math.tan(a)
        # R = self.minR * (2 * (R >= 0) - 1) if abs(R) < self.minR else R
        if servo_a == 0:
            return motor, motor, 0, 0
        a0 = math.radians(servo_a)
        # print(a, a0)
        if a0 > 0:
            phi0 = math.pi - (math.pi/2 + a0)
            # print(phi0)
            R = self.l * math.tan(phi0)
            l1 = R - self.w / 2
            l2 = R + self.w / 2
            phi1 = math.atan(l1 / self.l)
            phi2 = math.atan(l2 / self.l)
            a1 = math.pi - (math.pi/2 + phi1)
            a2 = math.pi - (math.pi/2 + phi2)
            return motor/self.r, motor/self.r, a1, a2
        elif a0 < 0:
            phi0 = math.pi - (math.pi/2 + abs(a0))
            # print(phi0)
            R = self.l * math.tan(phi0)
            l1 = R + self.w / 2
            l2 = R - self.w / 2
            phi1 = math.atan(l1 / self.l)
            phi2 = math.atan(l2 / self.l)
            a1 = -(math.pi - (math.pi/2 + phi1))
            a2 = -(math.pi - (math.pi/2 + phi2))
            return motor/self.r, motor/self.r, a1, a2


class HardwareSim(IHardware):
    def __init__(self, frame_wh, **params):
        super().__init__(frame_wh, **params)
        rospy.init_node('YASK_HardwareSim', anonymous=True)
        print(f"{Utils.colors.GREEN}[HardwareSim]{Utils.colors.ENDC} Node initialized successfully ")
        self._bridge = CvBridge()
        self._sub_img = rospy.Subscriber(params["img_topic"], Image, self._img_h)
        self._img = None
        # self._pub = rospy.Publisher("/asd", Twist)
        self._as = AckermannSimCmd1(0.150, 0.162, 0.066, 35)
        # ы
        self._lh = rospy.Publisher("/vehicle_v1/front_left_rev_controller/command", Float64)
        self._rh = rospy.Publisher("/vehicle_v1/front_right_rev_controller/command", Float64)
        self._lr = rospy.Publisher("/vehicle_v1/front_left_cont_controller/command", Float64)
        self._rr = rospy.Publisher("/vehicle_v1/front_right_cont_controller/command", Float64)

    def _img_h(self, msg):
        self._img = self._bridge.imgmsg_to_cv2(msg, "bgr8")

    def get(self):
        if self._img is None:
            return "Empty", None
        else:
            return "OK", cv2.resize(self._img, self.frame_wh)

    def set(self, servo: float, motor: float):
        # tw = Twist()
        # tw.linear.x = (motor-1500)*(0.15/50)
        # tw.angular.z = math.radians(np.clip(-(90-servo), -90, 90))*1.1
        # self._pub.publish(tw)
        motor1, motor2, wh1, wh2 = self._as.calcCmdServo((motor-1500)*(0.4/50), -(90-servo)*1.1)
        self._lh.publish(wh1)
        self._rh.publish(wh2)
        self._lr.publish(motor1)
        self._rr.publish(motor2)
        # print(tw)
        # return None

