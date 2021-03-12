from Hardware.Interface import IHardware
import beholder
import socket
import numpy as np
import cv2
import json
import Utils.colors

import numpy as np
import math

    #
    #     phi1 = math.atan2()
    #     phi2 = math.atan2()
    #
    # def calcCmdAngularVel(self, lvx, avz):
    # def calcCmd(self, lvx, avz):


class HardwareReal(IHardware):
    # DEFAULT_CMD = "s"
    IPADDRESS = "192.168.4.1"
    def __del__(self):
        self.set(87, 1500)
        self._sock.close()
        self._client.stop()
    def __init__(self, frame_wh, **params):
        super().__init__(frame_wh, **params)
        self._sock = socket.socket()
        self._server_address = (self.IPADDRESS, 1080)
        self._sock.connect(self._server_address)
        print(f"{Utils.colors.GREEN}[HardwareReal]{Utils.colors.ENDC} Connection Established")
        print("CCCCC", self.IPADDRESS)
        self._client = beholder.Client(zmq_host=self.IPADDRESS,
                         # zmq_host="192.168.1.145",
                         zmq_port=12345,
                         rtp_host="192.168.4.4",
                         # rtp_host="10.205.1.185",
                         rtp_port=5000,
                         rtcp_port=5001,
                         device="/dev/video0",
                         # width=1920,
                         # height=1080,
                         width=frame_wh[0],
                         height=frame_wh[1],
                         # width=640,
                         # height=480,
                         framerate=30,
                         encoding=beholder.Encoding.MJPEG,  # MJPEG,    #H264
                         limit=2)
        self._client.start()
        print(f"{Utils.colors.GREEN}[HardwareReal]{Utils.colors.ENDC} client start")
        status, frame = self._client.get_frame(0.25)
        status, frame = self._client.get_frame(0.25)
        status, frame = self._client.get_frame(0.25)
        print(f"{Utils.colors.YELLOW}[HardwareReal]{Utils.colors.ENDC} {status}")
        status, frame = self._client.get_frame(0.25)
        print(f"{Utils.colors.YELLOW}[HardwareReal]{Utils.colors.ENDC} {status}")
        self.set(87, 1500)

    def set(self, servo: float, motor: float):
        cmd = 'H00/' + str(int(motor)) + '/' + str(int(servo)) + "E"
        message = cmd.encode()
        self._sock.sendall(message)

    def get(self):
        status, frame = self._client.get_frame(0.25)
        if status == beholder.Status.OK:
            return "OK", cv2.resize(frame, self.frame_wh)
        elif status == beholder.Status.EOS:
            print(status)
            return "Error", None
            # break
        elif status == beholder.Status.Error:
            print(status)
            return "Error", None
            # break
        elif status == beholder.Status.Timeout:
            print(status)
            return "Timeout", None
        else:
            print(status)

