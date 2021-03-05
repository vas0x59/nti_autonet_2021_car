from Hardware.Interface import IHardware
import Beholder.beholder
import socket
import numpy as np
import cv2
import json
import Utils.colors

class HardwareReal(IHardware):
    DEFAULT_CMD = "s"
    IPadress = "192.168.4.1"

    def __init__(self, **params):
        self._params = params
        self._sock = socket.socket()
        self._server_address = (self.IPadress, 1080)
        self._sock.connect(self._server_addres)
        print("{Utils.colors.GREEN}[HardwareReal]{Utils.colors.ENDC} Connection Established")
        self._client = Beholder.beholder.Client(zmq_host=self.IPadress,
                         # zmq_host="192.168.1.145",
                         zmq_port=12345,
                         rtp_host="192.168.1.208",
                         # rtp_host="10.205.1.185",
                         rtp_port=5000,
                         rtcp_port=5001,
                         device="/dev/video0",
                         # width=1920,
                         # height=1080,
                         width=1280,
                         height=720,
                         # width=640,
                         # height=480,
                         framerate=30,
                         encoding=Beholder.beholder.Encoding.MJPEG,  # MJPEG,    #H264
                         limit=20)
        self._client.start()
        print("{Utils.colors.GREEN}[HardwareReal]{Utils.colors.ENDC} client start")
        status, frame = self._client.get_frame(0.25)
        status, frame = self._client.get_frame(0.25)
        status, frame = self._client.get_frame(0.25)

        
    def set(self, servo: float, motor: float):
        cmd = 'H00/' + str(motor) + '/' + str(servo) + "E"
        message = cmd.encode()
        self._sock.sendall(message)


    def get(self):
        status, frame = self._client.get_frame(0.25)
        if status == Beholder.beholder.Status.OK:
            return "OK", frame
        elif status == Beholder.beholder.Status.EOS:
            return "Error", np.zeros((480, 640, 3))
            # break
        elif status == Beholder.beholder.Status.Error:
            return "Error", np.zeros((480, 640, 3))
            # break
        elif status == Beholder.beholder.Status.Timeout:
            return "Timeout", np.zeros((480, 640, 3))
            
    # pass
    # def send_cmd():
    #     pass
    # # def __init__(self):
    # #     pass