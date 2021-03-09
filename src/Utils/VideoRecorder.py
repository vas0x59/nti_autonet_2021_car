import numpy as np
import cv2
import Utils.colors


class VideoRecorder:
    def __init__(self, out_f: str, fps: float, frame_wh: tuple):
        self.out_f = out_f
        self.fps = fps
        self.frame_wh = frame_wh
        self._out = cv2.VideoWriter(out_f, cv2.VideoWriter_fourcc(*'XVID'), fps, frame_wh)

    def write(self, img: np.array):
        if img.shape[1] == self.frame_wh[0] and img.shape[0] == self.frame_wh[1]:
            try:
                self._out.write(img)
            except:
                print(f"{Utils.colors.RED}[VideoRecorder]{Utils.colors.ENDC} write error")
        else:
            print(f"{Utils.colors.RED}[VideoRecorder]{Utils.colors.ENDC} img has incorrect shape")

    def stop(self):
        self._out.release()
        print(f"{Utils.colors.YELLOW}[VideoRecorder]{Utils.colors.ENDC} Stop")

