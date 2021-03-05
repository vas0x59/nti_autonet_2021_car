from Hardware import Hardware
from Utils.VideoRecorder import VideoRecorder
import Utils.colors
import time
import traceback

class Main:
    def __init__(self, PROJECT_FOLDER):
        self.PROJECT_FOLDER = PROJECT_FOLDER
        self.hard = Hardware()
        print(self.hard)

        self.frame_shape = self.hard.get().shape

        ### Video recorders
        FPS = 28
        ts = time.time()
        video_o_path = PROJECT_FOLDER + f"/video_temp/video_o_time_{int(ts)}.mkv"
        video_v_path = PROJECT_FOLDER + f"/video_temp/video_v_time_{int(ts)}.mkv"
        self.record_original = VideoRecorder(video_o_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        self.record_vis = VideoRecorder(video_v_path, FPS, (self.frame_shape[1], self.frame_shape[0]))
        try:
            self.main1()
        except Exception as ex:
            
            print(f"{Utils.colors.RED}[Main]{Utils.colors.ENDC} Exception {type(ex).__name__} {ex.args} in main1\n {traceback.format_exc()}")
        self.record_original.stop()
        self.record_vis.stop()

    def main1(self):
        qweqwe()
        

