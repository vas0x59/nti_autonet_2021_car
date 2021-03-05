import os
import sys
from Hardware.Interface import IHardware


if os.environ.get("YASK_HARDWARE") == "SIM":
    from Hardware.Sim import HardwareSim
    Hardware = HardwareSim
elif os.environ.get("YASK_HARDWARE") == "VIDEO":
    from Hardware.Video import HardwareVideo
    # HardwareVideo.params.
    Hardware = HardwareVideo
else:
    from Hardware.Real import HardwareReal
    Hardware = HardwareReal


