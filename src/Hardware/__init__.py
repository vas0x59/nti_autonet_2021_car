import os
import sys
from Hardware.Interface import IHardware


if os.environ.get("YASK_HARDWARE") == "SIM":
    from Hardware.Sim import HardwareSim
    Hardware = HardwareSim
else:
    from Hardware.Real import HardwareReal
    Hardware = HardwareReal


