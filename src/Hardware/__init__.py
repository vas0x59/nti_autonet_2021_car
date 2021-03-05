import os
import sys
from Hardware.Real import HardwareReal
from Hardware.Sim import HardwareSim
from Hardware.Interface import IHardware


if os.environ.get("YASK_HARDWARE") == "SIM":
    Hardware = HardwareSim
else:
    Hardware = HardwareReal


