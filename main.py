#
#                   ___           ___           ___
#       ___        /  /\         /  /\         /__/|
#      /__/|      /  /::\       /  /:/_       |  |:|
#     |  |:|     /  /:/\:\     /  /:/ /\      |  |:|
#     |  |:|    /  /:/~/::\   /  /:/ /::\   __|  |:|
#   __|__|:|   /__/:/ /:/\:\ /__/:/ /:/\:\ /__/\_|:|____
#  /__/::::\   \  \:\/:/__\/ \  \:\/:/~/:/ \  \:\/:::::/
#     ~\~~\:\   \  \::/       \  \::/ /:/   \  \::/~~~~
#       \  \:\   \  \:\        \__\/ /:/     \  \:\
#        \__\/    \  \:\         /__/:/       \  \:\
#                  \__\/         \__\/         \__\/
#
#
#                                           НТИ АТС 2021
#
#                                          Vasily Yuryev


import sys
import os
from scripts.colors import *
import traceback
import json
import importlib.util

################################################################################
PROJECT_FOLDER = os.path.dirname(os.path.realpath(__file__))
sys.path.append(PROJECT_FOLDER+"/src")
last_deploy_tag = open(PROJECT_FOLDER+"/last_deploy.txt", "r").read().strip()
# LOGO = open(PROJECT_FOLDER+"/LOGO.txt", "r").read()
LOGO = GREEN + "last_deploy_tag: " + str(last_deploy_tag) + ENDC + "\n"
print(LOGO)
################################################################################




# from Hardware import Hardware
# hard = Hardware()

if os.environ.get("YASK_HARDWARE") == "SIM":
    print(f"{YELLOW}[main.py]{ENDC}  Hardware: SIM")
    from Hardware.Sim import HardwareSim
    hard = HardwareSim((1280, 720), img_topic="/camera1/image_raw")
elif os.environ.get("YASK_HARDWARE") == "VIDEO":
    print(f"{YELLOW}[main.py]{ENDC}  Hardware: VIDEO")
    from Hardware.Video import HardwareVideo
    # HardwareVideo.params.

    hard = HardwareVideo((1280, 720), file=sys.argv[1])
else:
    print(f"{YELLOW}[main.py]{ENDC}  Hardware: REAL")
    from Hardware.Real import HardwareReal
    hard = HardwareReal((1280, 720))


exec_f_name = PROJECT_FOLDER+"/src/Exec/"+json.loads(open(PROJECT_FOLDER+"/configs/exec.json", "r").read())["exec"]

print(GREEN+f"[main.py] {ENDC} Starting {exec_f_name}")
print()
print()

exec_f_spec = importlib.util.spec_from_file_location("module.name", exec_f_name)
exec_f = importlib.util.module_from_spec(exec_f_spec)
exec_f_spec.loader.exec_module(exec_f)
try:
    exec_f.Main(PROJECT_FOLDER, hard)
except KeyboardInterrupt:
    print(f"{RED}[main.py]{ENDC} KeyboardInterrupt")
except Exception as ex:
    # template = "An exception of type {0} occurred. Arguments:\n{1!r}"
    # message = template.format(, ex.args)
    print(f"{RED}[main.py]{ENDC} Exception {type(ex).__name__} {ex.args} in Main\n {traceback.format_exc()}")

# if os.environ.get("YASK_HARDWARE") != "SIM"
print(GREEN+f"[main.py] {ENDC} Finish {exec_f_name}")

