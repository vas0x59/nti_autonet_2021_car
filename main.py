import sys
import os
from scripts.colors import *
import traceback

PROJECT_FOLDER = os.path.dirname(os.path.realpath(__file__))
# print(_f)
sys.path.append(PROJECT_FOLDER+"/src")


last_deploy_tag = open(PROJECT_FOLDER+"/last_deploy.txt", "r").read().strip()
LOGO = open(PROJECT_FOLDER+"/LOGO.txt", "r").read()
LOGO += GREEN + "last_deploy_tag: " + str(last_deploy_tag) + ENDC + "\n"
print(LOGO)


import json
import importlib.util
# from Hardware import Hardware
# hard = Hardware()
if os.environ.get("YASK_HARDWARE") == "SIM":
    from Hardware.Sim import HardwareSim
    print(f"{YELLOW}[main.py]{ENDC} SIM")
    hard = HardwareSim()
elif os.environ.get("YASK_HARDWARE") == "VIDEO":
    from Hardware.Video import HardwareVideo
    # HardwareVideo.params.
    print(f"{YELLOW}[main.py]{ENDC} VIDEO")
    hard = HardwareVideo(file=sys.argv[1])
else:
    from Hardware.Real import HardwareReal
    print(f"{YELLOW}[main.py]{ENDC} REAL")
    hard = HardwareReal()



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

