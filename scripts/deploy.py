import os
import sys
import time 
from colors import *
PROJECT_PATH = sys.argv[1]
print()
print(f"{GREEN}[Deploy script]{ENDC} Started")
print()

def select_models() -> list:
    global PROJECT_PATH
    print("Select models")
    out_files = []
    models_folders = os.listdir(PROJECT_PATH + "/models")
    models_folders = [mf for mf in models_folders if mf !="_.txt"]
    for i, mf in enumerate(models_folders):
        print(f"[{i}] {mf} ", end="\n")
    print()
    # for i, mf in enumerate(models_folders):
    #     print(f"[{i}] {mf} ", end="")
    #     # sys.stdin.flush()
    #     if input() == "y":
    #         out_files.append(mf)
    #     # print()
    
    out_files = list(map(lambda x: models_folders[int(x)], map(str.strip, input("Selected: ").split())))
    out_files = list(map(lambda x: "models"+"/"+x, out_files))
    return out_files

folders = []

while True:
    models = select_models()
    print("Models: \n   %s" % '\n   '.join(models))
    if input("OK? ") == "y":
        folders += models
        break
print()
folders += ["configs"]
folders += ["src"]
folders += ["scripts/colors.py", "scripts/deploy.py"]
folders += ["main.py"]
folders += ["LOGO.txt"]
folders += ["Raspberry"]
# folders = list(map(lambda x: x, folders))
deloy_tag = str(int(time.time()))
print(GREEN + "last_deploy_tag: " + str(deloy_tag) + ENDC + "\n")
out_f = f"deploy_temp/YASK_pack_{deloy_tag}.zip" 
open(PROJECT_PATH+"/last_deploy.txt", "w").write(deloy_tag)
# print(f"zip -r -9 {out_f} {' '.join(folders)}")
folders += ["last_deploy.txt"]
os.system(f"zip -r -9 {out_f} {' '.join(folders)}")
print(f"SAVED {out_f}")
print(GREEN + "last_deploy_tag: " + str(deloy_tag) + ENDC + "\n")
# print("all:")

##### ZIP



