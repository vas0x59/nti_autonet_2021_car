import os
import sys
import time 
PROJECT_PATH = sys.argv[1]


def select_models() -> list:
    global PROJECT_PATH
    print("Select models")
    out_files = []
    models_folders = os.listdir(PROJECT_PATH + "/models")
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
folders += ["main.py"]
# folders = list(map(lambda x: x, folders))
out_f = f"deploy_temp/YASK_pack_{int(time.time())}.zip" 
# print(f"zip -r -9 {out_f} {' '.join(folders)}")
os.system(f"zip -r -9 {out_f} {' '.join(folders)}")
print(f"SAVED {out_f}")

# print("all:")

##### ZIP



