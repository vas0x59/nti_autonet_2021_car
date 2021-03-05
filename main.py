import sys
import os
from scripts.colors import *

PROJECT_FOLDER = os.path.dirname(os.path.realpath(__file__))
# print(_f)
sys.path.append(PROJECT_FOLDER+"/src")


last_deploy_tag = open(PROJECT_FOLDER+"/last_deploy.txt", "r").read().strip()
LOGO = open(PROJECT_FOLDER+"/LOGO.txt", "r").read()
LOGO += GREEN + "last_deploy_tag: " + str(last_deploy_tag) + ENDC + "\n"
print(LOGO)


import json
import importlib.util
exec_f_name = PROJECT_FOLDER+"/src/Exec/"+json.loads(open(PROJECT_FOLDER+"/configs/exec.json", "r").read())["exec"]

print(GREEN+f"[main.py] {ENDC} Starting {exec_f_name}")
print()
print()

exec_f_spec = importlib.util.spec_from_file_location("module.name", exec_f_name)
exec_f = importlib.util.module_from_spec(exec_f_spec)
exec_f_spec.loader.exec_module(exec_f)
# foo.MyClass()




