import sys
import os
PROJECT_FOLDER = os.path.dirname(os.path.realpath(__file__))
# print(_f)
sys.path.append(PROJECT_FOLDER+"/src")
import json
import importlib.util
exec_f_name = PROJECT_FOLDER+"/src/Exec/"+json.loads(open(PROJECT_FOLDER+"/configs/exec.json", "r").read())["exec"]


exec_f_spec = importlib.util.spec_from_file_location("module.name", exec_f_name)
exec_f = importlib.util.module_from_spec(exec_f_spec)
exec_f_spec.loader.exec_module(exec_f)
# foo.MyClass()




