import sys, json
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
ph = [x for x in p["phases"] if x["name"] == "assemble_03_part_4_insert_09"][0]
print(json.dumps(ph["local_skill"], default=str, sort_keys=True))
