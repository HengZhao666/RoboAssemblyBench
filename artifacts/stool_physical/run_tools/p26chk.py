import sys
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
b = {ph["name"]: ph for ph in p["phases"]}
for n in ("assemble_03_part_4_stage_hover", "assemble_03_part_4_stage_place"):
    s = b[n]["local_skill"]; print(n, b[n]["timeout_steps"], s.get("physical_rotation_speed"), s.get("physical_rotation_lookahead"))
print("max_steps", p["max_steps"])
