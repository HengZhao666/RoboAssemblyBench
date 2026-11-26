import sys, json
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
b = {ph["name"]: ph for ph in p["phases"]}
print({k: b["assemble_03_part_4_stage_hover"]["local_skill"].get(k) for k in ("physical_rotation_speed", "physical_rotation_lookahead")})
a = b["assemble_03_part_4_close_and_attach"]["attach"][0].get("hold_force_control") or {}
print("cap4 hold", {k: a.get(k) for k in ("low", "target", "soft_limit")})
a = b["assemble_00_part_3_close_and_attach"]["attach"][0].get("hold_force_control") or {}
print("leg3 hold", {k: a.get(k) for k in ("low", "target", "soft_limit")})
print("phases", len(p["phases"]))
