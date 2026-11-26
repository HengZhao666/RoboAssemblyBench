import sys
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
n = [(ph["name"], ph["local_skill"].get("target_object_reference_point")) for ph in p["phases"] if (ph.get("local_skill") or {}).get("target_object_reference_point")]
print(len(n)); print(n[:3]); print([x for x in n if "part_4_insert_09" in x[0] or "part_4_stage" in x[0]])
