import sys
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
print([t for t in p["targets"] if t["name"] in ("part_4_assembled", "part_4_preinsert_hover")])
