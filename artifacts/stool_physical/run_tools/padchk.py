import sys
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
print([o for o in p["objects"] if o["name"] == "cap_staging_pad"])
tg = {t["name"]: t for t in p["targets"]}
print([(k, [round(v, 4) for v in tg[k]["position"]]) for k in ("part_4_staged", "part_6_staged", "part_0_staged", "part_8_staged")])
print("fixed", "cap_staging_pad" in p["domain_randomization"].get("fixed_objects", []), [g for g, v in p["domain_randomization"].get("groups", {}).items() if "cap_staging_pad" in (v.get("objects") or [])])
