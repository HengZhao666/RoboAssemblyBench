import sys
sys.path.insert(0, ".")
from roboassemblybench.core.task_registry import load_task_recipe
from roboassemblybench.core import plumbers_physical as pp
p = load_task_recipe("fabrica_stool_circular_ur5e_physical")
tg = {t["name"]: t for t in p["targets"]}
import json
parts = json.load(open("roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json"))["tasks"]["stool_circular"]["parts"]
for pid in ("6","4","0","8"):
    s = tg["part_%s_staged" % pid]["position"]; pt = [x for x in parts if str(x["part_id"])==pid][0]
    cx = s[0] + 0.5*(pt["bbox_min"][0]+pt["bbox_max"][0]); cy = s[1] + 0.5*(pt["bbox_min"][1]+pt["bbox_max"][1])
    lx, ly = cx-0.47, cy+0.14
    print(pid, "centre board-local mm", round(lx*1000,2), round(ly*1000,2), "cell residual", round((lx*1000)%25,3), round((ly*1000)%25,3), "z", round(s[2],4))
