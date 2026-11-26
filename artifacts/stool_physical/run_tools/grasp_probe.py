import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_conjugate, quat_rotate
p = load_task_recipe(sys.argv[1])
r = p['fabrica_canonical_resolved']
print(json.dumps({k: r.get(k) for k in r if 'grasp' in k and not isinstance(r.get(k),(list,dict))}, indent=0)[:600])
for ph in p['phases']:
    s = ph.get('local_skill') or {}
    if s.get('name') == 'ur5e_close_gripper':
        for a in ph.get('attach', []):
            g = a.get('grasp') or {}
        print(ph['name'], {k: s.get(k) for k in ('object','grasp_id','contact_box_offset','contact_box_scale')})
sel = r.get('grasp_selection') or {}
print(json.dumps(sel, default=str)[:3000])
