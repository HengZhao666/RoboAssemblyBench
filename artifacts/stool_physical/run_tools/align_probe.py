import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    s = ph.get('local_skill') or {}
    if s.get('name') == 'ur5e_close_gripper':
        a = (ph.get('attach') or [{}])[0]
        print(ph['name'], s.get('contact_box_orientation'), a.get('contact_box_orientation'), a.get('contact_box_scale'))
