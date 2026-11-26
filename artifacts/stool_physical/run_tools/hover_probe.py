import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'] == 'assemble_03_part_4_transport_hover':
        s = ph['local_skill']
        keys = [k for k in sorted(s) if any(w in k for w in ('tolerance', 'stable', 'settle', 'target', 'require', 'complete', 'speed', 'hold'))]
        print(json.dumps({k: s[k] for k in keys}, default=str)[:3500])
        print('advance', ph.get('advance'), 'timeout', ph.get('timeout_steps'))
