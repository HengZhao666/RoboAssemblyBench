import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'].startswith('assemble_03_part_4_') and ph['name'].split('part_4_')[1] in ('move_above','preshape','descend','close_and_attach','lift','pickup_clearance','release','retreat','park'):
        s = ph.get('local_skill') or {}
        small = {k: v for k, v in s.items() if not isinstance(v, (dict,)) and (any(w in k for w in ('target','grasp','object','offset','height','distance','approach','relative','reference','pose'))) }
        print('==', ph['name'], 'timeout', ph.get('timeout_steps'))
        print('  skill', s.get('name'), json.dumps(small, default=str)[:900])
        print('  other keys', [k for k in ph if k not in ('local_skill',)])
        if ph.get('advance'): print('  advance', json.dumps(ph['advance'])[:300])
        if ph.get('attach'): print('  attach object/target', ph['attach'][0].get('object'), ph['attach'][0].get('target'))
        if ph.get('gripper_commands'): print('  gripper', ph['gripper_commands'])
names = [t['name'] for t in p['targets']]
print([n for n in names if n.startswith('part_4_')])
