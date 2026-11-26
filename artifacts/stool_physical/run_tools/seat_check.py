import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'].endswith('_insert_09'):
        s = ph['local_skill']
        print(ph['name'], {k: s.get(k) for k in ('target_object_position_tolerance', 'target_object_axial_position_tolerance', 'target_object_lateral_position_tolerance', 'target_object_orientation_tolerance', 'target_object_stable_steps')})
print('success', [(e['object'][-1], round(e['position_tolerance'], 4), e['orientation_tolerance']) for e in p['success']])
