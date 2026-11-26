import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
tg = {t['name']: t for t in p['targets']}
for ph in p['phases']:
    if ph['name'].startswith('assemble_00_part_3_insert_0') and ph['name'][-1] in '089' or ph['name'] == 'assemble_00_part_3_release':
        s = ph.get('local_skill') or {}
        print(ph['name'], {k: s.get(k) for k in ('target_object_target', 'target_object_final_target', 'position_tolerance', 'target_object_position_tolerance', 'target_object_axial_position_tolerance', 'target_object_orientation_tolerance', 'target_object_stable_steps')}, 'advance', ph.get('advance'))
for n in ('part_3_insert_08', 'part_3_insert_09', 'part_3_assembled'):
    if n in tg: print(n, [round(v, 4) for v in tg[n]['position']])
