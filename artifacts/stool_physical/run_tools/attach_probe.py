import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'] in ('assemble_00_part_3_close_and_attach', 'assemble_02_part_6_close_and_attach'):
        a = ph['attach'][0]
        print(ph['name'], sorted(a.keys()))
        for k in ('object_in_tcp_orientation','object_in_tcp_position','grasp_orientation','relative_orientation','tcp_in_object_orientation','contact_box_offset','contact_box_scale','grasp_id','grasp_width_m'):
            if k in a: print('  ', k, a[k])
        s = ph['local_skill']
        print('  skill keys with grasp/orient:', [k for k in s if 'grasp' in k or 'orient' in k][:30])
