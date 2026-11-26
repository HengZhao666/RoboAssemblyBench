import json, sys, math
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_rotate
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    s = ph.get('local_skill') or {}
    if s.get('name') == 'ur5e_close_gripper':
        q = s.get('grasp_relative_orientation'); pos = s.get('grasp_relative_position')
        a = quat_rotate(q, [0.0, 1.0, 0.0]); z = quat_rotate(q, [0.0, 0.0, 1.0])
        yaw = math.degrees(math.atan2(a[1], a[0]))
        print(ph['name'], 'close_axis_obj', [round(v,3) for v in a], 'approach', [round(v,3) for v in z], 'yaw_from_x=%.1f' % yaw, 'rel_pos', [round(v,4) for v in pos] if pos else None, s.get('contact_box_scale'))
