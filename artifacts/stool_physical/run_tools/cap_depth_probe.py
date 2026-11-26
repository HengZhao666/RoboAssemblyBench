import json, sys
sys.path.insert(0, '.')
import roboassemblybench.core.plumbers_physical as pp
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_rotate, quat_conjugate
captured = {}
orig = pp.align_contact_boxes_to_grasps
def spy(phases, task, spec):
    for s in task['assembly_steps']:
        captured[str(s['move_part'])] = (s['move_grasp'], next(p for p in task['parts'] if str(p['part_id']) == str(s['move_part'])))
    return orig(phases, task, spec)
pp.align_contact_boxes_to_grasps = spy
from roboassemblybench.core.task_registry import load_task_recipe
load_task_recipe('fabrica_stool_circular_ur5e_physical')
for pid in ('6', '4', '0', '8', '3', '7', '5', '1'):
    g, part = captured[pid]
    c = g['grasp_center_m']; top = part['bbox_max'][2]; bot = part['bbox_min'][2]
    q = g['tcp_in_assembly_orientation']; p = g['tcp_in_assembly_position']
    approach = quat_rotate(q, [0.0, 0.0, 1.0])
    pinch = quat_rotate(quat_conjugate(q), [a - b for a, b in zip(c, p)])[2]
    print(pid, 'grasp id', g.get('grasp_id'), 'centre z below top %.1f mm' % ((top - c[2]) * 1000), 'part h %.1f' % ((top - bot) * 1000),
          'approach_z %.3f' % approach[2], 'pinch %.4f' % pinch, 'width', g['grasp_width_m'])
