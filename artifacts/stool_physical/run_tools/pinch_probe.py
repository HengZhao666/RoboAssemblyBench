import json, sys, statistics
sys.path.insert(0, '.')
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_conjugate, quat_rotate
d = json.load(open('roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json'))['tasks']
def pinch(g):
    q = g['tcp_in_assembly_orientation']; p = g['tcp_in_assembly_position']
    return quat_rotate(quat_conjugate(q), [c - x for c, x in zip(g['grasp_center_m'], p)])
for name in ('plumbers_block', 'stool_circular'):
    t = d[name]
    bs = [pinch(g)[2] for g in t['base_grasp_candidates'] if g.get('target_gripper') == 'robotiq-85']
    print(name, 'base n=%d pinch z min/med/max %.4f %.4f %.4f' % (len(bs), min(bs), statistics.median(bs), max(bs)))
    for s in t['assembly_steps']:
        ms = [pinch(g)[2] for g in s['move_grasp_candidates'] if g.get('target_gripper') == 'robotiq-85']
        if ms: print('  move', s['move_part'], 'n=%d pinch z %.4f %.4f %.4f' % (len(ms), min(ms), statistics.median(ms), max(ms)))
    g = t['base_grasp_candidates'][0]; print('  keys', sorted(g)[:30])
