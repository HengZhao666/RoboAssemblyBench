import json, sys, collections
sys.path.insert(0, '.')
d = json.load(open('roboassemblybench/assets/Fabrica/canonical_7_bundles/canonical_tasks.json'))['tasks']
for name in ('stool_circular','plumbers_block'):
    t=d[name]; c=t['base_grasp_candidates']
    w=collections.Counter(round(g['grasp_width_m'],4) for g in c)
    print(name, 'base widths', w.most_common(6), 'open_ratio', collections.Counter(round(g.get('robotiq_open_ratio',0),3) for g in c).most_common(4))
    g=c[0]; print(' conv', g['gripper_frame_conversion'], g['gripper_frame_rotation_wxyz'])
    s=t['assembly_steps'][0]; m=s['move_grasp_candidates']
    print(' move', s['move_part'], collections.Counter(round(g['grasp_width_m'],4) for g in m).most_common(4))
