import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'] == 'assemble_03_part_4_release':
        print(json.dumps({k: ph[k] for k in ph if k != 'local_skill'}, default=str)[:1500])
tg = {t['name']: t for t in p['targets']}
for n in ('part_4_assembled', 'part_2_assembled', 'part_4_pickup_clearance', 'part_4_assembly_clearance'):
    print(n, [round(v, 4) for v in tg[n]['position']], [round(v, 4) for v in tg[n]['orientation']])
for o in p['objects']:
    if o['name'] in ('optical_board', 'fabrica_fixture', 'fabrica_stool_circular_4'):
        print(o['name'], {k: o.get(k) for k in ('position', 'fabrica_bbox_min', 'fabrica_bbox_max', 'scale')})
r = {x['name']: x for x in p['robots']}
for n, x in r.items():
    print(n, x.get('position'), x.get('orientation'))
