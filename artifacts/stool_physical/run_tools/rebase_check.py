import sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
names = [ph['name'] for ph in p['phases']]
for i, ph in enumerate(p['phases']):
    if ph.get('rebase_from_object'):
        rb = ph['rebase_from_object']
        print(i, ph['name'], 'next=', names[i + 1], rb['object'][-1], rb['anchor_target'], rb.get('keep_anchor_target'), rb['targets'])
print('phases', len(names), 'success unchanged', [(e['object'][-1], round(e['position_tolerance'], 4)) for e in p['success']])
