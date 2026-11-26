import sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
names = [t['name'] for t in p['targets']]
print([n for n in names if n.startswith('part_0_')])
print([n for n in names if not n.startswith('part_')][:20])
for ph in p['phases']:
    if ph['name'].startswith('assemble_06_part_0_'):
        s = ph.get('local_skill') or {}
        print(ph['name'], {k: v for k, v in s.items() if isinstance(v, str) and ('target' in k) and v})
for ph in p['phases']:
    if ph.get('rebase_from_object'):
        print('REBASE', ph['name'], {k: (v if k != 'targets' else v[:6] + ['...%d' % len(v)]) for k, v in ph['rebase_from_object'].items()})
