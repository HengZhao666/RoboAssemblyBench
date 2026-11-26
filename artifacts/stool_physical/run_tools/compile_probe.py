import json, sys, collections
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
name = sys.argv[1]
p = load_task_recipe(name)
ph = p['phases']
print('recipe', name, 'phases', len(ph), 'max_steps', p.get('max_steps'))
print(json.dumps({k: p['fabrica_canonical_resolved'].get(k) for k in ('continuous_physics','grasp_execution_mode','base_part')}))
kinds = collections.Counter((x.get('local_skill') or {}).get('name') for x in ph)
print(kinds.most_common())
for x in ph[:]:
    s = x.get('local_skill') or {}
    print(x['name'], '|', x.get('robot') or s.get('robot'), '|', s.get('name'), '|', x.get('timeout_steps'), '|', (x.get('advance') or {}).get('type'))
for e in p['success']:
    print('SUCCESS', {k: e.get(k) for k in ('object','target','position_tolerance','orientation_tolerance','require_released','require_static')})
