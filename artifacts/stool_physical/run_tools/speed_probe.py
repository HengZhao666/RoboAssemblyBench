import json, sys
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
for ph in p['phases']:
    if ph['name'].startswith('assemble_00_part_3_') and any(ph['name'].endswith(s) for s in ('lift','pickup_clearance','assembly_clearance','transport_hover','insert_00')):
        s = ph['local_skill']
        keys = [k for k in s if any(w in k for w in ('speed','step','lookahead','velocity','rate','smooth'))]
        print(ph['name'], ph.get('timeout_steps'), {k: s[k] for k in sorted(keys)})
