from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')
p = R / 'roboassemblybench/core/plumbers_physical.py'; t = p.read_text()
old = "        'close_force_max_command_lead': 0.001,\n"
assert t.count(old) == 1
t = t.replace(old, "        # Gentle-seek command may lead the measured finger joint by this much.\n"
                   "        # Tilted stool grasps lag ~0.0011 rad statically, which deadlocks 0.001.\n"
                   "        'close_force_max_command_lead': _close_command_lead(spec),\n")
anchor = "def _close_max_seek(spec: dict) -> float:"
t = t.replace(anchor, '''def _close_command_lead(spec: dict) -> float:
    value = spec.get('physical_close_force_max_command_lead', 0.001)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0.0 < float(value) <= 0.01:
        raise ValueError('physical_close_force_max_command_lead must be in (0, 0.01] rad.')
    return float(value)


''' + anchor, 1)
p.write_text(t)
r = R / 'roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml'; y = r.read_text()
old = "  physical_close_force_max_seek: 0.08\n"
assert y.count(old) == 1
y = y.replace(old, old + "  # 45-degree tilted grasps hold the finger ~0.0011 rad behind its command;\n"
                         "  # the Plumbers 0.001 rad seek lead then never advances (smoke02/04).\n"
                         "  physical_close_force_max_command_lead: 0.004\n")
r.write_text(y)
print('ok')
