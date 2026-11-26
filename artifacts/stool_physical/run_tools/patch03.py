from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')
pp = R / 'roboassemblybench/core/plumbers_physical.py'
t = pp.read_text()
old = "        'close_force_max_seek': 0.03,\n"
assert t.count(old) == 1
t = t.replace(old, "        # Joint travel allowed past the nominal close preset while seeking first\n"
                   "        # contact. Wide stool base grasps (75 mm, near full open) need more.\n"
                   "        'close_force_max_seek': _close_max_seek(spec),\n")
anchor = "def apply_plumbers_physics("
t = t.replace(anchor, '''def _close_max_seek(spec: dict) -> float:
    value = spec.get('physical_close_force_max_seek', 0.03)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0.0 < float(value) <= 0.15:
        raise ValueError('physical_close_force_max_seek must be in (0, 0.15] rad.')
    return float(value)


''' + anchor, 1)
pp.write_text(t)
rp = R / 'roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml'
y = rp.read_text()
old = """  # Fabrica pinch points sit 160 mm (base) / 170 mm (moving parts) from the
  # converted Robotiq frame; the native Isaac pads are centred at 143 mm.
  # Shift every grasp along its approach so the pads land on the pinch point.
  physical_base_pad_center_m: 0.143
  physical_move_pad_centers_m:
"""
assert y.count(old) == 1
y = y.replace(old, """  # Fabrica pinch points sit 160 mm (base) / 170 mm (moving parts) from the
  # converted Robotiq frame. Native USD finger FK (outputs/stool_physical/
  # pad_fk.py) puts the inner-pad centroid ~121 mm (75 mm gap) / ~130 mm
  # (18 mm gap) from base_link; Plumbers' 143 mm at a 40 mm gap fixes the
  # control-frame offset at ~15 mm. Shift grasps so pads land on the pinch.
  physical_base_pad_center_m: 0.136
  # Seek past the nominal preset: the 75 mm base needs q ~0.131 vs 0.098.
  physical_close_force_max_seek: 0.08
  physical_move_pad_centers_m:
""")
y = y.replace("    '3': 0.143\n    '7': 0.143\n    '6': 0.143\n    '4': 0.143\n    '5': 0.143\n    '1': 0.143\n    '0': 0.143\n    '8': 0.143\n",
              "    '3': 0.145\n    '7': 0.145\n    '6': 0.145\n    '4': 0.145\n    '5': 0.145\n    '1': 0.145\n    '0': 0.145\n    '8': 0.145\n")
assert y.count("0.145") == 8
rp.write_text(y)
print('ok')
