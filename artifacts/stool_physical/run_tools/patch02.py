from pathlib import Path
p = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
t = p.read_text()
anchor = "fabrica_canonical:\n  continuous_physics: true\n"
assert t.count(anchor) == 1 and 'physical_base_pad_center_m' not in t
t = t.replace(anchor, anchor + """  # Fabrica pinch points sit 160 mm (base) / 170 mm (moving parts) from the
  # converted Robotiq frame; the native Isaac pads are centred at 143 mm.
  # Shift every grasp along its approach so the pads land on the pinch point.
  physical_base_pad_center_m: 0.143
  physical_move_pad_centers_m:
    '3': 0.143
    '7': 0.143
    '6': 0.143
    '4': 0.143
    '5': 0.143
    '1': 0.143
    '0': 0.143
    '8': 0.143
""")
p.write_text(t)
print('ok')
