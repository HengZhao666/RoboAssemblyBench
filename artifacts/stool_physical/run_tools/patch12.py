from pathlib import Path
r = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
y = r.read_text()
anchor = "  # Moving parts keep Fabrica's pinch (170 mm)"
assert y.count(anchor) == 1 and 'physical_move_pad_centers_m' not in y
y = y.replace(anchor, """  # Caps (18x18x30 mm) sit in tray pockets with only their tops exposed. At
  # the 170 mm Fabrica pinch only the pad tips touch the top edge (smoke12
  # cap 6: 4/8 N but every pad sample 35-50 mm above the cap box). Go 10 mm
  # deeper and check the native finger sweep against the tray first.
  physical_move_pad_centers_m:
    '6': 0.160
    '4': 0.160
    '0': 0.160
    '8': 0.160
""" + anchor)
old = "    2: {fixture: fabrica_fixture, margin_m: 0.0015, joint_step: 0.005, preload_joint_travel: 0.01}\n"
assert y.count(old) == 1
y = y.replace(old, old.replace("    2:", "    '2':") + "".join(
    f"    '{p}': {{fixture: fabrica_fixture, margin_m: 0.0015, joint_step: 0.005, preload_joint_travel: 0.01}}\n"
    for p in ('6', '4', '0', '8')))
y += """
# Debug-only (smoke tests, never a formal round): pick cap 6 right after the
# base to test cap grasps in ~12 min. Its insertion cannot succeed (no leg).
profiles:
  cap_first:
    task_name: fabrica_stool_circular_ur5e_physical_cap_first
    fabrica_canonical:
      assembly_step_order: ['6', '3', '7', '4', '5', '1', '0', '8']
"""
r.write_text(y)
print('ok')
