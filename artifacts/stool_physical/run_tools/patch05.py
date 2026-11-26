from pathlib import Path
r = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
y = r.read_text()
old = "    gripper_pad_dynamic_friction: 0.8\n"
assert y.count(old) == 2
y = y.replace(old, old + "    # Compliant pads (Plumbers C01 base hand): a part held by its tray pocket\n"
                         "    # cannot self-centre; rigid pads turn ~0.2 mm offset into 0/19 N (smoke06).\n"
                         "    gripper_pad_contact_stiffness: 5000.0\n"
                         "    gripper_pad_contact_damping: 15.0\n")
r.write_text(y)
print('ok')
