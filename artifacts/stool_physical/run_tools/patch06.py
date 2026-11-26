from pathlib import Path
r = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
y = r.read_text()
old = "    gripper_pad_contact_damping: 15.0\n"
assert y.count(old) == 2 and 'arm_drive_kp' not in y
y = y.replace(old, old + "    # Stool S23 tracking fix: default 80000/4000 leaves ~0.3 mm steady TCP\n"
                         "    # error, which stalls the 0.35 mm close-recentre servo (smoke07 part 3).\n"
                         "    arm_drive_kp: 160000.0\n"
                         "    arm_drive_kd: 5656.85424949238\n")
r.write_text(y)
print('ok')
