from pathlib import Path
r = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
y = r.read_text()
old = ("    # Stool S23 tracking fix: default 80000/4000 leaves ~0.3 mm steady TCP\n"
       "    # error, which stalls the 0.35 mm close-recentre servo (smoke07 part 3).\n"
       "    arm_drive_kp: 160000.0\n"
       "    arm_drive_kd: 5656.85424949238\n")
assert y.count(old) == 2
new = ("    # Gravity sag of the position drive scales 1/kp: 80000 -> 0.42 mm,\n"
       "    # 160000 (S23) -> 0.22 mm at the leg grasp, enough to give single-finger\n"
       "    # contact on pocket-held parts (smoke07/08). 4x S23, same kd=sqrt(200 kp).\n"
       "    arm_drive_kp: 640000.0\n"
       "    arm_drive_kd: 11313.708498984761\n")
y = y.replace(old, new)
r.write_text(y)
print('ok')
