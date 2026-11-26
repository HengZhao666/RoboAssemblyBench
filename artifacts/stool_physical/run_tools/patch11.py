from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')
p = R / 'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
t = p.read_text()
old = "            'contact_box_offset',\n            'caging_contact_distance',\n"
assert t.count(old) == 1
p.write_text(t.replace(old, "            'contact_box_offset',\n            'contact_box_orientation',\n            'caging_contact_distance',\n"))
print('ok')
