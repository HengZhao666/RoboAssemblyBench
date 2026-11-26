import sys, math
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
names = [ph['name'] for ph in p['phases']]
i = names.index('assemble_03_part_4_pickup_clearance')
print(names[i:i + 12])
byn = {ph['name']: ph for ph in p['phases']}
a = byn['assemble_03_part_4_descend']['local_skill']['grasp_relative_position']
b = byn['assemble_03_part_4_regrasp_descend']['local_skill']['grasp_relative_position']
print('shift mm', round(1000 * math.dist(a, b), 3))
print('attach offset tray', byn['assemble_03_part_4_close_and_attach']['attach'][0]['target']['offset'])
print('attach offset regrasp', byn['assemble_03_part_4_regrasp_close_and_attach']['attach'][0]['target']['offset'])
tg = {t['name']: t for t in p['targets']}
print('staged', tg['part_4_staged']['position'], 'hover', tg['part_4_stage_hover']['position'])
rb = [ph for ph in p['phases'] if ph['name'] == 'assemble_03_part_4_rebase_to_socket'][0]['rebase_from_object']['targets']
print('rebase targets', rb)
print('phases', len(names), 'max_steps', p['max_steps'])
