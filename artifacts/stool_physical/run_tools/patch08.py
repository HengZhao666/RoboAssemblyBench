from pathlib import Path
r = Path('/home/user/work/zhaoheng/stool_physical_20261120/roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml')
y = r.read_text()
start = y.index("  physical_move_pad_centers_m:\n")
end = y.index("    '8': 0.145\n") + len("    '8': 0.145\n")
y = y[:start] + ("  # Moving parts keep Fabrica's pinch (170 mm): the 57 mm native pad then\n"
                 "  # spans the pinch with the fingertip only ~7 mm beyond it. A 145 mm pad\n"
                 "  # centre pushed the tips ~32 mm into the tray pockets (smoke09 part 3).\n") + y[end:]
r.write_text(y)
print('ok')
