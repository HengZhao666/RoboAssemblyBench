"""Generalize the Plumbers continuous-physics branch to stool_circular."""
from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')

def sub(rel, old, new, count=1):
    p = R / rel
    t = p.read_text()
    n = t.count(old)
    assert n == count, (rel, old[:60], n)
    p.write_text(t.replace(old, new))

fc = 'roboassemblybench/core/fabrica_canonical.py'
sub(fc, "    plumbers_physics = assembly == 'plumbers_block' and bool(spec.get('continuous_physics', False))\n"
        "    if bool(spec.get('continuous_physics', False)) and not plumbers_physics:\n"
        "        raise ValueError('continuous_physics is supported for UR5e plumbers_block and the dedicated Beam compiler.')\n",
        "    # Stool reuses the generic Plumbers continuous-physics pass (pure physical\n"
        "    # grasps, no locks/pose writes); Beam keeps its dedicated compiler.\n"
        "    plumbers_physics = assembly in _GENERIC_PHYSICAL_ASSEMBLIES and bool(spec.get('continuous_physics', False))\n"
        "    if bool(spec.get('continuous_physics', False)) and not plumbers_physics:\n"
        "        raise ValueError('continuous_physics is supported for UR5e plumbers_block/stool_circular and the dedicated Beam compiler.')\n")
sub(fc, "    if assembly == 'plumbers_block' and spec.get('continuous_physics'):\n        from roboassemblybench.core.plumbers_physical import (\n            apply_plumbers_wait_step_limit,",
        "    if assembly in _GENERIC_PHYSICAL_ASSEMBLIES and spec.get('continuous_physics'):\n        from roboassemblybench.core.plumbers_physical import (\n            apply_plumbers_wait_step_limit,")
# module-level constant just before the compile entry point
t = (R / fc).read_text()
anchor = '\ndef compile_fabrica_canonical_recipe('
assert t.count(anchor) == 1
t = t.replace(anchor, "\n_GENERIC_PHYSICAL_ASSEMBLIES = frozenset({'plumbers_block', 'stool_circular'})\n\n" + anchor, 1)
(R / fc).write_text(t)

pp = 'roboassemblybench/core/plumbers_physical.py'
sub(pp, "    known = {entry['object'] for entry in success}\n",
        "    # Part ids are the trailing token of fabrica_<assembly>_<id> for every\n    # generic physical assembly (plumbers_block, stool_circular).\n"
        "    known_ids = {str(entry['object']).rsplit('_', 1)[-1] for entry in success}\n")
sub(pp, "            f'fabrica_plumbers_block_{part_id}' not in known\n", "            part_id not in known_ids\n", count=2)
sub(pp, "        key = entry['object'].removeprefix('fabrica_plumbers_block_')\n", "        key = str(entry['object']).rsplit('_', 1)[-1]\n")
sub(pp, "str(attach.get('object', '')).removeprefix('fabrica_plumbers_block_')", "str(attach.get('object', '')).rsplit('_', 1)[-1]", count=2)
sub(pp, "str(skill.get('object', '')).removeprefix('fabrica_plumbers_block_')", "str(skill.get('object', '')).rsplit('_', 1)[-1]")
sub(pp, "str(p.get('local_skill', {}).get('object', '')).removeprefix('fabrica_plumbers_block_')", "str(p.get('local_skill', {}).get('object', '')).rsplit('_', 1)[-1]")
assert 'fabrica_plumbers_block_' not in (R / pp).read_text().split('def apply_plumbers_physics')[1].split('add_plumbers_base_support')[0]

dp = 'toolkits/factory_dual_franka_assembly/demo_policy.py'
sub(dp, "        if not isinstance(recipe, str) or not recipe.startswith('fabrica_plumbers_block_ur5e_'):",
        "        if not isinstance(recipe, str) or not recipe.startswith(('fabrica_plumbers_block_ur5e_', 'fabrica_stool_circular_ur5e_')):")
print('patched')
