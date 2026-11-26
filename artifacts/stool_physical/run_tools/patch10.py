"""Opt-in contact box aligned with the selected grasp's pinch (hex stool legs)."""
from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')

# 1) Task: optional part-local box orientation composed onto the object pose.
p = R / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
t = p.read_text()
old = """        contact_box_offset = attach_spec.get('contact_box_offset')
        if contact_box_offset is not None:
            object_position = object_position + quat_rotate(
                object_orientation,
                np.asarray(contact_box_offset, dtype=float),
            )
        return object_position, object_orientation
"""
assert t.count(old) == 1
new = """        contact_box_offset = attach_spec.get('contact_box_offset')
        if contact_box_offset is not None:
            object_position = object_position + quat_rotate(
                object_orientation,
                np.asarray(contact_box_offset, dtype=float),
            )
        # Optional box rotation in the part frame (wxyz). Prismatic parts whose
        # pinched flats are not box faces (hexagonal stool legs) align the box
        # with the selected grasp; omitted keeps the original object-aligned box.
        local_orientation = attach_spec.get('contact_box_orientation')
        if local_orientation is not None:
            local_orientation = np.asarray(local_orientation, dtype=float).reshape(-1)
            if local_orientation.size != 4 or not np.all(np.isfinite(local_orientation)):
                raise ValueError('contact_box_orientation must be a finite wxyz quaternion')
            object_orientation = quat_multiply(object_orientation, local_orientation / np.linalg.norm(local_orientation))
        return object_position, object_orientation
"""
t = t.replace(old, new)
p.write_text(t)
assert 'quat_multiply' in t.split('def _contact_box_pose')[0], 'quat_multiply not imported in task module'

# 2) Compiler: align boxes after the physical pass, using the selected grasps.
pp = R / 'roboassemblybench/core/plumbers_physical.py'
t = pp.read_text()
t += '''

def align_contact_boxes_to_grasps(phases: list[dict], task: dict, spec: dict) -> None:
    """Rotate listed parts' contact boxes so box +y is the selected pinch axis.

    The strict contact model classifies finger samples against box faces. A
    hexagonal prism pinched across flats at 60 degrees to its bounding box
    shows one finger on box x and the other on box y and is rejected, even
    with opposing force contact. Only the box frame and pinch width change;
    gaps, forces, opposing-side and interior-patch requirements are unchanged.
    """
    parts = spec.get('physical_contact_box_align_to_grasp_parts', [])
    if not isinstance(parts, list) or not all(isinstance(x, str) for x in parts):
        raise ValueError('physical_contact_box_align_to_grasp_parts must list part ids')
    if not parts:
        return
    steps = {str(step['move_part']): step for step in task['assembly_steps']}
    audit = {}
    for part_id in parts:
        if part_id not in steps:
            raise ValueError(f'physical_contact_box_align_to_grasp_parts: unknown moving part {part_id}')
        grasp = steps[part_id]['move_grasp']
        axis = quat_rotate(grasp['tcp_in_assembly_orientation'], [0.0, 1.0, 0.0])
        if abs(axis[2]) > 0.2:
            raise ValueError(f'part {part_id}: pinch axis is not lateral to the part z axis')
        yaw = math.atan2(axis[1], axis[0]) - math.pi / 2.0
        orientation = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        width = float(grasp['grasp_width_m'])
        name_suffix = f'_{part_id}'
        touched = 0
        for phase in phases:
            for entry in [*phase.get('attach', []), phase.get('local_skill') or {}]:
                if not str(entry.get('object', '')).endswith(name_suffix) or 'contact_box_scale' not in entry:
                    continue
                scale = [float(v) for v in entry['contact_box_scale']]
                lateral = max(scale[0], scale[1])
                entry['contact_box_orientation'] = list(orientation)
                entry['contact_box_scale'] = [lateral, width, scale[2]]
                touched += 1
        if not touched:
            raise ValueError(f'part {part_id}: no contact box entries to align')
        audit[part_id] = dict(pinch_axis_in_part=[float(v) for v in axis], box_yaw_rad=yaw,
                              pinch_width_m=width, entries=touched)
    spec['_contact_box_alignment_audit'] = audit
'''
pp.write_text(t)

fc = R / 'roboassemblybench/core/fabrica_canonical.py'
t = fc.read_text()
old = "        apply_plumbers_physics(phases, success=success, spec=spec)\n"
assert t.count(old) == 1
t = t.replace(old, old + "        from roboassemblybench.core.plumbers_physical import align_contact_boxes_to_grasps\n\n"
                         "        align_contact_boxes_to_grasps(phases, task, spec)\n")
fc.write_text(t)

r = R / 'roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml'
y = r.read_text()
anchor = "  # Moving parts keep Fabrica's pinch (170 mm)"
assert y.count(anchor) == 1
y = y.replace(anchor, "  # Hexagonal legs: align the strict-contact box with the pinched flats\n"
                      "  # (smoke11: left finger on box x, right on box y, both 4.5 N opposing).\n"
                      "  physical_contact_box_align_to_grasp_parts: ['3', '7', '5', '1']\n" + anchor)
r.write_text(y)
print('ok')
