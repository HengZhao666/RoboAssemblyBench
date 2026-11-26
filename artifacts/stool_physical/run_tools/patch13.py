"""Opt-in: build the strict opposing pair from measured PhysX contact points."""
from pathlib import Path
R = Path('/home/user/work/zhaoheng/stool_physical_20261120')
p = R / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
t = p.read_text()
old = """        strict_sample_pair = (
            min(strict_sample_pairs, key=lambda item: item['combined_surface_gap']) if strict_sample_pairs else None
        )
"""
assert t.count(old) == 1
new = """        if not strict_sample_pairs and bool(attach_spec.get('strict_contact_from_force_points', False)):
            force_pair = self._force_point_strict_pair(
                left_finger_metrics,
                right_finger_metrics,
                contact_metrics,
                allowed_axes=allowed_axes,
                surface_gap_limit=strict_surface_gap,
                interior_margin=interior_margin,
            )
            if force_pair is not None:
                strict_sample_pairs.append(force_pair)
        strict_sample_pair = (
            min(strict_sample_pairs, key=lambda item: item['combined_surface_gap']) if strict_sample_pairs else None
        )
"""
t = t.replace(old, new)
anchor = "    def _strict_physical_grasp_contact(  # noqa: C901\n"
assert t.count(anchor) == 1
helper = '''    @staticmethod
    def _force_point_strict_pair(left, right, contact_metrics, *, allowed_axes, surface_gap_limit, interior_margin):
        """Opposing face pair from the measured PhysX finger contact points.

        Used only when opted in. The modelled pad sample grid can lie well above
        a short part whose exposed top is pinched by the pad tips; the actual
        contact points are then the faithful geometry. Each finger needs valid,
        non-truncated force details; its force-weighted point must lie on one
        face of the same box axis (opposite signs), within the surface-gap limit
        of that face and inside the face extents on the other two axes.
        """
        center = contact_metrics.get('contact_box_center')
        orientation = contact_metrics.get('contact_box_orientation')
        scale = contact_metrics.get('contact_box_scale')
        if center is None or orientation is None or scale is None:
            return None
        half = 0.5 * np.asarray(scale, dtype=float).reshape(-1)
        if half.size != 3:
            return None
        w, x, y, z = (float(v) for v in orientation)
        rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        center = np.asarray(center, dtype=float)

        def weighted_point(metric):
            observation = (metric or {}).get('force_observation') or {}
            if not observation.get('valid') or not observation.get('details_valid') or observation.get('details_at_capacity'):
                return None
            points, weights = [], []
            for contact in observation.get('contacts') or []:
                force = abs(float(contact.get('normal_force', 0.0)))
                point = contact.get('point_world')
                if force > 0.01 and point is not None:
                    points.append(rotation.T @ (np.asarray(point, dtype=float) - center))
                    weights.append(force)
            if not points:
                return None
            return np.average(np.asarray(points), axis=0, weights=np.asarray(weights))

        left_point, right_point = weighted_point(left), weighted_point(right)
        if left_point is None or right_point is None:
            return None
        best = None
        for axis in allowed_axes:
            index = {'x': 0, 'y': 1, 'z': 2}.get(axis)
            if index is None or left_point[index] * right_point[index] >= 0.0:
                continue
            gaps = [abs(half[index] - abs(float(point[index]))) for point in (left_point, right_point)]
            if max(gaps) > surface_gap_limit:
                continue
            inside = all(
                abs(float(point[other])) <= half[other] + interior_margin
                for point in (left_point, right_point)
                for other in range(3)
                if other != index
            )
            if not inside:
                continue
            candidate = {
                'axis': axis,
                'combined_surface_gap': float(sum(gaps)),
                'left_local_point': left_point.tolist(),
                'right_local_point': right_point.tolist(),
                'source': 'measured_force_points',
            }
            if best is None or candidate['combined_surface_gap'] < best['combined_surface_gap']:
                best = candidate
        return best

'''
t = t.replace(anchor, helper + anchor)
p.write_text(t)

s = R / 'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
u = s.read_text()
old = "            'contact_box_orientation',\n            'caging_contact_distance',\n"
assert u.count(old) == 1
s.write_text(u.replace(old, "            'contact_box_orientation',\n            'strict_contact_from_force_points',\n            'caging_contact_distance',\n"))

pp = R / 'roboassemblybench/core/plumbers_physical.py'
v = pp.read_text()
old = "    spec['_contact_box_alignment_audit'] = audit\n"
assert v.count(old) == 1
v = v.replace(old, old + '''

def apply_force_point_strict_contact(phases: list[dict], spec: dict) -> None:
    """Opt-in measured-contact-point strict pair for listed parts (see task helper)."""
    parts = spec.get('physical_strict_contact_from_force_points_parts', [])
    if not isinstance(parts, list) or not all(isinstance(x, str) for x in parts):
        raise ValueError('physical_strict_contact_from_force_points_parts must list part ids')
    for part_id in parts:
        touched = 0
        for phase in phases:
            for entry in [*phase.get('attach', []), phase.get('local_skill') or {}]:
                if str(entry.get('object', '')).endswith(f'_{part_id}') and 'contact_box_scale' in entry:
                    entry['strict_contact_from_force_points'] = True
                    touched += 1
        if not touched:
            raise ValueError(f'part {part_id}: no contact entries for force-point strict contact')
''')
pp.write_text(v)

fc = R / 'roboassemblybench/core/fabrica_canonical.py'
w = fc.read_text()
old = "        align_contact_boxes_to_grasps(phases, task, spec)\n"
assert w.count(old) == 1
w = w.replace(old, old + "        from roboassemblybench.core.plumbers_physical import apply_force_point_strict_contact\n\n"
                         "        apply_force_point_strict_contact(phases, spec)\n")
fc.write_text(w)

r = R / 'roboassemblybench/tasks/fabrica_stool_circular_ur5e_physical/recipe.yaml'
y = r.read_text()
start = y.index("  # Caps (18x18x30 mm) sit in tray pockets")
end = y.index("    '8': 0.160\n") + len("    '8': 0.160\n")
y = y[:start] + """  # Caps (18x18x30 mm) sit in tray pockets with only their tops exposed, so
  # the pads grip the top ~3 mm band of two opposing faces (PhysX contacts at
  # z=12..15 mm of a 15 mm half-height, smoke12). 10 mm deeper hits the tray
  # (smoke14: zero-force stall). The modelled pad samples lie 35-50 mm above the
  # cap, so qualify the opposing pair from the measured contact points instead.
  physical_strict_contact_from_force_points_parts: ['6', '4', '0', '8']
""" + y[end:]
r.write_text(y)
print('ok')
