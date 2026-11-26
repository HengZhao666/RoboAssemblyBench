"""Live read-only check of round 62 against round 61 (offsets also against round 55).

Before the first changed row (inside round 61's handover_left_2_unload_lift) every progress row
must equal round 61. Reports each phase start offset relative to round 55
(negative = earlier), the column-3 / cap-1 stack yaw relative to base
6 from the latest physics-trace row (traces start at step 45,000), and, in the
final verification, the latest installed-assembly support load record.
Usage: python prefix_check.py [round62_output_dir]
"""
import json
import math
import sys
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
R55 = R / 'candidate_repository/outputs/fabrica_beam_v232_window/progress_history.jsonl'
R61 = R / 'candidate_repository/outputs/fabrica_beam_v238_lift04/progress_history.jsonl'
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else R / 'candidate_repository/outputs/fabrica_beam_v239_unload'
CHANGED = 'handover_left_2_unload_lift'


def rows(path):
    out = []
    for line in Path(path).read_text().splitlines():
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            break
    return out


def starts(rs):
    out = {}
    for r in rs:
        out.setdefault(r['phase'], r['task_step'])
    return out


def tail_line(path, marker):
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 600000))
            lines = f.read().decode(errors='ignore').splitlines()[1:]
    except FileNotFoundError:
        return None
    for line in reversed(lines):
        if marker is None or marker in line[:160]:
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
    return None


def yaw(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


a, b, b55 = rows(OUT / 'progress_history.jsonl'), rows(R61), rows(R55)
first_diff = None
for i, (x, y) in enumerate(zip(a, b)):
    x2, y2 = dict(x), dict(y)
    for r in (x2, y2):
        r.pop('elapsed_wall_seconds', None)
    if x2 != y2:
        first_diff = i
        break
last = a[-1] if a else {}
changed_index = next((i for i, r in enumerate(b) if r['phase'] == CHANGED), None)
s56, s55 = starts(a), starts(b55)
offsets = {p: s56[p] - s55[p] for p in s56 if p in s55}
report = dict(rows=len(a), last_step=last.get('task_step'), last_phase=last.get('phase'),
              failed=last.get('failed'), success=last.get('success'),
              elapsed_wall_seconds=last.get('elapsed_wall_seconds'),
              steps_per_second=(round(last['task_step'] / last['elapsed_wall_seconds'], 2)
                                if last.get('elapsed_wall_seconds') else None),
              first_difference_row_vs_r61=first_diff, r61_changed_phase_row=changed_index,
              prefix_identical_so_far=first_diff is None or (changed_index is not None and first_diff >= changed_index),
              last_phase_offset_vs_r55=offsets.get(last.get('phase')),
              phase_start_offset_vs_r55={p: v for p, v in offsets.items() if v})
row = tail_line(OUT / 'physics_boundary_trace.jsonl', '"after_observation"')
if row:
    bodies = {x['name']: x for x in row['bodies']}
    base = yaw(bodies['fabrica_beam_6']['orientations'][0])
    report['stack'] = dict(step=row['task_step'],
                           column3_yaw_rel_base_mrad=round((yaw(bodies['fabrica_beam_3']['orientations'][0]) - base) * 1000, 2),
                           cap1_yaw_rel_base_mrad=round((yaw(bodies['fabrica_beam_1']['orientations'][0]) - base) * 1000, 2))
final = tail_line(OUT / 'final_pose_window_trace.jsonl', None)
if final:
    report['final_window'] = dict(count=final.get('count'), required=final.get('required_steps'),
                                  ready=final.get('ready'), qualified=final.get('qualified'),
                                  base_force=final.get('support', {}).get('vertical_force'),
                                  assembly_load={k: final.get('assembly_load', {}).get(k)
                                                 for k in ('valid', 'vertical_force', 'ready', 'error')})
print(json.dumps(report))
