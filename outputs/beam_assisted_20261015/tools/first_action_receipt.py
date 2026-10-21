"""Read-only receipt of published commands, SDK acceptance, and native growth."""
from pathlib import Path
import json
import sys
import time
from beam_workflow import record, save, sha


def rows(path, limit):
    with path.open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            step = row.get('step', row.get('cycle_step', 0))
            if step >= limit:
                break
            yield row


def main():
    p = Path(__file__).resolve().parents[1]
    run, cycle = sys.argv[1:]
    d = p / 'runs' / run
    started = time.monotonic()
    record(p / 'batch_D_20261016', cycle + '_first_action_receipt', 'analysis', 'start')
    published = {(r['step'], r['robot']): r['published_q']
                 for r in rows(d / 'assisted_trace.jsonl', 160)
                 if r.get('event') == 'published_command'}
    checks = []
    actual = {}
    maximum = 0.
    for row in rows(d / 'physics_boundary_trace.jsonl', 800):
        for robot, state in row['robots'].items():
            q = state['joint_positions']
            # Native articulation records retain their one-environment axis.
            assert len(q) == 1 and len(q[0]) >= 6, 'unexpected native joint layout'
            actual.setdefault(robot, []).append(q[0][:6])
            key = (row['cycle_step'], robot)
            if row['boundary'] != 'after_action' or key not in published:
                continue
            sdk = state['sdk_submission']
            arm = next(a for a in sdk['actions'] if a.get('joint_indices') == list(range(6)))
            error = max(abs(a-b) for a, b in zip(published[key], arm['joint_positions']))
            assert error < 1e-12 and sdk['batch_sequence'] > 0
            maximum = max(maximum, error)
            checks.append(dict(step=key[0], robot=robot, physics_index=row['physics_step_index'],
                               batch_sequence=sdk['batch_sequence']))
    assert len(checks) >= 8, 'missing actual accepted command samples'
    movement = {robot: max(max(abs(a-b) for a, b in zip(q, values[0]))
                           for q in values) for robot, values in actual.items()}
    assert len(movement) == 2 and all(v > 1e-4 for v in movement.values()), 'missing measured arm motion'
    entities = list(rows(d / 'entity_geometry_trace.jsonl', 160))
    assert len(entities) >= 2 and entities[-1]['step'] > entities[0]['step']
    assert all(len(r['body_poses']) == 44 for r in entities)
    progress = json.loads((d / 'visible_mesh_guard_progress.json').read_text())
    assert progress['samples'] > 0 and progress['last_step'] > 0 and progress['failures'] == 0
    receipt = dict(passed=True, run=run, tool_sha256=sha(__file__),
                   completed_SDK_submission_checks=len(checks), maximum_published_to_SDK_delta_rad=maximum,
                   actual_commands=checks, actual_arm_max_movement_rad=movement,
                   entity_rows_checked=len(entities), entity_pose_count=44, CAD_progress=progress,
                   elapsed_s=time.monotonic()-started,
                   scope='Real first accepted SDK commands, native arm motion and subsequent entity/CAD growth. No assembly or full tracking qualification.')
    save(p / (cycle + '_first_action_receipt.json'), receipt)
    record(p / 'batch_D_20261016', cycle + '_first_action_receipt', 'analysis', 'end',
           elapsed_s=receipt['elapsed_s'], passed=True)
    print(json.dumps({k: v for k, v in receipt.items() if k != 'actual_commands'}))


if __name__ == '__main__':
    main()
