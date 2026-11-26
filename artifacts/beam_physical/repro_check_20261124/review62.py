"""Manual review of round 62 (v239_unload) and ledger close-out. Read-only on results.

Writes round62_v239_unload/manual_review.json, backs up the ledger, then marks
round 59 reviewed_closed with full_assembly_success. Starts nothing.
"""
import datetime
import gzip
import hashlib
import json
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
Q = B / 'round62_v239_unload'
OUT = C / 'outputs/fabrica_beam_v239_unload'
OUT55 = C / 'outputs/fabrica_beam_v232_window'
BATCH = B / 'batch62_66_20261124'
P = B / 'offline_v239_unload'
LEDGER = B / 'round_ledger.json'


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 22), b''):
            h.update(block)
    return h.hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def starts(path):
    d = json.loads(path.read_text() if path.exists() else gzip.open(str(path) + '.gz', 'rt').read())
    d = d[0] if isinstance(d, list) else d
    m = d.get('metrics', d)
    return {t['to_phase']: t['step_counter'] for t in m['phase_transition_history']
            if t.get('transition_type') == 'advance'}


def main():
    episode = json.loads((OUT / 'episode_0000.json').read_text())
    m = episode['metrics']
    terminal = m['phase_transition_history'][-1]
    progress = [json.loads(x) for x in (OUT / 'progress_history.jsonl').read_text().splitlines()]
    wall = progress[-1]['elapsed_wall_seconds']
    final = m['beam_final_acceptance']
    objects = final['motion']['objects']
    diag = {d['object']: d for d in m['success_diagnostics']}
    s62 = {t['to_phase']: t['step_counter'] for t in m['phase_transition_history'] if t.get('transition_type') == 'advance'}
    s55 = starts(OUT55 / 'collect_results.json')
    keys = ['assemble_00_part_3_descend', 'assemble_01_part_2_descend', 'assemble_02_part_1_descend',
            'assemble_03_part_0_descend', 'assemble_03_part_0_release', 'hold_2_final_release',
            'hold_2_final_retreat', 'beam_final_unloaded_verification']
    videos = {p.name: dict(sha256=sha(p), bytes=p.stat().st_size)
              for p in sorted((OUT / 'episode_0000_live_videos').glob('*.mp4'))}
    writes = json.loads((OUT / 'state_write_counts.json').read_text())
    review = dict(
        round=62, candidate='v239_unload', reviewed_at_utc=now(), reviewer='Claude Code (claude-opus-5-5)',
        exit_code=int((OUT / 'exit_code').read_text().strip()), finished_at=(OUT / 'finished_at').read_text().strip(),
        originals={n: dict(sha256=sha(OUT / n), bytes=(OUT / n).stat().st_size)
                   for n in ('collect_results.json', 'episode_0000.json', 'progress_history.jsonl', 'state_write_counts.json')},
        videos=videos,
        metrics=dict(success=m['success'], failed=m['failed'], steps=m['steps'], terminal_reason=m['terminal_reason'],
                     timeout_count=m['timeout_count'], recovery_count=m['recovery_count']),
        terminal=dict((k, terminal.get(k)) for k in ('from_phase', 'to_phase', 'reason', 'step_counter', 'phase_step_counter')),
        prefix=dict(identical_to_round61_until='inside handover_left_2_unload_lift (step ~45,930)'),
        fixed_criteria=json.loads((P / 'analysis/fixed_all.json').read_text()),
        final_verification=dict(
            mode=final['mode'], count=final['count'], required=final['required_steps'], qualified=final['qualified'],
            phase_steps=terminal.get('phase_step_counter'),
            pose_drift_from_window_reference={k: dict(position_mm=round(v.get('position_drift', 0) * 1000, 3),
                                                      orientation_rad=round(v.get('orientation_drift', 0), 4))
                                              for k, v in objects.items()},
            targets={k: dict(position_error_mm=round(d['position_error'] * 1000, 3),
                             orientation_error_rad=round(d['orientation_error'], 4), pose_passed=d['pose_passed'],
                             released_passed=d['released_passed'], raw_static_passed=d.get('raw_static_passed'))
                     for k, d in diag.items()},
            raw_speeds_reported_separately={k: dict(linear=round(v['linear_speed'], 4), angular=round(v['angular_speed'], 3))
                                            for k, v in objects.items()},
            note='success by the human-approved fixed-reference pose window; with column 2 unloaded (0 N) before part 0 '
                 'the stack is static after both arms leave (part 0 8-step motion 0.015-0.02 mm, round 59: 0.22-0.38) and '
                 'the final window drift of part 0 is 0.109 mm / 0.0012 rad (round 59: 1.40 mm / 0.0286 rad). Part 0 still '
                 'reads ~4.1 N side load while both hands hold during the hold check (round 59 3.4 N, round 55 0 N); after '
                 'release it settles at ~1.1 N (round 55 ~1.0 N).'),
        progress=dict(wall_seconds=wall, steps_per_second=round(m['steps'] / wall, 2),
                      phase_starts=s62, round55_phase_starts={k: s55.get(k) for k in keys},
                      steps_saved_vs_round55={k: (s55[k] - s62[k]) if k in s55 and k in s62 else None for k in keys}),
        state_write_forbidden_after_init=writes.get('forbidden_calls_after_scene_initialization'),
        full_assembly_success=True, primary_task_success=True)
    write(Q / 'manual_review.json', review)

    raw = LEDGER.read_bytes()
    (BATCH / 'ledger_before_review62.json').write_bytes(raw)
    ledger = json.loads(raw)
    assert ledger['active_round'] is None
    row = next(r for r in ledger['rounds'] if r['round'] == 62)
    assert row['status'] == 'native_finished_pending_review', row['status']
    row.update(status='reviewed_closed', full_assembly_success=True, primary_task_success=True,
               terminal_reason=m['terminal_reason'], reviewed_at=review['reviewed_at_utc'],
               review=str(Q / 'manual_review.json'))
    batch = ledger['current_batch']
    assert batch['round_numbers'] == [62, 66]
    batch.update(stopped_after_round=62, stop_reason='fixed (human stop rule: 授权五轮，或修好停止)', remaining_rounds_unused=batch.get('remaining_rounds'))
    ledger.update(status='round62_reviewed_fixed_batch_stopped', updated_at=now(), authorization_exhausted=True,
                  stop_reason='Batch 62-66 stopped after round 62: repaired (full assembly success, stack static, final window margin restored); new runs need a new human instruction.',
                  repaired_full_assembly=dict(round=62, candidate='v239_unload', review=str(Q / 'manual_review.json')))
    write(LEDGER, ledger)
    print(json.dumps(dict(reviewed=True, terminal=review['terminal'], final=review['final_verification'],
                          progress={k: review['progress'][k] for k in ('wall_seconds', 'steps_per_second', 'steps_saved_vs_round55')},
                          videos=videos, forbidden_writes=review['state_write_forbidden_after_init']), ensure_ascii=False))


if __name__ == '__main__':
    main()
