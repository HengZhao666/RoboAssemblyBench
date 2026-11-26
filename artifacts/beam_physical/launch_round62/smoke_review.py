"""Review the round-62 launch-chain smoke (stop 420) against round 61's own rows.

Round 62 first changes inside handover_left_2_unload_lift (round-61 step 45,929), so every
smoke progress row before the terminal row must equal round 61 row for row.
Also checks identity/solver/asset admissions, the loaded module hashes and the
worker stats (velocity cache active, no fastcopy fallback). Not a formal round.
"""
import json
from pathlib import Path

Q = Path(__file__).resolve().parent
REF = Path('/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
           'fabrica_beam_v238_lift04/progress_history.jsonl')


def rows(path):
    out = []
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        row.pop('elapsed_wall_seconds', None)
        out.append(row)
    return out


def main():
    out = Q / 'smoke_out'
    a, b = rows(out / 'progress_history.jsonl'), rows(REF)
    body = a[:-1]
    episode = json.loads((out / 'episode_0000.json').read_text())['metrics']
    admissions = {k: json.loads((out / k).read_text()).get('passed') for k in (
        'runner_config_admission.json', 'runtime_solver_admission.json', 'native_scene_asset_admission.json',
        'passive_observer_close.json')}
    loaded = json.loads((out / 'runner_config_admission.json').read_text())['loaded_modules']
    stats = json.loads((out / 'opt_stats.json').read_text())
    report = dict(stop=420, rows=len(a), compared_rows=len(body), terminal=episode.get('terminal_reason'),
                  steps=episode.get('steps'), prefix_rows_identical_to_round61=body == b[:len(body)],
                  admissions=admissions, loaded_modules=loaded, opt_stats=stats, reference=str(REF),
                  scope='Launch-chain smoke; not a formal round, no assembly evidence. The only v239 behaviour change (handover '
                        'lift 0.2 mm, tolerance 0.4 mm) first acts at step 45,929; it is covered by '
                        'offline_v239_unload/config_check.json.')
    report['passed'] = bool(report['prefix_rows_identical_to_round61'] and all(admissions.values())
                            and report['terminal'] == 'smoke-step-limit' and stats['fastcopy_fallbacks'] == 0
                            and stats['velocity_cache_hits'] > 0 and 'support_state' in loaded and len(body) > 0)
    (Q / 'smoke_review.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'opt_stats'}, indent=1))


if __name__ == '__main__':
    main()
