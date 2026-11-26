"""Write round-62 records, open the five-round batch [62, 66] and set the row to ready_for_native.

Run once on the server after the smoke check passes AND after the human
go-ahead has been written to batch62_66_20261124/human_authorization.json
(fields: instruction, human_answer, at_utc). Backs up the ledger first.
Starts no simulation.
"""
import datetime
import hashlib
import json
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
Q = B / 'round62_v239_unload'
Q61 = B / 'round61_v238_lift04'
P = B / 'offline_v239_unload'
BATCH = B / 'batch62_66_20261124'
OUT = C / 'outputs/fabrica_beam_v239_unload'
LEDGER = B / 'round_ledger.json'
HUMAN = BATCH / 'human_authorization.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def main():
    human = json.loads(HUMAN.read_text())
    assert human['instruction'].strip() and human['human_answer'].strip() and human['at_utc']
    smoke = json.loads((Q / 'smoke_review.json').read_text())
    assert smoke['passed'], smoke
    config = json.loads((P / 'config_check.json').read_text())
    gate = json.loads((P / 'gate_check.json').read_text())
    ab = json.loads((P / 'velocity_cache_ab.json').read_text())
    assert config['passed'] and gate['passed'] and ab['passed']
    assert sha(Q / 'production_worker_config.json') == config['candidate_config_sha256']
    review = dict(
        passed=True, round=62, candidate='v239_unload', base_candidate='v238_lift04',
        changed_source_files=['roboassemblybench/core/beam_physical_compiler.py (vs round 61: lift completion '
                              'tolerance max(lift / 2, 0.4 mm); 1 mm default unchanged)'],
        changed_tools=['tools/preflight.py (BEAM_HANDOVER_UNLOAD_LIFT_M=0.0002)'],
        changed_observers=['diagnostics restored to round 59: full contact and physics traces every 8 steps from '
                           'step 45,000, hold trace on (bytes only)'],
        source_manifest_sha256=sha(Q / 'source_manifest.json'),
        worker_config_sha256=sha(Q / 'production_worker_config.json'),
        recipe_fingerprint=json.loads((Q / 'production_worker_config.json').read_text())['recipe_fingerprint'],
        human_pose_acceptance_approved=True,
        human_instruction=dict(path=str(HUMAN), instruction=human['instruction'], answer=human['human_answer'],
                               at_utc=human['at_utc']),
        diagnosis=[str(Q61 / 'manual_review.json'), str(P / 'analysis')],
        analysis=dict(
            heights='rounds 55 and 59 held column 2 at the same height during part 0 (-0.005 / +0.017 mm); part 0 '
                    'settled at -0.10 mm in both; rounds 60/61 lifted it +0.95 / +0.35 mm.',
            lateral='part 0 is servoed to the same nominal xy in both rounds; column 2 sat (-0.09, -0.53) mm (r55) '
                    'vs (-0.20, +0.29) mm (r59) off nominal: ~0.4 mm misalignment in both.',
            mechanism='round 55: 0 N on the base, column hanging on the compliant right-hand hold (500 N/m) could '
                      'yield sideways; round 59: 2.3 N base load locked it by friction, part 0 jammed (3.4 N side, '
                      'tilt 3.2 -> 9.2 mrad) and kept rocking.',
            unload='0.1 mm of rise already reads 0 N on the base (rounds 60/61); the right-hand handover sags '
                   '0.056 mm; the loaded left TCP settles 0.255 mm below its command (rounds 60/61).',
            prediction='0.2 mm command -> ~0.15 mm rise, ~0.1 mm after the handover, 0 N base load; part 0 meets the '
                       'column ~0.1 mm earlier than in round 59 (within the 0.25-0.55 mm hold-check settling).'),
        change=dict(lift='0.4 -> 0.2 mm; completion tolerance 0.2 -> 0.4 mm', kept='all round-61 changes'),
        aim='repair: part 0 seats without side jam on an unloaded, compliant column; full assembly success with '
            'part 0 not rocking.',
        fixed_criteria=['full assembly success', 'part 0 side load on column 2 < 1 N at the hold check',
                        'no sustained part-0 rocking after both arms leave',
                        'final window drift of parts 0/2 well below 2 mm / 0.03 rad'],
        falsifiable=['progress rows before the lift ends equal round 61',
                     'column 2 rise ~0.15 mm and 0 N base load after the lift; ~0.1 mm after the handover',
                     'left finger force on part 0 < 40 N; part 0 side load < 1 N at the hold check'],
        contingency='if part 0 still jams sideways: align the part-0 insertion target with the measured column-2 xy '
                    '(controller change); if it seats but the stack still rocks: examine the final release.',
        qualification_paths=dict(config_check=str(P / 'config_check.json'), gate_check=str(P / 'gate_check.json'),
                                 velocity_cache_ab=str(P / 'velocity_cache_ab.json'),
                                 production_preflight=str(Q / 'production_preflight.json'),
                                 phase_contract_preflight=str(Q / 'phase_contract_preflight.json'),
                                 smoke_review=str(Q / 'smoke_review.json')),
        rollback=dict(compiler=str(P / 'compiler_before62.py'), compiler_sha256=sha(P / 'compiler_before62.py')),
        not_changed='every phase other than the lift parameters, final unloaded verification, all gates',
        unverified=['whether the unloaded column really yields sideways as part 0 aligns'])
    write(Q / 'candidate_review.json', review)

    contract = json.loads((Q61 / 'experiment_contract.json').read_text())
    contract['round'] = 62
    contract['candidate'] = 'v239_unload'
    contract['handover_unload_lift'] = 'left hand lifts column 2 by 0.2 mm (unload) after the handover grasp'
    write(Q / 'experiment_contract.json', contract)

    tools = sorted(p for p in (Q / 'tools').iterdir() if p.suffix == '.py')
    write(Q / 'tools_manifest.json', dict(files=[dict(path=f'tools/{p.name}', sha256=sha(p)) for p in tools]))
    write(Q / 'launch_identity_manifest.json', dict(files=[dict(path=n, sha256=sha(Q / n)) for n in
          ('checked_worker.py', 'launch_round62.py', 'run_full_assembly.sh')]))

    raw = LEDGER.read_bytes()
    (BATCH / 'ledger_before_round62.json').write_bytes(raw)
    ledger = json.loads(raw)
    assert ledger['active_round'] is None and ledger['next_round'] == 62
    assert ledger['status'] == 'round61_reviewed' and ledger['authorization_exhausted'] is True
    assert not any(r['round'] == 62 for r in ledger['rounds'])
    assert ledger['final_support_load_approval']['applies_from_round'] == 56
    old = ledger['current_batch']
    assert old['round_numbers'] == [61, 61] and old['remaining_rounds'] == 0
    ledger['authorization_history'].append(dict(date='2026-11-24', text=human['instruction'],
        answer=human['human_answer'], supersedes='exhausted batch 61; new five-round batch 62-66 (stop when fixed)',
        backup=str(BATCH / 'ledger_before_round62.json')))
    ledger['previous_batch_61'] = old
    ledger['current_batch'] = dict(
        date='2026-11-24', maximum_rounds=5, completed_rounds=0, remaining_rounds=5, round_numbers=[62, 66],
        rounds_authorized=[62, 63, 64, 65, 66], stop_rule='stop when fixed (candidate_review fixed_criteria)', directory=str(BATCH), authorization_record=str(HUMAN),
        authorization=human['instruction'], human_answer=human['human_answer'], paused=False,
        unstarted_rounds_on_hold=[], currently_resumed_formal_rounds=[62, 63, 64, 65, 66])
    ledger.update(status='round62_ready', updated_at=now(), maximum_rounds=66, authorization_exhausted=False,
                  native_continuation_authorized=True, stop_reason=None,
                  latest_authorization=human['instruction'], latest_human_instruction=human['instruction'],
                  latest_human_instruction_at_utc=human['at_utc'])
    ledger['rounds'].append(dict(
        round=62, candidate='v239_unload', directory=str(Q), output=str(OUT), status='ready_for_native',
        native_started=False, launch_attempted=False, pid=None,
        source_manifest=str(Q / 'source_manifest.json'), source_manifest_sha256=sha(Q / 'source_manifest.json'),
        tools_manifest=str(Q / 'tools_manifest.json'), tools_manifest_sha256=sha(Q / 'tools_manifest.json'),
        worker_config_sha256=sha(Q / 'production_worker_config.json'), recipe_fingerprint=review['recipe_fingerprint'],
        candidate_review=str(Q / 'candidate_review.json'), experiment_contract=str(Q / 'experiment_contract.json'),
        human_pose_acceptance_approved=True, final_support_load_approved=True,
        review_owner='Claude Code per 2026-11-24 human instruction (batch 62-66)', prepared_at_utc=now()))
    write(LEDGER, ledger)
    print(json.dumps(dict(prepared=True, ledger_status=ledger['status'],
                          source_manifest_sha256=review['source_manifest_sha256'],
                          worker_config_sha256=review['worker_config_sha256'])))


if __name__ == '__main__':
    main()
