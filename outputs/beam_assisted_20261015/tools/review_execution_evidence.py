"""One independent, read-only SDK/native/three-view check of a finished run."""
import argparse
import collections
import json
from pathlib import Path
from beam_workflow import save, sha
from index_run import select
from review_cache import require_finished, immutable_inputs, dynamic_mesh_bodies


def completed_probe_boundary(gate, result, boundary, fixture):
    if gate.get('run_class') not in ('isolated_fourth_part_probe','isolated_selected_part_probe') or result.get('failed'):
        return False
    stop=result.get('assisted_boundary_stop')
    if not stop or stop!=boundary or stop.get('success') is not False:
        return False
    stamp=stop.get('physics_stamp') or {}
    return bool(stamp.get('valid') is True and type(stamp.get('index')) is int
        and type(stop.get('step')) is int and stop['step']<=result['steps']
        and stop.get('completed_phase')==fixture.get('stop_after_phase',fixture.get('stop_phase')))


def completed_assisted_sequence(result):
    # Only the original final five-part native certificate can excuse a
    # failure-only tail. A success flag or diagnostic boundary is insufficient.
    if not (result.get('execution_route')=='assisted' and result.get('success') is True
            and result.get('assisted_success') is True and result.get('failed') is False
            and result.get('phase_status')=='success' and result.get('terminal_reason')=='success-criteria-met'
            and not result.get('assisted_boundary_stop')):
        return False
    qualification=result.get('assisted_final_qualification') or {}
    certificate=qualification.get('certificate') or {}
    native=qualification.get('current_native_sample') or {}
    required=certificate.get('required_stable_physics_steps',0)
    expected={'fabrica_beam_'+n for n in ('6','3','2','1','0')}
    return bool(qualification.get('current_native_recheck',{}).get('ready') is True
        and result.get('success_detector',{}).get('criteria_passed') is True
        and type(required) is int and required>=240
        and certificate.get('stable_physics_steps',0)>=required
        and set(certificate.get('native_poses',{}))==expected
        and native.get('ready') is True and set(native.get('objects',{}))==expected
        and (certificate.get('physics_stamp') or {}).get('valid') is True
        and certificate.get('physics_stamp')==native.get('physics_stamp'))


def review(p, run, cycle):
    d = p / 'runs' / run
    require_finished(d)
    inputs = immutable_inputs(d)
    trace = select(d, 'assisted_trace.jsonl', 0, 2147483647)
    publications = {(r['step'], r['robot']): r for r in trace if r.get('event') == 'published_command'}
    event_clocks = collections.defaultdict(set)
    for r in trace:
        s = r.get('physics_stamp') or {}
        if s.get('valid'):
            event_clocks[r['step']].add((s['epoch'], s['index'], s['time']))
    before = {}; matched = set(); maximum = 0.; errors = []; native = valid = 0
    # Stream the large native file once instead of copying its rows into reports.
    with (d / 'physics_boundary_trace.jsonl').open() as stream:
        for line in stream:
            n = json.loads(line); native += 1; valid += bool(n.get('valid'))
            step = n['cycle_step']
            if n['boundary'] == 'before_action':
                before[step] = (n['physics_step_index'], n['physics_time'])
            if n['boundary'] != 'after_action': continue
            for who, raw in n['robots'].items():
                key = (step, who)
                if key not in publications: continue
                actions = [a for a in raw['sdk_submission']['actions']
                           if a.get('joint_indices') == list(range(6))]
                if len(actions) != 1:
                    errors.append(dict(step=step, robot=who, error='missing_or_duplicate_SDK_arm_action')); continue
                delta = max(abs(a-b) for a,b in zip(actions[0]['joint_positions'], publications[key]['published_q']))
                if len(actions[0]['joint_positions']) != 6: errors.append(dict(step=step, error='wrong_DOF_count'))
                maximum = max(maximum, delta); matched.add(key)
    missing = sorted(set(publications) - matched)
    clocks = collections.defaultdict(dict)
    with (d / 'video_frame_clock.jsonl').open() as stream:
        for line in stream:
            r = json.loads(line); s = r['physics_stamp']; key = (r['task_step_label'], s['epoch'], s['index'], s['time'])
            if not s.get('valid') or r['camera'] in clocks[key]: errors.append(dict(step=key[0], error='invalid_or_duplicate_camera_clock'))
            clocks[key][r['camera']] = r
    geometry = json.loads((d / 'scene_geometry.json').read_text())
    required_bodies = dynamic_mesh_bodies(geometry)
    all_mesh_bodies = {r['body'] for r in geometry['visible_meshes']}
    cameras = {'observation.images.front', 'observation.images.left_wrist', 'observation.images.right_wrist'}
    aligned = entities = 0; body_counts = set(); static_fallback = set()
    by_step = collections.defaultdict(list)
    for key, rows in clocks.items(): by_step[key[0]].append((key, rows))
    with (d / 'entity_geometry_trace.jsonl').open() as stream:
        for line in stream:
            r = json.loads(line); entities += 1; step = r['step']; body_counts.add(len(r['body_poses']))
            static_fallback.update(all_mesh_bodies-set(r['body_poses']))
            groups = by_step.get(step, []); n = before.get(step)
            correct = len(groups) == 1 and n is not None and required_bodies <= set(r['body_poses'])
            if correct:
                key, views = groups[0]
                correct = (set(views) == cameras and key[2] == n[0] and abs(key[3]-n[1]) < 1e-9
                           and key[1:] in event_clocks.get(step, set()))
            if correct: aligned += 1
            else: errors.append(dict(step=step, error='entity_native_camera_or_epoch_binding'))
    result = json.loads((d / 'collect_results.json').read_text())[0]
    terminal = result['phase_transition_history'][-1]
    phase = terminal.get('from_phase', terminal.get('phase'))
    obs = [r for r in trace if r.get('phase') == phase and r.get('event') == 'visual_pinch_observation']
    pinch = None
    if obs:
        values = [r['command_driver_q'] for r in obs]
        pinch = dict(samples=len(obs),command_range_rad=[min(values), max(values)],
                     max_stable=max(r['stable'] for r in obs),ready_samples=sum(r['ready'] for r in obs),
                     gap_range_m=[min(g['gap'] for r in obs for g in r['gaps']),max(g['gap'] for r in obs for g in r['gaps'])],
                     last=obs[-1],frozen_command_with_unqualified_gap=max(values)-min(values)<1e-12 and not any(r['ready'] for r in obs))
    tail_path=d/'post_failure_recording.json'
    if tail_path.exists():
        tail=json.loads(tail_path.read_text())
    else:
        gate=json.loads((d/'launch_gate.json').read_text())['gate']
        boundary=json.loads((d/'bounded_stop.json').read_text()) if (d/'bounded_stop.json').exists() else None
        fixture=json.loads((d/'probe_initialization.json').read_text()).get('fixture',{}) if (d/'probe_initialization.json').exists() else {}
        sequence_completed=completed_assisted_sequence(result)
        if not sequence_completed and not completed_probe_boundary(gate,result,boundary,fixture):
            raise ValueError('Expected original failure-tail report missing')
        tail=dict(required=False,reason=('original assisted final certificate completed without a failure' if sequence_completed else 'isolated diagnostic boundary completed without a failure'),
                  original_video_retained=True,full_assembly_claimed=False)
    report = dict(run=run,cycle=cycle,analysis_complete=True,
                  evidence_checks_passed=not errors and not missing and maximum<1e-12 and native==valid and entities==aligned,
                  source_after_sha256=sha(d/'source_after.json'),tool_sha256=sha(__file__),inputs=inputs,
                  SDK=dict(stored_submissions=len(matched),maximum_delta_rad=maximum,unmatched_publications=missing),
                  native=dict(samples=native,valid=valid),three_views=dict(entity_samples=entities,aligned=aligned,body_counts=sorted(body_counts),
                      required_dynamic_body_count=len(required_bodies),static_mesh_bodies_without_per_tick_reader=sorted(static_fallback)),
                  terminal_transition=terminal,terminal_pinch=pinch,errors=errors,
                  tail=tail,full_assembly_claimed=False,
                  limits=['Recorded samples only; physical tail has separate native trace, not the full CAD entity stream.',
                          'Raw native boundaries omit epoch; matching assisted-event epoch is explicitly joined.',
                          'Static visual-only mesh poses use exported body_rest in the complete CAD monitor; not per-tick native measurements.',
                          'A frozen command is an observation, not proof of its code-level root cause.'])
    save(p/(cycle+'_execution_evidence.json'),report)
    return report


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');ap.add_argument('cycle');a=ap.parse_args()
    r=review(Path(__file__).resolve().parents[1],a.run,a.cycle)
    print(json.dumps(dict(analysis_complete=r['analysis_complete'],evidence_checks_passed=r['evidence_checks_passed'],
                         SDK=r['SDK'],three_views=r['three_views'])))
