"""Read-only: the round-62 'fixed' criteria for a run, with rounds 55/59 as references.

Usage: python fixed_criteria.py fabrica_beam_v239_unload [more output names] > fixed_criteria.json
For each run: column-2 height above its target and base load after the handover
and during part 0; part-0 side load, tilt and finger forces at the hold check;
part-0 motion after both arms leave; final pose-window drift and raw speeds.
"""
import collections, gzip, json, math, os, statistics, sys
C = '/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
Z0 = 1.0025
TXY = (0.7844409166438082, -0.12056654334103567)


def op(p):
    return open(p) if os.path.exists(p) else gzip.open(p + '.gz', 'rt')


def tilt_mrad(q):
    w, x, y, z = q
    return 1000 * math.acos(max(-1, min(1, 1 - 2 * (x * x + y * y))))


def run_metrics(run):
    out = {}
    integ = collections.defaultdict(lambda: collections.defaultdict(list))
    with op(C + run + '/assembly_integrity_trace.jsonl') as f:
        for line in f:
            s = int(line[9:line.index(',')])
            if s < 44000:
                continue
            r = json.loads(line)
            e = {i['part']: i for i in r['interfaces']}
            d = integ[r['phase']]
            for p in ('2', '0'):
                if p in e:
                    c = e[p].get('interface_contact') or {}
                    fw = c.get('force_world') or [0, 0, 0]
                    d['p%s_F' % p].append(c.get('force') or 0)
                    d['p%s_lat' % p].append(math.hypot(fw[0], fw[1]))
    def m(ph, k):
        v = integ.get(ph, {}).get(k)
        return None if not v else dict(mean=round(statistics.mean(v), 3), max=round(max(v), 3), last=round(v[-1], 3))
    out['base_load_col2'] = {ph: m(ph, 'p2_F') for ph in ('handover_left_2_unload_lift', 'handover_left_2_release',
                             'assemble_03_part_0_insert_09', 'assemble_03_part_0_physical_hold_check')}
    out['part0_on_col2'] = {ph: dict(force=m(ph, 'p0_F'), lateral=m(ph, 'p0_lat')) for ph in (
        'assemble_03_part_0_physical_hold_check', 'assemble_03_part_0_release', 'assemble_03_part_0_retreat',
        'hold_2_final_release', 'beam_final_unloaded_verification')}
    phys = collections.defaultdict(list)
    with op(C + run + '/physics_boundary_trace.jsonl') as f:
        for line in f:
            if '"before_action"' not in line[:300]:
                continue
            r = json.loads(line)
            b = {x['name']: x for x in r['bodies'] if x['name'].startswith('fabrica_beam')}
            if 'fabrica_beam_2' not in b:
                continue
            phys[r['phase']].append((r['task_step'], b))
    def snap(ph, which):
        rows = phys.get(ph)
        if not rows:
            return None
        s, b = rows[0] if which == 'first' else rows[-1]
        res = dict(step=s)
        for k in ('fabrica_beam_2', 'fabrica_beam_0'):
            if k in b:
                p = b[k]['positions'][0]
                res[k[-1]] = dict(dz_mm=round((p[2] - Z0) * 1000, 3), dxy_mm=[round((p[0] - TXY[0]) * 1000, 3), round((p[1] - TXY[1]) * 1000, 3)],
                                  tilt_mrad=round(tilt_mrad(b[k]['orientations'][0]), 2))
        return res
    out['stack_snapshots'] = {f'{ph}:{w}': snap(ph, w) for ph in (
        'handover_left_2_unload_lift', 'hold_3_to_2_switch_release', 'handover_left_2_park', 'assemble_03_part_0_insert_09',
        'assemble_03_part_0_physical_hold_check', 'hold_2_final_release', 'beam_final_unloaded_verification') for w in ('first', 'last')}
    motion = {}
    for ph in ('assemble_03_part_0_park', 'hold_2_final_park', 'beam_terminal_home_franka_right', 'beam_final_unloaded_verification'):
        rows = phys.get(ph) or []
        if len(rows) < 2:
            continue
        d = [1000 * math.dist(b['fabrica_beam_0']['positions'][0], a['fabrica_beam_0']['positions'][0])
             for (_, a), (_, b) in zip(rows[:-1], rows[1:]) if 'fabrica_beam_0' in a and 'fabrica_beam_0' in b]
        w = [math.hypot(*b['fabrica_beam_0']['angular_velocities'][0]) for _, b in rows if 'fabrica_beam_0' in b]
        motion[ph] = dict(step8_mm_mean=round(statistics.mean(d), 4), ang_speed_mean=round(statistics.mean(w), 3),
                          ang_speed_max=round(max(w), 3))
    out['part0_motion_after_arms_leave'] = motion
    try:
        with op(C + run + '/episode_0000.json') as f:
            ep = json.load(f)
        mt = ep['metrics']
        fa = mt.get('beam_final_acceptance') or {}
        out['result'] = dict(success=mt.get('success'), terminal=mt.get('terminal_reason'), steps=mt.get('steps'))
        out['final_window'] = {k: dict(drift_mm=round(v.get('position_drift', 0) * 1000, 3), drift_rad=round(v.get('orientation_drift', 0), 4),
                                       raw_lin=round(v['linear_speed'], 4), raw_ang=round(v['angular_speed'], 3))
                               for k, v in ((fa.get('motion') or {}).get('objects') or {}).items()}
        out['final_window_steps'] = dict(count=fa.get('count'), phase_steps=mt['phase_transition_history'][-1].get('phase_step_counter'))
    except Exception as exc:  # running or missing
        out['result'] = repr(exc)
    return out


runs = sys.argv[1:] or ['fabrica_beam_v239_unload']
print(json.dumps({r: run_metrics(r) for r in runs}, indent=1))
