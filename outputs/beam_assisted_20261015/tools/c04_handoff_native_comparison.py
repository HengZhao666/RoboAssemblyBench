"""Read exactly one A15/A16 handoff window; never write simulator state."""
from pathlib import Path
import json, math, re, time
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
STEPS=(6528,6536,6544)
def selected(path,key):
    pat=re.compile(rb'"'+key.encode()+rb'"\s*:\s*(6528|6536|6544)(?=[,}])')
    result=[]
    for line in path.open('rb'):
        if pat.search(line):
            try:result.append(json.loads(line))
            except json.JSONDecodeError:pass
    return result
def diff(a,b):
    if len(a)!=len(b):raise ValueError('shape mismatch')
    return max((abs(float(x)-float(y)) for x,y in zip(a,b)),default=0.)
def arm(raw):return raw['joint_positions'][0][:6]
def driver(raw):return raw['joint_positions'][0][6:]
def native_target(raw):return raw['native_command_targets']['position_targets'][0]
def sdk(raw):
    return next(v['joint_positions'] for v in raw['sdk_submission']['actions'] if v.get('joint_indices')==list(range(6)))
def beam(n):return next(v for v in n['bodies'] if v.get('name')=='fabrica_beam_6')
def load(run):
    d=P/'runs'/run
    native={(r['cycle_step'],r['boundary']):r for r in selected(d/'physics_boundary_trace.jsonl','cycle_step')}
    pub={(r['step'],r['robot']):r for r in selected(d/'assisted_trace.jsonl','step') if r.get('event')=='published_command'}
    supports=[]
    for line in (d/'visual_support_trace.jsonl').open():
        try:supports.append(json.loads(line))
        except json.JSONDecodeError:pass
    return native,pub,supports
a,ap,asup=load('A15_beam_full');b,bp,bsup=load('A16_beam_full')
comparisons=[]
for step in STEPS:
    an,bn=a[(step,'before_action')],b[(step,'before_action')]
    row={'step':step,'A15_physics_index':an['physics_step_index'],'A16_physics_index':bn['physics_step_index'],
         'A15_phase':an['phase'],'A16_phase':bn['phase'],'robots':{}}
    for robot in an['robots']:
        ar,br=an['robots'][robot],bn['robots'][robot]
        row['robots'][robot]={'actual_arm_max_difference_rad':diff(arm(ar),arm(br)),
            'actual_all_gripper_max_difference_rad':diff(driver(ar),driver(br)),
            'native_arm_target_max_difference_rad':diff(native_target(ar)[:6],native_target(br)[:6]),
            'native_all_gripper_target_max_difference_rad':diff(native_target(ar)[6:],native_target(br)[6:]),
            'SDK_arm_target_max_difference_rad':diff(sdk(ar),sdk(br))}
    ab,bb=beam(an),beam(bn)
    row['beam6']={'position_max_difference_m':diff(ab['positions'][0],bb['positions'][0]),
        'orientation_component_max_difference':diff(ab['orientations'][0],bb['orientations'][0]),
        'linear_velocity_max_difference_m_s':diff(ab['linear_velocities'][0],bb['linear_velocities'][0]),
        'angular_velocity_max_difference_rad_s':diff(ab['angular_velocities'][0],bb['angular_velocities'][0])}
    comparisons.append(row)
commands=[]
for run,native,published in (('A15_beam_full',a,ap),('A16_beam_full',b,bp)):
    for step in STEPS:
        n=native[(step,'before_action')];pub=published.get((step,'franka_right'));after=native.get((step,'after_action'))
        raw=n['robots']['franka_right']
        row={'run':run,'step':step,'phase':n['phase'],'before_actual_arm_q':arm(raw),
            'before_actual_all_gripper_q':driver(raw),'before_native_arm_target_q':native_target(raw)[:6],
            'published_q':pub['published_q'] if pub else None,'published_stamp':pub.get('physics_stamp') if pub else None}
        if pub and after:
            row['after_action_SDK_q']=sdk(after['robots']['franka_right'])
            row['published_vs_SDK_max_difference_rad']=diff(pub['published_q'],row['after_action_SDK_q'])
        commands.append(row)
def support_summary(rows):
    return {'total_samples_at_read':len(rows),'key_samples':[v for v in rows if v['step'] in STEPS],
            'latest_samples':rows[-3:],'failure_count_at_read':sum(not v['ready'] for v in rows)}
print(json.dumps({'schema':'beam_C04_A15_A16_native_handoff_comparison_v1','read_at_unix':time.time(),
    'native_input_comparisons':comparisons,'right_commands':commands,
    'A15_support':support_summary(asup),'A16_support':support_summary(bsup),
    'scope':'Actual recorded endpoint comparison only; same cycle step and native physics index shown. No inferred intermediate native samples, force-support claim, or complete assembly success.'}))
