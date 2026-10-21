"""Make review clips from unchanged originals using recorded frame identities."""
from pathlib import Path
import json
import hashlib
import subprocess
import sys
import tarfile
from review_cache import phase_entries, fingerprint, reusable, file_bindings, require_finished, digest

P = Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
D = P / 'runs' / sys.argv[1]
require_finished(D)
plan_only = '--plan' in sys.argv[2:]
focus = set(sys.argv[sys.argv.index('--focus')+1].split(',')) if '--focus' in sys.argv[2:] else None
OUT = P / (D.name.split('_')[0] + '_key_nodes')
if not plan_only: OUT.mkdir(exist_ok=True)
FF = '/Embodied_AI/miniconda3/envs/fabrica/bin/ffmpeg'
FP = '/Embodied_AI/miniconda3/envs/fabrica/bin/ffprobe'
cameras = ['front', 'left_wrist', 'right_wrist']
clock = [json.loads(x) for x in (D / 'video_frame_clock.jsonl').read_text().splitlines()]
by_camera = {c: {r['frame_index']: r for r in clock if r['camera'] == 'observation.images.' + c} for c in cameras}
frames = sorted(by_camera['front'])
assert frames == list(range(len(frames)))
for i in frames:
    stamps = [by_camera[c][i]['physics_stamp'] for c in cameras]
    assert all(s['valid'] for s in stamps)
    assert len({(s['epoch'], s['index'], s['time']) for s in stamps}) == 1
front = by_camera['front']
def frame_at(step):
    return next((i for i in frames if front[i]['task_step_label'] >= step), len(frames))
result = json.loads((D / 'collect_results.json').read_text())[0]
events = phase_entries(result['phase_transition_history'])
spec=[]
def add(name,title,start_phase,end_phase,scope):
    if start_phase not in events:return
    start=max(0,events[start_phase]-64)
    end=events[end_phase]+96 if end_phase in events else None
    spec.append((name,title,start,end,scope))
add('01_grip','底梁夹持贴合','base_6_close_and_attach','base_6_lift',
    'Recorded grasp window. Measured-pose assistance and raw force qualification must be read separately from the original native trace.')
add('02_lift','底梁提起与清障','base_6_lift','base_6_assembly_clearance',
    'Recorded lift and pickup window; stage passage is determined by this run transition history.')
add('03_transport','底梁转向与搬运','base_6_assembly_clearance','base_6_place',
    'Recorded transport window. Three uncropped original views use the same recorded native physics stamp.')
add('04_install_release','底梁放置、开夹与退避','base_6_place','assemble_00_part_3_move_above',
    'Actual installation/release window through next component if reached; otherwise includes the first failure and any original tail.')
if 'base_6_visual_support' in events and 'assemble_00_part_3_lift' in events:
    add('05_right_visual_support','右臂近夹扶持与首柱提起','base_6_visual_support','assemble_00_part_3_lift',
        'Actual nonattached right visual-support acquisition through first-column lift. Three unchanged original views share recorded native stamps; gap qualification is in C_support_summary.json. No force support is claimed.')
for index,pid in enumerate(('3','2','1','0')):
    prefix=f'assemble_{index:02d}_part_{pid}_'
    next_phase=f'assemble_{index+1:02d}_part_'+('2','1','0','')[index]+'_move_above' if index<3 else 'beam_assisted_final_verification'
    add('05_part_'+pid,'构件'+pid+'夹持与装配',prefix+'close_and_attach',next_phase,
        'Original component operation window. Only reached transitions are evidence; a truncated clip does not certify release or installation.')
add('08_return_home','双臂安全回到初始位置','beam_assisted_return_home_franka_left','beam_assisted_final_verification',
    'Actual sequential empty-arm return from the installed frame through left home, right shoulder clearance and right home. Native final commands, actual joints and full geometry guards remain authoritative.')
add('09_final','最终五件验收','beam_assisted_final_verification',None,
    'Final native five-part window if reached; actual result/qualification report remains authoritative.')
if not spec:
    spec=[('00_initial_failure','初始运动与失败',0,None,
           'No grasp phase reached; original images and first failure only.')]
if focus is not None:
    assert focus and '' not in focus
    spec=[row for row in spec if row[0] in focus]
    if result.get('failed'):
        failure=(result.get('post_failure_recording') or {}).get('failure_task_step',result['steps'])
        spec.append(('00_first_failure_tail','首次失败与原物理尾段',max(0,failure-64),None,
                     'First failure and the original recorded physical tail. Tail completeness remains in the original post_failure_recording report.'))
    if not spec:
        spec=[('00_last_observation','最后实录观察',max(0,result['steps']-64),None,
               'Requested focused phases were not reached; last actual observation only.')]
manifest = {'run': D.name, 'source_run': str(D), 'layout': 'front640x360 | left_wrist480x360 | right_wrist480x360; full uncropped views',
            'fps': 30, 'all_three_views_same_recorded_physics_stamp': True,
            'clock_scope': 'Frame identities and native stamps matched; render ReferenceTime absolute relation remains unverified as in original clock log.',
            'formal_success': False, 'assisted_success': bool(result.get('assisted_success',False)),
            'actual_result': {k:result.get(k) for k in ('success','failed','steps','phase','phase_status','failure_reason')},
            'clips': []}
if focus is not None:manifest['focus_names']=sorted(focus)
source_videos = file_bindings([D / 'episode_0000_live_videos' / ('observation_images_' + c + '.mp4') for c in cameras])
source_clocks = digest(D / 'video_frame_clock.jsonl')
source_identity = digest(D / 'source_after.json')
prior_path = OUT / 'manifest.json'
prior = json.loads(prior_path.read_text()) if prior_path.exists() else {}
prior_clips = {r['name']: r for r in prior.get('clips', [])}
changed = False
for name, title, start, end, scope in spec:
    a = frame_at(start)
    b = frame_at(end) if end is not None else len(frames)
    if b<=a:continue
    out = OUT / (name + '.mp4')
    binding = fingerprint(dict(videos=source_videos, clocks=source_clocks, source=source_identity, start=a, end=b, layout=manifest['layout'],
                               encoding='libx264/veryfast/crf19/yuv420p/faststart/v1'))
    old = prior_clips.get(name)
    cached = bool(old and old.get('path') == str(out) and old.get('full_decode_exit') == 0
                  and reusable(old.get('cache'), binding))
    if plan_only:
        print(json.dumps(dict(name=name, first_frame=a, last_frame_exclusive=b, frames=b-a, reusable=cached,
                              legacy_output_preserved=out.exists() and not cached)))
        continue
    if cached:
        manifest['clips'].append(old)
        continue
    if out.exists():
        backup = OUT / 'before_rebuild'; backup.mkdir(exist_ok=True)
        saved = backup / (name + '.' + hashlib.sha256(out.read_bytes()).hexdigest()[:16] + '.mp4')
        if not saved.exists(): out.replace(saved)
        else: out.unlink()
    changed = True
    cmd = [FF, '-v', 'error', '-nostdin']
    for c in cameras:
        cmd += ['-i', str(D / 'episode_0000_live_videos' / ('observation_images_' + c + '.mp4'))]
    filters = []
    for i, w in enumerate([640, 480, 480]):
        filters.append('[%d:v]trim=start_frame=%d:end_frame=%d,setpts=PTS-STARTPTS,scale=%d:360:force_original_aspect_ratio=decrease,pad=%d:360:(ow-iw)/2:(oh-ih)/2,setsar=1[v%d]' % (i, a, b, w, w, i))
    filters.append('[v0][v1][v2]hstack=inputs=3[v]')
    cmd += ['-filter_complex', ';'.join(filters), '-map', '[v]', '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '19', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(out)]
    subprocess.run(cmd, check=True)
    probe = json.loads(subprocess.check_output([FP, '-v', 'error', '-count_frames', '-select_streams', 'v:0', '-show_entries', 'stream=width,height,nb_read_frames,r_frame_rate,duration', '-of', 'json', str(out)], text=True))
    assert int(probe['streams'][0]['nb_read_frames']) == b-a
    decode = subprocess.run([FF, '-v', 'error', '-i', str(out), '-f', 'null', '-'], capture_output=True, text=True)
    assert decode.returncode == 0 and not decode.stderr.strip()
    row = {'name': name, 'title': title, 'path': str(out), 'size': out.stat().st_size, 'sha256': hashlib.sha256(out.read_bytes()).hexdigest(),
           'source_frame_start_inclusive': a, 'source_frame_end_exclusive': b, 'source_video_time_start_s': a/30., 'source_video_time_end_s': b/30.,
           'first_frame': front[a], 'last_frame': front[b-1], 'scope': scope, 'probe': probe, 'full_decode_exit': decode.returncode}
    row['cache'] = dict(binding=binding, returncode=0, outputs=file_bindings([out]))
    manifest['clips'].append(row)
    print(json.dumps({k: row[k] for k in ['title', 'path', 'size', 'sha256', 'source_frame_start_inclusive', 'source_frame_end_exclusive']}, ensure_ascii=False), flush=True)
if plan_only: sys.exit(0)
if prior_path.exists() and changed:
    before = OUT / ('manifest.before.' + hashlib.sha256(prior_path.read_bytes()).hexdigest()[:16] + '.json')
    if not before.exists(): before.write_bytes(prior_path.read_bytes())
(OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
archive = P / (OUT.name + '.tar.gz')
if archive.exists() and not changed:
    print(json.dumps(dict(archive=str(archive), reused=True, clips=len(manifest['clips']))));sys.exit(0)
if archive.exists():
    backup = archive.with_name(archive.stem + '.before.' + hashlib.sha256(archive.read_bytes()).hexdigest()[:16] + '.tar.gz')
    if not backup.exists(): archive.replace(backup)
    else: archive.unlink()
with tarfile.open(archive, 'w:gz') as t:
    for f in [OUT/'manifest.json', *[Path(row['path']) for row in manifest['clips']]]:
        t.add(f, arcname=OUT.name + '/' + f.name)
print(json.dumps({'archive': str(archive), 'size': archive.stat().st_size, 'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}), flush=True)
