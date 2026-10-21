"""Pack three already-recorded original PNGs at one completed native clock."""
from pathlib import Path
import json,sys,tarfile,hashlib,argparse
from beam_workflow import save,sha

p=Path(__file__).resolve().parents[1];ap=argparse.ArgumentParser();ap.add_argument('run');ap.add_argument('label');ap.add_argument('--step',type=int);args=ap.parse_args();run,label=args.run,args.label;d=p/'runs'/run
def tail(name,size=1000000):
    f=d/name
    with f.open('rb') as s:
        s.seek(max(0,f.stat().st_size-size));lines=s.read().splitlines()
    rows=[]
    for line in lines:
        try:rows.append(json.loads(line))
        except (ValueError,UnicodeError):pass
    return rows
from index_run import select
selected_clocks=select(d,'video_frame_clock.jsonl',args.step,args.step) if args.step is not None else tail('video_frame_clock.jsonl')
groups={}
for row in selected_clocks:
    stamp=row['physics_stamp'];key=(row['task_step_label'],stamp['epoch'],stamp['index'])
    groups.setdefault(key,[]).append(row)
required={'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'}
entities={r['step']:r for r in (select(d,'entity_geometry_trace.jsonl',args.step,args.step) if args.step is not None else tail('entity_geometry_trace.jsonl',3000000))}
native={r['cycle_step']:r for r in (select(d,'physics_boundary_trace.jsonl',args.step,args.step,'before_action') if args.step is not None else tail('physics_boundary_trace.jsonl',5000000)) if r.get('boundary')=='before_action'}
choices=[key for key,rows in groups.items() if len(rows)==3 and {r['camera'] for r in rows}==required and key[0] in entities and key[0] in native and native[key[0]]['physics_step_index']==key[2]]
if not choices:raise ValueError('No three complete original views bound to entity and native before_action')
key=max(choices);rows=groups[key];step=key[0];n=native[step]
if not all(r['physics_stamp']['valid'] is True and abs(r['physics_stamp']['time']-n['physics_time'])<1e-9 for r in rows):raise ValueError('Native camera clock mismatch')
out=p/(run+'_'+label+'_snapshot');out.mkdir(exist_ok=False)
manifest=dict(run=run,label=label,step=step,phase=entities[step]['phase'],epoch=key[1],physics_index=key[2],images=[],scope='Original recorded PNGs; no rendering/physics/state change. Distinct render reference time retained.')
for r in rows:
    f=d/r['image_path'];data=f.read_bytes();name=r['camera'].replace('.','_')+'.png'
    (out/name).write_bytes(data)
    manifest['images'].append(dict(local_name=name,server_path=str(f),bytes=len(data),sha256=hashlib.sha256(data).hexdigest(),clock=r))
save(out/'manifest.json',manifest);save(out/'native.json',n);save(out/'entity.json',entities[step])
archive=out.with_suffix('.tar.gz')
with tarfile.open(archive,'w:gz') as t:
    for f in out.iterdir():t.add(f,arcname=f.name)
print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),step=step,phase=manifest['phase'])))
