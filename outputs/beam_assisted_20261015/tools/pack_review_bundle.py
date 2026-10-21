"""Downloadable original-video bundle; full native logs remain in run archive."""
from pathlib import Path
import hashlib,json,sys,tarfile
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');name=sys.argv[1];D=P/'runs'/name
assert (D/'finished_at').exists() and (D/'source_after.json').exists()
source=json.loads((D/'source_after.json').read_text())
assert not source['differences'] and not source.get('tool_differences',[])
videos=json.loads((D/'video_manifest.json').read_text());assert len(videos)==3
assert all(v['decode_exit']==0 and not v['decode_errors'].strip() for v in videos)
files=[p for p in D.glob('*.json') if p.name!='scene_geometry.json']
files += [D/n for n in ['video_frame_clock.jsonl','camera_native_trace.jsonl',
 'camera_native_registration.jsonl','camera_native_update_errors.jsonl','assisted_trace.jsonl'] if (D/n).exists()]
files += [D/v['relative_path'] for v in videos]
a=P/(name+'_review_bundle.tar.gz');assert not a.exists(),'cannot replace an existing evidence bundle'
with tarfile.open(a,'w:gz',compresslevel=3) as t:
 for p in sorted(set(files)):t.add(p,arcname=str(p.relative_to(D)),recursive=False)
report={'archive':str(a),'bytes':a.stat().st_size,'sha256':hashlib.sha256(a.read_bytes()).hexdigest(),
 'videos':3,'source_files':len(source['files']),'critical_tools':len(source.get('tool_files',[])),
 'raw_native_logs_location':str(D),'full_raw_archive':str(P/(name+'_evidence.tar.gz')),
 'scope':'three complete originals and unchanged run metadata/clock/assistance/camera traces; full raw native and geometry preserved separately'}
(P/(name+'_review_bundle_manifest.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
