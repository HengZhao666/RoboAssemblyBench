"""Preserve same-process initial image evidence for review before action."""
from pathlib import Path
import json,sys,tarfile,hashlib
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
name=sys.argv[1];D=P/'runs'/name;ready=D/'startup_images_ready.json'
if not ready.exists():
 print(json.dumps({'ready':False,'finished':(D/'finished_at').exists()}));raise SystemExit(2)
r=json.loads(ready.read_text());names=[p.name for p in D.glob('startup_*.json')]
names+=['initial_native_joint_preflight.json','runtime_configuration_binding.json',
 'expected_runtime_configuration.json','native_usd_startup_sync.json',
 'camera_runtime_visibility.json','camera_native_initialization.json','runtime_initialization_preflight.json']
for row in r['images']:
 p=Path(row['path']);assert p.parent==D and hashlib.sha256(p.read_bytes()).hexdigest()==row['sha256'];names.append(p.name)
a=P/(name+'_startup_review.tar.gz')
with tarfile.open(a,'w:gz') as t:
 for n in sorted(set(names)):
  if (D/n).exists():t.add(D/n,arcname=n,recursive=False)
print(json.dumps({'ready':True,'archive':str(a),'bytes':a.stat().st_size,
 'sha256':hashlib.sha256(a.read_bytes()).hexdigest(),'images':len(r['images']),
 'physics_stamp':r['physics_stamp']}))
