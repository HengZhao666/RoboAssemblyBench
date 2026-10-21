"""Seal the final C candidate without replacing a historical source version."""
from pathlib import Path
import hashlib,json,tarfile,sys
P=Path(__file__).resolve().parents[1];A=P/'assisted_repository';T=P/'tools'
version,run=sys.argv[1:];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
previous=json.loads((T/'assisted_manifest.json').read_text())
names=sorted(set(previous)|{'toolkits/factory_dual_franka_assembly/beam_assisted_servo.py','tests/toolkits/test_beam_assisted_object_servo.py'})
m={n:sha(A/n) for n in names};(T/'assisted_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
critical=sorted(set(json.loads((T/'assisted_tools_manifest.json').read_text()))|{
 'controller_descent_replay.py','robust_insert_corridor.py','held_fk_baseline_review.py',
 'nominal_bimanual.py','visual_support_check.py','run_checks.sh'})
tm={n:sha(T/n) for n in critical};(T/'assisted_tools_manifest.json').write_text(json.dumps(tm,indent=2)+'\n')
archives=[]
for filename,root,files in [(f'assisted_{version}_before_{run}.tar.gz',A,names),
    (f'assisted_{version}_tools_before_{run}.tar.gz',T,critical+['assisted_manifest.json','assisted_tools_manifest.json'])]:
 p=P/'versions'/filename
 if p.exists():raise RuntimeError('cannot replace sealed candidate '+str(p))
 with tarfile.open(p,'w:gz') as t:
  for n in files:t.add(root/n,arcname=n,recursive=False)
 archives.append({'path':str(p),'sha256':sha(p),'bytes':p.stat().st_size})
report={'source_files':len(m),'critical_tools':len(tm),'version':version,'run':run,'archives':archives}
(P/(run+'_sealed_version.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

