from pathlib import Path
import sys,json,tarfile,hashlib
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');name=sys.argv[1];D=P/'runs'/name
report=dict(run=name,task_steps=0,policy_actions=0,missing_views_reason='initialization aborted before first policy action; no original frames or full videos exist',source_after=json.loads((D/'source_after.json').read_text())['differences'],worker_exit=(D/'exit_code').read_text().strip(),guard_exit=(D/'visible_mesh_guard_exit_code').read_text().strip(),not_an_assisted_success=True)
(D/'initialization_failure_review.json').write_text(json.dumps(report,indent=2)+'\n');a=P/(name+'_initialization_evidence.tar.gz')
with tarfile.open(a,'w:gz',compresslevel=3) as t:
 for p in D.rglob('*'):
  if p.is_file():t.add(p,arcname=str(p.relative_to(P)),recursive=False)
 p=P/(name.split('_')[0]+'_launch.log');t.add(p,arcname=p.name,recursive=False)
r=dict(archive=str(a),bytes=a.stat().st_size,sha256=hashlib.sha256(a.read_bytes()).hexdigest(),review=report);(P/(name+'_initialization_archive_manifest.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
