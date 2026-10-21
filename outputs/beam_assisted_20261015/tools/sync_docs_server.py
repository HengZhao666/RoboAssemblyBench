from pathlib import Path
import tarfile,sys,io,json,hashlib
b=sys.stdin.buffer.read();t=tarfile.open(fileobj=io.BytesIO(b),mode='r:gz')
R=Path('/home/user/beam_integration_20261010_upstream6f57dae');P=R/'beam_assisted_20261015'
roots=[R,R/'candidate_repository',Path('/home/user/RoboAssemblyBench'),P/'assisted_repository'];rows=[]
for name in ['AGENTS.md','BEAM_REPAIR_HANDOFF.md','BEAM_REVIEW_CHECKLIST.md','BEAM_FINDINGS_INDEX.md','BEAM_ASSISTED_HANDOFF.md','BEAM_WORKFLOW_LOG.md']:
 data=t.extractfile(name).read();sha=hashlib.sha256(data).hexdigest()
 for root in roots:
  p=root/name;p.write_bytes(data);actual=hashlib.sha256(p.read_bytes()).hexdigest();assert actual==sha
  rows.append({'path':str(p),'sha256':actual})
p=P/'latest_documentation_sync.json';p.write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps({'verified_files':len(rows),'certificate':str(p)}))
