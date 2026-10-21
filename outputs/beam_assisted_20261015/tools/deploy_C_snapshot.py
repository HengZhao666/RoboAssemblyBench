from pathlib import Path
import sys,tarfile,io,hashlib,json
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');label=sys.argv[1];rows=[]
assert label.replace('_','').isalnum()
with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()),mode='r:gz') as t:
 for m in t.getmembers():
  n=Path(m.name);assert m.isfile() and not n.is_absolute() and '..' not in n.parts
  assert n.parts[0] in ['assisted_repository','tools','coordination_history','versions'] or len(n.parts)==1
  p=P/n;p.parent.mkdir(parents=True,exist_ok=True)
  assert p.parent.resolve().is_relative_to(P.resolve()),str(p)
  if p.is_symlink():p.unlink()
  data=t.extractfile(m).read();h=hashlib.sha256(data).hexdigest();
  if n.parts[0]=='versions' and p.exists():assert hashlib.sha256(p.read_bytes()).hexdigest()==h,'cannot replace preserved version'
  p.write_bytes(data);assert hashlib.sha256(p.read_bytes()).hexdigest()==h;rows.append({'path':str(n),'bytes':len(data),'sha256':h})
r={'label':label,'files':rows,'count':len(rows),'runtime_started':False};(P/(label+'_deployment.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'label':label,'verified_files':len(rows)}))
