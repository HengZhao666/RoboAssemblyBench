from pathlib import Path
import hashlib,json,shutil,tarfile
R=Path('/home/user/beam_integration_20261010_upstream6f57dae')
C=R/'candidate_repository'; P=R/'beam_assisted_20261015'; P.mkdir(exist_ok=True)
M=R/'grasp_repairs_20261012_20rounds/round24_native_release/source_manifest.json'
m=json.loads(M.read_text()); differences=[]
for f in m['files']:
 p=C/f['path']
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=f['sha256']: differences.append(f['path'])
assert not differences,differences
archive=P/'preserved_v201_source876.tar.gz'
if not archive.exists():
 with tarfile.open(archive,'w:gz') as t:
  for f in m['files']:t.add(C/f['path'],arcname=f['path'],recursive=False)
shutil.copy2(M,P/'preserved_v201_manifest.json')
A=P/'assisted_repository'; A.mkdir(exist_ok=True)
if not (P/'bootstrap_verification.json').exists():
 for f in m['files']:
  p=A/f['path'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(C/f['path'],p)
 def fill(src,dst):
  for p in src.iterdir():
   if p.name in {'outputs','.git','__pycache__','.pytest_cache','none'}:continue
   q=dst/p.name
   if q.is_symlink() or (q.exists() and not q.is_dir()):continue
   if not q.exists():q.symlink_to(p,target_is_directory=p.is_dir())
   elif p.is_dir():fill(p,q)
 fill(C,A)
 report={'preserved_source':str(C),'assisted_source':str(A),'verified_files':len(m['files']),'differences':differences,'archive':str(archive),'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'manifest_sha256':hashlib.sha256(M.read_bytes()).hexdigest(),'source_copy_isolated':True,'assets_shared_read_only_by_convention':True}
 (P/'bootstrap_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print((P/'bootstrap_verification.json').read_text())
