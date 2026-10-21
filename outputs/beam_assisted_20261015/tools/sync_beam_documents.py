"""One transaction for explicit document mirrors, SHA receipts and skill install.

No native launcher, ledger write, source deployment, implicit file discovery or
simulation imports. Reuse a tag only with the identical saved document bundle.
"""
import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

REMOTE = r'''
import pathlib,sys,tarfile,io,json,hashlib,importlib.util,datetime
data=sys.stdin.buffer.read()
with tarfile.open(fileobj=io.BytesIO(data)) as archive:
    assert set(archive.getnames())=={'request.json','bundle.tar.gz','sync_documents.py'}
    request=json.loads(archive.extractfile('request.json').read())
    bundle=archive.extractfile('bundle.tar.gz').read()
    program=archive.extractfile('sync_documents.py').read()
root=pathlib.Path(request['registry']['server_root'])
source=pathlib.Path(request['registry']['routes']['assisted']['source_root'])
p=source.parent;ledger=json.loads((p/'repair_batch_D_20261016.json').read_text())
assert ledger['native_attempts_started']==ledger['completed_cycles'], 'Native run is still open'
stage=p/'document_sync'/request['tag'];stage.mkdir(parents=True,exist_ok=True)
def sha(f):return hashlib.sha256(f.read_bytes()).hexdigest()
for name,content in [('bundle.tar.gz',bundle),('sync_documents.py',program)]:
    f=stage/name
    if f.exists():assert f.read_bytes()==content, 'Transaction tag already has different input'
    else:f.write_bytes(content)
spec=importlib.util.spec_from_file_location('explicit_sync',stage/'sync_documents.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
manifest,_=s.unpack(stage/'bundle.tar.gz')
assert set(manifest)==set(request['documents']['files'])
identities=[]
for scope,base,mp in [('assisted',source,p/'tools/assisted_manifest.json'),
                      ('critical_tools',p/'tools',p/'tools/assisted_tools_manifest.json'),
                      ('physical_hash_only',root/'candidate_repository',p/'tools/physical_manifest.json')]:
    expected=json.loads(mp.read_text())
    assert all(sha(base/n)==h for n,h in expected.items()), 'Source/tool drift before document sync'
    identities.append(dict(scope=scope,files=len(expected),manifest_sha256=sha(mp),differences=[]))
receipt=stage/'server_receipt.json'
s.apply(stage/'bundle.tar.gz',request['documents']['server_mirrors'],receipt)
s.verify(stage/'bundle.tar.gz',request['documents']['server_mirrors'],receipt)
for base in map(pathlib.Path,request['documents']['server_mirrors']):
    for name,row in manifest.items():assert sha(base/name)==row['sha256']
for base,mp in [(source,p/'tools/assisted_manifest.json'),(p/'tools',p/'tools/assisted_tools_manifest.json'),(root/'candidate_repository',p/'tools/physical_manifest.json')]:
    assert all(sha(base/n)==h for n,h in json.loads(mp.read_text()).items()), 'Source/tool changed during docs sync'
current=json.loads((p/'repair_batch_D_20261016.json').read_text());assert current==ledger
result=dict(passed=True,documentation_version=request['documents']['documentation_version'],
    tag=request['tag'],utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    document_bundle_sha256=sha(stage/'bundle.tar.gz'),explicit_files=len(manifest),
    local_documents=request['local_receipt'],server_documents=json.loads(receipt.read_text()),
    identities=identities,native_started_by_transaction=False,
    assisted_attempts_started=ledger['native_attempts_started'],assisted_cycles_closed=ledger['completed_cycles'],
    next_native_run_authorized=ledger['next_native_run_authorized'],ledger_unchanged=True,
    scope='Document/workflow synchronization only; sealed control source and critical tools unchanged')
target=root/request['documents']['receipt']
if target.exists():
    old=json.loads(target.read_text())
    assert old['tag']==request['tag'] and old['document_bundle_sha256']==result['document_bundle_sha256'], 'Never replace an old terminal certificate'
    result=old
output=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
for base in map(pathlib.Path,request['documents']['server_mirrors']):
    f=base/request['documents']['receipt'];s.atomic_write(f,output);assert f.read_bytes()==output
sys.stdout.buffer.write(output)
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base',required=True,type=Path)
    ap.add_argument('--tag',required=True)
    ap.add_argument('--install-skill',action='store_true')
    args=ap.parse_args()
    if not args.tag or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.tag):
        raise ValueError('Noncanonical transaction tag')
    base=args.base.resolve();documents=json.loads((base/'beam_routes/docs_manifest.json').read_text())
    registry=json.loads((base/'beam_routes/registry.json').read_text())
    out=base/'outputs'/args.tag;out.mkdir(exist_ok=True)
    spec=importlib.util.spec_from_file_location('explicit_sync',base/documents['sync_program'])
    sync=importlib.util.module_from_spec(spec);spec.loader.exec_module(sync)
    bundle=out/'documents.tar.gz'
    if bundle.exists():
        manifest,_=sync.unpack(bundle)
        assert set(manifest)==set(documents['files'])
        assert all(sync.digest((base/n).read_bytes())==r['sha256'] for n,r in manifest.items()), 'Tag reused after a document edit'
    else:sync.pack(base,base/'beam_routes/docs_manifest.json',bundle)
    local_receipt=out/'local_receipt.json'
    sync.apply(bundle,documents['local_mirrors'],local_receipt)
    sync.verify(bundle,documents['local_mirrors'],local_receipt)
    if args.install_skill:
        installed=Path(documents['skill_install']);before=out/'installed_skill_before.tar.gz'
        names=[n for n in documents['files'] if n.startswith('skills/simulation-repair-loop/')]
        if not before.exists():
            with tarfile.open(before,'w:gz') as t:
                for n in names:
                    rel=Path(n).relative_to('skills/simulation-repair-loop')
                    if (installed/rel).is_file():t.add(installed/rel,arcname=str(rel),recursive=False)
        for n in names:
            rel=Path(n).relative_to('skills/simulation-repair-loop')
            sync.atomic_write(installed/rel,(base/n).read_bytes())
            assert sync.digest((installed/rel).read_bytes())==sync.digest((base/n).read_bytes())
    request=dict(tag=args.tag,documents=documents,registry=registry,local_receipt=json.loads(local_receipt.read_text()))
    payload=io.BytesIO()
    with tarfile.open(fileobj=payload,mode='w:gz') as t:
        for n,data in [('request.json',json.dumps(request).encode()),('bundle.tar.gz',bundle.read_bytes()),('sync_documents.py',(base/documents['sync_program']).read_bytes())]:
            row=tarfile.TarInfo(n);row.size=len(data);t.addfile(row,io.BytesIO(data))
    result=subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=12',registry['server_host'],
                           'python3 -c '+shlex.quote(REMOTE)],input=payload.getvalue(),capture_output=True)
    if result.returncode:
        raise RuntimeError('sync_pending: '+result.stderr.decode(errors='replace')[-2000:])
    data=result.stdout;receipt=json.loads(data);assert receipt['passed'] is True
    target=base/documents['receipt']
    if target.exists():assert target.read_bytes()==data, 'Do not replace an existing certificate'
    sync.atomic_write(target,data)
    for root in map(Path,documents['local_mirrors']):
        sync.atomic_write(root/documents['receipt'],data)
        assert (root/documents['receipt']).read_bytes()==data
    print(json.dumps(dict(passed=True,explicit_files=len(documents['files']),
                         local_copies=receipt['local_documents']['copies'],server_copies=receipt['server_documents']['copies'],
                         certificate=str(target),certificate_sha256=hashlib.sha256(data).hexdigest(),
                         next_native_run_authorized=receipt['next_native_run_authorized'])))


if __name__=='__main__':main()
