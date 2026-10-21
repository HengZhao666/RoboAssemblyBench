"""Verify one transfer and install its complete original videos and node clips."""
import argparse,json,shutil,subprocess,sys,tarfile
from pathlib import Path
from beam_workflow import sha,save

def extract(source,destination):
    with tarfile.open(source) as archive:
        for m in archive.getmembers():
            n=Path(m.name)
            if not m.isfile() or n.is_absolute() or '..' in n.parts:raise ValueError('Unsafe transferred member')
            target=destination/n;target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(archive.extractfile(m).read())

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('archive',type=Path);a=ap.parse_args()
    p=Path(__file__).resolve().parents[1]
    with tarfile.open(a.archive) as t:manifest=json.loads(t.extractfile('transfer_manifest.json').read())
    run,cycle=manifest['run'],manifest['cycle'];label=run.split('_')[0]
    stage=p/(cycle+'_transfer_metadata');stage.mkdir(exist_ok=True);extract(a.archive,stage)
    checks=[]
    for row in manifest['files']:
        f=stage/row['relative_path']
        if f.stat().st_size!=row['bytes'] or sha(f)!=row['sha256']:raise ValueError('Transfer SHA mismatch: '+str(f))
        target=p/row['relative_path']
        if row['relative_path']=='repair_batch_D_20261016.json':target=p/(cycle+'_server_batch_at_download.json')
        if row['relative_path']=='batch_D_20261016/timing_events.jsonl':
            target=target.with_name(cycle+'_timing_events_server.jsonl')
            rolling=target.with_name('timing_events_server.jsonl')
            if not rolling.exists() or f.read_bytes().startswith(rolling.read_bytes()):shutil.copy2(f,rolling)
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,target)
        checks.append(dict(server_relative_path=row['relative_path'],local_path=str(target.resolve()),sha256=sha(target),bytes=target.stat().st_size))
    subprocess.run([sys.executable,str(p/'tools/verify_review_bundle.py'),label,str(p/(run+'_review_bundle.tar.gz'))],check=True)
    extract(p/(label+'_key_nodes.tar.gz'),p)
    nodes=json.loads((p/(label+'_key_nodes/manifest.json')).read_text());clips=[]
    for row in nodes['clips']:
        f=p/(label+'_key_nodes')/Path(row['path']).name
        if f.stat().st_size!=row['size'] or sha(f)!=row['sha256'] or row['full_decode_exit']!=0:raise ValueError('Node clip verification failed')
        clips.append(dict(path=str(f.resolve()),bytes=f.stat().st_size,sha256=sha(f)))
    videos=json.loads((p/(label+'_video_download_verification.json')).read_text())
    report=dict(passed=True,run=run,cycle=cycle,transferred_files=checks,videos=videos,key_clips=clips,original_videos=len(videos['videos']),original_video_bytes=sum(v['size'] for v in videos['videos']),scope='All three complete originals and reached key clips match server SHA/size; original server full-decode results retained. Full raw native archive remains on server.')
    save(p/(cycle+'_transfer_verification.json'),report)
    print(json.dumps({k:report[k] for k in ('passed','run','original_videos','original_video_bytes')}|dict(key_clips=len(clips))))
if __name__=='__main__':main()
