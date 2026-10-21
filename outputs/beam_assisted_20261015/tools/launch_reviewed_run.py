"""Launch the next reviewed D attempt through the unchanged normal launcher."""
from pathlib import Path
import argparse,datetime,json,subprocess,sys
from beam_workflow import sha,save,validate_attempt_window
from check_launch_gate import validate

def preflight_scene_network(gate,p,run):
    """Read identical primary URL before any Popen/native attempt claim."""
    spec=json.loads(Path(gate['delta_manifest']['path']).read_text())
    expected=spec.get('bounded_primary_scene_probe')
    if expected is None:return None
    helper=p/'tools/bounded_scene_http_probe.py'
    assert sha(helper)==expected['helper_sha256']
    from bounded_scene_http_probe import fetch
    path=p/'network_preflight'/run/(datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
    try:
        result=fetch()
        assert result['url']==expected['url'] and result['sha256']==expected['sha256']
    except Exception as exc:
        save(path,dict(passed=False,run=run,error=repr(exc),native_started=False,tool_sha256=sha(__file__)))
        raise RuntimeError('Primary URL preflight failed before native attempt claim; no attempt consumed') from exc
    save(path,dict(passed=True,run=run,result=result,helper_sha256=sha(helper),native_started=False,tool_sha256=sha(__file__)))
    return dict(path=str(path),sha256=sha(path))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');ap.add_argument('cycle');a=ap.parse_args()
    p=Path(__file__).resolve().parents[1];batch=json.loads((p/'repair_batch_D_20261016.json').read_text())
    started=batch['native_attempts_started'];cap=batch['maximum_native_attempts']
    validate_attempt_window(batch,a.run,a.cycle)
    if batch['next_native_run_authorized'] is not True:raise ValueError('Attempt authorization/cap/previous closure invalid')
    row=batch['cycles'][started]
    if row['id']!=a.cycle or row['planned_run']!=a.run or row.get('native_started') or batch['status']!=a.cycle+'_ready':raise ValueError('Next reviewed cycle identity/state mismatch')
    for root,manifest in ((p/'assisted_repository','assisted_manifest.json'),(p/'tools','assisted_tools_manifest.json'),(p.parent/'candidate_repository','physical_manifest.json')):
        data=json.loads((p/'tools'/manifest).read_text())
        if not all(sha(root/name)==value for name,value in data.items()):raise ValueError('Source/tool authority changed')
    if str(batch.get('workflow_version','')).startswith('W05'):
        manifest=json.loads((p.parent/'beam_routes/docs_manifest.json').read_text())
        receipt=batch.get('documents_sync_receipt') or {}
        if sha(receipt['path'])!=receipt.get('sha256'):
            raise ValueError('Current explicit document receipt changed')
        sync=json.loads(Path(receipt['path']).read_text())
        expected={str(Path(root)/name) for root in manifest['server_mirrors'] for name in manifest['files']}
        if not(sync.get('passed') is True and sync.get('roots')==manifest['server_mirrors']
               and sync.get('copies')==len(expected) and len(sync['files'])==len(expected)
               and {v['path'] for v in sync['files']}==expected
               and all(sha(v['path'])==v['sha256'] for v in sync['files'])):
            raise ValueError('Explicit manifest document synchronization stale')
    else:
        sync=json.loads((p/'latest_workflow_sync.json').read_text())
        if not(sync['passed'] is True and sync['coordinated_document_copies']==24 and all(sha(v['path'])==v['sha256'] for v in sync['files'])):
            raise ValueError('Current coordination synchronization stale')
    gatepath=p/'launch_gates'/(a.run+'.json');gate=json.loads(gatepath.read_text())
    validate(gate,run_name=a.run,source_manifest=p/'tools/assisted_manifest.json',tools_manifest=p/'tools/assisted_tools_manifest.json')
    if (p/'runs'/a.run).exists():raise ValueError('Run output already exists')
    network=preflight_scene_network(gate,p,a.run)
    logfile=p/(a.run.split('_')[0]+'_launch.log')
    with logfile.open('xb') as output:
        proc=subprocess.Popen([sys.executable,str(p/'tools/beam_workflow.py'),'run',str(p/'batch_D_20261016'),a.cycle+'_native','native','--','bash',str(p/'tools/run_beam_route.sh'),'assisted',a.run,'0'],cwd=p,stdin=subprocess.DEVNULL,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    receipt=dict(run=a.run,cycle=a.cycle,pid=proc.pid,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),launcher='normal source-bound assisted route; atomic claim under existing flock',launch_log=str(logfile),gate_sha256=sha(gatepath),source_manifest_sha256=sha(p/'tools/assisted_manifest.json'),tools_manifest_sha256=sha(p/'tools/assisted_tools_manifest.json'),network_preflight=network,tool_sha256=sha(__file__))
    save(p/(a.cycle+'_launch_receipt.json'),receipt);print(json.dumps(receipt))
if __name__=='__main__':main()
