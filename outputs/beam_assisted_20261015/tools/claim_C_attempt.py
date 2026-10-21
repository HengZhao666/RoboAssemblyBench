"""Claim one authorized C-batch native attempt; called under the Beam run lock."""
from pathlib import Path
import json,os,sys

def claim(path, run_name, cycle):
    path=Path(path);batch=json.loads(path.read_text())
    cap=batch['maximum_native_attempts'];started=batch['native_attempts_started']
    if not (type(cap) is int and 0 < cap <= 5 and type(started) is int and 0 <= started < cap):
        raise ValueError('C native attempt cap reached or invalid')
    if batch.get('next_native_run_authorized') is not True:
        raise ValueError('C batch is paused or not authorized')
    row=next((r for r in batch['cycles'] if r['id']==cycle),None)
    if row is None or row['planned_run']!=run_name or row.get('native_started'):
        raise ValueError('C attempt identity mismatched or already started')
    if row['id'] != batch['cycles'][started]['id']:
        raise ValueError('C native attempts must follow the bounded batch order')
    row.update(native_started=True,state='native_starting',actual_run=run_name)
    batch.update(native_attempts_started=started+1,current_cycle=cycle,status='native_running')
    temp=path.with_suffix('.claim.tmp');temp.write_text(json.dumps(batch,ensure_ascii=False,indent=2)+'\n');os.replace(temp,path)
    return {'claimed_cycle':cycle,'run':run_name,'native_attempts_started':started+1,'maximum_native_attempts':cap}

if __name__=='__main__':
    P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
    gate=json.loads(Path(sys.argv[1]).read_text())
    if str(gate.get('cycle','')).startswith('D'):
        from beam_workflow import claim as claim_current_batch
        print(json.dumps(claim_current_batch(P/'repair_batch_D_20261016.json',gate['run_name'],gate['cycle'])))
        raise SystemExit(0)
    if not str(gate.get('cycle','')).startswith('C'):raise SystemExit(0)
    print(json.dumps(claim(P/'repair_batch_C_20261015.json',gate['run_name'],gate['cycle'])))

