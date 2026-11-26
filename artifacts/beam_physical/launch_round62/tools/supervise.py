"""One persistent authorized native attempt; stop only this worker if needed."""
from pathlib import Path
import datetime
import json
import os
import shutil
import signal
import subprocess
import time
from resource_budget import GIB, startup_required, check_budget, startup_memory_required

ROOT = Path('/home/user/beam_integration_20261010_upstream6f57dae')
TOOLS = Path(__file__).resolve().parent.parent
ROUND = int(os.environ['BEAM_REPAIR_ROUND'])
OUT = ROOT/'candidate_repository/outputs'/os.environ['BEAM_OUTPUT_NAME']
LEDGER = ROOT/'grasp_repairs_20261012_20rounds/round_ledger.json'


def now(): return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write(path,data):
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temp.replace(path)


def memory_available():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):return int(line.split()[1])*1024


def validate_authorization(admission, number):
    assert not admission['paused'] and not admission.get('pause_pending') and not admission.get('authorization_exhausted')
    assert admission['active_round'] == number and number <= admission['maximum_rounds']
    batch = admission['current_batch']
    assert not batch['paused'] and batch['remaining_rounds'] > 0
    assert batch['round_numbers'][0] <= number <= batch['round_numbers'][1]
    entry = next(r for r in admission['rounds'] if r['round'] == number)
    assert entry['native_started'] is False and entry['status'] == 'ready_for_native'
    return entry

admission=json.loads(LEDGER.read_text())
entry=validate_authorization(admission, ROUND)
assert not OUT.exists()
prefix_budget=int(os.environ.get('BEAM_PREFIX_OUTPUT_BUDGET_GIB','0'))*GIB
budget_gib=int(os.environ.get('BEAM_FULL_OUTPUT_BUDGET_GIB','20'))
assert 14 <= budget_gib <= 20
if budget_gib != 20:
    assert os.environ.get('BEAM_STREAMING_CAPTURE') == '1'
    proof=json.loads((TOOLS/'capture_output_contract.json').read_text())
    assert proof['encoder_unit_test_passed'] and proof['full_output_budget_GiB']==budget_gib
full_budget=budget_gib*GIB
reserve=5*GIB
checkpoint=int(os.environ.get('BEAM_RESOURCE_CHECKPOINT_PHASE_INDEX','47'))
assert shutil.disk_usage(ROOT).free >= startup_required(prefix_budget,full_budget,reserve)
ledger_admission=json.loads(LEDGER.read_text())
assert not ledger_admission.get('authorization_exhausted') and ledger_admission['current_batch']['remaining_rounds'] > 0
assert ledger_admission['current_batch']['round_numbers'][0] <= ROUND <= ledger_admission['current_batch']['round_numbers'][1] and ROUND <= ledger_admission['maximum_rounds']
minimum_memory=startup_memory_required(int(os.environ.get('BEAM_START_MEMORY_MIN_GIB','20')),
    ledger_admission.get('shared_resource_permission',{}).get('allowed') is True,
    batch=ledger_admission.get('current_batch'),round_number=ROUND)
assert memory_available() >= minimum_memory
gpu=subprocess.run(['nvidia-smi','--query-compute-apps=pid,gpu_uuid,process_name,used_memory','--format=csv,noheader'],capture_output=True,text=True,check=True)
other_gpu_processes=[row for row in gpu.stdout.splitlines() if row.strip() and 'gnome-remote-desktop' not in row]
if other_gpu_processes:
    assert ledger_admission.get('shared_resource_permission',{}).get('allowed') is True,gpu.stdout
    free_gpu=subprocess.run(['nvidia-smi','--id=GPU-e064bda0-b445-2d7f-50d5-c1180deadea2','--query-gpu=memory.free','--format=csv,noheader,nounits'],capture_output=True,text=True,check=True)
    assert int(free_gpu.stdout.strip())>=12288,free_gpu.stdout
validate_authorization(json.loads(LEDGER.read_text()), ROUND)
OUT.mkdir()
write(OUT/'startup_resource_admission.json',dict(time=now(),minimum_start_memory_bytes=minimum_memory,
    startup_memory_authorization=ledger_admission.get('current_batch',{}).get('startup_memory_authorization'),
    observed_memory_available_bytes=memory_available(),observed_disk_free_bytes=shutil.disk_usage(ROOT).free,
    required_disk_free_bytes=startup_required(prefix_budget,full_budget,reserve),
    runtime_memory_floor_bytes=2*GIB,runtime_disk_floor_bytes=5*GIB,
    shared_permission=ledger_admission.get('shared_resource_permission')))
d=json.loads(LEDGER.read_text());assert not d['paused'] and d['active_round']==ROUND
row=next(r for r in d['rounds'] if r['round']==ROUND)
row.update(status='native_starting',native_started=False,launch_attempted=True,output=str(OUT),started_at=now(),
           acceptance_profile=os.environ.get('BEAM_PHYSICAL_ACCEPTANCE_PROFILE','plumbers_staged_v2'),reference='baiyu858/RoboAssemblyBench@9497b839',
           user_acceptance_confirmation=d['round51_final_acceptance_approval']['human_answer'])
write(LEDGER,d)
log=(OUT/'runner.log').open('w')
process=subprocess.Popen(['bash',str(TOOLS/'run_full_assembly.sh')],cwd=ROOT/'candidate_repository',
                         stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
row['pid']=process.pid;write(LEDGER,d)
write(TOOLS/'launch.json',dict(pid=process.pid,supervisor_pid=os.getpid(),output=str(OUT),started_at=now()))
requested=None
last_record=0
full_budget_admitted=not bool(prefix_budget)
while process.poll() is None:
    disk=shutil.disk_usage(ROOT);available=memory_available();clock=time.monotonic()
    if clock-last_record>=60:
        if not full_budget_admitted:
            progress_path=OUT/'progress.json'
            progress=json.loads(progress_path.read_text()) if progress_path.exists() else {}
            size=0
            for parent,dirs,names in os.walk(OUT):
                for name in names:
                    try:size+=(Path(parent)/name).stat().st_size
                    except FileNotFoundError:pass
            admission=check_budget(disk.free,size,progress.get('phase_index'),checkpoint,
                prefix_budget,full_budget,reserve)
            admission.update(time=now(),stage='full' if admission['full_due'] else 'measured_prefix',
                assembly_acceptance_unchanged=True)
            write(OUT/'storage_admission.json',admission)
            if admission['passed'] and admission['full_due']:
                full_budget_admitted=True
            elif not admission['passed'] and requested is None:
                requested=clock
                write(OUT/'resource_stop_request.json',dict(time=now(),reason='insufficient remaining output budget',budget=admission))
        with (OUT/'resource_snapshots.jsonl').open('a') as f:
            f.write(json.dumps(dict(time=now(),free_bytes=disk.free,memory_available_bytes=available,
                                   load=os.getloadavg(),worker_pid=process.pid))+'\n')
        last_record=clock
    if (disk.free<5*1024**3 or available<2*1024**3) and requested is None:
        requested=clock
        write(OUT/'resource_stop_request.json',dict(time=now(),free_bytes=disk.free,
            memory_available_bytes=available,reason='project output reserve or host memory floor'))
    if requested is not None and clock-requested>900:
        os.killpg(process.pid,signal.SIGTERM)
    time.sleep(10)
code=process.wait();log.close()
current=json.loads(LEDGER.read_text());record=next(r for r in current['rounds'] if r['round']==ROUND)
result_path=OUT/'collect_results.json'
result=json.loads(result_path.read_text()) if result_path.is_file() else None
record.update(status='native_finished_pending_review' if (OUT/'progress.json').exists() else 'startup_failed_no_native_worker',native_started=(OUT/'progress.json').exists(),pid=None,exit_code=code,finished_at=now(),
              result_file=str(result_path) if result is not None else None)
current['active_round']=None
current['completed_rounds']=max(current['completed_rounds'],ROUND)
current['next_round']=ROUND+1
current['current_batch']['completed_rounds']=ROUND-current['current_batch']['round_numbers'][0]+1
current['current_batch']['remaining_rounds']=current['current_batch']['maximum_rounds']-current['current_batch']['completed_rounds']
if current['current_batch']['remaining_rounds']==0:
    current['authorization_exhausted']=True
    current['stop_reason']='Authorized batch '+str(current['current_batch']['round_numbers'])+' consumed; no additional native run without new human authorization.'
if current['current_batch'].get('pause_after_current_round'):
    current['paused']=True;current['native_continuation_authorized']=False
    current['current_batch']['paused']=True
    current['pause_reason']='Authorized single51 attempt ended;52/53 remain held.'
write(LEDGER,current)
write(TOOLS/'native_finished.json',dict(exit_code=code,result=result,finished_at=now(),
                                      acceptance_pending_review=True,disk_after=shutil.disk_usage(ROOT)._asdict()))
