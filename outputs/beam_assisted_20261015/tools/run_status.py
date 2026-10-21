from pathlib import Path
import json, sys, collections
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
D=P/'runs'/sys.argv[1]
def tail_row(name):
 p=D/name
 if not p.exists():return None
 with p.open('rb') as f:
  f.seek(max(0,p.stat().st_size-100000));lines=f.read().splitlines()
 for line in reversed(lines):
  try:return json.loads(line)
  except Exception:pass
out={'run':D.name,'finished':(D/'finished_at').exists()}
if (D/'collect_results.json').exists():
 try:
  data=json.loads((D/'collect_results.json').read_text())
  if isinstance(data,list) and data:
   r=data[0];out['result']={k:r.get(k) for k in ['success','failed','steps','phase_status','failure_reason','phase_transition_history','assisted_boundary_stop','post_failure_recording']}
 except Exception:pass
for name in ['motion_trace.jsonl','assisted_trace.jsonl','contact_trace.jsonl']:
 r=tail_row(name)
 if r is not None:
  out[name]={k:v for k,v in r.items() if k in ['event','step','phase','robot','object','current_pose','target_pose','command_target_pose','current_q','command_q','published_q','actual_q','failure_reason','detail']}
if '--events' in sys.argv:
 events=[]
 for line in (D/'assisted_trace.jsonl').open():
  try:r=json.loads(line)
  except Exception:continue
  if r.get('event')!='published_command':events.append(r)
 out['events']=events
print(json.dumps(out,ensure_ascii=False))
