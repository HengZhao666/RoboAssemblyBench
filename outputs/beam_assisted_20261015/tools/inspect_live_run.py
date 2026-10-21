"""Compact read-only live state; never modifies or imports the simulator."""
import json,sys,time
from pathlib import Path
p=Path(__file__).resolve().parents[1];d=p/'runs'/sys.argv[1]
def load(f):
 try:return json.loads((d/f).read_text())
 except (OSError,ValueError):return None
def tail(f,size=262144):
 path=d/f
 if not path.exists():return []
 with path.open('rb') as s:
  s.seek(max(0,path.stat().st_size-size));data=s.read()
 rows=[]
 for line in data.splitlines():
  try:rows.append(json.loads(line))
  except ValueError:pass
 return rows
rows=tail('assisted_trace.jsonl');events={}
for r in rows:
 if r.get('event') in ['absolute_object_servo','alignment_reference','cold_actual_height_violation','strict_assembly_gate','published_command']:
  events[r['event']]={k:v for k,v in r.items() if k not in ['actual_q','requested_q','published_q','command_tcp_pose','current_object_pose','command_object_pose']}
last=rows[-1] if rows else {};start=(d/'started_at').read_text().strip() if (d/'started_at').exists() else None
print(json.dumps(dict(run=d.name,started=start,finished=(d/'finished_at').exists(),startup_ready=(d/'startup_images_ready.json').exists(),ACK=(d/'startup_image_review.json').exists(),last={k:last.get(k) for k in ['event','step','phase','physics_stamp']},events=events,visible_guard=load('visible_mesh_guard_progress.json'),results=({k:v for k,v in load('collect_results.json').items() if k in ['success','steps','trials','errors']} if load('collect_results.json') else None))),flush=True)
