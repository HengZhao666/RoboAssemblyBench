"""Complete visible-mesh and camera audits of the cycle's changed saved path."""
from pathlib import Path
import concurrent.futures,json,subprocess,sys
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');cycle=sys.argv[1]
paths=['base_v11_capture_path.jsonl',f'nominal_remaining_all_{cycle}_path.jsonl']
camera=P/(cycle+'_camera_full_path.json');mesh=P/(cycle+'_visible_full_path.json')
offsets={'reviewed_side':{r:[-.12,0.,.08] for r in ('franka_left','franka_right')}}
commands=[('visible',[sys.executable,str(P/'tools/audit_named_visible_paths.py'),mesh.name,*paths]),
 ('camera',[sys.executable,str(P/'tools/camera_visibility_preflight.py'),
 '--geometry',str(P/'runs/A08b_beam_full/scene_geometry.json'),
 '--trajectory',str(P/'runs/A10b_beam_full/entity_geometry_trace.jsonl'),
 *sum((['--trajectory',str(P/n)] for n in paths),[]),
 '--offsets-json',json.dumps(offsets),'--output',str(camera)])]
def run(item):
 name,args=item
 with (P/(cycle+'_'+name+'_audit.log')).open('w') as f:
  return name,subprocess.run(args,stdout=f,stderr=subprocess.STDOUT).returncode
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:codes=dict(pool.map(run,commands))
m=json.loads(mesh.read_text()) if mesh.exists() else {}
c=json.loads(camera.read_text()) if camera.exists() else {}
stats=c.get('results',{}).get('reviewed_side',{})
passed=bool(all(v==0 for v in codes.values()) and m.get('passed') and stats.get('samples',0)>0
 and stats.get('pinch_visibility_failures')==0 and stats.get('eye_failures')==0)
report={'passed':passed,'cycle':cycle,'exit_codes':codes,'visible_samples':m.get('samples'),
 'visible_non_target_failure_pairs':len(m.get('failures_above_precision',[])),
 'visible_grasp_failure_pairs':len(m.get('grasp_failures_above_precision',[])),
 'visible_meshes':c.get('visible_meshes'),'camera_checks':stats.get('samples'),
 'camera_visibility_failures':stats.get('pinch_visibility_failures'),
 'eye_failures':stats.get('eye_failures'),'min_visible_samples_per_pad':stats.get('min_visible_samples'),
 'evidence':[str(mesh),str(camera)],'scope':'complete nominal five-part54-mesh and two-camera paths; original native A10b camera states retained',
 'not_certified':['new actual dynamic controller trajectory','rendered image freshness/readability',
 'between-sample CCD','unexported warehouse','native cooked/SDF versus visible mesh']}
(P/(cycle+'_geometry_review.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report));raise SystemExit(0 if passed else 3)
