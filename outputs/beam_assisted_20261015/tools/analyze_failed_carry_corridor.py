"""Read-only comparison of failed actual/published CAD pair and nominal reuse domain."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];sys.path[:0]=[str(P/'assisted_repository'),str(P.parent/'runtime_tools/geometry_deps')]
import fcl
from beam_workflow import save,sha
from index_run import select
from sweep_model import SceneModel,pose_matrix
from visual_mesh_check import VisualMeshChecker
from toolkits.factory_dual_franka_assembly.beam_assisted_servo import anchored_object_prediction,object_pose_from_tcp
run,cycle=sys.argv[1:3];d=P/'runs'/run;result=json.loads((d/'collect_results.json').read_text())[0];last=result['phase_transition_history'][-1];failure=last['first_failure']['non_target'][0];step=failure['step'];phase=failure['phase'];model=SceneModel(d/'scene_geometry.json');checker=VisualMeshChecker(model,include_self=True)
na=select(d,'physics_boundary_trace.jsonl',step,step,'before_action')[0];en=select(d,'entity_geometry_trace.jsonl',step,step)[0];actual={k:np.asarray(v,float) for k,v in en['body_poses'].items()};qa={k:np.asarray(v['joint_positions'][0],float) for k,v in na['robots'].items()}
bodya,bodyb=failure['body_a'],failure['body_b'];indices=[next(i for i,m in enumerate(checker.meshes) if m['body']==b) for b in (bodya,bodyb)]
def pair(poses):
 for i in indices:
  t=poses[checker.meshes[i]['body']];checker.shapes[i].setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
 out=fcl.CollisionResult();hit=fcl.collide(*(checker.shapes[i] for i in indices),fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),out)
 return dict(intersects=bool(hit),depth_m=max((float(c.penetration_depth) for c in out.contacts),default=0.),distance_m=None if hit else float(fcl.distance(*(checker.shapes[i] for i in indices),fcl.DistanceRequest(),fcl.DistanceResult())))
actual_pair=pair(actual);assert abs(actual_pair['depth_m']-failure['depth'])<1e-9
rows=[json.loads(l) for l in (P/'C01_bimanual_repaired_v2_path.jsonl').open()];rows=[r for r in rows if r['phase']==phase];assert rows
checks=[];bname='fabrica_beam_2';closest=None
for i,r in enumerate(rows):
 q={k:np.asarray(v,float) for k,v in r['q'].items()};poses=model.poses(q);poses[bodyb]=np.asarray(r['part_poses'][bname]);nominal=pair(poses)
 dp=float(np.linalg.norm(poses[bodyb][:3,3]-actual[bodyb][:3,3]));angle=float(Rotation.from_matrix(actual[bodyb][:3,:3].T@poses[bodyb][:3,:3]).magnitude())
 ra=float(np.max(abs(q['franka_right'][:6]-qa['franka_right'][:6])));la=float(np.max(abs(q['franka_left'][:6]-qa['franka_left'][:6])))
 q['franka_right']=qa['franka_right'].copy();poses2=model.poses(q);poses2[bodyb]=poses[bodyb];actual_right=pair(poses2)
 row=dict(path_index=r.get('path_index',i),nominal_pair=nominal,same_nominal_payload_actual_right_pair=actual_right,payload_position_delta_m=dp,payload_angle_delta_rad=angle,left_q_max_delta_rad=la,right_q_max_delta_rad=ra)
 checks.append(row)
 if closest is None or dp<closest['payload_position_delta_m']:closest=row
# Original accepted SDK target, not an IK proposal, evaluated at0/.5/1 fractions.
accepted=select(d,'physics_boundary_trace.jsonl',step,step,'after_action')[0];qp={k:v.copy() for k,v in qa.items()}
for who,rs in accepted['robots'].items():qp[who][:6]=next(a['joint_positions'] for a in rs['sdk_submission']['actions'] if a.get('joint_indices')==list(range(6)))
relation=np.asarray(json.loads((P/(cycle+'_handoff_analysis.json')).read_text())['captured_controller_relation']);tcp=model.data['robots']['franka_left']['root_path']+'/wrist_3_link'
def pose(t):return dict(position=t[:3,3],orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]])
rp=pose(relation);origin=model.poses(qa);actual_object=pose(actual[bodyb]);actual_model=object_pose_from_tcp(pose(origin[tcp]),rp['position'],rp['orientation']);published=[]
for fraction in (0.,.5,1.):
 q={k:qa[k]+fraction*(qp[k]-qa[k]) for k in qa};fk=model.poses(q);poses={b:(t@np.linalg.inv(origin[b])@fk[b] if b in origin and '/robots/' in b else t) for b,t in actual.items()}
 predicted=anchored_object_prediction(actual_object,actual_model,object_pose_from_tcp(pose(fk[tcp]),rp['position'],rp['orientation']));poses[bodyb]=pose_matrix(predicted['position'],predicted['orientation']);published.append(dict(fraction=fraction,pair=pair(poses)))
report=dict(passed_analysis=True,run=run,step=step,phase=phase,actual_pair=actual_pair,original_stop_pair=failure,nominal_phase_states=len(rows),nominal_intersections_over1um=sum(c['nominal_pair']['depth_m']>1e-6 for c in checks),actual_right_nominal_payload_intersections_over1um=sum(c['same_nominal_payload_actual_right_pair']['depth_m']>1e-6 for c in checks),closest_nominal_payload=closest,nominal_minimum_gap_m=min((c['nominal_pair']['distance_m'] for c in checks if c['nominal_pair']['distance_m'] is not None),default=None),actual_final_SDK_sweep=published,checks=checks,tool_sha256=sha(__file__),nominal_path_sha256=sha(P/'C01_bimanual_repaired_v2_path.jsonl'),source_after_sha256=sha(d/'source_after.json'),scope='Exact failed visible mesh pair; finite nominal path states and3finalSDK sweep samples. No native physics or new repair. Nominal input/code SHA alone does not establish membership of actual carried pose/q in a reused corridor.')
save(P/(cycle+'_failed_carry_corridor.json'),report);print(json.dumps({k:v for k,v in report.items() if k not in ['checks','original_stop_pair']}))
