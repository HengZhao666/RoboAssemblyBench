"""Full-path joint graph sweeps calibrated to native initial body poses."""
from pathlib import Path
import sys,json
import numpy as np
C=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015/assisted_repository')
sys.path[:0]=[str(C),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
from toolkits.constraint_checking.integration.entity_geometry import pose_matrix,Collider,EntityGeometryChecker

def axis_rotation(axis,angle):
 v=np.eye(3)['XYZ'.index(axis)];K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
 m=np.eye(4);m[:3,:3]=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*K@K
 return m

def joint_pose(j):
 q=j['rot0'];return pose_matrix(j['pos0'],[q[3],*q[:3]])

class SceneModel:
 def __init__(self,path):
  self.data=json.loads(Path(path).read_text());d=self.data
  self.rest={k:np.asarray(v,float) for k,v in d['body_rest'].items()}
  for mesh in d['visible_meshes']:self.rest.setdefault(mesh['body'],np.asarray(mesh['body_rest'],float))
  self.joints={};self.gaps=[]
  for name,r in d['robots'].items():
   mapping={n:i for i,n in enumerate(r['joint_names'])};entries=[]
   for j in d['joints']:
    if j['robot']!=name or not j['enabled'] or len(j['parent'])!=1 or len(j['child'])!=1:continue
    parent,child=j['parent'][0],j['child'][0]
    if parent not in self.rest or child not in self.rest:
     self.gaps.append({'joint':j['path'],'reason':'missing_native_rest_body'});continue
    if j['type'] not in ('PhysicsFixedJoint','PhysicsRevoluteJoint'):
     self.gaps.append({'joint':j['path'],'reason':'unsupported_joint_type','type':j['type']});continue
    index=mapping.get(j['path'].rsplit('/',1)[-1])
    if j['type']=='PhysicsRevoluteJoint' and index is None:
     self.gaps.append({'joint':j['path'],'reason':'missing_native_dof_name'});continue
    anchor=joint_pose(j)
    entries.append({**j,'parent':parent,'child':child,'index':index,
      'anchor':anchor,'anchor_inverse':np.linalg.inv(anchor),
      'relative':np.linalg.inv(self.rest[parent])@self.rest[child]})
   self.joints[name]=entries
  self.checker=EntityGeometryChecker([Collider(**{**c,'points':np.asarray(c['points'])}) for c in d['colliders']],proximity=.003)
 def poses(self,joint_positions):
  result=dict(self.rest)
  for name,entries in self.joints.items():
   initial=np.asarray(self.data['robots'][name]['joint_positions'],float)
   q=np.asarray(joint_positions[name],float)
   if q.shape!=initial.shape:raise ValueError('full articulation joint count mismatch')
   children={j['child'] for j in entries};done=set(self.rest)-children;pending=list(entries)
   while pending:
    progressed=False
    for j in pending[:]:
     if j['parent'] not in done:continue
     delta=np.eye(4) if j['index'] is None else axis_rotation(j['axis'],q[j['index']]-initial[j['index']])
     result[j['child']]=result[j['parent']]@j['anchor']@delta@j['anchor_inverse']@j['relative']
     done.add(j['child']);pending.remove(j);progressed=True
    if not progressed:raise ValueError('joint graph unresolved: '+str([j['path'] for j in pending]))
  return result
 def initial_validation(self):
  q={n:r['joint_positions'] for n,r in self.data['robots'].items()};p=self.poses(q)
  return {'max_native_rest_matrix_error':max(float(np.max(abs(p[k]-v))) for k,v in self.rest.items()),
          'colliders':len(self.checker.colliders),'rest_bodies':len(self.rest),
          'visible_meshes':len(self.data['visible_meshes']),'joint_graph_gaps':self.gaps,
          'joints':{n:len(j) for n,j in self.joints.items()},
          'scope':'initial calibration only; nonzero native q validation required before nominal sweeps certify coverage'}
if __name__=='__main__':
 m=SceneModel(sys.argv[1]);print(json.dumps(m.initial_validation()))
