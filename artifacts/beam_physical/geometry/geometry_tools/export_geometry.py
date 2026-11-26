"""One-time, read-only geometry export for later complete articulation sweeps."""
from dataclasses import asdict
import json
import numpy as np

def export_geometry(monitor,out):
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 task=monitor.task;stage=monitor.stage;cache=UsdGeom.XformCache()
 meshes=[];joints=[];gaps=[]
 for path,group,robot,object_name in monitor.roots:
  root=stage.GetPrimAtPath(path)
  if not root or not root.IsValid():continue
  for prim in Usd.PrimRange(root,Usd.TraverseInstanceProxies()):
   if prim.IsA(UsdPhysics.Joint):
    joint=UsdPhysics.Joint(prim)
    joints.append({'path':str(prim.GetPath()),'robot':robot,'type':str(prim.GetTypeName()),
     'parent':[str(x) for x in joint.GetBody0Rel().GetTargets()],
     'child':[str(x) for x in joint.GetBody1Rel().GetTargets()],
     'pos0':list(joint.GetLocalPos0Attr().Get()),'rot0':list(joint.GetLocalRot0Attr().Get().GetImaginary())+[float(joint.GetLocalRot0Attr().Get().GetReal())],
     'pos1':list(joint.GetLocalPos1Attr().Get()),'rot1':list(joint.GetLocalRot1Attr().Get().GetImaginary())+[float(joint.GetLocalRot1Attr().Get().GetReal())],
     'axis':prim.GetAttribute('physics:axis').Get(),
     'enabled':joint.GetJointEnabledAttr().Get()})
   if not prim.IsA(UsdGeom.Gprim):continue
   image=UsdGeom.Imageable(prim)
   if image.ComputeVisibility()=='invisible' or image.ComputePurpose() not in ('default','render'):continue
   if prim.IsA(UsdGeom.Mesh):
    mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get(),float)
    counts=mesh.GetFaceVertexCountsAttr().Get();indices=mesh.GetFaceVertexIndicesAttr().Get();faces=[];offset=0
    for count in counts:
     face=list(indices[offset:offset+count]);offset+=count
     for i in range(1,count-1):faces.append([face[0],face[i],face[i+1]])
   elif prim.IsA(UsdGeom.Cube):
    size=float(UsdGeom.Cube(prim).GetSizeAttr().Get())/2
    points=np.array([[x,y,z] for x in [-size,size] for y in [-size,size] for z in [-size,size]])
    faces=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]]
   else:
    gaps.append({'prim':str(prim.GetPath()),'reason':'unsupported_visible_gprim','type':str(prim.GetTypeName())});continue
   owner=prim
   while owner and owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
    if owner==root:break
    owner=owner.GetParent()
   if not owner or not owner.IsValid():owner=root
   body=str(owner.GetPath());world=cache.GetLocalToWorldTransform(owner)
   rigid=Gf.Matrix4d(1);rigid.SetRotate(world.ExtractRotationQuat());rigid.SetTranslateOnly(world.ExtractTranslation())
   transform=cache.GetLocalToWorldTransform(prim)*rigid.GetInverse()
   local=np.asarray([list(transform.Transform(Gf.Vec3d(*map(float,p)))) for p in points])
   meshes.append({'prim':str(prim.GetPath()),'body':body,'group':group,'robot':robot,'object':object_name,
    'points':local.tolist(),'triangles':faces,'body_rest':np.asarray(rigid).T.tolist()})
 robots={}
 for name,robot in task.robots.items():
  robots[name]={'joint_names':list(robot.articulation.dof_names),
   'joint_positions':np.asarray(robot.articulation.get_joint_positions()).tolist(),
   'root_path':str(robot.articulation._root_prim_path)}
 report={'scope':'task robot/object roots; visible meshes including shells and noncolliding table legs; authored collision convex points; full-path/native initial mapping',
  'colliders':[asdict(c) for c in monitor.checker.colliders],
  'body_rest':{k:v.tolist() for k,v in monitor.current_poses().items()},
  'visible_meshes':meshes,'joints':joints,'robots':robots,'gaps':gaps,
  'scene_background_root_not_exported':True,'physics_stamp':task._beam_physics_stamp(),
  'resolved_scene_asset':getattr(task.cfg,'scene_asset_path',None),
  'scene_asset_source':getattr(task.cfg,'scene_asset_source',None),
  'scene_asset_selection_policy':getattr(task.cfg,'scene_asset_selection_policy',None)}
 (out/'scene_geometry.json').write_text(json.dumps(report,default=lambda x:x.tolist())+'\n')
 return report
