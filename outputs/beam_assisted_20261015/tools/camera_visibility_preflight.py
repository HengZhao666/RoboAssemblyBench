"""Read-only triangle visibility audit for virtual wrist cameras.

No simulator, FCL, USD or robot writes. Scene poses must come from a saved native
snapshot or the calibrated complete joint graph. Visibility is a mesh first-hit
test, not a rendered-image acceptance test. Open meshes and missing warehouse
geometry remain explicit limitations.
"""
from pathlib import Path
import argparse
import heapq
import json
import math
import numpy as np


def quaternion_matrix(q):
    w,x,y,z=np.asarray(q,float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def transform(pos,quat):
    T=np.eye(4);T[:3,:3]=quaternion_matrix(quat);T[:3,3]=pos
    return T


class PoseModel:
    def __init__(self,data):
        self.data=data;self.rest={k:np.asarray(v,float) for k,v in data['body_rest'].items()}
        for mesh in data['visible_meshes']:
            self.rest.setdefault(mesh['body'],np.asarray(mesh['body_rest'],float))
        self.joints={};self.gaps=[]
        for name,r in data['robots'].items():
            names={n:i for i,n in enumerate(r['joint_names'])};entries=[]
            for j in data['joints']:
                if j['robot']!=name or not j['enabled']:continue
                if len(j['parent'])!=1 or len(j['child'])!=1:
                    self.gaps.append(j['path']);continue
                parent,child=j['parent'][0],j['child'][0]
                if parent not in self.rest or child not in self.rest:
                    self.gaps.append(j['path']);continue
                if j['type'] not in ('PhysicsFixedJoint','PhysicsRevoluteJoint'):
                    self.gaps.append(j['path']);continue
                idx=names.get(j['path'].rsplit('/',1)[-1])
                if j['type']=='PhysicsRevoluteJoint' and idx is None:
                    self.gaps.append(j['path']);continue
                q=j['rot0'];anchor=transform(j['pos0'],[q[3],*q[:3]])
                entries.append((parent,child,idx,j.get('axis'),anchor,
                    np.linalg.inv(anchor),np.linalg.inv(self.rest[parent])@self.rest[child]))
            self.joints[name]=entries

    def poses(self,q):
        poses=dict(self.rest)
        for robot,entries in self.joints.items():
            actual=np.asarray(q[robot],float);initial=np.asarray(self.data['robots'][robot]['joint_positions'])
            if actual.shape!=initial.shape:raise ValueError('full articulation missing: '+robot)
            pending=list(entries);done=set(self.rest)-{j[1] for j in entries}
            while pending:
                progressed=False
                for item in pending[:]:
                    parent,child,idx,axis,A,invA,relative=item
                    if parent not in done:continue
                    delta=np.eye(4)
                    if idx is not None:
                        angle=actual[idx]-initial[idx];v=np.eye(3)['XYZ'.index(axis)]
                        K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
                        delta[:3,:3]=np.eye(3)+np.sin(angle)*K+(1-np.cos(angle))*K@K
                    poses[child]=poses[parent]@A@delta@invA@relative
                    done.add(child);pending.remove(item);progressed=True
                if not progressed:raise ValueError('joint graph unresolved')
        return poses


def point_triangle_distances(point,tri):
    a,b,c=tri[:,0],tri[:,1],tri[:,2];ab=b-a;ac=c-a;ap=point-a
    normal=np.cross(ab,ac);norm2=np.einsum('ij,ij->i',normal,normal)
    signed=np.einsum('ij,ij->i',ap,normal)
    proj=point-signed[:,None]*normal/np.maximum(norm2[:,None],1e-30)
    v=proj-a;aa=np.einsum('ij,ij->i',ab,ab);bb=np.einsum('ij,ij->i',ac,ac)
    cross=np.einsum('ij,ij->i',ab,ac);av=np.einsum('ij,ij->i',v,ab);bv=np.einsum('ij,ij->i',v,ac)
    den=aa*bb-cross*cross;u=(bb*av-cross*bv)/np.maximum(den,1e-30);w=(aa*bv-cross*av)/np.maximum(den,1e-30)
    inside=(norm2>1e-24)&(u>=0)&(w>=0)&(u+w<=1)
    result=np.where(inside,signed*signed/np.maximum(norm2,1e-30),np.inf)
    for first,second in ((a,b),(b,c),(c,a)):
        edge=second-first;fraction=np.clip(np.einsum('ij,ij->i',point-first,edge)/np.maximum(np.einsum('ij,ij->i',edge,edge),1e-30),0,1)
        err=point-first-fraction[:,None]*edge
        result=np.minimum(result,np.einsum('ij,ij->i',err,err))
    return result


class TriangleTree:
    def __init__(self,triangles):
        self.tri=np.asarray(triangles,float);lo=self.tri.min(1);hi=self.tri.max(1);centers=(lo+hi)*.5
        self.nodes=[]
        def build(indices):
            index=len(self.nodes);low=lo[indices].min(0);high=hi[indices].max(0)
            self.nodes.append(None)
            if len(indices)<=32:self.nodes[index]=(low,high,-1,-1,indices)
            else:
                axis=int(np.argmax(np.ptp(centers[indices],axis=0)));order=indices[np.argsort(centers[indices,axis])];mid=len(order)//2
                left=build(order[:mid]);right=build(order[mid:]);self.nodes[index]=(low,high,left,right,None)
            return index
        build(np.arange(len(self.tri)));self.low,self.high=self.nodes[0][:2]

    @staticmethod
    def ray_box(origin,direction,low,high,limit):
        near=0.;far=limit
        for axis in range(3):
            if abs(direction[axis])<1e-14:
                if origin[axis]<low[axis] or origin[axis]>high[axis]:return False
            else:
                a=(low[axis]-origin[axis])/direction[axis];b=(high[axis]-origin[axis])/direction[axis]
                near=max(near,min(a,b));far=min(far,max(a,b))
                if near>far:return False
        return True

    def first_hit(self,origin,direction,limit=1.00001):
        best=limit;found=False;stack=[0]
        while stack:
            low,high,left,right,ids=self.nodes[stack.pop()]
            if not self.ray_box(origin,direction,low,high,best):continue
            if ids is None:stack.extend((left,right));continue
            tri=self.tri[ids];a,b,c=tri[:,0],tri[:,1],tri[:,2];e1=b-a;e2=c-a
            h=np.cross(direction,e2);den=np.einsum('ij,ij->i',e1,h);good=abs(den)>1e-12
            inv=np.divide(1.,den,out=np.zeros_like(den),where=good);s=origin-a
            u=np.einsum('ij,ij->i',s,h)*inv;q=np.cross(s,e1)
            v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
            valid=good&(u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)&(t>1e-7)&(t<=best)
            if np.any(valid):best=float(t[valid].min());found=True
        return best if found else None

    def nearest_distance(self,point):
        best=float('inf');queue=[(0.,0)]
        while queue:
            bound,index=heapq.heappop(queue)
            if bound>=best:continue
            low,high,left,right,ids=self.nodes[index]
            if ids is not None:best=min(best,float(point_triangle_distances(point,self.tri[ids]).min()));continue
            for child in (left,right):
                lo,hi=self.nodes[child][:2];delta=np.maximum(np.maximum(lo-point,point-hi),0.);lower=float(delta@delta)
                if lower<best:heapq.heappush(queue,(lower,child))
        return math.sqrt(best)

    def winding(self,point):
        tri=self.tri-point;a,b,c=tri[:,0],tri[:,1],tri[:,2]
        na=np.linalg.norm(a,axis=1);nb=np.linalg.norm(b,axis=1);nc=np.linalg.norm(c,axis=1)
        numerator=np.einsum('ij,ij->i',a,np.cross(b,c))
        denominator=na*nb*nc+np.einsum('ij,ij->i',a,b)*nc+np.einsum('ij,ij->i',b,c)*na+np.einsum('ij,ij->i',c,a)*nb
        return float(np.arctan2(numerator,denominator).sum()/(2*np.pi))


def surface_samples(tri,count):
    area=np.linalg.norm(np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]),axis=1)
    centers=tri.mean(1);eligible=np.flatnonzero(area>max(area.max()*1e-5,1e-16))
    first=int(eligible[np.argmax(area[eligible])]);chosen=[first]
    distance=np.linalg.norm(centers-centers[first],axis=1)
    for _ in range(min(count,len(eligible))-1):
        index=int(eligible[np.argmax(distance[eligible])]);chosen.append(index)
        distance=np.minimum(distance,np.linalg.norm(centers-centers[index],axis=1));distance[chosen]=-1
    return centers[chosen]


class CameraAudit:
    def __init__(self,data,pad_samples=9,min_visible=2,min_clearance=.01):
        self.model=PoseModel(data);self.meshes=[];self.pads={};self.min_visible=min_visible;self.min_clearance=min_clearance
        for index,mesh in enumerate(data['visible_meshes']):
            tri=np.asarray(mesh['points'])[np.asarray(mesh['triangles'],int)];tree=TriangleTree(tri)
            self.meshes.append(dict(mesh=mesh,tree=tree))
            if mesh.get('robot') and 'fingertipsstep' in mesh['prim']:
                self.pads.setdefault(mesh['robot'],[]).append(dict(index=index,body=mesh['body'],samples=surface_samples(tri,pad_samples),center=np.asarray(mesh['points']).mean(0)))
        if any(len(self.pads.get(r,[]))!=2 for r in data['robots']):raise ValueError('two unique pad meshes required per robot')

    def audit(self,poses,robot,offset):
        root=self.model.data['robots'][robot]['root_path'];wrist=poses[root+'/wrist_3_link'];eye=wrist[:3,:3]@offset+wrist[:3,3]
        targets=[]
        for pad in self.pads[robot]:
            B=poses[pad['body']];targets.append(B[:3,:3]@pad['center']+B[:3,3])
        target=np.mean(targets,axis=0);forward=target-eye;forward/=np.linalg.norm(forward)
        right=np.cross(forward,wrist[:3,2]);right/=np.linalg.norm(right);up=np.cross(right,forward)
        local=[];eye_candidates=[];clearance=float('inf')
        for item in self.meshes:
            mesh,tree=item['mesh'],item['tree'];B=poses.get(mesh['body'],self.model.rest[mesh['body']]);p=(eye-B[:3,3])@B[:3,:3]
            local.append((p,B[:3,:3],B[:3,3]))
            delta=np.maximum(np.maximum(tree.low-p,p-tree.high),0.);box_distance=float(np.linalg.norm(delta))
            if box_distance<self.min_clearance:
                distance=tree.nearest_distance(p);clearance=min(clearance,distance)
                inside_box=bool(np.all(p>=tree.low)&np.all(p<=tree.high))
                winding=tree.winding(p) if inside_box else 0.
                eye_candidates.append(dict(mesh=mesh['prim'],distance=distance,winding=winding,AABB_inside=inside_box))
        pads=[]
        for pad in self.pads[robot]:
            B=poses[pad['body']];points=pad['samples']@B[:3,:3].T+B[:3,3];visible=0;blocked={};in_frame=0
            for point in points:
                relative=point-eye;depth=float(relative@forward)
                x=float(relative@right);y=float(relative@up)
                if depth<=.01 or abs(x/depth)>20.955/24 or abs(y/depth)>20.955*.75/24:
                    blocked['outside_frustum']=blocked.get('outside_frustum',0)+1;continue
                in_frame+=1;nearest=1.00001;hit_index=None
                for index,item in enumerate(self.meshes):
                    p,R,t=local[index];direction=(point-t)@R-p
                    hit=item['tree'].first_hit(p,direction,nearest)
                    if hit is not None and hit<nearest:nearest=hit;hit_index=index
                if hit_index==pad['index']:visible+=1
                else:
                    key=self.meshes[hit_index]['mesh']['prim'] if hit_index is not None else 'no_triangle_hit'
                    blocked[key]=blocked.get(key,0)+1
            center=B[:3,:3]@pad['center']+B[:3,3];relative=center-eye;depth=float(relative@forward)
            pads.append(dict(mesh=self.meshes[pad['index']]['mesh']['prim'],visible_samples=visible,total_samples=len(points),in_frustum=in_frame,blocked_by=blocked,
                projected_center_px=[320+float(relative@right)/depth*640*12/20.955,240-float(relative@up)/depth*640*12/20.955]))
        separation=float(np.linalg.norm(np.asarray(pads[0]['projected_center_px'])-pads[1]['projected_center_px']))
        eye_bad=any(x['distance']<self.min_clearance or abs(x['winding'])>.05 for x in eye_candidates)
        visible_ok=all(p['visible_samples']>=self.min_visible for p in pads)
        return dict(eye=eye.tolist(),eye_candidates=eye_candidates,eye_clearance_below_threshold=eye_bad,pads=pads,
            projected_pad_separation_px=separation,pinch_visibility_pass=visible_ok and separation>=12 and not eye_bad)


def read_poses(model,path,stride,max_samples,start_index=0,end_index=None):
    yielded=0
    for index,line in enumerate(Path(path).open()):
        if index<start_index:continue
        if end_index is not None and index>=end_index:return
        if index%stride:continue
        row=json.loads(line)
        if 'q' in row:
            poses=model.poses(row['q'])
            for name,B in row.get('part_poses',{}).items():poses[next(k for k in poses if k.endswith('/'+name))]=np.asarray(B,float)
            if 'payload_pose' in row:poses[next(k for k in poses if k.endswith('/fabrica_beam_6'))]=np.asarray(row['payload_pose'],float)
        else:poses={**model.rest,**{k:np.asarray(B,float) for k,B in row['body_poses'].items()}}
        yield index,row.get('phase',row.get('segment',row.get('step',row.get('task_step')))),poses
        yielded+=1
        if max_samples and yielded>=max_samples:return


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--geometry',required=True);parser.add_argument('--trajectory',action='append',default=[])
    parser.add_argument('--output',required=True);parser.add_argument('--stride',type=int,default=1);parser.add_argument('--max-samples',type=int,default=0)
    parser.add_argument('--start-index',type=int,default=0);parser.add_argument('--end-index',type=int)
    parser.add_argument('--closure-sweep',action='store_true');parser.add_argument('--pad-samples',type=int,default=9);parser.add_argument('--min-visible',type=int,default=2)
    parser.add_argument('--min-clearance',type=float,default=.01);parser.add_argument('--offsets-json');args=parser.parse_args()
    data=json.loads(Path(args.geometry).read_text());audit=CameraAudit(data,args.pad_samples,args.min_visible,args.min_clearance)
    offsets=json.loads(args.offsets_json) if args.offsets_json else {'old_closing_axis':{'franka_left':[0,-.16,.08],'franka_right':[0,.16,.08]},'raised_side':{'franka_left':[-.12,0,.12],'franka_right':[-.12,0,.12]},'oblique_side':{'franka_left':[-.12,-.12,.12],'franka_right':[-.12,.12,.12]}}
    stats={name:dict(samples=0,pinch_visibility_failures=0,eye_failures=0,first_failures=[],failure_phase_counts={},min_visible_samples=100000,min_projected_pad_separation_px=1e9,eye_AABB_candidates=[]) for name in offsets}
    sources=[]
    if args.closure_sweep:
        rows=[]
        for step,driver in enumerate(np.linspace(0,.8,17)):
            q={r:np.asarray(info['joint_positions'],float).copy() for r,info in data['robots'].items()}
            signs={'finger_joint':1,'right_outer_knuckle_joint':1,'left_inner_finger_joint':-1,'right_inner_finger_joint':1,'left_inner_finger_knuckle_joint':-1,'right_inner_finger_knuckle_joint':-1}
            for robot,info in data['robots'].items():
                names=info['joint_names'];change=driver-q[robot][names.index('finger_joint')]
                for name,sign in signs.items():q[robot][names.index(name)]+=sign*change
            rows.append((step,'closure_'+str(driver),audit.model.poses(q)))
        sources.append(('initial_arm_closure_sweep',rows))
    sources.extend((path,read_poses(audit.model,path,args.stride,args.max_samples,args.start_index,args.end_index)) for path in args.trajectory)
    if not sources:raise ValueError('supply trajectories or closure sweep')
    for source,rows in sources:
        for index,phase,poses in rows:
            for name,eyes in offsets.items():
                for robot,offset in eyes.items():
                    result=audit.audit(poses,robot,np.asarray(offset,float));s=stats[name];s['samples']+=1
                    s['pinch_visibility_failures']+=int(not result['pinch_visibility_pass']);s['eye_failures']+=int(result['eye_clearance_below_threshold'])
                    if not result['pinch_visibility_pass']:
                        key=Path(source).name+':'+str(phase)+':'+robot;s['failure_phase_counts'][key]=s['failure_phase_counts'].get(key,0)+1
                    s['min_visible_samples']=min(s['min_visible_samples'],*(p['visible_samples'] for p in result['pads']))
                    s['min_projected_pad_separation_px']=min(s['min_projected_pad_separation_px'],result['projected_pad_separation_px'])
                    if not result['pinch_visibility_pass'] and len(s['first_failures'])<12:s['first_failures'].append(dict(source=source,index=index,phase=phase,robot=robot,**result))
                    if result['eye_candidates'] and len(s['eye_AABB_candidates'])<100:s['eye_AABB_candidates'].append(dict(source=source,index=index,robot=robot,candidates=result['eye_candidates']))
            if index%100==0:print(json.dumps(dict(source=source,index=index,summary={k:dict(samples=v['samples'],failures=v['pinch_visibility_failures']) for k,v in stats.items()})),flush=True)
    report=dict(scope='first-hit visible pad surface samples against all exported visible triangles including complete grippers, payload, other arm and fixture; eye true triangle distances and generalized winding within AABB candidates; fixed camera clipping/frustum; no simulator',
        limitations=['Open visual meshes do not certify watertight solids; ambiguous eye winding is rejected.','Unexported warehouse geometry, rasterization/material/render freshness and between-sample motion not certified.','Pad mesh visibility is a proxy for readable pinch; native three-view images remain required.','Initial closure sweep alone does not cover actual transport or all five parts.'],
        geometry=str(Path(args.geometry).resolve()),visible_meshes=len(audit.meshes),triangles=sum(len(x['tree'].tri) for x in audit.meshes),joint_graph_gaps=audit.model.gaps,
        trajectories=args.trajectory,stride=args.stride,max_samples=args.max_samples,start_index=args.start_index,end_index=args.end_index,closure_sweep=args.closure_sweep,pad_samples=args.pad_samples,min_visible=args.min_visible,min_eye_clearance=args.min_clearance,offsets=offsets,results=stats)
    Path(args.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({name:{k:v for k,v in s.items() if k not in ('first_failures','eye_AABB_candidates')} for name,s in stats.items()}))


if __name__=='__main__':main()
