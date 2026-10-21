"""Visible-CAD pinch qualification for the explicitly assisted Beam route."""
from __future__ import annotations
import numpy as np
import fcl
from scipy.spatial.transform import Rotation
from .beam_assisted_ik import bounded_local_ik

def matrix(position,quat):
    w,x,y,z=quat;T=np.eye(4);T[:3,:3]=Rotation.from_quat([x,y,z,w]).as_matrix();T[:3,3]=position
    return T

class VisiblePinchGeometry:
    def __init__(self,data):
        self.data=data;self.rest={k:np.asarray(v,float) for k,v in data['body_rest'].items()};self.edges={};self.shapes={}
        for robot,info in data['robots'].items():
            indexes={n:i for i,n in enumerate(info['joint_names'])};entries=[]
            for j in data['joints']:
                if j['robot']!=robot or not j['enabled'] or len(j['parent'])!=1 or len(j['child'])!=1:continue
                parent,child=j['parent'][0],j['child'][0]
                if parent not in self.rest or child not in self.rest:raise ValueError('visual pinch lacks native joint body')
                anchor=matrix(j['pos0'],[j['rot0'][3],*j['rot0'][:3]])
                index=indexes.get(j['path'].rsplit('/',1)[-1])
                if j['type']=='PhysicsRevoluteJoint' and index is None:raise ValueError('visual pinch lacks native joint name')
                entries.append((parent,child,index,j['axis'],anchor,np.linalg.inv(anchor),np.linalg.inv(self.rest[parent])@self.rest[child]))
            self.edges[robot]=entries
        for mesh in data['visible_meshes']:
            if not (str(mesh['object']).startswith('fabrica_beam_') or mesh['body'].endswith(('/left_inner_finger','/right_inner_finger'))):continue
            points=np.asarray(mesh['points'],float);faces=np.asarray(mesh['triangles'],np.int32)
            shape=fcl.BVHModel();shape.beginModel(len(faces),len(points));shape.addSubModel(points,faces);shape.endModel()
            self.shapes.setdefault(mesh['body'],[]).append((mesh,fcl.CollisionObject(shape)))

    def poses(self,robot,q):
        result=dict(self.rest);initial=np.asarray(self.data['robots'][robot]['joint_positions'])
        entries=self.edges[robot];done=set(result)-{j[1] for j in entries};pending=list(entries)
        while pending:
            progressed=False
            for entry in pending[:]:
                parent,child,index,axis,A,Ai,relative=entry
                if parent not in done:continue
                delta=np.eye(4)
                if index is not None:
                    v=np.eye(3)['XYZ'.index(axis)]*(q[index]-initial[index]);delta[:3,:3]=Rotation.from_rotvec(v).as_matrix()
                result[child]=result[parent]@A@delta@Ai@relative;done.add(child);pending.remove(entry);progressed=True
            if not progressed:raise ValueError('unresolved visual joint graph')
        return result

    def gaps(self,robot,part,poses):
        root=self.data['robots'][robot]['root_path'];part_body=next(k for k in self.shapes if k.endswith('/'+part))
        values=[]
        for finger in ('left_inner_finger','right_inner_finger'):
            body=root+'/Gripper/Robotiq_2F_85/'+finger;minimum=np.inf;intersect=False
            for mesh,obj in self.shapes[body]+self.shapes[part_body]:
                T=poses[mesh['body']];obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
            for _,a in self.shapes[body]:
                for _,b in self.shapes[part_body]:
                    r=fcl.CollisionResult();hit=fcl.collide(a,b,fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),r)
                    if hit:
                        intersect=True;distance=-max((float(c.penetration_depth) for c in r.contacts),default=0.)
                    else:distance=float(fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()))
                    minimum=min(minimum,distance)
            values.append({'finger':finger,'gap':minimum,'intersect':intersect})
        return values

    def goal(self,robot,part,q,part_pose,clearance=.00023):
        info=self.data['robots'][robot];names=info['joint_names'];driver=names.index('finger_joint')
        signs={'finger_joint':1,'right_outer_knuckle_joint':1,'left_inner_finger_joint':-1,'right_inner_finger_joint':1,'left_inner_finger_knuckle_joint':-1,'right_inner_finger_knuckle_joint':-1}
        part_body=next(k for k in self.shapes if k.endswith('/'+part))
        def evaluate(value):
            trial=np.asarray(q,float).copy();change=value-q[driver]
            for name,sign in signs.items():trial[names.index(name)]+=sign*change
            poses=self.poses(robot,trial);poses[part_body]=part_pose
            return self.gaps(robot,part,poses)
        low=0.;g=evaluate(low)
        if any(x['intersect'] or x['gap']<clearance for x in g):raise ValueError('open visible fingers intersect the grasp target')
        high=None
        for value in np.linspace(.025,.8,32):
            g=evaluate(value)
            if any(x['intersect'] or x['gap']<clearance for x in g):high=float(value);break
            low=float(value)
        if high is None:raise ValueError('target does not fit inside a visible pinch')
        for _ in range(20):
            middle=(low+high)/2;g=evaluate(middle)
            if any(x['intersect'] or x['gap']<clearance for x in g):high=middle
            else:low=middle
        return low,evaluate(low)

    def center_shift(self,robot,part,q,part_pose,goal):
        names=self.data['robots'][robot]['joint_names'];trial=np.asarray(q,float).copy()
        change=goal-q[names.index('finger_joint')]
        for name,sign in {'finger_joint':1,'right_outer_knuckle_joint':1,'left_inner_finger_joint':-1,'right_inner_finger_joint':1,'left_inner_finger_knuckle_joint':-1,'right_inner_finger_knuckle_joint':-1}.items():trial[names.index(name)]+=sign*change
        poses=self.poses(robot,trial);body=next(k for k in self.shapes if k.endswith('/'+part));poses[body]=part_pose
        root=self.data['robots'][robot]['root_path']+'/Gripper/Robotiq_2F_85/'
        fingers=[root+'left_inner_finger',root+'right_inner_finger']
        axis=poses[fingers[0]][:3,3]-poses[fingers[1]][:3,3];axis/=np.linalg.norm(axis)
        def imbalance(distance):
            shifted=dict(poses)
            for finger in fingers:
                shifted[finger]=poses[finger].copy();shifted[finger][:3,3]+=axis*distance
            g=self.gaps(robot,part,shifted);return g[0]['gap']-g[1]['gap']
        epsilon=.00002;gradient=(imbalance(epsilon)-imbalance(-epsilon))/(2*epsilon)
        if not np.isfinite(gradient) or abs(gradient)<.2:raise ValueError('visible pinch centering has no reliable surface gradient')
        return axis*float(np.clip(-imbalance(0.)/gradient,-.00025,.00025))

def geometry(task):
    if not hasattr(task,'_assisted_visible_pinch'):
        data=getattr(task,'_assisted_geometry_snapshot',None)
        if data is None:raise RuntimeError('assisted route requires its native geometry initialization')
        task._assisted_visible_pinch=VisiblePinchGeometry(data)
    return task._assisted_visible_pinch

def native_gaps(task,robot,part):
    model=geometry(task);root=model.data['robots'][robot]['root_path'];poses={}
    p,Q=task._resolve_object(part).get_pose();body=next(k for k in model.shapes if k.endswith('/'+part));poses[body]=matrix(p,Q)
    for finger in ('left_inner_finger','right_inner_finger'):
        path=root+'/Gripper/Robotiq_2F_85/'+finger
        reader=getattr(task,'_assisted_native_body_readers',{}).get(path)
        if reader is None:reader=(getattr(task.robots[robot],'_rigid_body_map',{}) or {}).get(path)
        if reader is None or str(reader.unwrap().prim_path)!=path:raise RuntimeError('missing full-path visible finger native reader')
        p,Q=reader.get_pose();poses[path]=matrix(p,Q)
    return model.gaps(robot,part,poses),poses

def close_action(task,adapter,robot,part,action):
    model=geometry(task);key=(task.phase_index,task.phase_entry_step,robot,part)
    states=getattr(task,'_assisted_visual_close_states',None)
    if states is None:states={};task._assisted_visual_close_states=states
    q=np.asarray(task.robots[robot].articulation.get_joint_positions(),float)
    driver=model.data['robots'][robot]['joint_names'].index('finger_joint')
    state=states.get(key)
    if state is None:
        anchor=np.asarray(action['arm_joint_controller'][0],float).copy()
        state={'arm_anchor':anchor,'stable':0,'centering_count':0,'last_plan':-1000,'hold_openness':1-q[driver]/.8,'ready':False};states[key]=state
    actual,poses=native_gaps(task,robot,part)
    if any(x['intersect'] for x in actual):
        raise RuntimeError('actual visible finger/part surfaces intersect')
    part_body=next(k for k in model.shapes if k.endswith('/'+part))
    if 'goal_q' not in state or (task.step_counter-state['last_plan']>=48):
        goal,predicted=model.goal(robot,part,q,poses[part_body]);state.update(goal_q=goal,predicted=predicted,last_plan=task.step_counter)
        task._assisted_event('visual_pinch_plan',robot=robot,object=part,goal_q=goal,predicted=predicted)
        if max(x['gap'] for x in predicted)>.00025:
            if state['centering_count']>=12:raise RuntimeError('visual pinch could not center both finger faces')
            shift=model.center_shift(robot,part,q,poses[part_body],goal)
            control=task.robots[robot].controllers['arm_ik_controller'];base=control.get_ik_base_world_pose();wrapper=control._kinematics_solver
            wrapper.set_robot_base_pose(robot_position=base[0]/control._robot_scale,robot_orientation=base[1])
            raw=wrapper.get_kinematics_solver();frame=wrapper.get_end_effector_frame();forward=lambda x:raw.compute_forward_kinematics(frame,x)
            p,R=forward(state['arm_anchor']);quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
            new,detail=bounded_local_ik(forward,state['arm_anchor'],p+shift,quat)
            if new is None:raise RuntimeError('visual pinch centering inverse failed')
            state['arm_anchor']=new;state['centering_count']+=1;state['goal_q']=min(state['goal_q'],q[driver]);state['last_plan']=task.step_counter-24
            task._assisted_event('visual_pinch_center',robot=robot,object=part,shift=shift,detail=detail)
    goal=float(state['goal_q'])
    previous=float(state.get('driver_command',q[driver]))
    gap=min(x['gap'] for x in actual)
    # Close from measured surfaces; a settled driver bias must not push
    # through the visible clearance while chasing an ideal drive target.
    increment=min(goal-previous,(gap-.00023)/.15)
    requested=previous+float(np.clip(increment,-.001,.001))
    lead=float(np.clip((gap-.00004)/.15,0.,.01))
    command=float(np.clip(requested,q[driver]-.01,q[driver]+lead))
    command=float(np.clip(command,0,min(.8,goal)))
    state['driver_command']=command;openness=1-command/.8
    state['hold_openness']=openness
    ready=(all(not x['intersect'] and .000005<=x['gap']<=.00025 for x in actual)
           and abs(q[driver]-state.get('previous_driver_q',q[driver]))<.00005)
    stamp=task._beam_physics_stamp()
    if not stamp.get('valid'):raise RuntimeError('visual pinch requires the native physics clock')
    clock=(stamp.get('epoch'),stamp.get('index'))
    if state.get('clock')!=clock:
        state['stable']=state['stable']+1 if ready else 0;state['clock']=clock
        state['previous_driver_q']=float(q[driver])
    state['ready']=state['stable']>=12
    if task.step_counter%8==0:task._assisted_event('visual_pinch_observation',robot=robot,object=part,gaps=actual,actual_driver_q=q[driver],command_driver_q=command,max_driver_lead_rad=lead,stable=state['stable'],ready=state['ready'])
    return {'arm_joint_controller':[state['arm_anchor'].tolist()],'gripper_controller':[openness]},state['ready'],{'qualification':'dual_visible_CAD_surfaces','gaps':actual,'stable_physics_steps':state['stable']}

def ready(task,robot,part):
    key=(task.phase_index,task.phase_entry_step,robot,part);state=getattr(task,'_assisted_visual_close_states',{}).get(key,{})
    if not state.get('ready'):return False
    gaps,_=native_gaps(task,robot,part)
    return all(not x['intersect'] and .000005<=x['gap']<=.00025 for x in gaps)

def hold_command(task,robot,part):
    key=(task.phase_index,task.phase_entry_step,robot,part)
    return getattr(task,'_assisted_visual_close_states',{}).get(key,{}).get('hold_openness')
