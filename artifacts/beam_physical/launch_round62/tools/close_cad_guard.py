"""Read-only full-mesh screening of final commands in opted-in close stages.

This guard qualifies geometry only. Contact force, grasp/slide qualification,
stage completion and all physical success checks remain with the task.
"""
import itertools
import copy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np


class CloseCADGuard:
    def __init__(self, geometry, geometry_tools, out):
        sys.path.insert(0,str(geometry_tools))
        from joint_cad_guard import JointCADGuard
        self.geometry=JointCADGuard(geometry,geometry_tools,out)
        self.model=self.geometry.model
        self.out=Path(out);self.reader=None;self.task_identity=None
        self.mimic_cache={};self.calls=0;self.samples=0;self.seconds=0.
        self.by_phase={};self.first_inputs={};self.last=None;self.last_dump=0.
        self.mesh_by_prim={m['prim']:m for m in self.model.data['visible_meshes']}
        self.prev_goal={};self.prev_task_identity=None;self.payload_corners={}

    @staticmethod
    def _finite_arm_goal(values):
        try:
            values=np.asarray(values,float)
            if values.shape==(6,) and np.all(np.isfinite(values)):return values.copy()
        except (TypeError,ValueError):pass
        return None

    def _observe_goals(self,task,actions):
        if self.prev_task_identity!=id(task):
            self.prev_goal.clear();self.prev_task_identity=id(task)
        previous={k:None if v is None else v.copy() for k,v in self.prev_goal.items()}
        for robot,action in actions.items():
            action=action or {}
            if 'arm_joint_controller' in action:
                try:goal=self._finite_arm_goal(action['arm_joint_controller'][0])
                except (TypeError,IndexError):goal=None
                if goal is not None:self.prev_goal[robot]=goal
            if 'arm_ik_controller' in action:self.prev_goal[robot]=None
        return previous

    def _references(self,task,previous,q):
        references={};details={}
        for robot in q:
            cached=self._finite_arm_goal((previous or {}).get(robot))
            native=None;error=None
            try:
                applied=task.robots[robot].articulation.get_applied_action()
                native=self._finite_arm_goal(np.asarray(applied.joint_positions,float)[:6])
            except Exception as exc:error=repr(exc)[:200]
            references[robot]=cached if cached is not None else native
            details[robot]={'reference_source':'cache' if cached is not None else 'native' if native is not None else 'none',
                'cache_native_difference_rad':float(np.max(abs(cached-native))) if cached is not None and native is not None else None,
                'native_read_error':error}
        return references,details

    def _payload_corner_displacement(self,body,native,predicted):
        if body not in self.payload_corners:
            points=np.concatenate([np.asarray(m['points'],float) for m in self.model.data['visible_meshes'] if m['body']==body])
            self.payload_corners[body]=np.asarray(list(itertools.product(*zip(points.min(axis=0),points.max(axis=0)))))
        corners=self.payload_corners[body]
        start=corners@native[:3,:3].T+native[:3,3]
        end=corners@predicted[:3,:3].T+predicted[:3,3]
        return float(np.max(np.linalg.norm(end-start,axis=1)))

    def specs(self,task):
        from toolkits.factory_dual_franka_assembly.local_skills import normalize_local_skill_spec
        phase=task.get_current_phase_spec()
        return {robot:spec for robot in self.model.data['robots']
            if (spec:=normalize_local_skill_spec(phase,robot))
            and spec.get('name')=='ur5e_close_gripper'
            and spec.get('close_recenter_feedback_command_space') is True}

    def _entity(self,mesh):
        if not mesh['robot']:return mesh['object']
        root=self.model.data['robots'][mesh['robot']]['root_path']+'/'
        if not mesh['body'].startswith(root):raise ValueError('CAD robot/body identity mismatch')
        return mesh['robot']+'/'+mesh['body'][len(root):]

    @staticmethod
    def mimic_coefficients(*,stage,joint_records,joint_names,active_index):
        """Read the authored PhysX mimic gearing; never infer it from q ratios."""
        by_name={j['path'].rsplit('/',1)[-1]:j for j in joint_records if j['index'] is not None}
        active_name=joint_names[active_index]
        coefficients={active_name:1.}
        visiting=set()
        def coefficient(name):
            if name in coefficients:return coefficients[name]
            if name in visiting:raise ValueError('Cyclic mimic graph')
            visiting.add(name)
            record=by_name[name];prim=stage.GetPrimAtPath(record['path'])
            if not prim or not prim.IsValid():raise ValueError('Missing authored mimic joint '+name)
            relations=[rel for rel in prim.GetRelationships()
                if str(rel.GetName()).startswith('physxMimicJoint:')
                and str(rel.GetName()).endswith(':referenceJoint')]
            if len(relations)!=1:raise ValueError('Expected one authored mimic reference for '+name)
            rel=relations[0];targets=rel.GetTargets()
            if len(targets)!=1 or targets[0].GetParentPath()!=prim.GetPath().GetParentPath():
                raise ValueError('Mimic reference escapes the actual gripper joint graph')
            ref=targets[0].name
            if ref not in by_name:raise ValueError('Mimic reference missing from actual DOFs')
            gear=prim.GetAttribute(str(rel.GetName()).removesuffix('referenceJoint')+'gearing').Get()
            if gear is None or not np.isfinite(float(gear)):raise ValueError('Invalid authored mimic gearing')
            value=-float(gear)*coefficient(ref)
            visiting.remove(name);coefficients[name]=value
            return value
        result=np.zeros(len(joint_names))
        for index in range(6,len(joint_names)):result[index]=coefficient(joint_names[index])
        return result

    def _targets(self,task,actions,q,references,arm_details):
        end={robot:values.copy() for robot,values in q.items()}
        mapping={}
        for robot in q:
            action=actions.get(robot) or {}
            arm_details[robot].update(mode='no_arm_command',max_joint_increment_rad=0.)
            if 'arm_ik_controller' in action:raise ValueError('Close CAD requires final joint targets for both arms')
            if 'arm_joint_controller' in action:
                goal=np.asarray(action['arm_joint_controller'][0],float)
                if goal.shape!=(6,) or not np.all(np.isfinite(goal)):raise ValueError('Invalid final arm command')
                if np.max(np.abs(goal-q[robot][:6]))>.18+1e-12:raise ValueError('Final arm command outside existing tracking domain')
                ref=references[robot]
                increment=goal-q[robot][:6] if ref is None else goal-ref
                end[robot][:6]=q[robot][:6]+increment
                arm_details[robot].update(mode='absolute_fallback' if ref is None else 'increment',
                    max_joint_increment_rad=float(np.max(abs(increment))))
            if 'gripper_controller' not in action:continue
            controller=task.robots[robot].controllers['gripper_controller']
            value=controller._normalize_action(action['gripper_controller'][0])
            openness=1. if value=='open' else 0. if value=='close' else float(value)
            minimum=float(getattr(controller._robot_config,'gripper_close_openness',0.) or 0.)
            openness=float(np.clip(openness,max(minimum,0.),1.))
            gripper=controller._gripper
            indices=np.asarray(gripper.active_joint_indices,int).reshape(-1)
            opened=np.asarray(gripper.joint_opened_positions,float).reshape(-1)
            closed=np.asarray(gripper.joint_closed_positions,float).reshape(-1)
            if indices.shape!=(1,) or opened.shape!=(1,) or closed.shape!=(1,) or indices[0]<6:
                raise ValueError('Unexpected active gripper drive mapping')
            names=list(task.robots[robot].articulation.dof_names)
            if names!=self.model.data['robots'][robot]['joint_names']:raise ValueError('Articulation/CAD DOF names differ')
            key=(id(self.reader.stage),robot,tuple(names),int(indices[0]))
            if key not in self.mimic_cache:
                self.mimic_cache[key]=self.mimic_coefficients(stage=self.reader.stage,
                    joint_records=self.model.joints[robot],joint_names=names,active_index=int(indices[0]))
            target=float(closed[0]+openness*(opened[0]-closed[0]))
            end[robot]+=self.mimic_cache[key]*(target-q[robot][indices[0]])
            mapping[robot]={'active_index':int(indices[0]),'active_target':target,
                'openness':openness,'mimic_coefficients':self.mimic_cache[key].tolist()}
        if any(not np.all(np.isfinite(v)) for v in end.values()):raise ValueError('Nonfinite final articulation target')
        return end,mapping

    def check(self,task,actions,previous=None):
        if not self.specs(task):return True,None
        start=time.perf_counter();self.calls+=1;phase=task.phase
        self.by_phase[phase]=self.by_phase.get(phase,0)+1
        trace={'physics_index':None,'task_step':getattr(task,'step_counter',None),'arms':{},
            'holders':[],'payload_displacements_m':{},'max_skipped_payload_environment_depth_m':0.,'passed':False}
        try:
            from toolkits.constraint_checking.integration.entity_geometry import TaskEntityGeometryMonitor
            from toolkits.constraint_checking.integration.contact_policy import AssemblyContactPolicy
            stamp=task._beam_physics_stamp()
            trace['physics_index']=stamp.get('index')
            if (stamp.get('valid') is not True or type(stamp.get('index')) is not int
                    or type(stamp.get('epoch')) is not int):raise ValueError('Missing native clock')
            if self.reader is None or self.task_identity!=id(task):
                self.reader=TaskEntityGeometryMonitor(task);self.task_identity=id(task);self.mimic_cache.clear()
            actual={k:np.asarray(v,float) for k,v in self.reader.current_poses().items()}
            robot_bodies={m['body'] for m in self.model.data['visible_meshes'] if m['robot']}
            if not robot_bodies.issubset(actual):raise ValueError('Missing complete native robot/gripper body poses')
            if any(v.shape!=(4,4) or not np.all(np.isfinite(v)) for v in actual.values()):raise ValueError('Invalid native body pose')
            q={robot:np.asarray(task.robots[robot].articulation.get_joint_positions(),float).reshape(-1)
                for robot in self.model.data['robots']}
            if any(v.shape!=(12,) or not np.all(np.isfinite(v)) for v in q.values()):raise ValueError('Invalid full measured articulation')
            references,arm_details=self._references(task,previous,q)
            trace['arms']=arm_details
            end,mapping=self._targets(task,actions,q,references,arm_details)
            origin=self.model.poses(q)
            end_fk=self.model.poses(end)
            reference_q={robot:values.copy() for robot,values in q.items()}
            for robot,ref in references.items():
                if ref is not None:reference_q[robot][:6]=ref
            reference_fk=self.model.poses(reference_q)
            for robot in q:
                tcp=self.model.data['robots'][robot]['root_path']+'/Gripper/Robotiq_2F_85/base_link'
                arm_details[robot].update(tcp_increment_m=float(np.linalg.norm(end_fk[tcp][:3,3]-origin[tcp][:3,3])),
                    standing_lead_m=float(np.linalg.norm(reference_fk[tcp][:3,3]-origin[tcp][:3,3])) if references[robot] is not None else None)
            corrections={body:actual[body]@np.linalg.inv(origin[body]) for body in robot_bodies}
            native={**self.model.rest,**actual}
            policy=AssemblyContactPolicy();snapshot=policy.snapshot_from_task(task)
            trace['holders']=list(snapshot.holders)
            holders={}
            for obj,robot in snapshot.holders:holders.setdefault(obj,[]).append(robot)
            payloads={}
            for obj,robots in holders.items():
                paths={m['body'] for m in self.model.data['visible_meshes'] if m.get('object')==obj}
                if len(paths)!=1 or not robots:raise ValueError('Ambiguous held payload prediction')
                body=next(iter(paths))
                if body not in actual or any(robot not in q for robot in robots):raise ValueError('Missing native held payload')
                payloads[body]=[self.model.data['robots'][robot]['root_path']+'/Gripper/Robotiq_2F_85/base_link' for robot in sorted(set(robots))]
            relevant=robot_bodies|set(payloads)
            context={'physics_stamp':stamp,'actual_q':{k:v.tolist() for k,v in q.items()},
                'native_body_poses':{k:v.tolist() for k,v in actual.items()},'actions':copy.deepcopy(actions),
                'gripper_mapping':mapping,'holders':list(snapshot.holders),
                'arm_prediction':copy.deepcopy(arm_details)}
            self.first_inputs.setdefault(phase,context)
            # Both complete arm paths plus independent jaw endpoints, including
            # jaw-only movement with either arm stationary. Finite sampling;
            # this is not a CCD or force/dynamics certificate.
            arm_states=[(0.,0.),(.5,.5),(1.,1.),(0.,1.),(1.,0.)]
            checked=0
            for arms in arm_states:
                for jaws in itertools.product((0.,1.),repeat=2):
                    sample={}
                    for i,robot in enumerate(q):
                        sample[robot]=q[robot].copy()
                        sample[robot][:6]+=arms[i]*(end[robot][:6]-q[robot][:6])
                        sample[robot][6:]+=jaws[i]*(end[robot][6:]-q[robot][6:])
                    fk=self.model.poses(sample);poses=dict(native)
                    for body,correction in corrections.items():poses[body]=correction@fk[body]
                    # During a two-hand handover, screen each holder's predicted
                    # payload pose; choosing one silently would omit a domain.
                    bodies=list(payloads)
                    for tcps in itertools.product(*(payloads[body] for body in bodies)):
                        variant=dict(poses)
                        for body,tcp in zip(bodies,tcps):
                            variant[body]=poses[tcp]@np.linalg.inv(native[tcp])@native[body]
                            displacement=self._payload_corner_displacement(body,native[body],variant[body])
                            label=body.rsplit('/',1)[-1]+'|'+next(robot for robot in q if tcp.startswith(self.model.data['robots'][robot]['root_path']+'/'))
                            trace['payload_displacements_m'][label]=max(trace['payload_displacements_m'].get(label,0.),displacement)
                            if displacement>100e-6:
                                self.last={'passed':False,'phase':phase,'physics_stamp':stamp,
                                    'reason':'close_payload_command_increment_exceeds_100um','payload_body':body,
                                    'payload_displacement_m':displacement,'payload_holder':tcp,
                                    'arm_fractions':arms,'jaw_fractions':jaws,'input':context,'diagnostics':trace}
                                return False,self.last
                        self.geometry.scene_check(variant,phase,stamp['index']);self.samples+=1;checked+=1
                        for row in self.geometry.query.checker.rows:
                            if ((row['body_a'] in payloads and row['body_b'] not in robot_bodies)
                                    or (row['body_b'] in payloads and row['body_a'] not in robot_bodies)):
                                trace['max_skipped_payload_environment_depth_m']=max(trace['max_skipped_payload_environment_depth_m'],row['depth'])
                                continue
                            if row['depth']<=1e-6 or not relevant.intersection((row['body_a'],row['body_b'])):continue
                            event=NS(entity_a=self._entity(self.mesh_by_prim[row['mesh_a']]),
                                entity_b=self._entity(self.mesh_by_prim[row['mesh_b']]),distance=-row['depth'])
                            decision=policy.classify(event,task,snapshot=snapshot)
                            if decision.classification=='allowed_contact':continue
                            self.last={'passed':False,'phase':phase,'physics_stamp':stamp,'collision_candidate':row,
                                'arm_fractions':arms,'jaw_fractions':jaws,'payload_holders':list(tcps),'input':context}
                            self.last['diagnostics']=trace
                            return False,self.last
            trace['passed']=True
            self.last={'passed':True,'phase':phase,'physics_stamp':stamp,'meshes':52,'checked_samples':checked,'diagnostics':trace}
            return True,self.last
        except Exception as exc:
            self.last={'passed':False,'phase':phase,'reason':'close_CAD_probe_or_contract_error','error':repr(exc)}
            return False,self.last
        finally:
            self.seconds+=time.perf_counter()-start
            with (self.out/'close_cad_trace.jsonl').open('a') as stream:stream.write(json.dumps(trace)+'\n')
            if not self.last or not self.last.get('passed') or time.monotonic()-self.last_dump>60:self.flush()

    def flush(self,final=False):
        record={'final':final,'calls':self.calls,'samples':self.samples,'wall_seconds':self.seconds,
            'by_phase':self.by_phase,'first_inputs':self.first_inputs,'last':self.last,'meshes':52,
            'scope':'Changed close stages only; cached prior command (native applied fallback) predicts arm increments; absolute jaw/mimic targets unchanged. Full52mesh both-arm/held-payload/environment checks remain computed. Payload corner displacement above100um stops; at or below100um payload/nonrobot overlap is recorded and delegated to unchanged force/slip/task gates. Robot-body rows retain1um threshold and exact existing contact policy. Standing command lead is diagnostic only. Finite20samples per holder, not CCD/cooked-physics/force proof.'}
        path=self.out/'close_cad_guard.json';temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(record,indent=2)+'\n');temp.replace(path);self.last_dump=time.monotonic()

    def install(self,policy_class):
        original=policy_class.act
        def act(policy,task,*args,**kwargs):
            actions=original(policy,task,*args,**kwargs)
            previous=self._observe_goals(task,actions)
            if getattr(task,'failed',False) or getattr(task,'success',False):return actions
            passed,detail=self.check(task,actions,previous=previous)
            if passed:return actions
            task._set_terminal_state('failed',reason='close-final-command-CAD-screen',status='failed',detail=detail)
            return {robot:{} for robot in task.config.robot_names}
        policy_class.act=act
