"""Finite counterfactual replay of the loaded controller handoff, without physics.

Unchanged computations are reused only after whole-module AST projection.
The new path is checked with production final-q code and all54 visible meshes.
"""
import ast, copy, hashlib, json, os, re, sys, tarfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
A=P/'assisted_repository'; T=P/'tools'
sys.path[:0]=[str(A),str(T),str(P.parent/'runtime_tools/geometry_deps')]
from beam_workflow import sha, save
from sweep_model import SceneModel,pose_matrix
from visual_mesh_check import VisualMeshChecker
from visual_support_check import missing_dynamic_poses,check_visual_support
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
from toolkits.factory_dual_franka_assembly.beam_assisted_servo import anchored_object_prediction,object_pose_from_tcp

SK='toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
ST='toolkits/factory_dual_franka_assembly/beam_assisted_state.py'
CO='roboassemblybench/core/beam_assisted_compiler.py'
TE='tests/toolkits/test_beam_assisted_object_servo.py'
def require(value,message):
    if not value:raise ValueError(message)
def dump(node):return ast.dump(node,include_attributes=False)
def methods(tree,name):return next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
def restore(source,kind):
    tree=ast.parse(source)
    if kind==SK:
        class Remove(ast.NodeTransformer):
            def visit_FunctionDef(self,n):
                if n.name=='_assisted_phase_handoff':return None
                return self.generic_visit(n)
        Remove().visit(tree)
        f=methods(tree,'_assisted_absolute_object_servo_pose')
        f.body=[n for n in f.body if not(isinstance(n,ast.Assign) and any(isinstance(x,ast.Tuple) and any(isinstance(z,ast.Name) and z.id=='position_step' for z in x.elts) for x in n.targets))]
        call=next(n for n in ast.walk(f) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='absolute_object_servo')
        call.args[4:6]=[ast.parse(s,mode='eval').body for s in ("float(spec.get('cartesian_position_step', .001))","float(spec.get('cartesian_orientation_step', .004))")]
        f=methods(tree,'_assisted_verify_published_held_command')
        f.body=[n for n in f.body if not(isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='joint_cap' for t in n.targets)) and not(isinstance(n,ast.If) and 'beam_assisted_phase_handoff_ticks' in ast.unparse(n.test))]
        class Cap(ast.NodeTransformer):
            def visit_Name(self,n):return ast.Constant(.02) if n.id=='joint_cap' else n
        Cap().visit(f)
    elif kind==CO:
        class Remove(ast.NodeTransformer):
            def visit_If(self,n):
                if any(isinstance(x,ast.Constant) and x.value=='beam_assisted_phase_handoff_ticks' for x in ast.walk(n)) and len(n.body)==1 and isinstance(n.body[0],ast.Assign) and isinstance(n.body[0].targets[0],ast.Subscript) and ast.unparse(n.body[0].targets[0]).endswith("['beam_assisted_phase_handoff_ticks']"):return None
                return self.generic_visit(n)
        Remove().visit(tree)
    elif kind==ST:
        f=methods(tree,'_assisted_joint')
        f.body=[n for n in f.body if not(isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='local_rotation' for t in n.targets)) and not(isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and any(isinstance(x,ast.Constant) and x.value=='joint_anchor_readback' for x in n.value.args))]
        f=methods(tree,'_assisted_guard_actions')
        loop=next(n for n in ast.walk(f) if isinstance(n,ast.For) and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='held' for t in x.targets) for x in n.body))
        held=next(n for n in loop.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='held' for t in n.targets));loop.body.remove(held)
        idx=next(i for i,n in enumerate(loop.body) if isinstance(n,ast.Assign) and '_ur5e_plumbers_last_arm_command_q' in ast.unparse(n))
        loop.body.insert(idx+1,held)
        branch=next(n for n in loop.body if isinstance(n,ast.If) and 'held is not None or' in ast.unparse(n.test))
        branch.test=branch.test.values[1]
    return tree

def projection():
    with tarfile.open(P/'versions/assisted_v19_before_A16_beam_full.tar.gz') as t:
        old={m.name:t.extractfile(m).read() for m in t.getmembers() if m.isfile()}
    after=json.loads((P/'runs/A16_beam_full/source_after.json').read_text())
    require(not after['differences'] and not after['tool_differences'],'old source not frozen')
    require({k:hashlib.sha256(v).hexdigest() for k,v in old.items()}=={r['path']:r['sha256'] for r in after['files']},'old archive not source-after889')
    current=json.loads((T/'assisted_manifest.json').read_text())
    require(len(current)==889 and all(sha(A/k)==h for k,h in current.items()),'current source889 stale')
    changed=sorted(k for k,h in current.items() if h!=hashlib.sha256(old[k]).hexdigest())
    require(changed==sorted([SK,ST,CO,TE]),'source changes exceed reviewed four files')
    controls={}
    for kind in (SK,ST,CO):
        new=(A/kind).read_text(); expected=dump(ast.parse(old[kind]))
        require(dump(restore(new,kind))==expected,'AST projection differs: '+kind)
        # Alter an unchanged numeric/control expression: the projection must reject.
        needle={SK:".0015)",ST:"bounded_native_target(target[0], actual, previous.get(name))",CO:".0015"}[kind]
        changed_text=new.replace(needle,needle.replace('.0015','.0016').replace('target[0], actual,','target[0], actual+.001,'),1)
        require(changed_text!=new and dump(restore(changed_text,kind))!=expected,'projection negative control failed: '+kind)
        controls[kind]=True
    oldgate=json.loads((P/'launch_gates/A16_beam_full.json').read_text()); evidence={}
    for row in oldgate['checks']:
        for e in row['evidence']:
            require(sha(e['path'])==e['sha256'],'reused old evidence changed: '+e['path']);evidence[e['path']]=e['sha256']
    cfg=json.loads((P/'D01_actual_configuration_layout505_config.json').read_text())
    oldcfg=json.loads((P/'C04_actual_configuration_preflight_v19_layout505_config.json').read_text())
    stripped=copy.deepcopy(cfg);flags=[]
    def strip(value,path=''):
        if isinstance(value,dict):
            if 'beam_assisted_phase_handoff_ticks' in value:
                require(value.pop('beam_assisted_phase_handoff_ticks')==24,'flag duration changed');flags.append(path)
            for k,v in value.items():strip(v,path+'/'+k)
        elif isinstance(value,list):
            for i,v in enumerate(value):strip(v,path+'/'+str(i))
    strip(stripped);require(len(flags)==4,'not exactly four config flags')
    # Both fingerprints are recomputed from the actual expanded recipe payload.
    from roboassemblybench.core.task_registry import load_task_recipe,task_recipe_fingerprint
    payload=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    require(task_recipe_fingerprint(payload)==cfg['recipe_fingerprint'],'new recipe digest not actual')
    strip(payload);require(task_recipe_fingerprint(payload)==oldcfg['recipe_fingerprint'],'stripped recipe differs from original')
    stripped['recipe_fingerprint']=oldcfg['recipe_fingerprint']
    require(stripped==oldcfg,'config changes exceed four flags plus derived fingerprint')
    return dict(passed=True,source_files=889,changed_paths=changed,whole_AST_projection=True,negative_controls=controls,config_flag_count=4,
        current_source_sha256=current,tools_manifest_sha256=sha(T/'assisted_tools_manifest.json'),old_evidence_sha256=evidence,
        reused_scope={'cached_cases':600,'cached_mesh_states':3775,'original_cases':5943,'nominal_states':6306},
        limits=['Reused calculations cover unchanged branches and unchanged geometry inputs. New pickup mode transition is separately replayed below.'])

def pose(t):
    return dict(position=t[:3,3].copy(),orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]])
def replay():
    cfg=json.loads((P/'D01_actual_configuration_layout505_config.json').read_text())
    specs={v['name']:v for v in cfg['phase_specs']};model=SceneModel(P/'runs/A16_beam_full/scene_geometry.json')
    checker=VisualMeshChecker(model,include_self=True)
    rows=[json.loads(l) for l in (P/'C01_bimanual_repaired_v2_path.jsonl').open()]
    body={m['object']:m['body'] for m in model.data['visible_meshes'] if str(m['object']).startswith('fabrica_beam_')}
    dec=json.loads((P/'C04_A16_held_residual_decomposition.json').read_text())['runs'][1]
    native=next(v for v in dec['rows'] if v['step']==10632 and v['boundary']=='before_action')
    entities=next(json.loads(l) for l in (P/'runs/A16_beam_full/entity_geometry_trace.jsonl').open() if json.loads(l)['step']==10632)
    boundary=next(json.loads(l) for l in (P/'runs/A16_beam_full/physics_boundary_trace.jsonl').open() if json.loads(l).get('cycle_step')==10632 and json.loads(l).get('boundary')=='before_action')
    motions=[json.loads(l) for l in (P/'runs/A16_beam_full/motion_trace.jsonl').open() if 10631<=json.loads(l).get('step',-1)<=10637 and json.loads(l).get('robot')=='franka_left']
    fixtures=[]
    for part in (3,2,1,0):
        phase=next(n for n in specs if n.startswith('assemble_') and f'_part_{part}_' in n and n.endswith('_pickup_clearance'))
        anchors=[r for r in rows if r['phase']==phase]
        require(anchors,'missing pickup path')
        for r in (anchors[0],anchors[len(anchors)//2],anchors[-1]):
            q={k:np.asarray(v,float) for k,v in r['q'].items()};name=r['intended_grasp'][1];robot='franka_left';tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
            relation=np.linalg.inv(model.poses(q)[tcp])@np.asarray(r['part_poses'][name]);parts={body[k]:np.asarray(v,float) for k,v in r['part_poses'].items()}
            fixtures.append(dict(phase=phase,q=q,parts=parts,relation=relation,current=pose(np.asarray(r['part_poses'][name])),previous=q[robot][:6].copy(),kind='nominal_anchor_counterfactual'))
    q={k:np.asarray(v['joint_positions'][0],float) for k,v in boundary['robots'].items()}
    # Genuine nonzero published-actual preload from previous real lift command.
    previous=np.asarray(next(v['command_q'] for v in motions if v['step']==10631))
    fixtures.append(dict(phase=next(n for n in specs if '_part_3_' in n and n.endswith('_pickup_clearance')),q=q,
        parts={k:np.asarray(v,float) for k,v in entities['body_poses'].items()},relation=np.asarray(dec['captured_controller_relation']),
        current=pose(np.asarray(native['actual_object_pose'])),previous=previous,kind='A16_10632_loaded_entry_counterfactual'))
    results=[];negative=0;support=[]
    for i,f in enumerate(fixtures):
        robot='franka_left';name=specs[f['phase']]['local_skill']['object'];spec=specs[f['phase']]['local_skill'];qq={k:v.copy() for k,v in f['q'].items()};tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        qa=qq[robot][:6].copy();qprev=f['previous'].copy();rp=pose(f['relation']);current=f['current'];state={};stamp={'valid':True,'epoch':0,'index':100,'time':100/240,'dt':1/240}
        g={k:np.asarray(v,float) for k,v in cfg['target_poses'][spec['target_object_target']].items()}
        if spec.get('target_object_offset'):g['position']+=spec['target_object_offset']
        class Offline(Adapter):
            def _assisted_object_servo_context(self,**kw):return name,stamp,{},current,(rp['position'],rp['orientation']),state
            def _target_object_pose(self,**kw):return g
            def _current_arm_q(self,*args):return qa
            def _grasp_command_fk_pose(self,**kw):
                x={k:v.copy() for k,v in qq.items()};x[robot][:6]=kw['command_q'];return pose(model.poses(x)[tcp])
        adapter=Offline.__new__(Offline);adapter._insertion_lateral_alignment_active={}
        task=SimpleNamespace(phase_index=0,phase_entry_step=0,step_counter=0,_assisted_published_q={robot:qprev.copy()},_assisted_published_q_stamps={robot:{'epoch':0,'index':99}},_assisted_event=lambda *args,**kw:None)
        first=None;counts=0;caps=[]
        for age in range(25):
            stamp.update(index=100+age,time=(100+age)/240)
            command=adapter._assisted_absolute_object_servo_pose(phase_key=(0,),task=task,robot_name=robot,spec=spec,tracked_robots={},tracked_objects={},object_pose=current,target_object_pose=g)
            target=pose_matrix(command['position'],command['orientation'])
            def residual(x):
                ss={k:v.copy() for k,v in qq.items()};ss[robot][:6]=x;tt=model.poses(ss)[tcp]
                return np.r_[(tt[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@tt[:3,:3]).as_rotvec()]
            sol=least_squares(residual,qprev,max_nfev=60,ftol=1e-9,gtol=1e-9,xtol=1e-9)
            request=Adapter._limited_joint_target(current_q=qprev,target_q=sol.x,max_joint_step=spec['max_command_joint_step'])
            request=Adapter._limit_command_to_measured_state(current_q=qa,command_q=request,spec=spec)
            final,detail=adapter._assisted_verify_published_held_command(task=task,robot_name=robot,context=dict(spec=spec,phase_key=(0,),phase_index=0,phase_entry_step=0,tracked_robots={},tracked_objects={}),command_q=request,current_q=qa,previous_q=qprev)
            cap=state['phase_handoff_joint_step'];require(np.max(abs(final-qprev))<=cap+1e-10,'final cap exceeded');caps.append(cap)
            if first is None:first=float(np.max(abs(final-qprev)));require(first<1e-12,'entry publication preload changed')
            actual_model=object_pose_from_tcp(pose(model.poses(qq)[tcp]),rp['position'],rp['orientation'])
            for fraction in (0.,.5,1.):
                qs={k:v.copy() for k,v in qq.items()};qs[robot][:6]=qa+fraction*(final-qa);poses=model.poses(qs)
                # Anchor every dynamic frame to the actual/nominal fixture; robot frames follow FK deltas.
                origin=model.poses(qq)
                for b,mat in f['parts'].items():
                    poses[b]=mat@np.linalg.inv(origin[b])@poses[b] if b in origin and '/robots/' in b else mat
                pred=anchored_object_prediction(current,actual_model,object_pose_from_tcp(pose(model.poses(qs)[tcp]),rp['position'],rp['orientation']))
                poses[body[name]]=pose_matrix(pred['position'],pred['orientation'])
                require(not missing_dynamic_poses(model,poses),'missing dynamic entity')
                checker.check(poses,f['phase'],i*100+age,(robot,name));counts+=1
                support.append(check_visual_support(checker,specs[f['phase']]['beam_assisted_visual_support'],f['phase'],i*100+age)['ready'])
            qprev=final
        results.append(dict(kind=f['kind'],phase=f['phase'],ticks=25,mesh_states=counts,entry_publication_delta_rad=first,preload_rad=float(np.max(abs(f['previous']-qa))),final_joint_cap_rad=caps[-1]))
    mesh=checker.report();require(mesh['meshes']==54 and mesh['self_checked'] and all(support),'54mesh/support failed')
    require(not any(v['depth']>1e-6 for v in mesh['intersections']),'non-target CAD overlap >1um')
    return dict(passed=True,fixtures=len(fixtures),ticks_per_fixture=25,results=results,full_mesh=mesh,support_checks=len(support),support_all_ready=all(support),
        source_sha256={k:sha(A/k) for k in (SK,ST,CO,TE)},tool_sha256=sha(__file__),native_physics_executed=0,
        limits=['All new trajectories are kinematic counterfactuals; measured actual q and bodies are held fixed while command history accumulates. No dynamics outcome is claimed.',
            'A16 step10632 is reclassified as a hypothetical pickup entry; it was actually a lift sample, not the unlogged10637 body sample.','Three finite samples per command; no between-sample CCD or unexported warehouse certification.'])

if __name__=='__main__':
    report=projection();save(P/'D01_reuse_projection.json',report)
    result=replay();save(P/'D01_handoff_geometry.json',result)
    print(json.dumps(dict(projection=True,geometry=True,fixtures=result['fixtures'],samples=result['full_mesh']['samples'])))
