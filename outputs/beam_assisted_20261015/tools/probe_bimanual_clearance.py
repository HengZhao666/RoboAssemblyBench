"""Probe only changed part2 three-leg high transport, then splice prior evidence."""
import ast,copy,hashlib,json,sys
from pathlib import Path
import numpy as np
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
generator=P/'tools/nominal_bimanual.py'
tree=ast.parse(generator.read_text())
prefix=[]
for node in tree.body:
    if isinstance(node,ast.Try):break
    prefix.append(node)
ns={'__file__':str(generator)}
exec(compile(ast.Module(body=prefix,type_ignores=[]),str(generator),'exec'),ns)
import fcl
old_path=P/'C01_nominal_bimanual_path.jsonl'
old_report=P/'C01_nominal_bimanual.json'
old=[json.loads(line) for line in old_path.open()]
report=json.loads(old_report.read_text())
names=['assemble_01_part_2_'+suffix for suffix in ('pickup_clearance','assembly_clearance','transport_hover')]
first=next(i for i,r in enumerate(old) if r['phase']==names[0])
last=max(i for i,r in enumerate(old) if r['phase']==names[-1])
start=old[first-1]
reference=old[last]
relative=np.asarray(next(x['relative_pose'] for x in report['grasp_plans'] if x['object']=='fabrica_beam_2'))
phases=[next(p for p in ns['config']['phase_specs'] if p['name']==name) for name in names]
target_names=[phase['local_skill']['target_object_target'] for phase in phases[:2]]
original_targets=copy.deepcopy(ns['targets'])
v=ns['v'];m=ns['m']
right_indices=[i for i,x in enumerate(v.meshes) if x['robot']=='franka_right']
part_indices=[i for i,x in enumerate(v.meshes) if x['object']=='fabrica_beam_2']
base_record=ns['record']
cases=[];chosen=None;chosen_rows=None
for rise in (.06,.10,.14):
    v.rows=[];v.minimum=None;v.samples=0;v.pair_checks=0;v.grasp_contacts=[];v.intrinsic_interfaces=[]
    ns['q']={name:np.asarray(value,float) for name,value in start['q'].items()}
    ns['placed']={name:np.asarray(B,float) for name,B in start['part_poses'].items() if name!='fabrica_beam_2'}
    ns.update(robot='franka_left',held='fabrica_beam_2',relative=relative.copy(),rows=[],segments=[],support_failures=[],support_min=float('inf'),support_max=0.,support_active=True)
    ns['targets']=copy.deepcopy(original_targets)
    for name in target_names:ns['targets'][name]['position'][2]+=rise
    minimum=[float('inf'),None]
    def record(phase):
        base_record(phase)
        for i in right_indices:
            for j in part_indices:
                a,b=v.shapes[i],v.shapes[j]
                result=fcl.CollisionResult()
                hit=fcl.collide(a,b,fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
                distance=-max((float(c.penetration_depth) for c in result.contacts),default=0.) if hit else float(fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()))
                if distance<minimum[0]:minimum[:]=[distance,{'phase':phase,'sample':len(ns['rows'])-1,'mesh_a':v.meshes[i]['prim'],'mesh_b':v.meshes[j]['prim']}]
    ns['record']=record
    failure=None
    try:
        for phase in phases:ns['waypoint'](phase)
    except Exception as exc:failure=type(exc).__name__+': '+str(exc)
    audit=v.report()
    bad=[x for x in audit['pairs'] if x['max_depth']>1e-6]
    grips=[x for x in audit['intended_grasp_mesh_separations'] if x['signed_separation']< -1e-6]
    end_q=ns['q']['franka_left']
    branch_error=float(np.max(abs(end_q-np.asarray(reference['q']['franka_left']))))
    accepted=bool(failure is None and not bad and not grips and not ns['support_failures'] and minimum[0]>=.005 and branch_error<1e-5)
    case={'rise_m':rise,'accepted':accepted,'failure':failure,'samples':len(ns['rows']),'right_part2_minimum_distance_m':minimum[0],'right_part2_minimum_pair':minimum[1],
          'endpoint_max_q_difference_from_reused_suffix_rad':branch_error,'non_target_failure_pairs':bad,'grasp_failure_count':len(grips),'right_support_failure_count':len(ns['support_failures']),
          'audit':audit,'segments':copy.deepcopy(ns['segments'])}
    cases.append(case)
    print(json.dumps({key:case[key] for key in ('rise_m','accepted','failure','samples','right_part2_minimum_distance_m','endpoint_max_q_difference_from_reused_suffix_rad')}),flush=True)
    if accepted:
        chosen=case;chosen_rows=copy.deepcopy(ns['rows']);break
out=P/'C01_part2_clearance_probe.json'
payload={'passed':chosen is not None,'cases':cases,'chosen_rise_m':None if chosen is None else chosen['rise_m'],
    'replaced_old_range_inclusive':[first,last],'changed_phase_names':names,
    'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (generator,Path(__file__),old_path,old_report)},
    'full_native_claim':False,'scope':'only affected part2 pickup ascent/carry rotation/hover descent; 54 meshes with self; target/grasp/constant right-support and installed part3 kept; endpoint must return to previous suffix joint state within10urad'}
out.write_text(json.dumps(payload,indent=2)+'\n')
if chosen is None:raise SystemExit(3)
new_path=P/'C01_bimanual_repaired_path.jsonl'
combined=old[:first]+chosen_rows+[row for row in old[last+1:] if row['phase']!='base_6_park']
with new_path.open('w') as stream:
    for row in combined:stream.write(json.dumps(row)+'\n')
segments={'source_old_sha256':hashlib.sha256(old_path.read_bytes()).hexdigest(),
    'source_short_report_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),
    'reused_prefix_old_range':[0,first-1],'replacement_rows':len(chosen_rows),
    'reused_suffix_old_range':[last+1,len(old)-1],'omitted_old_phases':['base_6_park'],
    'chosen_rise_m':chosen['rise_m'],'rows':len(combined),'output':str(new_path),
    'sha256':hashlib.sha256(new_path.read_bytes()).hexdigest(),
    'suffix_q_endpoint_error_rad':chosen['endpoint_max_q_difference_from_reused_suffix_rad'],
    'conditions':['same robot initial/rest geometry','same original named targets except specified2 Z fields',
        'same measured grasp and right-support pose','all original failed part2 rows replaced, final failed rightpark removed',
        'new runtime dynamics still require native verification']}
(P/'C01_bimanual_reuse_manifest.json').write_text(json.dumps(segments,indent=2)+'\n')
print(json.dumps(segments),flush=True)
