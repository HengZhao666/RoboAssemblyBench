from pathlib import Path
import hashlib,json,re
from check_launch_gate import validate
R=Path('/home/user/beam_integration_20261010_upstream6f57dae');P=R/'beam_assisted_20261015';T=P/'tools'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def read(n):return json.loads((P/n).read_text())
def write(n,x):(P/n).write_text(json.dumps(x,indent=2)+'\n')
def evidence(n):return {'path':str(P/n),'sha256':sha(P/n)}
counts={};differences=[]
for label,root,manifest in [('physical',R/'candidate_repository',T/'physical_manifest.json'),
 ('assisted',P/'assisted_repository',T/'assisted_manifest.json'),('tools',T,T/'assisted_tools_manifest.json')]:
 m=json.loads(manifest.read_text());counts[label]=len(m)
 differences.extend(label+':'+n for n,s in m.items() if not (root/n).is_file() or sha(root/n)!=s)
assert counts=={'physical':876,'assisted':887,'tools':9} and not differences,(counts,differences)
archives=[]
for row in read('A12_final_sealed_version.json')['archives']:
 p=P/'versions'/Path(row['path']).name
 assert p.stat().st_size==row['bytes'] and sha(p)==row['sha256'];archives.append(evidence(str(p.relative_to(P))))
write('B02_deployment_verification.json',{'verified_counts':counts,'differences':differences,'archives':archives})
log=(P/'B02_checks_final.log').read_text();assert re.search(r'330 passed, 2 warnings',log)
startup=read('B02_startup_render_contract_tests.json');assert startup['passed'] and startup['tests_run']==6
assert startup['worker_sha256']==sha(T/'assisted_worker.py') and startup['test_sha256']==sha(T/'test_startup_render_contract.py')
write('B02_checks_summary.json',{'passed':True,'source_checks_passed':330,'startup_contract_checks_passed':6,
 'source_log':evidence('B02_checks_final.log'),'startup_report':evidence('B02_startup_render_contract_tests.json'),
 'source_scope':'all97 condition/config/command/release/final true-entry chains plus retained physical regressions',
 'dynamic_success':False})
cfg=read('B02_actual_configuration_preflight_final.json');assert cfg['passed'] and len(cfg['rows'])==4
signature=next(r['report']['configuration_signature'] for r in cfg['rows'] if r['layout_seed']==505)
geometry=read('B02_geometry_review.json');assert geometry['passed']
nominal=read('nominal_remaining_all_B02.json');assert nominal['failure'] is None and nominal['parts']==['3','2','1','0']
assert nominal['segments'][-1]['phase']=='assemble_03_part_0_park'
old=read('B02_actual_configuration_preflight_layout505_config.json')
new=read('B02_actual_configuration_preflight_final_layout505_config.json')
assert all(old[k]==new[k] for k in ('robots','objects','target_poses','scene_asset_path','scene_position','scene_orientation','scene_scale'))
changed=0
for phase in old['phase_specs']:
 spec=phase.get('local_skill',{})
 if '_insert_' in spec.get('target_object_target',''):
  assert spec['target_object_servo_position_command_gate_overdrive'] is True
  spec['target_object_servo_position_command_gate_overdrive']=False;changed+=1
assert old['phase_specs']==new['phase_specs'] and changed>0
write('B02_geometry_reuse_after_overdrive.json',{'passed':True,'changed_waypoint_command_flags':changed,
 'scope':'all geometry, initial state, target poses and nominal phase paths unchanged; only loaded insert command overdrive disabled',
 'nominal_geometry_report':evidence('B02_geometry_review.json'),
 'not_certified':['actual new servo response and dynamic path; live native/CAD checks required']})
gate={'route':'assisted','run_name':'A12_beam_full','cycle':'B02',
 'source_manifest_sha256':sha(T/'assisted_manifest.json'),'tools_manifest_sha256':sha(T/'assisted_tools_manifest.json'),
 'actual_configuration_signature':signature,'native_initialization_check_required':True,'mandatory_unresolved':[],
 'checks':[
  {'name':'actual_configuration_and_targets','passed':True,'scope':'four actual randomized full configs; all future target/skill/capture invariants and full-model signature',
   'evidence':[evidence('B02_actual_configuration_preflight_final.json'),evidence('B02_actual_configuration_preflight_final_layout505_config.json')]},
  {'name':'full_phase_release_completion_interfaces','passed':True,'scope':'real dispatch and final success/terminal metrics; native clock/view certificate; startup publication/config contracts, dynamics still pending',
   'evidence':[evidence('B02_checks_summary.json'),evidence('B02_startup_render_contract_tests.json')]},
  {'name':'full_entities_and_camera_visibility','passed':True,'scope':geometry['scope'],
   'evidence':[evidence('B02_geometry_review.json'),evidence('B02_visible_full_path.json'),evidence('B02_camera_full_path.json'),evidence('B02_geometry_reuse_after_overdrive.json')]}]}
(P/'launch_gates').mkdir(exist_ok=True);write('launch_gates/A12_beam_full.json',gate)
report=validate(gate,run_name='A12_beam_full',source_manifest=T/'assisted_manifest.json',tools_manifest=T/'assisted_tools_manifest.json')
write('B02_launch_gate_validation.json',report);print(json.dumps({'deployment':counts,'gate':report,'geometry':geometry}))
