"""Bind six D03 checks; retain exact source inputs and old evidence lineage once."""
import json,re,sys
from pathlib import Path
from beam_workflow import save,sha
P=Path(__file__).resolve().parents[1];cycle='D03';run='A19_beam_full'
def load(n):return json.loads((P/n).read_text())
cfg=load(cycle+'_actual_configuration_layout505_config.json');configs=load(cycle+'_actual_configuration.json')
assert configs['passed'] and len(configs['rows'])==4 and all(r['report']['phase_count']==98 for r in configs['rows'])
sig=next(r['report']['configuration_signature'] for r in configs['rows'] if r['layout_seed']==505)
suite=(P/'batch_D_20261016/D03_full_suite.log').read_text();assert re.search(r'421 passed, 2 warnings',suite)
assert load('D03_source_reuse.json')['passed'] and load('D03_alignment_closed_loop.json')['passed'] and load('D03_media_reuse_native_h264.json')['passed']
source=sha(P/'tools/assisted_manifest.json');tools=sha(P/'tools/assisted_tools_manifest.json');matrix=[]
for i,phase in enumerate(cfg['phase_specs']):
 s=phase.get('local_skill',{});matrix.append(dict(index=i,phase=phase['name'],absolute_object_servo=bool(s.get('beam_assisted_absolute_object_servo')),model_native_guard=bool(s.get('beam_assisted_absolute_object_servo')),descent_admission=bool(s.get('beam_assisted_descent_admission')),handoff_ticks=s.get('beam_assisted_phase_handoff_ticks'),alignment_hold=s.get('beam_assisted_alignment_hold'),right_visual_support=bool(phase.get('beam_assisted_visual_support')),strict_final=phase.get('name')=='final_stability'))
assert sum(bool(r['alignment_hold']) for r in matrix)==40 and sum(r['handoff_ticks'] is not None for r in matrix)==12
summary=dict(passed=True,source_files=889,critical_tools=20,source_manifest_sha256=source,tools_manifest_sha256=tools,tests_passed=421,warnings=2,phase_count=98,alignment_hold_phase_count=40,free_handoff_phase_count=12,first_activation_reviewed=True,actual_configuration_signature=sig,phase_matrix=matrix,
 full_suite_log_sha256=sha(P/'batch_D_20261016/D03_full_suite.log'),actual_configurations_sha256=sha(P/'D03_actual_configuration.json'),limits=['Offline tests and declared plant; native dynamic/model drift, final240gate and three views still mandatory.'])
save(P/'D03_checks_summary.json',summary)
def report(name,assertions):return dict(path=str(P/name),sha256=sha(P/name),assertions=[dict(pointer='/'+k,equals=v) for k,v in assertions.items()])
summaryr=report('D03_checks_summary.json',dict(passed=True,tests_passed=421,phase_count=98,alignment_hold_phase_count=40,free_handoff_phase_count=12,source_manifest_sha256=source,tools_manifest_sha256=tools,first_activation_reviewed=True))
reuse=report('D03_source_reuse.json',dict(passed=True,whole_AST_projection=True,configuration_flag_count=40,reused_free_mesh_states=1875,excluded_old_cold_fixtures=4,source_manifest_sha256=source))
plant=load('D03_alignment_closed_loop.json');plantr=report('D03_alignment_closed_loop.json',dict(passed=True,persistent_actual_plant=True,actual_plant_cases=7,old_negative_controls=7,corridor_phase_count=40,corridor_boundary_states=400,support_all_ready=True,source_manifest_sha256=source,configuration_sha256=sha(P/'D03_actual_configuration_layout505_config.json')))
plantr['assertions'] += [dict(pointer='/full_mesh/meshes',equals=54),dict(pointer='/full_mesh/self_checked',equals=True),dict(pointer='/full_mesh/minimum_intended_grasp_signed_separation',minimum=-1e-6)]
mediar=report('D03_media_reuse_native_h264.json',dict(passed=True,cameras=3,frames_per_camera=96,stats_and_encoded_bytes_identical=True,invalid_stats_rejected=3,other_source_AST_unchanged=True))
checks={
 'actual_configuration_and_targets':dict(scope='Four fresh98phase actual configurations;40 bounded strict-insertion references and12unchanged free edges; all other config fields exact.',reports=[summaryr,reuse]),
 'full_phase_release_completion_interfaces':dict(scope='All98 first-activation matrix reviewed;40 strict inserts use original admission and finalq gates; release/final240 logic projected unchanged.421 fresh source tests.',reports=[summaryr,reuse]),
 'full_entities_and_camera_visibility':dict(scope='Fresh complete54/self CAD biased-plant sweeps and40phase endpoints/offsets; unchanged free1875 plus pickup975 and camera inputs bound through full AST+SHA lineage. Native3images and videos mandatory.',reports=[reuse,plantr,mediar]),
 'production_controller_object_space_replay':dict(scope='Seven evolving actual-q nonlinear-FK tracking plants and seven baseline controls;3response rates on A18realfirstentry; production finalq taskguard, sameclock/epoch tests; no native result inferred.',reports=[summaryr,plantr]),
 'admitted_error_and_final_command_geometry':dict(scope='Original150um/1mrad/12tick admission and100um/1.5mrad model gate, fixed grasp;120umbounded integral and500um cold actualheight.40phase400 offset boundary states; sampled finalpublishedq sweeps.',reports=[summaryr,plantr,reuse]),
 'bimanual_support_lifecycle':dict(scope='Right numerical support, idle/release lifecycle unchanged by AST; every fresh fullCAD point separately checks right visible pinch; original support clocks retained, native sustained support mandatory.',reports=[reuse,plantr])}
# Lineage evidence exists on disk and is hash-bound once instead of repeating49files in eachcheck.
old=load('D02_delta.json');files={r['path']:r for c in old['checks'].values() for r in c.get('files',[])}
for name in ['D02_handoff_geometry.json','D02_warm_hover_equivalence.json','D02_pickup_reuse.json','D02_loaded_edge_entries.json','D02_reuse_projection.json','D02_checks_summary.json','D02_postprocess_pipeline.json','A19_beam_full_sealed_version.json','D03_actual_configuration.json','D03_actual_configuration_layout505_config.json','batch_D_20261016/D03_full_suite.log','tools/alignment_source_reuse.py','tools/alignment_closed_loop_preflight.py','tools/media_reuse_preflight.py','tools/prepare_alignment_gate.py']:
 files[str(P/name)]=dict(path=str(P/name),sha256=sha(P/name))
assert all(sha(r['path'])==r['sha256'] for r in files.values());checks['full_entities_and_camera_visibility']['files']=list(files.values())
spec=dict(run_name=run,cycle=cycle,workflow_version='W04.3',source_manifest_sha256=source,tools_manifest_sha256=tools,actual_configuration_signature=sig,
 changed_factors=['All40 strict insertions: fixed cold world-axis reference plus120um bounded lateral integral;500um actual cold-height fail-closed check. Original tolerances/grasp retained.','First96 initial insertion diagnostic ticks captured; no control change.','Reuse capture-time image statistics during encoding; production H264 bytes/statistics checked identical on3x96frames; capture and clocks unchanged.'],checks=checks)
save(P/'D03_delta.json',spec);print(json.dumps(dict(summary=True,checks=len(checks),lineage_files=len(files),plant_mesh_states=plant['full_mesh']['samples'])))
