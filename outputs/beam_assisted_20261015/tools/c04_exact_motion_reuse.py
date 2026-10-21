"""Exact C04 bridge: two reviewed state AST insertions, all other inputs sealed."""
import argparse,ast,copy,hashlib,json,tarfile
from pathlib import Path
STATE='toolkits/factory_dual_franka_assembly/beam_assisted_state.py'
TEST='tests/toolkits/test_beam_idle_hold.py'
OLD_GATE='db9008c96bff772b04256348ca4ec493f65c1374bee8b4ecf7580a18b1a63702'
IDLE_HASH='588385f369d28278fd120a98b4dfb6fdef6a9c973829a006ccbd4cb5c2cb82f3'
META_HASH='614462b57d2d1944e9ecf9ae63ee24504c39ce898b1e1139a208b3e01c055644'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def digest(v):return hashlib.sha256(v).hexdigest()
def require(v,m):
    if not v:raise ValueError(m)
def dump(n):return ast.dump(n,include_attributes=False)
def proof(old,new):
    t=ast.parse(new);idle=next(n for n in ast.walk(t) if isinstance(n,ast.FunctionDef) and n.name=='_assisted_idle_action')
    support=ast.parse("support = phase_spec.get('beam_assisted_visual_support') or {}").body[0]
    condition=ast.parse("support.get('robot') == robot_name",mode='eval').body
    assign=[n for n in idle.body if dump(n)==dump(support)]
    branches=[n for n in idle.body if isinstance(n,ast.If) and dump(n.test)==dump(condition)]
    if len(assign)!=1 or len(branches)!=1:return False
    if digest(dump(branches[0]).encode())!=IDLE_HASH:return False
    replaced=[]
    for n in idle.body:
        if n is assign[0]:continue
        if n is branches[0]:replaced.extend(n.orelse)
        else:replaced.append(n)
    idle.body=replaced
    guard=next(n for n in ast.walk(t) if isinstance(n,ast.FunctionDef) and n.name=='_assisted_guard_actions')
    condition=ast.parse("(current_phase.get('beam_assisted_visual_support') or {}).get('robot') == name",mode='eval').body
    metadata=[n for n in ast.walk(guard) if isinstance(n,ast.If) and dump(n.test)==dump(condition)]
    if len(metadata)!=1 or digest(dump(metadata[0]).encode())!=META_HASH:return False
    class Remove(ast.NodeTransformer):
        def visit_If(self,node):
            if node is metadata[0]:return None
            return self.generic_visit(node)
    Remove().visit(guard)
    return dump(t)==dump(ast.parse(old))
def controls(old,new):
    out={'exact_two_reviewed_insertions_accepted':proof(old,new),'unchanged_old_source_rejected':not proof(old,old),
         'comment_only_change_accepted':proof(old,new+'\n# comment only\n')}
    for name,before,after in (
      ('shared_guard_math_change_rejected','bounded_native_target(target[0], actual, previous.get(name))','bounded_native_target(target[0], actual+.001, previous.get(name))'),
      ('ordinary_idle_change_rejected','setdefault(robot_name, actual.copy())','setdefault(robot_name, actual+.001)'),
      ('freshness_relaxation_rejected',"not in (0, 1)","not in (0, 1, 2)"),
      ('active_branch_change_rejected',"pop(robot_name, None)","pop(robot_name, 1)")):
        changed=new.replace(before,after,1);out[name]=changed!=new and not proof(old,changed)
    out['unrelated_module_change_rejected']=not proof(old,new+'\nunreviewed_change=True\n')
    return out
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in ('repository','tools','source-after','config','control-report','geometry-report'):
        ap.add_argument('--'+n,type=Path,required=True)
    ap.add_argument('--output',type=Path);a=ap.parse_args();p=a.tools.parent
    oldgate=p/'launch_gates/A15_beam_full.json';require(sha(oldgate)==OLD_GATE,'original C03 gate changed')
    gate=json.loads(oldgate.read_text());original_evidence={}
    for row in gate['checks']:
        require(row['passed'] is True and row['evidence'],'original gate incomplete')
        for v in row['evidence']:
            h=sha(v['path']);require(h==v['sha256'],'original gate evidence changed: '+v['path'])
            original_evidence[v['path']]=dict(path=v['path'],sha256=h,expected_sha256=v['sha256'],unchanged=True)
    after=json.loads(a.source_after.read_text());require(after['differences']==after['tool_differences']==[],'A15 not sealed clean')
    oldmap={v['path']:v['sha256'] for v in after['files']};oldtools={v['path']:v['sha256'] for v in after['tool_files']}
    require(len(oldmap)==889 and len(oldtools)==19,'original889/19 coverage missing')
    seal=json.loads((p/'A15_beam_full_sealed_version.json').read_text());archives=[]
    for v in seal['archives']:
        f=p/'versions'/Path(v['path']).name;require(sha(f)==v['sha256'] and f.stat().st_size==v['bytes'],'sealed original archive changed');archives.append(f)
    with tarfile.open(archives[0]) as t:
        original={n.name:t.extractfile(n).read() for n in t.getmembers() if n.isfile()}
    require({k:digest(v) for k,v in original.items()}==oldmap,'source-after map differs from sealed original889')
    with tarfile.open(archives[1]) as t:require(json.loads(t.extractfile('assisted_tools_manifest.json').read())==oldtools,'tool-after map differs from original19 archive')
    current={k:sha(a.repository/k) for k in oldmap};changed=sorted(k for k in current if current[k]!=oldmap[k])
    require(changed==sorted([STATE,TEST]),'changes beyond exactly state/test')
    currenttools={k:sha(a.tools/k) for k in oldtools};require(currenttools==oldtools,'critical19 tool changed')
    old=original[STATE].decode();new=(a.repository/STATE).read_text();require(proof(old,new),'whole state AST changed beyond exact reviewed idle/metadata nodes')
    negative=controls(old,new);require(all(negative.values()),'AST contract control failed')
    control=json.loads(a.control_report.read_text());require(control['passed'] is True and control['state_support_idle_and_metadata_only_ast'] is True and control['guard_numerical_ast_unchanged'] is True,'handoff production replay failed')
    require(control['source_sha256']=={STATE:current[STATE],TEST:current[TEST]},'control report source stale')
    require(control['old_native_counterexample_reproduced'] is True and control['observed_old_target_jump_rad']>.0006 and control['new_first_idle_target_delta_rad']<1e-12,'old/new handoff counterexample absent')
    geometry=json.loads(a.geometry_report.read_text());require(geometry['passed'] is True and geometry['counterfactual']['total_states']==9 and geometry['counterfactual']['actual_complete_native_states_claimed']==0,'bounded counterfactual proof missing')
    expected={
      'actual_configuration':(a.config,sha(p/'C02_actual_configuration_preflight_v18_layout505_config.json')),
      'geometric_corridor_source_geometry':(p/'runs/A12_beam_full/scene_geometry.json','aac97863c2b892136d9a238bbea92d2c7740684b360c3cfc5b5eddf7a98c38fb'),
      'nominal_bimanual_path':(p/'C01_bimanual_repaired_v2_path.jsonl','d2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7'),
      'right_support_candidate':(p/'C01_right_support_candidate.json','8c248424e4f6c4db970d6df4a8d87171bebbd3b68d382e04dafad635ce50109e'),
      'right_support_acquisition_path':(p/'C01_right_support_path.jsonl','d15d23f448ab6fdb66fd371268d755b9ce540b22aad60e5e0b8eef1537022461'),
      'camera_full_report':(p/'C01_camera_full_path.json','0f3482bb761327244bee764830fbab894c649fc8afca0dc1cc16dd2b243862df'),
      'camera_right_report':(p/'C01_camera_right_final_v2_path.json','5423aaa9b5554fba34b83a28e435489b517c2009f497940ad46ebd186c8227d2')}
    unchanged={}
    for name,(path,h) in expected.items():
        actual=sha(path);require(actual==h,'reused input changed: '+name)
        unchanged[name]=dict(path=str(path),sha256=actual,expected_sha256=h,unchanged=True)
    def bound(f):return dict(path=str(f),sha256=sha(f))
    result=dict(schema='beam_C04_exact_motion_reuse_v1',passed=True,errors=[],tool_sha256=sha(__file__),source_files=889,critical_tools=19,
       source_change=dict(changed_paths=changed,whole_state_ast_restored=True,allowed_methods=['_assisted_idle_action','_assisted_guard_actions'],guard_numerical_ast_unchanged=True,ordinary_idle_ast_unchanged=True,
          old_state_sha256=oldmap[STATE],new_state_sha256=current[STATE],reviewed_idle_branch_ast_sha256=IDLE_HASH,reviewed_metadata_if_ast_sha256=META_HASH),
       current_source_sha256=current,critical_tools_sha256=currenttools,reference_gate=bound(oldgate),source_after=bound(a.source_after),source_archive=bound(archives[0]),tools_archive=bound(archives[1]),
       control_validation=bound(a.control_report),geometry_validation=bound(a.geometry_report),original_gate_evidence=original_evidence,unchanged_inputs=unchanged,ast_contract_controls=negative,
       reused_scope=dict(cached_cases=600,cached_full54_self_states=3775,original_C01_cases=5943,nominal_path_states=6306,source_tests_reused=0),fresh_source_tests_required=True,
       limits=['Only unchanged input calculations/mesh algorithms are reused. Actual support-idle handoff was absent from old600/5943 cases.',
         'New support targets are validated by separate production lifecycle replay; frozen-right CAD states remain counterfactual, not native dynamics.'],native_physics_executed=0)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')
if __name__=='__main__':main()
