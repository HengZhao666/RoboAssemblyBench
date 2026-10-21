"""Build any D-batch gate from declarative, independently bound assertions."""
import argparse,json
from pathlib import Path
from beam_workflow import save,sha
from check_launch_gate import validate

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('specification',type=Path)
    a=ap.parse_args();p=Path(__file__).resolve().parents[1]
    spec=json.loads(a.specification.read_text())
    for check in spec['checks'].values():
        for report in check['reports']:
            if report.get('sha256')!=sha(report['path']):raise ValueError('Expected report SHA absent or stale')
    source=p/'tools/assisted_manifest.json';tools=p/'tools/assisted_tools_manifest.json'
    if spec['source_manifest_sha256']!=sha(source) or spec['tools_manifest_sha256']!=sha(tools):raise ValueError('Specification is stale')
    gate=dict(route='assisted',run_name=spec['run_name'],cycle=spec['cycle'],workflow_version=spec['workflow_version'],
        source_manifest_sha256=sha(source),tools_manifest_sha256=sha(tools),
        right_arm_visual_support_required=True,control_safety_metric='object_space_after_final_joint_limits',
        mandatory_unresolved=[],native_initialization_check_required=True,
        actual_configuration_signature=spec['actual_configuration_signature'],
        delta_manifest=dict(path=str(a.specification),sha256=sha(a.specification)),
        checks=[dict(name=k,scope=v['scope'],passed=True,
            evidence=[dict(path=r['path'],sha256=r['sha256']) for r in v['reports']]+v.get('files',[])) for k,v in spec['checks'].items()])
    if spec.get('bounded_primary_scene_probe') is not None:gate['bounded_primary_scene_probe']=spec['bounded_primary_scene_probe']
    result=validate(gate,run_name=gate['run_name'],source_manifest=source,tools_manifest=tools)
    output=p/'launch_gates'/(gate['run_name']+'.json');save(output,gate)
    save(p/(gate['cycle']+'_launch_validation.json'),result)
    print(json.dumps(dict(gate=str(output),sha256=sha(output),validation=result)))
if __name__=='__main__':main()
