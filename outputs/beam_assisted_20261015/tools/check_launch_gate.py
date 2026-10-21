"""Refuse long assisted runs without source-bound, separately evidenced checks."""
from pathlib import Path
import hashlib
import json
import os
import sys

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def validate(gate, *, run_name, source_manifest, tools_manifest):
    if gate.get('route') != 'assisted' or gate.get('run_name') != run_name:
        raise ValueError('launch gate is for a different route/run')
    if gate.get('source_manifest_sha256') != digest(source_manifest):
        raise ValueError('launch checks are stale for this source manifest')
    if gate.get('tools_manifest_sha256') != digest(tools_manifest):
        raise ValueError('launch checks are stale for these critical tools')
    required = {'actual_configuration_and_targets', 'full_phase_release_completion_interfaces', 'full_entities_and_camera_visibility'}
    if str(gate.get('cycle', '')).startswith(('C', 'D')):
        required |= {'production_controller_object_space_replay',
                     'admitted_error_and_final_command_geometry',
                     'bimanual_support_lifecycle'}
        if not str(gate.get('workflow_version', '')).startswith(('W03', 'W04', 'W05')):
            raise ValueError('C batch requires the current recorded workflow')
        if gate.get('right_arm_visual_support_required') is not True:
            raise ValueError('C batch requires reviewed visible right-arm support')
        if gate.get('control_safety_metric') != 'object_space_after_final_joint_limits':
            raise ValueError('C batch checks must use the final published carried-object pose')
    checks = gate.get('checks', [])
    if {r.get('name') for r in checks} != required:
        raise ValueError('launch gate missing required independent checks')
    for row in checks:
        if row.get('passed') is not True or not row.get('scope'):
            raise ValueError('launch check failed or has no declared scope: ' + str(row.get('name')))
        for evidence in row.get('evidence', []):
            if digest(evidence['path']) != evidence['sha256']:
                raise ValueError('launch evidence changed: ' + evidence['path'])
        if not row.get('evidence'):
            raise ValueError('launch check has no evidence')
    if gate.get('mandatory_unresolved'):
        raise ValueError('mandatory issues unresolved: ' + str(gate['mandatory_unresolved']))
    if gate.get('native_initialization_check_required') is not True:
        raise ValueError('runtime initialization validation must remain enabled')
    signature = gate.get('actual_configuration_signature', '')
    if len(signature) != 64 or any(c not in '0123456789abcdef' for c in signature):
        raise ValueError('reviewed actual randomized configuration signature is missing')
    if str(gate.get('cycle', '')).startswith('D'):
        from beam_workflow import validate_delta
        if len(checks) != 6 or not str(gate.get('workflow_version', '')).startswith(('W04', 'W05')):
            raise ValueError('D batch requires six unique checks and the recorded current workflow')
        validate_delta(gate, source_manifest, tools_manifest)
    return {'passed': True, 'route': 'assisted', 'run_name': run_name,
            'cycle': gate.get('cycle'), 'checked': sorted(required),
            'runtime_image_and_dynamics_still_require_native_verification': True,
            'formal_success': False}

if __name__ == '__main__':
    source = Path(sys.argv[1])
    report = validate(json.loads(source.read_text()), run_name=sys.argv[2],
                      source_manifest=os.environ['BEAM_ROUTE_MANIFEST'],
                      tools_manifest=os.environ['BEAM_ROUTE_TOOLS_MANIFEST'])
    Path(sys.argv[3]).write_text(json.dumps({'gate': json.loads(source.read_text()),
                                           'gate_sha256': digest(source),
                                           'validation': report}, indent=2) + '\n')
    print(json.dumps(report))
