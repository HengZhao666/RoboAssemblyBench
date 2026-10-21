"""Protect the authoritative v169 phase and layout contract during integration."""
import hashlib
import copy
import json
from pathlib import Path

from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe


def _normalized(value):
    if isinstance(value, dict):
        return {key: _normalized(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_normalized(item) for item in value]
    if isinstance(value, float):
        return round(value, 12)
    return value


def _digest(value):
    payload = json.dumps(_normalized(value), sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(payload.encode()).hexdigest()


def test_continuous_beam_keeps_v169_phase_controls_and_layout():
    expected = json.loads((Path(__file__).parent / 'fixtures/beam_v169_recipe_contract.json').read_text())
    recipe = load_task_recipe('fabrica_beam_ur5e_staged', scene_profile='taoyuan_grscenes_tabletop')
    assert [phase['name'] for phase in recipe['phases']] == expected['phase_order']
    for phase in recipe['phases']:
        baseline_phase = copy.deepcopy(phase)
        if phase['name'] == 'assemble_00_part_3_close_and_attach':
            assert baseline_phase['local_skill'].pop('close_force_recenter_track_weak_side') is True
        else:
            assert not (baseline_phase.get('local_skill') or {}).get('close_force_recenter_track_weak_side', False)
        if phase['name'] == 'beam_final_unloaded_verification':
            # The intentional repair moves the same 240-step duration to the
            # complete predicate. Preserve every other v169 phase/layout field.
            assert baseline_phase['advance'].pop('physics_stable_steps') == 240
            table = baseline_phase['advance']['conditions'][0]
            assert table['type'] == 'beam_table_supported' and table['stable_steps'] == 1
            table['stable_steps'] = 240
        if phase['name'] in {
                'assemble_00_part_3_insert_09', 'assemble_01_part_2_insert_09',
                'assemble_02_part_1_insert_09', 'assemble_03_part_0_insert_09'}:
            # v193 advanced at 5.5 mm, then the unchanged installed 3 mm
            # guard rejected it. Verify the stricter six fields explicitly,
            # and preserve the frozen v169 digest for every other field.
            old_values = {
                'position_tolerance': .006, 'relaxed_position_tolerance': .015,
                'target_object_position_tolerance': .008,
                'relaxed_target_object_position_tolerance': .015,
                'orientation_tolerance': .08, 'target_object_orientation_tolerance': .10,
            }
            for key, old_value in old_values.items():
                expected_value = .025 if 'orientation' in key else .0015
                assert baseline_phase['local_skill'][key] == expected_value
                baseline_phase['local_skill'][key] = old_value
        assert _digest(baseline_phase) == expected['phase_digests'][phase['name']], phase['name']
    assert _normalized(recipe['targets']) == expected['targets']
    assert _normalized(recipe['success']) == expected['success']
    assert _normalized(recipe['domain_randomization']) == expected['domain_randomization']
