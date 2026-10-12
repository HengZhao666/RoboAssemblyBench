"""Protect the authoritative v169 phase and layout contract during integration."""
import hashlib
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
        assert _digest(phase) == expected['phase_digests'][phase['name']], phase['name']
    assert _normalized(recipe['targets']) == expected['targets']
    assert _normalized(recipe['success']) == expected['success']
    assert _normalized(recipe['domain_randomization']) == expected['domain_randomization']
