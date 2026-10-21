"""Reject deterministic assisted-route contract errors before the first action.

This is a configuration/interface check. It does not certify native motion,
contacts, mesh clearance, camera visibility or successful assembly.
"""
from __future__ import annotations

import math
import hashlib
import json
from enum import Enum
from pathlib import Path


SUPPORTED_ADVANCE_TYPES = frozenset({
    'beam_table_supported', 'all_of', 'any_of', 'timer',
    'local_skill_complete', 'local_skills_complete', 'skill_complete', 'skills_complete',
    'robot_targets_reached', 'robot_pose_reached',
    'object_targets_reached', 'object_pose_reached', 'object_lifted', 'objects_lifted',
    'objects_static', 'object_static', 'object_attached', 'objects_attached',
    'object_grasped', 'objects_grasped', 'object_detached', 'objects_detached',
    'object_released', 'objects_released', 'robot_object_contact',
    'gripper_object_contact', 'success_criteria_met',
})
BEAM_OBJECTS = frozenset('fabrica_beam_' + pid for pid in ('6', '3', '2', '1', '0'))


def _get(value, name, default=None):
    return value.get(name, default) if isinstance(value, dict) else getattr(value, name, default)


def _entries(value):
    return value if isinstance(value, list) else ([] if value is None else [value])


def _home_robot_configs(config):
    """Resolve logical arm names before or after the framework's env suffix.

    setup_offset_for_assets changes robot config names to <logical>_<env_id>,
    while phase/home targets keep logical names. Accept one complete naming
    scope and reject duplicate aliases, missing arms or mixed environments.
    """
    logical = list(_get(config, 'robot_names', []) or [])
    if (not logical or any(not isinstance(name, str) or not name for name in logical)
            or len(set(logical)) != len(logical)):
        raise ValueError('home robot names must be unique nonempty logical names')
    resolved = {}
    scopes = set()
    for robot in _get(config, 'robots', []) or []:
        actual = _get(robot, 'name')
        if not isinstance(actual, str):
            raise ValueError('home robot config lacks a string name')
        matches = []
        for name in logical:
            if actual == name:
                matches.append((name, None))
            elif actual.startswith(name + '_'):
                suffix = actual[len(name) + 1:]
                if suffix.isdecimal() and suffix == str(int(suffix)):
                    matches.append((name, int(suffix)))
        if len(matches) != 1:
            raise ValueError('home robot config name has no unique framework alias: ' + actual)
        name, scope = matches[0]
        if name in resolved:
            raise ValueError('home robot config repeats a logical alias: ' + name)
        resolved[name] = robot
        scopes.add(scope)
    if set(resolved) != set(logical):
        raise ValueError('home robot configs must cover every declared logical arm')
    if len(scopes) != 1:
        raise ValueError('home robot configs mix unscoped or different framework environments')
    return resolved


def _vector(value, size):
    try:
        return len(value) == size and all(math.isfinite(float(v)) for v in value)
    except (TypeError, ValueError):
        return False


def _plain(value):
    """Keep every model field while normalizing data containers for hashing."""
    if hasattr(value, 'model_dump'):
        # TaskCfg annotates robots/objects with their base config classes.
        # Default Pydantic dumping omits subclass initial-joint/drive fields.
        return _plain(value.model_dump(mode='python', serialize_as_any=True))
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(v) for v in value]
    if isinstance(value, Enum):
        return _plain(value.value)
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, 'tolist'):
        return _plain(value.tolist())
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    raise TypeError('Unsupported configuration value: ' + type(value).__name__)


def _signature(value):
    return hashlib.sha256(json.dumps(_plain(value), sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def configuration_signature(config):
    """Bind evidence to the entire task model; no configuration fields excluded.

    Asset file contents and simulator settings outside this task model still
    require their separate source/geometry/runtime evidence.
    """
    return _signature(config)


class AssistedConfigurationError(ValueError):
    def __init__(self, report):
        self.report = report
        super().__init__('assisted configuration preflight rejected: ' + '; '.join(report['errors']))


def assert_assisted_configuration(config_or_task, *, full_mode=True):
    """Inspect an episode config/model/dict, or the initialized task using it."""
    config = _get(config_or_task, 'cfg', _get(config_or_task, 'config', config_or_task))
    runtime = config is not config_or_task
    phases = _get(config_or_task, 'phase_specs', _get(config, 'phase_specs', []))
    targets = _get(config_or_task, 'target_poses', _get(config, 'target_poses', {}))
    objects = {str(_get(obj, 'name')) for obj in _get(config, 'objects', [])}
    objects.update(map(str, _get(config, 'tracked_object_names', [])))
    robots = set(map(str, _get(config, 'robot_names', [])))
    errors = []

    def check(condition, message):
        if not condition:
            errors.append(message)

    check(isinstance(phases, list) and bool(phases), 'phase_specs must be a nonempty list')
    check(isinstance(targets, dict) and bool(targets), 'target_poses must be a nonempty mapping')
    if not isinstance(phases, list) or not phases or not isinstance(targets, dict):
        raise AssistedConfigurationError({'passed': False, 'errors': errors})
    if runtime:
        check(_plain(phases) == _plain(_get(config, 'phase_specs')),
              'runtime phase_specs differ from reviewed task configuration')
        check(_plain(targets) == _plain(_get(config, 'target_poses')),
              'runtime target_poses differ from reviewed task configuration')
        check(_get(config_or_task, 'max_steps') == _get(config, 'max_steps'),
              'runtime task budget differs from reviewed task configuration')
    for name, pose in targets.items():
        check(isinstance(pose, dict) and _vector(pose.get('position'), 3)
              and _vector(pose.get('orientation'), 4)
              and any(abs(float(v)) > 1e-12 for v in pose['orientation']),
              f'{name}: every target requires a finite position and nonzero finite quaternion')
        if '_insert_' in str(name):
            final_name = str(name).split('_insert_', 1)[0] + '_assembled'
            final_pose = targets.get(final_name, {})
            if (isinstance(pose, dict) and _vector(pose.get('position'), 3)
                    and _vector(final_pose.get('position'), 3)):
                check(float(pose['position'][2]) >= float(final_pose['position'][2]) + .003 - 1e-12,
                      f'{name}: loaded insertion waypoint falls below the 3 mm capture boundary')
    names = [p.get('name') for p in phases]
    check(None not in names and len(set(names)) == len(names), 'phase names must be unique and present')
    check(BEAM_OBJECTS <= objects, 'five Beam objects are missing from configured objects')
    check(robots == {'franka_left', 'franka_right'}, 'assisted route requires the two configured UR5e arms')
    check(all(p.get('beam_assisted') is True for p in phases), 'every phase must select the assisted route')
    metadata = _get(config, 'benchmark_metadata', {})
    check(metadata.get('beam_execution_route') == 'assisted'
          and metadata.get('formal_physical_success') is False,
          'metadata must identify assisted route and exclude formal physical success')
    check(names[-1] == 'beam_assisted_final_verification', 'final assisted verification phase is missing')
    check(not any(str(name).endswith('_assisted_capture') for name in targets),
          'static assisted_capture aliases are forbidden; use the randomized original target plus offset')

    target_keys = {'target', 'target_name', 'target_object_target', 'target_object_final_target',
                   'target_pose_target', 'target_object_orientation_target', 'object_orientation_target'}
    object_keys = {'object', 'object_name', 'held_object', 'payload_object', 'offset_reference_object'}
    forbidden_nonempty = {'kinematic_carry', 'collision_filters', 'gripper_sibling_filters',
                          'rebase_from_object', 'rebase_targets', 'object_poses', 'joint_overrides'}
    forbidden_true = {'disable_collision_on_attach', 'disable_collision_on_lock',
                      'filter_gripper_collisions_on_attach', 'allow_noncontact_joint_attachment',
                      'pin_pose', 'teleport_to_target_once', 'snap_free_object', 'snap_on_open'}

    def inspect(value, path):
        if isinstance(value, list):
            for i, item in enumerate(value):
                inspect(item, f'{path}[{i}]')
        if not isinstance(value, dict):
            return
        for key, item in value.items():
            location = f'{path}.{key}'
            if key in target_keys and isinstance(item, str):
                check(item in targets, f'{location}: unknown target {item}')
            if key in object_keys and isinstance(item, str):
                check(item in objects, f'{location}: unknown object {item}')
            if key in {'robot', 'robot_name'} and isinstance(item, str):
                check(item in robots, f'{location}: unknown robot {item}')
            if key in forbidden_nonempty:
                check(not item, f'{location}: forbidden phase bypass')
            if key in forbidden_true:
                check(not item, f'{location}: forbidden phase bypass')
            if key in {'object_collisions', 'object_gravity'}:
                for entry in _entries(item):
                    check(isinstance(entry, dict) and entry.get('enabled') is True,
                          f'{location}: assistance must keep collisions and gravity enabled')
            inspect(item, location)

    def condition(advance, path):
        check(isinstance(advance, dict), f'{path}: condition must be a mapping')
        if not isinstance(advance, dict):
            return
        kind = advance.get('type', 'timer')
        check(kind in SUPPORTED_ADVANCE_TYPES, f'{path}: unsupported advance type {kind}')
        if kind in {'all_of', 'any_of'}:
            children = advance.get('conditions', advance.get('all' if kind == 'all_of' else 'any', []))
            check(isinstance(children, list) and bool(children), f'{path}: logical condition is empty')
            if isinstance(children, list):
                for i, child in enumerate(children):
                    condition(child, f'{path}.conditions[{i}]')
        if kind in {'object_targets_reached', 'object_pose_reached'}:
            entries = advance.get('objects')
            check(isinstance(entries, list) and bool(entries), f'{path}: object targets are empty')
            for entry in entries if isinstance(entries, list) else []:
                check(isinstance(entry, dict) and entry.get('object') in objects
                      and (entry.get('target') or entry.get('target_name')) in targets,
                      f'{path}: object target entry does not resolve')
        elif advance.get('objects') is not None:
            for entry in _entries(advance['objects']):
                name = entry.get('object') if isinstance(entry, dict) else entry
                check(name in objects, f'{path}: unknown condition object {name}')

    def skill_conditions(advance):
        if not isinstance(advance, dict):
            return
        if advance.get('type') in {'local_skill_complete', 'local_skills_complete',
                                   'skill_complete', 'skills_complete'}:
            yield advance
        children = advance.get('conditions', advance.get('all', advance.get('any', [])))
        for child in children if isinstance(children, list) else []:
            yield from skill_conditions(child)

    captures = {}
    attachments = {}
    installations = {}
    timeout_sum = 0
    for index, phase in enumerate(phases):
        path = str(phase.get('name', index))
        inspect(phase, path)
        condition(phase.get('advance', {}), path + '.advance')
        timeout = phase.get('timeout_steps', _get(config, 'phase_timeout_steps'))
        check(isinstance(timeout, int) and timeout > 0, f'{path}: positive timeout is required')
        if isinstance(timeout, int) and timeout > 0:
            timeout_sum += timeout
        skills = [phase.get('local_skill')] + list((phase.get('local_skills') or {}).values())
        outlets = set()
        for spec in filter(None, skills):
            admission = spec.get('beam_assisted_descent_admission')
            if admission is not None:
                check(isinstance(admission, dict) and admission.get('motion_metric', 'native_velocity')
                      in {'native_velocity', 'native_com_pose_difference'},
                      f'{path}: unsupported assisted descent motion metric')
            outlets.add((spec.get('robot'), spec.get('name') or spec.get('type')))
            check((spec.get('name') or spec.get('type')) in metadata.get('local_skills', {}),
                  f'{path}: local skill is absent from the registered task skill interface')
            check(spec.get('beam_assisted') is True and spec.get('use_arm_ik_controller') is False
                  and spec.get('limit_command_to_measured_state') is True
                  and spec.get('guard_ik_branch_jump') is True,
                  f'{path}: local skill bypasses the explicit measured native-joint command path')
            target = spec.get('target_object_target', '')
            if '_insert_' in target:
                check(spec.get('target_object_servo_position_command_gate_overdrive') is False,
                      f'{path}: loaded insertion must not accumulate commands past its waypoint')
            if target.endswith('_assembled'):
                name = spec.get('object')
                captures.setdefault(name, []).append(index)
                if name in BEAM_OBJECTS:
                    expected_name = 'part_' + name.rsplit('_', 1)[-1] + '_assembled'
                    check(target == expected_name, f'{path}: capture references another part target')
                check(spec.get('target_object_offset_frame') == 'world'
                      and _vector(spec.get('target_object_offset'), 3)
                      and all(abs(float(a)-b) <= 1e-12 for a, b in
                              zip(spec['target_object_offset'], [0., 0., .003])),
                      f'{path}: capture must be original assembled target plus world-Z 3 mm')
                check(spec.get('target_object_position') is None,
                      f'{path}: absolute object target would override randomized named target')
                check(all(spec.get(key) == .001 for key in (
                          'position_tolerance', 'relaxed_position_tolerance',
                          'target_object_position_tolerance', 'relaxed_target_object_position_tolerance'))
                      and all(spec.get(key) == .003 for key in (
                          'orientation_tolerance', 'target_object_orientation_tolerance'))
                      and 'target_object_axial_position_tolerance' not in spec
                      and 'target_object_lateral_position_tolerance' not in spec
                      and spec.get('target_object_servo_position_command_gate_overdrive') is False,
                      f'{path}: capture must preserve strict 1 mm/3 mrad full-pose gates and prohibit overdrive')
        for advance in skill_conditions(phase.get('advance', {})):
            explicit = advance.get('skill') or advance.get('skill_name') or advance.get('name')
            robot = advance.get('robot') or advance.get('robot_name')
            required = ([{'robot': robot, 'name': explicit}] if explicit is not None
                        else advance.get('skills'))
            if required is None:
                check(bool(outlets), f'{path}: skill completion has no local skill outlet')
                continue
            for entry in _entries(required):
                expected_robot = robot if isinstance(entry, str) else entry.get('robot', entry.get('robot_name', robot))
                expected_name = entry if isinstance(entry, str) else entry.get('name', entry.get('type'))
                check(any(name == expected_name and (expected_robot is None or owner == expected_robot)
                          for owner, name in outlets),
                      f'{path}: completion skill/robot does not match this phase local skill outlet')
        for attach in _entries(phase.get('attach')):
            name = attach.get('object')
            attachments.setdefault(name, []).append(index)
            check(attach.get('attachment_mode') == 'fixed_joint'
                  and attach.get('attachment_relative_pose_source') == 'current'
                  and attach.get('beam_assisted_visual_pinch') is True,
                  f'{path}: grasp must capture a qualified current measured pose')
        for lock in _entries(phase.get('lock')):
            if str(lock.get('target', '')).endswith('_assembled'):
                installations.setdefault(lock.get('object'), []).append(index)

    for name in sorted(BEAM_OBJECTS):
        a, c, i = attachments.get(name, []), captures.get(name, []), installations.get(name, [])
        check(len(a) == len(c) == len(i) == 1 and a[0] < c[0] < i[0] < len(phases)-1,
              f'{name}: expected one ordered grasp/capture/install before final verification')
    final = phases[-1]
    check(set(final.get('beam_assisted_installed_parts', [])) == BEAM_OBJECTS,
          'final installed-part membership must contain exactly five Beam parts')
    check(final.get('advance', {}).get('physics_stable_steps') == 240,
          'final verification must require 240 native physics samples')
    final_advance = final.get('advance', {})
    check(final_advance.get('type') == 'all_of', 'final verification must combine all conditions')
    for kind in ('object_targets_reached', 'objects_static', 'objects_detached'):
        matches = [c for c in final_advance.get('conditions', []) if c.get('type') == kind]
        members = {entry.get('object') if isinstance(entry, dict) else entry
                   for c in matches for entry in c.get('objects', [])}
        check(len(matches) == 1 and members == BEAM_OBJECTS,
              f'final {kind} must cover exactly all five Beam parts')
        if kind == 'object_targets_reached':
            for entry in (e for c in matches for e in c.get('objects', [])):
                check(entry.get('position_tolerance') == .003
                      and entry.get('orientation_tolerance') == .04,
                      'final object pose gates must retain 3 mm/40 mrad tolerances')
    check(set(_get(config, 'robot_names', [])) <= set(final.get('gripper_commands', {}))
          and all(v == 'open' for v in final.get('gripper_commands', {}).values()),
          'final verification must command both grippers open')
    home = final.get('beam_assisted_home_targets')
    return_phases = [p for p in phases[:-1] if p.get('beam_assisted_empty_return_guard')]
    if home is not None or return_phases:
        try:
            robot_configs = _home_robot_configs(config)
        except (TypeError, ValueError) as exc:
            robot_configs = {}
            check(False, 'final home robot configuration aliases are invalid: ' + str(exc))
        check(isinstance(home, dict) and set(home) == robots,
              'final home targets must cover both configured robots')
        check(final.get('beam_assisted_empty_return_guard') is True,
              'final home verification lacks final-command geometry protection')
        required_returns = ['beam_assisted_return_home_franka_left',
                            'beam_assisted_return_clearance_franka_right',
                            'beam_assisted_return_home_franka_right']
        check([p['name'] for p in return_phases] == required_returns,
              'safe final return must be left home then right lift then right home')
        if all(name in names for name in required_returns) and 'base_6_retreat' in names:
            check(names.index('base_6_retreat') < names.index(required_returns[0])
                  < names.index(required_returns[1]) < names.index(required_returns[2])
                  < len(phases)-1, 'safe final return must follow the original open-hand retreat')
        for name, goal in (home.items() if isinstance(home, dict) else []):
            robot = robot_configs.get(name)
            controllers = _get(robot, 'controllers', []) if robot is not None else []
            controller = next((c for c in controllers if _get(c, 'name') == 'arm_joint_controller'), None)
            joints = list(_get(controller, 'joint_names', []) or [])
            initial = _get(robot, 'initial_joint_positions', {}) or {}
            valid = (isinstance(goal, dict) and len(joints) == 6 and len(set(joints)) == 6
                     and all(joint in initial for joint in joints)
                     and goal.get('joint_names') == joints
                     and _vector(goal.get('joint_positions', []), 6)
                     and goal.get('joint_positions') == [initial[joint] for joint in joints]
                     and goal.get('tolerance_rad') == .01)
            check(valid, f'{name}: final home target differs from configured initial native branch')
            phase = next((p for p in return_phases
                          if p['name'] == 'beam_assisted_return_home_' + name), None)
            check(phase is not None and valid
                  and phase.get('local_skill', {}).get('joint_positions') == goal.get('joint_positions'),
                  f'{name}: return action does not target initial joints')
        for phase in return_phases:
            skill = phase.get('local_skill', {})
            check(skill.get('name') == 'move_arm_to_joint_positions'
                  and skill.get('robot') in robots
                  and skill.get('beam_assisted_native_joint_target') is True
                  and skill.get('unwrap_revolute_joints') is False
                  and skill.get('requires_held_object') is False
                  and skill.get('joint_target_stable_steps') == 8
                  and skill.get('joint_position_tolerance') == .01
                  and 0 < skill.get('max_joint_step', 0) <= .01
                  and 0 < skill.get('max_command_tracking_error', 0) <= .03,
                  phase['name'] + ': unsafe final return controller contract')
            check(set(phase.get('gripper_commands', {})) == robots
                  and all(v == 'open' for v in phase.get('gripper_commands', {}).values())
                  and not any(phase.get(k) for k in ('attach', 'detach', 'lock', 'fixture_lock'))
                  and set(phase.get('beam_assisted_installed_parts', [])) == BEAM_OBJECTS,
                  phase['name'] + ': final return must remain empty, open and protect all installed parts')
            delta = skill.get('beam_assisted_joint_target_delta')
            if phase['name'] == 'beam_assisted_return_clearance_franka_right':
                right = (home or {}).get('franka_right', {})
                joints = right.get('joint_names', [])
                expected = [(-.24 if joint == 'shoulder_lift_joint' else 0.) for joint in joints]
                check(len(joints) == 6 and delta == expected,
                      'right clearance must preserve measured entry joints and lift only the shoulder')
            else:
                check(delta is None, 'home phase cannot replace initial joint target with an offset')
    criteria = _get(config, 'success_criteria', [])
    check(len(criteria) == 5 and {c.get('object') for c in criteria} == BEAM_OBJECTS,
          'success criteria must contain exactly five Beam parts')
    inspect(criteria, 'success_criteria')
    for criterion in criteria:
        name = criterion.get('object')
        if name in BEAM_OBJECTS:
            check(criterion.get('target') == 'part_' + name.rsplit('_', 1)[-1] + '_assembled'
                  and criterion.get('position_tolerance') == .003
                  and criterion.get('orientation_tolerance') == .04
                  and criterion.get('require_released') is True
                  and criterion.get('require_static') is True,
                  f'{name}: final success pose/release/static contract changed')
    budget = _get(config_or_task, 'max_steps', _get(config, 'max_steps', 0))
    check(isinstance(budget, int) and budget > 0, 'a positive effective task budget is required')
    if full_mode:
        check(isinstance(budget, int) and budget >= timeout_sum + 1000,
              f'full-run max_steps={budget} is below all-phase budget {timeout_sum+1000}')

    dr = _get(config, 'domain_randomization', {})
    resolved = []
    for name in sorted(BEAM_OBJECTS):
        target_name = 'part_' + name.rsplit('_', 1)[-1] + '_assembled'
        target = targets.get(target_name, {})
        check(_vector(target.get('position'), 3) and _vector(target.get('orientation'), 4),
              f'{target_name}: resolved world pose is missing or nonfinite')
        if dr.get('enabled'):
            members = [g for g in dr.get('groups', {}).values() if target_name in g.get('targets', [])]
            check(len(members) == 1, f'{target_name}: randomized assembly target must have one layout group')
        if _vector(target.get('position'), 3):
            resolved.append({'object': name, 'target': target_name,
                'final_position': [float(v) for v in target['position']],
                'capture_position': [float(v)+d for v, d in zip(target['position'], [0.,0.,.003])]})

    if runtime:
        check(getattr(config_or_task, '_beam_assisted_enabled', False) is True,
              'runtime task did not activate the assisted route')
        for method in ('_assisted_idle_action', '_assisted_guard_actions'):
            check(callable(getattr(config_or_task, method, None)), f'runtime lacks {method} command outlet')
        bindings = getattr(config_or_task, '_object_prims', {})
        check(BEAM_OBJECTS <= set(bindings), 'runtime native object bindings are incomplete')
        for name in robots:
            robot = getattr(config_or_task, 'robots', {}).get(name)
            controllers = getattr(robot, 'controllers', {})
            check({'arm_joint_controller', 'gripper_controller'} <= set(controllers),
                  f'{name}: native joint/gripper controllers are not registered')

    if not errors:
        from types import SimpleNamespace
        from .plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        resolver_task = SimpleNamespace(target_poses=targets)
        for row in resolved:
            spec = phases[captures[row['object']][0]]['local_skill']
            actual = adapter._target_object_pose(task=resolver_task, spec=spec)
            check(actual is not None and all(abs(float(a)-b) <= 1e-12
                  for a, b in zip(actual['position'], row['capture_position'])),
                  f"{row['target']}: actual skill resolver disagrees with runtime 3 mm capture")

    config_signature = None
    phase_target_signature = None
    try:
        config_signature = configuration_signature(config)
        phase_target_signature = _signature({'phase_specs': phases, 'target_poses': targets})
    except (TypeError, ValueError) as exc:
        check(False, 'configuration signature could not be generated: ' + str(exc))
    report = {'passed': not errors, 'errors': errors, 'scope': 'resolved configuration and command interfaces',
        'configuration_signature': config_signature,
        'configuration_signature_schema': 'entire_task_model/v2',
        'configuration_signature_excluded_fields': [],
        'phase_target_signature': phase_target_signature,
        'runtime_phase_target_values_match_config': None if not runtime else not any(
            'runtime phase_specs differ' in error or 'runtime target_poses differ' in error for error in errors),
        'full_mode': bool(full_mode),
        'runtime_interfaces_checked': runtime, 'phase_count': len(phases), 'object_count': len(objects),
        'target_count': len(targets), 'phase_timeout_sum': timeout_sum, 'full_run_max_steps': budget,
        'final_home_required': home is not None, 'empty_return_phase_count': len(return_phases),
        'all_phase_budget_required': timeout_sum + 1000, 'full_run_budget_checked': bool(full_mode),
        'capture_targets': resolved, 'formal_physical_success': False,
        'not_certified': ['native constraint/contact dynamics', 'actual published or executed trajectory',
                          'full visible-mesh sweep and between-sample motion', 'rendered camera visibility',
                          'five-part assembly success']}
    if errors:
        raise AssistedConfigurationError(report)
    return report
