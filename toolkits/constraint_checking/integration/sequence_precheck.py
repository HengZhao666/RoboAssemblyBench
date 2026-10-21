"""Static, stateful precheck for a complete RoboAssemblyBench phase sequence."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Iterable


def _as_list(value) -> list:
    if value is None:
        return []
    return value if isinstance(value, list) else [value]


@dataclass
class SequencePrecheckReport:
    feasible: bool
    phase_count: int
    actions: list[dict] = field(default_factory=list)
    errors: list[dict] = field(default_factory=list)
    warnings: list[dict] = field(default_factory=list)
    final_holding: dict[str, str | None] = field(default_factory=dict)
    check_seconds: float = 0.0

    def to_dict(self) -> dict:
        return {
            'enabled': True,
            'mode': 'passive',
            'feasible': bool(self.feasible),
            'phase_count': int(self.phase_count),
            'normalized_action_count': len(self.actions),
            'actions': list(self.actions),
            'errors': list(self.errors),
            'warnings': list(self.warnings),
            'final_holding': dict(self.final_holding),
            'check_seconds': float(self.check_seconds),
        }


class AssemblySequencePrechecker:
    """Symbolically execute phase ownership changes without running simulation."""

    def check(
        self,
        *,
        phases: Iterable[dict],
        robot_names: Iterable[str] = (),
        object_names: Iterable[str] = (),
    ) -> SequencePrecheckReport:
        started = time.perf_counter()
        phases = [phase for phase in phases if isinstance(phase, dict)]
        known_robots = {str(name) for name in robot_names}
        known_objects = {str(name) for name in object_names}
        holding: dict[str, str | None] = {name: None for name in known_robots}
        # Keep each hand's permission: runtime sharing requires both existing
        # and incoming grasps to explicitly permit pure physical support.
        held_by: dict[str, dict[str, bool]] = {}
        actions: list[dict] = []
        errors: list[dict] = []
        warnings: list[dict] = []

        for index, phase in enumerate(phases):
            phase_name = str(phase.get('name') or f'phase_{index}')
            for fixture_lock in _as_list(phase.get('fixture_lock')):
                if not isinstance(fixture_lock, dict):
                    continue
                action = {
                    'phase_index': index,
                    'phase': phase_name,
                    'kind': 'fixture_lock',
                    'robot': None,
                    'object': self._text(fixture_lock.get('object') or fixture_lock.get('name')),
                }
                actions.append(action)
                self._validate_reference(action, known_robots, known_objects, errors)
            local_skills = self._local_skills(phase)
            for skill in local_skills:
                robot = self._text(skill.get('robot'))
                object_name = self._text(skill.get('object'))
                action = {
                    'phase_index': index,
                    'phase': phase_name,
                    'kind': 'skill',
                    'skill': self._text(skill.get('name')),
                    'robot': robot,
                    'object': object_name,
                }
                actions.append(action)
                self._validate_reference(action, known_robots, known_objects, errors)
                if robot and object_name:
                    owners = held_by.get(object_name, {})
                    skill_name = str(skill.get('name') or '').lower()
                    requires_payload = bool(skill.get('requires_held_object',
                        'move_part' in skill_name or 'hold_part' in skill_name))
                    shared_approach = (not requires_payload and owners and all(owners.values())
                        and self._shared_physical(self._next_attach(phases[index:], robot, object_name)))
                    if owners and robot not in owners and not shared_approach:
                        errors.append(
                            self._issue(
                                action,
                                'object_owned_by_other_robot',
                                f'{object_name} is held by {", ".join(owners)}, but {robot} is scheduled to manipulate it.',
                            )
                        )
                    if requires_payload and robot not in owners:
                        errors.append(
                            self._issue(
                                action,
                                'payload_not_held',
                                f'{robot} is scheduled to carry {object_name} before attaching it.',
                            )
                        )

            for attach in _as_list(phase.get('attach')):
                if not isinstance(attach, dict):
                    continue
                robot = self._text(attach.get('robot'))
                object_name = self._text(attach.get('object'))
                action = {
                    'phase_index': index,
                    'phase': phase_name,
                    'kind': 'attach',
                    'robot': robot,
                    'object': object_name,
                }
                actions.append(action)
                self._validate_reference(action, known_robots, known_objects, errors)
                if not robot or not object_name:
                    errors.append(self._issue(action, 'incomplete_attach', 'Attach requires robot and object.'))
                    continue
                current_payload = holding.get(robot)
                current_owners = held_by.get(object_name, {})
                if current_payload and current_payload != object_name:
                    errors.append(
                        self._issue(
                            action,
                            'end_effector_occupied',
                            f'{robot} already holds {current_payload} and cannot also attach {object_name}.',
                        )
                    )
                    continue
                permits_shared = self._shared_physical(attach)
                if (current_owners and robot not in current_owners
                        and not (permits_shared and all(current_owners.values()))):
                    errors.append(
                        self._issue(
                            action,
                            'double_grasp',
                            f'{object_name} is already held by {", ".join(current_owners)}; {robot} cannot attach it.',
                        )
                    )
                    continue
                holding[robot] = object_name
                held_by.setdefault(object_name, {})[robot] = permits_shared

            for detach in _as_list(phase.get('detach')):
                object_name = self._text(detach.get('object')) if isinstance(detach, dict) else self._text(detach)
                robot = self._text(detach.get('robot')) if isinstance(detach, dict) else None
                self._release(index, phase_name, object_name, robot, holding, held_by, actions, errors, warnings)

            for lock in _as_list(phase.get('lock')):
                if not isinstance(lock, dict):
                    continue
                object_name = self._text(lock.get('object'))
                # A free-object snap is fixture initialization/stabilization,
                # not a release. Keep it in the normalized trace without
                # requiring a preceding attachment.
                if lock.get('snap_free_object') and object_name not in held_by:
                    actions.append(
                        {
                            'phase_index': index,
                            'phase': phase_name,
                            'kind': 'fixture_lock',
                            'robot': None,
                            'object': object_name,
                        }
                    )
                    self._validate_reference(
                        actions[-1],
                        known_robots,
                        known_objects,
                        errors,
                    )
                    continue
                self._release(
                    index,
                    phase_name,
                    object_name,
                    None,
                    holding,
                    held_by,
                    actions,
                    errors,
                    warnings,
                    kind='place',
                )

            for robot, command in (phase.get('gripper_commands') or {}).items():
                if str(command).lower() != 'open':
                    continue
                payload = holding.get(str(robot))
                if payload:
                    self._release(index, phase_name, payload, str(robot), holding,
                                  held_by, actions, errors, warnings, kind='release')

        return SequencePrecheckReport(
            feasible=not errors,
            phase_count=len(phases),
            actions=actions,
            errors=errors,
            warnings=warnings,
            final_holding=holding,
            check_seconds=max(time.perf_counter() - started, 0.0),
        )

    @staticmethod
    def _shared_physical(attach: dict | None) -> bool:
        return bool(isinstance(attach, dict) and attach.get('allow_shared_physical_grasp')
                    and str(attach.get('attachment_mode', 'fixed_joint')).lower()
                    in {'pure_physical_grasp', 'contact_pure_physical_grasp'})

    @staticmethod
    def _next_attach(phases, robot, object_name):
        """Allow a free-hand approach only to its next declared shared grasp.

        Future permission never supplies a payload for a carry/hold skill.
        An earlier incompatible grasp cannot borrow a later permission.
        """
        for phase in phases:
            for attach in _as_list(phase.get('attach')):
                if (isinstance(attach, dict) and attach.get('robot') == robot
                        and attach.get('object') == object_name):
                    return attach
        return None

    @staticmethod
    def _local_skills(phase: dict) -> list[dict]:
        result = []
        singular = phase.get('local_skill')
        if isinstance(singular, dict):
            result.append(singular)
        plural = phase.get('local_skills')
        if isinstance(plural, dict):
            result.extend(item for item in plural.values() if isinstance(item, dict))
        elif isinstance(plural, list):
            result.extend(item for item in plural if isinstance(item, dict))
        return result

    @staticmethod
    def _validate_reference(action, known_robots, known_objects, errors) -> None:
        robot = action.get('robot')
        object_name = action.get('object')
        if robot and known_robots and robot not in known_robots:
            errors.append(AssemblySequencePrechecker._issue(action, 'unknown_robot', f'Unknown robot: {robot}.'))
        if object_name and known_objects and object_name not in known_objects:
            errors.append(
                AssemblySequencePrechecker._issue(action, 'unknown_object', f'Unknown object: {object_name}.')
            )

    @staticmethod
    def _release(
        index,
        phase_name,
        object_name,
        requested_robot,
        holding,
        held_by,
        actions,
        errors,
        warnings,
        *,
        kind='detach',
    ) -> None:
        action = {
            'phase_index': index,
            'phase': phase_name,
            'kind': kind,
            'robot': requested_robot,
            'object': object_name,
        }
        actions.append(action)
        if not object_name:
            errors.append(AssemblySequencePrechecker._issue(action, 'incomplete_release', 'Release requires object.'))
            return
        owners = held_by.get(object_name, {})
        if requested_robot and owners and requested_robot not in owners:
            errors.append(
                AssemblySequencePrechecker._issue(
                    action,
                    'release_by_non_owner',
                    f'{requested_robot} cannot release {object_name}, which is held by {", ".join(owners)}.',
                )
            )
            return
        if not owners:
            warnings.append(
                AssemblySequencePrechecker._issue(
                    action,
                    'release_without_attach',
                    f'{object_name} is placed or detached without a preceding attach.',
                )
            )
            return
        # Unqualified detach selects the primary hand, as the runtime does.
        owner = requested_robot or next(iter(owners))
        holding[owner] = None
        owners.pop(owner)
        if not owners:
            held_by.pop(object_name, None)
        action['robot'] = owner

    @staticmethod
    def _issue(action: dict, code: str, message: str) -> dict:
        return {
            'phase_index': int(action['phase_index']),
            'phase': str(action['phase']),
            'code': str(code),
            'message': str(message),
        }

    @staticmethod
    def _text(value: Any) -> str | None:
        return None if value is None else str(value)
