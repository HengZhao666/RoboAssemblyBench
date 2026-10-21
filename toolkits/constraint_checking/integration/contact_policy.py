"""Phase-aware classification for runtime collision candidates."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable

_CONTACT_PHASE_TOKENS = (
    'descend',
    'grasp',
    'close_gripper',
    'close_and_attach',
    'insert',
    'release',
    'settle',
)

_END_EFFECTOR_TOKENS = (
    'Gripper/Robotiq_2F_85/',
    'panda_hand',
    'panda_leftfinger',
    'panda_rightfinger',
)


@dataclass(frozen=True)
class ContactContext:
    """Assembly state needed to decide whether contact is intentional."""

    phase: str | None
    robot: str | None
    object_name: str | None
    contact_intended: bool


@dataclass(frozen=True)
class ContactDecision:
    """Classification attached to one detector candidate."""

    classification: str
    reason: str
    phase: str | None
    active_robot: str | None
    active_object: str | None


@dataclass(frozen=True)
class ContactSnapshot:
    context: ContactContext
    holders: tuple[tuple[str, str], ...]


class AssemblyContactPolicy:
    """Separate expected assembly contact, proximity, and collision."""

    def __init__(self, *, collision_threshold: float = 0.0, ignore_pairs: Iterable[Any] = ()):
        self.collision_threshold = float(collision_threshold)
        self.ignore_pairs = tuple(ignore_pairs)

    def classify(self, event, task, *, snapshot: ContactSnapshot | None = None) -> ContactDecision:
        snapshot = snapshot or self.snapshot_from_task(task)
        context = snapshot.context
        if any(rule.matches(event.entity_a, event.entity_b) for rule in self.ignore_pairs):
            return self._decision('allowed_contact', 'configured_ignore_pair', context)
        if self._is_expected_assembly_contact(event, context) or self._is_held_finger_contact(event, snapshot.holders):
            return self._decision('allowed_contact', 'phase_target_end_effector_contact', context)
        if float(event.distance) > self.collision_threshold:
            return self._decision('proximity', 'positive_surface_clearance', context)
        return self._decision('collision', 'capsule_surface_overlap', context)

    @classmethod
    def snapshot_from_task(cls, task, *, states=None) -> ContactSnapshot:
        return ContactSnapshot(cls.context_from_task(task), cls.holder_pairs_from_task(task, states=states))

    @staticmethod
    def holder_pairs_from_task(task, *, states=None) -> tuple[tuple[str, str], ...]:
        # Beam's public state getter calls _physical_hold_valid and mutates
        # grasp diagnostics. Classification must not invoke that machinery.
        attachments = getattr(task, '_attachments', None)
        pairs = set()
        if isinstance(attachments, dict):
            for name, state in attachments.items():
                if isinstance(state, dict) and isinstance(state.get('robot_name'), str):
                    pairs.add((str(name), state['robot_name']))
            for key, state in (getattr(task, '_beam_shared_grasps', {}) or {}).items():
                if isinstance(key, tuple) and len(key)==2:
                    pairs.add((str(key[0]), str(key[1])))
        else:
            if states is None:
                getter = getattr(task, 'get_tracked_object_states', None)
                states = getter() if callable(getter) else {}
            for name, state in (states or {}).items():
                if not isinstance(state, dict): continue
                holder = state.get('grasped_by') or state.get('attached_to')
                if isinstance(holder, str): pairs.add((str(name), holder))
        return tuple(sorted(pairs))

    @staticmethod
    def context_from_task(task) -> ContactContext:
        phase = getattr(task, 'phase', None)
        phase_spec = {}
        getter = getattr(task, 'get_current_phase_spec', None)
        if callable(getter):
            phase_spec = getter() or {}
        if not isinstance(phase_spec, dict):
            phase_spec = {}

        local_skill = phase_spec.get('local_skill')
        if not isinstance(local_skill, dict):
            local_skill = {}
        robot = local_skill.get('robot')
        object_name = local_skill.get('object')

        attach = phase_spec.get('attach')
        attach_items = attach if isinstance(attach, list) else [attach]
        for item in attach_items:
            if isinstance(item, dict):
                robot = robot or item.get('robot')
                object_name = object_name or item.get('object')

        lock = phase_spec.get('lock')
        lock_items = lock if isinstance(lock, list) else [lock]
        for item in lock_items:
            if isinstance(item, dict) and item.get('object'):
                object_name = object_name or item['object']
                break

        phase_text = str(phase or phase_spec.get('name') or '')
        if robot is None:
            if phase_text.startswith('left_'):
                robot = 'franka_left'
            elif phase_text.startswith('right_'):
                robot = 'franka_right'

        contact_intended = any(token in phase_text.lower() for token in _CONTACT_PHASE_TOKENS)
        return ContactContext(
            phase=phase_text or None,
            robot=None if robot is None else str(robot),
            object_name=None if object_name is None else str(object_name),
            contact_intended=contact_intended,
        )

    @staticmethod
    def _is_expected_assembly_contact(event, context: ContactContext) -> bool:
        if not context.contact_intended or not context.robot or not context.object_name:
            return False
        entities = (str(event.entity_a), str(event.entity_b))
        robot_entity = next((item for item in entities if item.startswith(f'{context.robot}/')), None)
        object_entity = next((item for item in entities if item == context.object_name), None)
        if robot_entity is None or object_entity is None:
            return False
        return AssemblyContactPolicy._is_exact_contact_finger(robot_entity, context.robot)

    @staticmethod
    def _is_exact_contact_finger(entity: str, robot: str) -> bool:
        # A capsule connecting the wrist/palm/knuckle to a finger does not
        # identify which body touched. Do not exempt the entire envelope.
        if '->' in entity:
            return False
        relative = entity.removeprefix(f'{robot}/')
        return relative in (
            'Gripper/Robotiq_2F_85/left_inner_finger',
            'Gripper/Robotiq_2F_85/right_inner_finger',
            'panda_leftfinger', 'panda_rightfinger',
        )

    @staticmethod
    def _is_held_finger_contact(event, holders) -> bool:
        for object_name, holder in holders:
            entities = (str(event.entity_a), str(event.entity_b))
            if str(object_name) not in entities:
                continue
            if any(AssemblyContactPolicy._is_exact_contact_finger(entity, holder)
                   for entity in entities if entity.startswith(f'{holder}/')):
                return True
        return False

    @staticmethod
    def _decision(
        classification: str,
        reason: str,
        context: ContactContext,
    ) -> ContactDecision:
        return ContactDecision(
            classification=classification,
            reason=reason,
            phase=context.phase,
            active_robot=context.robot,
            active_object=context.object_name,
        )
