"""Passive precheck of commanded UR5e joint segments using Lula FK."""

from __future__ import annotations

import time
from collections import Counter

import numpy as np

from .models import get_robot_collision_model, infer_task_robot_collision_model
from .runtime_monitor import RuntimeConstraintConfig, RuntimeConstraintMonitor


class _StaticXform:
    def __init__(self, position):
        self.position = np.asarray(position, dtype=float)

    def get_world_pose(self):
        return self.position, np.asarray([1.0, 0.0, 0.0, 0.0])


class StageTrajectoryPrechecker:
    """Check actual policy joint commands without changing or rejecting them."""

    def __init__(
        self,
        *,
        check_stride: int = 64,
        num_waypoints: int = 8,
        threshold: float | None = None,
        include_ground: bool = False,
        ignore_pairs=(),
        robot_model: str | None = None,
    ):
        self.check_stride = max(int(check_stride), 1)
        self.num_waypoints = max(int(num_waypoints), 2)
        self._threshold = threshold
        self._include_ground = bool(include_ground)
        self._requested_robot_model = robot_model
        self._model_selected = robot_model is not None
        self.model = None
        self.monitor = None
        self.detector = None
        self._observations = 0
        self._checks = 0
        self._segments = 0
        self._waypoints = 0
        self._violations = []
        self._errors = []
        self._reasons = Counter()
        self._total_seconds = 0.0
        self._ignore_pairs = list(ignore_pairs)
        self._frame_coverage = {}
        self._entity_segments = []
        self._entity_errors = []
        self._configure_model(get_robot_collision_model(robot_model))

    def observe(self, task, actions: dict) -> dict:
        step = int(getattr(task, 'step_counter', 0))
        self._observations += 1
        if step % self.check_stride:
            return {'checked': False, 'step': step, 'reason': 'stride_skip'}
        started = time.perf_counter()
        self._checks += 1
        try:
            self._select_task_model(task)
            if self.detector is None:
                return self._result(step, 'detector_unavailable')
            self.monitor.refresh_environment_from_task(task)
            paths = {}
            # Holding arms occupy space even when there is no new action.
            for robot_name in (getattr(task, 'robots', {}) or {}):
                robot_actions = (actions or {}).get(robot_name, {})
                path, reason = self._joint_path(task, str(robot_name), robot_actions)
                if path is None:
                    self._reasons[reason] += 1
                    continue
                paths[str(robot_name)] = path
                self._segments += 1
            if not paths:
                return self._result(step, 'no_joint_targets')

            self.monitor.initialize_entity_geometry(task)
            entity_monitor = self.monitor._entity_geometry
            current_entities = entity_monitor.current_poses() if entity_monitor is not None else None

            sample_count = max(len(path) for path in paths.values())
            events = []
            for waypoint_index in range(sample_count):
                link_positions_by_robot = {}
                for robot_name, path in paths.items():
                    q = path[min(waypoint_index, len(path) - 1)]
                    positions = self._forward_link_positions(task, robot_name, q)
                    if positions:
                        link_positions_by_robot[robot_name] = positions
                        events.extend(
                            self.detector.check_config(
                                positions,
                                step=step,
                                agent_name=f'{robot_name}@waypoint_{waypoint_index}',
                            )
                        )
                events.extend(self._check_inter_robot(link_positions_by_robot, step, waypoint_index))
                if current_entities is not None:
                    try:
                        poses = self._swept_entity_poses(task, paths, waypoint_index, current_entities)
                        entity_result = entity_monitor.checker.check(poses, step=step,
                            policy=self.monitor._contact_policy, task=task)
                        if len(self._entity_segments) < 5000:
                            self._entity_segments.append({'waypoint_index': waypoint_index, **entity_result})
                    except Exception as exc:
                        self._entity_errors.append(self._error('entity_segment', exc, step))
                self._waypoints += 1
            serialized = [self.monitor._event_to_dict(event) for event in events]
            serialized = [
                event
                for event in serialized
                if not any(rule.matches(event['entity_a'], event['entity_b']) for rule in self._ignore_pairs)
            ]
            self._violations.extend(serialized)
            return {
                'checked': True,
                'step': step,
                'reason': 'ok',
                'segments': len(paths),
                'violations': serialized,
            }
        except Exception as exc:
            error = self._error('observe', exc, step)
            self._record_error(error)
            return {'checked': True, 'step': step, 'reason': 'error', 'monitor_error': error}
        finally:
            self._total_seconds += max(time.perf_counter() - started, 0.0)

    def finalize(self) -> dict:
        minimum_distance = min(
            (float(event['distance']) for event in self._violations),
            default=None,
        )
        return {
            'enabled': True,
            'mode': 'passive',
            'robot_model': self.model.name,
            'check_stride': self.check_stride,
            'num_waypoints': self.num_waypoints,
            'observed_steps': self._observations,
            'checks': self._checks,
            'segments_checked': self._segments,
            'waypoints_checked': self._waypoints,
            'violation_total': len(self._violations),
            'violations_by_kind': dict(Counter(event['kind'] for event in self._violations)),
            'minimum_distance': minimum_distance,
            'events': list(self._violations),
            'skip_reasons': dict(self._reasons),
            'monitor_error': list(self._errors),
            'total_check_seconds': self._total_seconds,
            'frame_coverage': dict(self._frame_coverage),
            'geometry_mode': 'link_origin_capsules_only',
            'full_entity_sweep_verified': False,
            'entity_geometry_coverage': self.monitor._entity_geometry.finalize() if self.monitor._entity_geometry is not None else None,
            'entity_geometry_segments': list(self._entity_segments),
            'entity_geometry_errors': list(self._entity_errors),
            'entity_geometry_initialize_errors': list(self.monitor._monitor_errors),
            'gripper_prediction': 'current_articulation_frozen_during_arm_segment; gripper action sweep not modeled',
        }

    def _select_task_model(self, task) -> None:
        if self._model_selected:
            return
        self._configure_model(infer_task_robot_collision_model(task))
        self._model_selected = True

    def _configure_model(self, model) -> None:
        self.model = model
        self.monitor = RuntimeConstraintMonitor(
            RuntimeConstraintConfig(
                robot_model=model.name,
                threshold=self._threshold,
                include_ground=self._include_ground,
                ignore_pairs=list(self._ignore_pairs),
            )
        )
        self.detector = self.monitor.detector

    def _joint_path(self, task, robot_name: str, robot_actions):
        if not isinstance(robot_actions, dict):
            return None, 'invalid_robot_action'
        robot = (getattr(task, 'robots', {}) or {}).get(robot_name)
        if robot is None:
            return None, 'missing_robot'
        joint_controller = self._controller(robot, 'joint')
        ik_controller = self._controller(robot, 'ik')
        if joint_controller is None or ik_controller is None:
            return None, 'missing_controller'
        target = self._joint_target(robot_actions, joint_controller)
        subset = joint_controller.get_joint_subset()
        current = np.asarray(subset.get_joint_positions(), dtype=float).reshape(-1)
        if target is None:
            target = current
            self._reasons['holding_robot_current_joints'] += 1
        target = np.asarray(target, dtype=float).reshape(-1)
        if current.shape != target.shape or not np.all(np.isfinite(target)) or not np.all(np.isfinite(current)):
            return None, 'invalid_joint_target'
        return [current + (target - current) * fraction for fraction in np.linspace(0.0, 1.0, self.num_waypoints)], 'ok'

    @staticmethod
    def _controller(robot, kind: str):
        controllers = getattr(robot, 'controllers', {}) or {}
        try:
            from internutopia_extension.configs.robots.franka import (
                arm_ik_cfg,
                arm_joint_cfg,
            )

            configured_name = arm_joint_cfg.name if kind == 'joint' else arm_ik_cfg.name
            if configured_name in controllers:
                return controllers[configured_name]
        except Exception:
            pass
        for name, controller in controllers.items():
            lowered = str(name).lower()
            if kind == 'joint' and 'joint' in lowered and 'gripper' not in lowered:
                return controller
            if kind == 'ik' and ('ik' in lowered or 'inverse' in lowered):
                return controller
        return None

    @staticmethod
    def _joint_target(robot_actions: dict, joint_controller):
        configured_name = getattr(getattr(joint_controller, 'config', None), 'name', None)
        candidates = []
        if configured_name:
            candidates.append(configured_name)
        candidates.extend(name for name in robot_actions if 'joint' in str(name).lower())
        for name in candidates:
            value = robot_actions.get(name)
            if value is None:
                continue
            array = np.asarray(value, dtype=float)
            if array.ndim >= 2:
                array = array[0]
            return array
        return None

    def _forward_link_positions(self, task, robot_name: str, q: np.ndarray) -> dict:
        robot = task.robots[robot_name]
        controller = self._controller(robot, 'ik')
        solver = getattr(controller, '_kinematics_solver')
        lula_solver = getattr(solver, '_kinematics', solver)
        base_pose = controller.get_ik_base_world_pose()
        solver.set_robot_base_pose(
            robot_position=np.asarray(base_pose[0], dtype=float) / controller._robot_scale,
            robot_orientation=np.asarray(base_pose[1], dtype=float),
        )
        available = set(lula_solver.get_all_frame_names())
        result = {}
        mapping = {}
        missing = []
        for link_name in self.model.link_names:
            # Mounted gripper base_link must never alias the arm base_link.
            # Mounted frames require exact full-path names from the solver.
            frame_name = link_name
            if frame_name not in available:
                missing.append(link_name)
                continue
            position, _ = lula_solver.compute_forward_kinematics(frame_name, joint_positions=q)
            result[link_name] = np.asarray(position, dtype=float) * np.asarray(controller._robot_scale, dtype=float)
            mapping[link_name] = frame_name
        self._frame_coverage[robot_name] = {
            'available_solver_frames': sorted(available),
            'unique_mapping': mapping,
            'missing_model_frames': missing,
            'gripper_articulation_swept': not any('Gripper/' in name for name in missing),
        }
        return result

    def _swept_entity_poses(self, task, paths, index, current_entities):
        poses = dict(current_entities)
        mount_deltas = {}
        for robot_name, path in paths.items():
            robot = task.robots[robot_name]
            controller = self._controller(robot, 'ik')
            solver = controller._kinematics_solver
            lula = getattr(solver, '_kinematics', solver)
            available = set(lula.get_all_frame_names())
            current_q = np.asarray(self._controller(robot, 'joint').get_joint_subset().get_joint_positions(), dtype=float).reshape(-1)
            predicted_q = path[min(index, len(path)-1)]
            def frame_matrix(name, q):
                position, rotation = lula.compute_forward_kinematics(name, joint_positions=q)
                matrix = np.eye(4); matrix[:3,:3] = rotation
                matrix[:3,3] = np.asarray(position) * np.asarray(controller._robot_scale, dtype=float)
                return matrix
            mount = 'wrist_3_link' if self.model.name == 'ur5e_robotiq_2f85' else 'panda_hand'
            if mount not in available: raise ValueError(f'Missing mount frame {robot_name}/{mount}')
            delta = frame_matrix(mount, predicted_q) @ np.linalg.inv(frame_matrix(mount, current_q))
            mount_deltas[robot_name] = delta
            for body, (owner, link) in self.monitor._entity_geometry.body_robot_links.items():
                if owner != robot_name: continue
                if link in available:
                    link_delta = frame_matrix(link, predicted_q) @ np.linalg.inv(frame_matrix(link, current_q))
                elif link.startswith('Gripper/'):
                    link_delta = delta
                else:
                    raise ValueError(f'Missing unique body frame {robot_name}/{link}')
                poses[body] = link_delta @ current_entities[body]
        holders = dict(self.monitor._contact_policy.holder_pairs_from_task(task))
        for name, prim in (getattr(task, '_object_prims', {}) or {}).items():
            holder = holders.get(name)
            if isinstance(holder, str) and holder in mount_deltas:
                body = str(prim.GetPath())
                poses[body] = mount_deltas[holder] @ current_entities[body]
        return poses

    def _check_inter_robot(self, positions_by_robot, step, waypoint_index):
        if len(positions_by_robot) < 2:
            return []
        self.detector._agents.clear()
        for robot_name, positions in positions_by_robot.items():
            self.detector.add_agent(
                f'{robot_name}@waypoint_{waypoint_index}',
                {name: _StaticXform(position) for name, position in positions.items()},
            )
        return self.detector.check_inter_agent(step=step)

    def _result(self, step, reason):
        self._reasons[reason] += 1
        return {'checked': True, 'step': step, 'reason': reason, 'violations': []}

    def _record_error(self, error: dict) -> None:
        key = (error['stage'], error['type'], error['message'])
        for existing in self._errors:
            existing_key = (existing['stage'], existing['type'], existing['message'])
            if existing_key == key:
                existing['last_step'] = error['step']
                existing['occurrences'] = int(existing.get('occurrences', 1)) + 1
                return
        error['last_step'] = error['step']
        error['occurrences'] = 1
        self._errors.append(error)

    @staticmethod
    def _error(stage, exc, step):
        return {
            'stage': stage,
            'step': int(step),
            'type': type(exc).__name__,
            'message': str(exc),
        }
