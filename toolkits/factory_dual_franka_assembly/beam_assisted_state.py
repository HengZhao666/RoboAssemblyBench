"""Measured-pose assistance for the explicitly selected Beam route."""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path

import numpy as np


def bounded_native_target(target, actual, previous=None, step=.02, tracking=.12, *, unwrap=True):
    actual = np.asarray(actual, dtype=float).reshape(-1)
    target = np.asarray(target, dtype=float).reshape(-1)
    if target.shape != actual.shape or not np.all(np.isfinite([target, actual])):
        raise ValueError('invalid assisted arm command/state')
    if unwrap:
        target = actual + (target - actual + np.pi) % (2*np.pi) - np.pi
    anchor = actual
    if previous is not None:
        previous = np.asarray(previous, dtype=float).reshape(-1)
        if (previous.shape == actual.shape and np.all(np.isfinite(previous))
                and np.max(np.abs(previous-actual)) <= tracking):
            anchor = previous
    result = anchor + np.clip(target-anchor, -step, step)
    return np.clip(result, actual-tracking, actual+tracking)


def consecutive_native_gate(state, stamp, ready, required, poses=None):
    """Count a predicate once on consecutive, unchanged native physics ticks."""
    valid = (stamp.get('valid') is True and type(stamp.get('epoch')) is int
             and type(stamp.get('index')) is int)
    try:
        valid = bool(valid and stamp['epoch'] >= 0 and stamp['index'] >= 0
                     and math.isfinite(stamp['time']) and stamp['time'] >= 0
                     and math.isfinite(stamp['dt']) and stamp['dt'] > 0)
    except (KeyError, TypeError):
        valid = False
    if not valid:
        state.update(previous=None, count=0, pose_anchor=None)
        return False
    old = state.get('previous')
    same = bool(old and (old['epoch'], old['index']) == (stamp['epoch'], stamp['index']))
    if same and (old['time'] != stamp['time'] or old['dt'] != stamp['dt']):
        state.update(previous=None, count=0, pose_anchor=None)
        return False
    if not ready:
        state.update(previous=dict(stamp), count=0, pose_anchor=None)
        return False
    if not same:
        consecutive = bool(old and old['epoch'] == stamp['epoch']
            and stamp['index'] == old['index'] + 1
            and math.isclose(old['dt'], stamp['dt'], rel_tol=1e-9, abs_tol=1e-12)
            and math.isclose(stamp['time'] - old['time'], stamp['dt'],
                             rel_tol=1e-6, abs_tol=max(1e-10, stamp['dt'] * 1e-6)))
        if not consecutive:
            state['count'] = 0
            state['pose_anchor'] = None
        state['count'] = state.get('count', 0) + 1
        state['previous'] = dict(stamp)
    if poses is not None:
        from .planner_primitives import pose_error
        last_poses = state.get('last_poses')
        if same and last_poses is not None:
            changed = set(last_poses) != set(poses)
            for name, current in poses.items():
                previous = last_poses.get(name)
                if previous is None:
                    changed = True
                    continue
                dp, dq = pose_error(current_position=current['position'],
                    current_orientation=current['orientation'],
                    target_position=previous['position'], target_orientation=previous['orientation'])
                changed = bool(changed or dp > 1e-12 or dq > 1e-12
                    or current['body_path'] != previous['body_path']
                    or current['motion_view_id'] != previous['motion_view_id'])
            if changed:
                state.update(count=0, pose_anchor=None, last_poses=None)
                return False
        state['last_poses'] = copy.deepcopy(poses)
        anchor = state.get('pose_anchor')
        if anchor is not None:
            stable = set(anchor) == set(poses)
            for name, current in poses.items():
                previous = anchor.get(name)
                if previous is None:
                    stable = False
                    continue
                position_error, orientation_error = pose_error(
                    current_position=current['position'], current_orientation=current['orientation'],
                    target_position=previous['position'], target_orientation=previous['orientation'])
                stable = bool(stable and position_error <= .0003 and orientation_error <= .005
                    and current['body_path'] == previous['body_path']
                    and current['motion_view_id'] == previous['motion_view_id'])
            if not stable:
                state.update(count=0, pose_anchor=None)
                return False
        else:
            state['pose_anchor'] = copy.deepcopy(poses)
    return state.get('count', 0) >= max(int(required), 1)


class BeamAssistedState:
    def _assisted_forbid_body_rewrite(self, object_name, operation):
        if (getattr(self, '_beam_assisted_enabled', False)
                and str(object_name).startswith('fabrica_beam_')):
            raise RuntimeError(f'assisted Beam forbids {operation}: {object_name}')

    def _assisted_event(self, event, **detail):
        path = os.environ.get('BEAM_ASSISTED_TRACE_PATH')
        if path:
            with Path(path).open('a') as stream:
                stream.write(json.dumps({'event': event, 'step': int(self.step_counter),
                    'phase': self.phase, 'physics_stamp': self._beam_physics_stamp(),
                    'route': 'assisted', 'formal_physical_success': False,
                    **detail}, default=lambda x: x.tolist())+'\n')

    def _assisted_body_path(self, object_name):
        body = self._resolve_object(object_name)
        prim = self._object_prims[object_name]
        owners = list(self._iter_rigid_body_prims(prim))
        if len(owners) != 1:
            raise RuntimeError(f'assisted object must resolve one rigid body: {object_name}')
        return body, str(owners[0].GetPath())

    def _assisted_joint(self, path, body_path, position, orientation,
                        hand=None):
        from pxr import Gf, Sdf, UsdPhysics
        from isaacsim.core.utils.stage import get_current_stage
        from .planner_primitives import relative_pose
        stage = get_current_stage()
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
        joint = UsdPhysics.FixedJoint.Define(stage, path)
        joint.CreateJointEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        if hand is None:
            local_p, local_q = position, orientation
        else:
            hand_p, hand_q = hand.get_pose()
            local_p, local_q = relative_pose(base_position=hand_p,
                base_orientation=hand_q, world_position=position,
                world_orientation=orientation)
            joint.CreateBody1Rel().SetTargets([Sdf.Path(hand.unwrap().prim_path)])
        joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float, local_p)))
        joint.CreateLocalRot1Attr(Gf.Quatf(*map(float, local_q)))
        joint.CreateCollisionEnabledAttr(True)
        joint.CreateJointEnabledAttr(True)
        local_rotation = joint.GetLocalRot1Attr().Get()
        self._assisted_event('joint_anchor_readback', joint_path=path,
            body0=list(map(str, joint.GetBody0Rel().GetTargets())),
            body1=list(map(str, joint.GetBody1Rel().GetTargets())),
            local_position0=list(joint.GetLocalPos0Attr().Get()),
            local_position1=list(joint.GetLocalPos1Attr().Get()),
            local_orientation1=[float(local_rotation.GetReal()),
                                *map(float, local_rotation.GetImaginary())],
            authored_enabled=joint.GetJointEnabledAttr().Get(),
            collision_enabled=joint.GetCollisionEnabledAttr().Get(),
            measured_object_position=position, measured_object_orientation=orientation,
            scope='USD joint-attribute readback at creation; not a native solver-error measurement')
        return path

    def _assisted_remove_world_joint(self, object_name):
        joints = getattr(self, '_assisted_world_joints', {})
        path = joints.pop(object_name, None)
        getattr(self, '_assisted_install_alignment', {}).pop(object_name, None)
        if path:
            from isaacsim.core.utils.stage import get_current_stage
            get_current_stage().RemovePrim(path)
            self._assisted_event('remove_world_support', object=object_name,
                                 joint_path=path)

    def _assisted_attach(self, object_name, robot_name, phase_spec, attach_spec):
        from .planner_primitives import relative_pose
        metrics = self._gripper_contact_metrics(object_name, robot_name,
                                               attach_spec=attach_spec)
        strict = self._strict_physical_grasp_contact(object_name, metrics,
                                                    attach_spec=attach_spec)
        visual_pinch = bool(attach_spec.get('beam_assisted_visual_pinch'))
        visual_detail = None
        if visual_pinch:
            from .beam_assisted_visual_grasp import ready, native_gaps, hold_command
            if not ready(self, robot_name, object_name):
                raise RuntimeError('assisted joint rejected: visible surfaces are not in a clear dual pinch')
            gaps, _ = native_gaps(self, robot_name, object_name)
            visual_detail = {'qualification': 'dual_visible_CAD_surfaces', 'gaps': gaps,
                             'force_contact_is_not_the_assisted_qualification': True}
            saved = {'gripper_openness': hold_command(self, robot_name, object_name)}
        elif not strict.get('physical_contact_ready'):
            raise RuntimeError('assisted joint rejected: fingers are not touching the part')
        else:
            saved = getattr(self, '_ur5e_contact_hold_commands', {}).get(robot_name)
        if saved is None or not np.isfinite(saved.get('gripper_openness', np.nan)):
            raise RuntimeError('assisted grasp lacks its actual closing command')
        if visual_pinch:
            if not hasattr(self, '_ur5e_contact_hold_commands'):
                self._ur5e_contact_hold_commands = {}
            self._ur5e_contact_hold_commands[robot_name] = copy.deepcopy(saved)
        body, body_path = self._assisted_body_path(object_name)
        position, orientation = body.get_pose()
        hand = self._get_robot_hand_rigid_body(robot_name)
        if hand is None:
            raise RuntimeError('assisted joint rejected: missing native hand body')
        tcp_p, tcp_q = self._get_robot_task_pose(robot_name)
        rel_p, rel_q = relative_pose(base_position=tcp_p, base_orientation=tcp_q,
                                    world_position=position, world_orientation=orientation)
        captured_joint_positions = np.asarray(self.robots[robot_name].articulation.get_joint_positions()).copy()
        captured_hand_position, captured_hand_orientation = hand.get_pose()
        self._assisted_remove_world_joint(object_name)
        path = self._assisted_joint(self._attachment_joint_path(object_name),
                                   body_path, position, orientation, hand)
        self._attachment_joints[object_name] = path
        self._attachments[object_name] = {
            'robot_name': robot_name, 'position': rel_p.tolist(),
            'orientation': rel_q.tolist(), 'mode': 'fixed_joint',
            'joint_path': path, 'contact_metrics': copy.deepcopy(metrics),
            'collision_disabled': False, 'attach_spec': copy.deepcopy(attach_spec),
            'phase': phase_spec.get('name'), 'attach_step': int(self.step_counter),
            'attach_world_position': np.asarray(position).tolist(),
            'attach_world_orientation': np.asarray(orientation).tolist(),
        }
        self._attachments[object_name]['assisted_hold_openness'] = float(saved['gripper_openness'])
        self._locked_targets.pop(object_name, None)
        self._frozen_lock_poses.pop(object_name, None)
        self._locked_collision_states.pop(object_name, None)
        self._assisted_event('attach_measured_pose', object=object_name,
            robot=robot_name, joint_path=path, position=position,
            orientation=orientation, contact_metrics=metrics, strict_contact=strict,
            visible_pinch=visual_detail,
            full_joint_positions=captured_joint_positions,
            native_hand_position=captured_hand_position,
            native_hand_orientation=captured_hand_orientation)

    def _assisted_lock(self, object_name, target_name, lock_spec):
        if self._locked_targets.get(object_name) == target_name:
            return
        body, body_path = self._assisted_body_path(object_name)
        position, orientation = body.get_pose()
        released_robot = self._attachments.get(object_name, {}).get('robot_name')
        self._remove_attachment_joint(object_name)
        self._attachments.pop(object_name, None)
        path = self._attachment_joint_path(object_name) + '_world_support'
        self._assisted_joint(path, body_path, position, orientation)
        if not hasattr(self, '_assisted_world_joints'):
            self._assisted_world_joints = {}
        self._assisted_world_joints[object_name] = path
        if target_name.endswith('_assembled'):
            if not hasattr(self, '_assisted_install_alignment'):
                self._assisted_install_alignment = {}
            target = self.target_poses[target_name]
            self._assisted_install_alignment[object_name] = {
                'joint_path': path, 'position': np.asarray(position, float).copy(),
                'orientation': np.asarray(orientation, float).copy(),
                'target_position': np.asarray(target['position'], float).copy(),
                'target_orientation': np.asarray(target['orientation'], float).copy(),
                'release_robot': released_robot,
                'complete': False,
            }
        self._locked_targets[object_name] = target_name
        self._lock_pin_pose[object_name] = False
        self._locked_collision_states[object_name] = True
        self._frozen_lock_poses.pop(object_name, None)
        self._assisted_event('support_measured_pose', object=object_name,
            target=target_name, joint_path=path, position=position,
            orientation=orientation, target_pose=self.target_poses.get(target_name))

    def _assisted_update_install_alignment(self):
        if getattr(self, 'failed', False):
            return
        states = getattr(self, '_assisted_install_alignment', {})
        eligible = {name: s for name, s in states.items()
                    if not s['complete'] and (not s.get('release_robot') or
                       self._assisted_robot_gripper_is_open(s['release_robot']))}
        if not eligible:
            return
        stamp = self._beam_physics_stamp()
        if not stamp.get('valid'):
            raise RuntimeError('assisted installation alignment lacks a native physics clock')
        clock = (stamp.get('epoch'), stamp['index'])
        if getattr(self, '_assisted_alignment_clock', None) == clock:
            return
        self._assisted_alignment_clock = clock
        from pxr import Gf, UsdPhysics
        from isaacsim.core.utils.stage import get_current_stage
        stage = get_current_stage()
        for name, s in eligible.items():
            delta = s['target_position'] - s['position']
            length = float(np.linalg.norm(delta))
            s['position'] += delta * min(1., .0001 / max(length, 1e-15))
            q = s['orientation'] / np.linalg.norm(s['orientation'])
            target = s['target_orientation'] / np.linalg.norm(s['target_orientation'])
            if np.dot(q, target) < 0:
                target = -target
            angle = 2 * np.arccos(np.clip(np.dot(q, target), -1., 1.))
            ratio = min(1., .0002 / max(float(angle), 1e-15))
            q = (1-ratio)*q + ratio*target
            s['orientation'] = q / np.linalg.norm(q)
            joint = UsdPhysics.FixedJoint.Get(stage, s['joint_path'])
            if not joint:
                raise RuntimeError('assisted installation support joint disappeared')
            joint.GetLocalPos1Attr().Set(Gf.Vec3f(*map(float, s['position'])))
            joint.GetLocalRot1Attr().Set(Gf.Quatf(*map(float, s['orientation'])))
            if int(self.step_counter) % 8 == 0:
                self._assisted_event('installation_anchor_command', object=name,
                    anchor_position=s['position'], anchor_orientation=s['orientation'],
                    target_position=s['target_position'], target_orientation=s['target_orientation'],
                    max_anchor_step_m=.0001, max_anchor_step_rad=.0002)
            if length <= .0001 and angle <= .0002:
                s['complete'] = True
                self._assisted_event('installation_anchor_at_goal', object=name,
                    anchor_position=s['position'], anchor_orientation=s['orientation'],
                    max_anchor_step_m=.0001, max_anchor_step_rad=.0002,
                    actual_pose_still_requires_native_readback=True)

    def _assisted_release_open(self, phase_spec):
        final = phase_spec.get('name') == 'beam_assisted_final_verification'
        if not phase_spec.get('lock') and not phase_spec.get('detach') and not final:
            return True
        for name, command in phase_spec.get('gripper_commands', {}).items():
            if command != 'open':
                continue
            if not self._assisted_robot_gripper_is_open(name):
                return False
        installed = list(phase_spec.get('beam_assisted_installed_parts', []))
        if not installed:
            return True
        stamp = self._beam_physics_stamp()
        sample = self._assisted_installation_sample(installed, stamp, final=final)
        self._assisted_install_gate_sample = sample
        key = (self.phase_index, self.phase_entry_step, tuple(installed))
        gates = getattr(self, '_assisted_release_gates', None)
        if gates is None:
            gates = self._assisted_release_gates = {}
        gate = gates.setdefault(key, {})
        ready = consecutive_native_gate(gate, stamp, sample['ready'], 1 if final else 12,
                                        poses=sample['objects'])
        previous = gate.get('reported')
        signature = (sample['ready'], sample.get('reason'), ready)
        if previous != signature or int(self.step_counter) % 48 == 0:
            self._assisted_event('installation_native_gate', final=final,
                required_stable_physics_steps=1 if final else 12,
                stable_physics_steps=gate.get('count', 0), ready=ready, sample=sample)
            gate['reported'] = signature
        return ready

    def _assisted_native_body_motion(self, name, epoch, *, include_center_of_mass=False):
        """Read only existing exact-path views; qualification never initializes one."""
        from .beam_support_state import live_body_snapshot
        body, path = self._assisted_body_path(name)
        native = getattr(self, '_assisted_native_body_readers', {}).get(path)
        errors = []
        for source, wrapper in (('preflight_native_reader', native), ('resolved_object', body)):
            if wrapper is None:
                continue
            unwrap = getattr(wrapper, 'unwrap', None)
            reader = unwrap() if callable(unwrap) else wrapper
            if str(getattr(reader, 'prim_path', '')) != path:
                raise ValueError('Assisted native installation body path alias: ' + name)
            view = getattr(reader, '_rigid_prim_view', None)
            if view is None or not view.is_physics_handle_valid():
                errors.append(source + ':invalid_or_absent_live_view')
                continue
            observed = getattr(self, '_assisted_install_observed_views', None)
            if observed is None:
                observed = self._assisted_install_observed_views = {}
            # Retain read-only references so replacing a wrapper cannot reuse
            # an old Python identity while a pose window/certificate owns it.
            observed.setdefault((epoch, path, id(view)), view)
            result = live_body_snapshot(view)
            if include_center_of_mass:
                from .beam_physics_diagnostics import _body_com_snapshot
                com = _body_com_snapshot(view, np.asarray([result['position']], float),
                                         np.asarray([result['orientation']], float))
                if not com.get('valid') or com.get('position_frame') != 'world':
                    raise ValueError('Assisted transition lacks an existing live COM readback')
                result.update(center_of_mass_valid=True, center_of_mass_source=com['source'],
                    center_of_mass_position_frame='world', center_of_mass_orientation_order='wxyz',
                    center_of_mass_position=com['world_positions'][0],
                    center_of_mass_orientation=com['world_orientations'][0])
            result.update(body_path=path, motion_view_id=id(view),
                          motion_source='existing_exact_path_native_view',
                          existing_view_source=source)
            return result
        raise ValueError('No existing live assisted installation view for ' + name + '; ' + ','.join(errors))

    def _assisted_support_joint_live(self, name, state, body_path):
        from pxr import UsdPhysics
        from isaacsim.core.utils.stage import get_current_stage
        path = getattr(self, '_assisted_world_joints', {}).get(name)
        if not path or path != state.get('joint_path'):
            return False
        joint = UsdPhysics.FixedJoint.Get(get_current_stage(), path)
        return bool(joint and joint.GetJointEnabledAttr().Get()
                    and [str(p) for p in joint.GetBody0Rel().GetTargets()] == [body_path]
                    and not joint.GetBody1Rel().GetTargets())

    def _assisted_home_sample(self, targets, stamp):
        """Read existing native arm views on the configured raw initial branch."""
        result = {'ready': False, 'physics_stamp': dict(stamp), 'robots': {}}
        try:
            if (stamp.get('valid') is not True or type(stamp.get('epoch')) is not int
                    or type(stamp.get('index')) is not int or stamp['epoch'] < 0
                    or stamp['index'] < 0 or not math.isfinite(stamp['time'])
                    or not math.isfinite(stamp['dt']) or stamp['time'] < 0 or stamp['dt'] <= 0
                    or self._beam_physics_stamp() != stamp):
                raise ValueError('Home qualification lacks the current native physics clock')
            if not isinstance(targets, dict) or len(targets) != 2 or set(targets) != set(self.robots):
                raise ValueError('Home qualification must cover both configured robots')
            arm_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            for name, target in targets.items():
                robot = self.robots[name]
                config = robot.config
                configured = config.initial_joint_positions
                names = target.get('joint_names')
                goal = np.asarray(target.get('joint_positions'), dtype=float)
                if (names != arm_names or goal.shape != (6,) or not np.all(np.isfinite(goal))
                        or target.get('tolerance_rad') != .01
                        or not isinstance(configured, dict)
                        or any(j not in configured for j in arm_names)
                        or not np.array_equal(goal, np.asarray([configured[j] for j in arm_names], float))):
                    raise ValueError('Home target differs from the configured initial native branch: ' + name)
                articulation = robot.articulation
                view = articulation._articulation_view
                if not view.is_physics_handle_valid():
                    raise ValueError('Home qualification lacks an existing live articulation view: ' + name)
                native_names = list(articulation.dof_names)
                if (len(native_names) != 12 or len(set(native_names)) != 12
                        or any(native_names.count(j) != 1 for j in arm_names)):
                    raise ValueError('Home native joint names are incomplete or aliased: ' + name)
                raw = np.asarray(view.get_joint_positions(clone=True), dtype=float)
                if raw.shape != (1, len(native_names)) or not np.all(np.isfinite(raw)):
                    raise ValueError('Home qualification lacks complete finite native joint readback: ' + name)
                actual = raw[0, [native_names.index(j) for j in arm_names]]
                error = float(np.max(np.abs(actual - goal)))
                result['robots'][name] = {'ready': error <= .01,
                    'joint_names': list(names), 'target_joint_positions': goal.tolist(),
                    'joint_positions': actual.tolist(), 'maximum_error_rad': error,
                    'tolerance_rad': .01, 'native_dof_names': native_names,
                    'full_native_joint_positions': raw[0].tolist(),
                    'articulation_id': id(articulation), 'motion_view_id': id(view),
                    'source': 'same_existing_live_articulation_view.get_joint_positions',
                    'angle_comparison': 'raw_native_branch_no_modulo'}
                if (list(articulation.dof_names) != native_names
                        or articulation._articulation_view is not view
                        or not view.is_physics_handle_valid()):
                    raise ValueError('Home articulation identity changed during native readback: ' + name)
            if (len({r['articulation_id'] for r in result['robots'].values()}) != 2
                    or len({r['motion_view_id'] for r in result['robots'].values()}) != 2):
                raise ValueError('Home qualification aliased the two robot articulations/views')
            if self._beam_physics_stamp() != stamp:
                raise ValueError('Home physics clock changed during native readback')
            result['ready'] = all(r['ready'] for r in result['robots'].values())
            if not result['ready']:
                result['reason'] = 'native_arm_outside_initial_joint_tolerance'
        except Exception as exc:
            result['reason'] = f'{type(exc).__name__}: {exc}'
        return result

    def _assisted_installation_sample(self, names, stamp, *, final=False):
        """Qualify all installed actors at one clock, including actual motion."""
        from .planner_primitives import pose_error
        result = {'ready': False, 'physics_stamp': dict(stamp), 'objects': {}}
        try:
            if stamp.get('valid') is not True:
                raise ValueError('Installation native physics clock unavailable')
            if not names or len(set(names)) != len(names):
                raise ValueError('Installation body collection is empty or duplicated')
            states = getattr(self, '_assisted_install_alignment', {})
            for name in names:
                state = states.get(name)
                if not state or not state.get('complete'):
                    raise ValueError('Installation anchor has not reached its goal: ' + name)
                if name in getattr(self, '_attachments', {}):
                    raise ValueError('Installed part remains attached to a robot: ' + name)
                target_name = getattr(self, '_locked_targets', {}).get(name)
                if not target_name or not target_name.endswith('_assembled'):
                    raise ValueError('Installed part lacks its final target: ' + name)
                motion = self._assisted_native_body_motion(name, stamp['epoch'])
                if not self._assisted_support_joint_live(name, state, motion['body_path']):
                    raise ValueError('Installation world-support joint is absent or invalid: ' + name)
                target = self.target_poses[target_name]
                position_error, orientation_error = pose_error(
                    current_position=motion['position'], current_orientation=motion['orientation'],
                    target_position=target['position'], target_orientation=target['orientation'])
                within = bool(position_error <= (.003 if final else .001)
                    and orientation_error <= (.04 if final else .003))
                result['objects'][name] = {**motion, 'target': target_name,
                    'position_error': float(position_error), 'orientation_error': float(orientation_error),
                    'ready': within, 'velocity_qualification': 'raw_diagnostic_only_pose_window_required'}
                try:
                    result['objects'][name]['legacy_static_reading'] = self._object_velocity_metrics(name)
                except Exception as exc:
                    result['objects'][name]['legacy_static_reading'] = {'error': str(exc)}
            phases = getattr(self, 'phase_specs', [])
            has_home_targets = bool(final and phases and 'beam_assisted_home_targets' in phases[-1])
            if has_home_targets:
                home_targets = phases[-1]['beam_assisted_home_targets']
                result['home'] = self._assisted_home_sample(home_targets, stamp)
                if not result['home']['ready']:
                    raise ValueError('Final native home rejected: ' + str(result['home'].get('reason')))
            if self._beam_physics_stamp() != stamp:
                raise ValueError('Installation physics clock changed during native readback')
            result['ready'] = all(o['ready'] for o in result['objects'].values())
            if not result['ready']:
                result['reason'] = 'native_pose_outside_installation_limits'
        except Exception as exc:
            result['reason'] = f'{type(exc).__name__}: {exc}'
        if (not result['ready'] and final and getattr(self, 'phase_specs', [])
                and 'beam_assisted_home_targets' in self.phase_specs[-1]):
            # The ordinary runner may return before conditions_stable when
            # release/native qualification fails. Clear this new joint/object
            # window immediately rather than retaining a stale certificate.
            gate = getattr(self, '_assisted_final_gate', None)
            if gate is not None:
                gate.update(previous=None, count=0, pose_anchor=None,
                            last_poses=None, previous_home=None)
            self.__dict__.pop('_assisted_final_certificate', None)
        return result

    def _assisted_conditions_stable(self, advance, ready):
        """Final common window uses this route's clock, independent of table contact."""
        key = (self.phase_index, self.phase_entry_step, json.dumps(advance, sort_keys=True))
        state = getattr(self, '_assisted_final_gate', None)
        if state is None or state['key'] != key:
            state = self._assisted_final_gate = {'key': key}
        stamp = self._beam_physics_stamp()
        sample = getattr(self, '_assisted_install_gate_sample', {})
        ready = bool(ready and sample.get('ready')
                     and sample.get('physics_stamp') == stamp)
        if 'home' in sample:
            home = sample['home']
            ready = bool(ready and home.get('ready') and home.get('physics_stamp') == stamp
                         and set(home.get('robots', {})) == set(self.robots))
            previous_home = state.get('previous_home')
            if previous_home and ready:
                same = previous_home['physics_stamp'] == stamp
                for name, current in home['robots'].items():
                    old = previous_home['robots'].get(name, {})
                    replaced = any(current.get(k) != old.get(k) for k in
                                   ('articulation_id', 'motion_view_id', 'native_dof_names'))
                    changed_same_tick = same and current.get('full_native_joint_positions') != old.get('full_native_joint_positions')
                    if replaced or changed_same_tick:
                        ready = False
            state['previous_home'] = copy.deepcopy(home) if ready else None
        qualified = consecutive_native_gate(state, stamp, ready, advance['physics_stable_steps'],
                                            poses=sample.get('objects', {}))
        phases = getattr(self, 'phase_specs', [])
        if (qualified and phases and self.phase_index == len(phases) - 1
                and self.phase == phases[-1].get('name') == 'beam_assisted_final_verification'
                and int(advance['physics_stable_steps']) >= 240):
            self._assisted_final_certificate = {
                'identity': self._assisted_final_identity(phases[-1]),
                'phase_index': self.phase_index, 'phase_entry_step': self.phase_entry_step,
                'gate_key': key, 'physics_stamp': dict(stamp),
                'stable_physics_steps': state.get('count', 0),
                'required_stable_physics_steps': int(advance['physics_stable_steps']),
                'native_poses': copy.deepcopy(sample['objects']),
            }
            if 'home' in sample:
                self._assisted_final_certificate['native_home'] = copy.deepcopy(sample['home'])
        elif not qualified:
            self.__dict__.pop('_assisted_final_certificate', None)
        if state.get('reported_ready') != qualified or int(self.step_counter) % 48 == 0:
            self._assisted_event('final_native_common_window', ready=qualified,
                stable_physics_steps=state.get('count', 0),
                required_stable_physics_steps=advance['physics_stable_steps'],
                pose_drift_position_budget_m=.0003, pose_drift_orientation_budget_rad=.005)
            state['reported_ready'] = qualified
        return qualified

    def _assisted_final_identity(self, final):
        cfg = getattr(self, 'cfg', None)
        criteria = getattr(cfg, 'success_criteria', [])
        targets = {str(c.get('target') or c.get('target_name')):
                   self.target_poses.get(c.get('target') or c.get('target_name')) for c in criteria}
        return {'task_instance': id(self), 'task_name': getattr(self, 'name', None),
            'episode': [getattr(cfg, k, None) for k in ('recipe', 'seed', 'layout_seed', 'episode_idx')],
            'configuration': json.dumps({'final': final, 'criteria': criteria, 'targets': targets},
                sort_keys=True, allow_nan=False, default=lambda value: value.tolist())}

    def _assisted_final_success_ready(self):
        """Recheck this episode's completed native window, including after terminal transition."""
        from .planner_primitives import pose_error
        reason = None
        sample = None
        try:
            phases = self.phase_specs
            if not phases or getattr(self, 'failed', False):
                raise ValueError('Assisted final phase is absent or the episode failed')
            final = phases[-1]
            if final.get('name') != 'beam_assisted_final_verification':
                raise ValueError('Assisted final verification phase is absent')
            in_final = self.phase_index == len(phases) - 1 and self.phase == final['name']
            completed = (getattr(self, 'success', False) and self.phase == 'complete'
                         and self.phase_index == len(phases)
                         and getattr(self, 'phase_status', None) == 'success'
                         and getattr(self, 'terminal_reason', None) == 'success-criteria-met')
            if not (in_final or completed):
                raise ValueError('Assisted success cannot precede final verification')
            certificate = getattr(self, '_assisted_final_certificate', None)
            gate = getattr(self, '_assisted_final_gate', {})
            required = int(final.get('advance', {}).get('physics_stable_steps', 0))
            if (not certificate or required < 240
                    or certificate['identity'] != self._assisted_final_identity(final)
                    or certificate['phase_index'] != len(phases) - 1
                    or certificate['gate_key'] != gate.get('key')
                    or certificate['required_stable_physics_steps'] != required
                    or certificate['stable_physics_steps'] < required
                    or gate.get('count', 0) < required):
                raise ValueError('Assisted final native window is absent or belongs to another episode/phase')
            if in_final and self.phase_entry_step != certificate['phase_entry_step']:
                raise ValueError('Assisted final phase was re-entered after qualification')
            stamp = self._beam_physics_stamp()
            if not stamp.get('valid') or stamp != certificate['physics_stamp']:
                raise ValueError('Assisted final certificate is not from the current native physics sample')
            names = list(final.get('beam_assisted_installed_parts', []))
            expected = {'fabrica_beam_' + n for n in ('6', '3', '2', '1', '0')}
            criteria = self.cfg.success_criteria
            if (len(names) != 5 or set(names) != expected
                    or len(criteria) != 5 or {c['object'] for c in criteria} != expected
                    or set(certificate['native_poses']) != expected):
                raise ValueError('Assisted final native qualification must cover all five parts')
            commands = final.get('gripper_commands', {})
            if set(commands) != set(self.robots) or any(c != 'open' for c in commands.values()):
                raise ValueError('Assisted final phase does not open both grippers')
            if not all(self._assisted_robot_gripper_is_open(name) for name in commands):
                raise ValueError('Assisted final native grippers are not fully open')
            sample = self._assisted_installation_sample(names, stamp, final=True)
            if not sample['ready']:
                raise ValueError('Assisted final native readback rejected: ' + str(sample.get('reason')))
            for name, current in sample['objects'].items():
                recorded = certificate['native_poses'][name]
                dp, dq = pose_error(current_position=current['position'],
                    current_orientation=current['orientation'],
                    target_position=recorded['position'], target_orientation=recorded['orientation'])
                if (current['body_path'] != recorded['body_path']
                        or current['motion_view_id'] != recorded['motion_view_id']
                        or dp > 1e-9 or dq > 1e-8):
                    raise ValueError('Assisted final body/view changed within the certified physics sample: ' + name)
            if 'beam_assisted_home_targets' in final:
                home = sample.get('home', {})
                recorded_home = certificate.get('native_home', {})
                if (not home.get('ready') or not recorded_home.get('ready')
                        or home.get('physics_stamp') != stamp or recorded_home.get('physics_stamp') != stamp
                        or set(home.get('robots', {})) != set(self.robots)
                        or set(recorded_home.get('robots', {})) != set(self.robots)):
                    raise ValueError('Assisted final native home certificate is absent or incomplete')
                for name, current in home['robots'].items():
                    recorded = recorded_home['robots'][name]
                    if any(current.get(k) != recorded.get(k) for k in
                           ('articulation_id', 'motion_view_id', 'native_dof_names',
                            'joint_names', 'target_joint_positions', 'full_native_joint_positions')):
                        raise ValueError('Assisted final arm/view changed within the certified physics sample: ' + name)
            if self._beam_physics_stamp() != stamp:
                raise ValueError('Assisted final physics clock changed during success recheck')
        except Exception as exc:
            reason = f'{type(exc).__name__}: {exc}'
        self._assisted_success_native_sample = sample
        decision = {'ready': reason is None, 'reason': reason}
        if decision != getattr(self, '_assisted_success_last_decision', None):
            self._assisted_event('final_success_native_recheck', **decision)
            self._assisted_success_last_decision = decision
        return reason is None

    def _assisted_installed_poses_ready(self, names):
        sample = getattr(self, '_assisted_install_gate_sample', {})
        return bool(sample.get('ready') and sample.get('physics_stamp') == self._beam_physics_stamp()
                    and names and set(names).issubset(sample.get('objects', {})))

    def _assisted_robot_gripper_is_open(self, name):
        q = np.asarray(self.robots[name].articulation.get_joint_positions(), float)
        if q.shape != (12,) or not np.all(np.isfinite(q)):
            raise RuntimeError('assisted release lacks the complete native gripper state')
        return bool(np.max(np.abs(q[6:])) <= .05)

    def _assisted_detach(self, object_name):
        self._remove_attachment_joint(object_name)
        self._attachments.pop(object_name, None)
        self._assisted_event('detach_grasp', object=object_name)

    def _assisted_idle_action(self, robot_name, phase_spec):
        subset = self.robots[robot_name].controllers['arm_joint_controller'].get_joint_subset()
        actual = np.asarray(subset.get_joint_positions(), dtype=float)
        support = phase_spec.get('beam_assisted_visual_support') or {}
        if support.get('robot') == robot_name:
            # This entry is called only for an inactive arm. Preserve the last
            # published PD equilibrium instead of replacing it with measured q.
            stamp = self._beam_physics_stamp()
            published_stamp = getattr(self, '_assisted_published_q_stamps', {}).get(robot_name)
            anchor = np.asarray(getattr(self, '_assisted_published_q', {}).get(robot_name), dtype=float)
            if (stamp.get('valid') is not True or type(stamp.get('epoch')) is not int
                    or type(stamp.get('index')) is not int or stamp['epoch'] < 0 or stamp['index'] < 0
                    or not isinstance(published_stamp, dict)
                    or published_stamp.get('epoch') != stamp['epoch']
                    or type(published_stamp.get('index')) is not int
                    or stamp['index']-published_stamp['index'] not in (0, 1)
                    or anchor.shape != (6,) or actual.shape != (6,)
                    or not np.all(np.isfinite([anchor, actual]))
                    or np.max(np.abs(anchor-actual)) > .12+1e-12):
                raise ValueError('visual support idle lacks a fresh valid final-published arm target')
            anchor = anchor.copy()
        else:
            if not hasattr(self, '_assisted_idle_anchors'):
                self._assisted_idle_anchors = {}
            anchor = self._assisted_idle_anchors.setdefault(robot_name, actual.copy())
        action = {'arm_joint_controller': [anchor.tolist()]}
        command = phase_spec.get('gripper_commands', {}).get(robot_name)
        if command in ('open', 'close'):
            action['gripper_controller'] = [1.0 if command == 'open' else 0.0]
        elif isinstance(command, (int, float)):
            action['gripper_controller'] = [float(command)]
        elif command is not None:
            raise RuntimeError('assisted idle arm has an unsupported gripper command')
        return action

    def _assisted_active_arm(self, robot_name):
        getattr(self, '_assisted_idle_anchors', {}).pop(robot_name, None)

    def _assisted_arm_joint_limits(self, name):
        """Read the existing live native view; never invent a periodic range."""
        articulation = self.robots[name].articulation
        view = articulation._articulation_view
        if not view.is_physics_handle_valid():
            raise ValueError('assisted native joint limits lack a live physics handle')
        stamp = self._beam_physics_stamp()
        if stamp.get('valid') is not True or type(stamp.get('epoch')) is not int:
            raise ValueError('assisted native joint limits lack a live epoch')
        identity = (id(articulation), id(view), stamp['epoch'])
        cache = self.__dict__.setdefault('_assisted_arm_limits_cache', {})
        cached = cache.get(name)
        if cached is not None and cached['identity'] == identity:
            return cached['limits'].copy()
        names = list(articulation.dof_names)
        arm = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
        values = np.asarray(view.get_dof_limits(), dtype=np.float32)
        if (len(names) != len(set(names)) or values.shape != (1, len(names), 2)
                or any(names.count(n) != 1 for n in arm)):
            raise ValueError('assisted native joint limit names/shape are invalid')
        limits = values[0, [names.index(n) for n in arm]].copy()
        if not np.all(np.isfinite(limits)) or np.any(limits[:, 0] >= limits[:, 1]):
            raise ValueError('assisted native arm joint limits are invalid')
        cache[name] = dict(identity=identity, limits=limits.copy())
        self._assisted_event('native_arm_joint_limits', robot=name,
            joint_names=list(arm), limits_rad=limits, physics_stamp=stamp,
            source='same_existing_live_articulation_view.get_dof_limits')
        return limits

    def _assisted_guard_actions(self, actions):
        previous = getattr(self, '_assisted_published_q', {})
        phase = self.get_current_phase_spec() if hasattr(self, 'get_current_phase_spec') else {}
        empty_return = bool(phase.get('beam_assisted_empty_return_guard'))
        if empty_return:
            empty_stamp = self._beam_physics_stamp()
            if (empty_stamp.get('valid') is not True
                    or type(empty_stamp.get('epoch')) is not int
                    or type(empty_stamp.get('index')) is not int):
                raise ValueError('assisted empty return lacks a native physics clock')
            if getattr(self, '_attachments', {}):
                raise ValueError('assisted empty return cannot move an attached part')
            if set(actions) != set(self.robots):
                raise ValueError('assisted empty return lacks the complete dual-arm SDK batch')
            for name in self.robots:
                action = actions[name] or {}
                if action.get('arm_joint_controller') is None:
                    raise ValueError('assisted empty return lacks a final arm target: ' + name)
                opening = np.asarray(action.get('gripper_controller'), dtype=float)
                if (opening.shape != (1,) or not np.all(np.isfinite(opening))
                        or opening[0] != 1.0 or not self._assisted_robot_gripper_is_open(name)):
                    raise ValueError('assisted empty return requires both actual/commanded grippers open')
                config = self.robots[name].config
                if (config.gripper_open_position != 0.
                        or config.initial_joint_positions.get('finger_joint') != 0.):
                    raise ValueError('assisted empty return requires the configured zero-open gripper branch')
        if phase.get('local_skill', {}).get('beam_assisted_production_cad_guard'):
            # Predict both arms from this same SDK batch, including the idle
            # support arm's outer native clamp, before checking either sweep.
            pending = {}
            for name, action in actions.items():
                target = (action or {}).get('arm_joint_controller')
                if target is not None:
                    subset = self.robots[name].controllers['arm_joint_controller'].get_joint_subset()
                    actual = np.asarray(subset.get_joint_positions(), dtype=float)
                    pending[name] = bounded_native_target(target[0], actual, previous.get(name),
                        unwrap=not phase.get('beam_assisted_empty_return_guard', False))
            self._assisted_batch_arm_candidates = pending
        for name, action in actions.items():
            if not action:
                continue
            target = action.get('arm_joint_controller')
            if 'arm_ik_controller' in action and target is None:
                raise RuntimeError('assisted command must pass the explicit native joint guard')
            # The robot's explicit-joint override already suppresses this IK
            # entry. Remove it here so the recorded action has one arm path.
            redundant_ik = action.pop('arm_ik_controller', None) is not None
            if target is None:
                continue
            subset = self.robots[name].controllers['arm_joint_controller'].get_joint_subset()
            actual = np.asarray(subset.get_joint_positions(), dtype=float)
            bounded = bounded_native_target(target[0], actual, previous.get(name),
                unwrap=not phase.get('beam_assisted_empty_return_guard', False))
            context = getattr(self, '_beam_assisted_command_contexts', {}).get(name)
            current_phase = self.get_current_phase_spec() if hasattr(self, 'get_current_phase_spec') else {}
            from .local_skills import normalize_local_skill_spec
            current_skill = normalize_local_skill_spec(current_phase, name) or {}
            current_absolute = bool(current_skill.get('beam_assisted_absolute_object_servo')
                                    and current_skill.get('requires_held_object'))
            if current_absolute:
                if context is None:
                    raise ValueError('assisted final command has no current object-servo context')
                bounded, object_guard = context['adapter']._assisted_verify_published_held_command(
                    task=self, robot_name=name, context=context,
                    command_q=bounded, current_q=actual, previous_q=previous.get(name))
                if int(self.step_counter) % 8 == 0 or object_guard['accepted_ratio'] < 1.:
                    self._assisted_event('published_object_guard', robot=name, **object_guard)
            if current_phase.get('beam_assisted_native_joint_limits'):
                limits = self._assisted_arm_joint_limits(name)
                # Match the SDK/native float32 conversion exactly. This is
                # not a widened physical range or a modulo-angle comparison.
                native_target = np.asarray(bounded, dtype=np.float32)
                if np.any(native_target < limits[:, 0]) or np.any(native_target > limits[:, 1]):
                    self._assisted_event('native_joint_limit_rejected', robot=name,
                        requested_final_q=bounded, native_float32_target=native_target,
                        limits_rad=limits, physics_stamp=self._beam_physics_stamp())
                    raise ValueError('assisted final target exceeds actual native joint limits')
            action['arm_joint_controller'] = [bounded.tolist()]
            previous[name] = bounded.copy()
            held = next((s for s in getattr(self, '_attachments', {}).values()
                         if s.get('robot_name') == name
                         and s.get('attach_spec', {}).get('beam_assisted')), None)
            if held is not None or (current_phase.get('beam_assisted_visual_support') or {}).get('robot') == name:
                if not hasattr(self, '_assisted_published_q_stamps'):
                    self._assisted_published_q_stamps = {}
                stamp = self._beam_physics_stamp()
                self._assisted_published_q_stamps[name] = {
                    'epoch': stamp.get('epoch') if stamp.get('valid') is True else None,
                    'index': stamp.get('index') if stamp.get('valid') is True else None,
                }
            getattr(self, '_ur5e_plumbers_last_arm_command_q', {})[name] = bounded.copy()
            if held is not None:
                action['gripper_controller'] = [held['assisted_hold_openness']]
            entry = getattr(self, 'phase_entry_step', None)
            trace_spec = current_phase.get('local_skill') or {}
            trace_ticks = (24 if trace_spec.get('beam_assisted_phase_handoff_ticks') is not None
                           else 96 if trace_spec.get('beam_assisted_alignment_hold')
                           and current_phase.get('name', '').endswith('_insert_00') else -1)
            handoff_trace = bool(type(entry) is int
                and 0 <= int(self.step_counter)-entry <= trace_ticks)
            if (int(self.step_counter) % 8 == 0 or handoff_trace
                    or trace_spec.get('beam_assisted_transition_settle')):
                self._assisted_event('published_command', robot=name,
                    actual_q=actual, requested_q=np.asarray(target[0]),
                    published_q=bounded, redundant_ik_removed=redundant_ik)
        if empty_return:
            pending_final_targets = {
                name: np.asarray(actions[name]['arm_joint_controller'][0], dtype=float).copy()
                for name in self.robots}
            self._assisted_batch_arm_candidates = pending_final_targets
            guard = getattr(self, '_assisted_production_cad_guard', None)
            if not callable(guard) or self._beam_physics_stamp() != empty_stamp:
                raise ValueError('assisted empty return lacks a same-clock production CAD guard')
            allowed, detail = guard(empty=True, physics_stamp=empty_stamp,
                installed_objects=phase.get('beam_assisted_installed_parts', []))
            if not allowed:
                self._assisted_event('empty_return_cad_rejected', **detail)
                raise ValueError('assisted empty return final dual-arm sweep rejected')
            if self._beam_physics_stamp() != empty_stamp:
                raise ValueError('native clock changed during assisted empty return final guard')
            if int(self.step_counter) % 8 == 0:
                self._assisted_event('empty_return_published_guard', **detail)
        self._assisted_published_q = previous
        return actions
