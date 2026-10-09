"""Physical Beam support bookkeeping and measured table-load gates.

This module never writes a body pose, velocity, collision filter, or joint.
"""
from __future__ import annotations
import itertools
import json
import os
import numpy as np
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_rotate, pose_error


def table_support_ready(sample, minimum_force):
    return bool(sample.get('valid') and sample.get('motion_valid')
                and minimum_force <= sample['vertical_force'] <= 40.0
                and -.0005 <= sample['bottom_gap'] <= .001
                and sample['linear_speed'] <= .002 and sample['angular_speed'] <= .03)


def combine_support_surfaces(surfaces, world_corners):
    """Sum real, filtered object/surface forces in the object's footprint.

    The thick assembly collider is above the visual tabletop. Reading only the
    visual prim misses that load. Overlapping physical colliders exert separate
    forces; each configured body is sampled once, without synthesizing contact.
    """
    corners = np.asarray(world_corners)
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    under = [s for s in surfaces if np.all(np.asarray(s['lower'])[:2] <= hi[:2])
             and np.all(np.asarray(s['upper'])[:2] >= lo[:2])]
    if not under or any(not s['contact'].get('valid') or 'force_world' not in s['contact'] for s in under):
        raise ValueError('Physical support pair force unavailable')
    top = max(s['upper'][2] for s in under)
    return {'table_top': float(top), 'bottom_gap': float(lo[2]-top),
            'vertical_force': abs(sum(float(s['contact']['force_world'][2]) for s in under)),
            'support_surfaces': under}


def cube_world_bounds(size, usd_row_transform):
    """Compute analytic Cube bounds; Isaac authors a pre-scaled extent attribute.

    BBoxCache transforms that extent again. Transform the unscaled primitive
    corners instead, preserving USD's row-vector matrix convention.
    """
    half = float(size) / 2
    corners = np.array([(*p, 1.) for p in itertools.product([-half, half], repeat=3)])
    points = (corners @ np.asarray(usd_row_transform))[:, :3]
    return points.min(axis=0).tolist(), points.max(axis=0).tolist()


def live_body_snapshot(view):
    """Read one live body; never accept authored USD state as physical motion.

    Contact views are already initialized by the pair-force reader. Using that
    same view avoids initializing a second wrapper (and its state side effects).
    Copy all buffers before another getter can reuse them. Coordinates are world
    coordinates and angular velocity is rad/s, as in the tensor view API.
    """
    if not view.is_physics_handle_valid():
        raise ValueError('Beam motion view has no live physics handle')
    positions, orientations = view.get_world_poses(clone=True)
    position = np.array(positions, dtype=float, copy=True)
    orientation = np.array(orientations, dtype=float, copy=True)
    velocities = np.array(view.get_velocities(clone=True), dtype=float, copy=True)
    if (position.shape != (1, 3) or orientation.shape != (1, 4)
            or velocities.shape != (1, 6)
            or not all(np.all(np.isfinite(v)) for v in (position, orientation, velocities))
            or np.linalg.norm(orientation[0]) < 1e-9):
        raise ValueError('Invalid single-body Beam physics snapshot')
    return {'position': position[0].tolist(),
            'orientation': (orientation[0] / np.linalg.norm(orientation[0])).tolist(),
            'linear_velocity': velocities[0, :3].tolist(),
            'angular_velocity': velocities[0, 3:].tolist(),
            'linear_speed': float(np.linalg.norm(velocities[0, :3])),
            'angular_speed': float(np.linalg.norm(velocities[0, 3:])),
            'motion_valid': True, 'motion_source': 'live_pair_rigid_view',
            'physics_handle_valid': True, 'pose_frame': 'world'}


class BeamSupportState:
    def _attachment_for(self, object_name, robot_name=None):
        primary = self._attachments.get(object_name)
        if primary is not None and (robot_name is None or primary.get('robot_name') == robot_name):
            return primary
        return getattr(self, '_beam_shared_grasps', {}).get((object_name, robot_name))

    def _all_physical_grasps(self):
        return [*self._attachments.items(),
                *((name, state) for (name, _), state in getattr(self, '_beam_shared_grasps', {}).items())]

    def _store_physical_grasp(self, object_name, state):
        previous = self._attachments.get(object_name)
        if previous and previous['robot_name'] != state['robot_name']:
            if not (state['attach_spec'].get('allow_shared_physical_grasp')
                    and previous['attach_spec'].get('allow_shared_physical_grasp')
                    and previous['mode'] == state['mode'] == 'pure_physical_grasp'):
                raise RuntimeError('A second physical grasp requires explicit shared support')
            self._beam_shared_grasps[(object_name, state['robot_name'])] = state
        else:
            self._attachments[object_name] = state

    def _remove_physical_grasp(self, object_name, robot_name):
        """Remove only this hand; promote the other anchor without recapture."""
        shared = getattr(self, '_beam_shared_grasps', {})
        primary = self._attachments.get(object_name)
        if primary is not None and primary.get('robot_name') == robot_name:
            self._attachments.pop(object_name)
            for key, other in list(shared.items()):
                if key[0] == object_name:
                    self._attachments[object_name] = shared.pop(key)
                    break
        else:
            shared.pop((object_name, robot_name), None)

    def _beam_table_observation(self):
        step = int(self.step_counter)
        cached = getattr(self, '_beam_table_sample', None)
        if cached and cached['step'] == step:
            return cached
        sample = {'step': step, 'phase': self.phase, 'valid': False}
        try:
            from pxr import Usd, UsdGeom
            base = self._resolve_object('fabrica_beam_6')
            # Static table geometry is independent of target rebasing/randomization.
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy'])
            names = self.get_current_phase_spec().get('beam_table_support_objects', ['factory_tabletop_visual'])
            surfaces = []
            motion_view = None
            for name in dict.fromkeys(names):
                self._resolve_object(name)
                table = self._object_prims[name]
                if table.IsA(UsdGeom.Cube):
                    lower, upper = cube_world_bounds(UsdGeom.Cube(table).GetSizeAttr().Get(),
                        UsdGeom.Xformable(table).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
                else:
                    bounds = cache.ComputeWorldBound(table).ComputeAlignedRange()
                    lower, upper = list(bounds.GetMin()), list(bounds.GetMax())
                observation = self._pair_contact_observation(str(base.unwrap().prim_path), str(table.GetPath()))
                surfaces.append({'object':name, 'lower':lower, 'upper':upper, 'contact':observation})
                if motion_view is None and observation.get('valid'):
                    motion_view = self._get_contact_probe(str(base.unwrap().prim_path), str(table.GetPath()))
            snapshot = live_body_snapshot(motion_view)
            corners = itertools.product([-.0762, .0762], [-.00889, .00889], [0., .0127])
            world_corners = [np.asarray(snapshot['position']) + quat_rotate(snapshot['orientation'], p) for p in corners]
            support = combine_support_surfaces(surfaces, world_corners)
            # Diagnostic comparison only: legacy values cannot pass the gate.
            legacy = self._object_velocity_metrics('fabrica_beam_6')
            legacy_view = getattr(base.unwrap(), '_rigid_prim_view', None)
            sample.update(valid=True, **support, **snapshot,
                          physics_dt=float(self._contact_physics_dt()),
                          legacy_velocity=legacy,
                          legacy_physics_handle_valid=bool(legacy_view is not None and
                              legacy_view.is_physics_handle_valid()))
            from isaacsim.core.api import SimulationContext
            context = SimulationContext.instance()
            sample['physics_step_index'] = int(context.current_time_step_index)
            sample['physics_time'] = float(context.current_time)
            sample['body_solver_state'] = [
                {'path': str(body.GetPath()),
                 'velocity_iterations': body.GetAttribute('physxRigidBody:solverVelocityIterationCount').Get(),
                 'position_iterations': body.GetAttribute('physxRigidBody:solverPositionIterationCount').Get(),
                 'stabilization_threshold': body.GetAttribute('physxRigidBody:stabilizationThreshold').Get()}
                for body in self._iter_rigid_body_prims(self._object_prims['fabrica_beam_6'])]
        except Exception as exc:
            sample['error'] = f'{type(exc).__name__}: {exc}'
        self._beam_table_sample = sample
        trace = os.environ.get('BEAM_MOTION_TRACE_PATH')
        if trace and step % 8 == 0:
            with open(os.path.join(os.path.dirname(trace), 'table_support_trace.jsonl'), 'a') as f:
                f.write(json.dumps(sample, allow_nan=False) + '\n')
        return sample

    def _beam_table_supported(self, advance):
        sample = self._beam_table_observation()
        key = (self.phase_index, self.phase_entry_step)
        state = getattr(self, '_beam_support_stability', None)
        if state is None or state['key'] != key:
            state = self._beam_support_stability = {'key': key, 'last_step': None, 'count': 0}
        if sample['step'] != state['last_step']:
            ready = table_support_ready(sample, float(advance.get('minimum_vertical_force', 1.962)))
            state['count'] = state['count'] + 1 if ready else 0
            state['last_step'] = sample['step']
        return state['count'] >= int(advance.get('stable_steps', 96))

    def _beam_support_watchdog(self, phase):
        if not phase.get('beam_require_table_support'):
            return
        self._beam_assembly_integrity(phase)
        sample = self._beam_table_observation()
        # Motion during insertion is not itself a support-loss event. Require
        # measured table load and bounded separation, independent of hand hold.
        valid = (sample.get('valid') and sample['vertical_force'] >= .5
                 and -.001 <= sample['bottom_gap'] <= .0015)
        if valid:
            self._beam_support_missing_since = None
        else:
            start = getattr(self, '_beam_support_missing_since', None)
            if start is None:
                self._beam_support_missing_since = int(self.step_counter)
            elif int(self.step_counter) - start >= 48:
                self._set_terminal_state('failed', reason='beam-table-support-lost', status='failed',
                                         transition_type='failure', detail=sample)

    def _beam_assembly_integrity(self, phase):
        """Detect a held post pulling away from the seated base during insertion.

        Pair forces are recorded as evidence; zero normal force alone is not
        declared separation for an unloaded, fitted interface.
        """
        base = self._resolve_object('fabrica_beam_6')
        base_pose = base.get_pose()
        records = []
        for pid in phase.get('beam_installed_parts', []):
            if pid == '6':
                continue
            part = self._resolve_object('fabrica_beam_' + pid)
            position, orientation = part.get_pose()
            pe, ae = pose_error(current_position=position, current_orientation=orientation,
                                target_position=base_pose[0], target_orientation=base_pose[1])
            parent = base if pid in ['2','3'] else self._resolve_object('fabrica_beam_' + ('2' if pid=='0' else '3'))
            pair = self._pair_contact_observation(str(part.unwrap().prim_path), str(parent.unwrap().prim_path))
            records.append({'part':pid,'relative_position_error':float(pe),
                            'relative_orientation_error':float(ae),'interface_contact':pair})
            if pe > .003 or ae > .05:
                self._set_terminal_state('failed', reason='beam-installed-interface-displaced', status='failed',
                                         transition_type='failure', detail=records[-1])
                return
        step = int(self.step_counter)
        trace = os.environ.get('BEAM_MOTION_TRACE_PATH')
        if trace and step % 8 == 0 and getattr(self, '_beam_last_integrity_trace', None) != step:
            self._beam_last_integrity_trace = step
            with open(os.path.join(os.path.dirname(trace), 'assembly_integrity_trace.jsonl'), 'a') as f:
                f.write(json.dumps({'step':step,'phase':self.phase,'interfaces':records})+'\n')
