"""Passive checks of complete authored colliders using conservative convex enclosures.

This is deliberately distinct from PhysX's cooked decomposition/SDF geometry.
Separation of enclosures is useful; overlap is a candidate requiring spatial
review or native contact evidence. Every enabled collider is enumerated,
including instance proxies, and missing coverage prevents a safety claim.
"""
from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import os
import json
from pathlib import Path
import numpy as np


def pose_matrix(position, orientation):
    q = np.asarray(orientation, dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    result[:3, 3] = position
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite body pose')
    return result


@dataclass
class Collider:
    prim: str
    body: str
    entity: str
    group: str
    robot: str | None
    points: np.ndarray
    approximation: str
    contact_offset: float | None = None

    def __post_init__(self):
        if self.contact_offset is not None and (not np.isfinite(self.contact_offset) or self.contact_offset < 0):
            self.contact_offset = None


class EntityGeometryChecker:
    """Distance/candidate pairs for complete colliders, never a task controller."""

    def __init__(self, colliders, *, proximity=0.01):
        import fcl
        from scipy.spatial import ConvexHull
        self.fcl = fcl
        self.colliders = list(colliders)
        self.proximity = float(proximity)
        self.shapes = []
        for collider in self.colliders:
            points = np.unique(np.asarray(collider.points, dtype=float), axis=0)
            if not np.all(np.isfinite(points)) or len(points) < 4:
                raise ValueError(f'Invalid collider points: {collider.prim}')
            hull = ConvexHull(points)
            vertex_ids = hull.vertices
            vertices = points[vertex_ids]
            mapping = {int(v): i for i, v in enumerate(vertex_ids)}
            faces = []
            for triangle, plane in zip(hull.simplices, hull.equations):
                a, b, c = points[triangle]
                ordered = list(triangle)
                if np.dot(np.cross(b-a, c-a), plane[:3]) < 0:
                    ordered[1], ordered[2] = ordered[2], ordered[1]
                faces.extend([3] + [mapping[int(v)] for v in ordered])
            shape = fcl.Convex(vertices, len(hull.simplices), np.asarray(faces, dtype=np.int32))
            self.shapes.append((fcl.CollisionObject(shape), vertices))

    def check(self, poses, *, step=0, policy=None, task=None, pair_selector=None):
        fcl = self.fcl
        bounds = []
        for collider, (obj, vertices) in zip(self.colliders, self.shapes):
            if collider.body not in poses:
                raise KeyError(f'Missing collider body pose: {collider.body}')
            matrix = np.asarray(poses[collider.body], dtype=float)
            obj.setTransform(fcl.Transform(matrix[:3, :3], matrix[:3, 3]))
            world = vertices @ matrix[:3, :3].T + matrix[:3, 3]
            bounds.append((world.min(axis=0), world.max(axis=0)))
        pairs = 0
        events = []
        closest = None
        snapshot = policy.snapshot_from_task(task) if policy is not None else None
        for i, j in combinations(range(len(self.colliders)), 2):
            a, b = self.colliders[i], self.colliders[j]
            if pair_selector is not None and not pair_selector(a, b):
                continue
            # Articulated self pairs and static-static geometry require their
            # own topology policy. Cross arms, payload-arm, payload-payload
            # and all moving-environment pairs are explicitly covered here.
            if a.group == b.group or (a.robot is None and b.robot is None and
                                      a.group.startswith('static:') and b.group.startswith('static:')):
                continue
            pairs += 1
            low_a, high_a = bounds[i]; low_b, high_b = bounds[j]
            axis_gap = np.maximum(low_a-high_b, low_b-high_a)
            aabb_lower_bound = float(np.linalg.norm(np.maximum(axis_gap, 0)))
            margin = (a.contact_offset or 0) + (b.contact_offset or 0)
            if closest is not None and aabb_lower_bound > max(self.proximity + margin, closest['distance']):
                continue
            obj_a = self.shapes[i][0]; obj_b = self.shapes[j][0]
            collision = fcl.CollisionResult()
            hit = fcl.collide(obj_a, obj_b, fcl.CollisionRequest(num_max_contacts=1, enable_contact=True), collision)
            if hit:
                distance = -max((float(c.penetration_depth) for c in collision.contacts), default=0.)
            else:
                distance = float(fcl.distance(obj_a, obj_b, fcl.DistanceRequest(), fcl.DistanceResult()))
            if not np.isfinite(distance):
                raise ValueError(f'Invalid distance for {a.prim} versus {b.prim}')
            from types import SimpleNamespace
            event = SimpleNamespace(entity_a=a.entity, entity_b=b.entity, distance=distance)
            decision = policy.classify(event, task, snapshot=snapshot) if policy is not None else None
            classification = decision.classification if decision else ('collision' if hit else 'proximity')
            item = {'step': int(step), 'entity_a': a.entity, 'entity_b': b.entity,
                    'collider_a': a.prim, 'collider_b': b.prim,
                    'distance': distance, 'contact_offset_sum_m': margin,
                    'classification': classification,
                    'kind': 'authored_collider_convex_enclosure',
                    'reason': decision.reason if decision else 'geometric_candidate',
                    'native_pair_impulse_verified': False}
            if classification != 'allowed_contact' and (closest is None or distance < closest['distance']):
                closest = item
            if distance <= self.proximity + margin:
                events.append(item)
        return {'step': int(step), 'pairs_checked': pairs, 'events': events, 'closest_nonexempt_pair': closest}

    def coverage(self):
        return {'geometry_mode': 'convex_enclosure_of_enabled_authored_colliders',
                'colliders': [{'prim': c.prim, 'body': c.body, 'entity': c.entity,
                               'group': c.group, 'robot': c.robot,
                               'approximation': c.approximation, 'contact_offset_m': c.contact_offset}
                              for c in self.colliders],
                'unknown_contact_offset_colliders': [c.prim for c in self.colliders if c.contact_offset is None],
                'self_collision_checked': False, 'continuous_ccd_checked': False,
                'physx_cooked_decomposition_or_sdf_reconstructed': False}


def enumerate_usd_colliders(stage, roots):
    """roots: (prim path, group, robot name or None, tracked name or None)."""
    from pxr import Usd, UsdGeom, UsdPhysics
    try:
        from pxr import PhysxSchema
    except ImportError:
        PhysxSchema = None
    cache = UsdGeom.XformCache()
    colliders = []; missing = []; skipped = []
    for path, group, robot_name, object_name in roots:
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid():
            missing.append({'root': path, 'reason': 'missing_prim'}); continue
        found = 0
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                skipped.append({'prim': str(prim.GetPath()), 'reason': 'collision_disabled'}); continue
            owner = prim
            while owner and owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                if owner == root: break
                owner = owner.GetParent()
            if not owner or not owner.IsValid(): owner = root
            # Native body poses contain rotation/translation, not scale. Keep
            # any body/world scale in collider-local points instead of losing
            # it by removing the entire owner's scaled matrix.
            from pxr import Gf
            body_world = cache.GetLocalToWorldTransform(owner)
            rigid_world = Gf.Matrix4d(1)
            rigid_world.SetRotate(body_world.ExtractRotationQuat())
            rigid_world.SetTranslateOnly(body_world.ExtractTranslation())
            transform = cache.GetLocalToWorldTransform(prim) * rigid_world.GetInverse()
            if prim.IsA(UsdGeom.Mesh):
                source_points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            elif prim.IsA(UsdGeom.Cube):
                size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
                source_points = [[x, y, z] for x in [-size,size] for y in [-size,size] for z in [-size,size]]
            else:
                missing.append({'prim': str(prim.GetPath()), 'reason': 'unsupported_collider_type', 'type': str(prim.GetTypeName())}); continue
            points = np.asarray([list(transform.Transform(Gf.Vec3d(*map(float, p)))) for p in source_points])
            body = str(owner.GetPath())
            relative = body[len(path):].strip('/')
            entity = f'{robot_name}/{relative}' if robot_name else str(object_name)
            approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else prim.GetTypeName()
            offset = (PhysxSchema.PhysxCollisionAPI(prim).GetContactOffsetAttr().Get()
                      if PhysxSchema else prim.GetAttribute('physxCollision:contactOffset').Get())
            # Schema -inf means simulation-selected, not negative clearance.
            offset = float(offset) if offset is not None else None
            if offset is not None and (not np.isfinite(offset) or offset < 0):
                offset = None
            colliders.append(Collider(str(prim.GetPath()), body, entity, group, robot_name, points,
                                      str(approximation), offset))
            found += 1
        if not found:
            missing.append({'root': path, 'reason': 'no_enabled_supported_colliders'})
    return colliders, {'missing': missing, 'skipped': skipped}


class TaskEntityGeometryMonitor:
    def __init__(self, task):
        import omni.usd
        from .runtime_monitor import RuntimeConstraintMonitor, _load_xform_reader_cls
        self.task = task
        self.roots = []
        for name, robot in task.robots.items():
            path = RuntimeConstraintMonitor._robot_root_prim_path(robot)
            if path: self.roots.append((path, f'robot:{name}', str(name), None))
        tracked = getattr(task, '_object_prims', {}) or {}
        for name, prim in tracked.items():
            self.roots.append((str(prim.GetPath()), f'object:{name}', None, str(name)))
        for name, obj in (getattr(task, 'objects', {}) or {}).items():
            if name in tracked: continue
            config = getattr(obj, 'config', None)
            # UsdObject configs do not serialize a collider flag. Inspect the
            # referenced prim's enabled CollisionAPI rather than omitting it.
            if config and getattr(config, 'prim_path', None):
                self.roots.append((str(config.prim_path), f'static:{name}', None, str(name)))
        self.stage = omni.usd.get_context().get_stage()
        colliders, self.gaps = enumerate_usd_colliders(self.stage, self.roots)
        self.checker = EntityGeometryChecker(colliders)
        reader_cls = _load_xform_reader_cls()
        self.readers = {c.body: reader_cls(prim_path=c.body, reset_xform_properties=False) for c in colliders}
        self._native_readers = {}
        self.body_robot_links = {c.body: (c.robot, c.entity.split('/',1)[1]) for c in colliders if c.robot}
        self.steps = 0; self.events = []; self.minimum = None; self.errors = []
        self.pair_minima = {}; self.events_dropped = 0

    def _native_body(self, robot_name, body_path):
        # Suffix lookups can return the gripper base for the arm base_link.
        # Bind and verify the exact full USD body path instead.
        robot = self.task.robots[robot_name]
        cache = getattr(robot, '_rigid_body_map', {}) or {}
        native = cache.get(body_path)
        if native is None:
            native = self._native_readers.get(body_path)
            if native is None:
                from isaacsim.core.prims import SingleRigidPrim
                from isaacsim.core.simulation_manager import SimulationManager
                from types import SimpleNamespace
                # Bind existing rigid bodies without SDK's default transform
                # normalization and without adding contact sensors or setters.
                reader = SingleRigidPrim(prim_path=body_path, name=body_path,
                                         reset_xform_properties=False)
                view = SimulationManager.get_physics_sim_view()
                if view is None:
                    raise ValueError(f'No native simulation view for {body_path}')
                reader.initialize(physics_sim_view=view)
                native = SimpleNamespace(unwrap=lambda reader=reader: reader,
                                         get_pose=reader.get_world_pose)
                self._native_readers[body_path] = native
        if str(native.unwrap().prim_path) != body_path:
            raise ValueError(f'Native body path alias: {body_path} -> {native.unwrap().prim_path}')
        return native

    def current_poses(self):
        poses = {}
        objects = {str(prim.GetPath()): name for name,prim in (getattr(self.task,'_object_prims',{}) or {}).items()}
        for body, reader in self.readers.items():
            if body in self.body_robot_links:
                robot, _ = self.body_robot_links[body]
                native = self._native_body(robot, body)
                if not native.unwrap()._rigid_prim_view.is_physics_handle_valid():
                    raise ValueError(f'Invalid native rigid body handle {body}')
                p, q = native.get_pose()
            elif body in objects:
                native = (getattr(self.task, '_resolved_objects', {}) or {}).get(objects[body])
                if native is None:
                    raise ValueError(f'Missing existing object body reader {body}')
                if str(native.unwrap().prim_path) != body:
                    raise ValueError(f'Object body path alias: {body}')
                p, q = native.get_pose()
            else:
                p, q = reader.get_world_pose()
            poses[body] = pose_matrix(p, q)
        return poses

    def observe(self, step, policy):
        poses = self.current_poses()
        result = self.checker.check(poses, step=step, policy=policy, task=self.task)
        self.steps += 1
        closest = result['closest_nonexempt_pair']
        if closest and (self.minimum is None or closest['distance'] < self.minimum['distance']): self.minimum = closest
        self.events.extend(result['events'])
        overflow = max(len(self.events)-5000, 0)
        if overflow:
            self.events = self.events[overflow:]; self.events_dropped += overflow
        for event in result['events']:
            key = event['collider_a'] + ' | ' + event['collider_b']
            if key not in self.pair_minima or event['distance'] < self.pair_minima[key]['distance']:
                self.pair_minima[key] = event
        trace = os.environ.get('BEAM_ENTITY_GEOMETRY_TRACE_PATH')
        if trace:
            path = Path(trace); path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('a') as stream:
                stream.write(json.dumps({'phase': str(getattr(self.task,'phase','')),
                    'pose_source': 'live_exact_robot_wrappers; existing_object_rigid_wrappers; static_xform; no_hold_validation_getter',
                    'body_poses': {k:v.tolist() for k,v in poses.items()}, **result}) + '\n')
        return result

    def finalize(self):
        return {**self.checker.coverage(), **self.gaps, 'sampled_checks': self.steps,
                'minimum_nonexempt_pair': self.minimum, 'events': self.events,
                'events_retention': 'last_5000; uncapped_stream_when_configured',
                'events_dropped_from_summary': self.events_dropped,
                'pair_minima': list(self.pair_minima.values()),
                'geometry_errors': self.errors, 'full_entity_sweep_verified': False}
