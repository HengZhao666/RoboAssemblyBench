"""Native62: v239 = v238 with a 0.2 mm handover unload lift and completion tolerance max(lift/2, 0.4 mm) (batch 62-66, 2026-11-24); full diagnostics as round 59; v237 = v236 plus a left-hand lift of column 2 right after the handover grasp (human instruction 2026-11-20), lightweight traces; v236 = no intermediate five-part windows after part 0, v234 fast motion, part-0 release on target + final installed-assembly support load (human-approved) on v232, optimised worker with velocity cache; other physical and approved pose gates unchanged."""
from pathlib import Path
import hashlib
import json
import os
import runpy
import sys

R=Path('/home/user/beam_integration_20261010_upstream6f57dae');C=R/'candidate_repository'
Q=R/'grasp_repairs_20261012_20rounds/round62_v239_unload';TOOLS=Q/'tools'
CONFIG_SHA='3bf5d839e72c9d3751d10c265667d377b5efacade3a8ce24b19309c74592f100'
RECIPE_FP='8a437a4145eb9fcad37f1c8ca3f9e975ec79f401ffc4fc6729f9c11d39371d0f'
SOURCE_SHA='f173bbde0bb71f6d9808d991b5a43ac18787ec62494117ad06dce7a80b26da72'
ASSET_SHA='34cbf157e78f8b137fcec80c887bf4a9d2d2b75d1c592a6eca35e519a0806eaf'
SCENE_SHA='f07922ddc20bcaff23b404dbe09e30610972d2bfb25202ac7e76b65944550f65'
APPROVED={'controller':'d4e6f553ae63ed358860e8e63c99e683d14fe74a4ed971f1dabaf7674b8a3e3b',
          'compiler':'3bc2faa6b494300dd89eacd9c484a822e1958ab6163b5a4feedd05d91f4e89cc',
          'physical_compiler':'8317ed79537f75bf837ea7a8d8f32be119d3c850dc648c92a1b7b8d29aaa2b5d',
          'support_state':'8065411d711d4543eeb2554476ab1c7c9037b2baeb9b6e29a33fa88f47c7b8eb'}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_identity():
    assert sha(Q/'source_manifest.json')==SOURCE_SHA
    for f in json.loads((Q/'source_manifest.json').read_text())['files']:assert sha(C/f['path'])==f['sha256'],f['path']
    assert sha(Q/'production_worker_config.json')==CONFIG_SHA
    assert sha(Q/'asset_manifest.json') == ASSET_SHA
    assert sha(Q/'scene_asset_signatures.json') == SCENE_SHA
    assets = json.loads((Q/'asset_manifest.json').read_text())
    assert len(assets['files']) == 2
    for item in assets['files'] + assets['negative_control_assets']:
        assert sha(item['path']) == item['sha256'], item['path']
    assert os.environ.get('BEAM_COMMON_POSE_WINDOW') == '1'
    assert os.environ.get('BEAM_FINAL_POSE_ACCEPTANCE') == '1'
    assert os.environ.get('BEAM_CAP0_PICKUP_DEPTH_M') == '-0.0015'
    assert os.environ.get('BEAM_PART0_WINDOW_SPEEDUP') == '1'
    assert os.environ.get('BEAM_FAST_MOTION') == '1' and os.environ.get('BEAM_FINAL_ASSEMBLY_LOAD') == '1'
    assert os.environ.get('BEAM_PART0_RELEASE_AFTER_CLEAR') == '1'
    assert os.environ.get('BEAM_RELEASE_WITHOUT_STACK_WINDOW') == '1'
    assert os.environ.get('BEAM_HANDOVER_UNLOAD_LIFT') == '1'
    assert os.environ.get('BEAM_HANDOVER_UNLOAD_LIFT_M') == '0.0002'
    assert all(os.environ.get(k) == '1' for k in ('BEAM_OPT_PROBE_TRACE','BEAM_OPT_FASTCOPY','BEAM_OPT_FRAME_CACHE','BEAM_OPT_VELOCITY_CACHE'))
    assert os.environ.get('ROBOASSEMBLYBENCH_FINGERPRINT_REPO_ROOT') is None
    from toolkits.factory_dual_franka_assembly import plumbers_block_ur5e_skills as controller
    from roboassemblybench.core import beam_coordinated as compiler
    from roboassemblybench.core import beam_physical_compiler as physical_compiler
    from toolkits.factory_dual_franka_assembly import beam_support_state as support_state
    modules={}
    for name,module in [('controller',controller),('compiler',compiler),('physical_compiler',physical_compiler),('support_state',support_state)]:
        path=Path(module.__file__).resolve();assert path.is_relative_to(C) and sha(path)==APPROVED[name]
        modules[name]={'path':str(path),'sha256':sha(path)}
    return modules

def verify_scene_assets(task, prim_lookup=None):
    """Read the composed scene only; no setter, physics view, or extra reset."""
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics
    assert sha(Q/'scene_asset_signatures.json') == SCENE_SHA
    expected = json.loads((Q/'scene_asset_signatures.json').read_text())['files']
    observed = []
    for item in expected:
        name = item['object']
        prim = task._object_prims.get(name)
        if prim is None:
            # Object resolution is lazy. Read its existing scene path without
            # populating task caches or creating native body/contact views.
            path = task._scene.get(task.objects[name].name).unwrap().prim_path
            if prim_lookup is None:
                from isaacsim.core.utils.prims import get_prim_at_path
                prim_lookup = get_prim_at_path
            prim = prim_lookup(path)
        assert prim and prim.IsValid(), name
        collisions = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                      if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
        assert len(collisions) == 1, (name, len(collisions))
        cp = collisions[0]; mesh = UsdGeom.Mesh(cp)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
        indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        assert len(points) == item['points'] and len(counts) == item['faces']
        assert np.all(counts == 3)
        assert hashlib.sha256(points.tobytes()).hexdigest() == item['points_sha256']
        assert hashlib.sha256(indices.tobytes()).hexdigest() == item['indices_sha256']
        assert UsdPhysics.MeshCollisionAPI(cp).GetApproximationAttr().Get() == 'sdf'
        assert cp.GetAttribute('physxSDFMeshCollision:sdfResolution').Get() == 500
        observed.append(dict(item, actual_prim_path=str(cp.GetPath()), authored_sdf_resolution=item['sdf_resolution'], sdf_resolution=500))
    return {'passed': True, 'assets': observed, 'read_only': True,
            'scope': 'Actual composed USD collision arrays after original reset; not cooked-contact-response proof.'}

def verify_solver(env):
    assert os.environ.get('BEAM_PHYSICS_SOLVER_TYPE') == 'TGS'
    assert os.environ.get('BEAM_EXTERNAL_FORCES_EVERY_ITERATION') == '0', 'Original TGS external-force setting required'
    assert env.is_remote is False and env.env_num == env.proc_num == 1
    runner = env.runner
    assert runner.env_num == 1 and not runner.current_tasks, 'Tasks must not be reset before solver admission'
    world = runner._world
    context = world.get_physics_context()
    assert context.get_solver_type() == 'TGS', 'Requested TGS was not applied'
    gpu = context.is_gpu_dynamics_enabled()
    assert gpu is not None and not bool(gpu), 'TGS requires existing CPU dynamics'
    assert abs(float(context.get_physics_dt()) - 1/240) < 1e-12
    assert context.is_stablization_enabled() is False
    assert context._physx_scene_api.GetEnableExternalForcesEveryIterationAttr().Get() is False
    return {'passed': True, 'requested_solver': 'TGS', 'actual_solver': 'TGS',
            'gpu_dynamics_enabled': bool(gpu), 'global_stabilization': False,
            'physics_dt': float(context.get_physics_dt()), 'before_first_task_reset': True,
            'external_forces_every_iteration': False,
            'task_count': len(runner.current_tasks), 'scope': 'fresh isolated Beam environment'}


def guarded(original):
    def build(*args,**kwargs):
        configs=kwargs.get('task_configs',args[0] if args else [])
        assert len(configs)==1
        cfg=configs[0].model_dump();raw=(json.dumps(cfg,indent=2)+chr(10)).encode()
        assert hashlib.sha256(raw).hexdigest()==CONFIG_SHA,'Actual runner config differs before environment build'
        assert cfg['recipe_fingerprint']==RECIPE_FP
        assert os.environ.get('BEAM_PHYSICS_SOLVER_TYPE') == 'TGS', 'Runtime solver candidate mismatch'
        assert os.environ.get('BEAM_EXTERNAL_FORCES_EVERY_ITERATION') == '0', 'Original TGS external-force setting required before builder'
        modules=verify_identity()
        out=Path(os.environ['BEAM_CONTACT_TRACE_PATH']).parent
        evidence={'passed':True,'before_original_build_env':True,'native_step_occurred_in_this_guard':False,
            'worker_config_sha256':CONFIG_SHA,'recipe_fingerprint':RECIPE_FP,'source_manifest_sha256':SOURCE_SHA,'loaded_modules':modules}
        (out/'runner_config_admission.json').write_text(json.dumps(evidence,indent=2)+chr(10))
        env = original(*args,**kwargs)
        solver = verify_solver(env)
        (out/'runtime_solver_admission.json').write_text(json.dumps(solver,indent=2)+chr(10))
        original_reset = env.reset
        def reset(*reset_args, **reset_kwargs):
            observations = original_reset(*reset_args, **reset_kwargs)
            context = env.runner._world.get_physics_context()
            assert context.get_solver_type() == 'TGS' and not bool(context.is_gpu_dynamics_enabled())
            assert context.is_stablization_enabled() is False
            assert abs(float(context.get_physics_dt()) - 1/240) < 1e-12
            assert context._physx_scene_api.GetEnableExternalForcesEveryIterationAttr().Get() is False
            receipt = out/'native_solver_after_reset.json'
            if not receipt.exists():
                tasks = env.runner.current_tasks
                assert len(tasks) == 1
                evidence = dict(solver, before_first_task_reset=False,
                    after_first_original_reset=True, task_count=len(tasks),
                    physics_stamps=[t._beam_physics_stamp() for t in tasks.values()])
                assets = verify_scene_assets(next(iter(tasks.values())))
                (out/'native_scene_asset_admission.json').write_text(json.dumps(assets,indent=2)+chr(10))
                receipt.write_text(json.dumps(evidence,indent=2)+chr(10))
            return observations
        env.reset = reset
        return env
    return build

def main():
    sys.path[:0]=[str(TOOLS),str(C)]
    verify_identity()
    baseline=R/'grasp_repairs_20261012_20rounds/round24_native_release/full_assembly_worker.py'
    assert sha(baseline)=='cb2519d4635a871eab2310e2683bb48a549e46558caa5033568687e9ba6a8f95'
    from toolkits.factory_dual_franka_assembly import generate_demos as demo
    demo._build_env=guarded(demo._build_env)
    # The unchanged worker and its baseline observer wrap this guarded builder;
    # their complete dispatch/physics/capture paths remain in place.
    runpy.run_path(str(TOOLS/'worker.py'),run_name='__main__')

if __name__=='__main__':main()
