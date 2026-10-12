"""Initialization-only external-force comparison for one isolated Beam scene.

No simulator imports are needed here. The caller supplies the existing physics
context and World stage before simulation starts. No schemas/attributes are
created, and no GPU, solver, body, controller or acceptance setting is changed.
"""

EXTERNAL_FORCE_ATTRIBUTE = 'physxScene:enableExternalForcesEveryIteration'


class BeamExternalForceOverrideError(RuntimeError):
    def __init__(self, message, audit):
        super().__init__(message)
        self.audit = audit


def _context_scene_prim(physics_context, stage):
    # Isaac Sim 5.1 exposes prim_path from its actual _prim_path. Cross-check
    # against its existing PhysxSceneAPI; never guess a default scene path.
    path = physics_context.prim_path
    if not isinstance(path, str) or not path.startswith('/') or stage is None:
        raise ValueError('Physics context has no valid public scene path/World stage')
    prim = stage.GetPrimAtPath(path)
    schema = getattr(physics_context, '_physx_scene_api', None)
    getter = getattr(schema, 'GetPrim', None)
    if not schema or not callable(getter) or getter() != prim:
        raise ValueError('Existing context PhysxSceneAPI does not identify its public scene path')
    return prim, 'World.stage.GetPrimAtPath(physics_context.prim_path)', path


def _gpu_readback(physics_context):
    value = physics_context.is_gpu_dynamics_enabled()
    if type(value) is not bool:
        raise ValueError(f'GPU dynamics state is not a boolean readback: {value!r}')
    return value


def _tgs_readback(physics_context, audit, key):
    value = physics_context.get_solver_type()
    audit[key] = value if isinstance(value, str) or value is None else repr(value)
    if value != 'TGS':
        raise ValueError(f'External-force iteration experiment requires a TGS readback: {value!r}')
    return value


def _boolean_attribute_readback(attribute, name):
    value = attribute.Get()
    if type(value) is not bool:
        raise ValueError(f'{name} has no boolean USD readback: {value!r}')
    return value


def apply_beam_external_force_override(
    physics_context, stage, recipe_names, requested, *, isolated_scene=False,
):
    """Apply explicit '0'/'1' once before stepping, requiring exact USD readback.

    None is a strict no-op, including recipe iteration and all stage/context
    access. GPU dynamics may be either enabled or disabled; its live setting is
    recorded before/after and never written. A failed explicit experiment raises
    with its audit instead of allowing the simulation to proceed unverified.
    """
    audit = {'applied': False, 'changed': False, 'requested': requested,
             'attribute': EXTERNAL_FORCE_ATTRIBUTE,
             'configuration_source': 'existing_usd_scene_attribute',
             'native_solver_effect_verified': False}
    if requested is None:
        return dict(audit, reason='no_override_requested')
    if requested not in ('0', '1'):
        raise ValueError('Beam external-force override must explicitly request 0 or 1')
    desired = requested == '1'
    if isolated_scene is not True:
        raise ValueError('Beam external-force override requires an explicitly isolated Beam scene')
    if isinstance(recipe_names, (str, bytes)) or recipe_names is None:
        raise ValueError('Beam external-force override requires a collection of recipe names')
    names = list(recipe_names)
    if not names or any(not isinstance(name, str) or
                        not (name == 'fabrica_beam' or name.startswith('fabrica_beam_'))
                        for name in names):
        raise ValueError('Every recipe in the isolated scene must be a fabrica_beam task')
    audit.update(recipe_names=names, isolated_scene=True,
                 isolation_evidence='caller_scene_check_and_single_stage_physics_scene',
                 requested_enabled=desired)
    try:
        prim, source, path = _context_scene_prim(physics_context, stage)
        audit['scene_prim_source'] = source
        audit['scene_prim_path'] = path
        audit['scene_schema_source'] = 'physics_context._physx_scene_api.GetPrim'
        if not prim.IsValid() or str(prim.GetTypeName()) != 'PhysicsScene':
            raise ValueError('Physics context does not refer to a valid PhysicsScene prim')
        if str(prim.GetPath()) != path:
            raise ValueError('Physics context public scene path does not match the actual prim')
        if stage is None or prim.GetStage() != stage or stage.GetPrimAtPath(path) != prim:
            raise ValueError('Physics context scene prim does not belong to the World stage')
        scenes = [str(candidate.GetPath()) for candidate in stage.Traverse()
                  if candidate.IsValid() and str(candidate.GetTypeName()) == 'PhysicsScene']
        audit['stage_physics_scene_paths'] = scenes
        if scenes != [path]:
            raise ValueError('External-force experiment requires exactly the context PhysicsScene in the stage')
        attribute = prim.GetAttribute(EXTERNAL_FORCE_ATTRIBUTE)
        if attribute is None or not attribute.IsValid() or str(attribute.GetTypeName()) != 'bool':
            raise ValueError(f'Existing boolean schema attribute unavailable: {EXTERNAL_FORCE_ATTRIBUTE}')
        audit['readback_source'] = f'{source}.GetAttribute.Get'
        original = _boolean_attribute_readback(attribute, EXTERNAL_FORCE_ATTRIBUTE)
        audit['original'] = original
        _tgs_readback(physics_context, audit, 'solver_type')
        audit['solver_type_source'] = 'physics_context.get_solver_type'
        gpu = _gpu_readback(physics_context)
        audit['gpu_dynamics_enabled'] = gpu
        audit['gpu_dynamics_source'] = 'physics_context.is_gpu_dynamics_enabled'
        if original != desired:
            audit['setter_called'] = True
            audit['changed'] = None  # A failing USD setter can have changed state.
            setter_result = attribute.Set(desired)
            audit['setter_result'] = setter_result if type(setter_result) is bool else repr(setter_result)
            readback = _boolean_attribute_readback(attribute, EXTERNAL_FORCE_ATTRIBUTE)
            audit['readback'] = readback
            audit['changed'] = readback != original
            if setter_result is not True:
                raise ValueError('USD attribute setter did not confirm a successful write')
            if readback != desired:
                raise ValueError(f'External-force readback {readback!r} does not match requested {desired!r}')
        else:
            audit.update(readback=original, reason='already_requested_value')
        gpu_after = _gpu_readback(physics_context)
        audit['gpu_dynamics_readback_after'] = gpu_after
        if gpu_after != gpu:
            raise ValueError('GPU dynamics changed during the external-force experiment')
        _tgs_readback(physics_context, audit, 'solver_type_readback_after')
    except Exception as exc:
        audit['error'] = f'{type(exc).__name__}: {exc}'
        raise BeamExternalForceOverrideError(audit['error'], audit) from exc
    audit['applied'] = True
    return audit
