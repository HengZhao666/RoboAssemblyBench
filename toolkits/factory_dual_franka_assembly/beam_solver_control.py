"""Explicit solver comparison for an isolated Beam simulation only.

The caller must establish isolation from the actual scene/environment setup.
This helper does not infer isolation from a recipe label and cannot modify a
mixed scene. It changes neither stabilization nor success/contact thresholds.
"""


class BeamSolverOverrideError(RuntimeError):
    def __init__(self, message, audit):
        super().__init__(message)
        self.audit = audit


def apply_beam_solver_override(physics_context, recipe_names, requested, *, isolated_scene=False):
    """Apply a requested PGS/TGS solver and require matching readback.

    None is a strict no-op: neither context getters/setters nor recipe iteration
    run. For an override, the runner must pass isolated_scene=True only after
    checking that its actual global physics scene is dedicated to Beam tasks.
    """
    audit = {'applied': False, 'changed': False, 'requested': requested}
    if requested is None:
        return dict(audit, reason='no_override_requested')
    if requested not in ('PGS', 'TGS'):
        raise ValueError('Beam solver override must explicitly request PGS or TGS')
    if isolated_scene is not True:
        raise ValueError('Beam solver override requires an explicitly isolated Beam scene')
    if isinstance(recipe_names, (str, bytes)) or recipe_names is None:
        raise ValueError('Beam solver override requires a nonempty collection of recipe names')
    names = list(recipe_names)
    if not names or any(not isinstance(name, str) or
                        not (name == 'fabrica_beam' or name.startswith('fabrica_beam_'))
                        for name in names):
        raise ValueError('Every recipe in the isolated scene must be a fabrica_beam task')
    audit.update(recipe_names=names, isolated_scene=True, isolation_evidence='caller_scene_check')
    try:
        original = physics_context.get_solver_type()
        audit['original'] = original
        if original not in ('PGS', 'TGS'):
            raise ValueError(f'Unknown solver readback before override: {original!r}')
        if requested == 'PGS':
            gpu_dynamics = physics_context.is_gpu_dynamics_enabled()
            if gpu_dynamics is None:
                raise ValueError('GPU dynamics readback unavailable for a PGS override')
            audit['gpu_dynamics_enabled'] = bool(gpu_dynamics)
            if gpu_dynamics:
                raise ValueError('PGS override requires GPU dynamics already disabled; no GPU setting was changed')
        if original == requested:
            audit.update(applied=True, readback=original, reason='already_requested_solver')
            return audit
        audit['setter_called'] = True
        audit['changed'] = None  # Until readback, a failing setter may have partially changed state.
        physics_context.set_solver_type(requested)
        readback = physics_context.get_solver_type()
        audit['readback'] = readback
        audit['changed'] = readback != original if readback in ('PGS', 'TGS') else None
        if readback != requested:
            raise ValueError(f'Beam solver readback {readback!r} does not match requested {requested!r}')
    except Exception as exc:
        audit['error'] = f'{type(exc).__name__}: {exc}'
        raise BeamSolverOverrideError(audit['error'], audit) from exc
    audit['applied'] = True
    return audit
