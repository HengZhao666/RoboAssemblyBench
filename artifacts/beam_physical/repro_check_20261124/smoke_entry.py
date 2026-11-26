"""Smoke check of the round-62 launch chain (not a formal round).

Runs checked_worker.py's identity/config/solver/asset guards and the
round-62 optimised worker, then marks the task failed with reason
'smoke-step-limit' at BEAM_SMOKE_STOP_STEP. Output goes to a scratch folder.
"""
import os
import runpy
import sys
from pathlib import Path

Q = Path(__file__).resolve().parent
STOP = int(os.environ['BEAM_SMOKE_STOP_STEP'])
scope = {'__name__': 'checked_worker_smoke', '__file__': str(Q / 'checked_worker.py')}
exec(compile((Q / 'checked_worker.py').read_text(), str(Q / 'checked_worker.py'), 'exec'), scope)


def stop_wrap(inner):
    def build(*args, **kwargs):
        env = inner(*args, **kwargs)
        from internutopia_extension.tasks.factory_dual_franka_assembly_task import (
            FactoryDualFrankaAssemblyTask as Task)
        original = Task._update_task_state

        def update(task):
            result = original(task)
            if int(task.step_counter) >= STOP and not task.failed and not task.success:
                task._set_terminal_state('failed', reason='smoke-step-limit', status='failed',
                                         transition_type='failure', detail={'stop_step': STOP})
            return result

        Task._update_task_state = update
        return env
    return build


sys.path[:0] = [str(Q / 'tools'), str(scope['C'])]
scope['verify_identity']()
from toolkits.factory_dual_franka_assembly import generate_demos as demo  # noqa: E402
demo._build_env = stop_wrap(scope['guarded'](demo._build_env))
runpy.run_path(str(Q / 'tools/worker.py'), run_name='__main__')
