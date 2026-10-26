"""Load the archived v35 worker with portable geometry helpers and source roots.

The archived worker and control algorithms are intentionally unchanged. Only
deployment helpers are replaced; all native geometry/camera/SDK checks remain.
"""
from pathlib import Path
import importlib
import json
import os
import runpy
import sys

root = Path(os.environ['BEAM_ROUTE_SOURCE']).resolve()
tools = Path(os.environ['BEAM_ASSISTED_TOOLS']).resolve()
helpers = Path(__file__).resolve().parent
out = Path(os.environ['BEAM_ASSISTED_OUTPUT_DIR']).resolve()
sys.path[:0] = [str(root), str(helpers), str(tools)]
origins = {}
for name in ('sweep_model', 'visual_mesh_check',
             'toolkits.factory_dual_franka_assembly.beam_assisted_servo',
             'toolkits.factory_dual_franka_assembly.generate_demos',
             'roboassemblybench.core.beam_assisted_compiler'):
    module = importlib.import_module(name)
    path = Path(module.__file__).resolve()
    path.relative_to(root)
    origins[name] = str(path)
if Path(sys.modules['sweep_model'].__file__).resolve() != helpers / 'sweep_model.py':
    raise RuntimeError('Portable geometry helper was shadowed')
(out / 'runtime_import_binding.json').write_text(json.dumps(
    dict(source_root=str(root), modules=origins, passed=True), indent=2) + '\n')
runpy.run_path(str(tools / 'assisted_worker.py'), run_name='__main__')
