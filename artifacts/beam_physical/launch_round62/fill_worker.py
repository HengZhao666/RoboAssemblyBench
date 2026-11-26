"""Fill checked_worker.py from its template with the staged round-62 identities.

Run on the server after setup_round62.py and tools/preflight.py. The worker
config SHA must equal the one offline_v239_unload/config_check.json built.
"""
import hashlib
import json
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
Q = B / 'round62_v239_unload'
P = B / 'offline_v239_unload'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    check = json.loads((P / 'config_check.json').read_text())
    assert check['passed']
    config = Q / 'production_worker_config.json'
    assert sha(config) == check['candidate_config_sha256'], 'preflight config differs from the offline check'
    values = {
        '@CONFIG_SHA@': sha(config),
        '@RECIPE_FP@': json.loads(config.read_text())['recipe_fingerprint'],
        '@SOURCE_SHA@': sha(Q / 'source_manifest.json'),
        '@PHYSICAL_COMPILER_SHA@': sha(C / 'roboassemblybench/core/beam_physical_compiler.py'),
        '@SUPPORT_STATE_SHA@': sha(C / 'toolkits/factory_dual_franka_assembly/beam_support_state.py'),
    }
    assert values['@PHYSICAL_COMPILER_SHA@'] == sha(P / 'source/roboassemblybench/core/beam_physical_compiler.py')
    assert values['@SUPPORT_STATE_SHA@'] == sha(P / 'source/toolkits/factory_dual_franka_assembly/beam_support_state.py')
    text = (Q / 'checked_worker.py.template').read_text()
    for key, value in values.items():
        assert text.count(key) == 1, key
        text = text.replace(key, value)
    assert '@' not in text.split('"""', 2)[2].replace('@property', '')
    (Q / 'checked_worker.py').write_text(text)
    print(json.dumps(dict(values, checked_worker_sha256=sha(Q / 'checked_worker.py'))))


if __name__ == '__main__':
    main()
