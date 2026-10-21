"""Strictly reuse D01's unchanged pickup calculations, not its native result."""
import ast,copy,json,sys,tarfile
from pathlib import Path
from beam_workflow import save,sha
p=Path(__file__).resolve().parents[1];cycle=sys.argv[1];a=p/'assisted_repository'
sk='toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
def method(source,name):return next(n for n in ast.walk(ast.parse(source)) if isinstance(n,ast.FunctionDef) and n.name==name)
def dump(n):return ast.dump(n,include_attributes=False)
archive=p/'versions/assisted_v20_before_A17_beam_full.tar.gz'
seal=json.loads((p/'A17_beam_full_sealed_version.json').read_text())
expected=next(v['sha256'] for v in seal['archives'] if Path(v['path']).name==archive.name)
if sha(archive)!=expected:raise ValueError('D01 source archive changed')
with tarfile.open(archive) as bundle:old=bundle.extractfile(sk).read().decode()
current=(a/sk).read_text();old_helper=method(old,'_assisted_phase_handoff');new_helper=method(current,'_assisted_phase_handoff')
branches=[n for n in new_helper.body if isinstance(n,ast.If) and "spec.get('beam_assisted_descent_admission')" in ast.unparse(n.test)]
if len(branches)!=1:raise ValueError('New helper differs beyond one gated-only branch')
new_helper.body.remove(branches[0])
if dump(new_helper)!=dump(old_helper):raise ValueError('Pickup handoff math changed')
oldcfg=json.loads((p/'D01_actual_configuration_layout505_config.json').read_text())
cfg=json.loads((p/(cycle+'_actual_configuration_layout505_config.json')).read_text())
oldph={v['name']:v for v in oldcfg['phase_specs']};ph={v['name']:v for v in cfg['phase_specs']}
names=[n for n in ph if n.startswith('assemble_') and n.endswith('_pickup_clearance')]
if len(names)!=4 or any(ph[n]!=oldph[n] or ph[n]['local_skill'].get('beam_assisted_descent_admission') is not None for n in names):raise ValueError('Pickup phase inputs changed')
delta=json.loads((p/'D01_delta.json').read_text());matches=[r for c in delta['checks'].values() for r in c['reports'] if Path(r['path']).name=='D01_handoff_geometry.json']
geometry=p/'D01_handoff_geometry.json'
if not matches or any(v['sha256']!=sha(geometry) for v in matches):raise ValueError('D01 geometry proof changed')
g=json.loads(geometry.read_text())
if not(g['passed'] and g['fixtures']==13 and g['full_mesh']['samples']==975 and g['full_mesh']['meshes']==54 and g['full_mesh']['self_checked']):raise ValueError('D01 pickup coverage incomplete')
save(p/(cycle+'_pickup_reuse.json'),dict(passed=True,phases=names,phase_count=4,fixtures=13,full54_self_states=975,
    source_archive_sha256=sha(archive),original_geometry_sha256=sha(geometry),current_helper_sha256=sha(a/sk),
    current_configuration_sha256=sha(p/(cycle+'_actual_configuration_layout505_config.json')),tool_sha256=sha(__file__),
    scope='Exactly unchanged nongated pickup helper/phase inputs. New gated-only branch is inapplicable. New assembly/hover/cold-insertion edges have independent replay.',native_result_reused=False))
print(json.dumps(dict(passed=True,pickup_phases=4,fixtures=13,reused_mesh_states=975)))
