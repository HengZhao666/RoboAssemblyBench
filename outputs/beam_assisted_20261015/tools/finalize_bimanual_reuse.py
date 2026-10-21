from pathlib import Path
import copy,json,hashlib,numpy as np
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
source=P/'C01_bimanual_repaired_path.jsonl'
rows=[json.loads(line) for line in source.open()]
assert rows[-1]['phase']=='beam_assisted_final_verification'
assert rows[-2]['phase']=='base_6_retreat'
assert not any(row['phase']=='base_6_park' for row in rows)
old_final=copy.deepcopy(rows[-1])
rows[-1]={**copy.deepcopy(rows[-2]),'phase':'beam_assisted_final_verification'}
jumps={name:max(float(np.max(abs(np.asarray(b['q'][name][:6])-np.asarray(a['q'][name][:6])))) for a,b in zip(rows,rows[1:])) for name in rows[0]['q']}
assert max(jumps.values())<=.12000001,jumps
destination=P/'C01_bimanual_repaired_v2_path.jsonl'
with destination.open('w') as stream:
 for row in rows:stream.write(json.dumps(row)+'\n')
report={'passed':True,'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
 'base_reuse_manifest':str(P/'C01_bimanual_reuse_manifest.json'),
 'base_reuse_manifest_sha256':hashlib.sha256((P/'C01_bimanual_reuse_manifest.json').read_bytes()).hexdigest(),
 'output':str(destination),'sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),'rows':len(rows),
 'fix':'final verification row inherits the valid right retreat endpoint, never the removed old park endpoint',
 'max_adjacent_arm_joint_delta_rad':jumps,
 'replaced_final_old_right_q':old_final['q']['franka_right'],'final_right_q':rows[-1]['q']['franka_right'],
 'final_geometry_equals_last_verified_retreat':rows[-1]['q']==rows[-2]['q'] and rows[-1]['part_poses']==rows[-2]['part_poses'],
 'formal_physical_success':False}
(P/'C01_bimanual_reuse_v2_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report))
