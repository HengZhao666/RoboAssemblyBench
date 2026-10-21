"""Source-bound camera-only configuration reuse, including the derived recipe fingerprint.

emit-payload must run in the run_checks --configuration-report environment.
No simulator is created and no production files are modified.
"""
import argparse
import copy
import hashlib
import json
import os
import sys
from pathlib import Path

EXPECTED_ENV = {
 'INTERNUTOPIA_EXTENSION_PROFILE':'ur5e_assembly','PYTHONNOUSERSITE':'1',
 'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'1','BEAM_COLUMN3_PICKUP_DEPTH_M':'-0.004',
 'BEAM_COLUMN3_TRANSPORT_RAISE_M':'0.16','BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N':'16,24',
 'BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S':'0.20','BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N':'16,24',
 'BEAM_LOADED_TARGET_HOLD_SECONDS':'0','BEAM_GRIPPER_SHELL_VISUALS':'1'}
RECIPE='fabrica_beam_ur5e_assisted'
PROFILE='taoyuan_grscenes_tabletop'
sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
read=lambda path:json.loads(Path(path).read_text())

def changes(x,y,path=''):
 if isinstance(x,dict) and isinstance(y,dict):
  out=[]
  for k in sorted(set(x)|set(y)):
   out += [path+'/'+str(k)] if k not in x or k not in y else changes(x[k],y[k],path+'/'+str(k))
  return out
 if isinstance(x,list) and isinstance(y,list) and len(x)==len(y):
  return [item for i,(a,b) in enumerate(zip(x,y)) for item in changes(a,b,path+'/'+str(i))]
 return [] if x==y else [path]

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--root',type=Path,default=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015'))
 sub=parser.add_subparsers(dest='command',required=True)
 emit=sub.add_parser('emit-payload');emit.add_argument('output')
 certify=sub.add_parser('certify');certify.add_argument('old_configuration');certify.add_argument('new_configuration')
 certify.add_argument('output');certify.add_argument('reports',nargs='+');certify.add_argument('--payload-evidence',required=True)
 args=parser.parse_args();p=args.root
 resolve=lambda v:Path(v) if Path(v).is_absolute() else p/v
 sys.path.insert(0,str(p/'assisted_repository'))
 from roboassemblybench.core.task_registry import load_task_recipe,task_recipe_fingerprint
 sources=[p/'assisted_repository/roboassemblybench/core'/v for v in
          ('task_registry.py','beam_assisted_compiler.py','beam_physical_compiler.py','fabrica_canonical.py')]
 if args.command=='emit-payload':
  env={k:os.environ.get(k) for k in EXPECTED_ENV}
  assert env==EXPECTED_ENV,('payload environment differs from actual configuration launcher',env)
  env['ROBOASSEMBLYBENCH_FINGERPRINT_REPO_ROOT']=os.environ.get('ROBOASSEMBLYBENCH_FINGERPRINT_REPO_ROOT')
  before={str(v):sha(v) for v in sources}
  payload=load_task_recipe(RECIPE,scene_profile=PROFILE);fp=task_recipe_fingerprint(payload)
  assert payload['recipe_fingerprint']==fp
  assert before=={str(v):sha(v) for v in sources}
  output=resolve(args.output)
  output.write_text(json.dumps({'kind':'actual_compiled_recipe_payload_for_fingerprint_replay',
   'recipe':RECIPE,'scene_profile':PROFILE,'simulator_created':False,'compiler_environment':env,
   'source_sha256':before,'payload':payload,'computed_fingerprint':fp},indent=2)+'\n')
  print(json.dumps({'passed':True,'payload_evidence':str(output),'sha256':sha(output),'computed_fingerprint':fp,'simulator_created':False}));return
 old_path,new_path,output=map(resolve,(args.old_configuration,args.new_configuration,args.output))
 old,new=read(old_path),read(new_path);old=old.get('task_configs',[old])[0];new=new.get('task_configs',[new])[0]
 assert {k:v for k,v in old.items() if k not in ('camera_metadata','recipe_fingerprint')}=={k:v for k,v in new.items() if k not in ('camera_metadata','recipe_fingerprint')}
 changed=changes(old,new)
 assert changed and all(v.startswith('/camera_metadata/') or v=='/recipe_fingerprint' for v in changed)
 proof_path=resolve(args.payload_evidence);proof=read(proof_path)
 assert proof['kind']=='actual_compiled_recipe_payload_for_fingerprint_replay' and proof['simulator_created'] is False
 assert proof['recipe']==RECIPE and proof['scene_profile']==PROFILE
 assert {k:proof['compiler_environment'].get(k) for k in EXPECTED_ENV}==EXPECTED_ENV
 for path,value in proof['source_sha256'].items():assert sha(Path(path))==value
 for key,value in proof['compiler_environment'].items():
  if value is None:os.environ.pop(key,None)
  else:os.environ[key]=value
 actual=load_task_recipe(RECIPE,scene_profile=PROFILE)
 assert actual==proof['payload'],'saved payload differs from actual compiled recipe'
 assert task_recipe_fingerprint(actual)==actual['recipe_fingerprint']==proof['computed_fingerprint']==new['recipe_fingerprint']
 shadow=copy.deepcopy(actual);camera_key='camera_specs' if 'camera_specs' in shadow else 'cameras'
 targets=[(i,v) for i,v in enumerate(shadow[camera_key]) if v.get('view_type')=='wrist' and v.get('robot',v.get('owner'))=='franka_right']
 assert len(targets)==1;index,right=targets[0]
 old_meta=[v for v in old['camera_metadata'] if v.get('view_type')=='wrist' and v.get('robot',v.get('owner'))=='franka_right']
 new_meta=[v for v in new['camera_metadata'] if v.get('view_type')=='wrist' and v.get('robot',v.get('owner'))=='franka_right']
 assert len(old_meta)==len(new_meta)==1
 adjusted_metadata=copy.deepcopy(old['camera_metadata'])
 adjusted_right=next(v for v in adjusted_metadata if v.get('view_type')=='wrist' and v.get('robot',v.get('owner'))=='franka_right')
 for field in ('mount_offset','native_mount_offset'):adjusted_right[field]=copy.deepcopy(new_meta[0][field])
 assert adjusted_metadata==new['camera_metadata'],'camera metadata changed beyond the right mounting offsets'
 for field in ('mount_offset','native_mount_offset'):
  assert right[field]==new_meta[0][field];right[field]=copy.deepcopy(old_meta[0][field])
 payload_changes=changes(actual,shadow)
 assert payload_changes and all(v.startswith('/'+camera_key+'/'+str(index)+'/mount_offset/') or v.startswith('/'+camera_key+'/'+str(index)+'/native_mount_offset/') for v in payload_changes)
 old_derived=task_recipe_fingerprint(shadow)
 assert old_derived==old['recipe_fingerprint'],('restoring only right camera does not recover old fingerprint',old_derived,old['recipe_fingerprint'])
 reports=list(map(resolve,args.reports))
 result={'passed':True,'kind':'exact_camera_only_configuration_reuse',
  'old_configuration':{'path':str(old_path),'sha256':sha(old_path)},'new_configuration':{'path':str(new_path),'sha256':sha(new_path)},
  'excluded_top_level_fields':['camera_metadata','recipe_fingerprint'],'changed_paths':changed,
  'reused_reports':{v.name:sha(v) for v in reports},'derived_recipe_fingerprint':{
   'method':'actual_compiled_payload_and_right_camera_counterfactual','payload_evidence':{'path':str(proof_path),'sha256':sha(proof_path)},
   'recipe':RECIPE,'scene_profile':PROFILE,'new_fingerprint':new['recipe_fingerprint'],'old_counterfactual_fingerprint':old_derived,
   'changed_payload_paths':payload_changes,'camera_payload_key':camera_key,'right_camera_payload_index':index,
   'right_camera_old_offsets':{k:old_meta[0][k] for k in ('mount_offset','native_mount_offset')},
   'right_camera_new_offsets':{k:new_meta[0][k] for k in ('mount_offset','native_mount_offset')},'source_sha256':proof['source_sha256']},
  'scope':'Every other actual-model field is equal. Recipe fingerprint is independently recomputed from actual compiled payload and right-camera-only counterfactual. Reused reports must still bind production source/geometry; fresh camera evidence required.',
  'simulator_created':False,'formal_physical_success':False}
 output.write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'passed':True,'certificate':str(output),'sha256':sha(output),'changed_paths':changed,'new_fingerprint':new['recipe_fingerprint'],'old_counterfactual_fingerprint':old_derived}))

if __name__=='__main__':main()
