import json, collections, gzip, os
def op(path):
    return open(path) if os.path.exists(path) else gzip.open(path+'.gz','rt')
C='/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
RUNS={'r55':'fabrica_beam_v232_window','r59':'fabrica_beam_v236_no_window','r60':'fabrica_beam_v237_column2'}
PH=('assemble_03_part_0_insert_08','assemble_03_part_0_insert_09','assemble_03_part_0_physical_hold_check','assemble_03_part_0_restore_gravity','assemble_03_part_0_clear_support_filters','assemble_03_part_0_release')
for name,run in RUNS.items():
    mot={}; frc={}
    with op(C+run+'/motion_trace.jsonl') as f:
        for line in f:
            if not any('"%s"'%p in line[:200] for p in PH): continue
            r=json.loads(line)
            if r.get('robot')!='franka_left': continue
            mot[r['step']]=(r['phase'][19:],r['current_pose']['position'][2],r['command_target_pose']['position'][2],r['target_pose']['position'][2])
    with op(C+run+'/loaded_force_trace.jsonl') as f:
        for line in f:
            if not any('"%s"'%p in line[:200] for p in PH): continue
            r=json.loads(line)
            if r.get('robot')!='franka_left': continue
            frc[r['step']]=[round(x,1) for x in (r.get('control') or {}).get('forces',[])]
    integ={}
    with op(C+run+'/assembly_integrity_trace.jsonl') as f:
        for line in f:
            if not any('"%s"'%p in line[:120] for p in PH): continue
            r=json.loads(line); e={i['part']:i for i in r['interfaces']}
            integ[r['step']]=(round(e['2']['relative_position_error']*1000,2), round(e['0']['relative_position_error']*1000,2) if '0' in e else None,
                              round((e['0'].get('interface_contact') or {}).get('force') or 0,2) if '0' in e else None)
    steps=sorted(mot)
    # find insert_09 start and hold check start
    s9=next(s for s in steps if mot[s][0]=='insert_09'); sh=next((s for s in steps if mot[s][0]=='physical_hold_check'),steps[-1])
    print('==',name,'insert_09',s9,'hold_check',sh,'insert_09 len',sh-s9)
    pick=[s for s in steps if s9-40<=s<=sh+40]
    zref=mot[sh][1]
    for s in pick[::max(1,len(pick)//22)]+[x for x in pick if sh-3<=x<=sh+4]:
        ph,cz,cmd,tz=mot[s]
        print('  %d %-22s tcp_dz=%+.3f cmd-tcp=%+.3f tgt-tcp=%+.3f F=%s col2/p0err/p0F=%s'%(s,ph,(cz-zref)*1000,(cmd-cz)*1000,(tz-cz)*1000,frc.get(s),integ.get(s - s%8)))
