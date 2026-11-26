import json
C='/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
for run in ('fabrica_beam_v237_column2','fabrica_beam_v238_lift04'):
    tcp={}; z0=None
    with open(C+run+'/motion_trace.jsonl') as f:
        for line in f:
            if '"handover_left_2_unload_lift"' not in line[:200]: continue
            r=json.loads(line)
            if r.get('robot')!='franka_left': continue
            z=r['current_pose']['position'][2]; z0=z if z0 is None else z0; tcp[r['step']]=(z-z0)*1000
    rows=[]
    with open(C+run+'/table_support_trace.jsonl') as f: pass
    with open(C+run+'/assembly_integrity_trace.jsonl') as f:
        for line in f:
            s=int(line[9:line.index(',')])
            if s<45900: continue
            if s>46100: break
            r=json.loads(line); e={i['part']:i for i in r['interfaces']}
            c=e['2'].get('interface_contact') or {}
            rows.append((s,r['phase'][-12:],round(tcp.get(s,float('nan')),3),round(c.get('force') or 0,2),len(c.get('contacts') or [])))
    print(run); [print('  ',x) for x in rows]
