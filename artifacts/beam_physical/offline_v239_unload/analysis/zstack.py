import json, gzip, os, collections
C='/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
RUNS={'r55':'fabrica_beam_v232_window','r59':'fabrica_beam_v236_no_window','r60':'fabrica_beam_v237_column2','r61':'fabrica_beam_v238_lift04'}
Z0=1.0025
def op(p): return open(p) if os.path.exists(p) else gzip.open(p+'.gz','rt')
for name,run in RUNS.items():
    last=collections.OrderedDict()
    with op(C+run+'/physics_boundary_trace.jsonl') as f:
        for line in f:
            if '"after_physics"' not in line[:300] and '"before_action"' not in line[:300]: continue
            i=line.find('"phase"'); ph=line[i+10:i+80].split('"')[0] if i>=0 else '?'
            r=json.loads(line)
            ph=r.get('phase')
            b={x['name']:x['positions'][0] for x in r['bodies'] if x['name'].startswith('fabrica_beam')}
            last.setdefault(ph,[]).append((r['task_step'],b))
    print('==',name)
    for ph,rows in last.items():
        if not any(k in ph for k in ('handover_left_2','hold_2','hold_3_to_2','part_0_insert_0','part_0_physical','part_0_release','part_0_retreat','final','terminal','failed')): continue
        for tag,(s,b) in (('first',rows[0]),('last',rows[-1])):
            if '2' not in ''.join(b): continue
            c2=b.get('fabrica_beam_2'); c0=b.get('fabrica_beam_0'); c6=b.get('fabrica_beam_6')
            print('  %-40s %-5s %6d col2_dz=%+.3f cap0_dz=%s base6_dz=%+.3f'%(ph[:40],tag,s,(c2[2]-Z0)*1000,('%+.3f'%((c0[2]-Z0)*1000)) if c0 else '-',(c6[2]-Z0)*1000))
