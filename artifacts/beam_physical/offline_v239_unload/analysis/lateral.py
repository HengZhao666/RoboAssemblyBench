import json, gzip, os, math, collections
C='/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/outputs/'
RUNS={'r55':'fabrica_beam_v232_window','r59':'fabrica_beam_v236_no_window'}
T=(0.7844409166438082,-0.12056654334103567,1.0025)
PH=('handover_left_2_park','hold_2_close_and_attach','assemble_03_part_0_insert_00','assemble_03_part_0_insert_09','assemble_03_part_0_physical_hold_check','assemble_03_part_0_release','assemble_03_part_0_park','hold_2_final_release','hold_2_final_park','beam_final_unloaded_verification')
def op(p): return open(p) if os.path.exists(p) else gzip.open(p+'.gz','rt')
def tilt(q):
    w,x,y,z=q; zz=1-2*(x*x+y*y); return math.degrees(math.acos(max(-1,min(1,zz))))*1000*math.pi/180  # mrad
def axis(q):
    w,x,y,z=q  # body z axis in world
    return (2*(x*z+w*y), 2*(y*z-w*x))
for name,run in RUNS.items():
    last={}
    with op(C+run+'/physics_boundary_trace.jsonl') as f:
        for line in f:
            if '"before_action"' not in line[:300] or not any('"%s"'%p in line[:400] for p in PH): continue
            r=json.loads(line)
            if r.get('phase') not in PH: continue
            last.setdefault(r['phase'],[]).append(r)
    print('==',name)
    for ph in PH:
        if ph not in last: continue
        for tag,r in (('first',last[ph][0]),('last',last[ph][-1])):
            b={x['name']:x for x in r['bodies'] if x['name'].startswith('fabrica_beam')}
            out=[]
            for k in ('fabrica_beam_2','fabrica_beam_0','fabrica_beam_3','fabrica_beam_1'):
                if k not in b: continue
                p=b[k]['positions'][0]; q=b[k]['orientations'][0]; ax=axis(q)
                out.append('%s dxy=(%+.3f,%+.3f)mm tilt=%.1fmrad toward(%+.2f,%+.2f)'%(k[-1],(p[0]-T[0])*1000,(p[1]-T[1])*1000,tilt(q),ax[0]*1000,ax[1]*1000))
            print('  %-38s %-5s %6d | %s'%(ph[:38],tag,r['task_step'],' | '.join(out)))
