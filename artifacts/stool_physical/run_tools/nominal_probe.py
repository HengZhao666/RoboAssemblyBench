import json, sys, math
sys.path.insert(0, '.')
from roboassemblybench.core.task_registry import load_task_recipe
p = load_task_recipe('fabrica_stool_circular_ur5e_physical')
rows=[json.loads(l) for l in open(sys.argv[1])]
meas = rows[1]['objects']
def qmul(a,b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return [w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2]
for o in p['objects']:
    n=o['name']
    if n in meas:
        q0=o.get('orientation',[1,0,0,0]); q1=meas[n]['orientation']
        d=qmul([q0[0],-q0[1],-q0[2],-q0[3]], q1)
        ang=2*math.degrees(math.acos(min(1,abs(d[0]))))
        dp=[round(a-b,4) for a,b in zip(meas[n]['position'], o['position'])]
        print(n, 'nominal', [round(v,4) for v in o['position']], 'dpos', dp, 'rot_from_nominal_deg=%.1f'%ang, o.get('collision_approximation'), o.get('spawn_clearance'))
    if n in ('fabrica_fixture','fixture_support'):
        print(n, {k:o.get(k) for k in ('position','collision_approximation','kinematic_anchor','usd_path')})
