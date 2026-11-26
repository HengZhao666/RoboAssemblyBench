"""Offline native Robotiq pad depth vs opening, from the authored USD colliders."""
import sys, numpy as np
sys.path.insert(0, '.')
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
from pxr import Usd, UsdPhysics
import traceback, builtins
_out=open("/tmp/pad_fk_result.txt","w")
def print(*a, **k):
    builtins.print(*a, file=_out, flush=True)
try:
    from roboassemblybench.core.grasp_clearance import NativeGripper
    path = sys.argv[1]
    stage = Usd.Stage.Open(path)
    root = stage.GetDefaultPrim()
    names = [str(p.GetPath()) for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if p.IsA(UsdPhysics.RevoluteJoint)]
    print('revolute joints', [n.rsplit('/',1)[-1] for n in names][-8:])
    base = [str(p.GetPath()) for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()) if str(p.GetPath()).endswith('Robotiq_2F_85/base_link')][0]
    m = NativeGripper(root, base, 'finger_joint')
    print('bodies', sorted({e['body'].rsplit('/',1)[-1] for e in m.meshes}))
    sel = {}
    for e in m.meshes:
        b = e['body'].rsplit('/',1)[-1]
        if b in ('left_inner_finger', 'right_inner_finger', 'left_inner_finger_pad', 'right_inner_finger_pad'):
            sel.setdefault(b, []).append(e)
    print('selected', {k: len(v) for k, v in sel.items()})
    for q in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8):
        poses = m.link_transforms(q)
        out = {}
        for b, es in sel.items():
            v = np.vstack([(poses[e['body']] @ np.column_stack([e['mesh'].vertices, np.ones(len(e['mesh'].vertices))]).T).T[:, :3] for e in es])
            out[b] = (v.mean(0), v.min(0), v.max(0))
        print('q=%.2f' % q, {b: (np.round(c*1000,1).tolist(), np.round(lo*1000,1).tolist(), np.round(hi*1000,1).tolist()) for b, (c, lo, hi) in out.items()})
except Exception:
    print(traceback.format_exc())
app.close()
