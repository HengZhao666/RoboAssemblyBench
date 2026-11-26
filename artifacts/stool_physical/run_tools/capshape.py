from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys, builtins, traceback
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
out = open('/tmp/capshape_result.txt', 'w')
def say(*a): builtins.print(*a, file=out, flush=True)
try:
    for path in sys.argv[1:]:
        st = Usd.Stage.Open(path)
        cache = UsdGeom.XformCache()
        for prim in st.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                pts = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
                M = np.asarray(cache.GetLocalToWorldTransform(prim)).T
                w = (M @ np.c_[pts, np.ones(len(pts))].T).T[:, :3]
                approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None
                lo, hi = w.min(0), w.max(0)
                say(path.rsplit('/', 1)[-1], prim.GetPath(), 'n', len(w), 'approx', approx, 'bbox mm', np.round(lo*1000, 2), np.round(hi*1000, 2))
                for frac in (0.0005, 0.002, 0.005):
                    b = w[w[:, 2] <= lo[2] + frac]
                    say('  bottom within %.1f mm: n=%d x[%.1f,%.1f] y[%.1f,%.1f]' % (frac*1000, len(b), b[:,0].min()*1000, b[:,0].max()*1000, b[:,1].min()*1000, b[:,1].max()*1000))
                    t = w[w[:, 2] >= hi[2] - frac]
                    say('  top within %.1f mm: n=%d x[%.1f,%.1f] y[%.1f,%.1f]' % (frac*1000, len(t), t[:,0].min()*1000, t[:,0].max()*1000, t[:,1].min()*1000, t[:,1].max()*1000))
                say('  vertex centroid mm', np.round(w.mean(0)*1000, 2))
        for prim in st.Traverse():
            if prim.HasAPI(UsdPhysics.MassAPI):
                m = UsdPhysics.MassAPI(prim)
                say('  mass', m.GetMassAttr().Get(), 'com', m.GetCenterOfMassAttr().Get(), 'density', m.GetDensityAttr().Get())
except Exception:
    say(traceback.format_exc())
app.close()
