"""Local, bounded Cartesian continuation for the assisted Beam route."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation

def bounded_local_ik(forward, reference, target_position, target_orientation,
                     position_tolerance=1e-5, orientation_tolerance=1e-4,
                     radius=.12, max_iterations=30):
    reference=np.asarray(reference,float);q=reference.copy()
    target_position=np.asarray(target_position,float)
    w,x,y,z=np.asarray(target_orientation,float)
    target_rotation=Rotation.from_quat([x,y,z,w]).as_matrix()
    if reference.shape!=(6,) or not np.all(np.isfinite(reference)):
        raise ValueError('assisted local IK requires six finite native joints')
    def error(position,rotation):
        return np.r_[target_position-position,
            .1*Rotation.from_matrix(target_rotation@rotation.T).as_rotvec()]
    detail={}
    for iteration in range(max_iterations):
        position,rotation=forward(q);position=np.asarray(position);rotation=np.asarray(rotation)
        e=error(position,rotation)
        detail={'iterations':iteration,'position_error':float(np.linalg.norm(e[:3])),
                'orientation_error':float(np.linalg.norm(e[3:])/.1),
                'max_joint_delta':float(np.max(abs(q-reference)))}
        if detail['position_error']<=position_tolerance and detail['orientation_error']<=orientation_tolerance:
            return q,dict(detail,converged=True)
        J=np.empty((6,6));epsilon=1e-5
        for joint in range(6):
            trial=q.copy();trial[joint]+=epsilon;p,R=forward(trial)
            J[:3,joint]=(np.asarray(p)-position)/epsilon
            J[3:,joint]=.1*Rotation.from_matrix(np.asarray(R)@rotation.T).as_rotvec()/epsilon
        U,s,Vh=np.linalg.svd(J,full_matrices=False)
        delta=Vh.T@((s/(s*s+1e-8))*(U.T@e))
        delta*=min(1.,.02/max(float(np.max(abs(delta))),1e-15))
        accepted=False
        for scale in (1.,.5,.25,.125):
            candidate=np.clip(q+scale*delta,reference-radius,reference+radius)
            p,R=forward(candidate)
            if np.linalg.norm(error(np.asarray(p),np.asarray(R)))<np.linalg.norm(e)-1e-14:
                q=candidate;accepted=True;break
        if not accepted:return None,dict(detail,converged=False,reason='no_local_descent')
    return None,dict(detail,converged=False,reason='iteration_limit')
