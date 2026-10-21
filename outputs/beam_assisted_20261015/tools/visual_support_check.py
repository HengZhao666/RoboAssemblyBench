"""Read-only visible support qualification using already updated CAD objects."""
import math
import fcl

def missing_dynamic_poses(model, observed):
    required={m['body'] for m in model.data['visible_meshes']
              if m.get('robot') or str(m.get('object','')).startswith('fabrica_beam_')}
    return sorted(required-set(observed))

def check_visual_support(checker, contract, phase, step):
    robot, part = contract['robot'], contract['object']
    parts = [i for i,m in enumerate(checker.meshes) if m.get('object') == part]
    rows=[]
    for finger in ('left_inner_finger','right_inner_finger'):
        fingers=[i for i,m in enumerate(checker.meshes)
                 if m.get('robot') == robot and m['body'].endswith('/'+finger)]
        values=[]
        for i in parts:
            for j in fingers:
                result=fcl.CollisionResult()
                hit=fcl.collide(checker.shapes[i],checker.shapes[j],
                    fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
                gap=-max((float(c.penetration_depth) for c in result.contacts),default=0.) if hit else float(
                    fcl.distance(checker.shapes[i],checker.shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
                values.append(gap)
        gap=min(values) if values else None
        ready=bool(gap is not None and math.isfinite(gap)
            and contract['minimum_surface_gap_m'] <= gap <= contract['maximum_surface_gap_m'])
        rows.append({'finger':finger,'signed_gap_m':gap,'within_visual_support_gap':ready,
                     'mesh_pairs':len(values)})
    return {'step':step,'phase':phase,'robot':robot,'object':part,'ready':all(r['within_visual_support_gap'] for r in rows),
            'fingers':rows,'support_type':'nonattached_visual_pinch','force_support_claimed':False}

