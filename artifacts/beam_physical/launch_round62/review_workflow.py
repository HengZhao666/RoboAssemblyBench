"""Engine-free review of actual Beam phase contracts and completed batch timing.

This checks dispatch/default merging and nominal ownership, not dynamics or
collision clearance. Original configs/results remain untouched.
"""
from pathlib import Path
import argparse,copy,datetime,hashlib,json,math,sys,time

def review(config):
    from types import SimpleNamespace
    from toolkits.factory_dual_franka_assembly.local_skills import LocalSkillExecutor,normalize_local_skill_spec
    ex=LocalSkillExecutor();task=SimpleNamespace(cfg=SimpleNamespace(task_metadata=config.get('task_metadata',{})))
    robots=[x['name']for x in config['robots']];held=set();issues=[];exceptions=[];counts=dict(phases=0,skills=0,completion_bindings=0,ownership_checks=0,moving_force_bands=0,minimum_window_checks=0)
    def issue(kind,phase,**detail):issues.append(dict(kind=kind,phase=phase,**detail))
    def conditions(a):
        if not isinstance(a,dict):return
        yield a
        for x in a.get('conditions',[]):yield from conditions(x)
    names=[p['name']for p in config['phase_specs']]
    if len(names)!=len(set(names)):issue('duplicate_phase_name','<graph>')
    for phase in config['phase_specs']:
        counts['phases']+=1;name=phase['name'];skills={}
        for robot in robots:
            s=normalize_local_skill_spec(phase,robot)
            if s is None:continue
            s=ex._merge_task_defaults(task,s);skills[robot]=s;counts['skills']+=1
            if not(s.get('adapter')or s.get('adapter_class')):issue('unbound_adapter',name,robot=robot,skill=s['name'])
            obj=s.get('object')or s.get('held_object')
            if s.get('requires_held_object') and isinstance(obj,str):
                counts['ownership_checks']+=1
                if (obj,robot)not in held:issue('nominal_required_grasp_missing',name,robot=robot,object=obj)
            if s.get('requires_empty_arm'):
                counts['ownership_checks']+=1
                if any(r==robot for o,r in held):issue('nominal_empty_arm_still_loaded',name,robot=robot)
            if s['name']=='ur5e_close_gripper' and s.get('close_force_regulation'):
                low=float(s['close_force_low']);high=float(s['close_force_high']);target=s.get('close_force_balance_min')
                if name=='base_6_close_and_attach':
                    exceptions.append(dict(phase=name,scope='Base has native pass evidence; moving-grasp force-band rule does not generalize to base.',balance_target=target,clamp_low=low))
                else:
                    counts['moving_force_bands']+=1
                    if target is None or not low<=float(target)<=high:issue('moving_balance_target_outside_clamp_band',name,balance_target=target,clamp_band=[low,high])
        timeout=int(phase.get('timeout_steps',0))
        for cond in conditions(phase.get('advance',{})):
            if cond.get('type')=='local_skill_complete':
                counts['completion_bindings']+=1;s=skills.get(cond.get('robot'))
                if not s or s['name']!=cond.get('skill'):issue('completion_has_no_matching_skill',name,condition=cond,actual=None if not s else s['name'])
            for key in ['min_steps','stable_steps','physics_stable_steps']:
                if key in cond:
                    counts['minimum_window_checks']+=1
                    if timeout<int(cond[key]):issue('phase_timeout_below_required_window',name,timeout=timeout,window_key=key,window=cond[key])
        # Nominal registration after the stage; physical success is NOT inferred.
        for entry in phase.get('attach',[]):
            if entry.get('object') and entry.get('robot'):held.add((entry['object'],entry['robot']))
        for entry in phase.get('detach',[]):
            if isinstance(entry,str):held={pair for pair in held if pair[0]!=entry}
            else:held={pair for pair in held if not(pair[0]==entry.get('object')and(entry.get('robot')is None or pair[1]==entry['robot']))}
    return dict(issues=issues,counts=counts,reviewed_exceptions=exceptions,nominal_end_ownership=sorted(held),scope='Static necessary conditions only. Requires runtime ownership/contact, actual final publication and full geometry/native validation.')


def batch_timing(ledger):
    rows=[]
    for r in ledger['rounds']:
        if r['round']not in range(29,34):continue
        a=datetime.datetime.fromisoformat(r['started_at']);b=datetime.datetime.fromisoformat(r['finished_at'])
        rows.append(dict(round=r['round'],candidate=r['candidate'],start=r['started_at'],end=r['finished_at'],launch_to_exit_seconds=(b-a).total_seconds(),reason=r.get('reason')))
    gaps=[]
    for a,b in zip(rows,rows[1:]):gaps.append(dict(after=a['round'],before=b['round'],seconds=(datetime.datetime.fromisoformat(b['start'])-datetime.datetime.fromisoformat(a['end'])).total_seconds(),attribution='Uninstrumented mixture of analysis/preparation/cleanup/review/transfers and possible waiting; not native physics timing.'))
    return dict(rounds=rows,total_launch_to_exit_seconds=sum(x['launch_to_exit_seconds']for x in rows),gaps=gaps,note='Process intervals include initialization, native loop, failure tail and shutdown/encoding. Not pure engine compute.')


def phase_timing(path):
    rows=[json.loads(x)for x in path.read_text().splitlines()];entries=[]
    for row in rows:
        if not entries or row['phase']!=entries[-1]['phase']:entries.append(row)
    intervals=[]
    for a,b in zip(entries,entries[1:]):intervals.append(dict(phase=a['phase'],first_observed_step=a['task_step'],next_observed_step=b['task_step'],wall_seconds=b['elapsed_wall_seconds']-a['elapsed_wall_seconds'],native_intervals=b['task_step']-a['task_step']))
    return dict(top_observed_intervals=sorted(intervals,key=lambda x:x['wall_seconds'],reverse=True)[:10],policy_observation_to_failure_seconds=rows[-1]['elapsed_wall_seconds']-rows[0]['elapsed_wall_seconds'],scope='Existing post-step stage-transition observation intervals, approximately one control cycle at boundaries; not component profiler. Physics/control/render/log cost attribution remains unknown.')


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();B=a.root/'grasp_repairs_20261012_20rounds';P=B/'offline_v209_joint_dispatch';start=time.monotonic()
    candidates={}
    for n,folder in [('r29','round29_v205_recenter_reference'),('r30','round30_v206_feedback'),('r31','round31_v207_balance_band'),('r33','round33_v208_resource_retry'),('v209','offline_v209_joint_dispatch')]:
        path=B/folder/'production_worker_config.json';cfg=json.loads(path.read_text());candidates[n]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),**review(cfg))
    cfg=json.loads((P/'production_worker_config.json').read_text());negative=[]
    for label in ['missing_adapter','wrong_completion','held_arm_home','window_exceeds_timeout','moving_force_conflict']:
        c=copy.deepcopy(cfg)
        if label=='missing_adapter':
            s=c['phase_specs'][47]['local_skill'];s.pop('adapter');s.pop('adapter_class',None)
        elif label=='wrong_completion':c['phase_specs'][47]['advance']['skill']='misspelled_joint_skill'
        elif label=='held_arm_home':c['phase_specs'][47]['local_skill']['robot']='franka_left';c['phase_specs'][47]['local_skills'].pop('franka_left')
        elif label=='window_exceeds_timeout':c['phase_specs'][47]['advance']['min_steps']=9999
        else:c['phase_specs'][22]['local_skill']['close_force_balance_min']=2.0
        result=review(c)
        expected={'missing_adapter':'unbound_adapter','wrong_completion':'completion_has_no_matching_skill','held_arm_home':'nominal_empty_arm_still_loaded','window_exceeds_timeout':'phase_timeout_below_required_window','moving_force_conflict':'moving_balance_target_outside_clamp_band'}[label]
        assert expected in {x['kind']for x in result['issues']},(label,result['issues'])
        negative.append(dict(case=label,caught=[x['kind']for x in result['issues']]))
    assert not candidates['v209']['issues'],candidates['v209']['issues']
    assert any(x['kind']=='moving_balance_target_outside_clamp_band'for x in candidates['r30']['issues'])
    assert any(x['kind']=='unbound_adapter'for x in candidates['r33']['issues'])
    report=dict(passed=True,native_started=False,production_code_changed=False,configs=candidates,negative_cases=negative,batch_timing=batch_timing(json.loads((B/'round_ledger.json').read_text())),r33_stage_timing=phase_timing(a.root/'candidate_repository/outputs/fabrica_beam_v208_resource_retry/progress_history.jsonl'),elapsed_seconds=time.monotonic()-start)
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(passed=True,elapsed_seconds=report['elapsed_seconds'],config_issues={k:len(v['issues'])for k,v in candidates.items()},negative_cases=len(negative),timing=report['batch_timing'],top_phases=report['r33_stage_timing']['top_observed_intervals'][:4])))
if __name__=='__main__':main()
