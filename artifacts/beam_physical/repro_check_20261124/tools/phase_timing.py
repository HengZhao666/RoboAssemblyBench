"""Low-overhead aggregate timing; no extra state probes or action changes."""
from pathlib import Path
import functools,json,time
class PhaseTiming:
    def __init__(self,out):
        self.path=Path(out)/'phase_timing.json';self.rows={};self.task=None
        self.started=time.monotonic();self.last_flush=self.started
    def add(self,kind,phase,elapsed):
        row=self.rows.setdefault(str(phase),{})
        item=row.setdefault(kind,dict(calls=0,seconds=0.0));item['calls']+=1;item['seconds']+=elapsed
        if time.monotonic()-self.last_flush>=60:self.flush()
    def flush(self,final=False):
        data=dict(final=final,elapsed_wall_seconds=time.monotonic()-self.started,phases=self.rows,
            scope='policy_act and env_step separate inclusive intervals. env_step contains physics, task update, observation/render work; these subcomponents are not individually attributed. Raw logging may occur inside either interval. No new physical getters or controller changes.')
        tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(self.path);self.last_flush=time.monotonic()
    def install(self,env,policy_class):
        original_act=policy_class.act;original_step=env.step
        @functools.wraps(original_act)
        def act(policy,task,*args,**kwargs):
            self.task=task;phase=task.phase;start=time.perf_counter()
            try:return original_act(policy,task,*args,**kwargs)
            finally:self.add('policy_act',phase,time.perf_counter()-start)
        @functools.wraps(original_step)
        def step(*args,**kwargs):
            phase=getattr(self.task,'phase','<before_first_policy>');start=time.perf_counter()
            try:return original_step(*args,**kwargs)
            finally:self.add('env_step',phase,time.perf_counter()-start)
        policy_class.act=act;env.step=step
