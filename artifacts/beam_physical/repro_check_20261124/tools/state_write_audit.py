"""Once-only terminal dump of task setter call counts; never writes physics."""
import copy
import json
from pathlib import Path


class StateWriteAudit:
    def __init__(self, directory):
        self.path=Path(directory)/'state_write_counts.json'
        self.dumped=False
        self.task=None

    def begin_execution(self, task):
        # The first task update follows scene construction/reset. Counts before
        # this marker remain in the raw total, separately from execution counts.
        self.task=task
        if not getattr(task,'_beam_state_write_execution_started',False):
            task._beam_state_write_execution_start_step=int(task.step_counter)
            task._beam_state_write_execution_started=True

    def install(self, task_class):
        original=task_class._set_terminal_state
        audit=self
        def terminal(task,*args,**kwargs):
            result=original(task,*args,**kwargs)
            if task.success or task.failed:
                audit.dump(task)
            return result
        task_class._set_terminal_state=terminal
        return lambda:setattr(task_class,'_set_terminal_state',original)

    def dump(self, task, *, terminal_observed=True):
        if self.dumped:return
        rows=[];forbidden={};unknown=0
        counts=getattr(task,'_beam_state_write_counts',{})
        for (phase,event,obj),value in sorted(counts.items()):
            row={'phase':phase,'event':event,'object':obj,**copy.deepcopy(value)}
            execution=row.get('execution',{})
            number=int(execution.get('count',0));bad=0
            if obj.startswith('fabrica_beam_'):
                if event in {'zero_object_velocity','set_object_pose',
                             'create_fixed_attachment_joint','create_compliant_attachment_joint'}:
                    bad=number
                elif event=='set_kinematic_enabled':
                    bad=int(execution.get('enabled_true',0))
                    unknown+=int(execution.get('enabled_unspecified',0))
                elif event in {'set_rigid_body_simulated','set_object_collision','set_gravity_enabled'}:
                    bad=int(execution.get('enabled_false',0))
                    unknown+=int(execution.get('enabled_unspecified',0))
            row['forbidden_calls_after_scene_initialization']=bad
            if bad:forbidden[event]=forbidden.get(event,0)+bad
            rows.append(row)
        errors=int(getattr(task,'_beam_state_write_count_errors',0))
        started=bool(getattr(task,'_beam_state_write_execution_started',False))
        report={'scope':'All task _trace_beam_state_write call sites, regardless of sparse diagnostics selection.',
            'call_sites_only_not_setter_completion_or_effect':True,
            'initialization_boundary':'First task _update_task_state entry after scene construction/reset.',
            'execution_started':started,
            'execution_start_task_step':getattr(task,'_beam_state_write_execution_start_step',None),
            'dump_task_step':int(task.step_counter),'terminal_observed':terminal_observed,
            'success':bool(task.success),'failed':bool(task.failed),
            'counter_errors':errors,'unclassified_enable_calls':unknown,
            'complete':started and terminal_observed and errors==0 and unknown==0,
            'forbidden_calls_after_scene_initialization':sum(forbidden.values()),
            'forbidden_by_event':forbidden,'rows':rows}
        temp=self.path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(report,indent=2)+'\n');temp.replace(self.path)
        self.dumped=True

    def close(self):
        # An unexpected worker exit remains explicitly incomplete. Never turn
        # a missing terminal callback into a successful audit.
        if not self.dumped and self.task is not None:
            self.dump(self.task,terminal_observed=False)
