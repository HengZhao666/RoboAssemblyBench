"""A postfailure physical observation tail, separate from task evaluation."""
import json
import math
import os
from pathlib import Path


def _is_beam(task):
    return any(str(name).startswith('fabrica_beam_')
               for name in getattr(task, '_resolved_objects', {})) or any(
        p.get('beam_coordinated') for p in getattr(task, 'phase_specs', []))


def _native_recorder(output_dir, task):
    # This writer reads only already initialized native body/contact views.
    from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import BeamPhysicsDiagnostics
    writer = BeamPhysicsDiagnostics()
    writer.path = str(output_dir/'post_failure_physics_trace.jsonl')
    writer.objects = tuple(name for name in task._resolved_objects
                           if name.startswith('fabrica_beam_'))
    writer.phases = {'*'}
    writer.start, writer.end, writer.stride = 0, -1, 1
    return writer


def record_failure_tail(env, task, recorder, failure_metrics):
    """Keep the last published targets; issue no actions, resets or task ticks.

    Results remain latched at failure. Only physical time and the three live
    cameras advance. This is observation under retained targets, not a new
    policy rollout or a free-fall experiment. A simulator crash cannot supply
    a tail and must remain explicitly incomplete.
    """
    if not _is_beam(task) or failure_metrics.get('failed') is not True:
        return None
    seconds_text = os.environ.get('BEAM_POST_FAILURE_VIDEO_SECONDS', '5')
    output_dir = Path(recorder.output_dir)
    failure_step = next((v['step_counter'] for v in failure_metrics.get('phase_transition_history', [])
                         if v.get('transition_type')=='failure' and isinstance(v.get('step_counter'), int)),
                        int(task.step_counter))
    result = {'requested_seconds': seconds_text, 'completed': False,
              'failure_task_step': failure_step,
              'capture_entry_task_step': int(task.step_counter),
              'failure_reason': failure_metrics.get('terminal_reason'),
              'control_mode': 'retain_last_published_targets_no_new_actions',
              'evaluation_continues': False, 'physics_steps': 0,
              'native_samples': 0, 'native_valid_samples': 0, 'native_errors': [], 'frames_added': {}}
    original_counts = dict(recorder._written_png_counts)
    try:
        seconds = float(seconds_text)
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Postfailure seconds must be finite and nonnegative')
        if seconds == 0:
            result['disabled'] = True
            return result
        dt = float(env.get_dt())
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError('Postfailure recording requires a valid physical dt')
        count = math.ceil(seconds/dt)
        runner = env.runner
        if runner.current_tasks.get(task.name) is not task:
            raise RuntimeError('Failed scene already removed; cannot resume its physical state')
        failure_step = result['capture_entry_task_step']
        result['failure_physics_stamp'] = dict(task._beam_physics_stamp())
        result['physics_dt'] = dt
        result['planned_physics_steps'] = count
        result['expected_frames_per_camera'] = ((failure_step+count)//recorder.frame_stride
                                                - failure_step//recorder.frame_stride)
        try:
            native = _native_recorder(output_dir, task)
            first_sample = native.sample(task, 'after_physics', None, failure_step)
            result['failure_native_sample_valid'] = bool(first_sample and first_sample.get('valid'))
        except Exception as exc:
            native = None
            result['native_errors'].append(type(exc).__name__ + ': ' + str(exc))
        # Match image labels and actual render cadence. No added physics tick.
        runner.render_trigger = failure_step % recorder.frame_stride
        for offset in range(1, count+1):
            sample_step = failure_step+offset
            runner.render_trigger += 1
            render = runner.render_trigger >= recorder.frame_stride
            if render:
                runner.render_trigger = 0
            runner._render = render
            # Bypass runner.step: it would tick is_done/recompute failed metrics.
            runner._world.step(render=render)
            result['physics_steps'] += 1
            if sample_step % recorder.frame_stride:
                continue
            if native is not None:
                try:
                    sample = native.sample(task, 'after_physics', None, sample_step)
                    result['native_samples'] += int(sample is not None)
                    result['native_valid_samples'] += int(bool(sample and sample.get('valid')))
                except Exception as exc:
                    result['native_errors'].append(type(exc).__name__ + ': ' + str(exc))
                    native = None
            # Robot/sensor reads avoid task getters that requalify attachments.
            obs = {name: robot.get_obs() for name, robot in task.robots.items()}
            recorder.record(task, obs, step_index=sample_step, post_failure=True,
                            failure_task_step=result['failure_task_step'])
        result['end_physics_stamp'] = dict(task._beam_physics_stamp())
        first, last = result['failure_physics_stamp'], result['end_physics_stamp']
        if (not first.get('valid') or not last.get('valid')
                or first.get('epoch') != last.get('epoch')
                or last['index']-first['index'] != count
                or abs((last['time']-first['time'])-count*dt)>max(dt/10,1e-6)):
            raise RuntimeError('Postfailure physical clock did not advance continuously')
        result['completed_seconds'] = last['time']-first['time']
        result['completed'] = True
    except Exception as exc:
        result['error'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        result['frames_added'] = {
            key: value-original_counts.get(key, 0)
            for key, value in recorder._written_png_counts.items()}
        expected = {s['video_key'] for s in recorder.camera_specs}
        result['all_cameras_recorded'] = len(expected)==3 and all(
            result['frames_added'].get(key, 0)==result.get('expected_frames_per_camera', -1)
            and result['frames_added'].get(key, 0)>0 for key in expected)
        result['capture_complete'] = bool(result['completed'] and result['all_cameras_recorded'])
        result['native_complete'] = bool(result.get('failure_native_sample_valid')
            and not result['native_errors']
            and result['native_valid_samples']==result.get('expected_frames_per_camera', -1))
        (output_dir/'post_failure_recording.json').write_text(
            json.dumps(result, allow_nan=False, indent=2)+'\n')
    return result
