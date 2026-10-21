"""A failure remains latched while real physical time and all cameras advance."""
import ast
import copy
import importlib.util
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('failure_tail_under_test',
    ROOT/'toolkits/factory_dual_franka_assembly/beam_failure_video.py')
tail = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tail)


class Fixture:
    def __init__(self, path, dt=.05, stride=4):
        self.dt, self.index, self.physics_steps = dt, 1000, 0
        self.task = NS(name='task', step_counter=100, failed=True, success=False,
            phase='failed', _resolved_objects={'fabrica_beam_6': object()},
            robots={'left': NS(get_obs=lambda: {'camera': 'left'}),
                    'right': NS(get_obs=lambda: {'camera': 'right'})})
        self.task._beam_physics_stamp = lambda: dict(valid=True, epoch=0,
            index=self.index, time=self.index*self.dt, dt=self.dt)
        self.runner = NS(current_tasks={'task':self.task}, render_trigger=7,
                         _world=NS(step=self.step))
        self.env = NS(runner=self.runner, get_dt=lambda: self.dt)
        self.recorder = NS(output_dir=Path(path), frame_stride=stride,
            _written_png_counts={'front':20,'left':20,'right':20},
            camera_specs=[{'video_key':s} for s in ['front','left','right']],
            record=self.record)
        self.metrics = {'failed':True,'success':False,'steps':100,
            'terminal_reason':'original-reason','detail':{'first_failure':1000}}
        self.observations, self.native_samples = [], []

    def step(self, *, render):
        self.index += 1
        self.physics_steps += 1
        self.rendered = render

    def record(self, task, obs, *, step_index, post_failure, failure_task_step):
        assert self.rendered and post_failure and task is self.task
        assert set(obs)=={'left','right'} and step_index % self.recorder.frame_stride==0
        self.last_failure_step = failure_task_step
        self.observations.append((step_index,self.index))
        for key in self.recorder._written_png_counts:
            self.recorder._written_png_counts[key] += 1

    def native(self, task, boundary, context, cycle_step):
        self.native_samples.append((self.index,cycle_step))
        return {'clock_valid':True,'valid':True}

    def run(self, seconds=None):
        settings = {} if seconds is None else {'BEAM_POST_FAILURE_VIDEO_SECONDS':str(seconds)}
        with patch.dict(os.environ,settings,clear=True), patch.object(tail,'_native_recorder',
                return_value=NS(sample=self.native)):
            return tail.record_failure_tail(self.env,self.task,self.recorder,self.metrics)


class FailureTail(unittest.TestCase):
    def test_default_five_seconds_uses_physics_and_preserves_first_failure(self):
        with TemporaryDirectory() as path:
            f=Fixture(path); original=copy.deepcopy(f.metrics); result=f.run()
            self.assertTrue(result['capture_complete'])
            self.assertEqual(f.physics_steps,100)
            self.assertAlmostEqual(result['completed_seconds'],5.)
            self.assertEqual(result['frames_added'],dict(front=25,left=25,right=25))
            self.assertEqual(len(f.native_samples),26)
            self.assertTrue(result['native_complete'])
            self.assertEqual(f.metrics,original)
            self.assertEqual(f.task.step_counter,100)
            self.assertTrue(f.task.failed)
            self.assertFalse(f.task.success)
            self.assertEqual(json.loads((Path(path)/'post_failure_recording.json').read_text()),result)

    def test_first_failure_label_precedes_done_tick_without_relabeling_images(self):
        with TemporaryDirectory() as path:
            f=Fixture(path)
            f.metrics['phase_transition_history']=[{'transition_type':'failure','step_counter':99}]
            result=f.run(.4)
            self.assertEqual(result['failure_task_step'],99)
            self.assertEqual(result['capture_entry_task_step'],100)
            self.assertEqual(f.last_failure_step,99)
            self.assertEqual(f.observations[0][0],104)

    def test_only_two_configured_cameras_cannot_certify_three_view_tail(self):
        with TemporaryDirectory() as path:
            f=Fixture(path); f.recorder.camera_specs=f.recorder.camera_specs[:2]
            self.assertFalse(f.run(.4)['capture_complete'])

    def test_partial_interval_rounds_up_without_freezing_frames(self):
        with TemporaryDirectory() as path:
            f=Fixture(path,stride=1); result=f.run(.12)
            self.assertEqual(f.physics_steps,3)
            self.assertAlmostEqual(result['completed_seconds'],.15)
            self.assertEqual(len(set(index for _,index in f.observations)),3)

    def test_zero_is_an_explicit_disable_without_physical_step(self):
        with TemporaryDirectory() as path:
            f=Fixture(path); result=f.run(0)
            self.assertTrue(result['disabled'])
            self.assertFalse(result['capture_complete'])
            self.assertEqual(f.physics_steps,0)

    def test_invalid_duration_is_reported_without_touching_physics(self):
        for value in ['nan','inf','-1','nonsense']:
            with self.subTest(value=value),TemporaryDirectory() as path:
                f=Fixture(path); result=f.run(value)
                self.assertIn('error',result)
                self.assertFalse(result['completed'])
                self.assertEqual(f.physics_steps,0)

    def test_success_and_nonbeam_episodes_never_get_a_tail(self):
        with TemporaryDirectory() as path:
            for condition in ['success','nonbeam']:
                f=Fixture(path)
                if condition=='success':f.metrics.update(failed=False,success=True)
                else:f.task._resolved_objects={}
                self.assertIsNone(f.run())
                self.assertEqual(f.physics_steps,0)

    def test_scene_removed_cannot_be_reconstructed_as_same_state(self):
        with TemporaryDirectory() as path:
            f=Fixture(path); f.runner.current_tasks={}; result=f.run()
            self.assertIn('already removed',result['error'])
            self.assertEqual(f.physics_steps,0)

    def test_frozen_or_restarted_physics_clock_cannot_be_complete(self):
        with TemporaryDirectory() as path:
            f=Fixture(path)
            f.task._beam_physics_stamp=lambda:dict(valid=True,epoch=0,index=1000,time=50.)
            result=f.run(.2)
            self.assertFalse(result['completed'])
            self.assertIn('clock',result['error'])

    def test_missing_camera_is_explicitly_incomplete(self):
        with TemporaryDirectory() as path:
            f=Fixture(path)
            f.recorder.camera_specs.append({'video_key':'missing'})
            result=f.run(.4)
            self.assertTrue(result['completed'])
            self.assertFalse(result['all_cameras_recorded'])
            self.assertFalse(result['capture_complete'])

    def test_one_dropped_frame_does_not_certify_full_camera_tail(self):
        with TemporaryDirectory() as path:
            f=Fixture(path)
            original=f.record
            def dropping_record(*args,**kwargs):
                original(*args,**kwargs)
                if len(f.observations)==1:
                    f.recorder._written_png_counts['left']-=1
            f.recorder.record=dropping_record
            result=f.run(.4)
            self.assertTrue(result['completed'])
            self.assertFalse(result['all_cameras_recorded'])

    def test_native_read_failure_keeps_video_and_marks_missing_evidence(self):
        with TemporaryDirectory() as path:
            f=Fixture(path)
            with patch.dict(os.environ,{'BEAM_POST_FAILURE_VIDEO_SECONDS':'.4'},clear=True),\
                 patch.object(tail,'_native_recorder',side_effect=RuntimeError('missing cached views')):
                result=tail.record_failure_tail(f.env,f.task,f.recorder,f.metrics)
            self.assertTrue(result['capture_complete'])
            self.assertEqual(result['native_samples'],0)
            self.assertIn('missing cached views',result['native_errors'][0])

    def test_physical_exception_preserves_failure_and_partial_record(self):
        with TemporaryDirectory() as path:
            f=Fixture(path); original=copy.deepcopy(f.metrics)
            def broken_step(*,render):raise RuntimeError('physics unavailable')
            f.runner._world.step=broken_step
            result=f.run()
            self.assertFalse(result['completed'])
            self.assertIn('physics unavailable',result['error'])
            self.assertEqual(f.metrics,original)

    def test_native_tail_writer_selects_all_parts_without_creating_views(self):
        module='toolkits.factory_dual_franka_assembly.beam_physics_diagnostics'
        task=NS(_resolved_objects={'fabrica_beam_6':object(),'fabrica_beam_2':object(),
                                  'optical_board':object()})
        with patch.dict('sys.modules',{module:NS(BeamPhysicsDiagnostics=lambda:NS())}):
            recorder=tail._native_recorder(Path('/tmp'),task)
        self.assertEqual(recorder.objects,('fabrica_beam_6','fabrica_beam_2'))
        self.assertEqual(recorder.phases,{'*'})
        self.assertEqual(recorder.stride,1)


class ActualVideoRecorderMethod(unittest.TestCase):
    def record_method(self):
        tree=ast.parse((ROOT/'toolkits/factory_dual_franka_assembly/generate_demos.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LiveRolloutVideoRecorder')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='record')
        unit=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]))
        fake_cv=NS(COLOR_RGB2BGR=1,cvtColor=lambda image,mode:image,
                   imwrite=lambda path,image:Path(path).write_bytes(b'image-test-double'))
        env={'np':np,'os':os,'cv2':fake_cv,'json':json,'_sanitize_episode_value':lambda value:value}
        exec(compile(unit,'actual_prepared_recorder','exec'),env)
        return env['record']

    def test_tail_records_all_three_streams_and_physical_stamp_without_qualification(self):
        with TemporaryDirectory() as directory,patch.dict(os.environ,{'BEAM_VIDEO_CLOCK_TRACE':'1'}):
            root=Path(directory)
            recorder=NS(output_dir=root,frames_root_dir=root/'frames',frame_stride=8,
                camera_specs=[{'video_key':key} for key in ['front','left','right']],
                _frame_dirs={},_image_stats={},_written_png_counts={},_frame_shapes={},_sampled_steps=[],
                _running_image_stats_cls=lambda:NS(update=lambda frame:None),
                _sensor_frame_from_obs=lambda obs,**kwargs:np.zeros((4,4,3),dtype=np.uint8),
                _camera_bindings={key:dict(robot_name='robot',sensor_name=key) for key in ['front','left','right']})
            def forbidden():raise AssertionError('Task qualification getter called')
            task=NS(step_counter=100,get_tracked_object_states=forbidden,
                    _beam_physics_stamp=lambda:dict(valid=True,epoch=0,index=1008,time=4.2))
            obs={'robot':{'sensors':{key:{'render_reference_time':{
                'referenceTimeNumerator':126,'referenceTimeDenominator':30}}
                for key in ['front','left','right']}}}
            self.record_method()(recorder,task,obs,step_index=104,post_failure=True)
            self.assertEqual(recorder._written_png_counts,dict(front=1,left=1,right=1))
            self.assertIsNone(recorder._sampled_steps[-1]['objects'])
            clocks=[json.loads(line) for line in (root/'video_frame_clock.jsonl').read_text().splitlines()]
            self.assertEqual(len(clocks),3)
            for clock in clocks:
                self.assertEqual(clock['capture_segment'],'post_failure')
                self.assertEqual(clock['failure_task_step'],100)
                self.assertEqual(clock['task_step_label'],104)
                self.assertEqual(clock['physics_stamp']['index'],1008)


if __name__=='__main__':
    unittest.main()
