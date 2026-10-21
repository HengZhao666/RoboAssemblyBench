"""Image time must come from its render annotation, independently of task labels."""
import copy
import unittest
from types import SimpleNamespace as NS

from toolkits.factory_dual_franka_assembly.beam_video_clock import (
    render_reference_time, video_frame_clock,
)


class VideoClock(unittest.TestCase):
    def test_rational_render_time_keeps_exact_source_values(self):
        value = render_reference_time({'referenceTimeNumerator': 1073,
                                       'referenceTimeDenominator': 10})
        self.assertEqual(value['numerator'], 1073)
        self.assertEqual(value['denominator'], 10)
        self.assertAlmostEqual(value['seconds'], 107.3)
        self.assertTrue(value['valid'])

    def test_missing_invalid_or_failed_annotation_stays_unknown(self):
        for value in (None, {}, {'error': 'not initialized'},
                      {'referenceTimeNumerator': 1, 'referenceTimeDenominator': 0},
                      {'referenceTimeNumerator': -1, 'referenceTimeDenominator': 1},
                      {'referenceTimeNumerator': True, 'referenceTimeDenominator': 1},
                      {'referenceTimeNumerator': 1, 'referenceTimeDenominator': 1.0}):
            with self.subTest(value=value):
                self.assertFalse(render_reference_time(value)['valid'])

    def test_stale_render_and_current_physics_times_are_not_equated(self):
        task = NS(_beam_physics_stamp=lambda: {'valid': True, 'epoch': 0,
                   'index': 25851, 'time': 107.7125, 'dt': 1 / 240})
        observation = {'render_reference_time': {'referenceTimeNumerator': 1077,
                                                 'referenceTimeDenominator': 10}}
        original = copy.deepcopy(observation)
        value = video_frame_clock(task, step=25848, camera='front',
                                  frame_index=3230, sensor_observation=observation)
        self.assertAlmostEqual(value['physics_time_minus_render_time_s'], .0125)
        self.assertIn('unverified', value['clock_relation'])
        self.assertEqual(value['physics_stamp']['index'], 25851)
        self.assertEqual(observation, original)

    def test_unavailable_render_time_never_uses_task_step_or_physics_time(self):
        task = NS(_beam_physics_stamp=lambda: {'valid': True, 'index': 24,
                                              'time': .1, 'dt': 1 / 240})
        value = video_frame_clock(task, step=10000, camera='left', frame_index=0,
                                  sensor_observation={})
        self.assertFalse(value['render_reference_time']['valid'])
        self.assertNotIn('physics_time_minus_render_time_s', value)

    def test_only_existing_physics_clock_reader_is_called(self):
        class Task:
            def _beam_physics_stamp(self):
                return {'valid': False, 'reason': 'clock unavailable'}

            def __getattr__(self, name):
                raise AssertionError('Unexpected task/physics/qualification call: ' + name)
        value = video_frame_clock(Task(), step=8, camera='right', frame_index=0,
            sensor_observation={'render_reference_time': {'referenceTimeNumerator': 1,
                                                          'referenceTimeDenominator': 30}})
        self.assertFalse(value['physics_stamp']['valid'])
        self.assertTrue(value['render_reference_time']['valid'])


if __name__ == '__main__':
    unittest.main()
