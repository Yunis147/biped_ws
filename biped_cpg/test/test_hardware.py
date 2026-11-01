"""Hardware interlocks tested with generated feedback; no motor bus is opened."""
import math
import unittest
import numpy as np

from test_cpg import fresh_library
from biped_cpg.controller import Walker
from biped_cpg.hardware import ServoFeedback, HardwareWalker
from biped_cpg.planner import NAMES


class FeedbackTests(unittest.TestCase):
    def test_name_reordering_and_velocity_estimation(self):
        feedback = ServoFeedback()
        q = np.arange(12)*.001
        feedback.joints(NAMES[::-1], q[::-1], [], 1.)
        np.testing.assert_array_equal(feedback.q, q)
        self.assertIsNone(feedback.velocity)
        feedback.joints(NAMES, q+.001, [], 1.02)
        np.testing.assert_allclose(feedback.velocity, .05)

    def test_bad_positions_timestamps_and_missing_names(self):
        for names, q, age in ((NAMES[:-1], np.zeros(11), 0.),
                              (NAMES, np.full(12, math.nan), 0.),
                              (NAMES, np.zeros(12), 1.)):
            with self.subTest(age=age):
                f = ServoFeedback()
                with self.assertRaises(ValueError):
                    f.joints(names, q, [], 1., age)
                self.assertTrue(f.problem(1.))

    def test_bridge_readiness_bytes_startup_torque_and_health(self):
        f = ServoFeedback()
        f.joints(NAMES, np.zeros(12), np.zeros(12), 1.)
        for message, level, values in (('torque OFF', b'\x00', {'startup_positioning': 'False'}),
                                       ('torque ON', 0, {'startup_positioning': 'True'}),
                                       ('torque ON', 1, {'startup_positioning': 'False'}),
                                       ('torque ON', 0, {})):
            f.diagnostics(message, level, values, 1.)
            self.assertTrue(f.problem(1.))
        f.diagnostics('torque ON', b'\x00', {'startup_positioning': 'False'}, 1.)
        self.assertFalse(f.problem(1.))
        self.assertTrue(f.problem(1.3))


class HardwareWalkerTests(unittest.TestCase):
    def setUp(self):
        self.runner = HardwareWalker(Walker(fresh_library()))
        self.now = 1.
        self.measured = np.full(12, .01)
        self.feed()

    def feed(self):
        self.runner.feedback.joints(NAMES, self.measured, np.zeros(12), self.now)
        self.runner.feedback.diagnostics('torque ON', b'\x00', {'startup_positioning': 'False'}, self.now)

    def tick(self):
        self.now += .02
        if self.runner.target is not None:
            self.measured += np.clip(self.runner.target-self.measured, -.007, .007)
        self.feed()
        return self.runner.tick(self.now)

    def advance(self, predicate):
        for _ in range(8000):
            self.tick()
            self.assertFalse(self.runner.fault, self.runner.fault)
            if predicate():
                return
        self.fail('hardware controller did not reach target state')

    def start_walking(self):
        self.runner.command('start', self.now)
        self.advance(lambda: self.runner.walker.state == 'walking' and self.runner.walker.cpg.right > .1)

    def test_no_publish_until_command_and_measured_first_target(self):
        self.assertIsNone(self.runner.tick(self.now))
        self.runner.command('step', self.now)
        output = self.tick()
        np.testing.assert_array_equal(output, self.measured)
        self.advance(lambda: self.runner.walker.active and self.runner.walker.state == 'idle')
        self.assertEqual(self.runner.walker.landings, 1)
        self.assertFalse(self.runner.fault)

    def test_no_arbitrary_pose_or_moving_start(self):
        self.measured[:] = .06
        self.feed()
        with self.assertRaisesRegex(ValueError, 'near calibrated zero'):
            self.runner.command('start', self.now)
        self.measured[:] = .01
        self.feed()
        self.runner.feedback.velocity[:] = .2
        with self.assertRaisesRegex(ValueError, 'stationary'):
            self.runner.command('start', self.now)

    def test_pause_then_resume_on_tracking_lag(self):
        self.start_walking()
        phase = self.runner.walker.cpg.right
        target = self.runner.target.copy()
        self.measured = target.copy()
        self.measured[0] += .06
        self.now += .02
        self.feed()
        np.testing.assert_array_equal(self.runner.tick(self.now), target)
        self.assertTrue(self.runner.paused)
        self.assertEqual(self.runner.walker.cpg.right, phase)
        self.measured = target.copy()
        self.now += .02
        self.feed()
        self.runner.tick(self.now)
        self.assertGreater(self.runner.walker.cpg.right, phase)

    def test_persistent_lag_latches_hold(self):
        self.start_walking()
        target = self.runner.target.copy()
        for _ in range(110):
            self.measured = target.copy()
            self.measured[0] += .06
            self.now += .02
            self.feed()
            output = self.runner.tick(self.now)
            np.testing.assert_array_equal(output, target)
        self.assertIn('lag did not clear', self.runner.fault)
        with self.assertRaises(ValueError):
            self.runner.command('start', self.now)

    def test_large_error_and_stale_feedback_stop_progress(self):
        self.start_walking()
        target = self.runner.target.copy()
        self.measured = target.copy()
        self.measured[0] += .16
        self.now += .02
        self.feed()
        np.testing.assert_array_equal(self.runner.tick(self.now), target)
        self.assertIn('excessive', self.runner.fault)
        self.now += .3
        self.assertIsNone(self.runner.tick(self.now))

    def test_torque_off_suppresses_output(self):
        self.start_walking()
        self.now += .02
        self.runner.feedback.diagnostics('torque OFF', 0, {'startup_positioning': 'False'}, self.now)
        self.assertIsNone(self.runner.tick(self.now))
        self.assertIn('torque', self.runner.fault)

    def test_other_writer_suppresses_output(self):
        self.runner.command('step', self.now)
        self.tick()
        self.runner.fail('another writer', suppress=True)
        self.assertIsNone(self.tick())

    def test_stop_during_approach_holds_without_zero_snap(self):
        self.runner.command('start', self.now)
        self.tick()
        target = self.runner.target.copy()
        self.runner.command('stop', self.now)
        self.now += .02
        self.feed()
        np.testing.assert_array_equal(self.runner.tick(self.now), target)
