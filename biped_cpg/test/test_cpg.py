"""Offline math/state tests; no ROS nodes or hardware are started."""
from functools import lru_cache
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / package) for package in ('biped_cpg', 'biped_gait')]

from biped_cpg.controller import Library, Walker
from biped_cpg.curves import Curve
from biped_cpg.oscillator import CoupledCPG
from biped_cpg.planner import NAMES, Settings, model


@lru_cache(maxsize=1)
def library():
    return Library()


def fresh_library():
    result = Library.__new__(Library)
    result.settings = library().settings
    result.plans = dict(library().plans)
    return result


class OscillatorTests(unittest.TestCase):
    def test_two_legs_remain_opposed_and_period_is_correct(self):
        cpg = CoupledCPG(10.)
        for _ in range(500):
            cpg.step(.02)
        self.assertAlmostEqual(cpg.right, 2*math.pi, places=11)
        self.assertAlmostEqual(cpg.left-cpg.right, math.pi, places=11)

    def test_phase_coupling_recovers_from_a_phase_error(self):
        cpg = CoupledCPG(10., coupling=2.)
        cpg.left += .4
        before = abs(cpg.phase_error)
        for _ in range(100):
            cpg.step(.02)
        self.assertLess(abs(cpg.phase_error), before / 1000)

    def test_oscillator_is_independent_of_time_step_size(self):
        one, many = CoupledCPG(12.), CoupledCPG(12.)
        one.left += .3
        many.left += .3
        one.step(2.)
        for _ in range(200):
            many.step(.01)
        self.assertAlmostEqual(one.right, many.right, places=12)
        self.assertAlmostEqual(one.left, many.left, places=12)

    def test_invalid_time_and_settings_are_rejected(self):
        for period in (0., -1., float('nan')):
            with self.assertRaises(ValueError):
                CoupledCPG(period)
        with self.assertRaises(ValueError):
            CoupledCPG(10.).step(-.1)
        for kwargs in ({'body_roll': .2}, {'double_support': .333}, {'max_joint_speed': .5},
                       {'half_samples': 120.}, {'centre_crouch': .03}):
            with self.assertRaises(ValueError):
                Settings(**kwargs)


class PlannerTests(unittest.TestCase):
    def test_small_lean_foot_constraints_and_support_margin(self):
        for plan in library().plans.values():
            self.assertLess(plan.summary()['body_roll_deg'], 5.)
            self.assertGreaterEqual(plan.summary()['minimum_margin_mm'], 10.)
            for result in plan.metrics.values():
                self.assertLess(result['foot_error_mm'], .01)
                self.assertLess(result['floor_error_mm'], .01)
                self.assertLess(result['com_error_mm'], .01)
                self.assertLess(result['maximum_roll_deg'], 4.7)

    def test_continuous_seams_and_joint_motion_bounds(self):
        for plan in library().plans.values():
            np.testing.assert_array_equal(plan.entry.curve.rows[-1], plan.cycle.curve.sample(.5))
            np.testing.assert_array_equal(plan.cycle.curve.rows[0], plan.cycle.curve.rows[-1])
            for phase in (math.pi, 2*math.pi, 3*math.pi):
                self.assertLess(np.max(np.abs(plan.pose(phase+1e-7)-plan.pose(phase-1e-7))), 1e-6)
            for curve, duration in [(plan.entry.curve, plan.period/2), (plan.cycle.curve, plan.period),
                                    (plan.startup.curve, plan.startup_duration)]:
                # Check actual sampled commands, independently of the polynomial extrema calculation.
                samples = np.array([curve.sample(f) for f in np.linspace(0, 1, 4001)])
                dt = duration/4000
                self.assertLessEqual(np.max(np.abs(np.diff(samples, axis=0)))/dt, .35001)
                self.assertLessEqual(np.max(np.abs(np.diff(samples, n=2, axis=0)))/dt**2, .60001)

    def test_body_moves_forward_during_the_swing(self):
        for direction, plan in library().plans.items():
            first = plan.cycle.frame(.5*plan.settings.double_support)
            last = plan.cycle.frame(.5)
            step = plan.settings.forward_step if direction == 'forward' else -plan.settings.backward_step
            self.assertAlmostEqual(last.com[0]-first.com[0], step, places=10)
            self.assertEqual(first.support, 'l')
            middle = plan.cycle.frame(.25*(1+plan.settings.double_support))
            self.assertAlmostEqual(middle.feet['r'][2], plan.settings.clearance, places=10)

    def test_original_gait_configuration_is_unchanged(self):
        # Imports must not monkey-patch the original module's globals.
        self.assertEqual(model.BODY_ROLL, .20)
        self.assertEqual(model.CROUCH_DEPTH, .040)
        self.assertAlmostEqual(model.BODY_HEIGHT, model.ZERO_HEIGHT-.040)

    def test_invalid_starting_pose_is_rejected(self):
        from biped_cpg.planner import CompiledGait
        with self.assertRaises(ValueError):
            CompiledGait(initial=np.full(12, float('nan')))
        with self.assertRaises(ValueError):
            CompiledGait(direction='sideways')


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.walker = Walker(fresh_library())
        self.now = 1.

    def advance(self, until, maximum=240.):
        end = self.now+maximum
        while not until():
            self.now += .02
            output = self.walker.tick(self.now)
            self.assertNotEqual(self.walker.state, 'fault', self.walker.fault)
            if output is not None:
                self.assertTrue(np.all(np.isfinite(output)))
            if self.now > end:
                self.fail('controller did not reach the requested state')

    def test_no_commands_before_start_and_stop_while_preparing(self):
        self.assertIsNone(self.walker.tick(self.now))
        self.walker.command('start', self.now)
        self.walker.command('stop', self.now)
        self.advance(lambda: self.walker.state == 'idle')
        self.assertEqual(self.walker.landings, 0)
        self.assertGreater(np.max(np.abs(self.walker.last_pose)), .1)

    def test_one_step_then_hold_and_restart_in_reverse(self):
        self.walker.command('step', self.now)
        self.advance(lambda: self.walker.state == 'idle')
        self.assertEqual(self.walker.landings, 1)
        self.assertFalse(self.walker.status()['stop_pending'])
        held = self.walker.last_pose.copy()
        self.now += .02
        np.testing.assert_array_equal(self.walker.tick(self.now), held)
        self.walker.command('backward', self.now)
        np.testing.assert_array_equal(self.walker.plan.startup.curve.rows[0], held)
        self.advance(lambda: self.walker.state == 'walking' and self.walker.cpg.right > 1.)
        self.walker.command('stop', self.now)
        self.advance(lambda: self.walker.state == 'idle')
        self.assertEqual(self.walker.landings, 1)

    def test_stop_during_left_swing_finishes_left_landing(self):
        self.walker.command('forward', self.now)
        self.advance(lambda: self.walker.state == 'walking' and self.walker.cpg.right > 1.7*math.pi)
        self.walker.command('stop', self.now)
        self.advance(lambda: self.walker.state == 'idle')
        self.assertEqual(self.walker.landings, 2)
        self.assertEqual(self.walker.stop_side, 'left')

    def test_delayed_callback_does_not_jump_along_the_cycle(self):
        self.walker.command('forward', self.now)
        self.advance(lambda: self.walker.state == 'walking' and self.walker.cpg.right > .3)
        before = self.walker.cpg.right
        self.now += .1
        self.walker.tick(self.now)
        self.assertLessEqual(self.walker.cpg.right-before, self.walker.cpg.omega*.04+1e-10)

    def test_clock_gap_holds_target_and_reset_suppresses_output(self):
        self.walker.command('forward', self.now)
        self.advance(lambda: self.walker.state == 'walking')
        held = self.walker.last_pose.copy()
        self.now += .5
        np.testing.assert_array_equal(self.walker.tick(self.now), held)
        self.assertEqual(self.walker.state, 'fault')
        with self.assertRaises(ValueError):
            self.walker.command('start', self.now)
        self.assertIsNone(self.walker.tick(0.))
        self.assertTrue(self.walker.suppress)

    def test_restart_resynchronizes_clock_after_a_slow_compile(self):
        self.walker.command('start', self.now)
        self.now += 2.  # command construction may have blocked while a simulator advanced
        self.walker.tick(self.now)
        self.assertEqual(self.walker.state, 'preparing')
        self.assertAlmostEqual(self.walker.elapsed, 0.)

    def test_conflicting_writer_fault_does_not_send_commands(self):
        self.walker.command('start', self.now)
        self.walker.fail('another writer', suppress=True)
        self.assertIsNone(self.walker.tick(self.now+.02))
