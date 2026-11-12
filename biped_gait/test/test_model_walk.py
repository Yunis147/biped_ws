"""Joint-only controller regressions; never connects to motors or ROS."""
from functools import lru_cache
from pathlib import Path
import json
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'biped_gait'))
from biped_gait.model_walk import Plans, WalkingController, NAMES, m
from biped_gait.placement import Placement


@lru_cache(maxsize=1)
def plans():
    return Plans(precomputed=True)


def controller(simulation=False):
    p = Plans.__new__(Plans)
    p.settings, p.cache = plans().settings, dict(plans().cache)
    return WalkingController(p, simulation=simulation)


def feed(c, now, q=None, velocity=None):
    q = (c.commanded if c.commanded is not None else c.nominal) if q is None else q
    c.feedback.joints(NAMES, q, np.zeros(12) if velocity is None else velocity, now)
    c.feedback.diagnostics('torque ON', b'\x00', {'startup_positioning': 'False'}, now)


class ModelWalkTests(unittest.TestCase):
    def test_no_output_before_start_and_no_orientation_input_needed(self):
        c = controller()
        self.assertIsNone(c.tick(1.))
        with self.assertRaisesRegex(ValueError, 'joint'):
            c.command('start', 1.)
        feed(c, 1.)
        c.command('start', 1.)
        self.assertEqual(c.state, 'approach')
        self.assertIsNotNone(c.tick(1.))
        self.assertFalse(hasattr(c, 'imu'))
        self.assertFalse(any(k.startswith('imu') for k in json.loads(json.dumps(c.status(1.)))))

    def test_initial_crouch_completes_with_only_encoder_and_bridge_data(self):
        c = controller(); now = 1.; feed(c, now); c.command('stand', now)
        for _ in range(2000):
            now += .02; feed(c, now); c.tick(now)
            self.assertFalse(c.fault, c.status(now))
            if c.state == 'standing': break
        self.assertEqual(c.state, 'standing')
        self.assertEqual(c.landings, 0)
        self.assertGreater(min(-c.nominal[NAMES.index(s+'_knee_joint')] for s in 'rl'), .2)

    def test_incomplete_invalid_and_stale_joint_states_are_rejected(self):
        c = controller()
        for q in (np.zeros(11), np.full(12, np.nan), m.HI+.01):
            with self.assertRaises(ValueError): c.feedback.joints(NAMES, q, [], 1.)
        with self.assertRaisesRegex(ValueError, 'timestamp'):
            c.feedback.joints(NAMES, np.zeros(12), np.zeros(12), 1., .3)
        feed(c, 1.); c.command('stand', 1.)
        self.assertIsNone(c.tick(1.3))
        self.assertTrue(c.suppress)

    def test_bridge_health_and_torque_are_required_on_hardware(self):
        for message, level, values in [('torque OFF', 0, {}), ('torque ON', 2, {'startup_positioning':'False'}),
                                      ('torque ON', 0, {'startup_positioning':'True'})]:
            c = controller(); feed(c, 1.)
            c.feedback.diagnostics(message, level, values, 1.)
            with self.assertRaises(ValueError): c.command('start', 1.)

    def test_lag_pauses_progress_and_records_the_joint_on_timeout(self):
        c = controller(); feed(c, 1.); c.command('start', 1.)
        held = c.commanded.copy()
        for i in range(1, 112):
            now = 1.+i*.02; q = held.copy(); q[0] += .06
            feed(c, now, q); c.tick(now)
        self.assertIn('tracking', c.fault)
        np.testing.assert_array_equal(c.commanded, held)
        self.assertEqual(c.snapshot['tracking_joint'], NAMES[0])

    def test_landing_tracking_interruption_relaxes_only_swing_leg(self):
        c = controller(); row = c.plan.keyframes['swing_right']
        c.nominal = c.commanded = np.array([row[n] for n in NAMES])
        c.state = 'moving'; c.segment = ('swing_right','place_right'); c.support = 'l'; c.last_time = 1.
        feed(c, 1.); c.placement = Placement(c.commanded, 'l', c.s)
        held = c.commanded.copy(); measured = held.copy(); measured[0] += .03
        feed(c, 1.02, measured); c.tick(1.02)
        self.assertEqual(c.state, 'placement_hold', c.status(1.02))
        self.assertEqual(c.landings, 0)
        np.testing.assert_array_equal(c.relief_target[6:], held[6:])
        self.assertIn('contact not measured', c.reason)

    def test_one_step_and_reverse_restart_finish_before_stopping(self):
        c = controller(); now = 1.; feed(c, now); c.command('step', now)
        for _ in range(6500):
            now += .02; feed(c, now); c.tick(now)
            self.assertFalse(c.fault, c.status(now))
            if c.state == 'standing': break
        self.assertEqual((c.state,c.landings), ('standing',1))
        c.command('backward', now)
        stopping = False
        for _ in range(7500):
            now += .02; feed(c, now)
            if c.segment and c.segment[1].startswith('lift_') and not stopping:
                c.command('stop', now); stopping = True
            c.tick(now)
            self.assertFalse(c.fault, c.status(now))
            if c.state == 'standing': break
        self.assertTrue(stopping)
        self.assertEqual((c.state,c.landings), ('standing',1))
