"""Position-loop regressions; no ROS, joint feedback or motor connection."""
from functools import lru_cache
from pathlib import Path
import json
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'biped_gait'))
from biped_gait.model_walk import Plans, WalkingController, NAMES, m


@lru_cache(maxsize=1)
def plans():
    return Plans(precomputed=True)


def controller():
    copy = Plans.__new__(Plans)
    copy.settings, copy.cache = plans().settings, dict(plans().cache)
    return WalkingController(copy)


class PositionLoopTests(unittest.TestCase):
    def run_until(self, c, now, check, seconds=220.):
        previous = c.commanded.copy()
        for _ in range(int(seconds/.02)):
            now += .02
            target = c.tick(now)
            self.assertIsNotNone(target)
            self.assertLessEqual(np.max(np.abs(target-previous)),c.s.max_joint_speed*.02+1e-6)
            previous = target.copy()
            if check(): return now
        self.fail(str(c.status()))

    def test_no_feedback_is_needed_to_start_or_keep_publishing(self):
        c = controller()
        self.assertIsNone(c.tick(1.))
        c.command('start', 1.)
        self.assertFalse(hasattr(c,'feedback'))
        self.assertFalse(hasattr(c,'imu'))
        now = self.run_until(c, 1., lambda:c.landings>=2)
        self.assertIn(c.state,('moving','dwelling'))
        self.assertIsNotNone(c.tick(now+.02))
        self.assertEqual(json.loads(json.dumps(c.status()))['control'],'timed position loop')

    def test_one_step_finishes_and_keeps_publishing_the_centred_stance(self):
        c = controller(); c.command('step',1.)
        now = self.run_until(c,1.,lambda:c.state=='standing')
        self.assertEqual(c.landings,1)
        held = c.commanded.copy()
        for i in range(1,101): np.testing.assert_array_equal(c.tick(now+i*.02),held)

    def test_stop_during_swing_finishes_landing_before_centring(self):
        c = controller(); c.command('start',1.)
        now = self.run_until(c,1.,lambda:c.segment[1]=='swing_right')
        c.command('stop',now)
        self.run_until(c,now,lambda:c.state=='standing')
        self.assertEqual(c.landings,1)

    def test_explicit_hold_and_resume_keep_the_same_phase(self):
        c = controller(); c.command('start',1.)
        now = self.run_until(c,1.,lambda:c.segment[1]=='swing_right')
        now += .5; c.tick(now)
        segment,elapsed,held = c.segment,c.elapsed,c.commanded.copy()
        c.command('hold',now)
        for i in range(1,51): np.testing.assert_array_equal(c.tick(now+i*.02),held)
        self.assertEqual((c.segment,c.elapsed),(segment,elapsed))
        c.command('start',now+1.)
        c.tick(now+1.02)
        self.assertGreater(c.elapsed,elapsed)
        self.assertEqual(c.segment,segment)

    def test_stand_and_reverse_restart_preserve_the_starting_pose(self):
        c = controller(); c.command('step',1.)
        now = self.run_until(c,1.,lambda:c.state=='standing')
        held = c.commanded.copy()
        c.command('backward',now)
        np.testing.assert_allclose(c.tick(now),held,atol=1e-7)
        now = self.run_until(c,now,lambda:c.segment[1]=='swing_right')
        c.command('stop',now)
        self.run_until(c,now,lambda:c.state=='standing')
        self.assertEqual(c.landings,1)

    def test_stand_keeps_both_knees_bent(self):
        c = controller(); c.command('stand',1.)
        self.run_until(c,1.,lambda:c.state=='standing')
        self.assertEqual(c.landings,0)
        self.assertGreater(min(-c.commanded[NAMES.index(s+'_knee_joint')] for s in 'rl'),.2)

    def test_both_feet_lift_one_centimetre_in_both_directions(self):
        for p in plans().cache.values():
            for name in ('lift_right_1st','lift_right','lift_left','swing_right','swing_left'):
                stance=m.STANCE[name];swing='r' if stance=='l' else 'l'
                _,feet,_=m._ground_frame(p.keyframes[name],stance)
                height=feet[swing][:,2].min()-feet[stance][:,2].min()
                self.assertAlmostEqual(height,.010,places=6)
