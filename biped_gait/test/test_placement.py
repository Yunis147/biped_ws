"""Physical geometry regressions: bent knees, finite descent and landed anchors."""
import unittest
import numpy as np
from test_model_walk import plans
from biped_gait.model_walk import NAMES
from biped_gait.placement import Placement, PhaseGeometry, ground_estimate
from biped_gait import quasi_static_walk as m


class PlacementTests(unittest.TestCase):
    def row(self, name):
        return np.array([plans().cache['forward'].keyframes[name][n] for n in NAMES])

    def test_ground_height_changes_with_measured_knee_bend(self):
        q = self.row('ready')
        first = ground_estimate(q, 'l')
        q[NAMES.index('l_knee_joint')] -= .10
        second = ground_estimate(q, 'l')
        self.assertGreater(abs(first.body_height-second.body_height), .001)
        self.assertLess(first.body_height, m.ZERO_HEIGHT-.010)

    def test_descent_stops_at_the_measured_plane_and_stays_bent(self):
        q = self.row('swing_right')
        p = Placement(q, 'l', plans().settings)
        total = 0.
        for _ in range(400):
            before = ground_estimate(q, 'l').clearance
            target, descent, done, exhausted = p.target(q, q, .02)
            after = ground_estimate(target, 'l').clearance
            self.assertGreaterEqual(after, -1e-6)
            self.assertLessEqual(before-after, plans().settings.landing_speed*.02+1e-6)
            self.assertLess(target[NAMES.index('r_knee_joint')], -.12)
            total += descent; p.accept(descent); q = target
            if done: break
            self.assertFalse(exhausted)
        self.assertTrue(done)
        self.assertLessEqual(total, p.initial_clearance+1e-7)

    def test_touchdown_captures_measured_pose_instead_of_finishing_extension(self):
        measured = self.row('place_right')
        commanded = self.row('swing_right')
        p = Placement(commanded, 'l', plans().settings)
        target, descent, done, _ = p.target(commanded, measured, .02)
        self.assertFalse(done)
        self.assertEqual(descent, 0.)
        self.assertLess(np.max(np.abs(target-commanded)),1e-4)
        previous=commanded.copy();velocity=np.zeros(12)
        for _ in range(400):
            target,descent,done,_=p.target(previous,measured,.02)
            next_velocity=(target-previous)/.02
            self.assertLessEqual(np.max(np.abs(next_velocity-velocity))/.02,plans().settings.placement_capture_acceleration+.002)
            self.assertGreaterEqual(ground_estimate(target,'l').clearance,-1e-6)
            p.accept(descent);previous=target.copy();velocity=next_velocity
            if done:break
        self.assertTrue(done)
        self.assertAlmostEqual(ground_estimate(target, 'l').clearance, 0., places=6)
        np.testing.assert_array_equal(target[6:],commanded[6:])
        self.assertLess(target[NAMES.index('r_knee_joint')],-.12)
        # Once captured, a changing encoder sample cannot make targets chase gravity.
        later = measured.copy(); later[NAMES.index('r_knee_joint')] -= .005
        np.testing.assert_array_equal(p.target(target, later, .02)[0], target)

    def test_placement_entry_ramps_and_rejected_trials_do_not_advance_the_ramp(self):
        q=self.row('swing_right')
        p=Placement(q,'l',plans().settings)
        first,descent,_,_=p.target(q,q,.02)
        repeated=p.target(q,q,.02)[0]
        np.testing.assert_array_equal(first,repeated)
        self.assertLess(descent,.01*plans().settings.landing_speed*.02)
        for _ in range(60):
            target,descent,_,_=p.target(q,q,.02)
            p.accept(descent)
        self.assertAlmostEqual(descent,plans().settings.landing_speed*.02)

    def test_touchdown_retains_support_effort_and_levels_the_command_floor(self):
        commanded=self.row('swing_right')
        actual=dict(zip(NAMES,self.row('place_right')))
        rotation=m._foot_transform(actual,'l')[0].T
        # Model a loaded stance whose encoders show 1 mm of body sag.
        for side in 'rl':
            actual=m._leg_pose_ik(actual,side,m._sole_center(actual,side)+rotation.T@[0.,0.,.001],
                                  m._foot_transform(actual,side)[0])
        measured=np.array([actual[n] for n in NAMES])
        p=Placement(commanded,'l',plans().settings)
        previous=commanded.copy()
        for _ in range(400):
            target,descent,done,_=p.target(previous,measured,.02)
            np.testing.assert_array_equal(target[6:],commanded[6:])
            p.accept(descent);previous=target.copy()
            if done:break
        self.assertTrue(done)
        self.assertAlmostEqual(ground_estimate(target,'l').clearance,0.,places=6)
        self.assertGreater(np.max(np.abs(target[6:]-measured[6:])),.001)

    def test_a_nonmoving_leg_cannot_consume_an_unbounded_downward_search(self):
        measured = self.row('swing_right'); target = measured.copy()
        p = Placement(measured, 'l', plans().settings)
        for _ in range(400):
            target, descent, done, exhausted = p.target(target, measured, .02)
            p.accept(descent)
            if exhausted: break
        self.assertTrue(exhausted)
        self.assertFalse(done)
        self.assertLessEqual(p.commanded_descent, p.initial_clearance+1e-7)

    def test_next_transfer_preserves_both_landed_foot_anchors(self):
        plan = plans().cache['forward']
        start = self.row('place_right')
        # A bent stance 1 mm lower at touchdown.
        q = dict(zip(NAMES, start))
        for side in 'rl':
            q = m._leg_pose_ik(q, side, m._sole_center(q,side)+[0.,0.,.001], m._foot_transform(q,side)[0])
        anchor = np.array([q[n] for n in NAMES])
        geom = PhaseGeometry(plan.tables[('place_right','lean_right')][0], anchor, 'l')
        previous = anchor
        for u in np.linspace(0., 1., 41):
            previous = geom.pose(m.table_pose(plan.tables[('place_right','lean_right')], u), previous, u)
            _, _, feet, _ = geom.frame(previous)
            for side in 'rl': np.testing.assert_allclose(feet[side], geom.fa[side], atol=1e-6)
