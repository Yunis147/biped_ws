"""Physical geometry regressions: bent knees, finite descent and landed anchors."""
from dataclasses import replace
import unittest
import numpy as np
from test_feedback import plans, controller, feed
from biped_gait.feedback_walk import NAMES, attitude_pose_matrix, reference_rotation
from biped_gait.placement import Placement, PhaseGeometry, ground_estimate
from biped_gait.imu import tilt, rpy_matrix
from biped_gait import quasi_static_walk as m


class PlacementTests(unittest.TestCase):
    def row(self, name):
        return np.array([plans().cache['forward'].keyframes[name][n] for n in NAMES])

    def test_ground_height_changes_with_measured_knee_bend(self):
        q = self.row('ready')
        first = ground_estimate(q, [0.,0.], 'l')
        q[NAMES.index('l_knee_joint')] -= .10
        second = ground_estimate(q, [0.,0.], 'l')
        self.assertGreater(abs(first.body_height-second.body_height), .001)
        self.assertLess(first.body_height, m.ZERO_HEIGHT-.010)

    def test_descent_stops_at_the_measured_plane_and_stays_bent(self):
        q = self.row('swing_right')
        angles = tilt(reference_rotation(q))
        p = Placement(q, angles, 'l', plans().settings)
        total = 0.
        for _ in range(400):
            before = ground_estimate(q, angles, 'l').clearance
            target, descent, done, exhausted = p.target(q, q, angles, .02)
            after = ground_estimate(target, angles, 'l').clearance
            self.assertGreaterEqual(after, -1e-6)
            self.assertLessEqual(before-after, .002*.02+1e-6)
            self.assertLess(target[NAMES.index('r_knee_joint')], -.12)
            total += descent; p.accept(descent); q = target
            if done: break
            self.assertFalse(exhausted)
        self.assertTrue(done)
        self.assertLessEqual(total, p.initial_clearance+1e-7)

    def test_touchdown_captures_measured_pose_instead_of_finishing_extension(self):
        measured = self.row('place_right')
        commanded = self.row('swing_right')
        angles = tilt(reference_rotation(measured))
        p = Placement(commanded, angles, 'l', plans().settings)
        target, descent, done, _ = p.target(commanded, measured, angles, .02)
        self.assertTrue(done)
        self.assertEqual(descent, 0.)
        self.assertAlmostEqual(ground_estimate(target, angles, 'l').clearance, 0., places=6)
        self.assertLess(abs(target[NAMES.index('r_knee_joint')]-measured[NAMES.index('r_knee_joint')]), 1e-5)
        # Once captured, a changing encoder sample cannot make targets chase gravity.
        later = measured.copy(); later[NAMES.index('r_knee_joint')] -= .005
        np.testing.assert_array_equal(p.target(target, later, angles, .02)[0], target)

    def test_a_nonmoving_leg_cannot_consume_an_unbounded_downward_search(self):
        measured = self.row('swing_right'); target = measured.copy()
        angles = tilt(reference_rotation(measured))
        p = Placement(measured, angles, 'l', plans().settings)
        for _ in range(400):
            target, descent, done, exhausted = p.target(target, measured, angles, .02)
            p.accept(descent)
            if exhausted: break
        self.assertTrue(exhausted)
        self.assertFalse(done)
        self.assertLessEqual(p.commanded_descent, p.initial_clearance+1e-7)

    def test_next_transfer_preserves_both_landed_foot_anchors(self):
        plan = plans().cache['forward']
        start = self.row('place_right')
        # A bent stance 1 mm lower and 0.3 degrees more upright at touchdown.
        rotation = rpy_matrix(.005, 0.)
        anchor = attitude_pose_matrix(start, rotation)
        q = dict(zip(NAMES, anchor))
        for side in 'rl':
            q = m._leg_pose_ik(q, side, m._sole_center(q,side)+[0.,0.,.001], m._foot_transform(q,side)[0])
        anchor = np.array([q[n] for n in NAMES])
        geom = PhaseGeometry(plan.tables[('place_right','lean_right')][0], anchor, 'l')
        previous = anchor
        for u in np.linspace(0., 1., 41):
            previous = geom.pose(m.table_pose(plan.tables[('place_right','lean_right')], u), previous, u)
            _, _, feet, _ = geom.frame(previous)
            for side in 'rl': np.testing.assert_allclose(feet[side], geom.fa[side], atol=1e-6)

    def test_transfer_reverses_before_hard_tilt_fault_and_keeps_feedback(self):
        c = controller(); seg=('ready','lean_left')
        c.segment=seg; c.state='moving'; c.elapsed=c.plan.durations[seg]*.4; c.last_time=1.
        c.nominal=m.table_pose(c.plan.tables[seg],.4);c.commanded=c.nominal.copy()
        c.phase_geometry=PhaseGeometry(c.plan.tables[seg][0], c.plan.tables[seg][0], 'l')
        now=1.02; before=c.elapsed
        reference=tilt(reference_rotation(c.nominal))
        feed(c,now,angles=reference+[-.025,0.])
        result=c.tick(now)
        self.assertEqual(c.state,'recovering')
        self.assertLess(c.elapsed,before)
        self.assertFalse(c.fault)
        self.assertIsNotNone(result)
        self.assertGreater(c.correction[0],0.)

    def test_placement_interruption_relaxes_swing_and_keeps_posture_feedback(self):
        c=controller();q=self.row('swing_right');c.nominal=q.copy();c.commanded=q.copy()
        c.state='moving';c.segment=('swing_right','place_right');c.last_time=1.
        feed(c,1.);c.placement=Placement(q,c.imu.angles,'l',c.s)
        c._placement_hold('early contact cue',1.)
        before=c.commanded.copy()
        for i in range(1,31):
            now=1.+i*.02;feed(c,now,angles=tilt(reference_rotation(c.nominal))+[.01,0.])
            c.tick(now)
        self.assertFalse(c.fault,c.fault)
        self.assertEqual(c.state,'placement_hold')
        self.assertGreater(np.max(np.abs(c.commanded-before)), 1e-5)
        self.assertLess(c.correction[0],0.)
