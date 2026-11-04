"""Feedback and descent regression tests with synthetic inputs; no ROS/motor I/O."""
from functools import lru_cache
from pathlib import Path
import sys,math,unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'biped_gait'))
from biped_gait.feedback_walk import Plans, WalkingController, attitude_pose, reference_rotation, NAMES, m
from biped_gait.imu import tilt, rpy_matrix, TiltEstimator
from biped_gait.placement import Placement


@lru_cache(maxsize=1)
def plans():
    return Plans(precomputed=True)


def controller(simulation=False, **kwargs):
    p = Plans.__new__(Plans)
    p.settings, p.cache = plans().settings, dict(plans().cache)
    # These fixtures set tilt directly. Alignment tests supply real IMU history.
    kwargs.setdefault('align_imu_at_startup',False)
    return WalkingController(p, simulation=simulation, **kwargs)


def feed(c, now, q=None, angles=None, gyro=(0., 0., 0.)):
    q = (c.commanded if c.commanded is not None else c.nominal) if q is None else q
    c.feedback.joints(NAMES, q, np.zeros(12), now)
    c.feedback.diagnostics('torque ON', b'\x00', {'startup_positioning': 'False'}, now)
    c.imu.angles = tilt(reference_rotation(q)) if angles is None else np.array(angles)
    c.imu.rotation = rpy_matrix(*c.imu.angles)
    c.imu.gyro, c.imu.time, c.imu.calibrated = np.array(gyro), now, True


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.c = controller()
        self.now = 1.
        feed(self.c, self.now)

    def pose(self, name='ready', segment=None):
        self.c.nominal = np.array([self.c.plan.keyframes[name][n] for n in NAMES])
        self.c.commanded = self.c.nominal.copy()
        self.c.state = 'standing' if segment is None else 'moving'
        self.c.segment = segment
        self.c.last_time = self.now-.02
        feed(self.c, self.now)

    def test_no_commands_before_start_and_missing_inputs_refused(self):
        self.assertIsNone(self.c.tick(self.now))
        self.c.imu.time = None
        with self.assertRaisesRegex(ValueError, 'IMU'):
            self.c.command('start', self.now)

    def test_feedback_opposes_error_and_retains_world_feet(self):
        self.pose()
        nominal = self.c.nominal.copy()
        feed(self.c, self.now, angles=(.02, 0.))
        output = self.c.tick(self.now)
        self.assertLess(self.c.correction[0], 0.)
        old_body = reference_rotation(nominal)
        new_body = old_body @ rpy_matrix(*self.c.correction)
        for side in 'rl':
            q0, q1 = dict(zip(NAMES, nominal)), dict(zip(NAMES, output))
            np.testing.assert_allclose(new_body @ m._sole_center(q1, side), old_body @ m._sole_center(q0, side), atol=1e-7)
            np.testing.assert_allclose(new_body @ m._foot_transform(q1, side)[0], np.eye(3), atol=1e-6)

    def test_feedback_tracks_planned_lean_instead_of_zero(self):
        self.pose('lean_left')
        self.c.tick(self.now)
        np.testing.assert_allclose(self.c.requested, 0, atol=1e-9)
        self.assertAlmostEqual(self.c.imu.angles[0], -self.c.s.body_roll, places=6)

    def test_damping_filters_gyro_once_per_source_sample(self):
        self.pose()
        self.c.imu.sample_time=self.now
        self.c.tick(self.now)
        self.now+=.02
        feed(self.c,self.now,gyro=(.10,0.,0.))
        self.c.imu.sample_time=self.now
        self.c.tick(self.now)
        expected=.10*(1-math.exp(-.02/self.c.s.gyro_filter_time))
        self.assertAlmostEqual(self.c.balance_rate[0],expected)
        self.assertAlmostEqual(self.c.requested[0],-self.c.s.kd*expected)
        self.now+=.02
        feed(self.c,self.now,gyro=(.10,0.,0.))
        self.c.tick(self.now)
        self.assertAlmostEqual(self.c.balance_rate[0],expected)

    def test_lifted_left_foot_does_not_define_body_attitude(self):
        self.pose('swing_left')
        self.c.support='r'
        self.c.nominal[NAMES.index('l_ankle_roll_joint')] += .03
        self.c.commanded=self.c.nominal.copy()
        feed(self.c,self.now,angles=tilt(reference_rotation(self.c.nominal,'r')))
        self.c.tick(self.now)
        np.testing.assert_allclose(self.c.requested,0.,atol=1e-8)

    def test_corrections_and_complete_joint_updates_are_bounded(self):
        self.pose()
        previous = self.c.commanded.copy()
        for _ in range(60):
            self.now += .02
            feed(self.c, self.now, angles=(.02, .01), gyro=(.5, .5, 0.))
            output = self.c.tick(self.now)
            self.assertFalse(self.c.fault)
            self.assertLessEqual(np.max(np.abs(self.c.correction)), math.radians(1.5)+1e-10)
            self.assertLessEqual(np.max(np.abs(output-previous)), .25*.02+1e-7)
            previous = output.copy()

    def test_large_lean_holds_without_advancing(self):
        self.pose()
        before = self.c.commanded.copy()
        feed(self.c, self.now, angles=(math.radians(9), 0.))
        np.testing.assert_array_equal(self.c.tick(self.now), before)
        self.assertIn('tilt', self.c.fault)
        self.assertIsNotNone(self.c.snapshot)

    def landing(self, fraction=0.):
        segment = ('swing_right', 'place_right')
        self.pose('swing_right', segment)
        self.c.elapsed = self.c.plan.durations[segment]*fraction
        self.c.nominal = m.table_pose(self.c.plan.tables[segment], fraction)
        self.c.commanded = self.c.nominal.copy()
        feed(self.c, self.now)
        self.c.placement = Placement(self.c.feedback.q, self.c.imu.angles, 'l', self.c.s)
        return self.c.commanded.copy()

    def test_landing_stops_on_tilt_rotation_or_tracking(self):
        for symptom in ('tilt', 'rate', 'tracking'):
            self.c = controller()
            held = self.landing()
            if symptom == 'tilt':
                angles = self.c.imu.angles+[.03, 0]
                feed(self.c, self.now, angles=angles)
            elif symptom == 'rate':
                feed(self.c, self.now, gyro=(.13, 0., 0.))
            else:
                q = held.copy(); q[0] += .04
                feed(self.c, self.now, q=q, angles=tilt(reference_rotation(held)))
            np.testing.assert_array_equal(self.c.tick(self.now), held)
            self.assertEqual(self.c.state, 'placement_hold')
            self.assertIn('landing interrupted', self.c.interruption)
            self.assertEqual(self.c.landings, 0)
            if symptom == 'rate':
                snapshot=self.c.interruption_snapshot
                self.assertAlmostEqual(snapshot['gyro_rad_s'][0],.13)
                self.now += .02
                feed(self.c,self.now)
                self.c.tick(self.now)
                self.assertIs(self.c.interruption_snapshot,snapshot)
                self.assertAlmostEqual(snapshot['gyro_rad_s'][0],.13)

    def test_landing_stops_on_predicted_ground_mismatch(self):
        held = self.landing(.9)
        angles = tilt(reference_rotation(held)) + [.02, 0.]
        feed(self.c, self.now, angles=angles)
        np.testing.assert_array_equal(self.c.tick(self.now), held)
        self.assertLess(self.c.clearance, -.001)
        self.assertIn('contact NOT confirmed', self.c.interruption)

    def test_filtered_damping_does_not_hide_a_raw_landing_rate_warning(self):
        self.landing()
        self.c.tick(self.now)
        self.now+=.02
        feed(self.c,self.now,gyro=(.13,0.,0.))
        self.c.tick(self.now)
        self.assertLess(self.c.balance_rate[0],self.c.s.landing_rotation_rate)
        self.assertEqual(self.c.state,'placement_hold')
        self.assertIn('rotation_rate_rad_s',self.c.interruption)

    def test_nominal_landing_has_no_overtravel_and_bounded_descent(self):
        for p in plans().cache.values():
            for side in ('right', 'left'):
                seg = ('swing_'+side, 'place_'+side)
                table, duration = p.tables[seg], p.durations[seg]
                heights = []
                for u in np.linspace(0, 1, 401):
                    q = dict(zip(NAMES, m.table_pose(table, u)))
                    _, feet, _ = m._ground_frame(q, 'l' if side == 'right' else 'r')
                    swing, stance = feet[side[0]], feet['l' if side == 'right' else 'r']
                    heights.append(float(swing[:, 2].min()-stance[:, 2].min()))
                self.assertGreaterEqual(min(heights), -1e-6)
                self.assertLessEqual(np.max(np.abs(np.diff(heights)))/(duration/400), .002001)

    def test_lag_pause_is_kept_and_fault_snapshot_names_joint(self):
        self.pose()
        for _ in range(110):
            self.now += .02
            q = self.c.commanded.copy(); q[0] += .06
            feed(self.c, self.now, q=q, angles=(0., 0.))
            self.c.tick(self.now)
        self.assertIn('tracking', self.c.fault)
        self.assertEqual(self.c.snapshot['tracking_joint'], NAMES[0])
        self.assertAlmostEqual(self.c.snapshot['tracking_error_rad'], .06)

    def test_missing_imu_or_bad_bridge_suppresses_output(self):
        self.pose()
        self.now += .3
        self.assertIsNone(self.c.tick(self.now))
        self.assertTrue(self.c.suppress)
        self.c = controller();self.pose()
        self.c.feedback.diagnostics('torque ON', 2, {'startup_positioning':'False'}, self.now)
        self.assertIsNone(self.c.tick(self.now))
        self.assertIn('health', self.c.fault)

    def test_one_step_completes_and_stops(self):
        self.c.command('step', self.now)
        for _ in range(6000):
            self.now += .02
            feed(self.c, self.now)
            self.c.tick(self.now)
            self.assertFalse(self.c.fault, self.c.fault)
            if self.c.state == 'standing':
                break
        self.assertEqual(self.c.state, 'standing')
        self.assertEqual(self.c.landings, 1)
        # Restart from the held stride in reverse, stop during a lift, and
        # verify the airborne step finishes without resetting planted feet.
        self.c.command('backward', self.now)
        stopping = False
        for _ in range(6500):
            self.now += .02
            feed(self.c, self.now)
            if self.c.segment and self.c.segment[1].startswith('lift_') and not stopping:
                self.c.command('stop', self.now)
                stopping = True
            self.c.tick(self.now)
            self.assertFalse(self.c.fault, self.c.fault)
            if self.c.state == 'standing':
                break
        self.assertTrue(stopping)
        self.assertEqual(self.c.state, 'standing')
        self.assertEqual(self.c.landings, 1)

    def test_quiet_gyro_calibration_preserves_gravity_reference(self):
        imu = TiltEstimator(source='raw')
        for i in range(121):
            imu.update([.005, -.004, 0.], [0., 0., 9.81], [0., 0., 0., 0.], 1+i*.01, orientation_valid=False)
        bias = imu.calibrate(2.2)
        np.testing.assert_allclose(bias, [.005, -.004, 0.], atol=1e-10)
        self.assertTrue(imu.calibrated)
        self.assertLess(np.max(np.abs(imu.angles)), .01)
