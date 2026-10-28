"""Run with PYTHONPATH=biped_balance:biped_gait python3 -m unittest discover -s biped_balance/test -v."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'biped_balance'), str(ROOT/'biped_gait')]

from biped_balance.controller import BalanceExperiment, Settings
from biped_balance.estimation import Feedback, GRAVITY, TiltEstimator, rpy_matrix
from biped_balance.kinematics import LOWER, NAMES, UPPER, attitude_pose, leg_pose_jacobian, standing_poses


def quaternion(roll=0., pitch=0., yaw=0.):
    cr, sr, cp, sp, cy, sy = (math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2),
                             math.sin(pitch/2), math.cos(yaw/2), math.sin(yaw/2))
    return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]


class EstimationTests(unittest.TestCase):
    def test_quaternion_and_all_vectors_use_mount_transform(self):
        mount = math.pi/2
        estimate = TiltEstimator(mount_rpy=(mount, 0., 0.))
        rotation = rpy_matrix(mount, 0.)
        body_gyro = np.array([.01, .02, -.03])
        body_accel = rpy_matrix(.03, 0.).T @ [0., 0., GRAVITY]
        estimate.update(rotation.T@body_gyro, rotation.T@body_accel,
                        quaternion(mount+.03), 1.)
        np.testing.assert_allclose(estimate.angles, [.03, 0.], atol=1e-12)
        np.testing.assert_allclose(estimate.gyro, body_gyro, atol=1e-12)

    def test_raw_six_axis_initializes_from_gravity(self):
        estimate = TiltEstimator(source='raw')
        angles = [.08, -.04]
        acceleration = rpy_matrix(*angles).T @ [0., 0., GRAVITY]
        estimate.update([0, 0, 0], acceleration, [0, 0, 0, 0], 1., False)
        np.testing.assert_allclose(estimate.angles, angles, atol=1e-12)
        self.assertEqual(estimate.used_source, 'raw complementary')

    def test_raw_filter_rejects_acceleration_magnitude_during_motion(self):
        estimate = TiltEstimator(source='raw')
        estimate.update([0, 0, 0], [0, 0, GRAVITY], [], 1., False)
        estimate.update([.1, 0, 0], [GRAVITY*2, 0, GRAVITY], [], 1.1, False)
        self.assertAlmostEqual(estimate.angles[0], .01, places=10)
        self.assertAlmostEqual(estimate.angles[1], 0., places=10)

    def test_auto_falls_back_when_quaternion_is_unavailable(self):
        estimate = TiltEstimator()
        estimate.update([0, 0, 0], [0, 0, GRAVITY], [0, 0, 0, 0], 1., False)
        self.assertEqual(estimate.used_source, 'raw complementary')

    def test_calibration_estimates_gyro_bias_without_hiding_lean(self):
        estimate = TiltEstimator()
        for i in range(151):
            estimate.update([.01, -.02, .005], [0, 0, GRAVITY], quaternion(), 1+i*.01)
        np.testing.assert_allclose(estimate.calibrate(2.5), [.01, -.02, .005])
        estimate.update([.01, -.02, .005], [0, 0, GRAVITY], quaternion(), 2.51)
        np.testing.assert_allclose(estimate.gyro, 0., atol=1e-12)
        leaning = TiltEstimator()
        for i in range(151):
            leaning.update([0, 0, 0], [0, 0, GRAVITY], quaternion(.15), 1+i*.01)
        with self.assertRaisesRegex(ValueError, 'upright'):
            leaning.calibrate(2.5)

    def test_bad_samples_and_long_gaps_do_not_preserve_calibration(self):
        estimate = TiltEstimator()
        with self.assertRaises(ValueError):
            estimate.update([float('nan'), 0, 0], [0, 0, GRAVITY], quaternion(), 1.)
        for i in range(151):
            estimate.update([0, 0, 0], [0, 0, GRAVITY], quaternion(), 1+i*.01)
        estimate.calibrate(2.5)
        estimate.update([0, 0, 0], [0, 0, GRAVITY], quaternion(), 3.)
        self.assertFalse(estimate.calibrated)

    def test_named_joint_order_and_existing_load_diagnostics(self):
        feedback = Feedback(NAMES)
        feedback.joints(NAMES[::-1], list(range(12)), [], 1.)
        np.testing.assert_array_equal(feedback.q, list(range(12))[::-1])
        with self.assertRaises(ValueError):
            feedback.joints(NAMES[:-1], [0.]*11, [], 2.)
        feedback.diagnostics('torque ON', 0,
                             {NAMES[0]: 'id 7  30 C  7.4 V  load 123', 'startup_positioning': 'False'}, 1.)
        self.assertEqual(feedback.loads[NAMES[0]], 123)
        self.assertEqual(feedback.voltage[NAMES[0]], 7.4)
        self.assertEqual(feedback.temperature[NAMES[0]], 30)
        self.assertEqual(feedback.bridge_problem, '')
        feedback.diagnostics('torque ON', b'\x00', {}, 1.1)
        self.assertEqual(feedback.bridge_problem, '')
        feedback.diagnostics('torque ON', b'\x02', {}, 1.2)
        self.assertIn('health', feedback.bridge_problem)
        feedback.diagnostics('torque OFF', 0, {}, 2.)
        self.assertIn('OFF', feedback.bridge_problem)


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.poses, cls.tables = standing_poses()

    def test_geometric_jacobian_matches_independent_finite_differences(self):
        q = self.poses['center']
        from biped_gait import quasi_static_walk as model
        for side in 'rl':
            rotation, position, jacobian = leg_pose_jacobian(q, side)
            for k, joint in enumerate(model.LEG[side]):
                perturbed = q.copy()
                perturbed[NAMES.index(joint['name'])] += 1e-6
                r1, p1, _ = leg_pose_jacobian(perturbed, side)
                np.testing.assert_allclose(jacobian[:3, k], (p1-position)/1e-6, atol=2e-7)
                skew = ((r1-rotation)/1e-6) @ rotation.T
                np.testing.assert_allclose(jacobian[3:, k], [skew[2, 1], skew[0, 2], skew[1, 0]], atol=1e-6)

    def test_attitude_correction_preserves_both_world_foot_poses(self):
        for nominal in self.poses.values():
            for angles in ([.02, 0.], [0., -.02], [-.02, .015]):
                corrected = attitude_pose(nominal, angles)
                body = rpy_matrix(*angles)
                self.assertTrue(np.all(corrected >= LOWER) and np.all(corrected <= UPPER))
                for side in 'rl':
                    r0, p0, _ = leg_pose_jacobian(nominal, side)
                    r1, p1, _ = leg_pose_jacobian(corrected, side)
                    np.testing.assert_allclose(body@p1, p0, atol=3e-7)
                    np.testing.assert_allclose(body@r1, r0, atol=3e-6)

    def test_weight_transfer_keeps_spacing_and_flat_soles(self):
        for (a, b), table in self.tables.items():
            first_spacing = None
            for row in table:
                r1, p1, _ = leg_pose_jacobian(row, 'r')
                r2, p2, _ = leg_pose_jacobian(row, 'l')
                np.testing.assert_allclose(r1, np.eye(3), atol=1e-6)
                np.testing.assert_allclose(r2, np.eye(3), atol=1e-6)
                if first_spacing is None:
                    first_spacing = p1-p2
                np.testing.assert_allclose(p1-p2, first_spacing, atol=3e-7)


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.poses, cls.tables = standing_poses()

    def make(self, actuate=True, axes=True):
        # Reuse immutable geometry while exercising the real state machine.
        from unittest.mock import patch
        with patch('biped_balance.controller.standing_poses', return_value=(self.poses, self.tables)):
            c = BalanceExperiment(actuate=actuate, axes_verified=axes)
        for i in range(151):
            self.feed(c, 1+i*.01, q=np.zeros(12))
        c.handle('calibrate', 2.5)
        return c

    def feed(self, c, now, q=None, angles=(0., 0.), torque=True):
        if q is None:
            q = c.command if c.command is not None else np.zeros(12)
        c.feedback.joints(NAMES, q, [0.]*12, now)
        c.feedback.diagnostics('torque ON' if torque else 'torque OFF', 0,
                               {'startup_positioning': 'False'}, now)
        acceleration = rpy_matrix(*angles).T @ [0., 0., GRAVITY]
        c.imu.update([0, 0, 0], acceleration, quaternion(*angles), now)

    def prepare(self, c):
        c.handle('stand', 2.5)
        now = 2.5
        for _ in range(1800):
            now += .02
            self.feed(c, now)
            c.tick(now)
            if c.state == 'standing':
                return now
        self.fail('standing preparation did not finish: '+c.reason)

    def test_monitor_never_returns_an_actuator_target(self):
        c = self.make(actuate=False)
        for i in range(10):
            t = 2.52+i*.02
            self.feed(c, t, angles=(.02, 0.))
            self.assertIsNone(c.tick(t))
        with self.assertRaisesRegex(ValueError, 'monitor-only'):
            c.handle('stand', t)

    def test_axes_torque_and_publisher_checks_before_standing(self):
        c = self.make(axes=False)
        with self.assertRaisesRegex(ValueError, 'verify IMU'):
            c.handle('stand', 2.5)
        c = self.make()
        with self.assertRaisesRegex(ValueError, 'another node'):
            c.handle('stand', 2.5, other_publishers=1)
        with self.assertRaisesRegex(ValueError, 'no subscriber'):
            c.handle('stand', 2.5, command_subscribers=0)
        self.feed(c, 2.52, torque=False)
        with self.assertRaisesRegex(ValueError, 'OFF'):
            c.handle('stand', 2.52)

    def test_standing_positive_roll_requests_negative_body_correction(self):
        c = self.make()
        now = self.prepare(c)
        c.handle('balance_on', now)
        for _ in range(20):
            now += .02
            self.feed(c, now, angles=(.04, -.03))
            previous = c.command.copy()
            c.tick(now)
            self.assertLessEqual(np.max(np.abs(c.command-previous)), c.settings.max_joint_speed*.02+1e-8)
        self.assertLess(c.correction[0], 0.)
        self.assertGreater(c.correction[1], 0.)
        self.assertTrue(np.all(np.abs(c.correction) <= c.settings.max_correction))

    def test_encoder_lag_pauses_the_path_without_an_endpoint_jump(self):
        c = self.make()
        c.handle('stand', 2.5)
        now = 2.5
        for _ in range(140):
            now += .02
            self.feed(c, now)
            c.tick(now)
        elapsed = c.elapsed
        for _ in range(30):
            now += .02
            measured = c.command.copy()
            measured[3] += .06
            self.feed(c, now, q=measured)
            c.tick(now)
        self.assertAlmostEqual(c.elapsed, elapsed)
        self.assertIn('lag', c.reason)

    def test_stale_data_latches_fault_and_conflicting_writer_stops_output(self):
        c = self.make()
        now = self.prepare(c)
        before = c.command.copy()
        output = c.tick(now+.4)
        self.assertEqual(c.state, 'fault')
        np.testing.assert_array_equal(output, before)
        self.feed(c, now+.42)
        self.assertIsNone(c.tick(now+.42, other_publishers=1))
        self.assertEqual(c.state, 'fault')

    def test_large_tracking_error_does_not_drive_to_zero(self):
        c = self.make()
        now = self.prepare(c)
        c.handle('balance_on', now)
        last = c.command.copy()
        q = last.copy()
        q[3] += .2
        self.feed(c, now+.02, q=q)
        output = c.tick(now+.02)
        self.assertEqual(c.state, 'fault')
        np.testing.assert_array_equal(output, last)
        c.handle('balance_off', now+.02)
        self.assertFalse(c.balance_enabled)
        np.testing.assert_array_equal(c.command, last)

    def test_small_weight_transfer_finishes_with_both_feet_planned_down(self):
        c = self.make()
        now = self.prepare(c)
        c.handle('balance_on', now)
        c.handle('lean_left', now)
        for _ in range(800):
            now += .02
            self.feed(c, now)
            c.tick(now)
            if c.state == 'standing':
                break
        self.assertEqual(c.location, 'left')
        np.testing.assert_allclose(c.nominal, c.poses['left'], atol=1e-8)
        c.handle('hold', now)
        self.assertEqual(c.state, 'holding')
        c.handle('monitor', now)
        self.assertIsNone(c.tick(now))

    def test_invalid_settings_fail_before_actuation(self):
        with self.assertRaises(ValueError):
            BalanceExperiment(settings=Settings(kp_roll=float('nan')))


if __name__ == '__main__':
    unittest.main()
