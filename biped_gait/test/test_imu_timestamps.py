"""Regressions for queued IMU messages sharing a ROS simulation clock tick."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from biped_gait.imu import TiltEstimator


class SensorTimeTests(unittest.TestCase):
    def test_raw_filter_integrates_source_intervals_with_one_receipt_time(self):
        imu = TiltEstimator(source='raw')
        imu.update([.2, 0., 0.], [0., 0., 9.81], [0.]*4, 10., False, sample_time=1.)
        for stamp in (1.01, 1.02):
            # Accelerating sample: trust gyro integration, not gravity tilt.
            imu.update([.2, 0., 0.], [0., 0., 12.], [0.]*4, 10., False, sample_time=stamp)
        self.assertAlmostEqual(imu.angles[0], .004)
        self.assertEqual(imu.time, 10.)
        self.assertEqual(imu.sample_time, 1.02)

    def test_repeated_or_reversed_source_stamps_do_not_refresh_freshness(self):
        imu = TiltEstimator()
        args = ([0.]*3, [0., 0., 9.81], [0., 0., 0., 1.])
        imu.update(*args, 10., sample_time=100.)
        for stamp in (100., 99.99):
            with self.assertRaisesRegex(ValueError, 'source timestamps must increase'):
                imu.update(*args, 10.02, sample_time=stamp)
            self.assertEqual(imu.time, 10.)
            self.assertEqual(imu.sample_time, 100.)
            self.assertEqual(len(imu.history), 1)
        imu.update(*args, 10.02, sample_time=100.01)
        self.assertEqual(imu.time, 10.02)

    def test_receipt_clock_reset_is_still_rejected(self):
        imu = TiltEstimator()
        args = ([0.]*3, [0., 0., 9.81], [0., 0., 0., 1.])
        imu.update(*args, 10., sample_time=100.)
        with self.assertRaisesRegex(ValueError, 'receipt clock moved backwards'):
            imu.update(*args, 9., sample_time=100.01)
        self.assertEqual(imu.time, 10.)

    def test_a_real_source_gap_invalidates_calibration(self):
        imu = TiltEstimator()
        args = ([0.]*3, [0., 0., 9.81], [0., 0., 0., 1.])
        imu.update(*args, 10., sample_time=100.)
        imu.calibrated = True
        imu.update(*args, 10., sample_time=100.3)
        self.assertFalse(imu.calibrated)
        self.assertEqual(len(imu.history), 1)

    def test_calibration_uses_receipt_clock_with_batched_source_samples(self):
        imu = TiltEstimator(source='raw')
        for i in range(121):
            imu.update([.005, -.004, 0.], [0., 0., 9.81], [0.]*4,
                       1.+(i//2)*.02, False, sample_time=100.+i*.01)
        np.testing.assert_allclose(imu.calibrate(2.2), [.005, -.004, 0.])
        self.assertTrue(imu.calibrated)

    def test_invalid_source_stamp_does_not_refresh_freshness(self):
        imu = TiltEstimator()
        with self.assertRaisesRegex(ValueError, 'invalid IMU sample'):
            imu.update([0.]*3, [0., 0., 9.81], [0., 0., 0., 1.], 10., sample_time=float('nan'))
        self.assertIsNone(imu.time)


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'optional ROS Python unavailable')
class ImuCallbackTests(unittest.TestCase):
    def setUp(self):
        from biped_gait.walking_node import QuasiStaticWalker
        self.callback = QuasiStaticWalker.imu
        self.ros_now = self.receipt_now = 10.
        self.controller = SimpleNamespace(imu=TiltEstimator())
        self.warnings = []
        self.node = SimpleNamespace(controller=self.controller, imu_frame='imu_link',
            now=lambda: self.receipt_now,
            age=lambda h: self.ros_now-h.stamp.sec-h.stamp.nanosec*1e-9,
            warn=self.warnings.append)

    def deliver(self, stamp):
        from sensor_msgs.msg import Imu
        msg = Imu()
        msg.header.frame_id = 'imu_link'
        ns = round(stamp*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
        msg.orientation.w = 1.
        msg.linear_acceleration.z = 9.81
        self.callback(self.node, msg)

    def test_two_fresh_messages_can_share_the_same_ros_clock_tick(self):
        self.deliver(9.98)
        self.deliver(9.99)
        self.assertEqual(self.controller.imu_problem, '')
        self.assertFalse(self.warnings)
        self.assertAlmostEqual(self.controller.imu.sample_time, 9.99)
        self.assertEqual(self.controller.imu.time, 10.)

    def test_stale_and_repeated_source_messages_are_still_rejected(self):
        self.deliver(9.99)
        self.receipt_now = self.ros_now = 10.01
        self.deliver(9.99)
        self.assertIn('source timestamps', self.controller.imu_problem)
        self.assertEqual(self.controller.imu.time, 10.)
        self.deliver(9.5)
        self.assertIn('source timestamp', self.controller.imu_problem)
        self.assertEqual(self.controller.imu.time, 10.)

    def test_hardware_receipt_time_stays_in_the_monotonic_domain(self):
        self.ros_now, self.receipt_now = 1700000000., 123.
        self.deliver(1699999999.99)
        self.ros_now += .01
        self.receipt_now += .02
        self.deliver(1700000000.)
        self.assertEqual(self.controller.imu_problem, '')
        self.assertAlmostEqual(self.controller.imu.time, 123.02)
        self.assertAlmostEqual(self.controller.imu.sample_time, 1700000000.)
