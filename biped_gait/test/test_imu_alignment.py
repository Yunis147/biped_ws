"""Known-upright startup alignment, fixed reference and active balance feedback."""
import math
import unittest
import numpy as np
from test_feedback import controller
from biped_gait.feedback_walk import NAMES
from biped_gait.imu import TiltEstimator,rpy_matrix,GRAVITY


def quaternion(angles):
    r,p=np.asarray(angles)/2
    return np.array([math.sin(r)*math.cos(p),math.cos(r)*math.sin(p),
        -math.sin(r)*math.sin(p),math.cos(r)*math.cos(p)])


def multiply(a,b):
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),a[3]*b[3]-np.dot(a[:3],b[:3])]


class AlignmentTests(unittest.TestCase):
    offset=np.radians([2.2,-1.9])
    bias=np.array([.006,-.004,.002])

    def reading(self,imu,now,body=(0.,0.),rate=(0.,0.,0.),sample_time=None,fused=False):
        mount=rpy_matrix(*self.offset)
        accel=mount.T@rpy_matrix(*body).T@np.array([0.,0.,GRAVITY])
        gyro=mount.T@np.array(rate)+self.bias
        # Deliberately wrong onboard orientation: hardware must use six-axis data.
        q=multiply(quaternion(np.radians([3.,-2.])),multiply(quaternion(body),quaternion(self.offset))) if fused else [.3,.2,.1,.92736]
        imu.update(gyro,accel,q,now,sample_time=sample_time)

    def stationary(self,imu,fused=False):
        for i in range(71):self.reading(imu,10.+i*.02,fused=fused)
        return 11.4

    def test_upright_alignment_rotates_all_measurements_and_estimates_bias(self):
        imu=TiltEstimator(source='raw');now=self.stationary(imu)
        source_time=imu.sample_time
        imu.align_upright(now)
        np.testing.assert_allclose(imu.mount,rpy_matrix(*self.offset),atol=1e-10)
        np.testing.assert_allclose(imu.angles,[0.,0.],atol=1e-10)
        np.testing.assert_allclose(imu.accel,[0.,0.,GRAVITY],atol=1e-10)
        np.testing.assert_allclose(imu.gyro,[0.,0.,0.],atol=1e-10)
        np.testing.assert_allclose(imu.bias,imu.mount@self.bias,atol=1e-10)
        self.assertEqual(imu.sample_time,source_time)
        self.assertTrue(imu.calibrated)

    def test_alignment_is_fixed_and_later_real_tilt_is_measured(self):
        imu=TiltEstimator(source='raw');now=self.stationary(imu);imu.align_upright(now)
        alignment=imu.mount.copy()
        for i in range(1,11):
            self.reading(imu,now+i*.02,body=(math.radians(.9*i),0.),rate=(math.radians(45),0.,0.))
        self.assertGreater(math.degrees(imu.angles[0]),8.9)
        np.testing.assert_array_equal(imu.mount,alignment)
        with self.assertRaisesRegex(ValueError,'already fixed'):imu.align_upright(now+.2)

    def test_later_gyro_calibration_does_not_rezero_the_body(self):
        imu=TiltEstimator(source='raw');now=self.stationary(imu);imu.align_upright(now)
        for i in range(1,101):self.reading(imu,now+i*.02,body=(math.radians(3),0.))
        before=imu.angles.copy();mount=imu.mount.copy()
        imu.calibrate(now+2.)
        np.testing.assert_array_equal(imu.angles,before)
        np.testing.assert_array_equal(imu.mount,mount)
        self.assertGreater(math.degrees(imu.angles[0]),2.9)

    def test_moving_gravity_is_not_accepted_as_upright_alignment(self):
        imu=TiltEstimator(source='raw')
        for i in range(71):self.reading(imu,10.+i*.02,body=(math.radians(i%2*2),0.))
        mount=imu.mount.copy()
        with self.assertRaisesRegex(ValueError,'gravity direction is changing'):imu.align_upright(11.4)
        np.testing.assert_array_equal(imu.mount,mount)
        self.assertIsNone(imu.upright_alignment)

    def test_configured_mount_is_not_applied_twice(self):
        imu=TiltEstimator((*self.offset,0.),source='raw');now=self.stationary(imu)
        before=imu.mount.copy();imu.align_upright(now)
        np.testing.assert_allclose(imu.mount,before,atol=1e-10)

    def test_acceleration_is_not_treated_as_gravity_during_motion(self):
        imu=TiltEstimator(source='raw')
        imu.update([0.,0.,0.],[0.,0.,GRAVITY],[0.]*4,1.,False)
        imu.gravity_feedback=False
        for i in range(1,101):
            imu.update([0.,0.,0.],[0.,1.,GRAVITY],[0.]*4,1.+i*.02,False)
        np.testing.assert_allclose(imu.angles,[0.,0.],atol=1e-12)

    def test_controller_enables_gravity_correction_only_after_quiet_dwell(self):
        c=controller(imu_source='raw');c.state='standing';c.commanded=c.nominal.copy()
        for i in range(13):
            now=10.+i*.02
            self.reading(c.imu,now)
            c.feedback.joints(NAMES,np.zeros(12),np.zeros(12),now)
            c.feedback.diagnostics('torque ON',0,{'startup_positioning':'false'},now)
            c.tick(now)
        self.assertTrue(c.imu.gravity_feedback)
        c.state='moving';c._begin('neutral','ready')
        c.tick(now)
        self.assertFalse(c.imu.gravity_tau==.1)

    def test_hardware_auto_aligns_reported_orientation_before_commands(self):
        c=controller(align_imu_at_startup=True)
        self.assertEqual(c.imu.source,'auto')
        now=self.stationary(c.imu,fused=True)
        c.feedback.joints(NAMES,np.zeros(12),np.zeros(12),now)
        c.feedback.diagnostics('torque ON',0,{'startup_positioning':'false'},now)
        c.command('stand',now);c.tick(now)
        self.assertEqual(c.state,'approach')
        self.assertIsNotNone(c.imu.upright_alignment)
        self.assertLess(np.max(np.abs(c.imu.angles)),1e-8)
        self.assertEqual(c.imu.used_source,'quaternion')

    def test_fixed_world_and_mount_alignment_preserve_later_measured_tilt(self):
        imu=TiltEstimator();now=self.stationary(imu,fused=True)
        imu.align_upright(now)
        mount=imu.mount.copy();reference=imu.orientation_reference.copy()
        self.reading(imu,now+.02,body=np.radians([5.,-3.]),fused=True)
        np.testing.assert_allclose(imu.angles,np.radians([5.,-3.]),atol=1e-10)
        np.testing.assert_array_equal(imu.mount,mount)
        np.testing.assert_array_equal(imu.orientation_reference,reference)

    def test_reported_orientation_still_faults_on_a_real_fall_after_alignment(self):
        c=controller(align_imu_at_startup=True);now=self.stationary(c.imu,fused=True)
        c.imu.align_upright(now)
        c.nominal=c.plan.tables[('neutral','ready')][-1].copy()
        c.commanded=c.nominal.copy();c.state='standing';c.last_time=now
        self.reading(c.imu,now+.02,body=np.radians([9.,0.]),fused=True)
        c.feedback.joints(NAMES,c.commanded,np.zeros(12),now+.02)
        c.feedback.diagnostics('torque ON',0,{'startup_positioning':'false'},now+.02)
        c.tick(now+.02)
        self.assertIn('measured torso tilt',c.fault)

    def test_invalid_starting_joints_cannot_calibrate_a_new_upright_reference(self):
        c=controller(align_imu_at_startup=True,imu_source='raw');now=self.stationary(c.imu)
        c.feedback.joints(NAMES,np.zeros(12),np.zeros(12),now)
        c.feedback.diagnostics('torque ON',0,{'startup_positioning':'false'},now)
        c.command('stand',now)
        q=np.zeros(12);q[2]=.06
        c.feedback.joints(NAMES,q,np.zeros(12),now)
        c.tick(now)
        self.assertIsNone(c.imu.upright_alignment)
        self.assertIn('starting pose moved',c.fault)

    def test_aligned_imu_drives_joint_corrections_in_bent_standing_pose(self):
        c=controller(align_imu_at_startup=True);now=self.stationary(c.imu)
        c.imu.align_upright(now);mount=c.imu.mount.copy()
        c.nominal=c.plan.tables[('neutral','ready')][-1].copy()
        c.commanded=c.nominal.copy();c.state='standing';c.last_time=now
        previous=c.commanded.copy()
        for i in range(1,31):
            time=now+i*.02
            self.reading(c.imu,time,body=(i*.04/30,0.),rate=(.04/.6,0.,0.))
            c.feedback.joints(NAMES,c.commanded,np.zeros(12),time)
            c.feedback.diagnostics('torque ON',0,{'startup_positioning':'false'},time)
            output=c.tick(time)
            self.assertFalse(c.fault,c.status(time))
            self.assertLessEqual(np.max(np.abs(output-previous)),c.s.max_joint_speed*.02+1e-7)
            previous=output.copy()
        self.assertLess(c.correction[0],0.)
        self.assertGreater(np.max(np.abs(c.commanded-c.nominal)),1e-4)
        np.testing.assert_array_equal(c.imu.mount,mount)


if __name__=='__main__':unittest.main()
