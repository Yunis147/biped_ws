"""Hardware-startup regressions for the two-foot crouch before weight transfer."""
import json
import math
import unittest
import numpy as np
from test_feedback import controller,feed
from biped_gait.feedback_walk import NAMES,reference_rotation,m
from biped_gait.imu import tilt


class PreparationTests(unittest.TestCase):
    def preparing(self):
        c=controller()
        c.commanded=c.nominal.copy()
        feed(c,1.)
        c._begin('neutral','ready')
        c.elapsed=.2;c.last_time=1.
        c.nominal=m.table_pose(c.plan.tables[c.segment],c.elapsed/c.plan.durations[c.segment])
        c.commanded=c.nominal.copy()
        return c

    def test_logged_small_tilt_and_rate_allow_initial_crouch(self):
        c=self.preparing();before=c.elapsed
        feed(c,1.02,angles=np.radians([.77,-.88]),gyro=(0.,-math.radians(1.5),0.))
        predicted=c.imu.angles+c.s.transfer_lookahead*c.imu.gyro[:2]
        self.assertGreater(np.max(np.abs(predicted)),c.s.transfer_error)
        output=c.tick(1.02)
        self.assertEqual(c.state,'moving',c.status(1.02))
        self.assertEqual(c.segment,('neutral','ready'))
        self.assertGreater(c.elapsed,before)
        self.assertEqual(c.backoffs,0)
        self.assertFalse(c.preparation_paused)
        self.assertLess(max(-output[NAMES.index(s+'_knee_joint')] for s in 'rl'),.2)
        np.testing.assert_allclose(c.imu.angles,np.radians([.77,-.88]))

    def test_soft_tilt_limit_does_not_freeze_initial_straight_knees(self):
        c=self.preparing();before=c.elapsed;previous=c.commanded.copy()
        feed(c,1.02,angles=np.radians([1.6,0.]))
        output=c.tick(1.02)
        self.assertFalse(c.preparation_paused)
        self.assertGreater(c.elapsed,before)
        self.assertEqual(c.backoffs,0)
        self.assertLessEqual(np.max(np.abs(output-previous)),c.s.max_joint_speed*.02+1e-7)
        feed(c,1.04,angles=np.radians([.77,-.88]))
        c.tick(1.04)
        self.assertFalse(c.preparation_paused)
        self.assertGreater(c.elapsed,before)

    def test_rate_reading_does_not_latch_the_old_preparation_timeout(self):
        c=self.preparing();before=c.elapsed
        feed(c,1.02,gyro=(.11,0.,0.))
        c.tick(1.02)
        self.assertFalse(c.preparation_paused)
        self.assertGreater(c.elapsed,before)
        self.assertFalse(c.fault)
        self.assertEqual(c.backoffs,0)

    def test_preparation_status_is_json_serializable_while_moving_or_paused(self):
        c=self.preparing()
        for now,angle,paused in ((1.02,.01,False),(1.04,.03,False)):
            feed(c,now,angles=[angle,0.])
            c.tick(now)
            status=json.loads(json.dumps(c.status(now),allow_nan=False))
            self.assertIs(status['preparation_paused'],paused)

    def test_unsettled_crouch_blocks_lift_after_bending_instead_of_freezing_startup(self):
        c=self.preparing();before=c.elapsed
        for i in range(1,1300):
            now=1.+i*.02
            feed(c,now,angles=np.radians([1.6,0.]))
            c.tick(now)
            if c.fault:break
        self.assertIn('body/servos did not settle',c.fault)
        self.assertNotIn('initial crouch did not stabilize',c.fault)
        self.assertGreater(c.elapsed,before)
        self.assertEqual(c.landings,0)
        self.assertEqual(c.segment,('neutral','ready'))

    def test_hard_tilt_fault_still_holds_during_initial_crouch(self):
        c=self.preparing();before=c.commanded.copy()
        feed(c,1.02,angles=np.radians([8.5,0.]))
        np.testing.assert_array_equal(c.tick(1.02),before)
        self.assertIn('measured torso tilt',c.fault)

    def test_sideways_transfer_still_checks_the_logged_tilt_prediction(self):
        c=self.preparing()
        c.nominal=m.table_pose(c.plan.tables[('neutral','ready')],1.)
        c.commanded=c.nominal.copy()
        c._begin('ready','lean_left')
        c.elapsed=.2
        c.nominal=m.table_pose(c.plan.tables[c.segment],c.elapsed/c.plan.durations[c.segment])
        c.commanded=c.nominal.copy()
        feed(c,1.02,angles=tilt(reference_rotation(c.nominal))+np.radians([.77,-.88]),
             gyro=(0.,-math.radians(1.5),0.))
        c.tick(1.02)
        self.assertEqual(c.state,'recovering')
        self.assertEqual(c.backoffs,1)

    def test_stand_finishes_crouch_without_erasing_initial_tilt(self):
        c=controller();angles=np.radians([.77,-.88])
        feed(c,1.,angles=angles)
        c.command('stand',1.)
        for i in range(1200):
            now=1.+i*.02
            feed(c,now,angles=angles,gyro=(0.,-math.radians(1.5),0.))
            c.tick(now)
            self.assertFalse(c.fault,c.status(now))
            self.assertEqual(c.backoffs,0)
            if c.state=='standing':break
        self.assertEqual(c.state,'standing',c.status(now))
        self.assertEqual(c.landings,0)
        self.assertGreater(min(-c.nominal[NAMES.index(s+'_knee_joint')] for s in 'rl'),.2)
        np.testing.assert_allclose(c.imu.angles,angles)
        self.assertNotIn('IMU is moving',c.reason)


if __name__=='__main__':unittest.main()
