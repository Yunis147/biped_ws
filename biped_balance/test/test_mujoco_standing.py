"""Optional physics check of feedback direction; assumptions are not hardware measurements.

Automatically skipped without the MuJoCo and xacro Python modules. No simulator
package is modified, no ROS nodes run, and no hardware connection is opened.
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/p) for p in ('biped_balance', 'biped_gait', 'biped_mujoco')]


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('xacro'),
                     'optional MuJoCo/xacro physics dependencies are not installed')
class StandingPhysicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import xacro
        cls.package = ROOT/'robotv2_description'
        with tempfile.TemporaryDirectory(prefix='balance-urdf-') as folder:
            source = Path(folder)/'robot.xacro'
            source.write_text((cls.package/'urdf/robot.urdf.xacro').read_text().replace(
                '$(find robotv2_description)', str(cls.package)))
            cls.urdf = xacro.process_file(str(source), mappings={'use_sim': 'false'}).toxml()

    def run_case(self, balance):
        from biped_balance.controller import BalanceExperiment
        from biped_balance.kinematics import NAMES
        from biped_mujoco.sim_core import BipedSim
        # Match the bridge's default 500/1000 torque fraction, conditional on
        # the simulator's assumed voltage, stiffness, inertia and contact model.
        sim = BipedSim(self.urdf, self.package, {'torque_fraction': .5})
        self.addCleanup(sim._tmp.cleanup)
        controller = BalanceExperiment(actuate=True, axes_verified=True)
        started = pushed = False
        standing = None
        integral = peak = 0.
        for i in range(int(40/sim.model.opt.timestep)):
            now = sim.time
            if i % 10 == 0:
                quaternion, gyro, accel = sim.imu()
                controller.imu.update(gyro, accel, quaternion, now)
                controller.feedback.joints(NAMES, sim.joint_positions(), sim.joint_velocities(), now)
                controller.feedback.diagnostics('torque ON', 0, {'startup_positioning': 'False'}, now)
                if now >= 2 and not started:
                    controller.handle('calibrate', now)
                    controller.handle('stand', now)
                    started = True
                output = controller.tick(now)
                if output is not None:
                    sim.set_targets(output)
                self.assertNotEqual(controller.state, 'fault', controller.fault)
                if controller.state == 'standing' and standing is None:
                    standing = now
                    if balance:
                        controller.handle('balance_on', now)
                if standing is not None and now >= standing+3 and not pushed:
                    sim.push([3., 0, 0], .1)
                    pushed = True
            sim.step()
            self.assertFalse(sim.fallen())
            if pushed:
                angle = sim.tilt_deg()
                peak = max(peak, angle)
                integral += angle*angle*sim.model.opt.timestep
            if standing is not None and now >= standing+9:
                break
        self.assertIsNotNone(standing)
        self.assertTrue(pushed)
        return peak, integral

    def test_pd_reduces_pitch_disturbance_for_the_assumed_model(self):
        without = self.run_case(False)
        with_feedback = self.run_case(True)
        self.assertLess(with_feedback[0], without[0])
        self.assertLess(with_feedback[1], without[1])


if __name__ == '__main__':
    unittest.main()
