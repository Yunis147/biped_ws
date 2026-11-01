"""Optional physics tests. Measurements validate the gait; none feed its control."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / package) for package in ('biped_cpg', 'biped_gait', 'biped_mujoco')]

REPORTS = []


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('xacro'),
                     'optional MuJoCo/xacro dependencies are unavailable')
class PhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import xacro
        from test_cpg import fresh_library
        cls.make_library = staticmethod(fresh_library)
        cls.package = ROOT / 'robotv2_description'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.xacro'
            path.write_text((cls.package / 'urdf/robot.urdf.xacro').read_text().replace(
                '$(find robotv2_description)', str(cls.package)))
            cls.urdf = xacro.process_file(str(path), mappings={'use_sim': 'false'}).toxml()

    def run_walking(self, direction, stiffness):
        import mujoco
        from biped_cpg.controller import Walker
        from biped_cpg.planner import model, NAMES
        from biped_mujoco.sim_core import BipedSim
        sim = BipedSim(self.urdf, self.package, {'kp': stiffness, 'torque_fraction': .5})
        self.addCleanup(sim._tmp.cleanup)
        walker = Walker(self.make_library())
        started = stopped = False
        finished = None
        peak_roll = peak_pitch = peak_yaw = peak_torque = peak_tracking = 0.
        min_apex = math.inf
        x0 = None
        for tick in range(int(240/sim.model.opt.timestep)):
            if tick % 10 == 0:
                if sim.time >= 2 and not started:
                    walker.command(direction, sim.time)
                    started, x0 = True, sim.base_pose()[0][0]
                if walker.landings >= 8 and not stopped:
                    walker.command('stop', sim.time)
                    stopped = True
                output = walker.tick(sim.time)
                if output is not None:
                    sim.set_targets(output)
                self.assertNotEqual(walker.state, 'fault', walker.fault)
                if stopped and walker.state == 'idle' and finished is None:
                    finished = sim.time
                if walker.state == 'walking':
                    phase = walker.cpg.right
                    u = (phase % math.pi) / math.pi
                    swing_progress = (u-walker.plan.settings.double_support)/(1-walker.plan.settings.double_support)
                    if abs(swing_progress-.5) < .01:
                        swing = 'r' if int(phase/math.pi) % 2 == 0 else 'l'
                        q = dict(zip(NAMES, sim.joint_positions()))
                        _, feet = model._com_and_feet(q)
                        R = sim.data.xmat[sim.base_id].reshape(3, 3)
                        lowest = (feet[swing] @ R.T + sim.base_pose()[0])[:, 2].min()
                        min_apex = min(min_apex, float(lowest))
            sim.step()
            R = sim.data.xmat[sim.base_id].reshape(3, 3)
            peak_roll = max(peak_roll, abs(math.degrees(math.atan2(R[2, 1], R[2, 2]))))
            peak_pitch = max(peak_pitch, abs(math.degrees(math.asin(np.clip(-R[2, 0], -1, 1)))))
            peak_yaw = max(peak_yaw, abs(math.degrees(math.atan2(R[1, 0], R[0, 0]))))
            peak_torque = max(peak_torque, float(np.max(np.abs(sim.joint_efforts()))))
            peak_tracking = max(peak_tracking, float(np.max(np.abs(sim.joint_positions()-sim.targets))))
            self.assertFalse(sim.fallen(), (direction, stiffness, sim.time))
            self.assertLess(sim.tilt_deg(), 7.)
            if finished is not None and sim.time >= finished+3:
                break
        self.assertIsNotNone(finished, (direction, stiffness, walker.status()))
        self.assertGreaterEqual(walker.landings, 8)
        distance = sim.base_pose()[0][0] - x0
        self.assertGreater(distance*(1 if direction == 'forward' else -1), .08)
        self.assertLess(sim.tilt_deg(), 2.)
        self.assertLess(peak_yaw, 8.)
        self.assertGreater(min_apex, .004)
        self.assertLess(peak_tracking, .03)
        REPORTS.append(dict(direction=direction, stiffness_nm_per_rad=stiffness, torque_fraction=.5,
                            landings=walker.landings, distance_m=float(distance), max_roll_deg=peak_roll,
                            max_pitch_deg=peak_pitch, max_yaw_deg=peak_yaw, peak_effort_nm=peak_torque,
                            max_tracking_error_rad=peak_tracking, minimum_swing_apex_m=min_apex,
                            stopped=True, simulated_seconds=sim.time, period_seconds=walker.plan.period,
                            mujoco_version=mujoco.__version__))
        print('PHYSICS ' + json.dumps(REPORTS[-1]), flush=True)

    def test_forward_nine_landings_then_stop(self):
        for stiffness in (50., 100.):
            with self.subTest(stiffness=stiffness):
                self.run_walking('forward', stiffness)

    def test_backward_nine_landings_then_stop(self):
        for stiffness in (50., 100.):
            with self.subTest(stiffness=stiffness):
                self.run_walking('backward', stiffness)
