"""Optional MuJoCo regression, using the existing simulator without modifying it.

No feedback is fed into the gait. Servo stiffness, contact and torque conversion
are simulator assumptions, not measurements of the physical ST3215 motors.
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / name) for name in ('biped_gait', 'biped_mujoco')]


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('xacro'),
                     'optional MuJoCo/xacro dependencies are not installed')
class WalkingPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import xacro
        from biped_gait import quasi_static_walk
        cls.gait = quasi_static_walk
        cls.package = ROOT / 'robotv2_description'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'robot.xacro'
            path.write_text((cls.package / 'urdf/robot.urdf.xacro').read_text().replace(
                '$(find robotv2_description)', str(cls.package)))
            cls.urdf = xacro.process_file(str(path), mappings={'use_sim': 'false'}).toxml()

    def walk_and_stop(self, direction):
        from biped_mujoco.sim_core import BipedSim
        gait = self.gait
        _, tables = gait.build_tables(direction)
        # Bridge default = 500/1000 torque. Also lower the assumed servo stiffness
        # from the simulator's 100 to 50 N.m/rad to exercise position error.
        sim = BipedSim(self.urdf, self.package, {'torque_fraction': .5, 'kp': 50.})
        self.addCleanup(sim._tmp.cleanup)
        peak_tilt = 0.

        def run(pose, seconds):
            nonlocal peak_tilt
            start, ticks = sim.time, 0
            while sim.time - start < seconds:
                if ticks % 10 == 0:  # same 50 Hz target rate as the hardware bridge
                    sim.set_targets(pose((sim.time - start) / seconds))
                sim.step()
                ticks += 1
                peak_tilt = max(peak_tilt, sim.tilt_deg())
                self.assertFalse(sim.fallen(), (direction, sim.time))

        self.assertAlmostEqual(sim.total_mass(), gait.TOTAL_MASS, places=8)
        run(lambda u: np.zeros(12), 2.)
        first_x = sim.base_pose()[0][0]
        a, b, landings = 'neutral', 'ready', 0
        while landings < 6:
            table = tables[(a, b)]
            run(lambda u: gait.table_pose(table, u), gait.segment_duration(b, table))
            run(lambda u: table[-1], gait.DWELL_TIME.get(b, 0.) * gait.TIME_SCALE)
            if b.startswith('place_'):
                landings += 1
            a, b = b, gait.NEXT[b]
        stop = tables[(a, 'neutral')]
        run(lambda u: gait.table_pose(stop, u), gait.segment_duration('neutral', stop))
        run(lambda u: stop[-1], 3.)
        distance = sim.base_pose()[0][0] - first_x
        self.assertGreater(distance * (1 if direction == 'forward' else -1), .05)
        self.assertLess(peak_tilt, 16.)  # includes the intentionally planned 11.46° lean
        self.assertLess(sim.tilt_deg(), 3.)  # centred and upright after stopping

    def test_forward_six_landings_and_stop(self):
        self.walk_and_stop('forward')

    def test_backward_six_landings_and_stop(self):
        self.walk_and_stop('backward')
