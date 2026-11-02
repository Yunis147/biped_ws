"""Optional closed-loop MuJoCo regressions; no hardware connections."""
import importlib.util
from pathlib import Path
import sys,tempfile,unittest,math
import xml.etree.ElementTree as E
from dataclasses import replace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/p) for p in ('biped_gait', 'biped_mujoco')]


@unittest.skipUnless(importlib.util.find_spec('mujoco') and importlib.util.find_spec('xacro'), 'optional MuJoCo/xacro unavailable')
class WalkingPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import xacro
        cls.package = ROOT/'robotv2_description'
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'robot.xacro'
            p.write_text((cls.package/'urdf/robot.urdf.xacro').read_text().replace('$(find robotv2_description)', str(cls.package)))
            cls.urdf = xacro.process_file(str(p), mappings={'use_sim': 'false'}).toxml()

    def run_case(self, direction='forward', push=False, gain=True, obstacle=False):
        import mujoco
        from test_feedback import controller
        from biped_gait.feedback_walk import NAMES
        from biped_mujoco.sim_core import BipedSim
        sim = BipedSim(self.urdf, self.package, {'kp':50., 'torque_fraction':.5})
        self.addCleanup(sim._tmp.cleanup)
        if obstacle:
            # Introduce a 4 mm patch under the actual right toe just before
            # descent. Positioning after swing avoids confusing contact safety
            # with accumulated open-loop translation drift.
            root = E.parse(sim.mjcf_path).getroot()
            E.SubElement(root.find('worldbody'), 'geom', dict(name='early_ground', type='box',
                pos='10 10 .002', size='.005 .015 .002', friction='1 .005 .0001'))
            sim.model = mujoco.MjModel.from_xml_string(E.tostring(root, encoding='unicode'))
            for v in sim._vadr:
                sim.model.dof_armature[v] = sim.p['armature']
                sim.model.dof_damping[v] = sim.p['damping']
                sim.model.dof_frictionloss[v] = sim.p['frictionloss']
            sim.model.geom_solref[:, 0] = sim.p['contact_timeconst']
            sim.model.geom_friction[:, 0] = sim.p['friction']
            sim.data = mujoco.MjData(sim.model)
            sim.reset()
        c = controller(simulation=True)
        if not gain:
            c.s = replace(c.s, kp=1e-12, kd=1e-12)
        started = stopped = pushed = patch_placed = False
        finished = standing = None
        peak = integral = 0.
        for i in range(int(260/.002)):
            if i%10 == 0:
                now = sim.time
                quat, gyro, accel = sim.imu()
                c.imu.update(gyro, accel, quat, now)
                c.feedback.joints(NAMES, sim.joint_positions(), sim.joint_velocities(), now)
                if now>=2 and not started:
                    c.command('stand' if push else direction, now)
                    started = True
                if not push and not obstacle and c.landings>=3 and not stopped:
                    c.command('stop', now)
                    stopped = True
                if obstacle and not patch_placed and c.segment == ('swing_right', 'place_right'):
                    from biped_gait import quasi_static_walk as m
                    _, feet = m._com_and_feet(dict(zip(NAMES, sim.joint_positions())))
                    R = sim.data.xmat[sim.base_id].reshape(3, 3)
                    sole = feet['r'] @ R.T + sim.base_pose()[0]
                    gid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, 'early_ground')
                    sim.model.geom_pos[gid] = [sole[:, 0].max()-.008, sole[:, 1].mean(), .002]
                    mujoco.mj_forward(sim.model, sim.data)
                    patch_placed = True
                target = c.tick(now)
                if target is not None:
                    sim.set_targets(target)
                if c.fault:
                    if obstacle:
                        self.assertIn('landing interrupted', c.fault)
                        held = c.commanded.copy()
                        for j in range(100):
                            sim.step()
                            if j%10 == 0:
                                q,g,a = sim.imu()
                                c.imu.update(g,a,q,sim.time)
                                c.feedback.joints(NAMES,sim.joint_positions(),sim.joint_velocities(),sim.time)
                                np.testing.assert_array_equal(c.tick(sim.time), held)
                        return dict(fault=c.fault, peak=peak, landings=c.landings)
                    self.fail(str(c.status(now)))
                if c.state == 'standing':
                    if standing is None:
                        standing = now
                    if stopped and finished is None:
                        finished = now
                if push and standing is not None and not pushed and now>standing+2:
                    sim.push([3.,0.,0.],.1)
                    pushed = True
            sim.step()
            self.assertFalse(sim.fallen())
            peak = max(peak, sim.tilt_deg())
            if pushed:
                integral += sim.tilt_deg()**2*.002
            if push and standing is not None and sim.time>standing+8:
                break
            if finished is not None and sim.time>finished+3:
                break
        if obstacle:
            self.fail('raised-ground descent was not interrupted')
        if not push:
            self.assertEqual(c.landings,3)
            self.assertEqual(c.state,'standing')
            self.assertLess(peak, 6.5)
        print('PHYSICS',direction,'push',push,'feedback',gain,'peak',peak,'integral',integral,'landings',c.landings,flush=True)
        return dict(peak=peak, integral=integral)

    def test_forward_with_imu_three_landings_and_stop(self):
        self.run_case('forward')

    def test_backward_with_imu_three_landings_and_stop(self):
        self.run_case('backward')

    def test_feedback_reduces_standing_pitch_disturbance(self):
        without = self.run_case(push=True, gain=False)
        with_feedback = self.run_case(push=True)
        self.assertLess(with_feedback['integral'], without['integral'])

    def test_early_ground_stops_descent_instead_of_completing_the_step(self):
        result = self.run_case(obstacle=True)
        self.assertEqual(result['landings'],0)
