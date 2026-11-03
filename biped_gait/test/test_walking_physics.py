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
        import yaml
        from biped_mujoco.sim_core import DEFAULTS
        cls.package = ROOT/'robotv2_description'
        parameters=yaml.safe_load((ROOT/'biped_mujoco/config/mujoco.yaml').read_text())['mujoco_biped']['ros__parameters']
        cls.launch_parameters={k:parameters[k] for k in DEFAULTS if k in parameters}
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'robot.xacro'
            p.write_text((cls.package/'urdf/robot.urdf.xacro').read_text().replace('$(find robotv2_description)', str(cls.package)))
            cls.urdf = xacro.process_file(str(p), mappings={'use_sim': 'false'}).toxml()

    def run_case(self, direction='forward', push=False, gain=True, obstacle=False, transfer_push=False,
                 parameters=None, landings_required=3):
        import mujoco
        from test_feedback import controller
        from biped_gait.feedback_walk import NAMES
        from biped_mujoco.sim_core import BipedSim
        sim = BipedSim(self.urdf, self.package, parameters or {'kp':50., 'torque_fraction':.5})
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
        start_x = float(sim.base_pose()[0][0])
        lift_heights = {'r':0.,'l':0.}
        for i in range(int(260/.002)):
            if i%10 == 0:
                now = sim.time
                quat, gyro, accel = sim.imu()
                c.imu.update(gyro, accel, quat, now)
                c.feedback.joints(NAMES, sim.joint_positions(), sim.joint_velocities(), now)
                if now>=2 and not started:
                    c.command('stand' if push else direction, now)
                    started = True
                if transfer_push and not pushed and c.segment == ('ready','lean_left') and c.elapsed > c.plan.durations[c.segment]*.4:
                    sim.push([0.,3.,0.],.1)
                    pushed = True
                if transfer_push and c.backoffs and c.state == 'standing':
                    stopped = True
                if not push and not obstacle and not transfer_push and c.landings>=landings_required and not stopped:
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
                if c.segment and c.segment[1] in ('swing_right','swing_left'):
                    from biped_gait.placement import ground_estimate
                    stance = 'l' if c.segment[1] == 'swing_right' else 'r'
                    g = ground_estimate(sim.joint_positions(),c.imu.angles,stance)
                    lift_heights[g.swing] = max(lift_heights[g.swing],g.clearance)
                if target is not None:
                    sim.set_targets(target)
                if c.fault or c.state == 'placement_hold':
                    if obstacle:
                        self.assertEqual(c.state,'placement_hold',c.fault)
                        self.assertIn('landing interrupted', c.interruption)
                        held = c.commanded.copy()
                        for j in range(100):
                            sim.step()
                            if j%10 == 0:
                                q,g,a = sim.imu()
                                c.imu.update(g,a,q,sim.time)
                                c.feedback.joints(NAMES,sim.joint_positions(),sim.joint_velocities(),sim.time)
                                next_target=c.tick(sim.time)
                                self.assertIsNotNone(next_target)
                                self.assertFalse(c.fault,c.fault)
                                self.assertLessEqual(np.max(np.abs(next_target-held)),c.s.max_joint_speed*.02+1e-7)
                                held=next_target.copy()
                        return dict(interruption=c.interruption, peak=peak, landings=c.landings)
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
        if transfer_push:
            self.assertGreaterEqual(c.backoffs,1)
            self.assertEqual(c.landings,0)
            self.assertEqual(c.state,'standing')
        elif not push:
            self.assertEqual(c.landings,landings_required)
            self.assertEqual(c.state,'standing')
            self.assertLess(peak, 6.5)
            sign = 1 if direction == 'forward' else -1
            self.assertGreater(sign*(float(sim.base_pose()[0][0])-start_x),.015)
            self.assertGreater(min(lift_heights.values()),.001)
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

    def test_lateral_disturbance_during_transfer_returns_to_standing(self):
        self.run_case(transfer_push=True)

    def test_default_mujoco_motors_forward_six_landings(self):
        self.run_case(parameters=self.launch_parameters,landings_required=6)

    def test_default_mujoco_motors_backward_six_landings(self):
        self.run_case('backward',parameters=self.launch_parameters,landings_required=6)
