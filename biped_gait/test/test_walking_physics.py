"""Optional joint-only MuJoCo regressions; no hardware connections."""
import importlib.util
from pathlib import Path
import sys,tempfile,unittest
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

    def run_case(self, direction='forward', parameters=None, landings_required=6,
                 control_steps=(10,), sensor_delay=0., quantized=False, stand_only=False):
        from test_model_walk import controller
        from biped_gait.model_walk import NAMES
        from biped_gait import quasi_static_walk as m
        from biped_mujoco.sim_core import BipedSim
        sim = BipedSim(self.urdf, self.package, parameters or self.launch_parameters)
        self.addCleanup(sim._tmp.cleanup)
        c = controller(simulation=True)
        started = stopped = False
        finished = None
        peak = 0.
        start_x = float(sim.base_pose()[0][0])
        lift_heights = {'r':0.,'l':0.}
        history = []; sensor_index = 0; control_due = control_index = 0
        max_speed = 0.; self_contacts = set(); previous_control_time = None
        for i in range(int(420/.002)):
            if i%5 == 0:
                q = sim.joint_positions()
                if quantized: q = np.round(q/(2*np.pi/4096))*(2*np.pi/4096)
                history.append((sim.time,q,sim.joint_velocities()))
            if i == control_due:
                control_due += control_steps[control_index%len(control_steps)]; control_index += 1
                now = sim.time
                while sensor_index<len(history) and history[sensor_index][0]<=now-sensor_delay+1e-9:
                    stamp,q,v = history[sensor_index]
                    c.feedback.joints(NAMES,q,v,now,now-stamp)
                    sensor_index += 1
                if now>=2 and not started:
                    c.command('stand' if stand_only else direction,now); started=True
                if c.landings>=landings_required and not stopped:
                    c.command('stop',now); stopped=True
                before = None if c.commanded is None else c.commanded.copy()
                target = c.tick(now)
                if target is not None:
                    sim.set_targets(target)
                    if before is not None:
                        max_speed=max(max_speed,float(np.max(np.abs(target-before)))/(now-previous_control_time))
                previous_control_time = now
                if c.fault or c.state=='placement_hold': self.fail(str(c.status(now)))
                if c.segment and c.segment[1] in ('swing_right','swing_left'):
                    stance='l' if c.segment[1]=='swing_right' else 'r'
                    swing='r' if stance=='l' else 'l'
                    _,feet=m._com_and_feet(dict(zip(NAMES,sim.joint_positions())))
                    world=feet[swing] @ sim.data.xmat[sim.base_id].reshape(3,3).T + sim.base_pose()[0]
                    lift_heights[swing]=max(lift_heights[swing],float(world[:,2].min()))
                if c.state=='standing' and (stopped or stand_only) and finished is None: finished=now
            sim.step()
            self.assertFalse(sim.fallen(),str(c.status(sim.time)))
            peak=max(peak,sim.tilt_deg())
            # With the narrower stance, verify there are no inter-leg contacts.
            for contact in sim.data.contact:
                a,b=(int(sim.model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2))
                if a and b:
                    import mujoco
                    names=[mujoco.mj_id2name(sim.model,mujoco.mjtObj.mjOBJ_BODY,k) or '' for k in (a,b)]
                    if {n[:2] for n in names}=={'r_','l_'}: self_contacts.add(tuple(names))
            if finished is not None and sim.time>finished+3: break
        self.assertEqual(c.state,'standing',str(c.status(sim.time)))
        self.assertEqual(c.landings,0 if stand_only else landings_required)
        self.assertLess(peak,6.)
        self.assertFalse(self_contacts,self_contacts)
        self.assertLessEqual(max_speed,c.s.max_joint_speed+1e-5)
        if not stand_only:
            sign=1 if direction=='forward' else -1
            self.assertGreater(sign*(float(sim.base_pose()[0][0])-start_x),.015*landings_required)
            self.assertGreater(min(lift_heights.values()),.001)
        print('PHYSICS',direction,'kp',sim.p['kp'],'delay',sensor_delay,'quantized',quantized,
              'peak_deg',peak,'landings',c.landings,'displacement_m',float(sim.base_pose()[0][0])-start_x,
              'max_command_speed',max_speed,flush=True)

    def test_forward_six_landings_and_stop_without_inertial_input(self):
        self.run_case()

    def test_backward_six_landings_and_stop_without_inertial_input(self):
        self.run_case('backward')

    def test_weaker_motors_forward_twelve_landings(self):
        self.run_case(parameters={'kp':50.,'torque_fraction':.5},landings_required=12)

    def test_weaker_motors_backward_twelve_landings(self):
        self.run_case('backward',parameters={'kp':50.,'torque_fraction':.5},landings_required=12)

    def test_delayed_quantized_encoders_with_uneven_control(self):
        self.run_case(control_steps=(10,10,20,10),sensor_delay=.02,quantized=True)

    def test_stand_with_encoders_only(self):
        self.run_case(stand_only=True)
