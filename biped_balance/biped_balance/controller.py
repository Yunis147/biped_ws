"""Standing and double-support experiments. No ROS, torque writes, or foot-contact claims."""
from dataclasses import dataclass
import math

import numpy as np

from .estimation import Feedback, TiltEstimator
from .kinematics import INDEX, LOWER, NAMES, UPPER, attitude_pose, posture_table, standing_poses


@dataclass
class Settings:
    kp_roll: float = .30
    kp_pitch: float = .30
    kd_roll: float = .04
    kd_pitch: float = .04
    max_correction: float = math.radians(1.5)
    correction_rate: float = .05
    max_joint_speed: float = .25
    nominal_joint_speed: float = .12
    tracking_tolerance: float = .035
    fault_tracking_error: float = .15
    fault_tilt: float = math.radians(10)
    transfer_distance: float = .015
    transfer_duration: float = 6.

    def validate(self):
        values = vars(self)
        if not all(math.isfinite(v) and v >= 0 for v in values.values()):
            raise ValueError('controller settings must be finite and nonnegative')
        for name in ('max_joint_speed', 'nominal_joint_speed', 'correction_rate',
                     'tracking_tolerance', 'fault_tracking_error', 'fault_tilt', 'transfer_duration'):
            if values[name] <= 0:
                raise ValueError(name + ' must be positive')
        if self.nominal_joint_speed > self.max_joint_speed:
            raise ValueError('nominal_joint_speed must not exceed max_joint_speed')
        if self.fault_tracking_error <= self.tracking_tolerance:
            raise ValueError('fault tracking threshold must exceed normal tracking tolerance')
        if self.max_correction > math.radians(5) or self.fault_tilt > math.radians(20):
            raise ValueError('this standing experiment supports at most 5 deg correction / 20 deg fault tilt')


class BalanceExperiment:
    def __init__(self, *, actuate=False, axes_verified=False, mount_rpy=(0., 0., 0.),
                 imu_source='auto', filter_tau=.5, settings=None):
        self.settings = settings or Settings()
        self.settings.validate()
        self.actuate, self.axes_verified = actuate, axes_verified
        self.imu = TiltEstimator(mount_rpy, imu_source, filter_tau)
        self.feedback = Feedback(NAMES)
        self.poses, self.tables = standing_poses(self.settings.transfer_distance)
        self.state, self.location = 'monitoring', None
        self.balance_enabled = False
        self.command = self.nominal = None
        self.preview = self.poses['center'].copy()
        self.correction = np.zeros(2)
        self.requested = np.zeros(2)
        self.segment = None
        self.table = None
        self.elapsed, self.duration = 0., 1.
        self.last_time = None
        self.settled = 0.
        self.reason = 'monitoring only; no actuator commands'
        self.fault = ''

    def readiness(self, now, other_publishers=0, command_subscribers=1):
        if other_publishers:
            return 'another node publishes joint commands; stop the original walker before actuating'
        if command_subscribers < 1:
            return 'no subscriber on /position_controller/commands'
        return self.feedback.problem(now, self.imu)

    def handle(self, command, now, other_publishers=0, command_subscribers=1):
        command = command.strip().lower()
        if command == 'status':
            return 'status requested'
        if command == 'calibrate':
            if self.state != 'monitoring':
                raise ValueError('return to monitoring before recalibrating')
            problem = self.feedback.problem(now, self.imu, require_bridge=False)
            if problem:
                raise ValueError(problem)
            if np.max(np.abs(self.feedback.velocity)) > .03:
                raise ValueError('joints are moving; hold the supported robot still')
            bias = self.imu.calibrate(now)
            return f'gyro bias calibrated: {bias.tolist()} rad/s; tilt reference remains gravity upright'
        if command in ('monitor', 'hold'):
            self.balance_enabled = False
            if command == 'monitor':
                self.state, self.command, self.nominal = 'monitoring', None, None
                self.segment, self.location = None, None
                self.table = None
                self.elapsed, self.duration = 0., 1.
                self.correction[:] = 0
                self.fault = ''
                return 'monitoring only; publishing stopped (servo bridge holds its last output)'
            if self.command is not None:
                self.state = 'holding'
            return 'holding the last command; no automatic return to zero or torque change'
        if command == 'balance_off':
            # Disabling feedback must remain available even when a sensor or
            # posture check would prohibit starting a new experiment.
            self.balance_enabled = False
            return 'balance OFF; correction ramps to zero during normal operation, faults keep their hold'
        if command not in ('stand', 'balance_on', 'balance_off', 'lean_left', 'lean_right', 'center'):
            raise ValueError('use status, calibrate, stand, balance_on, balance_off, lean_left, lean_right, center, hold, monitor')
        if not self.actuate:
            raise ValueError('monitor-only launch: restart with actuate:=true for actuator experiments')
        if not self.axes_verified:
            raise ValueError('verify IMU X forward / Y left / Z up, then set imu_axes_verified:=true')
        problem = self.readiness(now, other_publishers, command_subscribers)
        if problem:
            raise ValueError(problem)
        if not self.imu.calibrated:
            raise ValueError('calibrate gyro bias while the supported robot is upright and still')
        if self.fault:
            raise ValueError('latched fault: ' + self.fault + '; support the robot and use monitor before restarting')
        if np.max(np.abs(self.imu.angles)) > math.radians(5):
            raise ValueError('hold the torso within 5 deg of upright before starting an experiment')
        if command == 'stand':
            if self.state not in ('monitoring', 'holding'):
                raise ValueError('already active; use hold or monitor before preparing again')
            q = self.feedback.q.copy()
            if np.any(q < LOWER) or np.any(q > UPPER):
                raise ValueError('measured joints outside model limits; verify servo calibration')
            center = self.poses['center']
            if np.max(np.abs(q-center)) < .08:
                goal, target = 'center', center
            elif np.max(np.abs(q)) < .08:
                goal, target = 'zero', np.zeros(12)
            else:
                raise ValueError('start near calibrated zero or the bent-knee center pose; no arbitrary-pose homing')
            self.balance_enabled = False
            self.correction[:] = 0
            self.command = self.nominal = q.copy()
            self._begin(('approach', goal), posture_table(q, target), 'preparing', 2.)
            self.last_time = now
            return 'approaching known stance slowly, then holding a bent-knee standing pose; balance initially OFF'
        if command in ('balance_on', 'balance_off'):
            if self.state != 'standing':
                raise ValueError('finish stand and wait for standing before changing balance mode')
            self.balance_enabled = command == 'balance_on'
            return 'balance ' + ('ON (bounded task-space PD)' if self.balance_enabled else 'OFF (correction ramps to zero)')
        if self.state != 'standing' or not self.balance_enabled:
            raise ValueError('weight transfer requires standing with balance ON')
        target = {'lean_left': 'left', 'lean_right': 'right', 'center': 'center'}[command]
        if target == self.location:
            return 'already at ' + target
        self._begin((self.location, target), self.tables[(self.location, target)],
                    'transferring', self.settings.transfer_duration)
        return f'planted-foot weight transfer {self.location} -> {target}; no foot lift'

    def _begin(self, segment, table, state, seconds):
        self.segment, self.table, self.state = segment, table, state
        slope = float(np.max(np.abs(np.diff(table, axis=0)))) * (len(table)-1)
        self.duration = max(seconds, slope/self.settings.nominal_joint_speed)
        self.elapsed = self.settled = 0.

    def _nominal_at(self, elapsed):
        if self.table is None or self.state == 'standing':
            return self.nominal
        coordinate = min(elapsed/self.duration, 1.) * (len(self.table)-1)
        index = min(int(coordinate), len(self.table)-2)
        fraction = coordinate-index
        return (1-fraction)*self.table[index] + fraction*self.table[index+1]

    def _pd(self, nominal):
        if self.imu.angles is None:
            return np.zeros(2)
        s = self.settings
        value = -np.array([s.kp_roll, s.kp_pitch])*self.imu.angles
        value -= np.array([s.kd_roll, s.kd_pitch])*self.imu.gyro[:2]
        value = np.clip(value, -s.max_correction, s.max_correction)
        # A straight leg cannot realize arbitrary torso attitude with fixed feet.
        bend = min(abs(nominal[INDEX[side+'_knee_joint']]) for side in 'rl')
        authority = float(np.clip((bend-.25)/.30, 0., 1.))
        phase_gain = .65 if self.state == 'transferring' else 1.
        return value*authority*phase_gain

    def _finish_segment(self):
        a, b = self.segment
        if a == 'approach' and b == 'zero':
            self._begin(('zero', 'center'), self.tables[('zero', 'center')], 'preparing', 5.)
            return
        self.state, self.location = 'standing', b
        self.table, self.segment = None, None
        self.nominal = self.poses[b].copy()
        self.reason = 'standing; balance ' + ('ON' if self.balance_enabled else 'OFF')

    def tick(self, now, other_publishers=0, command_subscribers=1):
        dt = .02 if self.last_time is None else now-self.last_time
        self.last_time = now
        if dt < 0:
            self.fault, self.state = 'control clock moved backwards', 'fault'
            self.imu.calibrated = False
        dt = min(max(dt, 0.), .04)
        problem = self.feedback.problem(now, self.imu, require_bridge=self.actuate)
        if self.state == 'monitoring':
            self.reason = problem or 'monitoring only; no actuator commands'
            if self.imu.angles is not None:
                self.requested = self._pd(self.poses['center'])
                try:
                    self.preview = attitude_pose(self.poses['center'], self.requested)
                except ValueError:
                    self.preview = self.poses['center'].copy()
            return None
        problem = self.readiness(now, other_publishers, command_subscribers)
        if problem:
            self.fault, self.state = problem, 'fault'
        elif not self.imu.calibrated:
            self.fault, self.state = 'IMU needs recalibration after a data gap', 'fault'
        elif np.max(np.abs(self.imu.angles)) > self.settings.fault_tilt:
            self.fault, self.state = 'torso tilt exceeds experiment limit; support the robot', 'fault'
        error = (0. if self.command is None or self.feedback.q is None else
                 float(np.max(np.abs(self.command-self.feedback.q))))
        if error > self.settings.fault_tracking_error:
            self.fault, self.state = 'large encoder tracking error; check calibration/load', 'fault'
        if self.state in ('fault', 'holding'):
            self.reason = self.fault or 'holding the last command'
            # Avoid fighting another writer or bridge startup; otherwise keep the
            # exact last command alive. Holding is not a fall-recovery controller.
            return (None if other_publishers or self.feedback.bridge_problem or not self.actuate
                    else self.command)
        held = error > self.settings.tracking_tolerance
        advance = 0. if held or self.state == 'standing' else dt
        desired = self._pd(self.nominal) if self.balance_enabled else np.zeros(2)
        self.requested = desired
        correction = self.correction + np.clip(desired-self.correction,
                                               -self.settings.correction_rate*dt,
                                               self.settings.correction_rate*dt)
        accepted = False
        for fraction in (1., .5, .25, .125, .0625):
            elapsed = min(self.elapsed+advance*fraction, self.duration)
            nominal = self._nominal_at(elapsed)
            angles = self.correction+fraction*(correction-self.correction)
            try:
                candidate = attitude_pose(nominal, angles)
            except ValueError:
                continue
            if np.max(np.abs(candidate-self.command)) > self.settings.max_joint_speed*dt+1e-9:
                continue
            self.elapsed, self.nominal, self.correction, self.command = elapsed, nominal, angles, candidate
            accepted = True
            break
        self.preview = self.command.copy()
        self.reason = ('encoder lag: trajectory paused' if held else
                       'correction/trajectory limited by IK or speed' if not accepted else
                       'standing balance active' if self.balance_enabled else 'position preparation / hold')
        if self.table is not None and self.elapsed >= self.duration-1e-8:
            at_target = np.max(np.abs(self.feedback.q-self.command)) <= self.settings.tracking_tolerance
            quiet = np.max(np.abs(self.feedback.velocity)) < .08
            self.settled = self.settled+dt if at_target and quiet else 0.
            if self.settled >= .3:
                self._finish_segment()
        return self.command.copy()

    def status(self, now):
        error = None
        joint = None
        if self.command is not None and self.feedback.q is not None:
            index = int(np.argmax(np.abs(self.command-self.feedback.q)))
            error = math.degrees(abs(self.command[index]-self.feedback.q[index]))
            joint = NAMES[index]
        age = lambda stamp: None if stamp is None else max(0., now-stamp)
        return dict(state=self.state, location=self.location, actuate=self.actuate,
                    axes_verified=self.axes_verified, calibrated=self.imu.calibrated,
                    balance_enabled=self.balance_enabled, reason=self.reason, fault=self.fault,
                    imu_source=self.imu.used_source,
                    roll_pitch_deg=None if self.imu.angles is None else np.degrees(self.imu.angles).tolist(),
                    gyro_rad_s=self.imu.gyro.tolist(), accel_norm=self.imu.accel_norm,
                    correction_deg=np.degrees(self.correction).tolist(),
                    requested_correction_deg=np.degrees(self.requested).tolist(),
                    joint_age_s=age(self.feedback.joint_time), imu_age_s=age(self.imu.time),
                    diagnostic_age_s=age(self.feedback.diagnostic_time), torque_on=self.feedback.torque_on,
                    bridge_status=self.feedback.bridge_problem,
                    voltage_v=self.feedback.voltage, temperature_c=self.feedback.temperature,
                    tracking_error_deg=error, tracking_joint=joint, loads_raw=self.feedback.loads,
                    segment=self.segment, progress=min(self.elapsed/self.duration, 1.))
