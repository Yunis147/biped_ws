"""Small-lean quasi-static gait with phase-aware IMU feedback and guarded descent.

All inputs are supplied by a ROS boundary or tests; this module does no I/O.
A landing is still an assumed contact, never a force/pressure measurement.
"""
from dataclasses import dataclass, asdict
import math
import numpy as np
from . import quasi_static_walk as m
from .imu import TiltEstimator, rpy_matrix, tilt
from .servo_feedback import ServoFeedback

NAMES = m.JOINT_ORDER


@dataclass(frozen=True)
class Settings:
    body_roll: float = .08
    crouch: float = .028
    centre_crouch: float = .020
    com_inset: float = .008
    clearance: float = .010
    lift_clearance: float = .006
    kp: float = .30
    kd: float = .04
    max_correction: float = math.radians(1.5)
    correction_rate: float = .05
    max_joint_speed: float = .25
    landing_speed: float = .002
    landing_tilt_error: float = math.radians(1.5)
    landing_rotation_rate: float = .12
    landing_tracking_error: float = .035
    landing_sole_tilt: float = math.radians(2.)
    penetration_tolerance: float = .001
    fault_tilt: float = math.radians(8)
    tracking_pause: float = .04
    tracking_fault: float = .15
    tracking_timeout: float = 2.
    settle_error: float = math.radians(1.5)
    settle_timeout: float = 4.
    settle_duration: float = .4

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('walking settings must be finite and positive')
        if self.body_roll > .10 or self.max_correction > math.radians(1.5):
            raise ValueError('lean is limited to 5.73 deg and correction to 1.5 deg')
        if not .012 <= self.centre_crouch <= self.crouch <= .035:
            raise ValueError('invalid body heights')
        if not self.lift_clearance <= self.clearance <= .012:
            raise ValueError('invalid foot clearances')
        if self.landing_speed > .002 or self.max_joint_speed > .35:
            raise ValueError('landing/joint speed exceeds the experimental limit')
        if not self.landing_tracking_error <= self.tracking_pause < self.tracking_fault <= .20:
            raise ValueError('invalid tracking thresholds')
        if self.body_roll+self.max_correction >= self.fault_tilt:
            raise ValueError('fault tilt must leave room for planned lean plus feedback')

    def profile(self):
        return {name: getattr(self, name) for name in ('body_roll', 'crouch', 'centre_crouch', 'com_inset', 'clearance', 'lift_clearance')}


class Plan:
    def __init__(self, direction, initial, settings):
        self.profile = settings.profile()
        self.initial = np.asarray(initial).copy()
        self.keyframes, self.tables = m.build_tables(direction, initial, self.profile)
        if not m.report(direction, self.keyframes, self.tables, lambda msg: None, self.profile):
            raise ValueError('small-lean gait failed geometry checks')
        self.metrics = m.gait_metrics(self.keyframes, self.tables, self.profile)
        self.durations = {}
        for segment, table in self.tables.items():
            duration = m.segment_duration(segment[1], table)
            if segment[1].startswith('place_'):
                # For z = clearance * (1 - smooth(t)), peak speed is 1.875*h/T.
                duration = max(duration, 1.875*settings.clearance/settings.landing_speed)
            self.durations[segment] = duration


class Plans:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.cache = {d: Plan(d, np.zeros(12), self.settings) for d in ('forward', 'backward')}

    def get(self, direction, initial):
        if not np.allclose(self.cache[direction].initial, initial, atol=1e-10, rtol=0):
            self.cache[direction] = Plan(direction, initial, self.settings)
        return self.cache[direction]


def reference_rotation(q):
    return m._foot_transform(dict(zip(NAMES, q)), 'l')[0].T


def attitude_pose(nominal, correction):
    """Coordinate all six joints of each leg, retaining both nominal world soles.

    R_new_body = R_nominal_body @ R_correction. The relative foot target becomes
    R_correction.T @ its original pose. This also works for a leaning nominal body.
    """
    q = dict(zip(NAMES, nominal))
    if np.max(np.abs(correction)) < 1e-10:
        return np.array(nominal).copy()
    rotation = rpy_matrix(*correction)
    for side in 'rl':
        original_r, _ = m._foot_transform(dict(zip(NAMES, nominal)), side)
        target = rotation.T @ m._sole_center(dict(zip(NAMES, nominal)), side)
        q = m._leg_pose_ik(q, side, target, rotation.T @ original_r)
    return np.array([q[n] for n in NAMES])


class WalkingController:
    def __init__(self, plans=None, *, simulation=False, imu_source='auto', mount=(0., 0., 0.)):
        self.plans = plans or Plans()
        self.s = self.plans.settings
        self.simulation = simulation
        self.imu = TiltEstimator(mount, imu_source)
        self.feedback = ServoFeedback()
        self.state = 'idle'
        self.direction = 'forward'
        self.plan = self.plans.cache['forward']
        self.nominal = np.zeros(12)
        self.commanded = None
        self.correction = np.zeros(2)
        self.requested = np.zeros(2)
        self.segment = None
        self.elapsed = self.waited = self.settled = 0.
        self.last_time = None
        self.stop_requested = self.single_step = self.stand_only = False
        self.pending = None
        self.lag_since = None
        self.fault = self.reason = ''
        self.snapshot = None
        self.suppress = False
        self.landings = 0
        self.tracking_joint = None
        self.tracking_error = 0.
        self.clearance = None
        self.landing_cues = {}
        self.arming_since = None
        self.pause_since = None
        self.resync = False

    def readiness(self, now):
        if getattr(self, 'imu_problem', ''):
            return self.imu_problem
        if self.feedback.joint_problem:
            return self.feedback.joint_problem
        if self.feedback.joint_time is None or not 0 <= now-self.feedback.joint_time <= .25:
            return 'missing/stale joint positions'
        if self.imu.time is None or not 0 <= now-self.imu.time <= .25:
            return 'missing/stale IMU'
        if not self.simulation:
            return self.feedback.problem(now)
        return ''

    def fail(self, reason, now, suppress=False):
        if not self.fault:
            if self.commanded is not None and self.feedback.q is not None:
                errors = np.abs(self.feedback.q-self.commanded)
                index = int(np.argmax(errors))
                self.tracking_error, self.tracking_joint = float(errors[index]), NAMES[index]
            self.fault = reason
            self.snapshot = dict(time=now, segment=self.segment, state=self.state,
                tracking_joint=self.tracking_joint, tracking_error_rad=self.tracking_error,
                measured=None if self.feedback.q is None else self.feedback.q.tolist(),
                target=None if self.commanded is None else self.commanded.tolist(),
                roll_pitch_deg=None if self.imu.angles is None else np.degrees(self.imu.angles).tolist(),
                bridge_problem=self.feedback.bridge_problem, landing_cues=dict(self.landing_cues),
                predicted_clearance_m=self.clearance, gyro_rad_s=self.imu.gyro.tolist(),
                imu_problem=getattr(self, 'imu_problem', ''), imu_age=None if self.imu.time is None else now-self.imu.time,
                joint_age=None if self.feedback.joint_time is None else now-self.feedback.joint_time,
                diagnostics=getattr(self, 'last_diagnostics', None))
        self.suppress |= suppress
        self.state, self.reason = 'fault', self.fault

    def command(self, text, now):
        text = text.strip().lower()
        if text == 'status':
            return
        if text == 'hold':
            if self.commanded is not None:
                self.fail('hold requested; last valid target retained', now)
            return
        if self.fault:
            raise ValueError('latched fault: '+self.fault+'; support the robot and restart from a known stance')
        if text == 'calibrate':
            if self.state not in ('idle', 'standing'):
                raise ValueError('calibrate only while standing still')
            self.imu.calibrate(now)
            return
        if text == 'stop':
            self.stop_requested = True
            if self.state in ('arming', 'approach', 'approach_settle'):
                self.fail('startup stopped; last valid target retained', now)
            return
        if text not in ('start', 'forward', 'backward', 'step', 'stand'):
            raise ValueError('use stand / start / forward / backward / step / stop / hold / calibrate / status')
        if self.state not in ('idle', 'standing'):
            raise ValueError('already moving; stop and wait for standing')
        problem = self.readiness(now)
        if problem:
            raise ValueError(problem)
        q, velocity = self.feedback.q, self.feedback.velocity
        if velocity is None or np.max(np.abs(velocity)) > .08:
            raise ValueError('wait for stationary servos')
        if np.max(np.abs(q-self.nominal)) > .05:
            raise ValueError('start near calibrated zero or the last held stance; no arbitrary-pose homing')
        direction = 'backward' if text == 'backward' else 'forward'
        self.plan = self.plans.get(direction, self.nominal)
        self.direction, self.pending = direction, text
        self.state, self.arming_since = 'arming', None
        self.last_time = None
        self.stop_requested = False
        self.single_step, self.stand_only = text == 'step', text == 'stand'
        self.landings = 0

    def _begin(self, a, b):
        self.segment, self.state = (a, b), 'moving'
        self.clearance, self.landing_cues = None, {}
        self.elapsed = self.waited = self.settled = 0.

    def _settled(self, error):
        return (np.max(np.abs(error)) <= self.s.settle_error and
                np.linalg.norm(self.imu.gyro[:2]) < .08 and self.tracking_error < .03)

    def _advance(self, now):
        arrived = self.segment[1]
        if arrived.startswith('place_'):
            self.landings += 1
        if arrived == 'neutral' or (arrived == 'ready' and self.stand_only):
            self.state, self.segment = 'standing', None
            return
        next_phase = 'neutral' if ((self.stop_requested or (self.single_step and self.landings))
                                   and arrived in m.SAFE_STOP) else m.NEXT[arrived]
        if next_phase in m.STANCE:
            com, feet = m._com_and_feet(dict(zip(NAMES, self.feedback.q)))
            R = rpy_matrix(*self.imu.angles)
            supporting = feet[m.STANCE[next_phase]] @ R.T
            margin = m._margin(m._hull(supporting[:, :2]), (R @ com)[:2])
            if margin < .005:
                self.fail('weight transfer has insufficient measured-model support margin; foot lift blocked', now)
                return
        self._begin(arrived, next_phase)

    def tick(self, now):
        dt = 0. if self.last_time is None else now-self.last_time
        self.last_time = now
        problem = self.readiness(now)
        if self.state == 'idle':
            return None
        if problem:
            if self.state == 'arming':
                self.arming_since = now if self.arming_since is None else self.arming_since
                if now-self.arming_since < 4.:
                    self.reason = problem
                    return None
            self.fail(problem, now, suppress=True)
        if self.state == 'arming' and not self.fault:
            if not self.imu.calibrated:
                try:
                    self.imu.calibrate(now)
                except ValueError as error:
                    self.arming_since = now if self.arming_since is None else self.arming_since
                    self.reason = str(error)
                    if now-self.arming_since > 4.:
                        self.fail(self.reason, now, suppress=True)
                    return None
            if np.max(np.abs(self.feedback.q-self.nominal)) > .05 or self.feedback.velocity is None or np.max(np.abs(self.feedback.velocity)) > .08:
                self.fail('starting pose moved while arming', now, suppress=True)
            else:
                actual = self.feedback.q.copy()
                self.approach = np.array([actual+m._smoothstep(u)*(self.nominal-actual) for u in np.linspace(0, 1, 81)])
                speed, acceleration = m._motion_bounds(self.approach)
                self.approach_duration = max(1., speed/.1, math.sqrt(acceleration/.3))
                self.commanded, self.state, self.elapsed = actual, 'approach', 0.
                dt = 0.
        if self.suppress:
            return None
        if self.commanded is not None:
            differences = np.abs(self.feedback.q-self.commanded)
            index = int(np.argmax(differences))
            self.tracking_error, self.tracking_joint = float(differences[index]), NAMES[index]
        if self.fault:
            return None if self.commanded is None else self.commanded.copy()
        if dt < 0 or dt > .25:
            self.fail('control timer gap/reset; last target held', now)
        if np.max(np.abs(self.imu.angles)) > self.s.fault_tilt:
            self.fail('measured torso tilt exceeds configured limit; last target held', now)
        if self.tracking_error > self.s.tracking_fault:
            self.fail('large servo tracking error; last target held', now)
        if self.fault:
            return self.commanded.copy()
        dt = min(max(dt, 0.), .04)
        if self.state in ('approach', 'approach_settle'):
            if self.tracking_error > self.s.tracking_pause:
                return self._lag(now)
            self.lag_since = None
            self.elapsed += dt
            if self.state == 'approach':
                self.commanded = m.table_pose(self.approach, self.elapsed/self.approach_duration)
                if self.elapsed >= self.approach_duration:
                    self.state, self.elapsed, self.settled = 'approach_settle', 0., 0.
            else:
                self.settled = self.settled+dt if self.tracking_error < .02 and np.max(np.abs(self.feedback.velocity)) < .08 else 0.
                if self.settled >= .2:
                    self.correction[:] = 0.
                    self._begin('neutral', 'ready')
                elif self.elapsed > 5.:
                    self.fail('starting stance did not settle', now)
            return self.commanded.copy()
        target_angles = tilt(reference_rotation(self.nominal))
        error = self.imu.angles-target_angles
        landing = self.segment is not None and self.segment[1].startswith('place_')
        if landing:
            stance = m.STANCE[self.segment[1]]
            _, feet = m._com_and_feet(dict(zip(NAMES, self.feedback.q)))
            R = rpy_matrix(*self.imu.angles)
            swing = 'r' if stance == 'l' else 'l'
            self.clearance = float((feet[swing] @ R.T)[:, 2].min()-(feet[stance] @ R.T)[:, 2].min())
            sole_rotation = R @ m._foot_transform(dict(zip(NAMES, self.feedback.q)), swing)[0]
            sole_tilt = math.acos(float(np.clip(sole_rotation[2, 2], -1., 1.)))
            self.landing_cues = {}
            if np.max(np.abs(error)) > self.s.landing_tilt_error:
                self.landing_cues['tilt_error_deg'] = float(np.degrees(np.max(np.abs(error))))
            if np.linalg.norm(self.imu.gyro[:2]) > self.s.landing_rotation_rate:
                self.landing_cues['rotation_rate_rad_s'] = float(np.linalg.norm(self.imu.gyro[:2]))
            if self.tracking_error > self.s.landing_tracking_error:
                self.landing_cues['tracking_error_rad'] = self.tracking_error
            if self.clearance < -self.s.penetration_tolerance:
                self.landing_cues['predicted_clearance_m'] = self.clearance
            if self.clearance < .002 and sole_tilt > self.s.landing_sole_tilt:
                self.landing_cues['near_ground_sole_tilt_deg'] = math.degrees(sole_tilt)
            if self.landing_cues:
                self.fail('landing interrupted: '+str(self.landing_cues)+'; contact NOT confirmed', now)
                return self.commanded.copy()
        if self.tracking_error > self.s.tracking_pause:
            return self._lag(now)
        self.lag_since = None
        if self.state == 'settling':
            self.waited += dt
            self.settled = self.settled+dt if self._settled(error) else 0.
            minimum = m.DWELL_TIME.get(self.segment[1], .4)*m.TIME_SCALE
            if self.waited >= minimum and self.settled >= self.s.settle_duration:
                self._advance(now)
            elif self.waited > minimum+self.s.settle_timeout:
                self.fail('body/servos did not settle; next phase blocked', now)
        if self.fault:
            return self.commanded.copy()
        # Backtrack the complete nominal progress AND attitude correction, never
        # clip separate joint angles. Hold if no synchronized candidate is feasible.
        for scale in (1., .5, .25, .125, 0.):
            next_elapsed = self.elapsed + (dt*scale if self.state == 'moving' else 0.)
            nominal = self.nominal.copy()
            if self.state == 'moving':
                nominal = m.table_pose(self.plan.tables[self.segment], next_elapsed/self.plan.durations[self.segment])
            reference = tilt(reference_rotation(nominal))
            rate = (reference-target_angles)/max(dt, 1e-6)
            requested = -self.s.kp*(self.imu.angles-reference)-self.s.kd*(self.imu.gyro[:2]-rate)
            bend = min(-nominal[NAMES.index(side+'_knee_joint')] for side in 'rl')
            authority = float(np.clip((bend-.20)/.30, 0., 1.))
            requested = np.clip(requested, -self.s.max_correction, self.s.max_correction)*authority*(.5 if landing else 1.)
            correction = self.correction+np.clip(requested-self.correction, -self.s.correction_rate*dt, self.s.correction_rate*dt)*scale
            try:
                candidate = attitude_pose(nominal, correction)
                if np.max(np.abs(candidate-self.commanded)) > self.s.max_joint_speed*dt+1e-7:
                    continue
            except ValueError:
                continue
            self.nominal, self.commanded = nominal, candidate
            self.correction, self.requested = correction, requested
            self.elapsed = next_elapsed
            if scale == 0. and self.state == 'moving':
                self.pause_since = now if self.pause_since is None else self.pause_since
                if now-self.pause_since > 2.:
                    self.fail('no feasible forward progress under feedback limits', now)
            else:
                self.pause_since = None
            if self.state == 'moving' and self.elapsed >= self.plan.durations[self.segment]:
                self.state, self.waited, self.settled = 'settling', 0., 0.
            break
        else:
            self.pause_since = now if self.pause_since is None else self.pause_since
            if now-self.pause_since > 1.:
                self.fail('no feasible bounded feedback target', now)
        self.reason = self.state if not self.fault else self.fault
        return self.commanded.copy()

    def _lag(self, now):
        self.lag_since = now if self.lag_since is None else self.lag_since
        self.reason = 'trajectory paused for servo tracking'
        if now-self.lag_since > self.s.tracking_timeout:
            self.fail('servo tracking did not recover; last target held', now)
        return self.commanded.copy()

    def status(self, now):
        return dict(state=self.state, segment=self.segment, reason=self.reason, fault=self.fault,
            fault_snapshot=self.snapshot, hardware_ready=not bool(self.readiness(now)), output_suppressed=self.suppress,
            calibrated=self.imu.calibrated, roll_pitch_deg=None if self.imu.angles is None else np.degrees(self.imu.angles).tolist(),
            planned_roll_pitch_deg=np.degrees(tilt(reference_rotation(self.nominal))).tolist(),
            correction_deg=np.degrees(self.correction).tolist(), maximum_lean_deg=math.degrees(self.s.body_roll),
            tracking_joint=self.tracking_joint, tracking_error_rad=self.tracking_error,
            predicted_swing_clearance_m=self.clearance, landings=self.landings,
            contact_measured=False, imu_source=self.imu.used_source)
