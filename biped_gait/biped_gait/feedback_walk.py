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
from .placement import Placement, PhaseGeometry

NAMES = m.JOINT_ORDER


@dataclass(frozen=True)
class Settings:
    body_roll: float = .06
    crouch: float = .026
    centre_crouch: float = .018
    com_inset: float = .010
    support_margin: float = .008
    clearance: float = .006
    lift_clearance: float = .004
    kp: float = .30
    kd: float = .04
    gyro_filter_time: float = .04
    max_correction: float = math.radians(1.5)
    correction_rate: float = .05
    max_joint_speed: float = .25
    landing_speed: float = .002
    landing_tilt_error: float = math.radians(1.5)
    landing_rotation_rate: float = .12
    landing_tracking_error: float = .035
    landing_sole_tilt: float = math.radians(2.)
    landing_clearance: float = 0.
    placement_tolerance: float = .0003
    minimum_knee_bend: float = .12
    landing_alignment_clearance: float = .004
    landing_xy_speed: float = .003
    landing_xy_tolerance: float = .001
    placement_ramp_time: float = 1.
    placement_capture_acceleration: float = .30
    transfer_error: float = math.radians(1.25)
    transfer_rate: float = .10
    transfer_rate_duration: float = .10
    transfer_lookahead: float = .25
    penetration_tolerance: float = .001
    fault_tilt: float = math.radians(8)
    tracking_pause: float = .04
    tracking_fault: float = .15
    tracking_timeout: float = 2.
    settle_error: float = math.radians(1.5)
    settle_timeout: float = 4.
    settle_duration: float = .4

    def __post_init__(self):
        if any(not math.isfinite(v) or (v < 0 if k == 'landing_clearance' else v <= 0)
               for k, v in asdict(self).items()):
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
        if not .006 <= self.support_margin <= .010 or self.landing_clearance > .001:
            raise ValueError('invalid support margin/placement clearance')
        if (self.landing_alignment_clearance > self.clearance or self.landing_xy_speed > .005
                or self.landing_xy_tolerance > .002 or self.placement_tolerance > .0005
                or self.minimum_knee_bend < .12):
            raise ValueError('placement settings exceed the experimental bounds')
        if not .5 <= self.placement_ramp_time <= 2. or self.placement_capture_acceleration > .30:
            raise ValueError('invalid placement smoothing settings')

    def profile(self):
        return {name: getattr(self, name) for name in ('body_roll', 'crouch', 'centre_crouch', 'com_inset', 'support_margin', 'clearance', 'lift_clearance')}


class Plan:
    def __init__(self, direction, initial, settings):
        self.profile = settings.profile()
        self.initial = np.asarray(initial).copy()
        rotation = reference_rotation(self.initial)
        template_initial = (self.initial.copy() if np.linalg.norm(rotation-np.eye(3)) < 1e-8
                            else attitude_pose_matrix(self.initial, rotation.T))
        template_q = dict(zip(NAMES,template_initial))
        heights = [m._sole_center(template_q,s)[2] for s in 'rl']
        if abs(heights[0]-heights[1]) > .001:
            raise ValueError('stopped soles differ by over 1 mm; support robot and restart from known stance')
        if abs(heights[0]-heights[1]) > 1e-6:
            # Only the offline template is levelled. PhaseGeometry keeps the
            # actual landed anchors when the node resumes this cached motion.
            for side in 'rl':
                target=m._sole_center(template_q,side);target[2]=float(np.mean(heights))
                template_q=m._leg_pose_ik(template_q,side,target,np.eye(3))
            template_initial=np.array([template_q[n] for n in NAMES])
        self.keyframes, self.tables = m.build_tables(direction, template_initial, self.profile)
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
    def __init__(self, settings=None, *, precomputed=False):
        self.settings = settings or Settings()
        if precomputed:
            from .plan_store import load
            self.cache = {d: load(self.settings, d, np.zeros(12)) for d in ('forward', 'backward')}
        else:
            self.cache = {d: Plan(d, np.zeros(12), self.settings) for d in ('forward', 'backward')}

    def get(self, direction, initial):
        if not np.allclose(self.cache[direction].initial, initial, atol=1e-10, rtol=0):
            self.cache[direction] = Plan(direction, initial, self.settings)
        return self.cache[direction]


def reference_rotation(q, stance='l'):
    return m._foot_transform(dict(zip(NAMES, q)), stance)[0].T


def attitude_pose(nominal, correction):
    """Coordinate all six joints of each leg, retaining both nominal world soles.

    R_new_body = R_nominal_body @ R_correction. The relative foot target becomes
    R_correction.T @ its original pose. This also works for a leaning nominal body.
    """
    if np.max(np.abs(correction)) < 1e-10:
        return np.array(nominal).copy()
    return attitude_pose_matrix(nominal, rpy_matrix(*correction))


def attitude_pose_matrix(nominal, rotation):
    q = dict(zip(NAMES, nominal))
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
        self.planned_rate = np.zeros(2)
        self.balance_rate = np.zeros(2)
        self.balance_rate_time = None
        self.transfer_paused = False
        self.transfer_rate_since = None
        self.preparation_paused = False
        self.preparation_pause_since = None
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
        self.placement = None
        self.interruption = ''
        self.backoffs = 0
        self.relief_target = None
        self.support = 'l'
        self.phase_geometry = None
        self.target_problem = ''
        self.interruption_snapshot = None
        self.capture_correction = None
        self.joint_source_time = None

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
        self.interruption = ''
        self.reason = 'arming'
        self.interruption_snapshot = None

    def _begin(self, a, b):
        self.segment, self.state = (a, b), 'moving'
        self.reason = 'moving'
        self.clearance, self.landing_cues = None, {}
        self.elapsed = self.waited = self.settled = 0.
        self.planned_rate[:] = 0.
        self.transfer_paused = False
        self.transfer_rate_since = None
        self.preparation_paused = False
        self.preparation_pause_since = None
        new_support = m.STANCE.get(b, self.support)
        self.support = new_support
        try:
            self.phase_geometry = PhaseGeometry(self.plan.tables[(a,b)][0], self.nominal, self.support)
        except ValueError as exc:
            self.fail(str(exc), self.last_time or 0.)
            return
        self.placement = None
        self.capture_correction = None
        if b.startswith('place_'):
            self.placement = Placement(self.feedback.q, self.imu.angles, m.STANCE[b], self.s,
                                       reference_goal=self.plan.tables[(a,b)][-1])

    def _placement_hold(self, reason, now):
        if self.interruption_snapshot is None:
            self.interruption_snapshot = dict(time=now, segment=self.segment, reason=reason,
                roll_pitch_deg=np.degrees(self.imu.angles).tolist(), gyro_rad_s=self.imu.gyro.tolist(),
                tracking_joint=self.tracking_joint, tracking_error_rad=self.tracking_error,
                measured=self.feedback.q.tolist(), target=self.commanded.tolist(),
                clearance_m=self.clearance, landing_cues=dict(self.landing_cues),
                placement_elapsed_s=self.elapsed, aligning=self.placement.aligning,
                capturing=self.placement.touch_target is not None,
                xy_error_m=self.placement.xy_error, imu_source_time=self.imu.sample_time,
                joint_source_time=self.joint_source_time,
                sensor_skew_s=None if self.joint_source_time is None or self.imu.sample_time is None else self.imu.sample_time-self.joint_source_time)
        self.interruption = reason+'; contact NOT confirmed; support robot before restarting'
        self.reason = self.interruption
        self.state = 'placement_hold'
        try:
            self.relief_target = self.placement.relief(self.commanded, self.feedback.q, self.imu.angles)
        except ValueError:
            self.fail('placement relief infeasible: '+reason, now)

    def _placing(self, dt, now, error):
        self.reason = 'placing from measured bent-leg geometry'
        bend = min(-self.nominal[NAMES.index(side+'_knee_joint')] for side in 'rl')
        authority = float(np.clip((bend-.20)/.30,0.,1.))
        requested = np.clip(-self.s.kp*error-self.s.kd*self.balance_rate,
                            -self.s.max_correction, self.s.max_correction)*authority*.5
        for scale in (1., .5, .25, .125):
            correction = self.correction+np.clip(requested-self.correction,
                -self.s.correction_rate*dt, self.s.correction_rate*dt)*scale*self.placement.ramp_gain(dt*scale)
            try:
                base, descent, done, exhausted = self.placement.target(
                    self.commanded, self.feedback.q, self.imu.angles, dt*scale)
                if self.placement.touch_target is not None and self.capture_correction is None:
                    self.capture_correction = self.correction.copy()
                reference_correction = self.correction if self.capture_correction is None else self.capture_correction
                delta = rpy_matrix(*reference_correction).T @ rpy_matrix(*correction)
                candidate = attitude_pose_matrix(base, delta)
                if np.max(np.abs(candidate-self.commanded)) > self.s.max_joint_speed*dt+1e-7:
                    if done:
                        difference = candidate-self.commanded
                        fraction = min(1., self.s.max_joint_speed*dt/float(np.max(np.abs(difference))))
                        candidate = self.commanded+fraction*difference
                        done = False
                    else:
                        continue
                self.placement.accept(descent)
                self.commanded, self.correction = candidate, correction
                self.requested = requested.copy()
                self.nominal = attitude_pose_matrix(candidate, rpy_matrix(*correction).T)
                self.elapsed += dt*scale
                self.pause_since = None
                if exhausted:
                    self.reason = 'placement height budget reached; waiting without further descent'
                if done:
                    # The following phase will keep the landed sole at measured
                    # ground height; it does not finish an old nominal extension.
                    self.state, self.waited, self.settled = 'settling', 0., 0.
                elif self.elapsed > self.plan.durations[self.segment]+self.s.settle_timeout+self.s.placement_ramp_time+self.placement.capture_duration:
                    self._placement_hold('measured placement height budget exhausted', now)
                return self.commanded.copy()
            except ValueError as exc:
                self._placement_hold(str(exc), now)
                return self.commanded.copy()
        self.pause_since = now if self.pause_since is None else self.pause_since
        if now-self.pause_since > 1.:
            self._placement_hold('no bounded placement target', now)
        return self.commanded.copy()

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
        # Smooth the rate used for damping, using sensor time rather than timer
        # callback count. Raw gyro and tilt still drive all interruption checks.
        sample_time = self.imu.sample_time if self.imu.sample_time is not None else self.imu.time
        if self.balance_rate_time is None:
            self.balance_rate = self.imu.gyro[:2].copy()
        elif sample_time > self.balance_rate_time:
            gain = -math.expm1(-(sample_time-self.balance_rate_time)/self.s.gyro_filter_time)
            self.balance_rate += gain*(self.imu.gyro[:2]-self.balance_rate)
        self.balance_rate_time = sample_time
        rate_dt = dt
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
            self.reason = self.fault or self.state
            return self.commanded.copy()
        target_angles = tilt(reference_rotation(self.nominal,self.support))
        error = self.imu.angles-target_angles
        # The initial crouch lowers the body with both feet planted. Near a
        # straight knee the attitude correction has no authority yet, so a
        # lateral transfer prediction must not reverse this preparation.
        preparing = self.state == 'moving' and self.segment == ('neutral','ready')
        self.preparation_paused = bool(preparing and (np.max(np.abs(error)) > self.s.settle_error
            or np.linalg.norm(self.imu.gyro[:2]) > self.s.transfer_rate))
        if self.preparation_paused:
            self.preparation_pause_since = now if self.preparation_pause_since is None else self.preparation_pause_since
            if now-self.preparation_pause_since > self.s.settle_timeout:
                self.fail('initial crouch did not stabilize; check level stance and IMU mounting; last target held',now)
                return self.commanded.copy()
        else:
            self.preparation_pause_since = None
        transfer = self.segment is not None and self.segment[1] in ('lean_left', 'lean_right', 'lean_left_cycle')
        self.transfer_paused = False
        if self.state == 'moving' and transfer:
            # Predict ERROR growth. The commanded body rotation is intentional,
            # so its rate must be removed from the measured gyro rate.
            error_rate = self.imu.gyro[:2]-self.planned_rate
            predicted = error+self.s.transfer_lookahead*error_rate
            rate_error = float(np.linalg.norm(error_rate))
            # A single rate spike must not reverse the entire transfer. Pause
            # its nominal progress, retain posture feedback, and require 100 ms
            # of sensor time before backing off on rate alone. Hysteresis keeps
            # a value near the threshold from repeatedly pausing and resuming.
            sample_time = self.imu.sample_time if self.imu.sample_time is not None else self.imu.time
            if rate_error > self.s.transfer_rate and self.transfer_rate_since is None:
                self.transfer_rate_since = sample_time
            if rate_error < .8*self.s.transfer_rate:
                self.transfer_rate_since = None
            rate_duration = 0. if self.transfer_rate_since is None else max(0.,sample_time-self.transfer_rate_since)
            self.transfer_paused = self.transfer_rate_since is not None
            tilt_unsafe = np.max(np.abs(error)) > self.s.transfer_error or np.max(np.abs(predicted)) > self.s.transfer_error
            rate_unsafe = self.transfer_paused and rate_duration+1e-9 >= self.s.transfer_rate_duration
            if tilt_unsafe or rate_unsafe:
                if self.interruption_snapshot is None:
                    self.interruption_snapshot = dict(kind='weight_transfer',time=now,segment=self.segment,
                        measured_tilt_deg=np.degrees(self.imu.angles).tolist(),
                        planned_tilt_deg=np.degrees(target_angles).tolist(),
                        tilt_error_deg=np.degrees(error).tolist(),
                        predicted_error_deg=np.degrees(predicted).tolist(),
                        gyro_rad_s=self.imu.gyro.tolist(),planned_rate_rad_s=self.planned_rate.tolist(),
                        error_rate_rad_s=error_rate.tolist(), elapsed_s=self.elapsed,
                        predicted_error_limit_deg=math.degrees(self.s.transfer_error),
                        rate_error_limit_rad_s=self.s.transfer_rate,
                        trigger='tilt prediction' if tilt_unsafe else 'persistent rotation rate',
                        rate_duration_s=rate_duration,
                        imu_source_time=self.imu.sample_time,joint_source_time=self.joint_source_time,
                        target=self.commanded.tolist(),measured=self.feedback.q.tolist())
                self.state = 'recovering'
                self.transfer_paused = False
                self.stop_requested = True
                self.backoffs += 1
                self.interruption = ('weight transfer backed off before foot lift; '
                    f'predicted error={float(np.degrees(np.max(np.abs(predicted)))):.3f} deg (limit {math.degrees(self.s.transfer_error):.3f}), '
                    f'roll/pitch error={np.degrees(error).round(2).tolist()} deg, '
                    f'planned rate={self.planned_rate.round(4).tolist()} rad/s, '
                    f'error rate={rate_error:.3f} rad/s (entry {self.s.transfer_rate:.3f}, release {.8*self.s.transfer_rate:.3f}, duration {rate_duration:.3f} s); '
                    f'trigger={"tilt prediction" if tilt_unsafe else "persistent rotation rate"}; posture feedback remains active')
        landing = self.segment is not None and self.segment[1].startswith('place_')
        if landing and self.state != 'placement_hold':
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
                self._placement_hold('landing interrupted: '+str(self.landing_cues), now)
                return self.commanded.copy()
        if self.tracking_error > self.s.tracking_pause:
            return self._lag(now)
        self.lag_since = None
        if landing and self.state == 'moving':
            return self.commanded.copy() if dt <= 0 else self._placing(dt, now, error)
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
        if not landing and self.state == 'moving' and self.segment[1].startswith('place_'):
            # _advance entered placement in this tick. Its first motion belongs
            # to the ramped placement controller, not the generic table branch.
            return self.commanded.copy()
        # Backtrack the complete nominal progress AND attitude correction, never
        # clip separate joint angles. Hold if no synchronized candidate is feasible.
        for scale in (1., .5, .25, .125, 0.):
            progress = -1. if self.state == 'recovering' else 1. if self.state == 'moving' and not (self.transfer_paused or self.preparation_paused) else 0.
            next_elapsed = max(0., self.elapsed+progress*dt*scale)
            nominal = self.nominal.copy()
            if self.state in ('moving', 'recovering'):
                nominal = m.table_pose(self.plan.tables[self.segment], next_elapsed/self.plan.durations[self.segment])
                if self.phase_geometry is not None:
                    try:
                        nominal = self.phase_geometry.pose(nominal, self.nominal, next_elapsed/self.plan.durations[self.segment])
                    except ValueError as exc:
                        self.target_problem = str(exc)
                        continue
            elif self.state == 'placement_hold':
                relief = attitude_pose_matrix(self.relief_target, rpy_matrix(*self.correction).T)
                delta = relief-nominal
                nominal += delta*min(1., .1*dt*scale/max(float(np.max(np.abs(delta))), 1e-9))
            reference = tilt(reference_rotation(nominal,self.support))
            rate = (reference-target_angles)/max(rate_dt, 1e-6)
            requested = -self.s.kp*(self.imu.angles-reference)-self.s.kd*(self.balance_rate-rate)
            bend = min(-nominal[NAMES.index(side+'_knee_joint')] for side in 'rl')
            authority = float(np.clip((bend-.20)/.30, 0., 1.))
            requested = np.clip(requested, -self.s.max_correction, self.s.max_correction)*authority*(.5 if landing else 1.)
            correction = self.correction+np.clip(requested-self.correction, -self.s.correction_rate*dt, self.s.correction_rate*dt)*scale
            try:
                candidate = attitude_pose(nominal, correction)
                if np.max(np.abs(candidate-self.commanded)) > self.s.max_joint_speed*dt+1e-7:
                    index = int(np.argmax(np.abs(candidate-self.commanded)))
                    self.target_problem = f'{NAMES[index]} target step {candidate[index]-self.commanded[index]:.6f} rad exceeds {self.s.max_joint_speed*dt:.6f}'
                    continue
            except ValueError as exc:
                self.target_problem = str(exc)
                continue
            self.nominal, self.commanded = nominal, candidate
            if rate_dt > 1e-6:
                self.planned_rate = rate.copy() if self.state in ('moving','recovering') else np.zeros(2)
            self.target_problem = ''
            self.correction, self.requested = correction, requested
            self.elapsed = next_elapsed
            if scale == 0. and self.state in ('moving', 'recovering'):
                self.pause_since = now if self.pause_since is None else self.pause_since
                if now-self.pause_since > 2.:
                    self.fail('no feasible forward progress under feedback limits', now)
            else:
                self.pause_since = None
            if self.state == 'moving' and self.elapsed >= self.plan.durations[self.segment]:
                self.state, self.waited, self.settled = 'settling', 0., 0.
            if self.state == 'recovering' and self.elapsed <= 0.:
                self.state = 'standing' if np.max(np.abs(tilt(reference_rotation(self.nominal,self.support)))) < .001 else 'transfer_hold'
                self.segment = None
            break
        else:
            self.pause_since = now if self.pause_since is None else self.pause_since
            if now-self.pause_since > 1.:
                self.fail('no feasible bounded feedback target', now)
        self.reason = self.fault or (self.interruption if self.interruption and self.state in ('recovering', 'placement_hold', 'transfer_hold', 'standing') else
            'initial crouch paused for posture to settle; both feet remain planted' if self.preparation_paused else
            'weight transfer paused for rotation to settle; posture feedback remains active' if self.transfer_paused else self.state)
        return self.commanded.copy()

    def _lag(self, now):
        self.lag_since = now if self.lag_since is None else self.lag_since
        self.reason = 'trajectory paused for servo tracking'
        if now-self.lag_since > self.s.tracking_timeout:
            self.fail('servo tracking did not recover; last target held', now)
        return self.commanded.copy()

    def status(self, now):
        placement = None if self.placement is None else dict(
            body_height_m=self.placement.last.body_height, initial_clearance_m=self.placement.initial_clearance,
            descent_start_clearance_m=getattr(self.placement,'descent_start_clearance',self.placement.initial_clearance),
            measured_clearance_m=self.placement.last.clearance, remaining_descent_m=self.placement.remaining,
            commanded_descent_m=self.placement.commanded_descent, contact_confirmed=False,
            aligning=self.placement.aligning, xy_error_m=self.placement.xy_error)
        return dict(state=self.state, segment=self.segment, reason=self.reason, fault=self.fault,
            fault_snapshot=self.snapshot, hardware_ready=not bool(self.readiness(now)), output_suppressed=self.suppress,
            calibrated=self.imu.calibrated, roll_pitch_deg=None if self.imu.angles is None else np.degrees(self.imu.angles).tolist(),
            planned_roll_pitch_deg=np.degrees(tilt(reference_rotation(self.nominal,self.support))).tolist(),
            planned_rate_rad_s=self.planned_rate.tolist(),
            balance_rate_rad_s=self.balance_rate.tolist(),
            transfer_rate_paused=self.transfer_paused,
            preparation_paused=self.preparation_paused,
            correction_deg=np.degrees(self.correction).tolist(), maximum_lean_deg=math.degrees(self.s.body_roll),
            tracking_joint=self.tracking_joint, tracking_error_rad=self.tracking_error,
            predicted_swing_clearance_m=self.clearance, landings=self.landings,
            contact_measured=False, imu_source=self.imu.used_source, placement=placement,
            interruption=self.interruption, interruption_snapshot=self.interruption_snapshot,
            transfer_backoffs=self.backoffs, target_problem=self.target_problem)
