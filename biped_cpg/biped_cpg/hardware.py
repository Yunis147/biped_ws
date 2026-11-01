"""Servo readiness and tracking interlocks; no IMU and no torque/service calls."""
from dataclasses import dataclass, asdict
import math

import numpy as np

from .curves import Curve
from .planner import NAMES, model, smooth


@dataclass(frozen=True)
class HardwareLimits:
    joint_timeout: float = .25
    diagnostic_timeout: float = 2.5
    start_tolerance: float = .05
    stationary_speed: float = .08
    tracking_pause: float = .04
    tracking_fault: float = .15
    tracking_timeout: float = 2.
    settle_tolerance: float = .02
    settle_duration: float = .2
    settle_timeout: float = 8.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('hardware limits must be finite and positive')
        if not self.settle_tolerance < self.tracking_pause < self.tracking_fault:
            raise ValueError('settle tolerance must be below pause threshold, below fault threshold')
        if self.start_tolerance > .08 or self.tracking_fault > .25:
            raise ValueError('start/tracking limits are too large for this hardware experiment')


class ServoFeedback:
    def __init__(self, limits=None):
        self.limits = limits or HardwareLimits()
        self.q = self.velocity = None
        self.joint_time = self.diagnostic_time = None
        self.joint_problem = ''
        self.bridge_problem = 'waiting for servo_bridge diagnostics'

    def joints(self, names, positions, velocities, now, stamp_age=0.):
        try:
            if not math.isfinite(stamp_age) or not -.05 <= stamp_age <= self.limits.joint_timeout:
                raise ValueError('stale/future joint-state source timestamp')
            if len(names) != len(positions) or len(set(names)) != len(names):
                raise ValueError('incomplete or duplicated joint-state names')
            values = dict(zip(names, positions))
            q = np.array([values.get(n, float('nan')) for n in NAMES])
            if not np.all(np.isfinite(q)):
                raise ValueError('joint states must contain all 12 finite positions')
            if np.any(q < model.LO) or np.any(q > model.HI):
                raise ValueError('measured joint outside URDF limits; check calibration')
            if len(velocities) == len(names) and np.all(np.isfinite(velocities)):
                speeds = dict(zip(names, velocities))
                velocity = np.array([speeds[n] for n in NAMES])
            elif self.q is not None and now > self.joint_time:
                velocity = (q-self.q)/(now-self.joint_time)
            else:
                velocity = None
            self.q, self.velocity, self.joint_time = q, velocity, now
            self.joint_problem = ''
        except ValueError as error:
            self.joint_problem = str(error)
            raise

    def diagnostics(self, message, level, values, now, stamp_age=0.):
        self.bridge_problem = 'invalid servo_bridge diagnostics'
        if not math.isfinite(stamp_age) or not -.05 <= stamp_age <= self.limits.diagnostic_timeout:
            raise ValueError('stale/future diagnostic source timestamp')
        if isinstance(level, (bytes, bytearray)) and len(level) == 1:
            level = level[0]
        if not isinstance(level, int) or level not in (0, 1, 2, 3):
            raise ValueError('invalid diagnostic level')
        self.diagnostic_time = now
        if message != 'torque ON':
            self.bridge_problem = 'servo torque is OFF'
        elif values.get('startup_positioning', '').lower() != 'false':
            self.bridge_problem = 'waiting for bridge startup positioning to finish'
        elif level:
            self.bridge_problem = 'servo bridge reports a health warning/error'
        else:
            self.bridge_problem = ''

    def problem(self, now):
        if self.joint_problem:
            return self.joint_problem
        if self.joint_time is None or not 0 <= now-self.joint_time <= self.limits.joint_timeout:
            return 'missing/stale /joint_states'
        if self.diagnostic_time is None or not 0 <= now-self.diagnostic_time <= self.limits.diagnostic_timeout:
            return 'missing/stale servo_bridge /diagnostics'
        return self.bridge_problem


class HardwareWalker:
    """Wrap the unchanged gait with a measured-pose approach and lag interlocks.

    now is monotonic time. Source ROS timestamps are checked at the input boundary.
    Stops never toggle torque. A latched fault requires a deliberate node restart.
    """
    def __init__(self, walker, limits=None):
        self.walker = walker
        self.feedback = ServoFeedback(limits)
        self.limits = self.feedback.limits
        self.phase = None
        self.pending = None
        self.target = None
        self.align = None
        self.elapsed = 0.
        self.last_time = None
        self.armed_at = self.settled_at = self.lag_since = None
        self.paused = False
        self.error = 0.
        self.reason = 'waiting for hardware feedback; no commands sent'
        self.fault = ''
        self.suppress = False

    @property
    def active(self):
        return self.walker.active or self.phase is not None or self.target is not None

    def fail(self, reason, suppress=False):
        if not self.fault:
            self.fault = reason
        self.suppress |= suppress
        self.phase = None
        self.walker.fail(self.fault, suppress=self.suppress)

    def command(self, command, now):
        command = command.strip().lower()
        if command == 'status':
            return 'status'
        if command == 'hold':
            if self.active:
                self.fail('hold requested; last target held')
            return 'holding'
        if self.fault:
            raise ValueError(f'latched fault: {self.fault}; support the robot and restart the node')
        if command == 'stop' and self.phase is not None:
            self.phase = None
            # Hold the measured-pose approach where it reached, without snapping
            # to zero. Restart requires returning to a known stance.
            self.fail('stopped during startup approach; last target held')
            return 'startup approach stopped; restart from a known stance'
        if command not in ('start', 'forward', 'backward', 'step'):
            return self.walker.command(command, now)
        if self.phase or self.walker.state != 'idle':
            raise ValueError('already moving; stop and wait for idle')
        problem = self.feedback.problem(now)
        if problem:
            raise ValueError(problem)
        q, velocity = self.feedback.q, self.feedback.velocity
        if velocity is None or np.max(np.abs(velocity)) > self.limits.stationary_speed:
            raise ValueError('wait for the servos to become stationary before starting')
        if np.max(np.abs(q-self.walker.last_pose)) > self.limits.start_tolerance:
            raise ValueError('start near calibrated zero, or the last CPG held stance; no arbitrary-pose homing')
        direction = 'backward' if command == 'backward' else 'forward'
        # Compile before moving. After compilation wait for fresh feedback again.
        self.walker.library.get(direction, self.walker.last_pose)
        self.pending, self.phase = command, 'arming'
        self.armed_at = self.last_time = None
        self.lag_since = self.settled_at = None
        return 'waiting for fresh stationary servo feedback, then approaching the known stance'

    def tick(self, now):
        dt = 0. if self.last_time is None else now-self.last_time
        self.last_time = now
        problem = self.feedback.problem(now)
        if self.phase == 'arming' and not self.fault:
            if self.armed_at is None:
                self.armed_at = now
            if problem:
                self.reason = problem
                if now-self.armed_at > 3.:
                    self.fail(problem, suppress=True)
                return None
            q, v = self.feedback.q, self.feedback.velocity
            if v is None or np.max(np.abs(v)) > self.limits.stationary_speed:
                self.fail('servos moved while arming', suppress=True)
            elif np.max(np.abs(q-self.walker.last_pose)) > self.limits.start_tolerance:
                self.fail('starting pose changed while arming', suppress=True)
            else:
                self.target = q.copy()
                rows = [q+smooth(u)*(self.walker.last_pose-q) for u in np.linspace(0, 1, 81)]
                self.align = Curve(rows)
                self.align_duration = self.align.duration(1., .10, .30)
                self.elapsed, self.phase = 0., 'aligning'
                dt = 0.
        elif self.active and problem:
            self.fail(problem, suppress=True)
        if self.active and (dt < 0 or dt > .25) and self.phase != 'arming':
            self.fail('hardware timer gap/reset; last target held')
        if self.suppress or not self.active:
            return None
        if self.target is not None and self.feedback.q is not None:
            self.error = float(np.max(np.abs(self.feedback.q-self.target)))
            if self.error > self.limits.tracking_fault:
                self.fail('excessive servo tracking error; last target held')
        if self.fault:
            return None if self.target is None else self.target.copy()
        self.paused = self.error > self.limits.tracking_pause
        if self.paused:
            self.reason = 'waiting for servos to catch up'
            if self.lag_since is None:
                self.lag_since = now
            if now-self.lag_since > self.limits.tracking_timeout:
                self.fail('servo tracking lag did not clear; last target held')
            self.walker.last_time = now
            return self.target.copy()
        self.lag_since = None
        dt = min(max(dt, 0.), .04)
        if self.phase == 'aligning':
            self.elapsed += dt
            self.target = self.align.sample(self.elapsed/self.align_duration)
            if self.elapsed >= self.align_duration:
                self.phase, self.elapsed, self.settled_at = 'settling', 0., None
        elif self.phase == 'settling':
            self.elapsed += dt
            v = self.feedback.velocity
            if self.error <= self.limits.settle_tolerance and v is not None and np.max(np.abs(v)) <= self.limits.stationary_speed:
                if self.settled_at is None:
                    self.settled_at = now
                if now-self.settled_at >= self.limits.settle_duration:
                    self.walker.command(self.pending, now)
                    self.phase = None
            else:
                self.settled_at = None
            if self.elapsed > self.limits.settle_timeout:
                self.fail('servos did not settle at the known starting stance')
        else:
            # Encoder lag pauses the clock, not individual joints or targets.
            output = self.walker.tick(now)
            if self.walker.state == 'fault':
                self.fail(self.walker.fault, self.walker.suppress)
            if output is not None:
                self.target = output.copy()
        self.reason = self.phase or self.walker.state
        return None if self.suppress or self.target is None else self.target.copy()

    def status(self, now):
        result = self.walker.status()
        result.update(state='fault' if self.fault else self.phase or self.walker.state,
                      active=self.active, fault=self.fault or self.walker.fault,
                      hardware_ready=not bool(self.feedback.problem(now)),
                      readiness=self.feedback.problem(now), tracking_paused=self.paused,
                      max_tracking_error_rad=self.error, reason=self.reason)
        return result
