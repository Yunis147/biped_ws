"""Read-only bridge feedback adapter; no serial access and no torque operations."""
from types import SimpleNamespace
import math
import numpy as np
from . import quasi_static_walk as model
NAMES = model.JOINT_ORDER

def FeedbackLimits():
    return SimpleNamespace(joint_timeout=.25, diagnostic_timeout=2.5)

class ServoFeedback:
    def __init__(self, limits=None):
        self.limits = limits or FeedbackLimits()
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
                index = int(np.flatnonzero((q < model.LO) | (q > model.HI))[0])
                raise ValueError(f'measured joint outside URDF limits: {NAMES[index]}={q[index]:.6f} rad, '
                    f'allowed [{model.LO[index]:.6f}, {model.HI[index]:.6f}]; check calibration')
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

