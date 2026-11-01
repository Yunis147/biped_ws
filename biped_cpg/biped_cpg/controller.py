"""Clock-driven walking state machine; it never reads robot/simulator sensors."""
import math

import numpy as np

from .oscillator import CoupledCPG
from .planner import CompiledGait, Settings


class Library:
    def __init__(self, settings=None):
        self.settings = settings or Settings()
        self.plans = {direction: CompiledGait(direction, self.settings) for direction in ('forward', 'backward')}

    def get(self, direction, initial):
        plan = self.plans[direction]
        if not np.allclose(plan.initial, initial, atol=1e-10, rtol=0):
            plan = CompiledGait(direction, self.settings, initial)
            self.plans[direction] = plan
        return plan


class Walker:
    rate_hz = 50.

    def __init__(self, library):
        self.library = library
        self.state = 'idle'
        self.active = False
        self.direction = 'forward'
        self.last_pose = np.zeros(12)
        self.plan = library.plans['forward']
        self.cpg = None
        self.fault = ''
        self.suppress = False
        self.stop_at = None
        self.stop_requested = self.single_step = False
        self.elapsed = 0.
        self.last_time = None
        self.resync = False
        self.landings = 0
        self.stop_side = None

    def fail(self, reason, suppress=False):
        self.state, self.fault, self.suppress = 'fault', reason, suppress

    def command(self, command, now):
        command = command.strip().lower()
        if command == 'status':
            return 'status'
        if command == 'hold':
            if self.active:
                self.fail('hold requested: keeping the last target')
            return 'holding'
        if self.state == 'fault':
            raise ValueError(f'fault: {self.fault}; restart the node and reset the simulation')
        if command == 'stop':
            if self.state == 'preparing':
                self.stop_requested = True
            elif self.state == 'walking':
                self.stop_at = (math.floor(self.cpg.right / math.pi + 1e-10)+1)*math.pi
            return 'stop requested: finish landing, then centre'
        if command not in ('start', 'forward', 'backward', 'step'):
            raise ValueError('use start / forward / backward / step / stop / hold / status')
        direction = 'backward' if command == 'backward' else 'forward'
        if self.state != 'idle':
            raise ValueError('already moving; stop and wait for idle before starting another command')
        # Validate/rebuild before changing state or publishing any new target.
        plan = self.library.get(direction, self.last_pose)
        self.plan, self.direction = plan, direction
        self.state, self.active = 'preparing', True
        self.elapsed, self.last_time = 0., now
        self.resync = True
        self.stop_requested, self.single_step = False, command == 'step'
        self.stop_at = self.stop_side = None
        self.cpg = None
        self.landings = 0
        return f'{direction}: preparing; CPG period {plan.period:.2f} s'

    def tick(self, now):
        if not math.isfinite(now):
            self.fail('nonfinite clock', suppress=True)
        if self.last_time is None:
            self.last_time = now
        dt = now - self.last_time
        self.last_time = now
        if self.active and dt < -1e-8:
            self.fail('simulation clock moved backwards; restart after resetting simulation', suppress=True)
        if self.resync and dt >= 0 and not self.suppress and self.state != 'fault':
            # A restart may compile a new stance while the separate simulator
            # advances. Synchronize on its first fresh clock; hold the old pose.
            if dt > 0:
                self.resync = False
            return self.last_pose.copy() if self.active else None
        if self.active and dt > .25:
            self.fail('clock/timer gap exceeded 0.25 s; target held')
        if not self.active or self.suppress:
            return None
        if self.state in ('idle', 'fault'):
            return self.last_pose.copy()
        # A late callback slows the rhythm instead of jumping along the path.
        dt = min(max(dt, 0.), 2/self.rate_hz)
        if self.state == 'preparing':
            self.elapsed += dt
            self.last_pose = self.plan.startup.curve.sample(self.elapsed / self.plan.startup_duration)
            if self.elapsed >= self.plan.startup_duration:
                self.elapsed = 0.
                if self.stop_requested:
                    self.state = 'idle'
                else:
                    self.cpg = CoupledCPG(self.plan.period)
                    self.stop_at = math.pi if self.single_step else None
                    self.state = 'walking'
        elif self.state == 'walking':
            previous = self.cpg.right
            phase = self.cpg.step(dt)
            self.landings += max(0, math.floor((phase+1e-10)/math.pi)-math.floor((previous+1e-10)/math.pi))
            if self.stop_at is not None and phase >= self.stop_at-1e-10:
                phase = self.stop_at
                self.stop_side = 'right' if round(phase / math.pi) % 2 else 'left'
                self.last_pose = self.plan.stops[self.stop_side].curve.rows[0].copy()
                self.state, self.elapsed = 'centering', 0.
            else:
                self.last_pose = self.plan.pose(phase)
        elif self.state == 'centering':
            self.elapsed += dt
            motion = self.plan.stops[self.stop_side]
            duration = self.plan.stop_durations[self.stop_side]
            self.last_pose = motion.curve.sample(self.elapsed / duration)
            if self.elapsed >= duration:
                self.state = 'idle'
                self.stop_at = None
                self.stop_requested = self.single_step = False
        return self.last_pose.copy()

    def status(self):
        result = dict(state=self.state, active=self.active, fault=self.fault, direction=self.direction,
                      landings=self.landings, stop_pending=self.state not in ('idle', 'fault') and (self.stop_at is not None or self.stop_requested),
                      phase_right=None if self.cpg is None else self.cpg.right % (2*math.pi),
                      phase_left=None if self.cpg is None else self.cpg.left % (2*math.pi))
        result.update(self.plan.summary())
        return result
