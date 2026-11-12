"""Precomputed quasi-static paths and a timed position loop; no sensor inputs."""
from dataclasses import dataclass, asdict
import math
import numpy as np
from . import quasi_static_walk as m

NAMES = m.JOINT_ORDER


@dataclass(frozen=True)
class Settings:
    step_length: float = .030
    stance_width: float = .090
    body_roll: float = .040
    crouch: float = .021
    centre_crouch: float = .018
    com_inset: float = .010
    support_margin: float = .008
    clearance: float = .010
    lift_clearance: float = .010
    max_joint_speed: float = .18
    landing_speed: float = .0015

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('walking settings must be finite and positive')
        if not self.centre_crouch <= self.crouch or not self.lift_clearance <= self.clearance:
            raise ValueError('invalid body height or lift settings')

    def profile(self):
        values = asdict(self)
        del values['max_joint_speed'], values['landing_speed']
        return dict(values, com_x_midpoint=True)


class Plan:
    """Solve and check the complete path offline; runtime only reads the tables."""
    def __init__(self, direction, initial, settings):
        self.initial = np.asarray(initial).copy()
        self.profile = settings.profile()
        self.keyframes, self.tables = m.build_tables(direction, self.initial, self.profile)
        if not m.report(direction, self.keyframes, self.tables, lambda message: None, self.profile):
            raise ValueError('walking geometry failed the offline path check')
        self.metrics = m.gait_metrics(self.keyframes, self.tables, self.profile)
        self.durations = {}
        for segment, table in self.tables.items():
            speed, _ = m._motion_bounds(table)
            duration = max(m.segment_duration(segment[1], table), speed/settings.max_joint_speed)
            if segment[1].startswith('place_'):
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


class WalkingController:
    """Play joint tables at fixed phase durations and repeat the walking cycle.

    No joint tracking, bridge status, IMU or contact checks affect this loop.
    All foot heights, knee angles and landing endpoints were solved offline.
    """
    def __init__(self, plans=None):
        self.plans = plans or Plans(precomputed=True)
        self.s = self.plans.settings
        self.plan = self.plans.cache['forward']
        self.direction = 'forward'
        self.nominal = np.zeros(12)
        self.commanded = None
        self.state = 'idle'
        self.segment = None
        self.elapsed = self.waited = 0.
        self.last_time = None
        self.stop_requested = self.single_step = self.stand_only = False
        self.landings = 0
        self.resume_state = None

    def command(self, text, now):
        text = text.lower().strip()
        if text == 'status':
            return
        if text == 'hold':
            if self.state in ('moving', 'dwelling'):
                self.resume_state, self.state = self.state, 'holding'
            return
        if text == 'stop':
            self.stop_requested = True
            if self.state == 'holding':
                self.state = self.resume_state
                self.last_time = now
            return
        if text not in ('start', 'forward', 'backward', 'step', 'stand'):
            raise ValueError('use start / forward / backward / step / stand / stop / hold / status')
        if self.state == 'holding' and text in ('start', self.direction):
            self.state = self.resume_state
            self.last_time = now
            return
        if self.state not in ('idle', 'standing'):
            raise ValueError('already walking; stop before starting a different motion')
        if self.state == 'standing' and text == 'stand':
            return
        self.direction = 'backward' if text == 'backward' else 'forward'
        self.plan = self.plans.get(self.direction, self.nominal)
        self.stop_requested = False
        self.single_step, self.stand_only = text == 'step', text == 'stand'
        self.landings = 0
        self.last_time = now
        self.commanded = self.nominal.copy()
        self._begin('neutral', 'ready')

    def _begin(self, source, destination):
        self.segment = (source, destination)
        self.state = 'moving'
        self.elapsed = self.waited = 0.

    def _advance(self):
        arrived = self.segment[1]
        if arrived == 'neutral' or (arrived == 'ready' and self.stand_only):
            self.state, self.segment = 'standing', None
            return
        stopping = self.stop_requested or (self.single_step and self.landings > 0)
        following = 'neutral' if stopping and arrived in m.SAFE_STOP else m.NEXT[arrived]
        self._begin(arrived, following)

    def tick(self, now):
        dt = 0. if self.last_time is None else max(0., now-self.last_time)
        self.last_time = now
        if self.state == 'idle':
            return None
        if self.state == 'moving':
            duration = self.plan.durations[self.segment]
            self.elapsed = min(duration, self.elapsed+dt)
            self.nominal = m.table_pose(self.plan.tables[self.segment], self.elapsed/duration)
            self.commanded = self.nominal.copy()
            if self.elapsed >= duration:
                self.state, self.waited = 'dwelling', 0.
                if self.segment[1].startswith('place_'):
                    self.landings += 1
        elif self.state == 'dwelling':
            self.waited += dt
            dwell = m.DWELL_TIME.get(self.segment[1], .4)*m.TIME_SCALE
            if self.waited >= dwell:
                self._advance()
        return self.commanded.copy()

    def status(self, now=None):
        return dict(state=self.state, segment=self.segment, direction=self.direction,
                    landings=self.landings, step_length_m=self.s.step_length,
                    foot_lift_m=self.s.lift_clearance,
                    maximum_planned_roll_deg=math.degrees(self.s.body_roll),
                    control='timed position loop', contact_measured=False)
