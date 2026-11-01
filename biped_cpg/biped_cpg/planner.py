"""URDF-based small-lean trajectories, indexed by the CPG's phase.

Only pure geometry/IK helpers are imported from biped_gait. Its constants and
functions are never patched, and its walker node is never instantiated.
"""
from dataclasses import dataclass, asdict
import math

import numpy as np
from biped_gait import quasi_static_walk as model

from .curves import Curve

NAMES = tuple(model.JOINT_ORDER)


@dataclass(frozen=True)
class Settings:
    body_roll: float = .08
    crouch: float = .028
    centre_crouch: float = .020
    com_inset: float = .008
    forward_step: float = .020
    backward_step: float = .016
    clearance: float = .010
    double_support: float = .6
    stride_progress: float = .5
    requested_period: float = 12.
    max_joint_speed: float = .35
    max_joint_acceleration: float = .6
    minimum_margin: float = .010
    half_samples: int = 120

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        ranges = dict(body_roll=(0., .12), crouch=(.015, .04), centre_crouch=(.01, .03),
                      com_inset=(0., .012), forward_step=(.005, .03), backward_step=(.005, .025),
                      clearance=(.004, .012), double_support=(.25, .6), stride_progress=(0., .5), requested_period=(4., 120.),
                      max_joint_speed=(.05, .35), max_joint_acceleration=(.1, 1.),
                      minimum_margin=(.005, .018))
        for name, (low, high) in ranges.items():
            if not low <= getattr(self, name) <= high:
                raise ValueError(f'{name} must be between {low} and {high}')
        if self.centre_crouch > self.crouch:
            raise ValueError('centre crouch cannot exceed single-support crouch')
        if not isinstance(self.half_samples, int) or not 80 <= self.half_samples <= 400:
            raise ValueError('half_samples must be an integer between 80 and 400')
        index = self.half_samples * self.double_support
        if abs(index - round(index)) > 1e-8:
            raise ValueError('double-support boundary must lie on a table sample')


@dataclass
class Frame:
    feet: dict
    com: np.ndarray
    roll: float
    height: float
    support: str | None = None

    def translated(self, offset):
        offset = np.asarray(offset)
        return Frame({s: f + offset for s, f in self.feet.items()}, self.com + offset[:2],
                     self.roll, self.height + offset[2], self.support)


def smooth(u):
    u = min(max(float(u), 0.), 1.)
    return u**3 * (10 + u*(-15 + 6*u))


def centered(frame, settings):
    return Frame({s: f.copy() for s, f in frame.feet.items()},
                 (frame.feet['r'][:2] + frame.feet['l'][:2]) * .5,
                 0., model.ZERO_HEIGHT - settings.centre_crouch)


def blend(start, end):
    def at(fraction):
        u = smooth(fraction)
        return Frame({s: start.feet[s] + u*(end.feet[s]-start.feet[s]) for s in 'rl'},
                     start.com + u*(end.com-start.com), start.roll + u*(end.roll-start.roll),
                     start.height + u*(end.height-start.height))
    return at


def half_step(start, support, step, settings, entry=False):
    swing = 'l' if support == 'r' else 'r'
    sign = 1 if support == 'l' else -1
    target_com = start.feet[support][:2].copy()
    target_com[1] -= sign * settings.com_inset
    target_com[0] -= 0. if entry else settings.stride_progress * step
    roll = -sign * settings.body_roll
    height = model.ZERO_HEIGHT - settings.crouch
    landing_x = start.feet[support][0] + step

    def at(fraction):
        u = min(max(float(fraction), 0.), 1.)
        feet = {s: f.copy() for s, f in start.feet.items()}
        if u < settings.double_support:
            t = smooth(u / settings.double_support)
            z = start.height + t*(height-start.height)
            if start.roll * roll < 0:
                z += (settings.crouch-settings.centre_crouch) * 16*t*t*(1-t)*(1-t)
            return Frame(feet, start.com + t*(target_com-start.com),
                         start.roll + t*(roll-start.roll), z)
        t = (u-settings.double_support) / (1-settings.double_support)
        feet[swing][0] += smooth(t) * (landing_x-feet[swing][0])
        feet[swing][2] = settings.clearance * math.sin(math.pi*t)**2
        com = target_com.copy()
        endpoint = start.feet[support][0] + settings.stride_progress * step
        com[0] += smooth(t) * (endpoint-com[0])
        return Frame(feet, com, roll, height, support)
    return at


class Motion:
    def __init__(self, name, frame_at, start_q, start_body, samples, stops=()):
        self.name, self.frame = name, frame_at
        q = dict(zip(NAMES, start_q))
        body = np.asarray(start_body, dtype=float).copy()
        rows, bodies = [np.array(start_q).copy()], [body.copy()]
        for index in range(1, samples+1):
            frame = frame_at(index / samples)
            body[2] = frame.height
            try:
                q, body = model._balanced_pose(frame.feet, frame.com, body, q, frame.roll)
            except (ValueError, np.linalg.LinAlgError) as error:
                raise ValueError(f'{name}, sample {index}/{samples}: {error}') from error
            rows.append(np.array([q[n] for n in NAMES]))
            bodies.append(body.copy())
        self.bodies = np.array(bodies)
        self.curve = Curve(rows, stops)

    def check(self, settings):
        margin = math.inf
        foot_error = floor_error = com_error = height_error = rotation_error = 0.
        min_knee = math.inf
        peak_roll = 0.
        for fraction in np.linspace(0, 1, (len(self.curve.rows)-1)*4+1):
            q_array = self.curve.sample(fraction)
            if np.any(q_array < model.LO+model.LIMIT_MARGIN-1e-9) or np.any(q_array > model.HI-model.LIMIT_MARGIN+1e-9):
                raise ValueError(f'{self.name}: interpolated joint limit exceeded')
            q = dict(zip(NAMES, q_array))
            frame = self.frame(fraction)
            rotation = model._rot((1, 0, 0), frame.roll)
            com, local_feet = model._com_and_feet(q)
            reference = frame.support or 'l'
            body = frame.feet[reference] - rotation @ local_feet[reference].mean(axis=0)
            feet = {s: vertices @ rotation.T + body for s, vertices in local_feet.items()}
            com = rotation @ com + body
            points = np.concatenate([feet[s] for s in ([frame.support] if frame.support else 'rl')])
            margin = min(margin, model._margin(model._hull(points[:, :2]), com[:2]))
            floor_error = max(floor_error, float(np.max(np.abs(points[:, 2]))),
                              max(0., -min(float(f[:, 2].min()) for f in feet.values())))
            com_error = max(com_error, float(np.linalg.norm(com[:2]-frame.com)))
            height_error = max(height_error, abs(float(body[2])-frame.height))
            for side in 'rl':
                foot_error = max(foot_error, float(np.linalg.norm(feet[side].mean(axis=0)-frame.feet[side])))
                relative_rotation, _ = model._foot_transform(q, side)
                rotation_error = max(rotation_error, float(np.linalg.norm(rotation @ relative_rotation-np.eye(3))))
                actual_body = relative_rotation.T
                peak_roll = max(peak_roll, abs(math.atan2(actual_body[2, 1], actual_body[2, 2])))
                min_knee = min(min_knee, -q[f'{side}_knee_joint'])
        if (margin < settings.minimum_margin or max(foot_error, floor_error, com_error, height_error) > .0002
                or rotation_error > .002 or peak_roll > settings.body_roll + .001
                or (self.name != 'startup' and min_knee < .12)):
            raise ValueError(f'{self.name}: margin={margin*1000:.3f} mm, foot={foot_error*1000:.4f} mm, '
                             f'floor={floor_error*1000:.4f} mm, CoM={com_error*1000:.4f} mm, '
                             f'min knee={math.degrees(min_knee):.2f} deg, lean={math.degrees(peak_roll):.3f} deg')
        return dict(margin_mm=margin*1000, foot_error_mm=foot_error*1000, floor_error_mm=floor_error*1000,
                    com_error_mm=com_error*1000, maximum_roll_deg=math.degrees(peak_roll))


class CompiledGait:
    def __init__(self, direction='forward', settings=None, initial=None):
        self.settings = cfg = settings or Settings()
        if direction not in ('forward', 'backward'):
            raise ValueError('direction must be forward or backward')
        self.direction = direction
        self.initial = np.zeros(12) if initial is None else np.asarray(initial, dtype=float).copy()
        if self.initial.shape != (12,) or not np.all(np.isfinite(self.initial)):
            raise ValueError('starting pose must contain 12 finite joint angles')
        if np.any(self.initial < model.LO+model.LIMIT_MARGIN) or np.any(self.initial > model.HI-model.LIMIT_MARGIN):
            raise ValueError('starting pose exceeds joint limits')
        q = dict(zip(NAMES, self.initial))
        feet = {s: model._sole_center(q, s) for s in 'rl'}
        for side in 'rl':
            if np.linalg.norm(model._foot_transform(q, side)[0]-np.eye(3)) > 1e-4:
                raise ValueError('start requires upright, flat, forward-facing feet')
        if abs(feet['r'][2]-feet['l'][2]) > 1e-5:
            raise ValueError('starting feet must share a floor plane')
        body = np.array([0., 0., -np.mean([f[2] for f in feet.values()])])
        com, _ = model._com_and_feet(q)
        start = Frame({s: f+body for s, f in feet.items()}, com[:2], 0., body[2])
        ready = centered(start, cfg)
        n, ds = cfg.half_samples, round(cfg.half_samples*cfg.double_support)
        self.startup = Motion('startup', blend(start, ready), self.initial, body, n)
        step = cfg.forward_step if direction == 'forward' else -cfg.backward_step
        entry_at = half_step(ready, 'l', step, cfg, entry=True)
        self.entry = Motion('entry', entry_at, self.startup.curve.rows[-1], self.startup.bodies[-1], n, (ds,))
        landed_right = entry_at(1.)
        second_at = half_step(landed_right, 'r', step, cfg)
        second = Motion('second half', second_at, self.entry.curve.rows[-1], self.entry.bodies[-1], n, (ds,))
        offset = np.array([-2*step, 0., 0.])
        landed_left = second_at(1.).translated(offset)
        first_at = half_step(landed_left, 'l', step, cfg)
        first = Motion('first half', first_at, second.curve.rows[-1], second.bodies[-1]+offset, n, (ds,))
        # All shared phase boundaries use a single canonical pose.
        first.curve.rows[-1] = self.entry.curve.rows[-1]
        rows = np.concatenate((first.curve.rows[:-1], second.curve.rows))
        rows[-1] = rows[0]
        self.cycle = Motion.__new__(Motion)
        self.cycle.name = 'cycle'
        self.cycle.frame = lambda fraction: first_at(2*fraction) if fraction <= .5 else second_at(2*fraction-1)
        self.cycle.curve = Curve(rows, (ds, n, n+ds))
        self.cycle.bodies = np.concatenate((first.bodies[:-1], second.bodies))
        self.stops = {
            'left': Motion('stop after left landing', blend(landed_left, centered(landed_left, cfg)),
                           rows[0], first.bodies[0], n),
            'right': Motion('stop after right landing', blend(landed_right, centered(landed_right, cfg)),
                            self.entry.curve.rows[-1], self.entry.bodies[-1], n),
        }
        # One period covers two steps. Check the special first step too.
        self.period = max(self.cycle.curve.duration(cfg.requested_period, cfg.max_joint_speed, cfg.max_joint_acceleration),
                          2*self.entry.curve.duration(0., cfg.max_joint_speed, cfg.max_joint_acceleration))
        self.startup_duration = self.startup.curve.duration(6., cfg.max_joint_speed, cfg.max_joint_acceleration)
        self.stop_durations = {side: motion.curve.duration(4., cfg.max_joint_speed, cfg.max_joint_acceleration)
                               for side, motion in self.stops.items()}
        self.metrics = {motion.name: motion.check(cfg) for motion in
                        (self.startup, self.entry, self.cycle, *self.stops.values())}

    def pose(self, phase):
        if not math.isfinite(phase) or phase < 0:
            raise ValueError('CPG phase must be finite and nonnegative')
        if phase < math.pi:
            return self.entry.curve.sample(phase / math.pi)
        return self.cycle.curve.sample((phase % (2*math.pi)) / (2*math.pi))

    def summary(self):
        return dict(direction=self.direction, mass_kg=model.TOTAL_MASS, model_path=str(model.MODEL_PATH),
                    body_roll_deg=math.degrees(self.settings.body_roll),
                    minimum_margin_mm=min(m['margin_mm'] for m in self.metrics.values()),
                    period_s=self.period, startup_s=self.startup_duration,
                    step_mm=(self.settings.forward_step if self.direction == 'forward' else self.settings.backward_step)*1000,
                    clearance_mm=self.settings.clearance*1000)
