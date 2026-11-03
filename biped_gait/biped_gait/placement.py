"""Bent-leg sole geometry and bounded placement. No contact-force measurement."""
from dataclasses import dataclass
import numpy as np
from . import quasi_static_walk as m
from .imu import rpy_matrix

NAMES = m.JOINT_ORDER


@dataclass
class GroundEstimate:
    rotation: np.ndarray
    plane_z: float
    body_height: float
    clearance: float
    swing: str
    stance: str


def ground_estimate(measured, angles, stance):
    """Use the supporting sole as a local plane, including ALL measured joints.

    The body origin is arbitrary: z=0 at the body, and negative z below it.
    Both feet are rotated into gravity axes before their lowest corners compare.
    Flat supporting-foot contact and a correct kinematic model remain assumptions.
    """
    rotation = rpy_matrix(*angles)
    _, feet = m._com_and_feet(dict(zip(NAMES, measured)))
    swing = 'r' if stance == 'l' else 'l'
    plane = float((feet[stance] @ rotation.T)[:, 2].min())
    clearance = float((feet[swing] @ rotation.T)[:, 2].min()-plane)
    return GroundEstimate(rotation, plane, -plane, clearance, swing, stance)


class PhaseGeometry:
    """Continue a cached body motion from the actual landing stance.

    Foot anchors remain fixed through double support. Root pose offsets return
    smoothly to the planned body pose; no joint-space blend stretches one foot
    back to an obsolete floor target after placement.
    """
    def __init__(self, reference, anchor, stance):
        self.stance = stance
        self.reference = np.array(reference).copy()
        self.anchor = np.array(anchor).copy()
        self.changed = not np.allclose(reference, anchor, atol=1e-8, rtol=0)
        self.r0, self.p0, self.f0, _ = self.frame(reference)
        self.ra, self.pa, self.fa, self.foot_rotations = self.frame(anchor)
        self.offset = self.ra @ self.r0.T
        self.angle = float(np.arccos(np.clip((np.trace(self.offset)-1)/2, -1., 1.)))
        if self.angle > .2 or np.linalg.norm(self.pa-self.p0) > .025:
            raise ValueError('landing stance is outside the body-path retargeting range')

    def frame(self, row):
        q = dict(zip(NAMES, row))
        rotation = m._foot_transform(q, self.stance)[0].T
        origin = m._sole_center(q, self.stance)
        position = -rotation @ origin
        feet = {s: rotation @ (m._sole_center(q,s)-origin) for s in 'rl'}
        rotations = {s: rotation @ m._foot_transform(q,s)[0] for s in 'rl'}
        return rotation, position, feet, rotations

    def pose(self, row, seed, fraction):
        if not self.changed: return np.array(row).copy()
        rotation, position, feet, _ = self.frame(row)
        decay = 1-m._smoothstep(float(np.clip(fraction, 0., 1.)))
        if self.angle > 1e-8:
            axis = np.array([self.offset[2,1]-self.offset[1,2], self.offset[0,2]-self.offset[2,0],
                             self.offset[1,0]-self.offset[0,1]])/(2*np.sin(self.angle))
            rotation = m._rot(axis, self.angle*decay) @ rotation
        position += decay*(self.pa-self.p0)
        q = dict(zip(NAMES, seed))
        for side in 'rl':
            target = self.fa[side]+feet[side]-self.f0[side]
            q = m._leg_pose_ik(q, side, rotation.T @ (target-position),
                               rotation.T @ self.foot_rotations[side])
        return np.array([q[n] for n in NAMES])


class Placement:
    def __init__(self, measured, angles, stance, settings, reference_goal=None):
        self.s = settings
        g = ground_estimate(measured, angles, stance)
        self.stance = stance
        self.initial_clearance = g.clearance
        self.remaining = max(0., g.clearance-self.s.landing_clearance)
        self.commanded_descent = 0.
        self.last = g
        self.touch_target = None
        self.aligning = reference_goal is not None
        self.alignment_height = max(g.clearance, settings.landing_alignment_clearance)
        reference = dict(zip(NAMES, measured if reference_goal is None else reference_goal))
        rotation = m._foot_transform(reference, stance)[0].T
        self.xy_offset = (rotation @ (m._sole_center(reference,g.swing)-m._sole_center(reference,stance)))[:2]
        self.xy_error = 0.
        self.motion_time = 0.
        self.trial_dt = 0.
        self.capture_elapsed = 0.
        self.capture_duration = 0.
        self.capture_start = None
        self.capture_ground = None

    def _capture_pose(self, dt):
        u = min(1., (self.capture_elapsed+dt)/self.capture_duration)
        pose = self.capture_start+m._smoothstep(u)*(self.touch_target-self.capture_start)
        return self.above_plane(pose,self.capture_ground), 0., u >= 1., False

    def ramp_gain(self, dt):
        return m._smoothstep(min(1.,(self.motion_time+dt)/self.s.placement_ramp_time))

    def above_plane(self, candidate, ground=None):
        """Keep interpolation above its specified supporting-foot plane.

        Capture freezes that plane together with the measured pose; repeatedly
        chasing a changing loaded-leg measurement would recreate extension.
        """
        g = self.last if ground is None else ground
        q = dict(zip(NAMES,candidate))
        _, feet = m._com_and_feet(q)
        height = float((feet[g.swing] @ g.rotation.T)[:,2].min()-g.plane_z)
        if height >= self.s.landing_clearance-1e-7:
            return np.asarray(candidate).copy()
        position = m._sole_center(q,g.swing)+g.rotation.T @ np.array([0.,0.,self.s.landing_clearance-height])
        q = m._leg_pose_ik(q,g.swing,position,m._foot_transform(q,g.swing)[0])
        target = np.array([q[n] for n in NAMES])
        if -target[NAMES.index(g.swing+'_knee_joint')] < self.s.minimum_knee_bend:
            raise ValueError('floor projection would straighten the swing knee too far')
        return target

    def target(self, commanded, measured, angles, dt):
        """Lower at most speed*dt from the CURRENT command toward measured ground.

        No straight-leg length or fixed torso height enters this calculation.
        A finite entry-height budget prevents an indefinite downward search.
        """
        g = self.last = ground_estimate(measured, angles, self.stance)
        self.trial_dt = dt
        q = dict(zip(NAMES, commanded))
        actual = dict(zip(NAMES, measured))
        stance_position = g.rotation @ m._sole_center(actual,g.stance)
        stance_rotation = g.rotation @ m._foot_transform(actual,g.stance)[0]
        yaw = np.arctan2(stance_rotation[1,0],stance_rotation[0,0])
        ground_rotation = m._rot((0.,0.,1.),yaw)
        xy_rotation = np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        xy_goal = stance_position[:2]+xy_rotation @ self.xy_offset
        actual_swing = g.rotation @ m._sole_center(actual,g.swing)
        self.xy_error = float(np.linalg.norm(actual_swing[:2]-xy_goal))
        if self.aligning and self.xy_error <= self.s.landing_xy_tolerance and g.clearance >= self.alignment_height-.001:
            self.aligning = False
            self.remaining = max(0., g.clearance-self.s.landing_clearance)
            self.descent_start_clearance = g.clearance
        if self.touch_target is not None:
            return self._capture_pose(dt)
        if g.clearance <= self.s.landing_clearance+self.s.placement_tolerance:
            if self.xy_error > self.s.landing_xy_tolerance:
                raise ValueError('foot reached the estimated floor before horizontal alignment')
            self.aligning = False
            # The measured bent leg has already reached the estimated floor.
            # Capture its angles; do not finish a remaining nominal extension.
            actual = dict(zip(NAMES, measured))
            # Capture both legs so the next body path starts from a coherent
            # measured bent-knee stance instead of an old support-leg preload.
            q = actual
            positions = {s: g.rotation @ m._sole_center(q,s) for s in 'rl'}
            for side in 'rl':
                positions[side][2] = g.plane_z+(max(g.clearance, self.s.landing_clearance) if side == g.swing else 0.)
                q = m._leg_pose_ik(q, side, g.rotation.T @ positions[side], g.rotation.T @ ground_rotation)
            target = np.array([q[n] for n in NAMES])
            if -target[NAMES.index(g.swing+'_knee_joint')] < self.s.minimum_knee_bend:
                raise ValueError('measured placement knee is too straight')
            self.touch_target = target.copy()
            self.capture_start = np.array(commanded).copy()
            self.capture_ground = g
            distance = float(np.max(np.abs(target-commanded)))
            self.capture_duration = max(.4, 1.875*distance/self.s.max_joint_speed,
                                        np.sqrt(5.774*distance/self.s.placement_capture_acceleration))
            return self._capture_pose(dt)
        _, feet = m._com_and_feet(q)
        current = float((feet[g.swing] @ g.rotation.T)[:, 2].min()-g.plane_z)
        if current < -self.s.penetration_tolerance:
            raise ValueError('existing swing target is below the measured support plane')
        # Enter the online alignment without a full-speed Cartesian step. The
        # ramp advances only after the whole candidate is accepted by the caller.
        gain = self.ramp_gain(dt)
        motion_dt = dt*gain
        if self.aligning:
            descent = 0.
            next_clearance = max(self.s.landing_clearance, current+float(np.clip(self.alignment_height-current,
                -self.s.landing_speed*motion_dt,self.s.landing_speed*motion_dt)))
        else:
            descent = min(self.s.landing_speed*motion_dt, self.remaining,
                          max(0., current-self.s.landing_clearance))
            next_clearance = max(self.s.landing_clearance, current-descent)
        position = g.rotation @ m._sole_center(q, g.swing)
        delta_xy = xy_goal-position[:2]
        position[:2] += delta_xy*min(1.,self.s.landing_xy_speed*motion_dt/max(float(np.linalg.norm(delta_xy)),1e-9))
        # Approach level gradually. Requiring the entire measured tilt error to
        # disappear in one tick would defeat the joint speed limit.
        world_rotation = g.rotation @ m._foot_transform(q, g.swing)[0]
        difference = ground_rotation @ world_rotation.T
        angle = float(np.arccos(np.clip((np.trace(difference)-1)/2, -1., 1.)))
        if angle > .35:
            raise ValueError('swing sole is too tilted for placement')
        if angle > 1e-8:
            axis = np.array([difference[2,1]-difference[1,2], difference[0,2]-difference[2,0],
                             difference[1,0]-difference[0,1]])/(2*np.sin(angle))
            world_rotation = m._rot(axis, min(angle, self.s.max_joint_speed*motion_dt*.25)) @ world_rotation
        poly = np.asarray(m.SOLE[g.swing]['poly'])
        offsets = np.column_stack((poly-poly.mean(axis=0), np.zeros(len(poly))))
        corner_offset = float((offsets @ world_rotation.T)[:, 2].min())
        position[2] = g.plane_z+next_clearance-corner_offset
        result = m._leg_pose_ik(q, g.swing, g.rotation.T @ position, g.rotation.T @ world_rotation)
        target = np.array([result[n] for n in NAMES])
        if -target[NAMES.index(g.swing+'_knee_joint')] < self.s.minimum_knee_bend:
            raise ValueError('placement would straighten the swing knee too far')
        _, corners = m._com_and_feet(result)
        predicted = float((corners[g.swing] @ g.rotation.T)[:, 2].min()-g.plane_z)
        if predicted < self.s.landing_clearance-1e-6:
            raise ValueError('placement target crosses the estimated floor')
        done = (next_clearance <= self.s.landing_clearance+1e-6 and
                g.clearance <= self.s.landing_clearance+self.s.placement_tolerance)
        exhausted = not self.aligning and self.remaining <= 1e-7 and not done
        return target, descent, done, exhausted

    def accept(self, descent):
        self.motion_time += self.trial_dt
        if self.touch_target is not None:
            self.capture_elapsed = min(self.capture_duration,self.capture_elapsed+self.trial_dt)
        self.remaining = max(0., self.remaining-descent)
        self.commanded_descent += descent

    def relief(self, commanded, measured, angles):
        """Remove swing-servo tracking pressure and request only a 1 mm lift.

        The stance leg retains its load-bearing target. The caller rate-limits
        the full transition and keeps posture feedback active while waiting.
        """
        g = ground_estimate(measured, angles, self.stance)
        q = dict(zip(NAMES, commanded))
        actual = dict(zip(NAMES, measured))
        for joint in m.LEG[g.swing]:
            q[joint['name']] = actual[joint['name']]
        position = m._sole_center(q, g.swing)+g.rotation.T @ np.array([0., 0., .001])
        rotation = m._foot_transform(q, g.swing)[0]
        q = m._leg_pose_ik(q, g.swing, position, rotation)
        return np.array([q[n] for n in NAMES])
