"""Task-space attitude corrections with the original gait's read-only robot model."""
import math

import numpy as np

from biped_gait import quasi_static_walk as model

from .estimation import rpy_matrix

NAMES = model.JOINT_ORDER
INDEX = {name: i for i, name in enumerate(NAMES)}
LOWER, UPPER = model.LO + model.LIMIT_MARGIN, model.HI - model.LIMIT_MARGIN


def leg_pose_jacobian(q, side):
    rotation, position = np.eye(3), np.zeros(3)
    origins, axes = [], []
    for joint in model.LEG[side]:
        position += rotation @ joint['xyz']
        origins.append(position.copy())
        axes.append(rotation @ np.asarray(joint['axis']))
        rotation = rotation @ model._rot(joint['axis'], q[INDEX[joint['name']]])
    jacobian = np.vstack((np.column_stack([np.cross(a, position-o) for a, o in zip(axes, origins)]),
                          np.column_stack(axes)))
    return rotation, position, jacobian


def _rotation_error(target, actual):
    difference = target @ actual.T
    return .5*np.array([difference[2, 1]-difference[1, 2], difference[0, 2]-difference[2, 0],
                        difference[1, 0]-difference[0, 1]])


def attitude_pose(nominal, body_angles):
    """Solve both 6-DOF legs for a small torso rotation, keeping world feet fixed.

    If R_body is commanded, each relative foot pose becomes R_body.T @ pose.
    This coordinates hip/knee/ankle motion and handles mirrored axes, instead
    of appending arbitrary ankle offsets or clipping individual joint targets.
    """
    nominal = np.asarray(nominal, float)
    q = nominal.copy()
    body = rpy_matrix(*body_angles)
    if np.max(np.abs(body_angles)) < 1e-10:
        return q
    for side in 'rl':
        original_r, original_p, _ = leg_pose_jacobian(nominal, side)
        target_r, target_p = body.T @ original_r, body.T @ original_p
        indices = [INDEX[j['name']] for j in model.LEG[side]]
        for _ in range(30):
            actual_r, actual_p, jacobian = leg_pose_jacobian(q, side)
            translation, orientation = target_p-actual_p, _rotation_error(target_r, actual_r)
            if np.linalg.norm(translation) < 2e-7 and np.linalg.norm(orientation) < 2e-6:
                break
            # Weight radians by a leg-length scale to condition the solve.
            weighted = jacobian.copy()
            weighted[3:] *= .1
            error = np.concatenate((translation, .1*orientation))
            delta = np.linalg.lstsq(weighted, error, rcond=1e-8)[0]
            norm = float(np.linalg.norm(delta))
            delta *= min(1., .12/max(norm, 1e-12))
            candidate = q[indices] + delta
            if np.any(candidate < LOWER[indices]) or np.any(candidate > UPPER[indices]):
                raise ValueError('balance correction exceeds a joint limit')
            q[indices] = candidate
        else:
            raise ValueError('balance IK did not converge')
    if not np.all(np.isfinite(q)):
        raise ValueError('non-finite balance target')
    return q


def posture_table(start, end, samples=80):
    """Synchronized joint-space approach used only near a known startup pose."""
    return np.array([start + (end-start)*model._smoothstep(i/samples) for i in range(samples+1)])


def foot_table(start, end, samples=80):
    """The existing upright, level-foot IK; both feet receive a common translation."""
    q = dict(zip(NAMES, start))
    first = {s: model._sole_center(q, s) for s in 'rl'}
    final_q = dict(zip(NAMES, end))
    last = {s: model._sole_center(final_q, s) for s in 'rl'}
    rows = []
    for i in range(samples+1):
        u = model._smoothstep(i/samples)
        q = model._pose_for_feet({s: first[s]+u*(last[s]-first[s]) for s in 'rl'}, q)
        rows.append([q[n] for n in NAMES])
    return np.array(rows)


def standing_poses(distance=.015):
    if not math.isfinite(distance) or not 0 <= distance <= .03:
        raise ValueError('transfer distance must be between 0 and 30 mm for this experiment')
    zero = np.zeros(12)
    _, targets = model._foot_targets('forward')
    q = model._pose_for_feet(targets['ready'], dict(zip(NAMES, zero)))
    poses = {'center': np.array([q[n] for n in NAMES])}
    for name, shift in (('left', distance), ('right', -distance)):
        q = model._pose_for_feet({s: targets['ready'][s]-[0, shift, 0] for s in 'rl'}, q)
        poses[name] = np.array([q[n] for n in NAMES])
    tables = {('zero', 'center'): foot_table(zero, poses['center'])}
    for a in poses:
        for b in poses:
            if a != b:
                tables[(a, b)] = foot_table(poses[a], poses[b])
    return poses, tables
