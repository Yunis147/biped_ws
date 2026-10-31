#!/usr/bin/env python3
"""
quasi_static_walk.py  —  floor-constrained quasi-static biped gait
Robot : AsterisCrack BipedRobot V2        Stack : ROS 2 Jazzy + Gazebo Harmonic

WHAT IT DOES
  Open-loop walking on a level floor with a planned sideways torso lean. Foot positions
  follow smooth Cartesian paths; leg IK solves all six joints, keeping soles
  level and their heading fixed. The walking height leaves knee bend available
  at touchdown. Weight transfer and stopping preserve the planted-foot spacing.
  Start lowers into the walking stance; stop holds a centred, bent-knee stance
  with the feet where they landed. No IMU or contact feedback is used.
  `python3 quasi_static_walk.py --check` validates the commanded geometry without ROS.

COMMANDS   (topic /walk_cmd, std_msgs/String)
  start | forward   walk forward         backward   walk backward
  stop              finish the step, then stand
  (change direction only while standing: send `stop`, wait, then the new direction)

RUN IN GAZEBO  (use sim time so the gait follows the simulator clock)
  ros2 run biped_gait quasi_static_walk --ros-args -p use_sim_time:=true
RUN ON THE REAL ROBOT (no sim time)
  ros2 run biped_gait quasi_static_walk

JOINT ORDER = controllers.yaml  (index: joint)
  0 r_hip_yaw  1 r_hip_roll  2 r_hip_pitch  3 r_knee  4 r_ankle_pitch  5 r_ankle_roll
  6 l_hip_yaw  7 l_hip_roll  8 l_hip_pitch  9 l_knee 10 l_ankle_pitch 11 l_ankle_roll
"""

import math
import sys

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, String
    HAVE_ROS = True
except ImportError:                      # lets `--check` run without ROS
    HAVE_ROS = False
    Node = object

# ═══════════════════════════════════════════════════════════════════════
#  TUNING — metres, radians and seconds. Validated against the rigid robot model.
# ═══════════════════════════════════════════════════════════════════════
# These are conservative motion choices, not dimensions copied from the robot.
CROUCH_DEPTH = 0.040       # 40 mm below the zero-angle standing height
BODY_ROLL = 0.20           # 11.46 degrees toward the supporting leg
STEP_LENGTH = {'forward': 0.020, 'backward': 0.016}  # landing ahead of the other foot
SWING_CLEARANCE = 0.008    # 8 mm, on a flat floor
MAX_JOINT_SPEED = 0.18     # rad/s, below the bridge's 0.35 rad/s soft-start rate
MAX_JOINT_ACCEL = 0.30     # rad/s²; used to lengthen each phase if necessary
MIN_SUPPORT_MARGIN = 0.010  # 10 mm inside the planned supporting area
TIME_SCALE = 1.5
PUBLISH_RATE = 50
TABLE_SAMPLES = 80
SELF_CHECK = True

# Minimum seconds before TIME_SCALE; actual times also obey speed/acceleration.
SEGMENT_TIME = {
    'ready': 5.0,
    'lean_left': 4.0, 'lean_right': 5.0, 'lean_left_cycle': 5.0,
    'lift_right_1st': 2.0, 'lift_right': 2.0, 'lift_left': 2.0,
    'swing_right': 2.5, 'swing_left': 2.5,
    'place_right': 3.0, 'place_left': 3.0,
    'neutral': 5.0,
}
DWELL_TIME = {'ready': 0.8, 'lean_left': 1.0, 'lean_right': 1.0,
              'lean_left_cycle': 1.0, 'place_right': 1.2, 'place_left': 1.2}

JOINT_ORDER = [
    'r_hip_yaw', 'r_hip_roll_joint', 'r_hip_pitch_joint', 'r_knee_joint',
    'r_ankle_pitch_joint', 'r_ankle_roll_joint',
    'l_hip_yaw', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_joint',
    'l_ankle_pitch_joint', 'l_ankle_roll_joint',
]

# Read masses, centres, joint geometry and scaled foot contacts from one source.
try:
    from biped_gait.robot_model import load_model, hull as _hull
except ImportError:  # direct python3 quasi_static_walk.py --check
    from robot_model import load_model, hull as _hull

LEG, BASE, SOLE, MODEL_PATH = load_model()
TOTAL_MASS = BASE['mass'] + sum(j['mass'] for side in 'rl' for j in LEG[side])

LIMITS = {j['name']: (j['lo'], j['hi']) for s in 'rl' for j in LEG[s]}
LO = np.array([LIMITS[n][0] for n in JOINT_ORDER])
HI = np.array([LIMITS[n][1] for n in JOINT_ORDER])
LIMIT_MARGIN = 0.02                                       # keep commands this far inside the limits

# ═══════════════════════════════════════════════════════════════════════
#  MATH 1 — rotations and the flat-foot (ankle) solver
# ═══════════════════════════════════════════════════════════════════════
def _rot(axis, a):
    """Rotation matrix for angle a about a joint axis (Rodrigues)."""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _foot_transform(q, side):
    """Sole-link pose in the torso frame."""
    R, p = np.eye(3), np.zeros(3)
    for j in LEG[side]:
        p += R @ j['xyz']
        R = R @ _rot(j['axis'], q[j['name']])
    return R, p


def _foot_up(side, q):
    return _foot_transform(q, side)[0][:, 2]


def solve_ankles(side, q):
    """Solve the two ankle angles exactly, without clipping an infeasible foot tilt.

    In the ankle-roll parent frame the required up vector v satisfies
    Rx(a) Ry(b) [0,0,1] = [sin(b), -sin(a) cos(b), cos(a) cos(b)].
    Joint limits are checked by the leg IK after it converges.
    """
    R = np.eye(3)
    for j in LEG[side][:4]:
        R = R @ _rot(j['axis'], q[j['name']])
    v = R.T @ np.array([0.0, 0.0, 1.0])
    q[f'{side}_ankle_roll_joint'] = math.atan2(-v[1], v[2]) / LEG[side][4]['axis'][0]
    q[f'{side}_ankle_pitch_joint'] = math.asin(float(np.clip(v[0], -1.0, 1.0)))


def _level_foot(side, q):
    # Roll/pitch compensation alone can twist a planted sole about vertical.
    # Cancel that twist at the hip yaw so its heading also stays fixed.
    q[f'{side}_hip_yaw'] = 0.0
    solve_ankles(side, q)
    R, _ = _foot_transform(q, side)
    q[f'{side}_hip_yaw'] = -math.atan2(R[1, 0], R[0, 0]) / LEG[side][0]['axis'][2]


def _sole_center(q, side):
    R, p = _foot_transform(q, side)
    local = np.append(np.mean(SOLE[side]['poly'], axis=0), SOLE[side]['z'])
    return p + R @ local


# Zero-angle standing height includes the NEW scaled collision-mesh sole plane.
ZERO_HEIGHT = -float(np.mean([_sole_center(dict.fromkeys(JOINT_ORDER, 0.0), s)[2] for s in 'rl']))
BODY_HEIGHT = ZERO_HEIGHT - CROUCH_DEPTH
CENTER_HEIGHT = ZERO_HEIGHT - 0.020

def _leg_ik(seed, side, target):
    """Solve sole xyz with a level, forward-facing foot on the bent-knee branch.

    An unreachable position or a joint-limit violation aborts table generation;
    never silently substitute a path that can drive a foot through the floor.
    """
    q = dict(seed)
    names = [f'{side}_hip_roll_joint', f'{side}_hip_pitch_joint', f'{side}_knee_joint']
    v = np.array([q[n] for n in names], dtype=float)
    target = np.asarray(target, dtype=float)

    def position(values):
        q.update(zip(names, values))
        _level_foot(side, q)
        return _sole_center(q, side)

    if np.linalg.norm(position(v) - target) > 1e-7 and abs(v[2]) < 0.03:
        # Leave the straight-leg singularity when lowering from the initial zero pose.
        v[1] -= 0.04
        v[2] = -0.08
    for _ in range(60):
        current = position(v)
        err = target - current
        if np.linalg.norm(err) < 1e-7:
            for j in LEG[side]:
                angle = q[j['name']]
                if not j['lo'] + LIMIT_MARGIN <= angle <= j['hi'] - LIMIT_MARGIN:
                    raise ValueError(f"{side} foot target exceeds {j['name']} limit: {angle:.4f} rad")
            return q
        h = 1e-6
        J = np.column_stack([(position(v + np.eye(3)[i] * h) - current) / h
                             for i in range(3)])
        delta = np.linalg.lstsq(J, err, rcond=None)[0]
        norm = float(np.linalg.norm(delta))
        if norm > 0.15:
            delta *= 0.15 / norm
        v += delta
        v[2] = min(v[2], -1e-6)  # do not cross through zero onto the hyperextended branch
    raise ValueError(f'{side} foot IK did not converge for target {target.tolist()}')


def _pose_for_feet(targets, seed):
    q = dict(seed)
    for side in 'rl':
        q = _leg_ik(q, side, targets[side])
    return q


def _smoothstep(u):
    # Quintic: zero velocity AND acceleration at each phase boundary.
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u))


def _leg_pose_ik(seed, side, target_position, target_rotation, check_limits=True):
    """Six-joint IK: put the sole at a position AND keep it flat in the world.

    The body may lean. A foot rotation measured relative to that body therefore
    need not be identity. The existing upright IK remains available separately.
    """
    q = dict(seed)
    names = [j['name'] for j in LEG[side]]
    if abs(q[names[3]]) < 0.01:
        q[names[2]], q[names[3]] = -0.04, -0.08
    for _ in range(70):
        R, p = np.eye(3), np.zeros(3)
        origins, axes = [], []
        for j in LEG[side]:
            p += R @ j['xyz']
            origins.append(p.copy())
            axes.append(R @ np.asarray(j['axis']))
            R = R @ _rot(j['axis'], q[j['name']])
        sole = p + R @ np.append(np.mean(SOLE[side]['poly'], axis=0), SOLE[side]['z'])
        difference = target_rotation @ R.T
        tilt_error = 0.5 * np.array([difference[2, 1] - difference[1, 2],
                                     difference[0, 2] - difference[2, 0],
                                     difference[1, 0] - difference[0, 1]])
        translation_error = target_position - sole
        if np.linalg.norm(translation_error) < 2e-8 and np.linalg.norm(tilt_error) < 2e-7:
            for j in (LEG[side] if check_limits else []):
                value = q[j['name']]
                if not j['lo'] + LIMIT_MARGIN <= value <= j['hi'] - LIMIT_MARGIN:
                    raise ValueError(f"{side} foot target exceeds {j['name']} limit: {value:.4f} rad")
            return q
        # A small joint turn moves the sole around that joint's axis. Those six
        # effects form a table (Jacobian); solve it for the six small angle changes.
        J = np.vstack((np.column_stack([np.cross(a, sole - o) for a, o in zip(axes, origins)]),
                       0.1 * np.column_stack(axes)))
        error = np.concatenate((translation_error, 0.1 * tilt_error))
        delta = np.linalg.lstsq(J, error, rcond=1e-8)[0]
        delta *= min(1.0, 0.12 / max(float(np.linalg.norm(delta)), 1e-12))
        for name, change in zip(names, delta):
            q[name] += change
        q[names[3]] = min(q[names[3]], -1e-6)
    raise ValueError(f'{side} full-pose IK did not converge')


def _balanced_pose(feet, desired_com, body, seed, roll):
    """Solve pelvis x/y so the WHOLE robot's CoM reaches the planned point.

    feet and body are in a shared ground frame. Feet are held fixed during a
    transfer. The lifted leg and asymmetric inertials remain in the mass sum.
    """
    rotation = _rot((1, 0, 0), roll)
    body, q = np.array(body, dtype=float), dict(seed)

    def evaluate(position, previous):
        result = dict(previous)
        for side in 'rl':
            result = _leg_pose_ik(result, side, rotation.T @ (feet[side] - position), rotation.T, check_limits=False)
        com, _ = _com_and_feet(result)
        return result, (rotation @ com + position)[:2] - desired_com

    for _ in range(25):
        q, error = evaluate(body, q)
        if np.linalg.norm(error) < 1e-7:
            for side in 'rl':
                for j in LEG[side]:
                    if not j['lo'] + LIMIT_MARGIN <= q[j['name']] <= j['hi'] - LIMIT_MARGIN:
                        raise ValueError(f"mass-balanced pose exceeds {j['name']} limit: {q[j['name']]:.4f} rad")
            return q, body
        response = np.zeros((2, 2))
        for axis in range(2):
            shifted = body.copy()
            shifted[axis] += 1e-5
            _, shifted_error = evaluate(shifted, q)
            response[:, axis] = (shifted_error - error) / 1e-5
        change = np.linalg.solve(response, -error)
        change *= min(1.0, 0.006 / max(float(np.linalg.norm(change)), 1e-12))
        body[:2] += change
    raise ValueError('whole-robot mass balance solve did not converge')


def _plan(direction, standing=None):
    if direction not in STEP_LENGTH:
        raise ValueError(f'unknown walking direction: {direction}')
    neutral = dict(zip(JOINT_ORDER, np.zeros(12) if standing is None else standing))
    if not all(np.isfinite(list(neutral.values()))):
        raise ValueError('invalid starting pose')
    feet = {s: _sole_center(neutral, s) for s in 'rl'}
    height = -float(np.mean([f[2] for f in feet.values()]))
    if any(np.linalg.norm(_foot_transform(neutral, s)[0] - np.eye(3)) > 1e-4 for s in 'rl'):
        raise ValueError('start requires an upright, level-foot standing pose')
    if abs(feet['r'][2] - feet['l'][2]) > 1e-5:
        raise ValueError('starting soles are not on the same ground plane')
    for side in 'rl':
        feet[side] = feet[side] + [0, 0, height]
    middle = (feet['r'][:2] + feet['l'][:2]) * 0.5
    initial_com, _ = _com_and_feet(neutral)
    step = STEP_LENGTH[direction] * (1 if direction == 'forward' else -1)
    states = {}

    def add(name, com, roll=0.0, z=BODY_HEIGHT):
        states[name] = dict(feet={s: f.copy() for s, f in feet.items()},
                            com=np.array(com).copy(), roll=roll, height=z)

    add('neutral', initial_com[:2], z=height)
    add('ready', middle, z=CENTER_HEIGHT)
    add('lean_left', feet['l'][:2], -BODY_ROLL)
    feet['r'][2] = SWING_CLEARANCE
    add('lift_right_1st', feet['l'][:2], -BODY_ROLL)
    feet['r'][0] = feet['l'][0] + step
    add('swing_right', feet['l'][:2], -BODY_ROLL)
    feet['r'][2] = 0.0
    add('place_right', feet['l'][:2], -BODY_ROLL)
    add('lean_right', feet['r'][:2], BODY_ROLL)
    feet['l'][2] = SWING_CLEARANCE
    add('lift_left', feet['r'][:2], BODY_ROLL)
    feet['l'][0] = feet['r'][0] + step
    add('swing_left', feet['r'][:2], BODY_ROLL)
    feet['l'][2] = 0.0
    add('place_left', feet['r'][:2], BODY_ROLL)
    add('lean_left_cycle', feet['l'][:2], -BODY_ROLL)
    feet['r'][2] = SWING_CLEARANCE
    add('lift_right', feet['l'][:2], -BODY_ROLL)
    return neutral, states


NEXT = {
    'neutral': 'ready', 'ready': 'lean_left',
    'lean_left': 'lift_right_1st', 'lift_right_1st': 'swing_right',
    'swing_right': 'place_right', 'place_right': 'lean_right',
    'lean_right': 'lift_left', 'lift_left': 'swing_left',
    'swing_left': 'place_left', 'place_left': 'lean_left_cycle',
    'lean_left_cycle': 'lift_right', 'lift_right': 'swing_right',
}
SAFE_STOP = ('ready', 'lean_left', 'place_right', 'lean_right', 'place_left', 'lean_left_cycle')
SEGMENTS = list(NEXT.items()) + [(k, 'neutral') for k in SAFE_STOP]
STANCE = {'lift_right_1st': 'l', 'lift_right': 'l', 'swing_right': 'l', 'place_right': 'l',
          'lift_left': 'r', 'swing_left': 'r', 'place_left': 'r'}


def _height_between(start, end, u):
    height = start['height'] + u * (end['height'] - start['height'])
    if start['roll'] * end['roll'] < 0:
        # Raise the pelvis in the middle of a transfer, where both legs are
        # under it. Lower again for the longer sideways reach in single support.
        height += (CENTER_HEIGHT - BODY_HEIGHT) * 16 * u*u * (1-u)*(1-u)
    return height


def _plan_keyframes(direction, standing=None):
    q, states = _plan(direction, standing)
    frames = {'neutral': dict(q)}
    bodies = {'neutral': np.array([0., 0., states['neutral']['height']])}
    body = bodies['neutral'].copy()
    previous = states['neutral']
    for name, state in list(states.items())[1:]:
        # Intermediate desired points avoid jumping straight across a joint limit
        # during Newton iterations when computing an isolated keyframe.
        for u in np.linspace(0, 1, 21)[1:]:
            feet = {s: previous['feet'][s] + u * (state['feet'][s] - previous['feet'][s]) for s in 'rl'}
            com = previous['com'] + u * (state['com'] - previous['com'])
            body[2] = _height_between(previous, state, u)
            roll = previous['roll'] + u * (state['roll'] - previous['roll'])
            try:
                q, body = _balanced_pose(feet, com, body, q, roll)
            except ValueError as exc:
                raise ValueError(f'{name} at {u:.3f}: {exc}') from exc
        frames[name], bodies[name] = dict(q), body.copy()
        previous = state
    return frames, bodies, states


def build_keyframes(direction, standing=None):
    return _plan_keyframes(direction, standing)[0]


def _foot_targets(direction, standing=None):
    """Body-frame targets, retained for existing read-only standing experiments."""
    frames = build_keyframes(direction, standing)
    return frames['neutral'], {name: {s: _sole_center(q, s) for s in 'rl'}
                               for name, q in frames.items()}


def build_tables(direction, standing=None):
    """Solve every sample for foot contact, torso lean AND whole-robot balance."""
    kf, bodies, states = _plan_keyframes(direction, standing)
    tables = {}
    for a, b in SEGMENTS:
        start = states[a]
        if b == 'neutral':
            end = dict(feet=start['feet'], com=(start['feet']['r'][:2] + start['feet']['l'][:2]) * 0.5,
                       roll=0.0, height=CENTER_HEIGHT)
        else:
            end = states[b]
            if (a, b) == ('lift_right', 'swing_right'):
                # The next stride is translated in the world. Reuse its joint
                # pattern while keeping the current left support foot anchored.
                offset = start['feet']['l'] - end['feet']['l']
                end = dict(end, feet={s: f + offset for s, f in end['feet'].items()},
                           com=end['com'] + offset[:2])
        rows, previous, body = [], kf[a], bodies[a].copy()
        for k in range(TABLE_SAMPLES + 1):
            u = _smoothstep(k / TABLE_SAMPLES)
            feet = {s: start['feet'][s] + u * (end['feet'][s] - start['feet'][s]) for s in 'rl'}
            com = start['com'] + u * (end['com'] - start['com'])
            body[2] = _height_between(start, end, u)
            roll = start['roll'] + u * (end['roll'] - start['roll'])
            if k:
                try:
                    previous, body = _balanced_pose(feet, com, body, previous, roll)
                except ValueError as exc:
                    raise ValueError(f'{a} -> {b}, sample {k}: {exc}') from exc
            rows.append([previous[n] for n in JOINT_ORDER])
        tables[(a, b)] = np.array(rows)
        # Use one canonical joint vector at shared phase boundaries. Independent
        # solves agree to tiny tolerances, but even that numerical seam is avoidable.
        if b != 'neutral':
            tables[(a, b)][-1] = [kf[b][n] for n in JOINT_ORDER]
    return kf, tables


def _interpolation_coefficients(table):
    slopes = np.zeros_like(table)
    slopes[1:-1] = (table[2:] - table[:-2]) * 0.5
    first, last, m0, m1 = table[:-1], table[1:], slopes[:-1], slopes[1:]
    return 2*first - 2*last + m0 + m1, -3*first + 3*last - 2*m0 - m1, m0, first


def table_pose(table, fraction):
    """Cubic interpolation: continuous joint velocity and stopped phase ends."""
    if fraction <= 0:
        return table[0].copy()
    if fraction >= 1:
        return table[-1].copy()
    coordinate = fraction * (len(table) - 1)
    index = min(int(coordinate), len(table) - 2)
    u = coordinate - index
    # Compute just this interval; no allocation of a full table at 50 Hz.
    first, last = table[index], table[index + 1]
    m0 = (last - table[index - 1]) * .5 if index else np.zeros(12)
    m1 = (table[index + 2] - first) * .5 if index + 2 < len(table) else np.zeros(12)
    a, b = 2*first - 2*last + m0 + m1, -3*first + 3*last - 2*m0 - m1
    return ((a*u + b)*u + m0)*u + first


def _motion_bounds(table):
    a, b, c, _ = _interpolation_coefficients(table)
    # Exact extrema of each cubic's first and second derivatives.
    u = np.clip(np.divide(-b, 3*a, out=np.zeros_like(a), where=np.abs(a) > 1e-15), 0, 1)
    speed = max(float(np.max(np.abs(c))), float(np.max(np.abs(3*a + 2*b + c))),
                float(np.max(np.abs(3*a*u*u + 2*b*u + c)))) * (len(table) - 1)
    acceleration = max(float(np.max(np.abs(2*b))), float(np.max(np.abs(6*a + 2*b)))) * (len(table) - 1)**2
    return speed, acceleration


def segment_duration(destination, table):
    """Lengthen slow phases to respect all joints; never clip individual angles."""
    speed, acceleration = _motion_bounds(table)
    return max(SEGMENT_TIME[destination] * TIME_SCALE,
               speed / MAX_JOINT_SPEED, math.sqrt(acceleration / MAX_JOINT_ACCEL))

# ═══════════════════════════════════════════════════════════════════════
#  MATH 2 — centre of mass and the support polygon (used for the self-check)
# ═══════════════════════════════════════════════════════════════════════
def _com_and_feet(q):
    """Forward kinematics: centre of mass of the whole robot and the 4 sole corners of each foot,
    all in the torso frame.   CoM = sum(m_i * r_i) / sum(m_i)."""
    mtot = BASE['mass']
    com = BASE['mass'] * np.array(BASE['com'])
    feet = {}
    for side in 'rl':
        T = np.eye(4)
        for j in LEG[side]:
            M = np.eye(4)
            M[:3, 3] = j['xyz']
            R = np.eye(4)
            R[:3, :3] = _rot(j['axis'], q[j['name']])
            T = T @ M @ R                                  # this link's frame in the torso frame
            com += j['mass'] * (T @ np.append(j['com'], 1.0))[:3]
            mtot += j['mass']
        pts = np.array([[x, y, SOLE[side]['z'], 1.0] for x, y in SOLE[side]['poly']])
        feet[side] = (T @ pts.T).T[:, :3]
    return com / mtot, feet



def _margin(poly, pt):
    """Signed distance from point to a convex polygon: + inside, - outside."""
    if len(poly) < 3:
        if not poly:
            return -math.inf
        return -min(math.hypot(pt[0] - p[0], pt[1] - p[1]) for p in poly)
    m = 1e9
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        n = math.hypot(ex, ey)
        m = min(m, ((pt[0] - a[0]) * -ey + (pt[1] - a[1]) * ex) / n)
    return m


def _ground_frame(q, reference='l'):
    """Rotate body-frame values into axes parallel to the flat supporting sole."""
    rotation = _foot_transform(q, reference)[0].T
    com, feet = _com_and_feet(q)
    return rotation @ com, {s: f @ rotation.T for s, f in feet.items()}, rotation


def gait_metrics(kf, tables):
    """Check the full interpolated motion, using the INTENDED support foot.

    Rotating into the supporting sole's frame accounts for the planned body
    roll; body-frame vertical is not gravity vertical when the torso leans.
    """
    worst, where = math.inf, None
    ground_error = spacing_error = rotation_error = 0.0
    joint_error = max_speed = max_acceleration = com_error = 0.0
    min_knee_bend = math.inf
    max_body_roll = 0.0
    for seg, table in tables.items():
        a, b = seg
        loaded = STANCE.get(b)
        reference = loaded or 'l'
        _, first_feet, _ = _ground_frame(dict(zip(JOINT_ORDER, table[0])), reference)
        spacing = first_feet['r'].mean(axis=0) - first_feet['l'].mean(axis=0)
        duration = segment_duration(b, table)
        speed, acceleration = _motion_bounds(table)
        max_speed = max(max_speed, speed / duration)
        max_acceleration = max(max_acceleration, acceleration / duration**2)
        # Check quarter points as well as rows, using the SAME cubic as the timer.
        for fraction in np.linspace(0, 1, TABLE_SAMPLES * 4 + 1):
            row = table_pose(table, fraction)
            q = dict(zip(JOINT_ORDER, row))
            com, feet, rotation = _ground_frame(q, reference)
            down = [loaded] if loaded else list('rl')
            points = np.concatenate([feet[s] for s in down])
            m = _margin(_hull(points[:, :2]), com[:2]) * 1000.0
            if m < worst:
                worst, where = m, seg
            ground = float(points[:, 2].mean())
            error = max(float(np.ptp(points[:, 2])),
                        max(0.0, ground - min(f[:, 2].min() for f in feet.values())))
            ground_error = max(ground_error, error)
            if not loaded:
                current = feet['r'].mean(axis=0) - feet['l'].mean(axis=0)
                spacing_error = max(spacing_error, float(np.linalg.norm(current - spacing)))
            else:
                com_error = max(com_error, float(np.linalg.norm(com[:2] - feet[loaded][:, :2].mean(axis=0))))
            for side in 'rl':
                R, _ = _foot_transform(q, side)
                rotation_error = max(rotation_error, float(np.linalg.norm(rotation @ R - np.eye(3)) / math.sqrt(2)))
                if a != 'neutral':
                    min_knee_bend = min(min_knee_bend, -q[f'{side}_knee_joint'])
            max_body_roll = max(max_body_roll, abs(math.atan2(rotation[2, 1], rotation[2, 2])))
            joint_error = max(joint_error, float(np.max(LO + LIMIT_MARGIN - row)),
                              float(np.max(row - HI + LIMIT_MARGIN)))
    clearance = math.inf
    for name, swing, stance in (('swing_right', 'r', 'l'), ('swing_left', 'l', 'r')):
        _, feet, _ = _ground_frame(kf[name], stance)
        clearance = min(clearance, float(feet[swing][:, 2].min() - feet[stance][:, 2].max()) * 1000)
    return dict(margin=worst, where=where, clearance=clearance,
                ground_error=ground_error * 1000, spacing_error=spacing_error * 1000,
                rotation_error=rotation_error, joint_error=joint_error,
                max_speed=max_speed, max_acceleration=max_acceleration,
                com_error=com_error * 1000, min_knee_bend=min_knee_bend, max_body_roll=max_body_roll)

def check_gait(direction, kf, tables):
    """Compatibility helper: return worst static margin, segment and clearance (mm)."""
    metrics = gait_metrics(kf, tables)
    return metrics['margin'], metrics['where'], metrics['clearance']


def report(direction, kf, tables, log):
    m = gait_metrics(kf, tables)
    ok = (m['margin'] >= MIN_SUPPORT_MARGIN * 1000
          and m['clearance'] >= SWING_CLEARANCE * 1000 - 0.1 and m['ground_error'] <= 0.2
          and m['spacing_error'] <= 0.2 and m['rotation_error'] <= 0.002 and m['joint_error'] <= 1e-8
          and m['max_speed'] <= MAX_JOINT_SPEED + 1e-9
          and m['max_acceleration'] <= MAX_JOINT_ACCEL + 1e-9
          and m['com_error'] <= 0.2 and m['min_knee_bend'] >= 0.12)
    log(f"[check] {direction:8s}: static margin {m['margin']:.1f} mm "
        f"(at {m['where']}), clearance {m['clearance']:.1f} mm, "
        f"floor error {m['ground_error']:.3f} mm, foot-spacing error {m['spacing_error']:.3f} mm, "
        f"CoM centring error {m['com_error']:.3f} mm, peak speed {m['max_speed']:.3f} rad/s, "
        f"peak acceleration {m['max_acceleration']:.3f} rad/s² "
        f"{'OK' if ok else 'FAILED: revise the foot targets or robot model before walking'}")
    return ok


# ═══════════════════════════════════════════════════════════════════════
#  ROS 2 NODE
# ═══════════════════════════════════════════════════════════════════════
class QuasiStaticWalker(Node):

    def __init__(self):
        super().__init__('quasi_static_walker')
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.create_subscription(String, '/walk_cmd', self.walk_cmd_cb, 10)

        self.get_logger().info(f'Model: {MODEL_PATH}; mass {TOTAL_MASS:.6f} kg, zero height {ZERO_HEIGHT*1000:.2f} mm')
        self.get_logger().info('Pre-computing gait tables ...')
        self.keyframes, self.tables = {}, {}
        for d in ('forward', 'backward'):
            self.keyframes[d], self.tables[d] = build_tables(d)
            if SELF_CHECK and not report(d, self.keyframes[d], self.tables[d], self.get_logger().info):
                raise ValueError(f'{d} gait failed the geometry/balance check')

        self.neutral = np.zeros(12)
        self.state = 'idle'              # idle | walking
        self.direction = 'forward'
        self.stop_req = False
        self._dwell_until = None
        self.cur = self.nxt = None
        self.table = None
        self.T = 1.0
        self.t0 = 0.0

        self.create_timer(1.0 / PUBLISH_RATE, self._tick)
        self.get_logger().info("Ready.  ros2 topic pub --once /walk_cmd std_msgs/msg/String \"{data: 'start'}\"")

    # ---- commands --------------------------------------------------------
    def walk_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('start', 'forward', 'backward'):
            d = 'backward' if cmd == 'backward' else 'forward'
            if self.state == 'idle':
                start = np.array([self.keyframes[d]['neutral'][n] for n in JOINT_ORDER])
                if not np.allclose(start, self.neutral, atol=1e-8):
                    # A stop leaves the feet in their current stride. Build the next
                    # startup from that stance instead of resetting both planted feet.
                    try:
                        kf, tables = build_tables(d, self.neutral)
                        if SELF_CHECK and not report(d, kf, tables, self.get_logger().info):
                            raise ValueError('restart gait failed the geometry/balance check')
                    except ValueError as exc:
                        self.get_logger().error(f'Cannot start: {exc}')
                        return
                    self.keyframes[d], self.tables[d] = kf, tables
                self.direction, self.stop_req = d, False
                self.state = 'walking'
                self.t0 = self._now()
                self._begin('neutral', 'ready')
                self.get_logger().info(f'Walking {d}.')
            elif d != self.direction:
                self.get_logger().warn('Already walking the other way: send `stop`, wait until it stands, then retry.')
            else:
                self.stop_req = False        # cancels a pending stop
        elif cmd == 'stop':
            if self.state == 'walking':
                self.stop_req = True
                self.get_logger().info('Stop requested: finishing the step, then standing.')
        else:
            self.get_logger().warn(f"Unknown command '{cmd}'. Use start / forward / backward / stop.")

    # ---- segment handling ------------------------------------------------
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _begin(self, a, b):
        self.cur, self.nxt = a, b
        self.T = segment_duration(b, self.tables[self.direction][(a, b)])
        self.table = self.tables[self.direction][(a, b)]
        self.get_logger().info(f'  {a} -> {b}  ({self.T:.1f} s)')

    def _advance(self):
        arrived = self.nxt
        # Do not skip physical phases to catch up after a delayed timer callback.
        self.t0 = self._now()
        if arrived == 'neutral':
            self.neutral = self.table[-1].copy()
            self.state = 'idle'
            self.get_logger().info('Standing.')
            return
        nxt = 'neutral' if (self.stop_req and arrived in SAFE_STOP) else NEXT[arrived]
        if arrived in DWELL_TIME:
            # A timed settle, not a measurement of ground contact or balance.
            self._dwell_until = self.t0 + DWELL_TIME[arrived] * TIME_SCALE
        else:
            self._begin(arrived, nxt)

    def _tick(self):
        if self.state == 'idle':
            pose = self.neutral
        elif self._dwell_until is not None:
            pose = self.table[-1]
            if self._now() >= self._dwell_until:
                arrived = self.nxt
                # Evaluate stop now: a command received during the settle must count.
                nxt = 'neutral' if self.stop_req and arrived in SAFE_STOP else NEXT[arrived]
                self._dwell_until = None
                self.t0 = self._now()
                self._begin(arrived, nxt)
        else:
            t = (self._now() - self.t0) / self.T
            if t >= 1.0:
                pose = self.table[-1]
                self._advance()
            else:
                pose = table_pose(self.table, t)
        msg = Float64MultiArray()
        msg.data = pose.tolist()  # already checked; clipping would spoil coordinated feet
        self.pub.publish(msg)


def main(args=None):
    if '--check' in sys.argv:                  # offline self-check, no ROS needed
        print(f'Model: {MODEL_PATH}\nMass: {TOTAL_MASS:.6f} kg; zero height: {ZERO_HEIGHT*1000:.3f} mm; '
              f'centre height: {CENTER_HEIGHT*1000:.3f} mm; single-support height: {BODY_HEIGHT*1000:.3f} mm')
        ok = True
        for d in ('forward', 'backward'):
            kf, tab = build_tables(d)
            ok &= report(d, kf, tab, print)
        sys.exit(0 if ok else 1)
    if not HAVE_ROS:
        sys.exit('ROS 2 Python packages not found: source your ROS 2 / workspace setup.bash first.')
    rclpy.init(args=args)
    node = None
    try:
        node = QuasiStaticWalker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
