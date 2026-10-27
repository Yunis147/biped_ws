#!/usr/bin/env python3
"""
quasi_static_walk.py  —  floor-constrained quasi-static biped gait
Robot : AsterisCrack BipedRobot V2        Stack : ROS 2 Jazzy + Gazebo Harmonic

WHAT IT DOES
  Open-loop walking, assuming an upright torso and a level floor. Foot positions
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
LEAN_ANGLE    = 0.38     # sets lateral pelvis travel; actual hip roll comes from IK
BODY_HEIGHT   = 0.265    # torso origin above the soles while walking (neutral: 0.284535)
SWING_CLEARANCE = 0.012  # lift the entire level sole 12 mm; knee angle comes from IK
LEG_REACH     = 0.178136 # hip roll -> ankle roll, from the URDF joint offsets
STEP_PITCH    = {'forward':  0.12, 'backward':  0.08}   # sets half-step distance: LEG_REACH * sin(angle)
LIFTOFF_SHIFT = {'forward':  0.03, 'backward':  0.025}  # sets small fore/aft pelvis travel
TIME_SCALE    = 1.5      # scales motion and settling times; cannot fix a static balance error
PUBLISH_RATE  = 50       # Hz
TABLE_SAMPLES = 80       # limits foot-position error during runtime joint interpolation
SELF_CHECK    = True     # run the stability check at start-up

# Conservative seconds per segment, indexed by the pose it ends in.
SEGMENT_TIME = {
    'ready': 3.0,                                             # lower into the bent-knee stance
    'lean_left': 2.5, 'lean_right': 3.5, 'lean_left_cycle': 3.5,   # weight shift (both feet down)
    'lift_right_1st': 1.5, 'lift_right': 1.5, 'lift_left': 1.5,    # raise the foot
    'swing_right': 2.0, 'swing_left': 2.0,                         # swing it forward
    'place_right': 3.5, 'place_left': 3.5,                         # put it down more slowly
    'neutral': 2.5,                                                # return to standing
}
DWELL_TIME = {'ready': 0.6, 'lean_left': 0.6, 'lean_right': 0.6,
              'lean_left_cycle': 0.6, 'place_right': 1.2, 'place_left': 1.2}

JOINT_ORDER = [
    'r_hip_yaw', 'r_hip_roll_joint', 'r_hip_pitch_joint', 'r_knee_joint',
    'r_ankle_pitch_joint', 'r_ankle_roll_joint',
    'l_hip_yaw', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_joint',
    'l_ankle_pitch_joint', 'l_ankle_roll_joint',
]

# ═══════════════════════════════════════════════════════════════════════
#  ROBOT CONSTANTS — copied from robot.urdf.xacro (do not edit)
#  Each leg lists its joints in KINEMATIC order (hip yaw > hip roll > hip pitch >
#  knee > ankle roll > ankle pitch).  xyz = joint origin in the parent link frame,
#  axis = joint axis, lo/hi = joint limits (rad), mass/com = inertial data of the
#  link that joint moves (kg, m in that link's frame).
# ═══════════════════════════════════════════════════════════════════════
LEG = {
 'r': [
    dict(name='r_hip_yaw', xyz=[-0.00159, -0.021672, -0.00015], axis=(0, 0, -1), lo=-0.785398, hi=0.785398,
         mass=0.106649, com=[0.000373, -0.028289, -0.025073]),   # child link: r_hip_upper_link_1
    dict(name='r_hip_roll_joint', xyz=[0.0191, -0.030829, -0.040776], axis=(-1, 0, 0), lo=-0.436332, hi=1.570796,
         mass=0.027339, com=[-0.018233, 0.000383, -0.01695]),   # child link: r_hip_lower_link_1
    dict(name='r_hip_pitch_joint', xyz=[-0.018625, 0.020625, -0.0339], axis=(0, 1, 0), lo=-1.570796, hi=1.570796,
         mass=0.174697, com=[7e-06, -0.020639, -0.036026]),   # child link: r_thigh_link_1
    dict(name='r_knee_joint', xyz=[0.0, -0.002, -0.072052], axis=(0, -1, 0), lo=-2.094395, hi=2.094395,
         mass=0.111030, com=[-5.4e-05, -0.018976, -0.052521]),   # child link: r_calf_link_1
    dict(name='r_ankle_roll_joint', xyz=[0.018625, -0.019101, -0.072184], axis=(-1, 0, 0), lo=-1.396263, hi=1.396263,
         mass=0.027339, com=[-0.018233, 0.000383, -0.01695]),   # child link: r_ankle_link_1
    dict(name='r_ankle_pitch_joint', xyz=[-0.018625, 0.020625, -0.0339], axis=(0, 1, 0), lo=-0.610865, hi=1.48353,
         mass=0.102795, com=[0.012165, -0.020745, -0.00995]),   # child link: r_foot_link_1
],
 'l': [
    dict(name='l_hip_yaw', xyz=[-0.00159, 0.021672, -0.00015], axis=(0, 0, 1), lo=-0.785398, hi=0.785398,
         mass=0.106649, com=[0.000373, 0.028289, -0.025073]),   # child link: l_hip_upper_link_1
    dict(name='l_hip_roll_joint', xyz=[0.0191, 0.030829, -0.040776], axis=(1, 0, 0), lo=-0.436332, hi=1.570796,
         mass=0.027339, com=[-0.018233, -0.000383, -0.01695]),   # child link: l_hip_lower_link_1
    dict(name='l_hip_pitch_joint', xyz=[-0.018625, -0.020625, -0.0339], axis=(0, 1, 0), lo=-1.570796, hi=1.570796,
         mass=0.174697, com=[7e-06, 0.020639, -0.036026]),   # child link: l_thigh_link_1
    dict(name='l_knee_joint', xyz=[0.0, 0.002, -0.072052], axis=(0, -1, 0), lo=-2.094395, hi=2.094395,
         mass=0.111030, com=[-5.4e-05, 0.018976, -0.052521]),   # child link: l_calf_link_1
    dict(name='l_ankle_roll_joint', xyz=[0.018625, 0.019101, -0.072184], axis=(1, 0, 0), lo=-1.396263, hi=1.396263,
         mass=0.027339, com=[-0.018233, -0.000383, -0.01695]),   # child link: l_ankle_link_1
    dict(name='l_ankle_pitch_joint', xyz=[-0.018625, -0.020625, -0.0339], axis=(0, 1, 0), lo=-0.610865, hi=1.48353,
         mass=0.102795, com=[0.012165, 0.020745, -0.00995]),   # child link: l_foot_link_1
],
}
BASE = dict(mass=0.722478, com=[0.003008, -0.002156, 0.031318])          # current robot.urdf.xacro
# Sole of each foot = rectangle, expressed in the foot link frame (x, y) at height z.
SOLE = {
 'r': dict(z=-0.031573, poly=[[-0.028729, -0.0392], [0.061271, -0.0392], [0.061271, -0.003], [-0.028729, -0.003]]),
 'l': dict(z=-0.031573, poly=[[-0.028729, 0.003], [0.061271, 0.003], [0.061271, 0.0392], [-0.028729, 0.0392]]),
}

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
    """Sole-link pose in the upright torso frame."""
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


def _foot_targets(direction, standing=None):
    """Cartesian gait with constant walking height and a shared ground plane.

    In single support the torso translates relative to the anchored stance foot.
    In double support both foot targets receive the SAME translation, preserving
    their world positions and spacing. Touchdown only changes swing-foot z.
    """
    neutral = dict(zip(JOINT_ORDER, np.zeros(12) if standing is None else standing))
    initial = {s: _sole_center(neutral, s) for s in 'rl'}
    zero = {n: 0.0 for n in JOINT_ORDER}
    nominal_x = _sole_center(zero, 'r')[0]
    middle = (initial['r'] + initial['l']) * 0.5
    ready = {s: initial[s] - np.array([middle[0] - nominal_x, middle[1], 0.0])
             for s in 'rl'}
    for side in 'rl':
        ready[side][2] = -BODY_HEIGHT
    sign = 1.0 if direction == 'forward' else -1.0
    half_step = sign * LEG_REACH * math.sin(STEP_PITCH[direction])
    shift = LEG_REACH * math.sin(LEAN_ANGLE)
    advance = sign * LEG_REACH * math.sin(LIFTOFF_SHIFT[direction])

    def pose(lean, xr, xl, lifted=None, initial_stride=False):
        out = {s: ready[s].copy() for s in 'rl'}
        for side, x in (('r', xr), ('l', xl)):
            out[side][0] = (ready[side][0] if initial_stride else nominal_x) + x
            out[side][1] -= lean
            out[side][2] += SWING_CLEARANCE if side == lifted else 0.0
        return out

    return neutral, {
        'neutral': initial,
        'ready': ready,
        'lean_left': pose(shift, 0, 0, initial_stride=True),
        'lift_right_1st': pose(shift, -advance, -advance, 'r', initial_stride=True),
        'swing_right': pose(shift, half_step, -half_step, 'r'),
        'place_right': pose(shift, half_step, -half_step),
        'lean_right': pose(-shift, half_step, -half_step),
        'lift_left': pose(-shift, half_step - advance, -half_step - advance, 'l'),
        'swing_left': pose(-shift, -half_step, half_step, 'l'),
        'place_left': pose(-shift, -half_step, half_step),
        'lean_left_cycle': pose(shift, -half_step, half_step),
        'lift_right': pose(shift, -half_step - advance, half_step - advance, 'r'),
    }


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
# Intended loaded leg for each single-support segment, including its boundaries.
STANCE = {'lift_right_1st': 'l', 'lift_right': 'l', 'swing_right': 'l', 'place_right': 'l',
          'lift_left': 'r', 'swing_left': 'r', 'place_left': 'r'}


def build_keyframes(direction, standing=None):
    q, targets = _foot_targets(direction, standing)
    frames = {'neutral': q}
    for name, feet in targets.items():
        if name != 'neutral':
            q = _pose_for_feet(feet, q)
            frames[name] = q
    return frames


def build_tables(direction, standing=None):
    """Solve every phase in foot space, including startup, transfer and stopping."""
    kf = build_keyframes(direction, standing)
    _, targets = _foot_targets(direction, standing)
    tables = {}
    ready_midpoint = (targets['ready']['r'] + targets['ready']['l']) * 0.5
    for a, b in SEGMENTS:
        start = targets[a]
        if b == 'neutral':
            # Stop in the current stride: centre the pelvis without dragging the feet
            # together or extending the legs. The node holds this exact final pose.
            middle = (start['r'] + start['l']) * 0.5
            offset = np.array([middle[0] - ready_midpoint[0], middle[1], 0.0])
            end = {s: start[s] - offset for s in 'rl'}
        else:
            end = targets[b]
        rows, previous = [], kf[a]
        for k in range(TABLE_SAMPLES + 1):
            u = _smoothstep(k / TABLE_SAMPLES)
            feet = {s: start[s] + (end[s] - start[s]) * u for s in 'rl'}
            q = _pose_for_feet(feet, previous)
            previous = q
            rows.append([q[n] for n in JOINT_ORDER])
        tables[(a, b)] = np.array(rows)
    return kf, tables


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


def _hull(P):
    P = sorted(set(map(tuple, np.round(P, 6))))
    if len(P) < 3:
        return P

    def cr(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


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


def gait_metrics(kf, tables):
    """Check intended contacts, including double support and runtime interpolation.

    Do not infer the supporting leg from whichever foot happens to be lowest:
    that would hide a swing-foot penetration by treating it as the new support.
    """
    worst, where = math.inf, None
    ground_error = spacing_error = rotation_error = 0.0
    joint_error = 0.0
    for seg, table in tables.items():
        a, b = seg
        loaded = STANCE.get(b)
        _, first_feet = _com_and_feet(dict(zip(JOINT_ORDER, table[0])))
        spacing = first_feet['r'].mean(axis=0) - first_feet['l'].mean(axis=0)
        # The timer interpolates joint values between rows; check inside each interval too.
        samples = [row * (1.0 - u) + nxt * u
                   for row, nxt in zip(table[:-1], table[1:]) for u in (0.0, 0.5)]
        samples.append(table[-1])
        for row in samples:
            q = dict(zip(JOINT_ORDER, row))
            com, feet = _com_and_feet(q)
            down = [loaded] if loaded else list('rl')
            points = np.concatenate([feet[s] for s in down])
            m = _margin(_hull(points[:, :2]), com[:2]) * 1000.0
            if m < worst:
                worst, where = m, seg
            if a == 'neutral':  # initial lowering: the pelvis changes height
                error = float(np.ptp(points[:, 2]))
            else:
                error = float(np.max(np.abs(points[:, 2] + BODY_HEIGHT)))
                error = max(error, max(0.0, -BODY_HEIGHT - min(f[:, 2].min() for f in feet.values())))
            ground_error = max(ground_error, error)
            if not loaded:
                current = feet['r'].mean(axis=0) - feet['l'].mean(axis=0)
                spacing_error = max(spacing_error, float(np.linalg.norm(current - spacing)))
            for side in 'rl':
                R, _ = _foot_transform(q, side)
                rotation_error = max(rotation_error, float(np.linalg.norm(R - np.eye(3)) / math.sqrt(2)))
            joint_error = max(joint_error, float(np.max(LO + LIMIT_MARGIN - row)),
                              float(np.max(row - HI + LIMIT_MARGIN)))
    clearance = min((_sole_center(kf[name], swing)[2] - _sole_center(kf[name], stance)[2]) * 1000
                    for name, swing, stance in (('swing_right', 'r', 'l'), ('swing_left', 'l', 'r')))
    return dict(margin=worst, where=where, clearance=clearance,
                ground_error=ground_error * 1000, spacing_error=spacing_error * 1000,
                rotation_error=rotation_error, joint_error=joint_error)


def check_gait(direction, kf, tables):
    """Compatibility helper: return worst static margin, segment and clearance (mm)."""
    metrics = gait_metrics(kf, tables)
    return metrics['margin'], metrics['where'], metrics['clearance']


def report(direction, kf, tables, log):
    m = gait_metrics(kf, tables)
    ok = (m['margin'] >= 3.0 and m['clearance'] >= 10.0 and m['ground_error'] <= 0.2
          and m['spacing_error'] <= 0.2 and m['rotation_error'] <= 0.002 and m['joint_error'] <= 1e-8)
    log(f"[check] {direction:8s}: static margin {m['margin']:.1f} mm "
        f"(at {m['where']}), clearance {m['clearance']:.1f} mm, "
        f"floor error {m['ground_error']:.3f} mm, foot-spacing error {m['spacing_error']:.3f} mm "
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
        self.T = SEGMENT_TIME[b] * TIME_SCALE
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
                x = min(max(t, 0.0), 1.0) * TABLE_SAMPLES
                i = min(int(x), TABLE_SAMPLES - 1)
                f = x - i
                pose = self.table[i] * (1.0 - f) + self.table[i + 1] * f
        msg = Float64MultiArray()
        msg.data = np.clip(pose, LO, HI).tolist()
        self.pub.publish(msg)


def main(args=None):
    if '--check' in sys.argv:                  # offline self-check, no ROS needed
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
