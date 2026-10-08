#!/usr/bin/env python3
"""
quasi_static_walk.py  —  model-based quasi-static biped gait (v2)
Robot : AsterisCrack BipedRobot V2        Stack : ROS 2 Jazzy + Gazebo Harmonic

WHAT IT DOES
  Open-loop walking.  No RL, no sensor feedback.  A short list of key poses
  (12 joint angles each) is played slowly, with a smooth S-curve between them.

WHAT IS DIFFERENT FROM THE FIRST VERSION
  1. Flat feet are SOLVED, not guessed.  For every pose, the two ankle angles are
     computed from the robot's forward kinematics so the sole is exactly level.
  2. The lean is sized from the real foot position.  The old 0.142 rad left the
     centre of mass ~19 mm outside the stance foot; 0.40 rad puts it ~10 mm inside.
  3. The foot is lifted AFTER the body has moved over the stance foot
     ("lift" key poses), not while it is still behind it.
  4. Segment durations come from a ZMP calculation (z_c/g * acceleration), so
     the single-support moves are slow enough to count as quasi-static.
  5. `stop` finishes the current step and only returns to neutral from a pose where
     both feet are on the ground (stopping mid-step would tip the robot).
  6. At start-up the script checks its own gait with the robot model and prints
     the worst-case stability margin.  `python3 quasi_static_walk.py --check`
     does this without ROS.

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
#  TUNING  (radians).  Values below were solved/verified with the robot model.
# ═══════════════════════════════════════════════════════════════════════
LEAN_ANGLE    = 0.40     # hip-roll lean toward the stance foot (limit is 0.436)
KNEE_BEND     = {'forward': -0.80, 'backward': -0.65}   # swing-knee flexion (negative = bends back)
STEP_PITCH    = {'forward':  0.12, 'backward':  0.08}   # hip pitch of the stride (leg swing)
LIFTOFF_SHIFT = {'forward':  0.06, 'backward':  0.04}   # small forward body shift just before lift-off
TIME_SCALE    = 1.0      # 1.0 = verified timing.  >1.0 = slower (safer), <1.0 = faster (re-check!)
PUBLISH_RATE  = 50       # Hz
TABLE_SAMPLES = 40       # samples per segment in the pre-computed pose tables
SELF_CHECK    = True     # run the stability check at start-up

# seconds per segment, by the key pose it ends in (from the ZMP calculation)
SEGMENT_TIME = {
    'lean_left': 2.5, 'lean_right': 2.5, 'lean_left_cycle': 2.5,   # weight shift (both feet down)
    'lift_right_1st': 1.5, 'lift_right': 1.5, 'lift_left': 1.5,    # raise the foot
    'swing_right': 2.0, 'swing_left': 2.0,                         # swing it forward
    'place_right': 1.5, 'place_left': 1.5,                         # put it down
    'neutral': 2.5,                                                # return to standing
}

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
         mass=0.100649, com=[0.000373, -0.028289, -0.025073]),   # child link: r_hip_upper_link_1
    dict(name='r_hip_roll_joint', xyz=[0.0191, -0.030829, -0.040776], axis=(-1, 0, 0), lo=-0.436332, hi=1.570796,
         mass=0.021339, com=[-0.018233, 0.000383, -0.01695]),   # child link: r_hip_lower_link_1
    dict(name='r_hip_pitch_joint', xyz=[-0.018625, 0.020625, -0.0339], axis=(0, 1, 0), lo=-1.570796, hi=1.570796,
         mass=0.168697, com=[7e-06, -0.020639, -0.036026]),   # child link: r_thigh_link_1
    dict(name='r_knee_joint', xyz=[0.0, -0.002, -0.072052], axis=(0, -1, 0), lo=-2.094395, hi=2.094395,
         mass=0.10503, com=[-5.4e-05, -0.018976, -0.052521]),   # child link: r_calf_link_1
    dict(name='r_ankle_roll_joint', xyz=[0.018625, -0.019101, -0.072184], axis=(-1, 0, 0), lo=-1.396263, hi=1.396263,
         mass=0.021339, com=[-0.018233, 0.000383, -0.01695]),   # child link: r_ankle_link_1
    dict(name='r_ankle_pitch_joint', xyz=[-0.018625, 0.020625, -0.0339], axis=(0, 1, 0), lo=-0.610865, hi=1.48353,
         mass=0.096795, com=[0.012165, -0.020745, -0.00995]),   # child link: r_foot_link_1
],
 'l': [
    dict(name='l_hip_yaw', xyz=[-0.00159, 0.021672, -0.00015], axis=(0, 0, 1), lo=-0.785398, hi=0.785398,
         mass=0.100649, com=[0.000373, 0.028289, -0.025073]),   # child link: l_hip_upper_link_1
    dict(name='l_hip_roll_joint', xyz=[0.0191, 0.030829, -0.040776], axis=(1, 0, 0), lo=-0.436332, hi=1.570796,
         mass=0.021339, com=[-0.018233, -0.000383, -0.01695]),   # child link: l_hip_lower_link_1
    dict(name='l_hip_pitch_joint', xyz=[-0.018625, -0.020625, -0.0339], axis=(0, 1, 0), lo=-1.570796, hi=1.570796,
         mass=0.168697, com=[7e-06, 0.020639, -0.036026]),   # child link: l_thigh_link_1
    dict(name='l_knee_joint', xyz=[0.0, 0.002, -0.072052], axis=(0, -1, 0), lo=-2.094395, hi=2.094395,
         mass=0.10503, com=[-5.4e-05, 0.018976, -0.052521]),   # child link: l_calf_link_1
    dict(name='l_ankle_roll_joint', xyz=[0.018625, 0.019101, -0.072184], axis=(1, 0, 0), lo=-1.396263, hi=1.396263,
         mass=0.021339, com=[-0.018233, -0.000383, -0.01695]),   # child link: l_ankle_link_1
    dict(name='l_ankle_pitch_joint', xyz=[-0.018625, -0.020625, -0.0339], axis=(0, 1, 0), lo=-0.610865, hi=1.48353,
         mass=0.096795, com=[0.012165, 0.020745, -0.00995]),   # child link: l_foot_link_1
],
}
BASE = dict(mass=0.622478, com=[0.003008, -0.002156, 0.031318])          # base_link (torso), from the hardware repo
# Sole of each foot = rectangle, expressed in the foot link frame (x, y) at height z.
SOLE = {
 'r': dict(z=-0.031573, poly=[[-0.028729, -0.0392], [0.061271, -0.0392], [0.061271, -0.003], [-0.028729, -0.003]]),
 'l': dict(z=-0.031573, poly=[[-0.028729, 0.003], [0.061271, 0.003], [0.061271, 0.0392], [-0.028729, 0.0392]]),
}

LIMITS = {j['name']: (j['lo'], j['hi']) for s in 'rl' for j in LEG[s]}
LO = np.array([LIMITS[n][0] for n in JOINT_ORDER])
HI = np.array([LIMITS[n][1] for n in JOINT_ORDER])
UPPER = [n for n in JOINT_ORDER if 'ankle' not in n]     # joints we choose; ankles are solved
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


def _foot_up(side, q):
    """Where the foot's z-axis points, in the torso frame, for joint angles q.
    The foot is flat when this is (0, 0, 1)."""
    R = np.eye(3)
    for j in LEG[side]:
        R = R @ _rot(j['axis'], q[j['name']])
    return R[:, 2]


def solve_ankles(side, q):
    """Choose ankle roll + ankle pitch so the sole is level (Newton iteration on the 2
    tilt components).  Works for ANY hip/knee angles, which the old 'ankle = knee - hip'
    rule does not (the joint order is roll->pitch, so roll and pitch don't simply add)."""
    nr, npi = f'{side}_ankle_roll_joint', f'{side}_ankle_pitch_joint'
    ar, ap = q.get(nr, 0.0), q.get(npi, 0.0)

    def err(a, b):
        q[nr], q[npi] = a, b
        return _foot_up(side, q)[:2]

    for _ in range(30):
        e = err(ar, ap)
        if math.hypot(e[0], e[1]) < 1e-10:
            break
        h = 1e-6
        J = np.column_stack([(err(ar + h, ap) - e) / h, (err(ar, ap + h) - e) / h])
        d = np.linalg.solve(J, -e)
        ar, ap = ar + d[0], ap + d[1]
    lo, hi = LIMITS[npi]
    ap = min(max(ap, lo + LIMIT_MARGIN), hi - LIMIT_MARGIN)      # ankle pitch limit is only -0.611
    lo, hi = LIMITS[nr]
    ar = min(max(ar, lo + LIMIT_MARGIN), hi - LIMIT_MARGIN)
    q[nr], q[npi] = ar, ap


# ═══════════════════════════════════════════════════════════════════════
#  GAIT DESIGN — key poses
# ═══════════════════════════════════════════════════════════════════════
#  Sign facts from the URDF (these cost us a lot of falls the first time):
#   * hip-roll axes are mirrored (r: -X, l: +X)  ->  lean left = r_roll +L, l_roll -L
#   * hip-pitch axis is +Y: a POSITIVE pitch swings the foot BACKWARD, so "forward" is negative
#   * knee axis is -Y: NEGATIVE knee angle bends the knee the normal way (foot goes back)
def build_keyframes(direction):
    s = 1.0 if direction == 'forward' else -1.0
    L = LEAN_ANGLE
    K = KNEE_BEND[direction]
    F = -s * STEP_PITCH[direction]       # hip-pitch value that puts a leg FORWARD
    d = s * LIFTOFF_SHIFT[direction]     # both legs pitched back = body moves forward

    def pose(rr, lr, rp, lp, rk, lk):
        q = {n: 0.0 for n in JOINT_ORDER}
        q.update(r_hip_roll_joint=rr, l_hip_roll_joint=lr, r_hip_pitch_joint=rp,
                 l_hip_pitch_joint=lp, r_knee_joint=rk, l_knee_joint=lk)
        solve_ankles('r', q)
        solve_ankles('l', q)
        return q

    #                          r_roll l_roll  r_pitch l_pitch  r_knee l_knee
    return {
        'neutral':         pose(0,    0,      0,      0,       0,     0),
        'lean_left':       pose(+L,   -L,     0,      0,       0,     0),   # weight over LEFT foot
        'lift_right_1st':  pose(+L,   -L,     d,      d,       K,     0),   # lift right foot (1st step)
        'swing_right':     pose(+L,   -L,     F,      -F,      K,     0),   # right leg forward, left pushes back
        'place_right':     pose(+L,   -L,     F,      -F,      0,     0),   # right foot down
        'lean_right':      pose(-L,   +L,     F,      -F,      0,     0),   # weight over RIGHT foot
        'lift_left':       pose(-L,   +L,     F + d,  -F + d,  0,     K),
        'swing_left':      pose(-L,   +L,     -F,     F,       0,     K),
        'place_left':      pose(-L,   +L,     -F,     F,       0,     0),
        'lean_left_cycle': pose(+L,   -L,     -F,     F,       0,     0),
        'lift_right':      pose(+L,   -L,     -F + d, F + d,   K,     0),
    }


# order of key poses (the cycle repeats from swing_right)
NEXT = {
    'neutral': 'lean_left',
    'lean_left': 'lift_right_1st', 'lift_right_1st': 'swing_right',
    'swing_right': 'place_right', 'place_right': 'lean_right',
    'lean_right': 'lift_left', 'lift_left': 'swing_left',
    'swing_left': 'place_left', 'place_left': 'lean_left_cycle',
    'lean_left_cycle': 'lift_right', 'lift_right': 'swing_right',
}
# poses where BOTH feet are on the ground -> the only places it is safe to start stopping
SAFE_STOP = ('lean_left', 'place_right', 'lean_right', 'place_left', 'lean_left_cycle')
SEGMENTS = list(NEXT.items()) + [(k, 'neutral') for k in SAFE_STOP]


def blend(qa, qb, u, warm=None):
    """Pose at fraction u of the way from key pose qa to qb.  Hip/knee angles follow a smoothstep
    S-curve (zero speed at both ends); ankles are re-solved so the feet stay flat all the way."""
    s = u * u * (3.0 - 2.0 * u)
    q = {n: qa[n] + (qb[n] - qa[n]) * s for n in UPPER}
    for n in JOINT_ORDER:
        if 'ankle' in n:
            q[n] = warm[n] if warm else 0.0
    solve_ankles('r', q)
    solve_ankles('l', q)
    return q


def build_tables(direction):
    """Pre-compute every segment as a table of poses, so the 50 Hz loop only interpolates."""
    kf = build_keyframes(direction)
    tables = {}
    for a, b in SEGMENTS:
        rows, prev = [], None
        for k in range(TABLE_SAMPLES + 1):
            q = blend(kf[a], kf[b], k / TABLE_SAMPLES, prev)
            prev = q
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
        return -min(math.hypot(pt[0] - p[0], pt[1] - p[1]) for p in poly)
    m = 1e9
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        n = math.hypot(ex, ey)
        m = min(m, ((pt[0] - a[0]) * -ey + (pt[1] - a[1]) * ex) / n)
    return m


def check_gait(direction, kf, tables):
    """Walk through the whole gait pose by pose.  Whenever only one foot is on the ground,
    the CoM must be inside that foot (quasi-static balance).  Returns the worst margin (mm)
    and the swing-foot clearance (mm)."""
    path = ['neutral', 'lean_left', 'lift_right_1st', 'swing_right', 'place_right', 'lean_right',
            'lift_left', 'swing_left', 'place_left', 'lean_left_cycle', 'lift_right',
            'swing_right', 'place_right']
    segs = list(zip(path[:-1], path[1:])) + [(k, 'neutral') for k in SAFE_STOP]
    worst, where = 1e9, None
    for seg in segs:
        for row in tables[seg]:
            q = dict(zip(JOINT_ORDER, row))
            com, feet = _com_and_feet(q)
            zmin = min(f[:, 2].min() for f in feet.values())
            down = [s for s in 'rl' if feet[s][:, 2].min() <= zmin + 0.0015]
            if len(down) == 2 or not down:
                continue                       # both feet down: always supported
            pts = [p[:2] for p in feet[down[0]] if p[2] <= zmin + 0.0015]
            m = _margin(_hull(pts), com[:2]) * 1000.0
            if m < worst:
                worst, where = m, seg
    clearance = 1e9
    for nm in ('swing_right', 'swing_left'):
        _, feet = _com_and_feet(kf[nm])
        zr, zl = feet['r'][:, 2].min(), feet['l'][:, 2].min()
        clearance = min(clearance, abs(zr - zl) * 1000.0)
    return worst, where, clearance


def report(direction, kf, tables, log):
    worst, where, clr = check_gait(direction, kf, tables)
    ok = worst >= 3.0 and clr >= 10.0
    log(f"[check] {direction:8s}: worst one-foot stability margin {worst:5.1f} mm "
        f"(at {where[0]} -> {where[1]}), swing-foot clearance {clr:4.1f} mm  "
        f"{'OK' if ok else 'WARNING: low margin - slow down (TIME_SCALE) or lower the step'}")
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
            if SELF_CHECK:
                report(d, self.keyframes[d], self.tables[d], self.get_logger().info)

        self.neutral = np.zeros(12)
        self.state = 'idle'              # idle | walking
        self.direction = 'forward'
        self.stop_req = False
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
                self.direction, self.stop_req = d, False
                self.state = 'walking'
                self.t0 = self._now()
                self._begin('neutral', 'lean_left')
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
        self.t0 += self.T                      # exact carry-over, no timing drift
        if arrived == 'neutral':
            self.state = 'idle'
            self.get_logger().info('Standing.')
            return
        nxt = 'neutral' if (self.stop_req and arrived in SAFE_STOP) else NEXT[arrived]
        self._begin(arrived, nxt)

    def _tick(self):
        if self.state == 'idle':
            pose = self.neutral
        else:
            t = (self._now() - self.t0) / self.T
            while t >= 1.0 and self.state == 'walking':
                self._advance()
                if self.state == 'walking':
                    t = (self._now() - self.t0) / self.T
            if self.state == 'idle':
                pose = self.neutral
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
    node = QuasiStaticWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
