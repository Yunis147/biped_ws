"""Six-axis tilt estimation and existing servo-bridge feedback, independent of ROS."""
from collections import deque
import math
import re

import numpy as np

GRAVITY = 9.81


def rpy_matrix(roll, pitch, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or not .8 <= norm <= 1.2:
        raise ValueError('invalid IMU quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def tilt(rotation):
    return np.array([math.atan2(rotation[2, 1], rotation[2, 2]),
                     math.atan2(-rotation[2, 0], math.hypot(rotation[2, 1], rotation[2, 2]))])


class TiltEstimator:
    def __init__(self, mount_rpy=(0., 0., 0.), source='auto', tau=.5):
        if source not in ('auto', 'quaternion', 'raw') or not math.isfinite(tau) or tau <= 0:
            raise ValueError('invalid IMU source or complementary-filter time constant')
        if np.shape(mount_rpy) != (3,) or not np.all(np.isfinite(mount_rpy)):
            raise ValueError('imu_mount_rpy must contain three finite radians')
        # R_body_from_sensor: transform BOTH accelerometer and gyro, and the
        # sensor orientation. A frame_id string alone cannot rotate measurements.
        self.mount = rpy_matrix(*mount_rpy)
        self.source, self.tau = source, tau
        self.angles = self.rotation = None
        self.gyro = np.zeros(3)
        self.bias = np.zeros(3)
        self.accel_norm = 0.
        self.time = None
        self.used_source = 'waiting'
        self.calibrated = False
        self.history = deque(maxlen=500)

    def update(self, gyro, accel, quaternion, now, orientation_valid=True):
        gyro, accel = np.asarray(gyro, float), np.asarray(accel, float)
        if (gyro.shape != (3,) or accel.shape != (3,) or not math.isfinite(now)
                or not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel))):
            raise ValueError('invalid IMU sample')
        gyro, accel = self.mount @ gyro, self.mount @ accel
        if np.linalg.norm(gyro) > 20:
            raise ValueError('implausible gyro rate; expected rad/s')
        dt = None if self.time is None else now - self.time
        if dt is not None and dt <= 0:
            raise ValueError('IMU timestamps must increase')
        if dt is not None and dt > .25:
            self.calibrated = False
            self.angles = None
            self.history.clear()
        rotation = None
        if self.source != 'raw' and orientation_valid:
            try:
                rotation = quaternion_matrix(quaternion) @ self.mount.T
            except ValueError:
                pass
        norm = float(np.linalg.norm(accel))
        if not math.isfinite(norm) or norm > 1000:
            raise ValueError('implausible accelerometer sample; expected m/s^2')
        trusted = abs(norm - GRAVITY) < .15 * GRAVITY
        if rotation is not None:
            angles = tilt(rotation)
            used_source = 'quaternion'
        elif self.source == 'quaternion':
            raise ValueError('configured quaternion source has no valid orientation')
        else:
            gravity_angles = np.array([math.atan2(accel[1], accel[2]),
                                       math.atan2(-accel[0], math.hypot(accel[1], accel[2]))])
            if self.angles is None:
                if not trusted:
                    raise ValueError('raw filter needs a stationary gravity sample to initialize')
                angles = gravity_angles
            else:
                # Body gyro -> Euler rates. Only roll and pitch are needed;
                # six-axis yaw drift is irrelevant to this standing controller.
                roll, pitch = self.angles
                wx, wy, wz = gyro - self.bias
                if abs(pitch) > 1.2:
                    raise ValueError('body pitch outside the supported tilt-estimation range')
                rates = [wx + math.tan(pitch)*(math.sin(roll)*wy + math.cos(roll)*wz),
                         math.cos(roll)*wy - math.sin(roll)*wz]
                angles = self.angles + np.asarray(rates) * min(dt, .25)
                if trusted:
                    alpha = math.exp(-min(dt, .25) / self.tau)
                    difference = np.arctan2(np.sin(gravity_angles - angles), np.cos(gravity_angles - angles))
                    angles += (1-alpha) * difference
            rotation, used_source = rpy_matrix(*angles), 'raw complementary'
        self.angles, self.rotation = angles, rotation
        self.gyro, self.accel_norm, self.time = gyro - self.bias, norm, now
        self.used_source = used_source
        self.history.append((now, gyro.copy(), angles.copy(), norm))

    def calibrate(self, now):
        samples = [s for s in self.history if now - s[0] <= 1.5]
        if (self.time is None or now - self.time > .25 or len(samples) < 30
                or samples[-1][0] - samples[0][0] < 1.):
            raise ValueError('need at least one second of fresh stationary IMU data')
        if any(np.max(np.abs(s[2])) > math.radians(5) for s in samples):
            raise ValueError('hold the torso upright; check imu_mount_rpy (tilt is not tared away)')
        if any(np.linalg.norm(s[1]) > .06 or abs(s[3] - GRAVITY) > .1*GRAVITY for s in samples):
            raise ValueError('IMU is moving; hold still before calibrating gyro bias')
        self.bias = np.mean([s[1] for s in samples], axis=0)
        self.calibrated = True
        return self.bias.copy()


class Feedback:
    def __init__(self, names):
        self.names = list(names)
        self.q = self.velocity = None
        self.joint_time = self.diagnostic_time = None
        self.torque_on = False
        self.bridge_problem = 'waiting for servo_bridge diagnostics'
        self.loads = {}
        self.voltage = {}
        self.temperature = {}

    def joints(self, names, positions, velocities, now):
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise ValueError('joint state names/positions are incomplete or duplicated')
        positions = dict(zip(names, positions))
        if not all(n in positions and math.isfinite(positions[n]) for n in self.names):
            raise ValueError('joint states must contain all 12 finite joint positions')
        q = np.array([positions[n] for n in self.names])
        if len(velocities) == len(names) and np.all(np.isfinite(velocities)):
            speed = dict(zip(names, velocities))
            self.velocity = np.array([speed[n] for n in self.names])
        elif self.q is not None and now > self.joint_time:
            self.velocity = (q - self.q) / (now - self.joint_time)
        else:
            self.velocity = np.zeros(len(q))
        self.q, self.joint_time = q, now

    def diagnostics(self, message, level, values, now):
        # diagnostic_msgs/DiagnosticStatus.level is an octet: Jazzy exposes it
        # as one byte, while some adapters/ROS versions supply an integer.
        if isinstance(level, (bytes, bytearray)):
            if len(level) != 1:
                raise ValueError('invalid diagnostic level byte')
            level = level[0]
        if not isinstance(level, int) or not 0 <= level <= 3:
            raise ValueError('invalid diagnostic level')
        self.torque_on = message == 'torque ON'
        self.diagnostic_time = now
        self.loads = {}
        self.voltage = {}
        self.temperature = {}
        for name in self.names:
            text = values.get(name, '')
            match = re.search(r'\bload\s+(-?\d+)', text)
            if match:
                self.loads[name] = int(match.group(1))
            match = re.search(r'(\d+(?:\.\d+)?)\s+V\b', text)
            if match:
                self.voltage[name] = float(match.group(1))
            match = re.search(r'(-?\d+)\s+C\b', text)
            if match:
                self.temperature[name] = int(match.group(1))
        if not self.torque_on:
            self.bridge_problem = 'servo torque is OFF'
        elif values.get('startup_positioning', 'false').lower() == 'true':
            self.bridge_problem = 'bridge startup positioning is active'
        elif level > 0:
            self.bridge_problem = 'servo bridge reports a health warning/error'
        else:
            self.bridge_problem = ''

    def problem(self, now, imu, require_bridge=True):
        for stamp, timeout, name in ((self.joint_time, .25, '/joint_states'), (imu.time, .25, '/imu')):
            if stamp is None or not 0 <= now - stamp <= timeout:
                return 'missing/stale ' + name
        if require_bridge:
            if self.diagnostic_time is None or not 0 <= now - self.diagnostic_time <= 2.5:
                return 'missing/stale /diagnostics'
            if self.bridge_problem:
                return self.bridge_problem
        return ''
