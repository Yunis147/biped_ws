"""Body orientation, gyro calibration and fixed upright startup alignment."""
from collections import deque
import math

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


def gravity_tilt(accel):
    return np.array([math.atan2(accel[1], accel[2]),
                     math.atan2(-accel[0], math.hypot(accel[1], accel[2]))])


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
        self.accel = np.zeros(3)
        self.upright_alignment = None
        self.orientation_reference = np.eye(3)
        self.gravity_feedback = True
        self.gravity_tau = None
        # Receipt time is used for freshness and quiet calibration. Sensor time
        # is used for integration; several callbacks can share one /clock tick.
        self.time = None
        self.sample_time = None
        self.used_source = 'waiting'
        self.calibrated = False
        self.history = deque(maxlen=500)

    def update(self, gyro, accel, quaternion, now, orientation_valid=True, *, sample_time=None):
        sample_time = now if sample_time is None else sample_time
        gyro, accel = np.asarray(gyro, float), np.asarray(accel, float)
        if (gyro.shape != (3,) or accel.shape != (3,) or not math.isfinite(now)
                or not math.isfinite(sample_time)
                or not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(accel))):
            raise ValueError('invalid IMU sample')
        gyro, accel = self.mount @ gyro, self.mount @ accel
        if np.linalg.norm(gyro) > 20:
            raise ValueError('implausible gyro rate; expected rad/s')
        if self.time is not None and now < self.time:
            raise ValueError('IMU receipt clock moved backwards')
        dt = None if self.sample_time is None else sample_time - self.sample_time
        if dt is not None and dt <= 0:
            raise ValueError('IMU source timestamps must increase')
        if (dt is not None and dt > .25) or (self.time is not None and now-self.time > .25):
            self.calibrated = False
            self.angles = None
            self.history.clear()
        rotation = None
        if self.source != 'raw' and orientation_valid:
            try:
                rotation = self.orientation_reference @ quaternion_matrix(quaternion) @ self.mount.T
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
            gravity_angles = gravity_tilt(accel)
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
                if trusted and self.gravity_feedback:
                    alpha = math.exp(-min(dt, .25) / (self.gravity_tau or self.tau))
                    difference = np.arctan2(np.sin(gravity_angles - angles), np.cos(gravity_angles - angles))
                    angles += (1-alpha) * difference
            rotation, used_source = rpy_matrix(*angles), 'raw complementary'
        self.angles, self.rotation = angles, rotation
        self.gyro, self.accel_norm, self.time = gyro - self.bias, norm, now
        self.accel = accel.copy()
        self.sample_time = sample_time
        self.used_source = used_source
        self.history.append((now, gyro.copy(), angles.copy(), norm, accel.copy()))

    def _stationary_samples(self, now, check_tilt=True):
        samples = [s for s in self.history if now - s[0] <= 1.5]
        if (self.time is None or now - self.time > .25 or len(samples) < 30
                or samples[-1][0] - samples[0][0] < 1.):
            raise ValueError('need at least one second of fresh stationary IMU data')
        if check_tilt and any(np.max(np.abs(s[2])) > math.radians(5) for s in samples):
            raise ValueError('hold the torso upright; check imu_mount_rpy (tilt is not tared away)')
        if any(np.linalg.norm(s[1]) > .06 or abs(s[3] - GRAVITY) > .1*GRAVITY for s in samples):
            raise ValueError('IMU is moving; hold still before calibrating gyro bias')
        return samples

    def calibrate(self, now):
        samples = self._stationary_samples(now)
        self.bias = np.mean([s[1] for s in samples], axis=0)
        self.calibrated = True
        return self.bias.copy()

    def align_upright(self, now):
        """One-time sensor/body alignment in a known upright, stationary pose.

        The controller permits this only at its initial calibrated joint pose.
        Rotate gyro, acceleration and orientation together; never subtract a
        changing tilt offset while walking or during later gyro calibration.
        """
        if self.upright_alignment is not None:
            raise ValueError('upright IMU alignment is already fixed for this run')
        samples = self._stationary_samples(now, check_tilt=False)
        mean_accel = np.mean([s[4] for s in samples], axis=0)
        initial_tilt = gravity_tilt(mean_accel)
        if np.max(np.abs(initial_tilt)) > math.radians(5):
            raise ValueError('startup sensor/body offset exceeds 5 deg; check imu_mount_rpy')
        spread = max(float(np.max(np.abs(gravity_tilt(s[4])-initial_tilt))) for s in samples)
        if spread > math.radians(.5):
            raise ValueError('gravity direction is changing; hold the startup pose still')
        alignment = rpy_matrix(*initial_tilt)
        mean_orientation = np.mean([s[2] for s in samples],axis=0)
        yaw = math.atan2(self.rotation[1,0],self.rotation[0,0])
        reported_upright = rpy_matrix(*mean_orientation,yaw) @ alignment.T
        raw_gyro = self.gyro+self.bias
        self.mount = alignment @ self.mount
        self.bias = alignment @ np.mean([s[1] for s in samples], axis=0)
        self.gyro = alignment @ raw_gyro-self.bias
        self.accel = alignment @ self.accel
        if self.used_source == 'raw complementary':
            if self.source == 'auto':self.source = 'raw'
            self.angles = gravity_tilt(self.accel)
            self.rotation = rpy_matrix(*self.angles)
        else:
            self.orientation_reference = reported_upright.T @ self.orientation_reference
            self.rotation = reported_upright.T @ self.rotation @ alignment.T
            self.angles = tilt(self.rotation)
        self.history = deque(((t, alignment@g, gravity_tilt(alignment@a) if self.source == 'raw'
            else tilt(reported_upright.T@rpy_matrix(*angles,yaw)@alignment.T), norm, alignment@a)
            for t,g,angles,norm,a in self.history), maxlen=500)
        self.calibrated = True
        self.upright_alignment = dict(time=now,initial_tilt_deg=np.degrees(initial_tilt).tolist(),
            initial_orientation_tilt_deg=np.degrees(mean_orientation).tolist(),
            gravity_spread_deg=math.degrees(spread),samples=len(samples),mount_matrix=self.mount.tolist(),
            orientation_reference=self.orientation_reference.tolist())
        return self.bias.copy()
