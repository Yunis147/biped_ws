"""ROS interface for the separate standing-balance experiment."""
import csv
from contextlib import suppress
from dataclasses import fields
import json
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String

from .controller import BalanceExperiment, Settings
from .kinematics import NAMES


class BalanceNode(Node):
    def __init__(self):
        super().__init__('biped_balance')
        self.declare_parameter('actuate', False)
        self.declare_parameter('imu_axes_verified', False)
        self.declare_parameter('imu_source', 'auto')
        self.declare_parameter('imu_mount_rpy', [0., 0., 0.])
        self.declare_parameter('filter_tau', .5)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('record_path', '')
        self.declare_parameter('record_rate', 10.)
        for field in fields(Settings):
            self.declare_parameter(field.name, float(field.default))
        parameter = lambda name: self.get_parameter(name).value
        settings = Settings(**{f.name: parameter(f.name) for f in fields(Settings)})
        self.get_logger().info('Preparing standing and planted-foot transfer tables ...')
        self.experiment = BalanceExperiment(
            actuate=parameter('actuate'), axes_verified=parameter('imu_axes_verified'),
            mount_rpy=parameter('imu_mount_rpy'), imu_source=parameter('imu_source'),
            filter_tau=parameter('filter_tau'), settings=settings)
        self.imu_frame = parameter('imu_frame')
        self.record_rate = float(parameter('record_rate'))
        if not np.isfinite(self.record_rate) or not 0 < self.record_rate <= 50:
            raise ValueError('record_rate must be in (0, 50] Hz')
        self._csv_file = self._csv = None
        record_path = parameter('record_path')
        if record_path:
            path = Path(record_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = path.open('x', newline='', encoding='utf-8')
            self._csv = csv.writer(self._csv_file)
            self._csv.writerow(['time', 'state', 'reason', 'roll_deg', 'pitch_deg',
                                'gyro_x', 'gyro_y', 'gyro_z', 'accel_norm', 'calibrated',
                                'balance_enabled', 'correction_roll_deg', 'correction_pitch_deg',
                                'tracking_error_deg', 'tracking_joint', 'torque_on',
                                'min_voltage_v', 'max_temperature_c'] +
                               ['position_'+n for n in NAMES] + ['target_'+n for n in NAMES] +
                               ['speed_'+n for n in NAMES] +
                               ['load_raw_'+n for n in NAMES])
            self.get_logger().info(f'Recording new CSV: {path}')
        self.command_pub = (self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
                            if self.experiment.actuate else None)
        self.preview_pub = self.create_publisher(Float64MultiArray, '/balance/preview', 10)
        self.status_pub = self.create_publisher(String, '/balance/status', 10)
        self.create_subscription(Imu, '/imu', self._imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._joints, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostics, 10)
        self.create_subscription(String, '/balance/cmd', self._command, 10)
        self.create_subscription(Float64MultiArray, '/position_controller/commands', self._observed_command, 10)
        self.observed_command = None
        self.observed_command_time = None
        self.last_input_error = ''
        self._last_error_log = -float('inf')
        self._last_record = self._last_status = -float('inf')
        self._last_state = None
        self.create_timer(.02, self._tick)
        self.add_on_set_parameters_callback(self._set_parameters)
        self.get_logger().info('Ready in monitoring state. Use /balance/cmd: status, calibrate, stand, '
                               'balance_on, balance_off, lean_left, lean_right, center, hold, monitor. '
                               f'Actuator publisher {"enabled" if self.command_pub else "absent (monitor-only)"}.')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _set_parameters(self, parameters):
        # Avoid reporting successful parameter changes that the controller would
        # not actually use. Settings are validated together at the next launch.
        if any(p.name != 'use_sim_time' for p in parameters):
            return SetParametersResult(successful=False, reason='balance settings are startup-only; restart with overrides')
        return SetParametersResult(successful=True)

    def _fresh_stamp(self, header, timeout):
        stamp = header.stamp.sec + header.stamp.nanosec*1e-9
        if stamp <= 0 or not -.05 <= self._now()-stamp <= timeout:
            raise ValueError('stale/invalid source timestamp; check ROS clocks and sensor driver')

    def _input_error(self, error):
        self.last_input_error = str(error)
        now = self._now()
        if now-self._last_error_log >= 2:
            self.get_logger().warning(self.last_input_error)
            self._last_error_log = now

    def _imu(self, message):
        try:
            self._fresh_stamp(message.header, .25)
            if message.header.frame_id != self.imu_frame:
                raise ValueError(f'expected IMU frame {self.imu_frame}; got {message.header.frame_id}')
            if message.angular_velocity_covariance[0] < 0 or message.linear_acceleration_covariance[0] < 0:
                raise ValueError('IMU gyro and accelerometer are both required')
            gyro, accel, q = message.angular_velocity, message.linear_acceleration, message.orientation
            self.experiment.imu.update([gyro.x, gyro.y, gyro.z], [accel.x, accel.y, accel.z],
                                       [q.x, q.y, q.z, q.w], self._now(),
                                       orientation_valid=message.orientation_covariance[0] >= 0)
        except ValueError as error:
            self._input_error(error)

    def _joints(self, message):
        try:
            self._fresh_stamp(message.header, .25)
            self.experiment.feedback.joints(message.name, message.position, message.velocity, self._now())
        except ValueError as error:
            self._input_error(error)

    def _diagnostics(self, message):
        try:
            self._fresh_stamp(message.header, 2.5)
            for status in message.status:
                if status.name.rsplit('/', 1)[-1] == 'servo_bridge':
                    self.experiment.feedback.diagnostics(status.message, status.level,
                                                         {v.key: v.value for v in status.values}, self._now())
        except ValueError as error:
            self._input_error(error)

    def _observed_command(self, message):
        if len(message.data) == len(NAMES) and np.all(np.isfinite(message.data)):
            self.observed_command = np.array(message.data)
            self.observed_command_time = self._now()

    def _writers(self):
        own = 1 if self.command_pub is not None else 0
        return max(0, self.count_publishers('/position_controller/commands')-own)

    def _receivers(self):
        # Our own recording subscription is not a hardware command consumer.
        return max(0, self.count_subscribers('/position_controller/commands')-1)

    def _command(self, message):
        try:
            result = self.experiment.handle(message.data, self._now(), self._writers(),
                                            self._receivers())
            self.get_logger().info(result)
        except ValueError as error:
            self.get_logger().warning(str(error))
        self._publish_status()

    def _status(self):
        now = self._now()
        status = self.experiment.status(now)
        status['last_input_warning'] = self.last_input_error
        status['other_command_publishers'] = self._writers()
        if (self.experiment.state == 'monitoring' and self.observed_command is not None
                and self.experiment.feedback.q is not None):
            error = np.abs(self.observed_command-self.experiment.feedback.q)
            index = int(np.argmax(error))
            status['observed_target_age_s'] = now-self.observed_command_time
            status['tracking_error_deg'] = float(np.degrees(error[index]))
            status['tracking_joint'] = NAMES[index]
        return status

    def _publish_status(self):
        status = self._status()
        message = String()
        message.data = json.dumps(status, allow_nan=False)
        self.status_pub.publish(message)
        return status

    def _tick(self):
        now = self._now()
        target = self.experiment.tick(now, self._writers(), self._receivers())
        if self.command_pub is not None and target is not None:
            message = Float64MultiArray()
            message.data = target.tolist()
            self.command_pub.publish(message)
        preview = Float64MultiArray()
        preview.data = self.experiment.preview.tolist()
        self.preview_pub.publish(preview)
        state = self.experiment.state
        if state != self._last_state:
            self.get_logger().info(f'{state}: {self.experiment.reason}')
            self._last_state = state
        if now-self._last_status >= .5:
            self._publish_status()
            self._last_status = now
        if self._csv is not None and now-self._last_record >= 1/self.record_rate:
            status = self._status()
            angles = status['roll_pitch_deg'] or ['', '']
            q = self.experiment.feedback.q
            speed = self.experiment.feedback.velocity
            target = self.experiment.command if self.experiment.command is not None else self.observed_command
            self._csv.writerow([now, state, status['reason'], *angles, *status['gyro_rad_s'],
                                status['accel_norm'], status['calibrated'], status['balance_enabled'],
                                *status['correction_deg'], status['tracking_error_deg'], status['tracking_joint'],
                                status['torque_on'], min(status['voltage_v'].values(), default=''),
                                max(status['temperature_c'].values(), default=''),
                                *(q.tolist() if q is not None else ['']*12),
                                *(target.tolist() if target is not None else ['']*12),
                                *(speed.tolist() if speed is not None else ['']*12),
                                *[status['loads_raw'].get(n, '') for n in NAMES]])
            self._csv_file.flush()
            self._last_record = now

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BalanceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch may forward a second SIGINT while cleanup is in progress.
        with suppress(KeyboardInterrupt, ExternalShutdownException):
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
