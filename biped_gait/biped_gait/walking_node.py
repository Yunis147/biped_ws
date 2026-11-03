"""ROS boundary for the IMU-feedback quasi-static walker. No torque operations."""
import contextlib
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64MultiArray, String
from .feedback_walk import WalkingController, Plans, Settings
from .plan_store import prepare


class QuasiStaticWalker(Node):
    def __init__(self):
        super().__init__('quasi_static_walker')
        self.simulation = bool(self.get_parameter('use_sim_time').value)
        default = Settings()
        settings = Settings(**{field.name: self.declare_parameter(field.name, getattr(default, field.name)).value
                               for field in fields(Settings)})
        source = self.declare_parameter('imu_source', 'auto').value
        mount = self.declare_parameter('imu_mount_rpy', [0., 0., 0.]).value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu_link').value
        self.get_logger().info('Loading precomputed quasi-static paths for this robot and settings ...')
        self.controller = WalkingController(Plans(settings, precomputed=True), simulation=self.simulation, imu_source=source, mount=mount)
        self.plan_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='walking_plan')
        self.preparation = None
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/walk/status', 10)
        self.create_subscription(String, '/walk_cmd', self.command, 10)
        self.create_subscription(Imu, '/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
        self.timer_clock = self.get_clock() if self.simulation else Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=self.timer_clock)
        self.create_timer(.5, self.publish_status, clock=self.timer_clock)
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='restart to change compiled walking/IMU settings'))
        self.last_warning = ''
        self.last_state = None
        self.get_logger().info(f'Ready: IMU feedback required, planned lean {math.degrees(settings.body_roll):.2f} deg, correction limited to {math.degrees(settings.max_correction):.2f} deg. '
                               'No joint commands until stand/start/step; quiet gyro calibration is automatic.')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9 if self.simulation else time.monotonic()

    def age(self, header):
        stamp = header.stamp.sec+header.stamp.nanosec*1e-9
        return self.get_clock().now().nanoseconds*1e-9-stamp if stamp > 0 else float('inf')

    def warn(self, error):
        if str(error) != self.last_warning:
            self.get_logger().warning(str(error))
            self.last_warning = str(error)

    def imu(self, message):
        try:
            if message.header.frame_id != self.imu_frame:
                raise ValueError('IMU frame mismatch: expected '+self.imu_frame)
            if not -.05 <= self.age(message.header) <= .25:
                raise ValueError('stale/invalid IMU source timestamp')
            if message.angular_velocity_covariance[0] < 0 or message.linear_acceleration_covariance[0] < 0:
                raise ValueError('IMU gyro and accelerometer required')
            g, a, q = message.angular_velocity, message.linear_acceleration, message.orientation
            sample_time = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.controller.imu.update([g.x, g.y, g.z], [a.x, a.y, a.z], [q.x, q.y, q.z, q.w],
                self.now(), orientation_valid=message.orientation_covariance[0] >= 0,
                sample_time=sample_time)
            self.controller.imu_problem = ''
        except ValueError as error:
            self.controller.imu_problem = str(error)
            self.warn(error)

    def joints(self, message):
        try:
            self.controller.feedback.joints(message.name, message.position, message.velocity, self.now(), self.age(message.header))
        except ValueError as error:
            self.warn(error)

    def diagnostics(self, message):
        for status in message.status:
            if status.name.rsplit('/', 1)[-1] == 'servo_bridge':
                try:
                    level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
                    values = {v.key: v.value for v in status.values}
                    age = self.age(message.header)
                    self.controller.last_diagnostics = dict(level=level, message=status.message, values=values,
                                                            age_s=age if math.isfinite(age) else None)
                    self.controller.feedback.diagnostics(status.message, status.level, values, self.now(), age)
                except ValueError as error:
                    self.warn(error)

    def check_start(self):
        if self.count_publishers('/position_controller/commands') > 1:
            raise ValueError('another joint-command publisher is active')
        if not self.count_subscribers('/position_controller/commands'):
            raise ValueError('launch your robot bringup or simulator first')
        if self.simulation and self.now() <= 0:
            raise ValueError('waiting for simulation clock')

    def command(self, message):
        try:
            text = message.data.lower().strip()
            if text in ('stop', 'hold') and self.preparation is not None:
                self.preparation[0].cancel()
                self.preparation = None
            if text in ('start', 'forward', 'backward', 'step', 'stand'):
                self.check_start()
                if self.preparation is not None:
                    raise ValueError('path calculation is already in progress')
                c = self.controller
                if not c.fault and c.state in ('idle', 'standing'):
                    problem = c.readiness(self.now())
                    if problem:
                        raise ValueError(problem)
                    direction = 'backward' if text == 'backward' else 'forward'
                    initial = c.nominal.copy()
                    if not np.allclose(c.plans.cache[direction].initial, initial, atol=1e-10, rtol=0):
                        # Keep sensor callbacks and held-pose commands running while
                        # rebuilding from the stance left by a completed step.
                        future = self.plan_worker.submit(prepare, c.s, direction, initial)
                        self.preparation = (future, text, direction, initial)
                        self.get_logger().info('Preparing path; current stance feedback remains active')
                        return
            elif self.preparation is not None and text not in ('status', 'stop', 'hold'):
                raise ValueError('wait for path calculation, or cancel with stop/hold')
            self.controller.command(text, self.now())
        except ValueError as error:
            self.warn(error)
        self.publish_status()

    def tick(self):
        if self.preparation is not None and self.preparation[0].done():
            future, text, direction, initial = self.preparation
            self.preparation = None
            try:
                plan = future.result()
                c = self.controller
                if c.fault or c.state not in ('idle', 'standing') or not np.allclose(initial, c.nominal, atol=1e-10, rtol=0):
                    raise ValueError('prepared path discarded: starting stance/state changed')
                self.check_start()
                c.plans.cache[direction] = plan
                c.command(text, self.now())
            except Exception as error:
                self.warn('path preparation failed: '+str(error))
        if self.controller.state != 'idle' and self.count_publishers('/position_controller/commands') > 1:
            self.controller.fail('another joint-command publisher is active', self.now(), suppress=True)
        target = self.controller.tick(self.now())
        if target is not None:
            message = Float64MultiArray()
            message.data = target.tolist()
            self.pub.publish(message)
        key = (self.controller.state, self.controller.segment, self.controller.reason)
        if key != self.last_state:
            self.get_logger().info(str(key))
            self.last_state = key

    def publish_status(self):
        message = String()
        result = self.controller.status(self.now())
        result['mode'] = 'simulation' if self.simulation else 'hardware'
        result['preparing_path'] = self.preparation is not None
        message.data = json.dumps(result, allow_nan=False)
        self.status_pub.publish(message)

    def destroy_node(self):
        self.plan_worker.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QuasiStaticWalker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):
            if node is not None:
                node.destroy_node()
        with contextlib.suppress(KeyboardInterrupt):
            if rclpy.ok():
                rclpy.shutdown()
