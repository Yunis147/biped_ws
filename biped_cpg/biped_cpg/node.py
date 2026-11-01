"""ROS 2 interface for hardware or simulation CPG walking. No IMU feedback."""
import contextlib
from dataclasses import fields
import json
import sys
import time

from .controller import Library, Walker
from .planner import Settings


def main(args=None):
    if '--check' in sys.argv:
        library = Library()
        for plan in library.plans.values():
            print(json.dumps(plan.summary(), indent=2))
        return
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rcl_interfaces.msg import SetParametersResult
    from std_msgs.msg import Float64MultiArray, String
    from sensor_msgs.msg import JointState
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.clock import Clock
    from rclpy.clock_type import ClockType
    from rclpy.qos import qos_profile_sensor_data
    from .hardware import HardwareWalker, HardwareLimits

    class CPGNode(Node):
        def __init__(self):
            super().__init__('biped_cpg')
            self.simulation = bool(self.get_parameter('use_sim_time').value)
            default = Settings()
            values = {field.name: self.declare_parameter(field.name, getattr(default, field.name)).value
                      for field in fields(Settings)}
            settings = Settings(**values)
            self.get_logger().info('Compiling and checking small-lean CPG trajectories ...')
            self.walker = Walker(Library(settings))
            self.controller = self.walker
            self.input_error = ''
            if not self.simulation:
                default_limits = HardwareLimits()
                limits = HardwareLimits(**{field.name: self.declare_parameter(
                    'hardware_'+field.name, getattr(default_limits, field.name)).value
                    for field in fields(HardwareLimits)})
                self.controller = HardwareWalker(self.walker, limits)
                self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
                self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
            self.timer_clock = self.get_clock() if self.simulation else Clock(clock_type=ClockType.STEADY_TIME)
            for plan in self.walker.library.plans.values():
                self.get_logger().info(json.dumps(plan.summary()))
            self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
            self.status_pub = self.create_publisher(String, '/cpg/status', 10)
            self.create_subscription(String, '/cpg_cmd', self.command, 10)
            self.create_timer(1/Walker.rate_hz, self.tick, clock=self.timer_clock)
            self.create_timer(.5, self.publish_status, clock=self.timer_clock)
            self.add_on_set_parameters_callback(lambda params: SetParametersResult(
                successful=False, reason='restart the CPG node to change compiled gait settings'))
            self.previous_state = 'idle'
            self.get_logger().info(('SIMULATION' if self.simulation else 'HARDWARE') + ': Ready; no joint commands until /cpg_cmd receives start, forward, backward or step.')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9 if self.simulation else time.monotonic()

        def stamp_age(self, header):
            stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            return self.get_clock().now().nanoseconds * 1e-9 - stamp if stamp > 0 else float('inf')

        def invalid_input(self, error):
            if str(error) != self.input_error:
                self.get_logger().warning(str(error))
                self.input_error = str(error)

        def joints(self, message):
            try:
                self.controller.feedback.joints(message.name, message.position, message.velocity,
                                                self.now(), self.stamp_age(message.header))
            except ValueError as error:
                self.invalid_input(error)

        def diagnostics(self, message):
            for status in message.status:
                if status.name.rsplit('/', 1)[-1] == 'servo_bridge':
                    try:
                        self.controller.feedback.diagnostics(status.message, status.level,
                            {v.key: v.value for v in status.values}, self.now(), self.stamp_age(message.header))
                    except ValueError as error:
                        self.invalid_input(error)

        def status(self):
            result = self.walker.status() if self.simulation else self.controller.status(self.now())
            result['mode'] = 'simulation' if self.simulation else 'hardware'
            return result

        def other_writer(self):
            # This node owns exactly one publisher, including while idle.
            return self.count_publishers('/position_controller/commands') > 1

        def command(self, message):
            command = message.data.strip().lower()
            try:
                if command in ('start', 'forward', 'backward', 'step'):
                    if self.simulation and self.now() <= 0:
                        raise ValueError('waiting for a running simulation /clock')
                    if self.other_writer():
                        raise ValueError('another joint-command publisher is running; stop the other walker first')
                    if not self.count_subscribers('/position_controller/commands'):
                        raise ValueError('no subscriber on joint commands; launch bringup or a simulator first')
                reply = self.controller.command(command, self.now())
                self.get_logger().info(reply)
            except ValueError as error:
                self.get_logger().error(str(error))
            self.publish_status()

        def tick(self):
            if self.controller.active and self.other_writer():
                self.controller.fail('another joint-command publisher appeared', suppress=True)
            output = self.controller.tick(self.now())
            if output is not None:
                message = Float64MultiArray()
                message.data = output.tolist()
                self.pub.publish(message)
            state = self.status()['state']
            if state != self.previous_state:
                self.get_logger().info(f'State: {state}' +
                                       (f' ({self.walker.fault})' if self.walker.fault else ''))
                self.previous_state = state

        def publish_status(self):
            message = String()
            message.data = json.dumps(self.status(), allow_nan=False)
            self.status_pub.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = CPGNode()
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


if __name__ == '__main__':
    main()
