"""ROS 2 interface for simulation CPG walking. No sensor subscriptions."""
import contextlib
from dataclasses import fields
import json
import sys

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

    class CPGNode(Node):
        def __init__(self):
            super().__init__('biped_cpg')
            if not self.get_parameter('use_sim_time').value:
                raise ValueError('this experiment requires use_sim_time:=true; start a simulator first')
            default = Settings()
            values = {field.name: self.declare_parameter(field.name, getattr(default, field.name)).value
                      for field in fields(Settings)}
            settings = Settings(**values)
            self.get_logger().info('Compiling and checking small-lean CPG trajectories ...')
            self.walker = Walker(Library(settings))
            for plan in self.walker.library.plans.values():
                self.get_logger().info(json.dumps(plan.summary()))
            self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
            self.status_pub = self.create_publisher(String, '/cpg/status', 10)
            self.create_subscription(String, '/cpg_cmd', self.command, 10)
            self.create_timer(1/Walker.rate_hz, self.tick)
            self.create_timer(.5, self.publish_status)
            self.add_on_set_parameters_callback(lambda params: SetParametersResult(
                successful=False, reason='restart the CPG node to change compiled gait settings'))
            self.previous_state = 'idle'
            self.get_logger().info('Ready; no joint commands until /cpg_cmd receives start, forward, backward or step.')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def other_writer(self):
            # This node owns exactly one publisher, including while idle.
            return self.count_publishers('/position_controller/commands') > 1

        def command(self, message):
            command = message.data.strip().lower()
            try:
                if command in ('start', 'forward', 'backward', 'step'):
                    if self.now() <= 0:
                        raise ValueError('waiting for a running simulation /clock')
                    if self.other_writer():
                        raise ValueError('another joint-command publisher is running; stop the other walker first')
                    if not self.count_subscribers('/position_controller/commands'):
                        raise ValueError('no simulator is subscribed to joint commands')
                reply = self.walker.command(command, self.now())
                self.get_logger().info(reply)
            except ValueError as error:
                self.get_logger().error(str(error))
            self.publish_status()

        def tick(self):
            if self.walker.active and self.other_writer():
                self.walker.fail('another joint-command publisher appeared', suppress=True)
            output = self.walker.tick(self.now())
            if output is not None:
                message = Float64MultiArray()
                message.data = output.tolist()
                self.pub.publish(message)
            if self.walker.state != self.previous_state:
                self.get_logger().info(f'State: {self.walker.state}' +
                                       (f' ({self.walker.fault})' if self.walker.fault else ''))
                self.previous_state = self.walker.state

        def publish_status(self):
            message = String()
            message.data = json.dumps(self.walker.status(), allow_nan=False)
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
