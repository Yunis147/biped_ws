"""ROS publisher for the timed quasi-static position loop."""
import contextlib
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
import json
import math
import time

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, String
from .model_walk import Plans, Settings, WalkingController
from .plan_store import prepare


class QuasiStaticWalker(Node):
    def __init__(self):
        super().__init__('quasi_static_walker')
        self.simulation = bool(self.get_parameter('use_sim_time').value)
        defaults = Settings()
        settings = Settings(**{
            field.name: self.declare_parameter(field.name, getattr(defaults, field.name)).value
            for field in fields(Settings)})
        self.get_logger().info('Loading precomputed walking positions ...')
        self.controller = WalkingController(Plans(settings, precomputed=True))
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='walking_plan')
        self.preparation = None
        self.pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/walk/status', 10)
        self.create_subscription(String, '/walk_cmd', self.command, 10)
        timer_clock = self.get_clock() if self.simulation else Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.tick, clock=timer_clock)
        self.create_timer(.5, self.publish_status, clock=timer_clock)
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=False, reason='restart to load different walking positions'))
        self.last_state = None
        self.get_logger().info(
            f'Ready: timed position loop at 50 Hz; step {settings.step_length*100:.1f} cm, '
            f'foot lift {settings.lift_clearance*100:.1f} cm, '
            f'planned torso roll {math.degrees(settings.body_roll):.2f} deg. '
            'Send stand/start/step to begin.')

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9 if self.simulation else time.monotonic()

    def command(self, message):
        text = message.data.lower().strip()
        c = self.controller
        try:
            if self.preparation is not None:
                if text in ('stop', 'hold'):
                    self.preparation[0].cancel()
                    self.preparation = None
                elif text != 'status':
                    raise ValueError('preparing the next path; use stop to cancel')
            starting = text in ('start', 'forward', 'backward', 'step', 'stand')
            if starting and c.state in ('idle', 'standing') and not (text == 'stand' and c.state == 'standing'):
                direction = 'backward' if text == 'backward' else 'forward'
                initial = c.nominal.copy()
                if not np.allclose(c.plans.cache[direction].initial, initial, atol=1e-10, rtol=0):
                    # A restart after a completed step has different foot spacing.
                    # Continue publishing the standing pose while solving that entry.
                    self.preparation = (self.worker.submit(prepare, c.s, direction, initial), text, direction)
                    self.get_logger().info('Preparing entry from the current standing pose ...')
                    return
            c.command(text, self.now())
        except ValueError as error:
            self.get_logger().warning(str(error))
        self.publish_status()

    def tick(self):
        if self.preparation is not None and self.preparation[0].done():
            future, text, direction = self.preparation
            self.preparation = None
            try:
                self.controller.plans.cache[direction] = future.result()
                self.controller.command(text, self.now())
            except Exception as error:
                self.get_logger().error('Path preparation failed: '+str(error))
        target = self.controller.tick(self.now())
        if target is not None:
            self.pub.publish(Float64MultiArray(data=target.tolist()))
        state = (self.controller.state, self.controller.segment)
        if state != self.last_state:
            self.get_logger().info(str(state))
            self.last_state = state

    def publish_status(self):
        status = self.controller.status()
        status['mode'] = 'simulation' if self.simulation else 'hardware'
        status['preparing_path'] = self.preparation is not None
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))

    def destroy_node(self):
        self.worker.shutdown(wait=False, cancel_futures=True)
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
            if rclpy.ok():
                rclpy.shutdown()
