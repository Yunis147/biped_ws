"""Run the existing MuJoCo class through this package's ROS entry point.

The simulator is reused unchanged as a Python module. This also works when
colcon discovers biped_mujoco as a plain Python package without an ament hook.
"""
import contextlib


def main(args=None):
    try:
        from biped_mujoco.mujoco_node import MujocoBiped
    except ModuleNotFoundError as error:
        if error.name == 'biped_mujoco':
            raise SystemExit('biped_mujoco is missing; build --packages-up-to biped_cpg and source install/setup.bash') from error
        raise
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoBiped()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with contextlib.suppress(KeyboardInterrupt):
            if node is not None:
                node.destroy_node()
        with contextlib.suppress(KeyboardInterrupt):
            if rclpy.ok():
                rclpy.shutdown()
