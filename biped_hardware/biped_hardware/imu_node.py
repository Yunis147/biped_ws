"""imu_node: publishes the BNO055 as sensor_msgs/Imu on /imu — the same topic and frame the Gazebo IMU uses."""
from __future__ import annotations

import sys

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt8MultiArray
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False
    Node = object

from .bno055 import BNO055, MockBNO055
from .config import load_config


class ImuNode(Node):
    def __init__(self, sensor=None):
        super().__init__("imu_node")
        self.declare_parameter("mock", False)
        self.declare_parameter("hardware_config", "")
        cfg = load_config(self.get_parameter("hardware_config").value or None).imu
        self.declare_parameter("i2c_bus", int(cfg.get("i2c_bus", 7)))
        self.declare_parameter("address", int(cfg.get("address", 0x28)))
        self.declare_parameter("mode", str(cfg.get("mode", "imuplus")))
        self.declare_parameter("rate_hz", float(cfg.get("rate_hz", 100)))
        self.declare_parameter("frame_id", str(cfg.get("frame_id", "imu_link")))
        gp = lambda k: self.get_parameter(k).value                      # noqa: E731

        self.frame_id = gp("frame_id")
        if sensor is not None:
            self.sensor = sensor
        elif gp("mock"):
            self.sensor = MockBNO055()
        else:
            self.sensor = BNO055(int(gp("i2c_bus")), int(gp("address")), gp("mode"))
        self.sensor.initialize()                     # raises with a clear message if the chip is not found
        self.pub = self.create_publisher(Imu, "/imu", 10)
        self.pub_cal = self.create_publisher(UInt8MultiArray, "/imu/calibration", 10)
        self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
        self.get_logger().info(f"imu_node publishing /imu at {gp('rate_hz')} Hz, frame '{self.frame_id}'")
        self._cal_warned = False

    def _tick(self) -> None:
        r = self.sensor.read()
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = r.quaternion
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = r.gyro
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = r.accel
        m.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-2]
        m.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        m.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.pub.publish(m)
        c = UInt8MultiArray()
        c.data = [int(v) for v in r.calibration]          # sys, gyro, accel, mag  (3 = fully calibrated)
        self.pub_cal.publish(c)

    def shutdown(self) -> None:
        self.sensor.close()


def main(args=None):
    if not HAVE_ROS:
        sys.exit("ROS 2 Python packages not found: source /opt/ros/<distro>/setup.bash and your workspace first.")
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[imu_node] {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
