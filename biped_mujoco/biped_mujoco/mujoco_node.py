"""mujoco_biped: the BipedRobot V2 in MuJoCo, speaking the same ROS 2 topics as the Gazebo setup.

  listens   /position_controller/commands   Float64MultiArray, 12 joint angles (rad), controller order
            /mujoco/push                    geometry_msgs/WrenchStamped: shove a body (see below)
  services  /mujoco/reset                   std_srvs/Trigger: back to the standing pose
  publishes /clock                          simulation time -> run every other node with use_sim_time:=true
            /joint_states                   sensor_msgs/JointState
            /imu                            sensor_msgs/Imu (frame imu_link; accelerometer includes gravity)
            /mujoco/odom                    nav_msgs/Odometry of the torso in the world frame
            tf  world -> base_link          (robot_state_publisher supplies the rest)

Everything is stamped with SIMULATION time, so the walking node (use_sim_time:=true) sees exactly the clock the
physics runs on, even if this machine can't keep real-time.

External forces:  ros2 topic pub --once /mujoco/push geometry_msgs/msg/WrenchStamped \
                    "{header: {frame_id: base_link}, wrench: {force: {x: 3.0, y: 0.0, z: 0.0}}}"
  Force is in the WORLD frame, applied at the body's centre of mass for `push_duration_s` (default 0.1 s) of
  simulation time.  An empty frame_id means the torso.  In the viewer you can also double-click a body and
  Ctrl+drag to push it with the mouse.
"""
from __future__ import annotations

import contextlib
import math
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, WrenchStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
    HAVE_ROS = True
except ImportError:                      # lets the module be imported (and tested) without ROS
    HAVE_ROS = False
    Node = object

from .sim_core import DEFAULTS, JOINTS, BipedSim


class MujocoBiped(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_biped")
        d = self.declare_parameter
        d("robot_description", "")
        d("package_dir", "")
        d("headless", False)
        d("real_time_factor", 1.0)          # 1 = real time, 0 = as fast as possible
        d("publish_rate_hz", 100.0)
        d("push_duration_s", 0.1)
        d("max_sim_time_s", 0.0)            # 0 = run forever (set for automated tests)
        for k, v in DEFAULTS.items():
            d(k, float(v))
        gp = lambda k: self.get_parameter(k).value                  # noqa: E731

        urdf = gp("robot_description")
        if not urdf:
            raise RuntimeError("parameter robot_description is empty (the launch file passes the xacro output)")
        pkg = gp("package_dir")
        if not pkg:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory("robotv2_description")
        self.sim = BipedSim(urdf, Path(pkg), {k: float(gp(k)) for k in DEFAULTS})
        self.headless = bool(gp("headless"))
        self.rtf = float(gp("real_time_factor"))
        self.pub_period = 1.0 / float(gp("publish_rate_hz"))
        self.push_duration = float(gp("push_duration_s"))
        self.max_sim_time = float(gp("max_sim_time_s"))

        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_odom = self.create_publisher(Odometry, "/mujoco/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "/position_controller/commands", self._on_cmd, 10)
        self.create_subscription(WrenchStamped, "/mujoco/push", self._on_push, 10)
        self.create_service(Trigger, "/mujoco/reset", self._on_reset)
        self.get_logger().info(
            f"MuJoCo model ready: {self.sim.model.nbody - 1} bodies, mass {self.sim.total_mass():.3f} kg, "
            f"torque limit {self.sim.p['stall_torque_nm'] * self.sim.p['torque_fraction']:.2f} N.m/joint, "
            f"kp {self.sim.p['kp']:g}.  Run other nodes with use_sim_time:=true (this node publishes /clock).")

    # ---- inputs ----------------------------------------------------------------------------------------------------
    def _on_cmd(self, msg) -> None:
        q = np.array(msg.data, dtype=float)
        if q.shape != (len(JOINTS),) or not np.all(np.isfinite(q)):
            self.get_logger().warning(f"ignored command: need {len(JOINTS)} finite values, got {list(msg.data)[:14]}")
            return
        self.sim.set_targets(q)

    def _on_push(self, msg) -> None:
        w = msg.wrench
        body = msg.header.frame_id or None
        try:
            self.sim.push((w.force.x, w.force.y, w.force.z), self.push_duration, (w.torque.x, w.torque.y, w.torque.z), body)
            self.get_logger().info(f"push on {body or 'torso'}: ({w.force.x:g}, {w.force.y:g}, {w.force.z:g}) N "
                                   f"for {self.push_duration:g} s")
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _on_reset(self, request, response):
        self.sim.reset()
        response.success, response.message = True, "reset to the standing pose"
        return response

    # ---- outputs ---------------------------------------------------------------------------------------------------
    def _stamp(self):
        t = self.sim.time
        s = Time()
        s.sec = int(math.floor(t))
        s.nanosec = int((t - math.floor(t)) * 1e9)
        return s

    def _publish(self) -> None:
        sim, stamp = self.sim, self._stamp()
        clock = Clock()
        clock.clock = stamp
        self.pub_clock.publish(clock)

        js = JointState()
        js.header.stamp = stamp
        js.name = list(JOINTS)
        js.position = [float(v) for v in sim.joint_positions()]
        js.velocity = [float(v) for v in sim.joint_velocities()]
        js.effort = [float(v) for v in sim.joint_efforts()]
        self.pub_js.publish(js)

        q, gyro, acc = sim.imu()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "imu_link"
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = q
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(v) for v in gyro)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(v) for v in acc)
        imu.orientation_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-2]
        imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.pub_imu.publish(imu)

        pos, quat = sim.base_pose()
        lin, ang = sim.base_velocity()
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = stamp, "world", sim.base_name
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = (float(v) for v in pos)
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = quat
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = (float(v) for v in lin)   # world frame
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = (float(v) for v in ang)  # torso frame
        self.pub_odom.publish(od)

        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "world", sim.base_name
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(v) for v in pos)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = quat
        self.tf.sendTransform(tf)

    # ---- main loop -------------------------------------------------------------------------------------------------
    def run(self) -> None:
        sim = self.sim
        viewer_cm = contextlib.nullcontext(None)
        if not self.headless:
            import mujoco.viewer
            viewer_cm = mujoco.viewer.launch_passive(sim.model, sim.data)
        with viewer_cm as viewer:
            wall0, sim0 = time.perf_counter(), sim.time
            next_pub, next_view = sim.time, 0.0
            while rclpy.ok() and (viewer is None or viewer.is_running()):
                if self.max_sim_time and sim.time >= self.max_sim_time:
                    break
                rclpy.spin_once(self, timeout_sec=0)              # commands, pushes, reset
                now = time.perf_counter()
                target = sim0 + (now - wall0) * self.rtf if self.rtf > 0 else sim.time + 0.02
                if target - sim.time > 0.5:                       # fell far behind real time: don't burst to catch up
                    wall0, sim0, target = now, sim.time, sim.time
                lock = viewer.lock() if viewer is not None else contextlib.nullcontext()
                steps = 0
                with lock:
                    while sim.time < target and steps < 200:
                        sim.step()
                        steps += 1
                        if sim.time >= next_pub:
                            self._publish()
                            next_pub += self.pub_period
                if viewer is not None and now >= next_view:
                    viewer.sync()
                    next_view = now + 1.0 / 60.0
                if self.rtf > 0 and steps == 0:
                    time.sleep(0.0005)


def main(args=None) -> None:
    if not HAVE_ROS:
        sys.exit("ROS 2 Python packages not found: source /opt/ros/<distro>/setup.bash and your workspace first.")
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoBiped()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[mujoco_biped] {exc}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
