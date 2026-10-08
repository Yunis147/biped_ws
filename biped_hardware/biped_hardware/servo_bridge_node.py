"""servo_bridge: ROS 2 side of the real robot.

Startup modes:

1. startup_position=false:
   - torque remains OFF
   - servos are only read

2. startup_position=true:
   - torque is enabled safely
   - current pose is captured first
   - robot smoothly moves to calibrated zero/midpoint
   - startup movement is NOT interrupted by the normal command watchdog
   - after reaching midpoint, normal command watchdog becomes active
   - torque remains ON and holds that position
"""

from __future__ import annotations

import sys

try:
    import rclpy
    from diagnostic_msgs.msg import (
        DiagnosticArray,
        DiagnosticStatus,
        KeyValue,
    )
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import SetBool

    HAVE_ROS = True

except ImportError:
    HAVE_ROS = False
    Node = object

from .bridge_core import BridgeCore
from .config import load_config
from .bus_factory import open_bus
from .servo_array import ServoArray


if HAVE_ROS:

    class ServoBridge(Node):

        def __init__(self) -> None:
            super().__init__("servo_bridge")

            # --------------------------------------------------------------
            # Parameters
            # --------------------------------------------------------------

            self.declare_parameter(
                "mock",
                False,
            )

            self.declare_parameter(
                "port",
                "",
            )

            self.declare_parameter(
                "torque_on_start",
                False,
            )

            self.declare_parameter(
                "startup_position",
                False,
            )

            self.declare_parameter(
                "hardware_config",
                "",
            )

            self.declare_parameter(
                "servos_config",
                "",
            )

            self.declare_parameter(
                "calibration_file",
                "",
            )

            gp = lambda k: self.get_parameter(k).value

            # --------------------------------------------------------------
            # Configuration
            # --------------------------------------------------------------

            self.cfg = load_config(
                gp("hardware_config") or None,
                gp("servos_config") or None,
                gp("calibration_file") or None,
            )

            log = self.get_logger()

            if self.cfg.calibration_loaded:
                log.info(
                    f"calibration loaded from "
                    f"{self.cfg.calibration_path}"
                )
            else:
                log.warning(
                    "no calibration file loaded"
                )

            # --------------------------------------------------------------
            # Hardware
            # --------------------------------------------------------------

            self.mock = bool(
                gp("mock")
            )

            requested_port = (
                gp("port")
                if gp("port")
                else None
            )

            self.bus = open_bus(
                self.cfg,
                port=requested_port,
                mock=self.mock,
            )

            self.arr = ServoArray(
                self.bus,
                self.cfg,
            )

            # --------------------------------------------------------------
            # Verify servos
            # --------------------------------------------------------------

            missing = []

            for joint in self.cfg.joints:

                try:
                    ok = self.arr.ping(
                        joint.servo_id
                    )

                except Exception:
                    ok = False

                if not ok:
                    missing.append(
                        f"{joint.name}(id={joint.servo_id})"
                    )

            if missing:
                raise RuntimeError(
                    "Missing servos: "
                    + ", ".join(missing)
                )

            log.info(
                "All configured servos detected."
            )

            # --------------------------------------------------------------
            # Bridge core
            # --------------------------------------------------------------

            self.core = BridgeCore(
                self.arr,
                self.cfg,
            )

            # --------------------------------------------------------------
            # ROS publishers
            # --------------------------------------------------------------

            self.joint_pub = self.create_publisher(
                JointState,
                "/joint_states",
                10,
            )

            self.diag_pub = self.create_publisher(
                DiagnosticArray,
                "/diagnostics",
                10,
            )

            # --------------------------------------------------------------
            # ROS subscribers
            # --------------------------------------------------------------

            self.cmd_sub = self.create_subscription(
                Float64MultiArray,
                "/position_controller/commands",
                self._on_cmd,
                10,
            )

            # --------------------------------------------------------------
            # Torque service
            # --------------------------------------------------------------

            self.torque_srv = self.create_service(
                SetBool,
                "~/torque",
                self._on_torque,
            )

            # --------------------------------------------------------------
            # Control timer
            # --------------------------------------------------------------

            rate_hz = float(
                self.cfg.loop.get(
                    "rate_hz",
                    50,
                )
            )

            self.timer = self.create_timer(
                1.0 / rate_hz,
                self._update,
            )

            # Diagnostics timer
            self.diag_timer = self.create_timer(
                1.0,
                self._diagnostics,
            )

            self._started_at = self._now()

            # --------------------------------------------------------------
            # Startup behavior
            # --------------------------------------------------------------

            startup_position = bool(
                gp("startup_position")
            )

            torque_on_start = bool(
                gp("torque_on_start")
            )

            # ==============================================================
            # STARTUP POSITIONING
            # ==============================================================

            if startup_position:

                log.info(
                    "startup_position=true: "
                    "enabling torque and moving "
                    "to calibrated midpoint."
                )

                # ----------------------------------------------------------
                # Enable torque safely.
                #
                # BridgeCore first captures the current physical position
                # and commands every servo to hold that position.
                # ----------------------------------------------------------

                ok, msg = self.core.enable_torque()

                if not ok:
                    log.error(msg)

                    raise RuntimeError(
                        "Could not enable torque "
                        "for startup positioning."
                    )

                log.info(msg)

                # ----------------------------------------------------------
                # Build calibrated midpoint pose.
                #
                # zero_steps is the calibrated servo encoder position.
                # Convert those encoder values into URDF radians.
                # ----------------------------------------------------------

                midpoint = [
                    joint.steps_to_rad(
                        joint.zero_steps
                    )
                    for joint in self.cfg.joints
                ]

                # ----------------------------------------------------------
                # IMPORTANT FIX
                #
                # This is an INTERNAL startup command.
                #
                # It must not be stopped by the normal
                # command_timeout_s watchdog after 0.5 seconds.
                # ----------------------------------------------------------

                self.core.startup_positioning = True

                accepted = self.core.set_command(
                    midpoint,
                    self._now(),
                )

                if not accepted:
                    self.core.startup_positioning = False

                    raise RuntimeError(
                        "Startup midpoint command "
                        "was rejected."
                    )

                log.info(
                    "Startup midpoint command accepted. "
                    "Robot will move there using "
                    "the soft-start limit."
                )

            # ==============================================================
            # NORMAL TORQUE-ON START
            # ==============================================================

            elif torque_on_start:

                ok, msg = self.core.enable_torque()

                if not ok:
                    log.error(msg)

                else:
                    log.info(msg)

            # --------------------------------------------------------------
            # Final startup status
            # --------------------------------------------------------------

            log.info(
                f"servo_bridge ready "
                f"({'MOCK, no hardware' if self.mock else 'real servos'}); "
                f"torque is "
                f"{'ON' if self.core.torque else 'OFF'}."
            )

        # ------------------------------------------------------------------
        # Time
        # ------------------------------------------------------------------

        def _now(self) -> float:
            return (
                self.get_clock().now().nanoseconds
                * 1e-9
            )

        # ------------------------------------------------------------------
        # Position commands
        # ------------------------------------------------------------------

        def _on_cmd(
            self,
            msg,
        ) -> None:

            self.core.set_command(
                list(msg.data),
                self._now(),
            )

        # ------------------------------------------------------------------
        # Torque service
        # ------------------------------------------------------------------

        def _on_torque(
            self,
            request,
            response,
        ):

            if request.data:

                ok, msg = (
                    self.core.enable_torque()
                )

            else:

                ok, msg = (
                    self.core.disable_torque()
                )

                # If torque is manually disabled,
                # startup positioning is no longer active.
                self.core.startup_positioning = False

            response.success = ok
            response.message = msg

            return response

        # ------------------------------------------------------------------
        # Main update loop
        # ------------------------------------------------------------------

        def _update(self) -> None:

            now = self._now()

            self.core.update(
                now
            )

            self._publish_joint_states()

        # ------------------------------------------------------------------
        # Joint states
        # ------------------------------------------------------------------

        def _publish_joint_states(self) -> None:

            if self.core.meas is None:
                return

            msg = JointState()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.name = list(
                self.cfg.names
            )

            msg.position = [
                float(x)
                for x in self.core.meas
            ]

            msg.velocity = [
                float(x)
                for x in self.core.vel
            ]

            self.joint_pub.publish(
                msg
            )

        # ------------------------------------------------------------------
        # Diagnostics
        # ------------------------------------------------------------------

        def _diagnostics(self) -> None:

            msg = DiagnosticArray()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            status = DiagnosticStatus()

            status.name = "servo_bridge"

            status.hardware_id = (
                "biped"
            )

            if self.core.torque:
                status.level = (
                    DiagnosticStatus.OK
                )
                status.message = (
                    "torque ON"
                )
            else:
                status.level = (
                    DiagnosticStatus.OK
                )
                status.message = (
                    "torque OFF"
                )

            status.values.append(
                KeyValue(
                    key="startup_positioning",
                    value=str(
                        self.core.startup_positioning
                    ),
                )
            )

            status.values.append(
                KeyValue(
                    key="watchdog",
                    value=str(
                        self.core.watchdog
                    ),
                )
            )

            status.values.append(
                KeyValue(
                    key="soft_start",
                    value=str(
                        self.core.soft_start
                    ),
                )
            )

            # ----------------------------------------------------------
            # Servo health
            # ----------------------------------------------------------

            for name, level, text in (
                self.core.health()
            ):

                status.values.append(
                    KeyValue(
                        key=name,
                        value=text,
                    )
                )

                if level == "ERROR":
                    status.level = (
                        DiagnosticStatus.ERROR
                    )

                elif (
                    level == "WARN"
                    and status.level
                    != DiagnosticStatus.ERROR
                ):
                    status.level = (
                        DiagnosticStatus.WARN
                    )

            # ----------------------------------------------------------
            # Warnings
            # ----------------------------------------------------------

            if self.core.warnings:

                status.values.append(
                    KeyValue(
                        key="warnings",
                        value=" | ".join(
                            self.core.warnings[-5:]
                        ),
                    )
                )

            msg.status.append(
                status
            )

            self.diag_pub.publish(
                msg
            )

        # ------------------------------------------------------------------
        # Shutdown
        # ------------------------------------------------------------------

        def destroy_node(self):

            # ----------------------------------------------------------
            # IMPORTANT:
            # torque_off_on_shutdown is handled here so Ctrl+C
            # disables torque before the node exits.
            # ----------------------------------------------------------

            torque_off = bool(
                self.cfg.safety.get(
                    "torque_off_on_shutdown",
                    False,
                )
            )

            if (
                torque_off
                and self.core.torque
            ):

                self.get_logger().info(
                    "Shutdown requested: "
                    "disabling servo torque."
                )

                ok, msg = (
                    self.core.disable_torque()
                )

                if ok:
                    self.get_logger().info(
                        msg
                    )
                else:
                    self.get_logger().error(
                        msg
                    )

            else:

                # Ensure startup mode is cleared.
                self.core.startup_positioning = False

            # ----------------------------------------------------------
            # Close hardware bus
            # ----------------------------------------------------------

            try:
                self.bus.close()

            except Exception as exc:
                self.get_logger().warning(
                    f"error closing bus: {exc}"
                )

            super().destroy_node()


def main(args=None):

    if not HAVE_ROS:
        print(
            "ROS 2 Python dependencies are not available.",
            file=sys.stderr,
        )
        return 1

    rclpy.init(
        args=args
    )

    node = None

    try:

        node = ServoBridge()

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:

            try:
                node.destroy_node()

            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )