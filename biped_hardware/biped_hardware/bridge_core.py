"""BridgeCore: the safety logic between "12 joint angles in radians" and "12 servos".
No ROS in here, so it can be tested without a robot.

Every cycle (`update`):
  1. read all 12 servos (position, load, voltage, temperature)
  2. while torque is ON: take the newest commanded pose
     -> clamp to the soft joint limits
     -> limit how fast it may change (slew limit)
     -> convert to servo steps
     -> write all 12 at once.

Safety behaviours:
  * Torque is OFF at start-up. Turning it on first sets every servo's target to where
    it already is, so nothing jumps; then the first commanded pose is approached slowly.
  * Commands are clamped to the joint limits and rate-limited, so a bad command
    cannot slam a joint.
  * If normal commands stop arriving, the last pose is held.
  * Startup positioning is exempt from the normal command watchdog until the
    calibrated midpoint is reached.
"""

from __future__ import annotations

import logging
import math

import numpy as np

from .config import Config
from .serial_bus import SerialBusError
from .servo_array import ServoArray

log = logging.getLogger(__name__)


class BridgeCore:
    def __init__(self, arr: ServoArray, cfg: Config) -> None:
        self.arr, self.cfg = arr, cfg
        self.names = cfg.names
        self.n = len(self.names)

        s = cfg.safety
        margin = float(s.get("limit_margin_rad", 0.02))

        self.lo = np.array(
            [j.lower + margin for j in cfg.joints]
        )
        self.hi = np.array(
            [j.upper - margin for j in cfg.joints]
        )

        self.max_speed = float(
            s.get("max_joint_speed_rad_s", 1.2)
        )
        self.soft_speed = float(
            s.get("soft_start_speed_rad_s", 0.35)
        )

        self.servo_speed = int(
            s.get("servo_speed_cap_steps_s", 1500)
        )
        self.servo_accel = int(
            s.get("servo_accel", 30)
        )
        self.torque_limit = int(
            s.get("torque_limit", 500)
        )

        self.cmd_timeout = float(
            cfg.loop.get("command_timeout_s", 0.5)
        )

        self.max_temp = float(
            s.get("max_temperature_c", 65)
        )
        self.min_volt = float(
            s.get("min_voltage_v", 6.0)
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------

        self.meas = None
        self.vel = np.zeros(self.n)

        self.status = {
            n: None for n in self.names
        }

        # Latest commanded pose.
        self.cmd_target = None

        # Time at which cmd_target was received.
        self.cmd_time = None

        # Actual pose we are currently sending to the servos.
        self.cmd_out = None

        # Torque state.
        self.torque = False

        # True while the soft-start speed limit is active.
        self.soft_start = False

        # True when normal command watchdog has triggered.
        self.watchdog = False

        # --------------------------------------------------------------
        # IMPORTANT:
        #
        # Startup midpoint movement is an internally generated command.
        # It must NOT be stopped by the normal 0.5 s command watchdog.
        # --------------------------------------------------------------
        self.startup_positioning = False

        self.read_failures = 0
        self._t_prev = None

        self.warnings: list[str] = []

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def set_command(self, values, now: float) -> bool:
        """Accept a new commanded pose (12 radians, controller order)."""

        if len(values) != self.n:
            self._warn(
                f"command has {len(values)} values, "
                f"expected {self.n}: ignored"
            )
            return False

        arr = np.array(values, dtype=float)

        if not np.all(np.isfinite(arr)):
            self._warn(
                "command contains NaN/inf: ignored"
            )
            return False

        self.cmd_target = arr
        self.cmd_time = now

        return True

    # ------------------------------------------------------------------
    # Torque
    # ------------------------------------------------------------------

    def enable_torque(self) -> tuple[bool, str]:
        """Enable torque while first holding the current physical pose."""

        if self.torque:
            return True, "torque already on"

        try:
            # First command every servo to its current position.
            # This prevents a jump when torque is enabled.
            steps = self.arr.hold_current(
                self.servo_speed,
                self.servo_accel,
            )

        except SerialBusError as exc:
            return (
                False,
                f"refusing to enable torque: {exc}",
            )

        self.meas = np.array(
            [
                j.steps_to_rad(steps[j.name])
                for j in self.cfg.joints
            ]
        )

        # Current physical pose becomes our starting output.
        self.cmd_out = self.meas.copy()

        self.arr.set_torque_limit(
            self.torque_limit
        )

        self.arr.set_torque(True)

        self.torque = True
        self.soft_start = True
        self.watchdog = False

        return (
            True,
            (
                f"torque ON "
                f"(limit {self.torque_limit}/1000). "
                f"Holding the current pose; the commanded pose "
                f"will be approached at "
                f"{self.soft_speed:.2f} rad/s"
            ),
        )

    def disable_torque(self) -> tuple[bool, str]:
        """Disable torque on all servos."""

        try:
            self.arr.set_torque(False)

        except SerialBusError as exc:
            return (
                False,
                f"could not disable torque: {exc}",
            )

        self.torque = False

        return (
            True,
            "torque OFF - the robot is limp",
        )

    # ------------------------------------------------------------------
    # One control cycle
    # ------------------------------------------------------------------

    def update(self, now: float) -> None:
        dt = (
            0.02
            if self._t_prev is None
            else min(
                max(now - self._t_prev, 1e-3),
                0.2,
            )
        )

        self._t_prev = now

        self._read(dt)

        if self.torque and self.cmd_out is not None:
            self._write(now, dt)

    # ------------------------------------------------------------------
    # Read servos
    # ------------------------------------------------------------------

    def _read(self, dt: float) -> None:
        status = self.arr.read_status()

        self.status = status

        silent = [
            n
            for n, s in status.items()
            if s is None
        ]

        self.read_failures = (
            self.read_failures + 1
            if silent
            else 0
        )

        if len(silent) == self.n:
            return

        new = np.array(
            [
                (
                    j.steps_to_rad(
                        status[j.name].steps
                    )
                    if status[j.name] is not None
                    else (
                        self.meas[i]
                        if self.meas is not None
                        else 0.0
                    )
                )
                for i, j in enumerate(self.cfg.joints)
            ]
        )

        if self.meas is not None:
            raw_vel = (
                new - self.meas
            ) / dt

            self.vel = (
                0.6 * self.vel
                + 0.4 * raw_vel
            )

        self.meas = new

    # ------------------------------------------------------------------
    # Write servos
    # ------------------------------------------------------------------

    def _write(self, now: float, dt: float) -> None:

        # --------------------------------------------------------------
        # Select target
        # --------------------------------------------------------------

        if self.cmd_target is None:

            # Nothing commanded yet.
            # Keep holding current output.
            target = self.cmd_out

        elif (
            now - self.cmd_time > self.cmd_timeout
            and not self.startup_positioning
        ):

            # ----------------------------------------------------------
            # NORMAL COMMAND WATCHDOG
            #
            # This applies to walking / external ROS commands.
            #
            # If commands stop arriving for > 0.5 s, hold the last pose.
            # ----------------------------------------------------------

            target = self.cmd_out

            if not self.watchdog:
                self.watchdog = True

                self._warn(
                    f"no command for "
                    f"{self.cmd_timeout:.1f} s: "
                    f"holding the last pose"
                )

        else:

            # ----------------------------------------------------------
            # Normal command OR startup midpoint command.
            #
            # Startup positioning is intentionally allowed to continue
            # beyond command_timeout because it is an internal command.
            # ----------------------------------------------------------

            target = self.cmd_target

            self.watchdog = False

        # --------------------------------------------------------------
        # Clamp target to safe joint limits
        # --------------------------------------------------------------

        target = np.clip(
            target,
            self.lo,
            self.hi,
        )

        # --------------------------------------------------------------
        # Slew-rate limiting
        # --------------------------------------------------------------

        limit = (
            self.soft_speed
            if self.soft_start
            else self.max_speed
        )

        step = limit * dt

        self.cmd_out = (
            self.cmd_out
            + np.clip(
                target - self.cmd_out,
                -step,
                step,
            )
        )

        # --------------------------------------------------------------
        # Detect target reached
        # --------------------------------------------------------------

        if (
            self.soft_start
            and self.cmd_target is not None
            and not self.watchdog
            and float(
                np.max(
                    np.abs(
                        target - self.cmd_out
                    )
                )
            ) < 0.01
        ):

            self.soft_start = False

            # If this was startup positioning,
            # startup mode is now complete.
            if self.startup_positioning:
                self.startup_positioning = False

                log.info(
                    "startup midpoint reached: "
                    "soft-start finished, "
                    "normal command watchdog is now active"
                )
            else:
                log.info(
                    "commanded pose reached: "
                    "soft-start finished, speed limit "
                    "now %.2f rad/s",
                    self.max_speed,
                )

        # --------------------------------------------------------------
        # Convert radians -> servo encoder steps
        # --------------------------------------------------------------

        steps = {
            j.name: j.rad_to_steps(
                float(self.cmd_out[i])
            )
            for i, j in enumerate(self.cfg.joints)
        }

        # --------------------------------------------------------------
        # Write all servos
        # --------------------------------------------------------------

        try:
            self.arr.write_positions(
                steps,
                self.servo_speed,
                self.servo_accel,
            )

        except SerialBusError as exc:
            self._warn(
                f"write failed: {exc}"
            )

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def health(
        self,
    ) -> list[tuple[str, str, str]]:
        """Return [(joint, level, text)]."""

        out = []

        for j in self.cfg.joints:

            s = self.status.get(j.name)

            if s is None:
                out.append(
                    (
                        j.name,
                        "ERROR",
                        (
                            f"servo {j.servo_id} "
                            f"not answering"
                        ),
                    )
                )
                continue

            level = "OK"

            text = (
                f"id {j.servo_id}  "
                f"{s.temp_c} C  "
                f"{s.voltage_v:.1f} V  "
                f"load {s.load}"
            )

            if s.temp_c >= self.max_temp:
                level = "WARN"
                text += "  TOO HOT"

            if s.voltage_v < self.min_volt:
                level = "WARN"
                text += "  LOW VOLTAGE"

            out.append(
                (
                    j.name,
                    level,
                    text,
                )
            )

        return out

    # ------------------------------------------------------------------
    # Warning helper
    # ------------------------------------------------------------------

    def _warn(self, msg: str) -> None:
        self.warnings.append(msg)
        log.warning(msg)