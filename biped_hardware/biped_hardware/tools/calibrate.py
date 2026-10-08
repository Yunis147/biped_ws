"""
biped_calibrate — calibrate the robot.

This version supports TWO independent servo buses/boards.

Current hardware arrangement:
    /dev/ttyACM0 -> right leg -> IDs 7,8,9,10,11,12
    /dev/ttyACM1 -> left leg  -> IDs 2,3,4,5,6,13

The board is discovered automatically from which servo IDs answer.

Stop servo_bridge first (this talks to the servos directly).

Commands:

    ros2 run biped_hardware biped_calibrate zero
    ros2 run biped_hardware biped_calibrate zero --method software

    ros2 run biped_hardware biped_calibrate directions

    ros2 run biped_hardware biped_calibrate ranges

    ros2 run biped_hardware biped_calibrate show

Results go to ~/.config/biped/calibration.yaml
(the previous file is kept as .bak).
"""

from __future__ import annotations

import argparse
import copy
import sys
import time

from ..config import STEPS_PER_RAD, Config, save_calibration
from .common import add_common_args, ask, say, setup


# ============================================================================
# TWO SERVO BOARDS
# ============================================================================

BOARD_PORTS = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
]


# ============================================================================
# ZERO POSE DESCRIPTION
# ============================================================================

ZERO_POSE_TEXT = """\
ZERO POSE  (every joint at 0 rad — this is the pose the walking code calls 'standing'):

  * both legs STRAIGHT (knees fully straight, not locked hard)
  * both feet FLAT on the ground, pointing straight FORWARD, parallel to each other
  * thighs vertical, hips level, torso upright and not twisted
  * the robot symmetric left/right (no leaning)

Easiest: stand the robot on a flat table, hold it upright by the torso, and use a square or a printed
foot outline to line the feet up.  Torque is OFF now, so the joints move freely."""


# ============================================================================
# DIRECTION TEST DEFINITIONS
# ============================================================================

# What a POSITIVE URDF angle should physically do.
#
# The knee is intentionally tested in the NEGATIVE direction because that is
# the normal mechanically useful bending direction.

TESTS = {
    "hip_yaw": (
        +1,
        "the FOOT TURNS OUTWARD (toes point away from the other leg)",
    ),

    "hip_roll": (
        +1,
        "the whole LEG SWINGS SIDEWAYS, OUTWARD (foot moves away from the other leg)",
    ),

    "hip_pitch": (
        +1,
        "the LEG SWINGS BACKWARD (foot moves behind the robot)",
    ),

    "knee": (
        -1,
        "the KNEE BENDS the normal way (lower leg swings backward, heel toward the back)",
    ),

    "ankle_pitch": (
        +1,
        "the TOES GO DOWN (heel goes up)",
    ),

    "ankle_roll": (
        +1,
        "the foot TILTS so its OUTER edge goes UP (sole turns to face inward)",
    ),
}


TEST_ORDER = [
    "hip_yaw",
    "hip_roll",
    "hip_pitch",
    "knee",
    "ankle_pitch",
    "ankle_roll",
]


# ============================================================================
# HELPERS
# ============================================================================

def joint_kind(name: str) -> str:
    """
    Convert:

        r_hip_pitch_joint -> hip_pitch
        l_knee_joint      -> knee

    """
    base = name[2:]
    base = base.replace("_joint", "")
    return base


def _table(rows, header, out=say) -> None:
    if not rows:
        out("  " + "  ".join(str(h) for h in header))
        return

    widths = [
        max(len(str(r[i])) for r in [header] + rows)
        for i in range(len(header))
    ]

    out(
        "  "
        + "  ".join(
            str(h).ljust(w)
            for h, w in zip(header, widths)
        )
    )

    for r in rows:
        out(
            "  "
            + "  ".join(
                str(c).ljust(w)
                for c, w in zip(r, widths)
            )
        )


# ============================================================================
# TWO-BOARD SERVO ARRAY
# ============================================================================

class DualServoArray:
    """
    Wrapper around two independent ServoArray objects.

    The important part is that every ServoArray operation is executed with
    ONLY the joints that physically exist on that board.

    This prevents a ServoArray on ACM1 from trying to communicate with
    right-leg servos that physically live on ACM0.
    """

    def __init__(self, cfg: Config, arrays):
        self.cfg = cfg
        self.arrays = arrays

        # servo_id -> board index
        #
        # Example:
        #   7  -> 0
        #   8  -> 0
        #   ...
        #   2  -> 1
        #   3  -> 1
        self.servo_bus: dict[int, int] = {}

        # board index -> joints on that board
        self.bus_joints: dict[int, list] = {}

        self._discover_buses()

    # ------------------------------------------------------------------------
    # Run a ServoArray method using only the joints belonging to that board.
    # ------------------------------------------------------------------------

    def _filtered_call(self, bus_index: int, method: str, *args, **kwargs):
        arr = self.arrays[bus_index]

        # ServoArray uses arr.cfg.joints internally.
        #
        # Temporarily replace the joint list with ONLY the joints physically
        # connected to this board.
        old_joints = arr.cfg.joints

        arr.cfg.joints = self.bus_joints[bus_index]

        try:
            return getattr(arr, method)(*args, **kwargs)

        finally:
            # Always restore the original configuration.
            arr.cfg.joints = old_joints

    # ------------------------------------------------------------------------
    # Automatically discover which servo IDs live on which board.
    # ------------------------------------------------------------------------

    def _discover_buses(self) -> None:
        found: dict[int, int] = {}

        for bus_index, arr in enumerate(self.arrays):

            # At discovery time we intentionally ping using the complete
            # configuration because we need to find which IDs answer here.
            present = arr.ping_all()

            for joint in self.cfg.joints:

                if present.get(joint.name, False):

                    servo_id = joint.servo_id

                    if servo_id in found:
                        raise RuntimeError(
                            f"Servo ID {servo_id} answered on both boards "
                            f"({BOARD_PORTS[found[servo_id]]} and "
                            f"{BOARD_PORTS[bus_index]})."
                        )

                    found[servo_id] = bus_index

        # Check that every expected servo was found.
        missing = [
            joint.name
            for joint in self.cfg.joints
            if joint.servo_id not in found
        ]

        if missing:
            raise RuntimeError(
                "Missing servos across both boards: "
                + ", ".join(missing)
            )

        self.servo_bus = found

        # Build the joint list for each physical board.
        self.bus_joints = {
            bus_index: [
                joint
                for joint in self.cfg.joints
                if self.servo_bus[joint.servo_id] == bus_index
            ]
            for bus_index in range(len(self.arrays))
        }

        # Every board should have at least one servo.
        for bus_index, joints in self.bus_joints.items():

            if not joints:
                raise RuntimeError(
                    f"No configured servos found on {BOARD_PORTS[bus_index]}"
                )

        # Print the discovered mapping.
        say("\nDetected servo buses:")

        for bus_index, joints in self.bus_joints.items():
            ids = [str(j.servo_id) for j in joints]

            say(
                f"  {BOARD_PORTS[bus_index]}: "
                f"{', '.join(ids)}"
            )

        say("")

    # ------------------------------------------------------------------------
    # Find the board containing a specific joint.
    # ------------------------------------------------------------------------

    def _bus_for(self, joint) -> int:
        try:
            return self.servo_bus[joint.servo_id]

        except KeyError:
            raise RuntimeError(
                f"No bus found for {joint.name} "
                f"(servo {joint.servo_id})"
            )

    # ------------------------------------------------------------------------
    # Ping all 12 servos.
    # ------------------------------------------------------------------------

    def ping_all(self):
        result = {
            joint.name: False
            for joint in self.cfg.joints
        }

        for bus_index in range(len(self.arrays)):

            present = self._filtered_call(
                bus_index,
                "ping_all",
            )

            for joint in self.bus_joints[bus_index]:
                result[joint.name] = bool(
                    present.get(joint.name, False)
                )

        return result

    # ------------------------------------------------------------------------
    # Read one servo position.
    # ------------------------------------------------------------------------

    def read_steps(self, joint):
        bus_index = self._bus_for(joint)

        return self.arrays[bus_index].read_steps(joint)

    # ------------------------------------------------------------------------
    # Calibrate one servo's internal midpoint.
    # ------------------------------------------------------------------------

    def calibrate_midpoint(self, joint):
        bus_index = self._bus_for(joint)

        return self.arrays[bus_index].calibrate_midpoint(joint)

    # ------------------------------------------------------------------------
    # Read and hold the current position of all 12 servos.
    #
    # THIS IS THE IMPORTANT FIX.
    # ------------------------------------------------------------------------

    def hold_current(self, speed, accel):
        result = {}

        for bus_index in range(len(self.arrays)):

            values = self._filtered_call(
                bus_index,
                "hold_current",
                speed,
                accel,
            )

            result.update(values)

        return result

    # ------------------------------------------------------------------------
    # Set torque limit on both boards.
    # ------------------------------------------------------------------------

    def set_torque_limit(self, limit):
        for bus_index in range(len(self.arrays)):

            self._filtered_call(
                bus_index,
                "set_torque_limit",
                limit,
            )

    # ------------------------------------------------------------------------
    # Enable / disable torque on both boards.
    # ------------------------------------------------------------------------

    def set_torque(self, enabled):
        for bus_index in range(len(self.arrays)):

            self._filtered_call(
                bus_index,
                "set_torque",
                enabled,
            )

    # ------------------------------------------------------------------------
    # Write positions.
    #
    # Commands are automatically split between the two boards.
    # ------------------------------------------------------------------------

    def write_positions(self, values, speed, accel):

        grouped = {
            bus_index: {}
            for bus_index in range(len(self.arrays))
        }

        for name, value in values.items():

            joint = next(
                (
                    joint
                    for joint in self.cfg.joints
                    if joint.name == name
                ),
                None,
            )

            if joint is None:
                raise KeyError(
                    f"Unknown joint: {name}"
                )

            bus_index = self._bus_for(joint)

            grouped[bus_index][name] = value

        for bus_index, commands in grouped.items():

            if not commands:
                continue

            self._filtered_call(
                bus_index,
                "write_positions",
                commands,
                speed,
                accel,
            )


# ============================================================================
# CREATE THE TWO-BOARD SETUP
# ============================================================================

def setup_dual(args):
    """
    Create two independent SerialBus + ServoArray instances.

    Board assignment is NOT hardcoded by left/right.

    We simply open both boards and discover which servo IDs answer on each.
    """

    arrays = []
    buses = []

    cfg = None

    for port in BOARD_PORTS:

        board_args = copy.copy(args)

        # Force this setup instance to use this physical port.
        board_args.port = port

        board_cfg, bus, arr = setup(board_args)

        if cfg is None:
            cfg = board_cfg

        arrays.append(arr)
        buses.append(bus)

    dual = DualServoArray(
        cfg,
        arrays,
    )

    return cfg, buses, dual


# ============================================================================
# ZERO CALIBRATION
# ============================================================================

def cmd_zero(
    cfg: Config,
    arr,
    method: str,
    cal_path,
    ask_fn=ask,
    out=say,
) -> int:

    present = arr.ping_all()

    missing = [
        name
        for name, ok in present.items()
        if not ok
    ]

    if missing:

        out(
            "These servos do not answer: "
            + ", ".join(missing)
            + "\nRun biped_scan first."
        )

        return 1

    # Torque must be OFF while manually positioning the robot.
    arr.set_torque(False)

    out(ZERO_POSE_TEXT)

    answer = ask_fn(
        "\nHold the robot in the zero pose, then press Enter "
        "to read the servos (or 'q' to quit): "
    ).strip()

    if answer == "q":
        return 1

    # Read every encoder.
    before = {
        joint.name: arr.read_steps(joint)
        for joint in cfg.joints
    }

    out("\nCurrent encoder readings:")

    rows = []

    for joint in cfg.joints:

        steps = before[joint.name]

        offset_deg = (
            (steps - 2048)
            * 360.0
            / 4096.0
        )

        rows.append(
            (
                joint.name,
                joint.servo_id,
                steps,
                f"{offset_deg:+7.1f} deg from 2048",
            )
        )

    _table(
        rows,
        (
            "joint",
            "id",
            "steps",
            "offset",
        ),
        out,
    )

    answer = ask_fn(
        "\nIs the robot STILL held exactly in the zero pose? "
        "Save this as zero? [y/N] "
    ).strip().lower()

    if answer != "y":

        out("Cancelled, nothing changed.")

        return 1

    result: dict[str, dict] = {}

    for joint in cfg.joints:

        if method == "servo":

            after = arr.calibrate_midpoint(joint)

            if abs(after - 2048) <= 4:

                result[joint.name] = {
                    "zero_steps": 2048
                }

                continue

            out(
                f"  {joint.name}: servo mid-point command "
                f"read back {after}, expected 2048 -> "
                f"using a software offset instead"
            )

        result[joint.name] = {
            "zero_steps": before[joint.name]
        }

    # Check whether the software zero would cause encoder wrap-around
    # inside the configured joint range.
    k = STEPS_PER_RAD

    bad = []

    for joint in cfg.joints:

        zero = result[joint.name]["zero_steps"]

        a = (
            zero
            + joint.direction
            * joint.lower
            * k
        )

        b = (
            zero
            + joint.direction
            * joint.upper
            * k
        )

        if min(a, b) < 0 or max(a, b) > 4095:

            bad.append(
                f"{joint.name} "
                f"(zero {zero}: range would run "
                f"{min(a, b):.0f}..{max(a, b):.0f})"
            )

    if bad:

        out(
            "\nPROBLEM: the encoder would wrap around "
            "inside the joint range for: "
            + "; ".join(bad)
            + "\nUse the default servo mid-point method "
            "(it moves zero to 2048), or re-seat the servo "
            "horn on that joint a few teeth away and "
            "calibrate again. Nothing was saved."
        )

        return 1

    path = save_calibration(
        cal_path,
        result,
    )

    out(
        f"\nSaved zero positions to {path}"
    )

    out(
        "Next: "
        "ros2 run biped_hardware "
        "biped_calibrate directions"
    )

    return 0


# ============================================================================
# DIRECTION CALIBRATION
# ============================================================================

def cmd_directions(
    cfg: Config,
    arr,
    cal_path,
    ask_fn=ask,
    out=say,
    sleep=time.sleep,
    delta_deg: float = 10.0,
    torque_limit: int = 300,
    speed: int = 300,
    accel: int = 10,
) -> int:

    # ------------------------------------------------------------------------
    # Verify that all 12 servos answer.
    # ------------------------------------------------------------------------

    present = arr.ping_all()

    missing = [
        name
        for name, ok in present.items()
        if not ok
    ]

    if missing:

        out(
            "These servos do not answer: "
            + ", ".join(missing)
        )

        return 1

    # ------------------------------------------------------------------------
    # Safety message.
    # ------------------------------------------------------------------------

    out(
        "DIRECTION CHECK.\n"
        "The robot must be SUPPORTED with its legs free to move "
        "(hanging from a stand, or lying on its back), "
        "NOT standing on its own.\n\n"
        "Torque will turn ON at a low limit and hold the current pose."
    )

    if ask_fn("Ready? [y/N] ").strip().lower() != "y":
        return 1

    # ------------------------------------------------------------------------
    # THIS NOW WORKS WITH TWO BOARDS.
    #
    # hold_current() separately reads:
    #
    #   ACM0 -> right six
    #   ACM1 -> left six
    #
    # instead of making each board look for all 12.
    # ------------------------------------------------------------------------

    steps0 = arr.hold_current(
        speed,
        accel,
    )

    # Low torque limit for direction testing.
    arr.set_torque_limit(
        torque_limit
    )

    arr.set_torque(True)

    delta = (
        delta_deg
        * 3.141592653589793
        / 180.0
    )

    changes: dict[str, dict] = {}

    # Sort:
    #
    # hip yaw
    # hip roll
    # hip pitch
    # knee
    # ankle pitch
    # ankle roll
    #
    # For each type, right then left.

    order = sorted(
        cfg.joints,
        key=lambda joint: (
            TEST_ORDER.index(
                joint_kind(joint.name)
            ),
            joint.name,
        ),
    )

    quit_all = False

    try:

        for joint in order:

            if quit_all:
                break

            kind = joint_kind(
                joint.name
            )

            sign, description = TESTS[kind]

            # Start with the currently configured direction.
            direction = joint.direction

            for attempt in (1, 2):

                home = steps0[joint.name]

                target = (
                    home
                    + direction
                    * sign
                    * delta
                    * STEPS_PER_RAD
                )

                out(
                    f"\n{joint.name} "
                    f"(servo {joint.servo_id}): "
                    f"moving {delta_deg:.0f} deg "
                    f"in the URDF "
                    f"{'positive' if sign > 0 else 'NEGATIVE'} "
                    f"direction."
                )

                out(
                    f"   EXPECTED: {description}"
                )

                # The DualServoArray automatically sends this
                # command to the correct physical board.
                arr.write_positions(
                    {
                        joint.name: target
                    },
                    speed,
                    accel,
                )

                sleep(1.5)

                answer = ask_fn(
                    "   Did it move that way?  "
                    "[y]es  "
                    "[n]o, the opposite  "
                    "[s]kip  "
                    "[q]uit: "
                ).strip().lower()

                # Return to the original position before doing
                # anything else.
                arr.write_positions(
                    {
                        joint.name: home
                    },
                    speed,
                    accel,
                )

                sleep(1.5)

                # ------------------------------------------------------------
                # Correct direction.
                # ------------------------------------------------------------

                if answer == "y":

                    if direction != joint.direction:

                        changes[joint.name] = {
                            "direction": direction
                        }

                        out(
                            f"   -> direction flipped to "
                            f"{direction:+d} "
                            f"(will be saved)"
                        )

                    break

                # ------------------------------------------------------------
                # Quit entire test.
                # ------------------------------------------------------------

                if answer == "q":

                    quit_all = True

                    break

                # ------------------------------------------------------------
                # Skip this joint.
                # ------------------------------------------------------------

                if answer == "s":

                    break

                # ------------------------------------------------------------
                # Wrong direction on first attempt:
                # try the opposite sign.
                # ------------------------------------------------------------

                if answer == "n" and attempt == 1:

                    direction = -direction

                    out(
                        f"   -> trying direction "
                        f"{direction:+d}"
                    )

                    continue

                # ------------------------------------------------------------
                # Neither direction worked.
                # ------------------------------------------------------------

                out(
                    "   Still not the expected motion "
                    "with either sign: something else is "
                    "wrong (wrong servo ID on this joint? "
                    "horn mounted on a different joint?). "
                    "Not changing anything for this joint."
                )

                break

    finally:

        # ALWAYS turn torque off even if the user quits
        # or an exception occurs.
        try:
            arr.set_torque(False)
        except Exception:
            pass

        out("\nTorque OFF.")

    # ------------------------------------------------------------------------
    # Save verified direction changes.
    # ------------------------------------------------------------------------

    if changes:

        path = save_calibration(
            cal_path,
            changes,
        )

        out(
            f"Saved {len(changes)} direction change(s) "
            f"to {path}: "
            + ", ".join(changes)
        )

    else:

        out(
            "All tested joints already had "
            "the correct direction."
        )

    return 0


# ============================================================================
# RANGE CALIBRATION
# ============================================================================

def cmd_ranges(
    cfg: Config,
    arr,
    cal_path,
    ask_fn=ask,
    out=say,
) -> int:

    present = arr.ping_all()

    missing = [
        name
        for name, ok in present.items()
        if not ok
    ]

    if missing:

        out(
            "These servos do not answer: "
            + ", ".join(missing)
        )

        return 1

    # Torque OFF because the user moves the joints manually.
    arr.set_torque(False)

    out(
        "RANGE RECORDING. Torque is OFF.\n"
        "For each joint, move it BY HAND to each end of its "
        "safe range, stopping a little BEFORE anything touches "
        "or binds.\n"
        "The soft limits then keep the robot inside what it "
        "can really do.\n\n"
        "(Press Enter without moving to keep the default limit; "
        "type 's' to skip a joint.)"
    )

    result: dict[str, dict] = {}

    for joint in cfg.joints:

        answer = ask_fn(
            f"\n{joint.name}: "
            "move to its LOWER end "
            "(URDF-negative side), "
            "press Enter [s=skip] "
        ).strip()

        if answer == "s":
            continue

        a = joint.steps_to_rad(
            arr.read_steps(joint)
        )

        ask_fn(
            f"{joint.name}: "
            "move to its UPPER end, "
            "press Enter "
        )

        b = joint.steps_to_rad(
            arr.read_steps(joint)
        )

        lo = min(a, b)
        hi = max(a, b)

        # Never allow a range wider than the URDF.
        lo = max(
            lo,
            joint.lower,
        )

        hi = min(
            hi,
            joint.upper,
        )

        if hi - lo < 0.1:

            out(
                f"  range {lo:+.2f}..{hi:+.2f} rad "
                "is suspiciously small: ignored"
            )

            continue

        out(
            f"  recorded "
            f"{lo:+.3f} .. {hi:+.3f} rad  "
            f"({lo * 57.2958:+.0f} .. "
            f"{hi * 57.2958:+.0f} deg)"
        )

        result[joint.name] = {
            "lower": round(lo, 4),
            "upper": round(hi, 4),
        }

    if result:

        path = save_calibration(
            cal_path,
            result,
        )

        out(
            f"\nSaved {len(result)} joint range(s) "
            f"to {path}"
        )

        out(
            "Check that the walking gait still fits: "
            "hip roll must reach about -0.40 rad, "
            "ankle pitch about -0.59."
        )

    return 0


# ============================================================================
# SHOW CALIBRATION
# ============================================================================

def cmd_show(
    cfg: Config,
    out=say,
) -> int:

    out(
        f"calibration file: "
        f"{cfg.calibration_path}  "
        f"({'loaded' if cfg.calibration_loaded else 'NOT FOUND - using servos.yaml defaults'})"
    )

    _table(
        [
            (
                joint.name,
                joint.servo_id,
                f"{joint.direction:+d}",
                joint.zero_steps,
                f"{joint.lower:+.3f}",
                f"{joint.upper:+.3f}",
            )
            for joint in cfg.joints
        ],
        (
            "joint",
            "id",
            "dir",
            "zero_steps",
            "lower",
            "upper",
        ),
        out,
    )

    return 0


# ============================================================================
# MAIN
# ============================================================================

def main(argv=None) -> None:

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    sub = parser.add_subparsers(
        dest="cmd",
        required=True,
    )

    # ------------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------------

    for name in (
        "zero",
        "directions",
        "ranges",
        "show",
    ):

        subparser = sub.add_parser(
            name
        )

        add_common_args(
            subparser
        )

        if name == "zero":

            subparser.add_argument(
                "--method",
                choices=(
                    "servo",
                    "software",
                ),
                default="servo",
                help=(
                    "servo = store the midpoint "
                    "inside each servo "
                    "(zero becomes 2048); "
                    "software = only remember "
                    "the current encoder reading "
                    "in calibration.yaml"
                ),
            )

    args = parser.parse_args(
        argv
    )

    # ------------------------------------------------------------------------
    # SHOW DOES NOT NEED TO OPEN THE SERIAL PORTS.
    # ------------------------------------------------------------------------

    if args.cmd == "show":

        # A normal setup is enough to load the effective
        # calibration configuration.
        cfg, bus, arr = setup(args)

        try:
            rc = cmd_show(cfg)

        finally:
            bus.close()

        sys.exit(rc)

    # ------------------------------------------------------------------------
    # ALL HARDWARE COMMANDS USE BOTH BOARDS.
    # ------------------------------------------------------------------------

    cfg, buses, arr = setup_dual(
        args
    )

    cal_path = cfg.calibration_path

    try:

        if args.cmd == "zero":

            rc = cmd_zero(
                cfg,
                arr,
                args.method,
                cal_path,
            )

        elif args.cmd == "directions":

            rc = cmd_directions(
                cfg,
                arr,
                cal_path,
            )

        elif args.cmd == "ranges":

            rc = cmd_ranges(
                cfg,
                arr,
                cal_path,
            )

        else:

            raise RuntimeError(
                f"Unknown command: {args.cmd}"
            )

    finally:

        # Close BOTH serial ports.
        for bus in buses:

            try:
                bus.close()
            except Exception:
                pass

    sys.exit(rc)


# ============================================================================

if __name__ == "__main__":
    main()