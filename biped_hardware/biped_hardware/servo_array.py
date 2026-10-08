"""
ServoArray: interface to the 12 ST3215 servos.

The real robot uses two physical serial buses:

    /dev/ttyACM0 -> right leg
        IDs 7, 8, 9, 10, 11, 12

    /dev/ttyACM1 -> left leg
        IDs 2, 3, 4, 5, 6, 13

ServoArray keeps the same public API as the original single-bus
implementation, so BridgeCore and the calibration tools do not need
to know that there are two physical buses.

For a single bus or MockBus, behaviour is unchanged.
For a MultiBus, every operation is automatically split by servo ID.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from .config import Config, JointCfg
from .serial_bus import SerialBusError
from .st3215.protocol import (
    encode_ping,
    encode_read,
    encode_sync_read,
    encode_sync_write,
    encode_write,
    pack_u16,
    unpack_u16,
)
from .st3215.registers import Reg

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Servo status
# ---------------------------------------------------------------------------

@dataclass
class Status:
    steps: int
    speed: int
    load: int
    voltage_v: float
    temp_c: int


def _parse_status(data: bytes) -> Status:
    """
    Parse the 8-byte ST3215 status block.

    Layout:
        position : 2 bytes
        speed    : 2 bytes
        load     : 2 bytes
        voltage  : 1 byte
        temp     : 1 byte
    """

    if len(data) < Reg.STATUS_LEN:
        raise SerialBusError(
            f"short status packet: expected {Reg.STATUS_LEN} bytes, got {len(data)}"
        )

    return Status(
        steps=unpack_u16(data, 0) & 0x0FFF,
        speed=unpack_u16(data, 2) & 0x7FFF,
        load=unpack_u16(data, 4) & 0x03FF,
        voltage_v=data[6] * 0.1,
        temp_c=data[7],
    )


# ---------------------------------------------------------------------------
# ServoArray
# ---------------------------------------------------------------------------

class ServoArray:

    def __init__(self, bus, cfg: Config) -> None:
        self.bus = bus
        self.cfg = cfg

        self.ids = [j.servo_id for j in cfg.joints]

        self.clamped_writes = 0

    # ------------------------------------------------------------------
    # Bus routing
    # ------------------------------------------------------------------

    def _bus_for_id(self, servo_id: int):
        """
        Return the physical bus that owns servo_id.

        Normal single-bus operation:
            return self.bus

        MultiBus operation:
            return the bus assigned to this servo ID.
        """

        if hasattr(self.bus, "bus_for_id"):
            return self.bus.bus_for_id(servo_id)

        return self.bus

    def _group_joints_by_bus(self):
        """
        Group configured joints by physical bus.

        Returns:
            {
                bus_object: [JointCfg, JointCfg, ...]
            }
        """

        groups = {}

        for joint in self.cfg.joints:
            bus = self._bus_for_id(joint.servo_id)
            groups.setdefault(bus, []).append(joint)

        return groups

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        """
        Ping one servo.

        For MultiBus the correct physical bus is selected automatically.
        """

        try:
            bus = self._bus_for_id(servo_id)

            bus.transfer(
                encode_ping(servo_id),
                response_data_len=0,
            )

            return True

        except SerialBusError:
            return False

    def ping_all(self) -> dict[str, bool]:
        """
        Ping every configured servo.

        Returns:
            {
                "r_hip_yaw": True,
                "r_hip_roll_joint": True,
                ...
            }
        """

        return {
            joint.name: self.ping(joint.servo_id)
            for joint in self.cfg.joints
        }

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def read_status(self) -> dict[str, Status | None]:
        """
        Read position/load/voltage/temperature for all servos.

        With two buses:

            ACM0 -> one SYNC_READ for right leg
            ACM1 -> one SYNC_READ for left leg

        If a SYNC_READ fails or a servo does not answer, that servo is
        retried individually.
        """

        out: dict[str, Status | None] = {}

        # Initialize all entries.
        for joint in self.cfg.joints:
            out[joint.name] = None

        groups = self._group_joints_by_bus()

        for bus, joints in groups.items():

            ids = [joint.servo_id for joint in joints]

            # ----------------------------------------------------------
            # First try SYNC_READ on this physical bus.
            # ----------------------------------------------------------

            packet = encode_sync_read(
                Reg.STATUS_START,
                Reg.STATUS_LEN,
                ids,
            )

            try:
                got = bus.sync_read(
                    packet,
                    ids,
                    Reg.STATUS_LEN,
                )

            except SerialBusError as exc:
                log.debug(
                    "SYNC_READ failed on bus: %s",
                    exc,
                )
                got = {}

            # ----------------------------------------------------------
            # Parse responses.
            #
            # Missing servos are handled below with individual reads.
            # ----------------------------------------------------------

            for joint in joints:

                data = got.get(joint.servo_id)

                if data is not None:
                    try:
                        out[joint.name] = _parse_status(data)
                    except Exception as exc:
                        log.debug(
                            "failed to parse status from %s: %s",
                            joint.name,
                            exc,
                        )

        # ------------------------------------------------------------------
        # Individual fallback reads.
        # ------------------------------------------------------------------

        for joint in self.cfg.joints:

            if out[joint.name] is not None:
                continue

            bus = self._bus_for_id(joint.servo_id)

            try:
                data = bus.transfer(
                    encode_read(
                        joint.servo_id,
                        Reg.STATUS_START,
                        Reg.STATUS_LEN,
                    ),
                    response_data_len=Reg.STATUS_LEN,
                )

                out[joint.name] = _parse_status(data)

            except SerialBusError:
                out[joint.name] = None

            except Exception as exc:
                log.debug(
                    "status read failed for %s: %s",
                    joint.name,
                    exc,
                )
                out[joint.name] = None

        return out

    def read_steps(self, joint: JointCfg) -> int:
        """
        Read the current encoder position of one joint.
        """

        bus = self._bus_for_id(joint.servo_id)

        data = bus.transfer(
            encode_read(
                joint.servo_id,
                Reg.CURRENT_POS_L,
                2,
            ),
            response_data_len=2,
        )

        return unpack_u16(data, 0) & 0x0FFF

    # ------------------------------------------------------------------
    # Writing positions
    # ------------------------------------------------------------------

    def _build_position_payload(
        self,
        servo_id: int,
        steps: float,
        speed: int,
        accel: int,
    ) -> tuple[int, bytes]:

        s = int(round(steps))

        if s < 0 or s > 4095:
            self.clamped_writes += 1
            s = max(0, min(4095, s))

        payload = (
            bytes([
                max(0, min(254, int(accel)))
            ])
            + pack_u16(s)
            + pack_u16(0)       # run time
            + pack_u16(int(speed))
        )

        return servo_id, payload

    def write_positions(
        self,
        steps: dict[str, float],
        speed: int,
        accel: int,
    ) -> None:
        """
        Write target positions.

        With two physical buses the 12-joint command is automatically
        split into two SYNC_WRITE packets.

            ACM0 -> right leg
            ACM1 -> left leg
        """

        groups = self._group_joints_by_bus()

        for bus, joints in groups.items():

            entries = []

            for joint in joints:

                if joint.name not in steps:
                    continue

                sid, payload = self._build_position_payload(
                    joint.servo_id,
                    steps[joint.name],
                    speed,
                    accel,
                )

                entries.append(
                    (sid, payload)
                )

            if not entries:
                continue

            packet = encode_sync_write(
                Reg.ACCELERATION,
                7,
                entries,
            )

            bus.send_no_reply(packet)

    # ------------------------------------------------------------------
    # Torque
    # ------------------------------------------------------------------

    def set_torque(
        self,
        on: bool,
        names: list[str] | None = None,
    ) -> None:
        """
        Enable/disable torque.

        Automatically splits the command across the two buses.
        """

        groups = self._group_joints_by_bus()

        for bus, joints in groups.items():

            selected = [
                joint
                for joint in joints
                if names is None or joint.name in names
            ]

            if not selected:
                continue

            entries = [
                (
                    joint.servo_id,
                    bytes([1 if on else 0]),
                )
                for joint in selected
            ]

            bus.send_no_reply(
                encode_sync_write(
                    Reg.TORQUE_ENABLE,
                    1,
                    entries,
                )
            )

    def set_torque_limit(self, limit: int) -> None:
        """
        Set torque limit for all servos.
        """

        limit = max(
            0,
            min(1000, int(limit)),
        )

        groups = self._group_joints_by_bus()

        for bus, joints in groups.items():

            entries = [
                (
                    joint.servo_id,
                    pack_u16(limit),
                )
                for joint in joints
            ]

            if entries:
                bus.send_no_reply(
                    encode_sync_write(
                        Reg.TORQUE_LIMIT_L,
                        2,
                        entries,
                    )
                )

    # ------------------------------------------------------------------
    # PID
    # ------------------------------------------------------------------

    def apply_pid(
        self,
        p: int,
        d: int,
        i: int,
    ) -> None:
        """
        Write P/D/I values to every servo EEPROM.

        This is deliberately done servo-by-servo because EEPROM writes
        are individual transactions.
        """

        for joint in self.cfg.joints:

            bus = self._bus_for_id(joint.servo_id)
            sid = joint.servo_id

            # Unlock EEPROM.
            bus.transfer(
                encode_write(
                    sid,
                    Reg.LOCK_FLAG,
                    bytes([0]),
                ),
                response_data_len=0,
            )

            # Write PID.
            bus.transfer(
                encode_write(
                    sid,
                    Reg.PID_P,
                    bytes([
                        int(p),
                        int(d),
                        int(i),
                    ]),
                ),
                response_data_len=0,
            )

            # Lock EEPROM again.
            bus.transfer(
                encode_write(
                    sid,
                    Reg.LOCK_FLAG,
                    bytes([1]),
                ),
                response_data_len=0,
            )

    # ------------------------------------------------------------------
    # Hold current position
    # ------------------------------------------------------------------

    def hold_current(
        self,
        speed: int,
        accel: int,
    ) -> dict[str, int]:
        """
        Make every servo's target equal to its current position.

        This is important when enabling torque so the robot does not
        suddenly jump toward an old command.
        """

        status = self.read_status()

        missing = [
            name
            for name, value in status.items()
            if value is None
        ]

        if missing:
            raise SerialBusError(
                "no reply from: " + ", ".join(missing)
            )

        steps = {
            name: value.steps
            for name, value in status.items()
        }

        self.write_positions(
            steps,
            speed,
            accel,
        )

        return steps

    # ------------------------------------------------------------------
    # Mid-point calibration
    # ------------------------------------------------------------------

    def calibrate_midpoint(
        self,
        joint: JointCfg,
    ) -> int:
        """
        Ask the ST3215 servo to store its current physical position as
        the servo-side midpoint.

        The servo should subsequently report approximately 2048 steps
        at that physical position.
        """

        bus = self._bus_for_id(joint.servo_id)

        bus.transfer(
            encode_write(
                joint.servo_id,
                Reg.TORQUE_ENABLE,
                bytes([128]),
            ),
            response_data_len=0,
        )

        return self.read_steps(joint)

    # ------------------------------------------------------------------
    # Servo ID change
    # ------------------------------------------------------------------

    def change_id(
        self,
        old_id: int,
        new_id: int,
    ) -> None:

        if not (1 <= new_id <= 253):
            raise ValueError(
                "servo ID must be 1..253"
            )

        bus = self._bus_for_id(old_id)

        # Unlock EEPROM.
        bus.transfer(
            encode_write(
                old_id,
                Reg.LOCK_FLAG,
                bytes([0]),
            ),
            response_data_len=0,
        )

        # Change ID.
        bus.transfer(
            encode_write(
                old_id,
                Reg.ID,
                bytes([new_id]),
            ),
            response_data_len=0,
        )

        # Re-lock using new ID.
        bus.transfer(
            encode_write(
                new_id,
                Reg.LOCK_FLAG,
                bytes([1]),
            ),
            response_data_len=0,
        )

        # Keep MultiBus's ID map correct if applicable.
        if hasattr(self.bus, "update_id"):
            self.bus.update_id(
                old_id,
                new_id,
            )