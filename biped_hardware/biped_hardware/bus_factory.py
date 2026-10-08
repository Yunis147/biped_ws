"""
Servo bus factory.

The real robot has two ST3215 servo controller boards:

    /dev/ttyACM0 -> IDs 7, 8, 9, 10, 11, 12
    /dev/ttyACM1 -> IDs 2, 3, 4, 5, 6, 13

When the configured port is "auto", this module discovers which physical
serial port contains which servo IDs and creates a MultiBus.

The rest of the hardware stack can continue using:

    bus = open_bus(...)
    arr = ServoArray(bus, cfg)

without needing to know about the two physical buses.
"""

from __future__ import annotations

import glob
import logging

from .config import Config
from .mock_bus import MockBus
from .serial_bus import SerialBus, SerialBusError
from .st3215.protocol import encode_ping

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Candidate serial ports
# ---------------------------------------------------------------------------

def candidate_ports() -> list[str]:
    """
    Return possible serial ports.

    Ordering is deterministic.
    """

    ports = (
        sorted(glob.glob("/dev/ttyUSB*"))
        + sorted(glob.glob("/dev/ttyACM*"))
    )

    ports += [
        p
        for p in ("/dev/ttyTHS1",)
        if glob.glob(p)
    ]

    return ports


# ---------------------------------------------------------------------------
# MultiBus
# ---------------------------------------------------------------------------

class MultiBus:
    """
    Collection of independent SerialBus objects.

    Each servo ID is mapped to exactly one physical bus.

    Example:

        ID 7  -> ACM0
        ID 8  -> ACM0
        ...
        ID 12 -> ACM0

        ID 2  -> ACM1
        ID 3  -> ACM1
        ...
        ID 13 -> ACM1
    """

    def __init__(
        self,
        buses: list[SerialBus],
        id_to_bus: dict[int, SerialBus],
    ) -> None:

        self.buses = list(buses)
        self.id_to_bus = dict(id_to_bus)

    # ------------------------------------------------------------------
    # Routing
    # ------------------------------------------------------------------

    def bus_for_id(self, servo_id: int) -> SerialBus:
        """
        Return the physical bus assigned to servo_id.
        """

        try:
            return self.id_to_bus[int(servo_id)]
        except KeyError:
            raise SerialBusError(
                f"servo ID {servo_id} is not assigned to any physical bus"
            )

    def update_id(
        self,
        old_id: int,
        new_id: int,
    ) -> None:

        bus = self.id_to_bus.pop(
            int(old_id),
            None,
        )

        if bus is not None:
            self.id_to_bus[int(new_id)] = bus

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """
        Close every physical bus.
        """

        for bus in self.buses:
            try:
                bus.close()
            except Exception as exc:
                log.warning(
                    "error closing servo bus: %s",
                    exc,
                )

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def is_open(self) -> bool:
        return any(
            bus.is_open
            for bus in self.buses
        )


# ---------------------------------------------------------------------------
# Ping helper
# ---------------------------------------------------------------------------

def _ping(
    bus: SerialBus,
    servo_id: int,
) -> bool:
    """
    Ping one servo directly on one physical bus.

    This is intentionally independent of ServoArray because discovery
    happens before ServoArray is constructed.
    """

    try:
        bus.transfer(
            encode_ping(servo_id),
            response_data_len=0,
        )
        return True

    except SerialBusError:
        return False

    except Exception:
        return False


# ---------------------------------------------------------------------------
# Mock bus
# ---------------------------------------------------------------------------

def _open_mock_bus(
    cfg: Config,
    time_fn=None,
):
    """
    Open the simulated bus.

    Mock mode remains a single logical bus because there is no real
    physical board to split.
    """

    kwargs = {}

    if time_fn is not None:
        kwargs["time_fn"] = time_fn

    bus = MockBus(
        [
            joint.servo_id
            for joint in cfg.joints
        ],
        **kwargs,
    )

    bus.open()

    return bus


# ---------------------------------------------------------------------------
# Real bus discovery
# ---------------------------------------------------------------------------

def _open_auto_multi_bus(
    cfg: Config,
):
    """
    Discover all configured servo IDs across all available serial ports.

    The algorithm:

      1. Find candidate /dev/ttyACM* and /dev/ttyUSB* ports.
      2. Open each candidate.
      3. Ping every configured servo ID.
      4. Record which port answered each ID.
      5. Reject duplicate IDs.
      6. Require every configured ID to be found.
      7. Return MultiBus.
    """

    ports = candidate_ports()

    if not ports:
        raise SerialBusError(
            "no serial port found. Looked for "
            "/dev/ttyUSB*, /dev/ttyACM*, /dev/ttyTHS1.\n"
            "Run: ls /dev/tty*"
        )

    baud = int(
        cfg.bus.get(
            "baud_rate",
            1_000_000,
        )
    )

    timeout = float(
        cfg.bus.get(
            "timeout_s",
            0.01,
        )
    )

    echo = bool(
        cfg.bus.get(
            "expect_echo",
            False,
        )
    )

    expected_ids = [
        joint.servo_id
        for joint in cfg.joints
    ]

    opened_buses: list[SerialBus] = []

    # ID -> physical bus
    id_to_bus: dict[int, SerialBus] = {}

    # Port -> IDs found there
    found_by_port: dict[str, list[int]] = {}

    # --------------------------------------------------------------
    # Probe every candidate port.
    # --------------------------------------------------------------

    for port in ports:

        bus = SerialBus(
            port,
            baud,
            timeout=timeout,
            expect_echo=echo,
        )

        try:
            bus.open()

        except Exception as exc:
            log.debug(
                "could not open %s: %s",
                port,
                exc,
            )
            continue

        found_here: list[int] = []

        for servo_id in expected_ids:

            if _ping(bus, servo_id):

                # Same ID answering on two physical buses is a wiring/
                # configuration problem.
                if servo_id in id_to_bus:
                    previous_bus = id_to_bus[servo_id]

                    raise SerialBusError(
                        f"servo ID {servo_id} answered on more than one "
                        f"serial bus. Check for duplicate servo IDs."
                    )

                id_to_bus[servo_id] = bus
                found_here.append(servo_id)

        if found_here:

            opened_buses.append(bus)
            found_by_port[port] = found_here

            log.info(
                "found servo IDs %s on %s",
                found_here,
                port,
            )

        else:

            # This port opened but none of our expected servo IDs
            # answered, so it is not one of the two servo buses.
            bus.close()

    # --------------------------------------------------------------
    # Verify that every configured servo was found.
    # --------------------------------------------------------------

    missing_ids = [
        servo_id
        for servo_id in expected_ids
        if servo_id not in id_to_bus
    ]

    if missing_ids:

        for bus in opened_buses:
            bus.close()

        details = []

        for port, ids in found_by_port.items():
            details.append(
                f"{port}: IDs {ids}"
            )

        found_text = (
            "\n  ".join(details)
            if details
            else "none"
        )

        raise SerialBusError(
            "could not discover all configured ST3215 servos.\n"
            f"Missing IDs: {missing_ids}\n"
            f"Found:\n  {found_text}\n\n"
            "Expected physical arrangement:\n"
            "  /dev/ttyACM0 -> IDs 7,8,9,10,11,12\n"
            "  /dev/ttyACM1 -> IDs 2,3,4,5,6,13\n\n"
            "Check:\n"
            "  - both servo boards are powered\n"
            "  - both USB cables are connected\n"
            "  - servo IDs are correct\n"
            "  - both boards use 1,000,000 baud\n"
            "  - your user has dialout permission"
        )

    # --------------------------------------------------------------
    # Print final mapping.
    # --------------------------------------------------------------

    for port, ids in found_by_port.items():
        log.info(
            "servo bus %s -> IDs %s",
            port,
            ids,
        )

    return MultiBus(
        opened_buses,
        id_to_bus,
    )


# ---------------------------------------------------------------------------
# Public factory
# ---------------------------------------------------------------------------

def open_bus(
    cfg: Config,
    port: str | None = None,
    mock: bool = False,
    baud: int | None = None,
    time_fn=None,
):
    """
    Open the servo bus.

    Modes:

        mock=True
            -> MockBus

        port="/dev/ttyACM0"
            -> one SerialBus

        port="auto" / port=None with hardware.yaml "auto"
            -> automatically discover all servo IDs across the
               available physical serial buses and return MultiBus.
    """

    # --------------------------------------------------------------
    # Mock mode
    # --------------------------------------------------------------

    if mock:
        return _open_mock_bus(
            cfg,
            time_fn=time_fn,
        )

    # --------------------------------------------------------------
    # Configuration
    # --------------------------------------------------------------

    configured_port = (
        port
        or cfg.bus.get(
            "port",
            "auto",
        )
    )

    baud_rate = int(
        baud
        or cfg.bus.get(
            "baud_rate",
            1_000_000,
        )
    )

    timeout = float(
        cfg.bus.get(
            "timeout_s",
            0.01,
        )
    )

    echo = bool(
        cfg.bus.get(
            "expect_echo",
            False,
        )
    )

    # --------------------------------------------------------------
    # Explicit single port.
    #
    # Useful for testing one leg/one board.
    # --------------------------------------------------------------

    if configured_port != "auto":

        bus = SerialBus(
            configured_port,
            baud_rate,
            timeout=timeout,
            expect_echo=echo,
        )

        try:
            bus.open()

        except Exception as exc:
            raise SerialBusError(
                f"could not open {configured_port}: {exc}"
            ) from exc

        log.info(
            "servo bus opened on %s @ %d baud",
            configured_port,
            baud_rate,
        )

        return bus

    # --------------------------------------------------------------
    # Automatic two-board discovery.
    # --------------------------------------------------------------

    return _open_auto_multi_bus(cfg)