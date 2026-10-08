"""biped_scan — find servos on both ST3215 buses.

    ros2 run biped_hardware biped_scan

    Scans:
        /dev/ttyACM0
        /dev/ttyACM1

    Options:
        --all          Scan every ID 0..253
        --try-bauds    Try other ST3215 baud rates if nothing answers
"""
from __future__ import annotations

import argparse
import sys

from ..serial_bus import SerialBus, SerialBusError
from ..st3215.protocol import encode_ping, encode_read, unpack_u16
from ..st3215.registers import BAUD_RATES, Reg
from .common import add_common_args, say, setup


# ------------------------------------------------------------
# Scan IDs on ONE bus
# ------------------------------------------------------------

def scan_ids(bus, ids) -> list[int]:
    found = []

    for i in ids:
        try:
            bus.transfer(
                encode_ping(i),
                response_data_len=0
            )
            found.append(i)

        except SerialBusError:
            pass

    return found


# ------------------------------------------------------------
# Read information from one servo
# ------------------------------------------------------------

def describe(bus, sid) -> str:
    try:
        d = bus.transfer(
            encode_read(sid, Reg.CURRENT_POS_L, 2),
            2
        )

        pos = unpack_u16(d) & 0x0FFF

        v = bus.transfer(
            encode_read(sid, Reg.CURRENT_VOLTAGE, 2),
            2
        )

        return (
            f"position {pos:4d}  "
            f"voltage {v[0] * 0.1:4.1f} V  "
            f"temperature {v[1]} C"
        )

    except SerialBusError as exc:
        return f"(could not read: {exc})"


# ------------------------------------------------------------
# Scan one physical board
# ------------------------------------------------------------

def scan_port(
    port,
    cfg,
    baud,
    ids,
    out
):
    out("")
    out("=" * 60)
    out(f"Scanning {port} @ {baud} baud")
    out("=" * 60)

    bus = SerialBus(
        port,
        baud,
        timeout=float(cfg.bus.get("timeout_s", 0.01))
    )

    try:
        bus.open()
    except Exception as exc:
        out(f"ERROR: could not open {port}: {exc}")
        return [], False

    try:
        found = scan_ids(bus, ids)

        if not found:
            out("servos answering: NONE")
            bus.close()
            return [], True

        out(f"servos answering: {found}")

        expected = {
            j.servo_id: j.name
            for j in cfg.joints
        }

        for sid in found:
            name = expected.get(
                sid,
                "(not in servos.yaml)"
            )

            out(
                f"  id {sid:3d}  "
                f"{name:22s}  "
                f"{describe(bus, sid)}"
            )

        bus.close()
        return found, True

    except Exception as exc:
        out(f"ERROR while scanning {port}: {exc}")

        try:
            bus.close()
        except Exception:
            pass

        return [], False


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def run(args, bus=None, out=say) -> int:

    # We still use setup() to load your hardware.yaml,
    # servos.yaml, timeout, etc.
    cfg, _, arr = setup(args, bus)

    ids = (
        range(0, 254)
        if args.all
        else range(1, 31)
    )

    # --------------------------------------------------------
    # Ports
    # --------------------------------------------------------

    ports = [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    ]

    # If user explicitly supplies --port,
    # scan only that port.
    if args.port:
        ports = [args.port]

    baud = args.baud or cfg.bus.get(
        "baud_rate",
        1000000
    )

    # --------------------------------------------------------
    # Scan both boards
    # --------------------------------------------------------

    results = {}

    for port in ports:

        found, opened = scan_port(
            port,
            cfg,
            baud,
            ids,
            out
        )

        results[port] = found

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    out("")
    out("=" * 60)
    out("FINAL SUMMARY")
    out("=" * 60)

    for port, found in results.items():
        out(
            f"{port}: "
            f"{found if found else 'NONE'}"
        )

    # --------------------------------------------------------
    # Expected servo IDs from servos.yaml
    # --------------------------------------------------------

    expected = {
        j.servo_id: j.name
        for j in cfg.joints
    }

    # Combine IDs found on both buses
    all_found = []

    for found in results.values():
        all_found.extend(found)

    # Remove duplicates while preserving order
    all_found = list(dict.fromkeys(all_found))

    out("")
    out(f"All detected IDs: {all_found}")

    # --------------------------------------------------------
    # Missing / unexpected
    # --------------------------------------------------------

    missing = [
        f"{name} (id {sid})"
        for sid, name in expected.items()
        if sid not in all_found
    ]

    extra = [
        sid
        for sid in all_found
        if sid not in expected
    ]

    if missing:
        out("")
        out("MISSING:")
        for item in missing:
            out(f"  {item}")

    if extra:
        out("")
        out(
            f"UNEXPECTED ids on the buses: {extra}"
        )

    # --------------------------------------------------------
    # Final status
    # --------------------------------------------------------

    out("")

    if not missing and not extra:
        out(
            "OK: all 12 expected servos answer "
            "across the two buses."
        )
        return 0

    return 1


def main(argv=None) -> None:

    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    add_common_args(p)

    p.add_argument(
        "--all",
        action="store_true",
        help="scan IDs 0..253"
    )

    p.add_argument(
        "--try-bauds",
        action="store_true",
        help="try other ST3215 baud rates"
    )

    sys.exit(
        run(
            p.parse_args(argv)
        )
    )


if __name__ == "__main__":
    main()