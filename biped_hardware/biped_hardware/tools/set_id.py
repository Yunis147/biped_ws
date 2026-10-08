"""biped_set_id — give servos their IDs (stored in the servo's EEPROM).

Brand-new servos all have ID 1, so they must be connected ONE AT A TIME.

    ros2 run biped_hardware biped_set_id --guided          # walks you through all 12 joints
    ros2 run biped_hardware biped_set_id --from 1 --to 13  # one servo (only that servo on the bus)
"""
from __future__ import annotations

import argparse
import sys

from ..st3215.protocol import encode_ping
from ..serial_bus import SerialBusError
from .common import add_common_args, ask, say, setup
from .scan import scan_ids


def change_one(arr, bus, old: int, new: int, out=say) -> bool:
    if old == new:
        out(f"  already ID {new}")
        return True
    if arr.ping(new):
        out(f"  ID {new} is already used by another servo on this bus: disconnect it first")
        return False
    arr.change_id(old, new)
    ok = arr.ping(new) and not arr.ping(old)
    out(f"  {'OK: servo is now ID ' + str(new) if ok else 'FAILED: servo did not answer on the new ID'}")
    return ok


def run_single(args, bus=None, out=say) -> int:
    cfg, bus, arr = setup(args, bus)
    if not arr.ping(args.from_id):
        out(f"no servo answers on ID {args.from_id}")
        return 1
    others = [i for i in scan_ids(bus, range(1, 31)) if i != args.from_id]
    if others:
        out(f"other servos are on the bus: {others}.  Connect only the one you want to change.")
        return 1
    return 0 if change_one(arr, bus, args.from_id, args.to_id, out) else 1


def run_guided(args, bus=None, out=say, ask_fn=ask) -> int:
    cfg, bus, arr = setup(args, bus)
    out("Guided ID assignment. For each joint, connect ONLY that servo to the adapter.\n")
    failures = 0
    for j in cfg.joints:
        ask_fn(f"Connect ONLY the servo for  {j.name}  (it will become ID {j.servo_id}). Press Enter ... ")
        found = scan_ids(bus, range(0, 31)) or scan_ids(bus, range(31, 254))
        if len(found) != 1:
            out(f"  expected exactly one servo on the bus, found {found}. Skipping {j.name}.")
            failures += 1
            continue
        if not change_one(arr, bus, found[0], j.servo_id, out):
            failures += 1
    out("\nDone. Now reconnect all servos in the chain and run:  ros2 run biped_hardware biped_scan")
    return 0 if failures == 0 else 1


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--guided", action="store_true")
    p.add_argument("--from", dest="from_id", type=int)
    p.add_argument("--to", dest="to_id", type=int)
    a = p.parse_args(argv)
    if a.guided:
        sys.exit(run_guided(a))
    if a.from_id is None or a.to_id is None:
        p.error("use --guided, or both --from and --to")
    sys.exit(run_single(a))


if __name__ == "__main__":
    main()
