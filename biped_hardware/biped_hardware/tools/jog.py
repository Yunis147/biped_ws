"""biped_jog — move ONE joint at a time by hand-typed commands, to fine-tune a joint's zero by eye.
Stop servo_bridge first.  Support the robot (stand / hanging) — torque is ON at a low limit.

    ros2 run biped_hardware biped_jog

Commands:  +  -     move 1 degree in the URDF positive / negative direction
           ++ --    move 5 degrees
           0        go back to the saved zero
           set0     the joint is exactly at zero NOW: save this as its zero
           n  p     next / previous joint        <name or id>  select a joint        q  quit
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from ..config import STEPS_PER_RAD, Config, save_calibration
from .common import add_common_args, ask, say, setup


def run(cfg: Config, arr, cal_path, method: str = "servo", ask_fn=ask, out=say, sleep=time.sleep,
        torque_limit: int = 300, speed: int = 300, accel: int = 10) -> int:
    missing = [n for n, ok in arr.ping_all().items() if not ok]
    if missing:
        out("These servos do not answer: " + ", ".join(missing))
        return 1
    if ask_fn("The robot must be SUPPORTED. Torque turns ON (low limit) and holds the current pose. Ready? [y/N] ").strip().lower() != "y":
        return 1
    target = dict(arr.hold_current(speed, accel))          # joint name -> commanded steps
    arr.set_torque_limit(torque_limit)
    arr.set_torque(True)
    joints = {j.name: j for j in cfg.joints}
    names = list(joints)
    idx = 0
    out(__doc__.split("Commands:")[1])
    while True:
        j = joints[names[idx]]
        here = j.steps_to_rad(target[j.name])
        cmd = ask_fn(f"[{j.name} id{j.servo_id}  {math.degrees(here):+6.1f} deg] > ").strip().lower()
        if cmd == "q":
            break
        if cmd in ("n", "p"):
            idx = (idx + (1 if cmd == "n" else -1)) % len(names)
            continue
        sel = next((n for n in names if n == cmd or str(joints[n].servo_id) == cmd), None)
        if sel:
            idx = names.index(sel)
            continue
        if cmd in ("+", "-", "++", "--"):
            deg = (5.0 if len(cmd) == 2 else 1.0) * (1 if cmd[0] == "+" else -1)
            new_rad = max(j.lower, min(j.upper, here + math.radians(deg)))
        elif cmd == "0":
            new_rad = 0.0
        elif cmd == "set0":
            arr.set_torque(False, [j.name])
            if method == "servo":
                after = arr.calibrate_midpoint(j)
                zero = 2048 if abs(after - 2048) <= 4 else arr.read_steps(j)
            else:
                zero = arr.read_steps(j)
            path = save_calibration(cal_path, {j.name: {"zero_steps": zero}})
            joints[j.name] = j = type(j)(j.name, j.servo_id, j.direction, zero, j.lower, j.upper)
            target[j.name] = arr.read_steps(j)
            arr.write_positions({j.name: target[j.name]}, speed, accel)
            arr.set_torque(True, [j.name])
            out(f"   saved zero_steps={zero} for {j.name} to {path}")
            continue
        else:
            out("   ? use + - ++ -- 0 set0 n p q or a joint name")
            continue
        target[j.name] = j.rad_to_steps(new_rad)
        arr.write_positions({j.name: target[j.name]}, speed, accel)
        sleep(0.4)
    arr.set_torque(False)
    out("Torque OFF.")
    return 0


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--method", choices=("servo", "software"), default="servo")
    a = p.parse_args(argv)
    cfg, bus, arr = setup(a)
    try:
        rc = run(cfg, arr, cfg.calibration_path, a.method)
    finally:
        bus.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
