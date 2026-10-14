"""biped_mujoco_check — run the REAL walking-node logic (biped_gait) against the MuJoCo model, with no ROS running.

    ros2 run biped_mujoco biped_mujoco_check                       # stand 5 s, then walk forward 70 s
    ros2 run biped_mujoco biped_mujoco_check --direction backward
    ros2 run biped_mujoco biped_mujoco_check --push 4 --push-dir x --push-time 7   # shove the torso 4 N for 0.1 s at t=7 s
    ros2 run biped_mujoco biped_mujoco_check --kp 25 --torque-fraction 0.5        # try other servo settings

Prints whether it stood / walked / fell, how far it went, the largest torso tilt, and the peak torque each joint needed.
This is the quickest way to try a change before running the full ROS simulation.
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import types
from pathlib import Path

import numpy as np

from .sim_core import DEFAULTS, JOINTS, BipedSim


# ---- tiny ROS stand-ins, so the walking node can run here with no ROS (a fake clock = simulation time) ----------------
class _Msg:
    def __init__(self, data=None):
        self.data = data


class _Pub:
    last = None

    def publish(self, m):
        self.last = list(m.data)


class _FakeNode:
    def __init__(self, name):
        self.t, self.logs, self._timers, self._subs, self._pubs = 0.0, [], [], {}, []

    def create_publisher(self, t, topic, q):
        p = _Pub()
        self._pubs.append(p)
        return p

    def create_subscription(self, t, topic, cb, q):
        self._subs[topic] = cb

    def create_timer(self, period, cb):
        self._timers.append(cb)

    def get_clock(self):
        h = self
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(h.t * 1e9)))

    def get_logger(self):
        h = self
        return types.SimpleNamespace(info=lambda m, **k: h.logs.append(m), warn=lambda m, **k: h.logs.append(m),
                                     warning=lambda m, **k: h.logs.append(m), error=lambda m, **k: h.logs.append(m))


def load_walker():
    """Import biped_gait.quasi_static_walk with ROS replaced by the stand-ins above."""
    for name in ("rclpy", "rclpy.node", "std_msgs", "std_msgs.msg"):
        sys.modules[name] = types.ModuleType(name)
    sys.modules["rclpy.node"].Node = _FakeNode
    sys.modules["std_msgs.msg"].Float64MultiArray = _Msg
    sys.modules["std_msgs.msg"].String = _Msg
    sys.modules.pop("biped_gait.quasi_static_walk", None)
    import biped_gait.quasi_static_walk as walker
    return walker


def run_walk(sim: BipedSim, walker, seconds: float = 70.0, direction: str = "forward", settle: float = 2.0,
             push: dict | None = None, hold_standing: bool = False) -> dict:
    """Drive `sim` with the walking node.  push = dict(t=sim seconds, force=(fx,fy,fz) N, dur=s)."""
    node = walker.QuasiStaticWalker()
    dt = sim.model.opt.timestep
    tick = max(1, int(round(0.02 / dt)))                   # the walking node publishes at 50 Hz
    started, start_pos, peak, max_tilt, fell_at, pushed = False, None, np.zeros(len(JOINTS)), 0.0, None, False
    sumsq, nsamp = np.zeros(len(JOINTS)), 0
    for i in range(int(seconds / dt)):
        node.t = sim.time
        if i % tick == 0:
            for cb in node._timers:
                cb()
            if node._pubs[0].last is not None:
                sim.set_targets(np.array(node._pubs[0].last))
        if not started and not hold_standing and sim.time >= settle:
            node._subs["/walk_cmd"](_Msg(direction))
            started, start_pos = True, sim.base_pose()[0]
        if push and not pushed and sim.time >= push["t"]:
            sim.push(push["force"], push.get("dur", 0.1))
            pushed = True
        sim.step()
        eff = np.abs(sim.joint_efforts())
        peak = np.maximum(peak, eff)
        if started:
            sumsq += eff ** 2
            nsamp += 1
        tilt = sim.tilt_deg()
        if started:
            max_tilt = max(max_tilt, tilt)
        if fell_at is None and sim.fallen():
            fell_at = sim.time
            break
    pos = sim.base_pose()[0]
    start = start_pos if start_pos is not None else pos
    return dict(fell_at=fell_at, dx=float(pos[0] - start[0]), dy=float(pos[1] - start[1]), z=float(pos[2]),
                max_tilt=max_tilt, peak_torque=peak, rms_torque=np.sqrt(sumsq / max(nsamp, 1)), t=sim.time, segment=f"{node.cur}->{node.nxt}")


def _urdf_text(urdf: str | None, package_dir: Path) -> str:
    if urdf:
        return Path(urdf).read_text()
    out = subprocess.run(["xacro", str(package_dir / "urdf" / "robot.urdf.xacro"), "use_sim:=false"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit("could not run xacro (is ROS sourced?). Pass --urdf <expanded urdf file>.\n" + out.stderr)
    return out.stdout


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", help="expanded URDF (default: run xacro on robotv2_description)")
    ap.add_argument("--package-dir", help="robotv2_description share folder (default: found via ament)")
    ap.add_argument("--seconds", type=float, default=70.0)
    ap.add_argument("--direction", choices=("forward", "backward"), default="forward")
    ap.add_argument("--push", type=float, default=0.0, help="push force in N (0.1 s)")
    ap.add_argument("--push-dir", choices=("x", "-x", "y", "-y"), default="x")
    ap.add_argument("--push-time", type=float, default=7.0)
    for k, v in DEFAULTS.items():
        ap.add_argument("--" + k.replace("_", "-"), type=float, default=v)
    a = ap.parse_args(argv)
    params = {k: getattr(a, k) for k in DEFAULTS}

    if a.package_dir:
        pkg = Path(a.package_dir)
    else:
        from ament_index_python.packages import get_package_share_directory
        pkg = Path(get_package_share_directory("robotv2_description"))
    sim = BipedSim(_urdf_text(a.urdf, pkg), pkg, params)
    walker = load_walker()

    print(f"model: {sim.model.nbody - 1} bodies, mass {sim.total_mass():.3f} kg, spawn height {sim.spawn_z:.4f} m, "
          f"torque limit {params['stall_torque_nm'] * params['torque_fraction']:.2f} N.m per joint, kp {params['kp']:g}")
    r = run_walk(sim, walker, seconds=5.0, hold_standing=True)
    print(f"standing 5 s: {'FELL' if r['fell_at'] else 'stable'}  tilt {sim.tilt_deg():.2f} deg  height {r['z']:.4f} m")
    sim.reset()
    sign = {"x": (1, 0, 0), "-x": (-1, 0, 0), "y": (0, 1, 0), "-y": (0, -1, 0)}[a.push_dir]
    push = dict(t=a.push_time, force=tuple(a.push * c for c in sign), dur=0.1) if a.push > 0 else None
    r = run_walk(sim, walker, a.seconds, a.direction, push=push)
    verdict = f"FELL at t={r['fell_at']:.1f} s" if r["fell_at"] else "kept walking"
    print(f"walk {a.direction} {a.seconds:g} s: {verdict}; moved {r['dx']:+.3f} m forward/back, {r['dy']:+.3f} m sideways; "
          f"max torso tilt {r['max_tilt']:.1f} deg")
    stall = params["stall_torque_nm"]
    print("torque per joint while walking (N.m; % of the configured stall torque).  peak = brief maximum, rms = sustained load,")
    print("which is what heats a servo:")
    for n, t, q in sorted(zip(JOINTS, r["peak_torque"], r["rms_torque"]), key=lambda x: -x[1])[:6]:
        print(f"   {n:22s} peak {t:5.2f} ({t / stall * 100:3.0f}%)   rms {q:5.2f} ({q / stall * 100:3.0f}%)")


if __name__ == "__main__":
    main()
