"""Tests for biped_mujoco.  No ROS needed (the node is exercised with stand-in ROS modules).
    cd biped_mujoco && python3 -m pytest test/ -v
Needs: pip install mujoco xacro pytest
"""
from __future__ import annotations

import math
import re
import shutil
import subprocess
import sys
import types
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / "biped_mujoco"))
sys.path.insert(0, str(SRC / "biped_gait"))

mujoco = pytest.importorskip("mujoco")
from biped_mujoco.sim_core import JOINTS, BipedSim  # noqa: E402


@pytest.fixture(scope="session")
def urdf_and_pkg(tmp_path_factory):
    """Expanded URDF (use_sim:=false) + the package dir for the meshes."""
    if shutil.which("xacro") is None:
        pytest.skip("xacro not installed")
    tmp = tmp_path_factory.mktemp("desc")
    pkg = tmp / "robotv2_description"
    shutil.copytree(SRC / "robotv2_description", pkg)
    for f in pkg.rglob("*.xacro"):                      # '$(find pkg)' needs a ROS environment; point it at the copy
        f.write_text(f.read_text().replace("$(find robotv2_description)", str(pkg)))
    out = subprocess.run(["xacro", str(pkg / "urdf" / "robot.urdf.xacro"), "use_sim:=false"], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    return out.stdout, pkg


@pytest.fixture()
def sim(urdf_and_pkg):
    return BipedSim(*urdf_and_pkg)


# ---------------------------------------------------------------------------- the model
def test_base_link_is_a_floating_body_with_its_mass(sim):
    """Regression: MuJoCo's URDF importer welds the root link to the world and drops its mass (two legs bolted to the sky)."""
    assert sim.model.nq == 19 and sim.model.nv == 18                  # 12 hinges + a 7/6-DOF free joint
    assert sim.base_name == "base_link"
    assert sim.total_mass() == pytest.approx(1.650, abs=0.002)         # 1.65 kg INCLUDING the 0.62 kg torso


def test_spawn_height_is_computed_from_the_feet(sim):
    assert sim.spawn_z == pytest.approx(0.2865, abs=0.001)             # lowest sole point is 0.2845 m below base_link


def test_joint_order_matches_the_controller_yaml():
    yaml_text = (SRC / "robotv2_description" / "config" / "controllers.yaml").read_text()
    order = re.findall(r"^\s*-\s+(\w+)\s*$", yaml_text.split("position_controller:\n  ros__parameters:")[1], re.M)
    assert JOINTS == order


def test_actuators_and_imu_exist(sim):
    assert sim.model.nu == 12 and sim.model.nsensor == 3
    f = sim.p["stall_torque_nm"] * sim.p["torque_fraction"]
    assert float(sim.model.actuator_forcerange[0][1]) == pytest.approx(f, rel=1e-3)   # real servo torque, not 8 N.m


def test_every_joint_limit_comes_from_the_urdf(sim):
    ankle = JOINTS.index("r_ankle_pitch_joint")
    lo, hi = sim.model.actuator_ctrlrange[ankle]
    assert (lo, hi) == pytest.approx((-0.610865, 1.48353), abs=1e-4)


# ---------------------------------------------------------------------------- physics
def test_stands_still(sim):
    sim.step(int(5 / sim.model.opt.timestep))
    assert not sim.fallen() and sim.tilt_deg() < 0.5
    pos, _ = sim.base_pose()
    assert pos[2] == pytest.approx(0.2845, abs=0.002) and math.hypot(pos[0], pos[1]) < 0.005


def test_imu_reads_gravity_not_a_fake_identity(sim):
    sim.step(int(2 / sim.model.opt.timestep))
    q, gyro, acc = sim.imu()
    assert np.linalg.norm(acc) == pytest.approx(9.81, abs=0.1) and acc[2] > 9.5      # proper acceleration: +g on z at rest
    assert abs(q[3]) > 0.999 and np.linalg.norm(gyro) < 0.05
    sim.push([30, 0, 0], 0.1)
    sim.step(int(0.6 / sim.model.opt.timestep))
    assert np.linalg.norm(sim.imu()[1]) > 0.1 or sim.tilt_deg() > 5          # a real, moving robot is not reported as level


def test_an_undamped_servo_cannot_walk(urdf_and_pkg):
    """armature 0 -> servo damping kv 0.  Standing still is fine, but the gait falls within seconds (measured: 1.4 s in)."""
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg, {"armature": 0.0})
    r = run_walk(sim, _walker(), seconds=15.0)
    assert r["fell_at"] is not None


def test_push_topples_it_and_reset_recovers(sim):
    sim.push([30, 0, 0], 0.1)
    sim.step(int(1.5 / sim.model.opt.timestep))
    assert sim.fallen()
    sim.reset()
    sim.step(int(2 / sim.model.opt.timestep))
    assert not sim.fallen()


def test_small_push_is_survived_standing(sim):
    sim.push([2.0, 0, 0], 0.1)
    sim.step(int(3 / sim.model.opt.timestep))
    assert not sim.fallen()


def test_push_is_additive_and_does_not_wipe_other_forces(sim):
    """Forces applied by someone else (the viewer's mouse drag) must survive our own push bookkeeping."""
    sim.data.xfrc_applied[sim.base_id, 1] = 0.5
    sim.push([1.0, 0, 0], 0.01)
    sim.step(10)
    sim.step(20)                                    # push expired
    assert sim.data.xfrc_applied[sim.base_id, 1] == pytest.approx(0.5)
    assert sim.data.xfrc_applied[sim.base_id, 0] == pytest.approx(0.0)


def _walker():
    from biped_mujoco.check import load_walker
    return load_walker()


def test_the_real_gait_walks_forward(urdf_and_pkg):
    """End to end: the actual quasi_static_walk logic drives the MuJoCo model."""
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg)
    r = run_walk(sim, _walker(), seconds=40.0)
    assert r["fell_at"] is None
    assert r["dx"] > 0.12 and r["max_tilt"] < 6.0           # measured: ~0.2 m and ~1 deg


def test_the_real_gait_walks_backward(urdf_and_pkg):
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg)
    r = run_walk(sim, _walker(), seconds=40.0, direction="backward")
    assert r["fell_at"] is None and r["dx"] < -0.10


def test_torque_demand_is_a_brief_peak_not_a_sustained_load(urdf_and_pkg):
    """Measured: knees peak at ~1.39 N.m (73 % of a 7.4 V ST3215's stall) but the sustained (rms) load is ~0.2 N.m."""
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg, {"torque_fraction": 20.0})          # effectively uncapped; stall base stays 1.912
    r = run_walk(sim, _walker(), seconds=40.0)
    assert 1.0 < r["peak_torque"].max() < 1.9
    assert r["rms_torque"].max() < 0.9                                 # hip roll ~0.6, everything else lower


def test_it_still_walks_at_half_torque(urdf_and_pkg):
    """The hardware default torque_limit is 500/1000.  At 7.4 V that is 0.96 N.m: the gait still walks (a little more tilt)."""
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg, {"torque_fraction": 0.5})
    r = run_walk(sim, _walker(), seconds=40.0)
    assert r["fell_at"] is None and r["dx"] > 0.12 and r["max_tilt"] < 6.0


def test_servo_stiffness_below_the_threshold_falls(urdf_and_pkg):
    from biped_mujoco.check import run_walk
    sim = BipedSim(*urdf_and_pkg, {"kp": 15.0})
    r = run_walk(sim, _walker(), seconds=30.0)
    assert r["fell_at"] is not None                                   # measured: falls at ~10 s


# ---------------------------------------------------------------------------- the ROS node (stand-in ROS)
class _Auto:
    """Message stand-in: any attribute can be read or assigned."""
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        v = _Auto()
        object.__setattr__(self, name, v)
        return v


def _install_fake_ros(monkeypatch):
    log = {"published": {}, "services": {}, "subs": {}, "sent_tf": 0}

    class FakeNode:
        def __init__(self, name):
            self._params, self.logs = {}, []

        def declare_parameter(self, name, default=None):
            self._params[name] = default

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._params[name])

        def create_publisher(self, typ, topic, q):
            log["published"].setdefault(topic, [])
            return types.SimpleNamespace(publish=lambda m, t=topic: log["published"][t].append(m))

        def create_subscription(self, typ, topic, cb, q):
            log["subs"][topic] = cb

        def create_service(self, typ, name, cb):
            log["services"][name] = cb

        def get_logger(self):
            h = self
            return types.SimpleNamespace(**{k: (lambda m, **kw: h.logs.append(m)) for k in ("info", "warning", "warn", "error")})

        def destroy_node(self):
            pass

    class FakeTF:
        def __init__(self, node):
            pass

        def sendTransform(self, t):
            log["sent_tf"] += 1

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    mod("rclpy", ok=lambda: True, spin_once=lambda node, timeout_sec=0: None, init=lambda **k: None, shutdown=lambda: None)
    mod("rclpy.node", Node=FakeNode)
    mod("builtin_interfaces"), mod("builtin_interfaces.msg", Time=_Auto)
    mod("geometry_msgs"), mod("geometry_msgs.msg", TransformStamped=_Auto, WrenchStamped=_Auto)
    mod("nav_msgs"), mod("nav_msgs.msg", Odometry=_Auto)
    mod("rosgraph_msgs"), mod("rosgraph_msgs.msg", Clock=_Auto)
    mod("sensor_msgs"), mod("sensor_msgs.msg", Imu=_Auto, JointState=_Auto)
    mod("std_msgs"), mod("std_msgs.msg", Float64MultiArray=_Auto)
    mod("std_srvs"), mod("std_srvs.srv", Trigger=_Auto)
    mod("tf2_ros", TransformBroadcaster=FakeTF)
    sys.modules.pop("biped_mujoco.mujoco_node", None)
    import biped_mujoco.mujoco_node as node_mod
    return node_mod, log


def _node(monkeypatch, urdf_and_pkg, **params):
    """A MujocoBiped built on the stand-in ROS, with the parameters the launch file would normally provide."""
    node_mod, log = _install_fake_ros(monkeypatch)
    urdf, pkg = urdf_and_pkg
    provided = {"robot_description": urdf, "package_dir": str(pkg), "headless": True, "real_time_factor": 0.0,
                "publish_rate_hz": 100.0, "push_duration_s": 0.1, "max_sim_time_s": 2.0, **params}

    def declare(self, name, default=None):
        self._params[name] = provided.get(name, default)
    monkeypatch.setattr(node_mod.Node, "declare_parameter", declare)
    return node_mod.MujocoBiped(), log


def test_node_publishes_clock_joint_states_imu_odom_and_tf(monkeypatch, urdf_and_pkg):
    n, log = _node(monkeypatch, urdf_and_pkg)
    n.run()
    for topic in ("/clock", "/joint_states", "/imu", "/mujoco/odom"):
        assert 150 < len(log["published"][topic]) < 250, topic       # 100 Hz for 2 s of simulated time
    assert log["sent_tf"] == len(log["published"]["/imu"])
    js = log["published"]["/joint_states"][-1]
    assert list(js.name) == JOINTS and len(js.position) == 12 and len(js.effort) == 12
    imu = log["published"]["/imu"][-1]
    assert imu.linear_acceleration.z == pytest.approx(9.81, abs=0.2) and imu.header.frame_id == "imu_link"
    clocks = [c.clock.sec + c.clock.nanosec * 1e-9 for c in log["published"]["/clock"]]
    assert all(b > a for a, b in zip(clocks, clocks[1:])), "/clock must be strictly increasing"
    odom = log["published"]["/mujoco/odom"][-1]
    assert odom.pose.pose.position.z == pytest.approx(0.2845, abs=0.003) and odom.child_frame_id == "base_link"


def test_node_rejects_bad_commands_and_follows_good_ones(monkeypatch, urdf_and_pkg):
    n, log = _node(monkeypatch, urdf_and_pkg)
    n._on_cmd(types.SimpleNamespace(data=[0.0] * 5))
    n._on_cmd(types.SimpleNamespace(data=[float("nan")] * 12))
    assert any("ignored command" in m for m in n.logs) and np.allclose(n.sim.targets, 0)
    cmd = [0.0] * 12
    cmd[JOINTS.index("l_hip_pitch_joint")] = -0.3
    n._on_cmd(types.SimpleNamespace(data=cmd))
    n.sim.step(int(2 / n.sim.model.opt.timestep))
    assert n.sim.joint_positions()[JOINTS.index("l_hip_pitch_joint")] == pytest.approx(-0.3, abs=0.03)


def test_node_push_and_reset(monkeypatch, urdf_and_pkg):
    n, log = _node(monkeypatch, urdf_and_pkg)
    msg = types.SimpleNamespace(header=types.SimpleNamespace(frame_id=""),
                                wrench=types.SimpleNamespace(force=types.SimpleNamespace(x=30.0, y=0.0, z=0.0),
                                                             torque=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)))
    n._on_push(msg)
    n.sim.step(int(1.5 / n.sim.model.opt.timestep))
    assert n.sim.fallen()
    resp = types.SimpleNamespace(success=False, message="")
    n._on_reset(None, resp)
    assert resp.success and not n.sim.fallen()
    bad = types.SimpleNamespace(header=types.SimpleNamespace(frame_id="no_such_body"), wrench=msg.wrench)
    n._on_push(bad)
    assert any("unknown body" in m for m in n.logs)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
