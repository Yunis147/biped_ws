"""Automated tests for biped_hardware, run entirely on MockBus (no serial port / servos needed).
    cd biped_hardware && python3 -m pytest test/ -v       (or: python3 test/test_hardware.py)
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
import pytest

from biped_hardware.bridge_core import BridgeCore
from biped_hardware.config import JointCfg, load_config
from biped_hardware.mock_bus import MockBus
from biped_hardware.servo_array import ServoArray
from biped_hardware.st3215.protocol import pack_u16, unpack_u16


def make(cal="/nonexistent.yaml", raw=None):
    cfg = load_config(calibration_yaml=cal)
    t = [0.0]
    bus = MockBus([j.servo_id for j in cfg.joints], initial_raw=raw, time_fn=lambda: t[0])
    bus.open()
    return cfg, bus, ServoArray(bus, cfg), t


# ---------------------------------------------------------------------- config / conversions
def test_servo_ids_match_the_hardware_repo():
    cfg = load_config()
    ids = {j.name: j.servo_id for j in cfg.joints}
    assert ids == {"r_hip_yaw": 7, "r_hip_roll_joint": 8, "r_hip_pitch_joint": 9, "r_knee_joint": 10,
                   "r_ankle_pitch_joint": 12, "r_ankle_roll_joint": 11, "l_hip_yaw": 13,
                   "l_hip_roll_joint": 2, "l_hip_pitch_joint": 3, "l_knee_joint": 4,
                   "l_ankle_pitch_joint": 6, "l_ankle_roll_joint": 5}


def test_joint_order_matches_the_gazebo_controller():
    cfg = load_config()
    yaml_order = ["r_hip_yaw", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_joint", "r_ankle_pitch_joint",
                 "r_ankle_roll_joint", "l_hip_yaw", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_joint",
                 "l_ankle_pitch_joint", "l_ankle_roll_joint"]
    assert cfg.names == yaml_order


def test_rad_steps_roundtrip():
    j = JointCfg("x", 1, direction=1, zero_steps=2048, lower=-2.0, upper=2.0)
    for r in (-1.9, -0.5, 0.0, 0.3, 1.9):
        assert j.steps_to_rad(round(j.rad_to_steps(r))) == pytest.approx(r, abs=1e-3)
    j2 = JointCfg("x", 1, direction=-1, zero_steps=1000, lower=-2.0, upper=2.0)
    assert j2.rad_to_steps(0.0) == 1000
    assert j2.rad_to_steps(1.0) < 1000                      # negative direction: +rad -> fewer steps


def test_steps_to_rad_unwraps_across_the_encoder_seam():
    # zero near the 0/4095 seam: a joint angle just negative of zero must not jump to +2pi
    j = JointCfg("x", 1, direction=1, zero_steps=10, lower=-0.5, upper=0.5)
    assert j.steps_to_rad(4090) == pytest.approx(j.steps_to_rad(4090) )
    assert -0.1 < j.steps_to_rad(4090) < 0        # 10 -> 4090 is a small negative step, not +6.2 rad


def test_duplicate_servo_ids_are_rejected(tmp_path):
    bad = tmp_path / "servos.yaml"
    bad.write_text("joint_order: [a, b]\nservos:\n  a: {id: 5, direction: 1, zero_steps: 2048, lower: -1, upper: 1}\n"
                   "  b: {id: 5, direction: 1, zero_steps: 2048, lower: -1, upper: 1}\n")
    hw = tmp_path / "hardware.yaml"; hw.write_text("bus: {}\n")
    with pytest.raises(ValueError, match="duplicate"):
        load_config(hardware_yaml=hw, servos_yaml=bad, calibration_yaml="/nonexistent.yaml")


def test_bad_direction_is_rejected(tmp_path):
    bad = tmp_path / "servos.yaml"
    bad.write_text("joint_order: [a]\nservos:\n  a: {id: 5, direction: 2, zero_steps: 2048, lower: -1, upper: 1}\n")
    hw = tmp_path / "hardware.yaml"; hw.write_text("bus: {}\n")
    with pytest.raises(ValueError, match="direction"):
        load_config(hardware_yaml=hw, servos_yaml=bad, calibration_yaml="/nonexistent.yaml")


# ---------------------------------------------------------------------- protocol / mock bus
def test_u16_pack_matches_feetech_byte_order():
    assert unpack_u16(pack_u16(1000)) == 1000
    assert pack_u16(0x0102) == bytes([0x02, 0x01])            # little-endian, like the real servo


def test_mock_bus_round_trips_a_write_then_read():
    cfg, bus, arr, t = make()
    arr.write_positions({"l_knee_joint": 2048}, 1500, 30)
    arr.set_torque(True)
    t[0] += 0.5
    assert arr.read_steps(cfg.joint("l_knee_joint")) == pytest.approx(2048, abs=2)


def test_mock_bus_detects_a_missing_servo():
    cfg, bus, arr, t = make()
    del bus.servos[cfg.joint("l_knee_joint").servo_id]
    st = arr.read_status()
    assert st["l_knee_joint"] is None
    assert all(v is not None for k, v in st.items() if k != "l_knee_joint")


def test_speed_cap_limits_how_fast_a_servo_moves():
    cfg, bus, arr, t = make()
    arr.hold_current(1500, 30)
    arr.set_torque(True)
    arr.write_positions({"l_knee_joint": 2048 + 1000}, 1500, 30)
    t[0] += 0.2
    assert arr.read_steps(cfg.joint("l_knee_joint")) == pytest.approx(2048 + 300, abs=2)


def test_servo_midpoint_zeroes_the_reading():
    cfg, bus, arr, t = make()
    j = cfg.joint("r_hip_yaw")
    bus.move_by_hand(j.servo_id, 3100)
    assert arr.calibrate_midpoint(j) == 2048


# ---------------------------------------------------------------------- BridgeCore safety behaviour
def test_torque_on_holds_current_pose_first_no_jump():
    cfg, bus, arr, t = make()
    raw = {j.servo_id: j.rad_to_steps(-0.5) if j.name == "l_knee_joint" else 2048 for j in cfg.joints}
    cfg, bus, arr, t = make(raw=raw)
    core = BridgeCore(arr, cfg)
    ok, _ = core.enable_torque()
    assert ok
    assert core.meas[cfg.names.index("l_knee_joint")] == pytest.approx(-0.5, abs=0.01)


def test_commands_are_clamped_to_the_soft_joint_limits():
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    core.enable_torque()
    cmd = [0.0] * 12
    cmd[cfg.names.index("r_hip_roll_joint")] = 5.0           # way past the 1.57 rad limit
    for _ in range(600):
        t[0] += 0.02
        core.set_command(cmd, t[0])
        core.update(t[0])
    idx = cfg.names.index("r_hip_roll_joint")
    # one encoder step (2*pi/4096 ~= 0.0015 rad) of slack for quantization
    assert core.meas[idx] <= core.hi[idx] + 2 * math.pi / 4096
    assert core.meas[idx] == pytest.approx(core.hi[idx], abs=0.01)


def test_speed_limit_is_obeyed_on_a_step_command():
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    core.enable_torque()
    for _ in range(100):
        t[0] += 0.02
        core.set_command([0.0] * 12, t[0])
        core.update(t[0])
    cmd = [0.0] * 12
    cmd[cfg.names.index("r_hip_pitch_joint")] = 1.0
    for _ in range(5):
        t[0] += 0.02
        core.set_command(cmd, t[0])
        core.update(t[0])
    idx = cfg.names.index("r_hip_pitch_joint")
    assert core.meas[idx] == pytest.approx(0.1 * core.max_speed, abs=0.03)   # 0.1 s elapsed


def test_watchdog_holds_the_last_pose_when_commands_stop():
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    core.enable_torque()
    for _ in range(150):
        t[0] += 0.02
        core.set_command([0.1] * 12, t[0])
        core.update(t[0])
    before = core.meas.copy()
    for _ in range(100):                      # 2 s, no new commands: well past the 0.5 s timeout
        t[0] += 0.02
        core.update(t[0])
    assert core.watchdog is True
    assert np.allclose(core.meas, before, atol=0.01)


def test_malformed_commands_are_rejected_not_crashed_on():
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    assert core.set_command([0.0] * 5, 0.0) is False
    assert core.set_command([float("nan")] * 12, 0.0) is False
    assert core.set_command([0.0] * 12, 0.0) is True


def test_enable_torque_refuses_if_a_servo_is_silent():
    cfg, bus, arr, t = make()
    del bus.servos[cfg.joint("l_knee_joint").servo_id]
    core = BridgeCore(arr, cfg)
    ok, msg = core.enable_torque()
    assert ok is False
    assert "l_knee_joint" in msg
    assert core.torque is False


def test_health_flags_a_missing_servo_as_error_and_hot_servo_as_warn():
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    core.enable_torque()
    bus.servos[cfg.joint("r_knee_joint").servo_id].temp_c = 90
    del bus.servos[cfg.joint("l_knee_joint").servo_id]
    core.update(t[0])
    rows = {n: (lvl, txt) for n, lvl, txt in core.health()}
    assert rows["l_knee_joint"][0] == "ERROR"
    assert rows["r_knee_joint"][0] == "WARN"
    assert "HOT" in rows["r_knee_joint"][1]


def test_walker_poses_stay_inside_the_joints_commands_get_clamped_to():
    """Feed real gait poses (from biped_gait) through BridgeCore and check nothing silently misbehaves."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "biped_gait"))
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "qsw", Path(__file__).resolve().parents[2] / "biped_gait" / "biped_gait" / "quasi_static_walk.py")
    qsw = importlib.util.module_from_spec(spec)
    sys.modules["rclpy"] = None
    spec.loader.exec_module(qsw)
    cfg, bus, arr, t = make()
    core = BridgeCore(arr, cfg)
    core.enable_torque()
    kf, _ = qsw.build_tables("forward")
    for name, q in kf.items():
        pose = [q[n] for n in cfg.names]
        for _ in range(5):
            t[0] += 0.02
            core.set_command(pose, t[0])
            core.update(t[0])
    assert arr.clamped_writes == 0, f"a walker pose needed clamping: outside the soft limits by design margin"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
