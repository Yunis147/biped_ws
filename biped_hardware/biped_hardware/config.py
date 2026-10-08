"""Loads servos.yaml + hardware.yaml (+ the user's calibration.yaml) and converts between
joint angles (radians, URDF convention) and servo encoder steps (0..4095)."""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, replace
from pathlib import Path

import yaml

STEPS_PER_REV = 4096
STEPS_PER_RAD = STEPS_PER_REV / (2.0 * math.pi)
DEFAULT_CALIBRATION = Path.home() / ".config" / "biped" / "calibration.yaml"


@dataclass(frozen=True)
class JointCfg:
    name: str
    servo_id: int
    direction: int          # +1 / -1
    zero_steps: int         # encoder value at 0 rad
    lower: float            # soft limits, rad
    upper: float

    def rad_to_steps(self, rad: float) -> float:
        """Angle (rad) -> encoder steps (float, NOT clamped)."""
        return self.zero_steps + self.direction * rad * STEPS_PER_RAD

    def steps_to_rad(self, steps: int) -> float:
        """Encoder steps -> angle (rad).  Unwrapped around zero so a joint range that straddles the
        0/4095 encoder seam still reads continuously."""
        delta = ((int(steps) - self.zero_steps + 2048) % STEPS_PER_REV) - 2048
        return self.direction * delta / STEPS_PER_RAD


@dataclass
class Config:
    joints: list[JointCfg]
    bus: dict = field(default_factory=dict)
    loop: dict = field(default_factory=dict)
    safety: dict = field(default_factory=dict)
    pid: dict = field(default_factory=dict)
    imu: dict = field(default_factory=dict)
    calibration_path: Path = DEFAULT_CALIBRATION
    calibration_loaded: bool = False

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]

    def joint(self, name: str) -> JointCfg:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(name)

    def by_id(self, servo_id: int) -> JointCfg:
        for j in self.joints:
            if j.servo_id == servo_id:
                return j
        raise KeyError(servo_id)


def _share_dir() -> Path:
    """Folder holding servos.yaml / hardware.yaml: the installed share dir, or ../config in a source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("biped_hardware")) / "config"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config"


def _read_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_config(hardware_yaml: str | os.PathLike | None = None,
                servos_yaml: str | os.PathLike | None = None,
                calibration_yaml: str | os.PathLike | None = None) -> Config:
    share = _share_dir()
    hw = _read_yaml(Path(hardware_yaml) if hardware_yaml else share / "hardware.yaml")
    sv = _read_yaml(Path(servos_yaml) if servos_yaml else share / "servos.yaml")
    cal_path = Path(calibration_yaml).expanduser() if calibration_yaml else DEFAULT_CALIBRATION
    cal = _read_yaml(cal_path).get("servos", {}) if cal_path.exists() else {}

    order = sv["joint_order"]
    joints: list[JointCfg] = []
    for name in order:
        s = dict(sv["servos"][name])
        s.update(cal.get(name, {}))
        direction = int(s["direction"])
        if direction not in (-1, 1):
            raise ValueError(f"{name}: direction must be +1 or -1, got {direction}")
        j = JointCfg(name=name, servo_id=int(s["id"]), direction=direction,
                     zero_steps=int(s["zero_steps"]), lower=float(s["lower"]), upper=float(s["upper"]))
        if not (0 <= j.zero_steps <= 4095):
            raise ValueError(f"{name}: zero_steps {j.zero_steps} outside 0..4095")
        if j.lower >= j.upper:
            raise ValueError(f"{name}: lower >= upper")
        joints.append(j)
    ids = [j.servo_id for j in joints]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate servo IDs in servos.yaml: {ids}")
    return Config(joints=joints, bus=hw.get("bus", {}), loop=hw.get("loop", {}), safety=hw.get("safety", {}),
                  pid=hw.get("pid", {}), imu=hw.get("imu", {}), calibration_path=cal_path,
                  calibration_loaded=bool(cal))


def save_calibration(path: str | os.PathLike, per_joint: dict[str, dict]) -> Path:
    """Merge `per_joint` ({joint: {zero_steps|direction|lower|upper: value}}) into the calibration file.
    The previous file is kept as <name>.bak."""
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {"servos": {}}
    if path.exists():
        data = _read_yaml(path) or {"servos": {}}
        data.setdefault("servos", {})
        path.replace(path.with_suffix(path.suffix + ".bak"))
    for name, vals in per_joint.items():
        data["servos"].setdefault(name, {}).update(vals)
    header = ("# Written by biped_calibrate.  Overrides zero_steps / direction / lower / upper from servos.yaml.\n"
              "# Safe to edit by hand; delete a joint's entry to fall back to servos.yaml.\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write(header)
        yaml.safe_dump(data, f, sort_keys=False)
    return path
