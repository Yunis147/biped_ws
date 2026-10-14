"""sim_core: the MuJoCo side of the BipedRobot V2 simulation.  Pure MuJoCo + numpy — NO ROS in here, so it can be
tested (and its numbers reproduced) without a ROS installation.  mujoco_node.py is the thin ROS wrapper around it.

What it does
  1. Takes the robot's URDF (xacro output, use_sim:=false) and builds a MuJoCo model from it.
  2. Fixes what MuJoCo's URDF importer gets wrong for a walking robot:
       * the URDF root link (base_link) is WELDED to the world and its mass thrown away, leaving two legs bolted
         to the sky.  We give base_link a 6-DOF floating joint so the whole robot can stand, tip and walk.
  3. Adds what a URDF does not contain: a floor, servo-like position actuators, an IMU, joint friction /
     reflected rotor inertia, and contact settings that are stiff enough for a 37 mm wide foot.

The servo and contact numbers are ASSUMPTIONS (see DEFAULTS and docs/MUJOCO.md); the experiments behind them are
summarised there.  In particular: contact_timeconst matters a lot — MuJoCo's default (0.02 s) is so soft that the
foot rolls onto its edge and the gait's ~10 mm stability margin disappears.
"""
from __future__ import annotations

import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import mujoco
except ImportError as exc:                                   # a friendly message instead of a bare traceback
    raise ImportError("The MuJoCo Python package is not installed.  Install it with:\n"
                      "    pip install --user --break-system-packages mujoco") from exc
import numpy as np

KGCM_TO_NM = 0.0980665

# Same order as robotv2_description/config/controllers.yaml (= index into /position_controller/commands).
JOINTS = [
    "r_hip_yaw", "r_hip_roll_joint", "r_hip_pitch_joint", "r_knee_joint", "r_ankle_pitch_joint", "r_ankle_roll_joint",
    "l_hip_yaw", "l_hip_roll_joint", "l_hip_pitch_joint", "l_knee_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
]

DEFAULTS = dict(
    stall_torque_nm=19.5 * KGCM_TO_NM,  # Waveshare ST3215, 7.4 V version: 19.5 kg.cm.  (12 V version: 30 kg.cm = 2.94 N.m)
    torque_fraction=1.0,                # share of stall torque the servo may use (the hardware torque_limit / 1000)
    kp=100.0,                           # N.m/rad  position-loop stiffness.  Gait needs >= ~25; real value unknown
    damping_ratio=1.0,                  # kv = 2 * ratio * sqrt(kp * armature)  (critical damping)
    armature=0.005,                     # kg.m^2   reflected rotor inertia of a geared servo.  The servo damping kv is
                                        #          DERIVED from it, so armature 0 = an undamped servo: it stands, but the
                                        #          gait falls 1.4 s into the walk (measured).  >= 0.001 works.
    damping=0.05,                       # N.m.s/rad gear-train viscous friction
    frictionloss=0.03,                  # N.m      gear-train dry friction
    friction=1.0,                       # sliding friction coefficient, applied to EVERY geom (contacts use the pair max)
    contact_timeconst=0.005,            # s        ground contact softness (MuJoCo default 0.02 is far too soft here)
    timestep=0.002,                     # s
    spawn_clearance=0.002,              # m        feet start this far above the floor
)


class BipedSim:
    def __init__(self, urdf_text: str, package_dir: str | Path, params: dict | None = None) -> None:
        """urdf_text   : expanded URDF (xacro robot.urdf.xacro use_sim:=false)
        package_dir : folder that `package://robotv2_description/` points to (contains meshes/)"""
        self.p = {**DEFAULTS, **(params or {})}
        p = self.p
        self._tmp = tempfile.TemporaryDirectory(prefix="biped_mujoco_")
        tmp = Path(self._tmp.name)

        # ---- 1. URDF: resolve mesh paths, give the root link a floating joint ---------------------------------
        urdf = urdf_text.replace("package://robotv2_description/", str(package_dir).rstrip("/") + "/")
        root = ET.fromstring(urdf)
        links = {l.get("name") for l in root.findall("link")}
        children = {j.find("child").get("link") for j in root.findall("joint")}
        roots = sorted(links - children)
        if len(roots) != 1:
            raise ValueError(f"URDF must have exactly one root link, found {roots}")
        self.base_name = roots[0]
        anchor = ET.Element("link", {"name": "mj_world_anchor"})
        fj = ET.Element("joint", {"name": "floating_base_joint", "type": "floating"})
        ET.SubElement(fj, "parent", {"link": "mj_world_anchor"})
        ET.SubElement(fj, "child", {"link": self.base_name})
        root.insert(0, fj)
        root.insert(0, anchor)
        urdf_path = tmp / "robot_floating.urdf"
        ET.ElementTree(root).write(urdf_path, encoding="utf-8", xml_declaration=True)

        m0 = mujoco.MjModel.from_xml_path(str(urdf_path))
        d0 = mujoco.MjData(m0)
        mujoco.mj_forward(m0, d0)
        self.spawn_z = -self._lowest_collision_point(m0, d0) + p["spawn_clearance"]
        jr = {mujoco.mj_id2name(m0, mujoco.mjtObj.mjOBJ_JOINT, i): m0.jnt_range[i].copy() for i in range(m0.njnt)}
        missing = [n for n in JOINTS if n not in jr]
        if missing:
            raise ValueError(f"joints missing from the URDF: {missing}")

        # ---- 2. MJCF: floor, IMU site + sensors, position actuators ------------------------------------------
        stage = tmp / "stage.xml"
        mujoco.mj_saveLastXML(str(stage), m0)
        mj = ET.parse(stage).getroot()
        wb = mj.find("worldbody")
        # a visible checkerboard floor + sky (a plain grey plane blends into the background and looks like "no ground")
        assets = mj.find("asset")
        if assets is None:
            assets = ET.Element("asset")
            mj.insert(list(mj).index(wb), assets)
        ET.SubElement(assets, "texture", {"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7",
                                          "rgb2": "0 0 0", "width": "512", "height": "3072"})
        ET.SubElement(assets, "texture", {"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge",
                                          "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8",
                                          "width": "300", "height": "300"})
        ET.SubElement(assets, "material", {"name": "groundplane", "texture": "groundplane", "texuniform": "true",
                                           "texrepeat": "5 5", "reflectance": "0.2"})
        ET.SubElement(wb, "geom", {"name": "ground", "type": "plane", "size": "10 10 0.1", "condim": "3",
                                   "material": "groundplane", "friction": f"{p['friction']:g} 0.005 0.0001"})
        imu_pos = self._imu_offset(root)
        for b in wb.findall("body"):
            if b.get("name") == self.base_name:
                b.set("pos", f"0 0 {self.spawn_z:.5f}")
                ET.SubElement(b, "site", {"name": "imu_site", "pos": " ".join(f"{v:.6f}" for v in imu_pos), "size": "0.004"})
        force = p["stall_torque_nm"] * p["torque_fraction"]
        kv = 2.0 * p["damping_ratio"] * math.sqrt(p["kp"] * p["armature"])
        act = ET.SubElement(mj, "actuator")
        for n in JOINTS:
            ET.SubElement(act, "position", {"name": "act_" + n, "joint": n, "kp": f"{p['kp']:g}", "kv": f"{kv:.5g}",
                                            "ctrlrange": f"{jr[n][0]:.6f} {jr[n][1]:.6f}",
                                            "forcerange": f"{-force:.5f} {force:.5f}"})
        sn = ET.SubElement(mj, "sensor")
        ET.SubElement(sn, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu_site"})
        ET.SubElement(sn, "gyro", {"name": "imu_gyro", "site": "imu_site"})
        ET.SubElement(sn, "accelerometer", {"name": "imu_accel", "site": "imu_site"})
        opt = mj.find("option")
        if opt is None:
            opt = ET.Element("option")
            mj.insert(0, opt)
        opt.set("timestep", f"{p['timestep']:g}")
        opt.set("gravity", "0 0 -9.81")
        opt.set("integrator", "implicitfast")
        mjcf_path = tmp / "biped.xml"
        ET.ElementTree(mj).write(mjcf_path, encoding="utf-8", xml_declaration=True)
        self.mjcf_path = mjcf_path

        # ---- 3. load; per-joint dynamics and contact softness are plain model arrays -------------------------------
        self.model = m = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = d = mujoco.MjData(m)
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        self._qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self._vadr = np.array([m.jnt_dofadr[j] for j in jid])
        self._aid = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_" + n) for n in JOINTS])
        for v in self._vadr:
            m.dof_armature[v], m.dof_damping[v], m.dof_frictionloss[v] = p["armature"], p["damping"], p["frictionloss"]
        m.geom_solref[:, 0] = p["contact_timeconst"]
        m.geom_friction[:, 0] = p["friction"]
        self.base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.base_name)
        sid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n) for n in ("imu_quat", "imu_gyro", "imu_accel")}
        self._sens = {k: (m.sensor_adr[v], m.sensor_dim[v]) for k, v in sid.items()}
        self.targets = np.zeros(len(JOINTS))
        self._pushes: list[list] = []         # [body_id, force(3), torque(3), t_end]
        self._applied = np.zeros((m.nbody, 6))
        self.reset()

    # ---- helpers ---------------------------------------------------------------------------------------------------
    @staticmethod
    def _lowest_collision_point(m, d) -> float:
        low = 1e9
        for g in range(m.ngeom):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            low = min(low, float(((v @ d.geom_xmat[g].reshape(3, 3).T) + d.geom_xpos[g])[:, 2].min()))
        return low

    @staticmethod
    def _imu_offset(root) -> list[float]:
        for j in root.findall("joint"):
            if "imu" in j.get("name", "") and j.find("origin") is not None:
                return [float(v) for v in j.find("origin").get("xyz", "0 0 0").split()]
        return [0.0, 0.0, 0.0]

    # ---- state ------------------------------------------------------------------------------------------------------
    @property
    def time(self) -> float:
        return float(self.data.time)

    def reset(self) -> None:
        """Back to the standing pose, at rest, feet just above the floor."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, self.spawn_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.targets[:] = 0.0
        self.data.ctrl[self._aid] = 0.0
        self._pushes.clear()
        self._applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, q: np.ndarray) -> None:
        """Commanded joint angles (rad), controller order.  The actuators clamp them to the joint ranges."""
        self.targets = np.asarray(q, dtype=float)
        self.data.ctrl[self._aid] = self.targets

    def push(self, force_world, duration: float, torque_world=(0.0, 0.0, 0.0), body: str | None = None) -> None:
        """External wrench on a body (default: the torso) for `duration` seconds of SIMULATION time."""
        bid = self.base_id if body is None else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        if bid < 0:
            raise ValueError(f"unknown body '{body}'")
        self._pushes.append([bid, np.array(force_world, float), np.array(torque_world, float), self.time + duration])

    def step(self, n: int = 1) -> None:
        m, d = self.model, self.data
        for _ in range(n):
            now = d.time
            self._pushes = [p for p in self._pushes if p[3] > now]
            want = np.zeros_like(self._applied)
            for bid, f, t, _ in self._pushes:
                want[bid, :3] += f
                want[bid, 3:] += t
            d.xfrc_applied += want - self._applied     # additive: keeps forces from other sources (the viewer's mouse drag)
            self._applied = want
            mujoco.mj_step(m, d)

    # ---- measurements ------------------------------------------------------------------------------------------------
    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self._qadr].copy()

    def joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self._vadr].copy()

    def joint_efforts(self) -> np.ndarray:
        return self.data.actuator_force[self._aid].copy()

    def imu(self):
        """-> (quaternion x,y,z,w ; angular velocity rad/s ; acceleration m/s^2 incl. gravity), all in the IMU frame."""
        s = self.data.sensordata
        a, _ = self._sens["imu_quat"]
        w, x, y, z = s[a: a + 4]
        g, _ = self._sens["imu_gyro"]
        c, _ = self._sens["imu_accel"]
        return (float(x), float(y), float(z), float(w)), s[g: g + 3].copy(), s[c: c + 3].copy()

    def base_pose(self):
        """-> (position xyz, quaternion x,y,z,w) of the torso in the world frame."""
        d = self.data
        w, x, y, z = d.xquat[self.base_id]
        return d.xpos[self.base_id].copy(), (float(x), float(y), float(z), float(w))

    def base_velocity(self):
        """-> (linear velocity in the world frame, angular velocity in the torso frame)."""
        return self.data.qvel[0:3].copy(), self.data.qvel[3:6].copy()

    def tilt_deg(self) -> float:
        """Angle between the torso's up axis and the world up axis (0 = upright)."""
        up = self.data.xmat[self.base_id].reshape(3, 3)[2, 2]
        return math.degrees(math.acos(max(-1.0, min(1.0, float(up)))))

    def fallen(self) -> bool:
        return self.tilt_deg() > 45.0 or float(self.data.xpos[self.base_id][2]) < 0.15

    def total_mass(self) -> float:
        return float(sum(self.model.body_mass))
