# BipedRobot V2 — simulation + real hardware

Four ROS 2 (Jazzy) packages. The same walking node and the same `/position_controller/commands` /
`/joint_states` topics drive both Gazebo and the real robot — only what's *underneath* those topics
changes.

| package | what it is |
|---|---|
| `robotv2_description` | URDF, meshes, Gazebo bringup, RViz config. `use_sim:=true/false` switches whether the Gazebo-only parts (gz_ros2_control, the simulated IMU sensor) are included. |
| `biped_gait` | `quasi_static_walk` — the open-loop quasi-static walking node. Identical in sim and on hardware. See `docs/walking_math_explained.md` for the full derivation of every constant in it. |
| `biped_hardware` | The real robot: `servo_bridge` (12× Waveshare ST3215 over a serial bus), `imu_node` (BNO055), and the `biped_scan` / `biped_set_id` / `biped_calibrate` / `biped_jog` command-line tools. Has a `mock:=true` mode that behaves like real servos at the packet level, with no hardware attached. |
| `biped_bringup` | One-command launch files: `sim.launch.py`, `hardware.launch.py`. |

```bash
cd ~/biped_ws
colcon build --symlink-install
source install/setup.bash
```

## Simulation

```bash
ros2 launch biped_bringup sim.launch.py walk:=true
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```
`sim.launch.py` just includes `robotv2_description`'s `gazebo.launch.py` (`rviz:=true`, `world:=...` etc.
all still work) and, with `walk:=true`, `biped_gait`'s `walk.launch.py`.

## Real hardware

**Read `docs/CALIBRATION.md` first and follow it in order — ID assignment, zero pose, and especially the
direction check.** A flipped sign there is exactly the kind of thing that made the robot fall over
repeatedly in simulation before the gait was tuned; on hardware it does the same thing, just with a real
robot.

Dry run with no servos connected (everything behaves like real hardware at the packet level, including
speed limits and a missing-servo error if you unplug something):
```bash
ros2 launch biped_bringup hardware.launch.py mock:=true rviz:=true walk:=true
```

Real robot, once calibrated:
```bash
ros2 launch biped_bringup hardware.launch.py
ros2 service call /servo_bridge/torque std_srvs/srv/SetBool "{data: true}"   # robot must be supported/ready
ros2 launch biped_gait walk.launch.py                                        # in another terminal
```
Torque is OFF until that service call. `servo_bridge` refuses to enable torque if any servo doesn't
answer. Once on, it eases into whatever pose is commanded at a slow soft-start speed rather than jumping.
If `/position_controller/commands` stops arriving for 0.5 s, it holds the last pose (does not go limp, does
not drift).

## IMU

Published as `sensor_msgs/Imu` on `/imu` — same topic and message type in both sim and hardware. On
hardware this is a BNO055 over I²C (`imu_node`, from `biped_hardware`); `mock:=true` substitutes a
stationary reading so everything downstream keeps working with no sensor attached.

## Documents

- `docs/CALIBRATION.md` — the calibration procedure.
- `docs/walking_math_explained.md` — the complete derivation of every number the gait uses (centre of
  mass, the support-polygon stability check, forward kinematics, the ZMP timing argument, and how the
  flat-foot ankle angles are solved).

## Licensing note

`biped_hardware`'s servo protocol layer (`biped_hardware/st3215/`, `serial_bus.py`) and the BNO055 driver
are adapted from **AsterisCrack/BipedRobotJetson** (MIT). See `src/biped_hardware/NOTICE.md`. Everything
else in this workspace was written for this project.
