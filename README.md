# BipedRobot V2 — simulation + real hardware

Four ROS 2 (Jazzy) packages. The same walking node and the same `/position_controller/commands` /
`/joint_states` topics drive both Gazebo and the real robot — only what's *underneath* those topics
changes.

| package | what it is |
|---|---|
| `robotv2_description` | URDF, meshes, Gazebo bringup, RViz config. `use_sim:=true/false` switches whether the Gazebo-only parts (gz_ros2_control, the simulated IMU sensor) are included. |
| `biped_gait` | `quasi_static_walk` — the open-loop quasi-static walking node, shared by simulation and hardware. See [biped_gait/README.md](biped_gait/README.md) for the current foot-position gait and offline checks. |
| `biped_hardware` | The real robot: `servo_bridge` (12× Waveshare ST3215 over a serial bus), `imu_node` (BNO055), and the `biped_scan` / `biped_set_id` / `biped_calibrate` / `biped_jog` command-line tools. Has a `mock:=true` mode that behaves like real servos at the packet level, with no hardware attached. |
| `biped_bringup` | One-command launch files: `sim.launch.py`, `hardware.launch.py`. |

```bash
cd ~/biped_ws
colcon build --symlink-install
source install/setup.bash
```




# BipedRobot V2

Build once, then source every terminal:
```bash
cd ~/biped_ws && colcon build --symlink-install && source install/setup.bash
```

---

## Gazebo Simulation

```bash
# Simulation only
ros2 launch biped_bringup sim.launch.py

# Simulation + walking node
ros2 launch biped_bringup sim.launch.py walk:=true
```

Once running, send walk commands in a new terminal:
```bash
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```

Other commands: `forward`, `backward`

---

## MuJoCo Simulation

Install once if you haven't:
```bash
pip install --user --break-system-packages mujoco
```

```bash
# Simulation only
ros2 launch biped_mujoco mujoco.launch.py

# Simulation + walking node
ros2 launch biped_mujoco mujoco.launch.py walk:=true


```
Once running, send walk commands in a new terminal:
```bash
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```

Other commands: `forward`, `backward`

#To push the robot:
```bash
ros2 topic pub --once /mujoco/push geometry_msgs/msg/WrenchStamped \
  "{header: {frame_id: base_link}, wrench: {force: {x: 3.0, y: 0.0, z: 0.0}}}"
```

Reset to standing pose:
```bash
ros2 service call /mujoco/reset std_srvs/srv/Trigger
```

---

## Real Hardware

### 1. Check the port

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

### 2. Scan the servo bus

```bash
ros2 run biped_hardware biped_scan
```

All 12 servos should answer. If any are missing, check the wiring and power.

### 3. Calibration (first time only)

```bash
# Step 1: assign IDs (connect one servo at a time)
ros2 run biped_hardware biped_set_id --guided

# Step 2: set the zero pose (stand robot upright, move joints by hand)
ros2 run biped_hardware biped_calibrate zero

# Step 3: verify each joint moves the right direction
ros2 run biped_hardware biped_calibrate directions
```

### 4. After calibration — check your settings

```bash
ros2 run biped_hardware biped_calibrate show
```

### 5. Run the hardware

```bash

# Hardware with torque ON at start (robot must be supported and ready)
ros2 launch biped_bringup hardware.launch.py startup_position:=true

# Hardware + walking node
ros2 launch biped_bringup hardware.launch.py startup_position:=true walk:=true
```

Once running, send walk commands in a new terminal:
```bash
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```

Other commands: `forward`, `backward`

> **Before enabling torque:** make sure the robot is supported by a stand or held by hand, and that calibration is done. Torque enables holding at the current joint positions, then eases into commanded poses slowly.

### Useful extras

```bash
# Fine-tune a joint's zero position by eye
ros2 run biped_hardware biped_jog

# Watch temperatures and voltage
ros2 topic echo /diagnostics
```
