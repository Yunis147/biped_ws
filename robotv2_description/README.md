# robotv2_description — BipedRobot V2 on ROS2 Jazzy + Gazebo Harmonic

Ported from `envs/assets/robotV2/Robot_description` (the version confirmed working in ROS1/Gazebo11),
not from V1. Native Ubuntu 24.04, no Docker.

## What was fixed vs. the original repo package
| Problem in original | Fix here |
|---|---|
| `package.xml`/launch files said `Robotv2URDF_description`, `Robot.urdf` said `Robot_description` | one name everywhere: `robotv2_description` |
| Launch files pointed at a non-existent `Robotv2URDF.xacro` | uses `urdf/robot.urdf.xacro` |
| Mesh URIs pointed at the wrong package | `package://robotv2_description/meshes/...` (26 refs, all files exist) |
| `libgazebo_ros_control` + `Robot.trans` (ROS1 only) | `<ros2_control>` + `gz_ros2_control` |
| Foot friction 0.2 (very slippery) | 1.0 on feet and ground |
| `selfCollide` on every link | removed (jitters in Harmonic) |

Verified offline: xacro expands, the 12 revolute joints match `ros2_control` and `controllers.yaml`,
all mesh files exist, and forward kinematics on the collision meshes gives the lowest foot point
0.2845 m below `base_link` (spawn height 0.29 m).
**Not** verified: running inside Gazebo itself (needs your machine).

## Install (Ubuntu 24.04 + ROS2 Jazzy)
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

## Build
```bash
mkdir -p ~/biped_ws2/src
tar xzf ~/Downloads/robotv2_description_ros2_jazzy.tar.gz -C ~/biped_ws2/src
cd ~/biped_ws2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Stage 1 — RViz only
```bash
ros2 launch robotv2_description display.launch.py
```
RViz opens with Fixed Frame `base_link` and the robot model already added; sliders move the joints.

## Stage 2 — Gazebo
```bash
ros2 launch robotv2_description gazebo.launch.py
```
Order of events: Gazebo starts, ~5 s later the robot is spawned, then `joint_state_broadcaster`
and `position_controller` load. Check in another terminal (with `source install/setup.bash`):
```bash
ros2 control list_controllers      # both should say "active"
ros2 topic echo /joint_states --once
```

## Stage 3 — move a joint
Command order (12 values, radians):
`r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee, r_ankle_pitch, r_ankle_roll, l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee, l_ankle_pitch, l_ankle_roll`

Bend the right knee:
```bash
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0, 0.5,0,0,  0,0,0, 0,0,0]}"
```
Back to zero:
```bash
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0, 0,0,0,0,0,0]}"
```

## IMU (needed later for balance feedback)
```bash
ros2 launch robotv2_description gazebo.launch.py world:=flat_world_imu.sdf
ros2 topic echo /imu --once
```
Uses the Sensors system (ogre2). If that world crashes on your graphics setup, stay on the default world.

## Troubleshooting — paste the output of these if something fails
```bash
ros2 topic echo /robot_description --once | head -5     # empty => robot_state_publisher / xacro failed
ros2 node list                                          # expect /robot_state_publisher, /controller_manager
gz topic -l | head -30
echo $GZ_SIM_RESOURCE_PATH                              # should end with .../install/robotv2_description/share/..
```
Joint limits (from the URDF): hip roll -25°..90°, hip pitch ±90°, knee ±120°, ankle roll ±80°,
ankle pitch -35°..85°, hip yaw ±45°. Stay inside them.

Note on the position interface: `gz_ros2_control` drives joint velocity proportional to position
error, so joints behave like stiff servos (no torque limit). That is a reasonable stand-in for the
real hobby-servo robot, but real motors will be weaker/laggier than this simulation.
