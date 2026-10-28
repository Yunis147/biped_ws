# Biped balance experiments

This separate ROS 2 package is the place for new hardware balance work. It does
not modify `biped_gait`, hardware drivers, URDF, or existing launch files. It reads
the original gait's robot geometry and inverse kinematics as a dependency.

The current stage covers IMU/encoder monitoring, bent-knee standing, bounded
IMU PD feedback, and small weight transfers with **both feet planted**. It does
not walk, lift a foot, detect ground contact, or recover from a fall. Walking
integration follows after the standing and transfer tests work on hardware.

## First run: monitor and record

Keep your existing servo bridge and IMU publisher running. This package expects
`/imu` (`sensor_msgs/Imu`), `/joint_states` (`sensor_msgs/JointState`), and the
existing `/diagnostics` from `servo_bridge`. Hardware nodes must use wall time.

```bash
source /opt/ros/jazzy/setup.bash
cd /external_storage/manasa/biped
colcon build --symlink-install --packages-up-to biped_balance
source install/setup.bash
ros2 launch biped_balance balance.launch.py record_path:=/tmp/biped-balance-first.csv
```

In another sourced terminal:

```bash
ros2 topic echo /balance/status --once
```

The default launch creates **no publisher** on `/position_controller/commands`.
It records body tilt/rate, measured joint angles and speeds, observed actuator
targets, tracking error, raw servo loads, minimum servo voltage and maximum
temperature. CSV is recorded at 10 Hz; choose a new file
name on each run because existing files are not overwritten. `/balance/preview`
contains model targets only and is not connected to the servo command topic.

Your first hardware task is to record 20–30 seconds while supported: upright
and still, then small known roll and pitch movements, then upright again.
Confirm that `roll_pitch_deg` changes on the expected axis and returns close to
zero. Body convention is X forward, Y left, Z up: raising the robot's left side
gives positive roll; tipping the torso forward gives positive pitch. Gyro signs
must agree with the direction of these movements.

When upright and stationary, expect roll/pitch near zero, gyro near 0 rad/s,
and acceleration magnitude near 9.81 m/s² (the acceleration channel must include
gravity). Check for fresh sensor timestamps and all 12 named joints.

If the mounting differs, set `imu_mount_rpy` in **this package's** config. These
are radians defining `R_body_from_sensor = Rz(yaw) Ry(pitch) Rx(roll)`; the node
rotates gyro, acceleration, and quaternion orientation consistently. Do not
change signs by guessing controller gains. A frame name alone does not rotate
measurements. `imu_frame` defaults to `imu_link` and can be changed for your
actual IMU publisher.

`imu_source: auto` uses a valid fused quaternion when available. Otherwise it
uses a gyro/accelerometer complementary filter. Set `imu_source: raw` for a
raw-only sensor whose driver leaves a misleading identity quaternion. No
magnetometer or absolute heading is needed. The filter reduces accelerometer
correction when acceleration magnitude differs from gravity; it cannot remove
every acceleration-induced tilt error.

If `/imu` is missing, check your existing IMU driver's log before proceeding.
The repository's hardware YAML currently spells its BNO055 mode `imupLus`,
whereas that driver accepts lowercase `imuplus`. If using that driver, you can
run it with `ros2 run biped_hardware imu_node --ros-args -p mode:=imuplus`
after stopping any duplicate IMU node. This is a runtime override; this package
does not edit the hardware configuration. A different six-axis driver can be
used as long as its message units, frame, covariance flags, and timestamps agree.

## Second run: supported standing experiment

Use a tether or overhead support. Verify servo zeros/directions and IMU axes
first. Stop the original walking node before actuator testing; an idle walker
still publishes zeros. The new controller refuses to start if another actuator
command publisher is detected. ROS publisher discovery is a diagnostic, not a
hard realtime ownership guarantee.

Launch with the existing hardware bridge running, using a new CSV file:

```bash
ros2 launch biped_balance balance.launch.py actuate:=true imu_axes_verified:=true record_path:=/tmp/biped-balance-standing.csv
```

The node does not enable torque, change servo PID/limits, or move on launch.
Enable torque through your existing bridge procedure while the robot is
supported. It must report healthy `torque ON` diagnostics, with startup
positioning completed. The physical initial pose must be near the calibrated
zero stance or the model's bent-knee center stance.

Hold the torso upright and stationary for at least 1.5 seconds, then:

```bash
ros2 topic pub --once /balance/cmd std_msgs/msg/String "{data: 'calibrate'}"
ros2 topic pub --once /balance/cmd std_msgs/msg/String "{data: 'stand'}"
```

Wait for `/balance/status` to report `standing`. `stand` approaches a known
stance slowly and lowers into the existing model's 265 mm bent-knee stance.
Balance remains OFF during this preparation. It does not home an arbitrary
folded pose. When standing and supported, request:

```bash
ros2 topic pub --once /balance/cmd std_msgs/msg/String "{data: 'balance_on'}"
```

Check the feedback direction with very small controlled disturbances. Initial
gains are conservative experiment settings, **not tuned hardware gains**. A
positive measured torso tilt requests a negative torso correction. Leg IK
coordinates hip, knee, and ankle targets using the actual mirrored joint axes.
Under the commanded torso attitude, both nominal foot positions and headings
are preserved. This mathematical constraint does not establish actual contact
or exact servo tracking.

Only after standing feedback is repeatable, try one planted-foot transfer:

```bash
ros2 topic pub --once /balance/cmd std_msgs/msg/String "{data: 'lean_left'}"
# Wait for standing/location=left, then:
ros2 topic pub --once /balance/cmd std_msgs/msg/String "{data: 'center'}"
```

`lean_right` is also available. The default pelvis transfer is only 15 mm, with
both feet down. It is not the original gait's full single-support weight shift.

Commands `hold`, `balance_off`, `monitor`, and `status` are available on
`/balance/cmd`. `hold` freezes the last actuator target. `balance_off` gradually
removes feedback correction while standing. `monitor` stops command publishing
and leaves the bridge holding its last output. None disables motor torque.

Fresh feedback, encoder tracking, joint limits, command speed, and torso tilt
are checked. Faults latch and hold the last target where possible; a conflicting
publisher or unhealthy/torque-OFF bridge suppresses output. Holding a pose is
not fall recovery. Support the robot before recovery; use `monitor`, inspect
the reason, and repeat preparation. A long IMU data gap invalidates calibration.
Changing settings requires restarting this node. Gyro calibration is in memory
for that run and never hides a constant body lean by redefining upright.

## What the measurements can tell us

Encoder position/speed and IMU roll/pitch/rates can reveal tracking lag,
oscillation, wrong correction signs, and transfer-induced tilt. Raw ST3215 load
values are read from existing bridge diagnostics (normally 1 Hz). They are not
calibrated torque in N.m or measured sole pressure and are not used to assert
touchdown. Source timestamps are checked, but software cannot detect every
case where a driver republishes cached physical measurements.

The next controller changes should be based on the recorded standing and
transfer results: tune gains, then introduce one small lift/landing, then
alternating steps in this package. No physical hardware validation has been
performed by creating this package.

## Software checks

From the source directory:

```bash
python3 -B -m unittest discover -s biped_balance/test -v
```

Tests cover IMU mounting/fallback/bias, mirrored-axis kinematics, fixed-foot
attitude corrections, planted-foot spacing, monitor-only behavior, publisher
conflicts, encoder lag, speed limits, and fault holds. They do not establish
stability on the physical robot.

Design references: [ROBOTIS gyro-based balance parameters](https://emanual.robotis.com/docs/en/popup/op3_WalkingParam.msg/)
and [Analog Devices on accelerometer tilt limitations](https://www.analog.com/en/resources/app-notes/an-1057.html).
