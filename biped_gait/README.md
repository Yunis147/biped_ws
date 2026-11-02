# Quasi-static walking with IMU feedback

This executable now uses IMU feedback, a smaller planned lean, and guarded slow
landing. **Only `biped_gait` was changed.** CPG, the servo bridge, the URDF, and the
separate balance package are unchanged. The unresolved communication/status-read
issues in the bridge are not fixed or bypassed by adding IMU feedback.

Your hardware results showed two failures: falling during the large lean and a
foot pushing the ground during placement. The changes address them separately:

| Behaviour | Previous executable | Updated executable |
|---|---|---|
| Planned sideways body lean | 11.46° | **4.58°** |
| Lowering for sideways reach | 40 mm | **28 mm** |
| Up/down change in weight transfer | 20 mm | **8 mm** |
| Actual posture feedback | None | Roll/pitch and gyro feedback, **±1.5° correction** |
| Lift | 8 mm before swing | **6 mm first, increasing to 10 mm during swing** |
| Placement | Timed descent | **At most 2 mm/s planned descent**, with IMU/encoder guards |
| Progress to the next phase | Timed holds | Minimum dwell plus measured posture/tracking settling |
| Commands before start | Repeated zero targets | **None** |

The executable entry remains `quasi_static_walk`. Its ROS interface is in
[walking_node.py](biped_gait/walking_node.py), the controller in
[feedback_walk.py](biped_gait/feedback_walk.py), and active settings in
[config/walk.yaml](config/walk.yaml). Geometry/IK helpers remain in
[quasi_static_walk.py](biped_gait/quasi_static_walk.py).

Some helper constants and calls retain the older defaults for compatibility
with packages that import the pure math, including CPG. **Those legacy defaults
do not set this node's lean.** The executable explicitly passes the new profile
and validates it before enabling its command timer. The old open-loop ROS node
has been removed; `--check` also checks the new profile.

## Run after your existing robot bringup

Build from the workspace root, then source the installation in each terminal:

```bash
cd /external_storage/manasa/biped
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to biped_gait \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Start your usual `biped_bringup` with its walker disabled (do not pass
`walk:=true`). Keep the IMU enabled. Stop the CPG node and any actuator-enabled
`biped_balance` node completely so there is one joint-command publisher.
Monitor-only `biped_balance` can still observe/record.

Then start this node separately:

```bash
ros2 launch biped_gait walk.launch.py
# Equivalent, using default settings:
# ros2 run biped_gait quasi_static_walk
```

Hardware mode uses monotonic computer time, fresh `/joint_states`, the existing
`servo_bridge` diagnostics, and `/imu` in `imu_link`. It never enables torque or
calls motor services. Wait for bringup to finish positioning near calibrated
zero. Gyro bias is calibrated automatically from at least one second of quiet
IMU samples before movement; this does not redefine an existing lean as upright.
Your completed IMU direction/monitor tests do not need repeating.

Use another sourced terminal:

```bash
# Optional: see corrections while both feet remain planted, inside THIS node.
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stand'}"
# One right-foot step and return to the centred bent-knee stance:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'step'}"
# Continuous walking:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
# Finish the airborne step and centre:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```

Wait for `standing` before issuing another start/direction command. `backward`
selects reverse walking. `hold` freezes the last target and latches a fault;
`calibrate` explicitly recalibrates gyro bias while stationary. No automatic
walking occurs at launch. Start requires the measured joints near calibrated
zero, or the last held nominal CPG-free stance, and approximately stationary.
A small synchronized approach captures the measured starting pose first.
If a new path must be calculated from a stopped stance, calculation runs in a
background worker while sensor checks and current stance commands continue.
`preparing_path` in status shows this wait; `stop` cancels the request.

```bash
ros2 topic echo /walk/status --once
```

Status includes planned/measured lean, correction, the joint with largest
tracking error, predicted swing clearance, and a **latched first-fault snapshot**.
The snapshot stores phase, joint targets/measurements, IMU tilt, landing cues,
source ages and bridge diagnostics. `hardware_ready` reports current input readiness; it does
not clear a latched fault. Support the robot and restart from a known stance
after a fault. Hold is not fall recovery and does not turn torque off.

The following cases block or stop movement: missing/stale IMU or encoder data,
bridge health errors, torque OFF/startup positioning, a competing command
publisher, excessive tilt, persistent motor lag, or an infeasible feedback pose.
The bridge can still republish cached encoder values with fresh stamps; this
node cannot prove that every physical status read was fresh.

For simulation use `ros2 launch biped_gait walk.launch.py sim:=true`, with a
simulator publishing `/clock`, `/imu` and `/joint_states`, and its other walker
disabled. Simulation mode does not require physical bridge diagnostics. The
reported physics results below are MuJoCo results; Gazebo was not run here.

## Why the robot still needs to move its weight

Imagine a vertical line from the average position of the robot's mass to the
floor. With one foot up, that line needs to lie inside the supporting sole.
This is a static model check, not a measurement of actual ground forces.
The new target is **8 mm inward from the sole centre**, reducing the body travel
needed compared with the previous sole-centred target. It leaves about 10.49 mm
of sideways support margin in the current model.

The following basic calculations describe the URDF. They are separate from the
chosen motion settings and cannot establish that the model matches the real
assembled robot.

## 2. Units and directions

The code uses metres, kilograms, seconds, and radians.

```text
1 metre = 1000 mm
1 kilogram = 1000 g
angle in degrees = angle in radians × 180 / pi
pi is about 3.14159

X = forward
Y = left
Z = up
```

For example, `0.020 m = 20 mm`, and `0.20 rad = 11.46 degrees`.
“Roll” means leaning sideways. With these axes, negative body roll leans left;
positive body roll leans right. These are planned angles, not IMU measurements.

## 3. What the updated URDF says

Each leg has six joints. Following the links from the body to the ground gives:

```text
body → hip yaw → hip roll → hip pitch → knee → ankle roll → ankle pitch → foot
```

Yaw turns around the up/down axis. Roll tilts sideways. Pitch tilts forward/back.
The physical chain has ankle **roll before pitch**. The command message lists
ankle **pitch before roll**. The code handles both orders explicitly.

### Mass and the centre of each separate part

The table uses grams and millimetres for readability. Each centre is measured
from **that part's own origin**, not from the floor or from the body.
These are the `<mass>` and `<inertial><origin>` entries of the current URDF.

| Part | Mass (g) | Centre X (mm) | Centre Y (mm) | Centre Z (mm) |
|---|---:|---:|---:|---:|
| Body/base | 325.1 | 0.766651 | -1.787673 | 33.975024 |
| Right upper hip | 86.5 | -0.111097 | -34.974486 | -12.755038 |
| Right lower hip | 22.2 | -18.643150 | -5.230758 | -5.418970 |
| Right thigh | 157.1 | 0.672256 | -25.691949 | -25.638526 |
| Right calf | 92.1 | 1.122282 | -23.608079 | -40.654266 |
| Right ankle | 22.8 | -16.743150 | -3.804758 | -4.582970 |
| Right foot | 102.5 | 19.198808 | -24.336599 | 0.423413 |
| Left upper hip | 86.5 | -0.679295 | 31.238295 | -12.755038 |
| Left lower hip | 22.2 | -19.243150 | 2.687242 | -5.418970 |
| Left thigh | 157.1 | 0.072106 | 24.055129 | -26.041769 |
| Left calf | 92.1 | 0.522282 | 21.649235 | -40.654266 |
| Left ankle | 22.8 | -17.343150 | 3.261242 | -4.582970 |
| Left foot | 102.5 | 18.705493 | 24.676201 | 0.608178 |

Add one leg:

```text
86.5 + 22.2 + 157.1 + 92.1 + 22.8 + 102.5 = 483.2 g

Whole robot = 325.1 + 483.2 + 483.2
            = 1291.5 g
            = 1.2915 kg

Weight on Earth = mass × gravity
                = 1.2915 × 9.81
                = 12.6696 newtons
```

The `imu_link` has zero mass in this URDF, so it adds nothing to that sum. The
IMU is not used for control. Its physical mass must already be included in the
URDF's other mass entries for this total to represent the real robot.

The left and right centres are slightly different. The code reads each side
separately instead of assuming they are exact mirror images.

The URDF also gives each part's **inertia**: how hard it is to speed up its
rotation. Inertia matters for moving-body forces. The static CoM calculation
uses mass and position, not inertia. The physics tests use the URDF inertias
through the existing simulator. This walker does not solve full moving-body
force equations or command motor torque.

### Distances between joint origins

These offsets are measured from the preceding part. They are unchanged in the
URDF update. Values below are in millimetres.

| Joint | Right X, Y, Z | Left X, Y, Z |
|---|---|---|
| Hip yaw | -1.590, -21.672, -0.150 | -1.590, +21.672, -0.150 |
| Hip roll | +19.100, -30.829, -40.776 | +19.100, +30.829, -40.776 |
| Hip pitch | -18.625, +20.625, -33.900 | -18.625, -20.625, -33.900 |
| Knee | 0, -2.000, -72.052 | 0, +2.000, -72.052 |
| Ankle roll | +18.625, -19.101, -72.184 | +18.625, +19.101, -72.184 |
| Ankle pitch | -18.625, +20.625, -33.900 | -18.625, -20.625, -33.900 |

Right hip yaw turns about `-Z`, while left hip yaw turns about `+Z`. Right hip
and ankle roll turn about `-X`; left ones turn about `+X`. Both hip pitches and
ankle pitches turn about `+Y`; both knees turn about `-Y`.

This is why equal physical motions do not always need equal signed commands.
The code uses the axes from the URDF when calculating each joint's effect.

### The floor-contact rectangle is read from the foot mesh

A mesh is a list of small triangles. The URDF tells us how to scale it and where
to put it relative to the foot joint:

```text
point in foot coordinates = mesh point × mesh scale + collision origin
```

Do this separately for X, Y, and Z. Using the STL numbers directly, or using
only the mesh scale, would give the wrong foot dimensions and standing height.

For example, the lowest foot-mesh Z is about `-284.535` in the STL's units.
The new right-foot Z scale is `0.000962561189483049`, and the collision origin Z
is `0.252962 m`:

```text
sole Z = -284.535 × 0.000962561189483049 + 0.252962
       = -0.020920675 m
       = -20.920675 mm relative to the foot-link origin
```

The code uses the bottom corners, not the bounds of parts higher up the foot.
For the current files:

| Quantity | Right foot | Left foot |
|---|---:|---:|
| Sole X interval in foot coordinates (mm) | -28.729181 to +61.270819 | -28.729181 to +61.270819 |
| Sole Y interval in foot coordinates (mm) | -40.738216 to -3.760237 | +3.855143 to +40.930303 |
| Length along X (mm) | 90.000000 | 90.000000 |
| Width along Y (mm) | 36.977978 | 37.075159 |
| Sole Z relative to foot origin (mm) | -20.920675 | -20.920675 |

The model reader currently requires flat rectangular soles, the present six-joint
chains, and the present joint-axis directions. It rejects unsupported changes
instead of quietly applying the wrong formulas. The current xacro has concrete
link/joint entries; moving them into xacro macros would require updating this
reader to expand those macros first.

## 4. Calculate the zero-angle standing height

At zero angles the joint origins can simply be added. Their downward Z distances
sum to:

```text
0.150 + 40.776 + 33.900 + 72.052 + 72.184 + 33.900
= 252.962 mm from the body origin to the foot-link origin

Standing height = 252.962 + 20.920675
                = 273.882675 mm
```

This is the height of `base_link` above the soles, not the height of the head.
The old script used a sole offset of `31.573 mm` and a zero height of
`284.535 mm`; those values no longer describe the current collision geometry.

At zero angles, the sole centres relative to the body are:

```text
right: X = 15.155819 mm, Y = -54.601227 mm
left:  X = 15.155819 mm, Y = +54.744723 mm

Sideways centre-to-centre spacing = 54.744723 - (-54.601227)
                                = 109.345950 mm
```

For example, right-foot X is the sum of joint X offsets, `-1.115 mm`, plus the
middle of the sole's X interval:

```text
-1.115 + (-28.729181 + 61.270819) / 2 = 15.155819 mm
```

When starting or restarting, the code uses the current held foot spacing. It
does not drag planted feet back to a predetermined stance width.

## 5. Calculate the whole robot's centre of mass

A heavy part counts more than a light part. For a simple example, put 1 kg at
X = 0 cm and 3 kg at X = 10 cm:

```text
average X = (1 × 0 + 3 × 10) / (1 + 3) = 7.5 cm
```

For the robot we do exactly the same calculation for all 13 massive parts:

```text
CoM X = sum of (each mass × that part's X position) / total mass
CoM Y = sum of (each mass × that part's Y position) / total mass
CoM Z = sum of (each mass × that part's Z position) / total mass
```

All positions must first be measured from the same origin. The local centres in
section 3 cannot be averaged directly.

At zero angles, add the joint offsets to each local centre. For example, the
right thigh's origin has Y = `-31.876 mm`, and its local mass centre has
Y = `-25.691949 mm`. Its mass centre relative to the body is therefore
`-57.567949 mm`. Its contribution to the sideways numerator is
`0.1571 × (-0.057567949)` in kg·m.

When a joint bends, rotate the parts below it before adding their positions.
For a turn around Y, one basic rotation rule is:

```text
new X = X × cos(angle) + Z × sin(angle)
new Z = -X × sin(angle) + Z × cos(angle)
```

The code applies the corresponding rule around each joint's own axis. A
rotation matrix is just a compact table for doing those three-coordinate sums.
This chain calculation is called **forward kinematics**: start with joint angles
and find where the parts end up.

For the current zero-angle pose the complete mass sum gives, relative to the body:

```text
CoM ≈ [2.503100, -1.065391, -93.9740] mm
```

Adding the body height puts that CoM about `179.9087 mm` above the floor. During
walking, both legs move and the body leans, so the code recalculates the sum at
every planned pose. The mass of the lifted leg still counts.

## 6. Where the smaller lean and heights come from

The zero-angle body height is 273.882675 mm. The active profile uses:

```text
centred body height = 273.882675 - 20 = 253.882675 mm
single-support body height = 273.882675 - 28 = 245.882675 mm
planned lean = 0.08 radians = 4.58 degrees
```

These 20/28 mm and 4.58° values are chosen motion settings checked against the
whole path, rather than quantities determined by mass alone. A lower initial
lift of 6 mm avoids an ankle-limit violation at the trailing foot. Clearance
increases to 10 mm as that foot advances. The supporting foot remains anchored;
all six joints are solved together to keep the soles level and pointing forward.

The narrower sole is about 36.977978 mm wide. At its centre the sideways margin
would be `36.977978 / 2 = 18.488989 mm`. Moving the desired CoM 8 mm inward gives:

```text
nominal support margin = 18.488989 - 8 = 10.488989 mm
```

With both feet planted, the body rises toward the centred height during transfer
and lowers again for sideways reach. Both planted feet keep their spacing.
The body X/Y position is solved repeatedly from the full mass sum, including the
raised leg. The body origin itself is not the centre of mass.

## 7. How IMU corrections work

The tilt estimator is copied from the hardware-checked `biped_balance` utility.
It can use a valid orientation quaternion, or fuse a six-axis accelerometer and
gyroscope with a complementary filter. `imu_mount_rpy` rotates the sensor into
the robot's body axes. Gyro rates are radians/second; accelerometer readings
include gravity, approximately 9.81 m/s² at rest. Source stamps and frame IDs are
checked. Invalid or stale IMU input never silently switches to open-loop walking.

For each roll/pitch direction:

```text
tilt error = measured tilt - planned tilt
requested correction = -0.30 * tilt error
                       -0.04 * (measured gyro rate - planned rotation rate)
```

The same principle of gyro-based hip/ankle balance is used in
[ROBOTIS walking controllers](https://emanual.robotis.com/docs/en/popup/op3_WalkingParam.msg/).
The gains here are conservative initial settings, not tuned physical-robot gains.

For example, suppose the planned left lean is -4.58° but the body has leaned
left to -6°. The error is -1.42°, so the proportional term requests +0.426° of
body correction, reducing the extra left lean. It does **not** try to cancel the
entire planned lean and hold zero degrees during a weight transfer.

Corrections are capped at ±1.5° and changed gradually. Authority fades near a
straight knee, where fixed-foot attitude changes become difficult. Correction
authority is halved during landing. A measured absolute torso tilt over 8°
freezes the last valid target instead of continuing the planned motion.

The correction is converted into coordinated hip, knee and ankle targets. For
a small body rotation `R`, each nominal relative foot pose becomes `R.T * pose`.
In plain terms: turn the body while keeping both intended foot positions fixed
in the room. This works for a leaning nominal pose too. No individual joint
angle is clipped to hide an unreachable correction; the entire progress/correction
is reduced together. Complete joint updates are limited to 0.25 rad/s.

## 8. Landing: limiting the push, without pretending to measure pressure

A position-controlled servo keeps trying to reach its target after early ground
contact. IMU feedback alone cannot measure that contact or the downward force.
The controller therefore adds several limits **in addition** to posture feedback:

- Planned descent speed is at most **2 mm/s**. Lowering 10 mm takes at least
  9.375 seconds with the smooth start/stop curve, because its peak speed is
  `1.875 * height / duration`.
- The nominal trajectory ends at the assumed floor plane. There is no extra
  downward search stroke or forced knee straightening after placement.
- During placement, a tilt error over **1.5°**, horizontal gyro rate over
  **0.12 rad/s**, or joint tracking error over **0.035 rad** immediately holds
  the last valid target and latches a landing fault.
- Relative foot height is estimated using **measured IMU tilt and encoder angles**.
  An estimated swing-sole penetration over **1 mm** relative to the supporting
  sole blocks further descent.
- Near the ground (estimated clearance below 2 mm), a swing sole tilted more than
  **2°** also interrupts descent. This catches some heel/toe-first contacts that
  a body-tilt check alone can miss.
- Before lifting the next foot, body motion and joint tracking must settle.
  The measured-pose mass model must also retain at least 5 mm supporting margin.
  Otherwise the next lift is blocked.

A trip is reported as **contact NOT confirmed**. The node does not automatically
resume, lower the foot farther, declare touchdown, or switch support after that
fault. It keeps the last valid target where inputs remain healthy; stale input
or a bridge fault suppresses output so the existing bridge watchdog takes over.

These checks can miss early contact without appreciable tilt/tracking/height
cues, and are affected by sensor/model error. They do **not** guarantee that the
foot cannot press the ground. `contact_measured` remains `false` in status.
Normal successful placement still assumes contact on a level floor; there are
no pressure or force sensors. Physical validation with support remains necessary.

## 9. Phases and timing

The sequence remains quasi-static: lower/centre, transfer left, lift right, swing
right, place right, transfer right, lift left, swing left, place left, repeat.
Stop finishes an airborne step and centres with the feet where they landed.
Each phase uses a smooth progress curve:

```text
s = 10*u^3 - 15*u^4 + 6*u^5       (u goes from 0 to 1)
```

This starts/stops the planned Cartesian path gently. There are 81 solved joint
poses per phase. Cubic interpolation joins them with continuous joint velocity.
Nominal timing respects 0.18 rad/s and 0.30 rad/s², with slower landing added.
The full feedback command also respects the 0.25 rad/s joint-update limit.

A phase waits at least its settling dwell, then requires quiet body rotation,
posture error within 1.5°, and tracking error below 0.03 rad for 0.4 seconds.
Failure to settle blocks the next phase. General servo lag above 0.04 rad pauses
progress; lag lasting over two seconds or error over 0.15 rad latches a fault.
Those safeguards are not loosened to hide the unresolved hardware lag issue.
Landing uses the tighter immediate 0.035 rad check described above.

## 10. Verification and limitations

Run offline checks without connecting to the robot:

```bash
cd /external_storage/manasa/biped/src
python3 -B biped_gait/biped_gait/quasi_static_walk.py --check
python3 -B -m unittest discover -s biped_gait/test -v
```

The tests independently read the URDF and scaled STL corners, check the new
profile's complete interpolated paths, verify feedback directions and fixed
world-foot poses, speed/correction limits, missing sensor handling, first-fault
snapshots, single-step stopping, and interrupted landing cues. Optional physics
tests require MuJoCo and xacro.

All 25 gait tests pass, together with 16 unchanged CPG compatibility tests.
One physics case places a 4 mm raised patch beneath the swing toe just before
the first descent: the controller interrupts placement and retains its last
target rather than completing that landing. This checks one early-contact case,
not every possible ground contact.

The tested MuJoCo model uses the current URDF, 50 Hz control, assumed servo
stiffness 50 N·m/rad, and 50% of its assumed stall torque. With actual simulated
IMU and encoder inputs, the controller completes three landings in each direction
and stops. Peak tilt is about 4.94°. In a standing 3 N / 0.1 s forward-push test,
feedback reduces peak tilt from about 4.19° to 3.75° and the integrated squared
tilt from 3.23 to 2.81 degree²·seconds. These are simulation results, not hardware
gain validation or a torque rating.

An isolated ROS test with generated hardware feedback verifies automatic quiet
calibration, powered-stand command flow, corrective direction, continued commands
and sensor callbacks during a background path rebuild, and output suppression
after IMU loss. No real servo bus is opened by those tests.

The description package still notes that its CAD mass model and restored meshes
have not had articulated alignment validated. This controller cannot establish
that geometry matches the assembled robot, detect every cached driver sample,
or measure contact forces. The reported hardware communication/status-read
problem still needs its own investigation. CPG is unchanged; IMU integration
into CPG is deferred until that issue is resolved.
