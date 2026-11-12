# Quasi-static walking: model and joint positions

This package plans slow walking from the robot's URDF dimensions, link masses,
foot meshes and servo positions. **It does not read `/imu`, calibrate a gyro,
apply tilt corrections, or wait for an IMU check.** The previous IMU controller
and its parameters have been removed from this package.

The new default step is **30 mm in either direction**. Planned torso roll is
**2.29 degrees**, compared with the previous active profile's 3.44 degrees.
The pelvis still moves sideways to put the robot's weight over the supporting
foot. The foot is lowered using the measured bent-leg geometry, at no more than
**1.5 mm/s**, with a finite lowering distance and no intended floor penetration.

This is a model of level-ground walking. Joint encoders alone cannot measure
body tilt relative to gravity, actual floor height, ground force, or slipping.
The calculated floor is accurate only when the supporting sole is flat on the
same level floor and the dimensions/calibration match the robot. Simulation
success does not establish that the physical robot will walk without falling.

## Build and run

From the workspace root:

```bash
colcon build --packages-select biped_gait --symlink-install
source install/setup.bash
```

Start MuJoCo with the existing launch file:

```bash
ros2 launch biped_mujoco mujoco.launch.py walk:=true
```

Then request motion in another sourced terminal:

```bash
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'step'}"
```

`walk:=true` starts the node; it does not automatically send a walking request.
Use `start` for continuous forward walking, `backward` for reverse walking,
`stand` for the bent standing pose, and `stop` to finish the current step and
return to a centred stance. `hold` retains the last target and latches a fault.
`status` requests a status update. Read it with:

```bash
ros2 topic echo --once /walk/status
```

For hardware, use the existing bringup. Its existing `imu` launch argument can
turn off the unused IMU driver:

```bash
ros2 launch biped_bringup hardware.launch.py walk:=true imu:=false
```

Or start your normal bringup with `walk:=false`, then run only this walker:

```bash
ros2 launch biped_gait walk.launch.py
```

Start upright at calibrated joint zero, with both soles on a level floor.
Fresh joint states and healthy bridge diagnostics are required; torque control
still belongs to bringup. The walker publishes nothing before a motion request.
For an initial physical test, support the robot and request one `step`.

## 1. What keeps a slow robot balanced?

The centre of mass is the average position of all the robot's mass, with heavier
parts counting more. If a 1 kg part is at position 0 and a 2 kg part is at
position 3 cm, their combined centre is:

```text
centre = (1 × 0 + 2 × 3) / (1 + 2) = 2 cm
```

The code uses this same calculation in three directions, for every link:

```text
M = sum of link masses
C = sum(mass of each link × position of its mass centre) / M
weight = M × 9.81
```

Positions change when joints bend, so the mass of the lifted leg is included.
From the current URDF:

| Part | Mass |
|---|---:|
| Base | 0.3251 kg |
| Right leg, all six links | 0.4832 kg |
| Left leg, all six links | 0.4832 kg |
| Total | **1.2915 kg** |

Each leg's link masses, from hip to foot, are 0.0865, 0.0222, 0.1571,
0.0921, 0.0228 and 0.1025 kg. Their sum is 0.4832 kg. The weight is about
12.67 N. These values are read from the URDF rather than copied into the planner.

Imagine dropping a vertical line from `C` to the floor. During a very slow
movement, that line should land inside the area covered by the supporting
sole. With both feet planted, the available area includes the space between
the feet. With one foot lifted, only the other sole counts.

The feet are about 90 mm long and 37 mm wide. The exact bottom outlines come
from the scaled foot meshes. The planner requires the mass centre's projection
to stay at least **8 mm inside** the available area. This is a margin against
small errors, not a measurement that the foot is carrying the expected load.

## 2. Why there is still sideways motion

Bending both knees lowers the body. Bending a single knee can also move the
pelvis, but it only helps balance if the mass centre moves over the other foot.
Lowering the body alone cannot move a mass centre that is between the feet into
one foot's support area.

The planner therefore moves the pelvis sideways while keeping the soles flat.
It uses a small torso angle to keep all six joints of each leg within their
allowed ranges. Zero torso roll was not reachable for this profile at the
initial stance width with the required support margin. Reducing the angle
without checking reach would just produce invalid joint commands.

The changes that reduce lean are:

1. Use **0.040 radians = 2.2918 degrees** of torso roll in single support.
2. Bring the feet to **90 mm centre spacing** during the first swing, instead
   of keeping the initial 109.346 mm spacing for every step.
3. Move the mass centre forward/backward with the swing, rather than forcing
   it to stay at the loaded foot's fore/aft centre.
4. Adjust body height during weight transfer so both legs remain reachable.

The first right swing includes about **19.346 mm inward movement**:
`109.346 - 90 = 19.346 mm`. This happens while that foot is lifted. Subsequent
steps keep the narrower spacing. It leaves about 53 mm between the sole edges;
MuJoCo tests also check for contact between the two legs. This is a planned
stance change, not a change to servo calibration or the URDF.

For left support, the desired sideways mass-centre position is 10 mm inward
from the left sole centre. Right support uses the mirrored value. Half the
foot width is about 18.49 mm, giving an inward-edge margin of:

```text
18.49 - 10 = 8.49 mm
```

The checked minimum margin over both complete default paths is **8.489 mm**.
The outer-edge distance is larger. Fore/aft position is the midpoint of the
two sole centres; with a 30 mm step that is 15 mm from the loaded sole centre,
well inside its 45 mm half length.

These angles, heights and spacing are selected settings that pass the robot
geometry checks; they are not unique values forced by physics.

## 3. What a 3 cm step means

When the right foot lands, its centre is 30 mm ahead of the left foot. Next,
the left foot lands 30 mm ahead of the right foot. In steady walking, one foot
therefore travels 60 mm from its previous landing to its next landing:

```text
right lands at x = 30 mm
left lands at x = 60 mm
right lands at x = 90 mm
```

Backward uses the same distances with the opposite sign. Physical body travel
can differ because this is a position-command gait without slip measurement.

## 4. Joint angles, bent knees and body height

A link starts at its joint position. Turning that joint rotates everything
below it. The code repeatedly adds the joint offsets and applies the joint
rotations, from the torso through all six joints to the sole. This calculation
is called forward kinematics. It also locates every link's mass centre.

The reverse calculation finds the six joint angles that put a sole at a chosen
position and angle. This is inverse kinematics. Keeping both position and
angle matters: a sole centre at the correct height can still have a toe below
the floor if the sole is tilted.

The current model's zero-angle torso-origin height above the soles is:

```text
H0 = 273.882675 mm
centred standing height = H0 - 18 mm = 255.882675 mm
single-support height = H0 - 24 mm = 249.882675 mm
```

The 18 and 24 mm reductions are chosen body-path settings. The solver checks
reach and balance for the resulting angles. During a left-to-right transfer,
the body is raised smoothly by up to 6 mm in the middle, where the legs are
more directly below it. For a transfer fraction `u` from 0 to 1:

```text
extra height = 6 mm × 16 × u² × (1-u)²
```

This is zero at either end and 6 mm at `u=0.5`.

Knee angles are calculated from the entire geometry. They are not guessed
from a straight leg length. The centred ready pose has about 52.7 degrees of
knee bend. The checked walking phases keep at least about **25.10 degrees** of
bend. The 6.88-degree (`0.12 rad`) knee minimum is an additional limit used by
online placement; it is not the intended walking knee angle.

## 5. How landing avoids the old extra extension

At landing entry, read all twelve actual servo positions. Use the supporting
foot's calculated rotation to define local axes in which its sole is flat.
This assumes that sole really is flat on the floor; no gravity sensor is used.

Calculate all bottom corners of both soles in those axes:

```text
floor_z = lowest supporting-sole corner
clearance = lowest swing-sole corner - floor_z
body height = -floor_z, when the torso origin is z=0
```

The calculation includes the current knee bend, hip angles, ankle angles and
mesh offsets. Changing a supporting knee changes the estimated body height.
There is no command to stretch the leg to its straight length.

Before descent, finish horizontal alignment while holding the entry clearance
(with a 1 mm minimum). This avoids demanding another large lift when servo
compliance has reduced the measured clearance. Then start lowering gradually
over a 1 second ramp. With a 20 ms control tick:

```text
maximum descent per tick = 1.5 mm/s × 0.020 s = 0.030 mm
actual descent = smallest of:
  maximum descent per tick after the ramp
  remaining allowed descent
  distance left to the calculated floor
```

The target height accounts for the lowest sole corner, not just the centre.
Inverse kinematics keeps the sole level and the knee bent. A target crossing
the calculated plane is rejected. Lowering has a finite distance budget based
on clearance at entry to descent. If the foot does not follow, the controller
cannot keep adding downward travel indefinitely.

When encoder geometry reaches within **0.3 mm** of the estimated plane, save
the measured relative foot position and smoothly finish at the common model
floor. Retain the loaded leg's existing target: copying its sagged encoder
angles into the next command would release supporting effort. Express the
swing-foot target in that retained support frame. Do not replay the rest of
an old straightening trajectory. The next weight transfer keeps these foot
positions fixed while moving the body; it does not pull a landed foot back to
an obsolete planned floor position.

A swing-joint error over 0.025 rad, excessive relative sole angle near the plane,
or a model clearance below -1 mm interrupts placement. The response holds the
phase and requests only 1 mm of swing-foot relief while retaining the stance
leg's target. It never treats that event as confirmed ground contact.

**This limits commanded extension. It does not directly measure touchdown or
prevent all ground pressure.** A floor height error, loose frame, calibration
error or tilted supporting foot can still make the physical foot touch before
the calculated height.

## 6. Smooth motion and joint feedback

The robot moves through: centred crouch, weight transfer, lift, swing, landing,
weight transfer to the other side, then the mirrored step. Both feet are
planted during weight transfer. The initial lift is 5 mm; swing clearance is
6 mm. These are conservative choices for a smooth, level floor.

Each planned movement uses:

```text
s(u) = 10u³ - 15u⁴ + 6u⁵
position = start + s(u) × (end - start)
```

At either end the speed and acceleration are zero. The plan contains 81 joint
samples per phase, with smooth interpolation between them. Phase time is
lengthened to keep planned speed below 0.18 rad/s and acceleration below
0.30 rad/s². Runtime checks limit the whole joint vector together so one joint
is not clipped independently and made to push a foot through the floor.

For a straight 6 mm descent, `s` has maximum slope 1.875, so the nominal
placement table needs at least:

```text
T = 1.875 × 6 mm / (1.5 mm/s) = 7.50 seconds
```

Online placement uses its own measured height and can finish sooner when the
model plane is reached. It does not wait out that time by continuing extension.

Joint feedback still checks whether servos follow commands. A 0.04 rad error
pauses path progress; 0.15 rad is a hard fault. A pause lasting 2 seconds faults.
Each phase also waits for joint errors below 0.03 rad and speeds below
0.08 rad/s for 0.4 seconds, plus its planned dwell. Stale joint states, unhealthy
bridge reports and competing command publishers still block movement.
These are servo/communication checks; there is no body-tilt gate.

Before lifting a foot, encoder-based geometry must show at least 5 mm support
margin on the remaining foot. This is a runtime check of the model, not a
measurement of true balance.

## 7. Startup speed and saved paths

Normal startup loads the checked forward/backward paths bundled with the
package. It does **not** solve the entire gait every time. A fingerprint checks
the URDF masses/dimensions/limits, sole outlines, planner source and profile.
A mismatch refuses the old table instead of silently using incorrect geometry.

After changing the model or path settings, compile new paths once:

```bash
ros2 run biped_gait compile_walk
```

For a custom parameter file, use `compile_walk --config /path/to/walk.yaml` and
pass the same file to `walk.launch.py config:=/path/to/walk.yaml`.
Compiling sends no joint commands. Restarting from a completed step can require
a new path for that changed foot spacing; this calculation runs in the
background while the node continues sending its held target.

The active settings are in [config/walk.yaml](config/walk.yaml) and
[model_walk.py](biped_gait/model_walk.py). The shared geometry helpers keep their
older default constants for `biped_cpg` and `biped_balance` compatibility; the
quasi-static executable always supplies this package's explicit new profile.
Those two packages, the robot model and bringup were not changed.

## 8. Verification

Run from the source directory, with NumPy available:

```bash
python3 -B -m unittest discover -s biped_gait/test -v
```

MuJoCo and xacro enable the physics tests; otherwise those tests are skipped.
They feed only joint positions/velocities to the walker. True simulator pose is
used only to check the result. Tests include forward/backward six-step walking
and stopping, twelve-step walking with weaker motors, delayed encoder samples,
encoder quantization,
initial standing, finite descent and preservation of landed foot positions.
They also independently check the URDF mass sum, joint limits, support margins,
sole flatness, floor height, and the 30 mm step.

With the workspace sourced, test the actual ROS launch in an isolated domain:

```bash
python3 -B biped_gait/test/run_mujoco_walk.py --landings 6 --domain 219
python3 -B biped_gait/test/run_mujoco_walk.py --direction backward --landings 6 --domain 220
```

The harness restricts discovery to localhost and verifies that the walking node
has no `/imu` subscription. Simulation motor stiffness, friction, ground
contact and calibration remain assumptions rather than measured hardware data.

Validated results for this profile (flat simulated floor):

| Run | Landings | Peak torso tilt | Body travel |
|---|---:|---:|---:|
| Default motors, forward | 6 | 2.455° | +173.9 mm |
| Default motors, backward | 6 | 2.460° | -156.3 mm |
| Weaker motors, forward | 12 | 2.602° | +353.1 mm |
| Weaker motors, backward | 12 | 2.616° | -338.1 mm |
| Delayed/quantized encoders, forward | 6 | 2.455° | +174.0 mm |

Default stiffness was 100 N·m/rad; the weaker case used 50 N·m/rad and half
available simulated torque. The delayed case included 20 ms sensor delay,
4096 positions per turn, and control intervals of 20–40 ms. All runs stopped
in standing, remained within the 0.18 rad/s command limit, and had no detected
contact between the two legs. All 32 gait tests passed. Both actual ROS launch
checks completed six landings and stopped, with zero walker IMU subscriptions.
The 45 existing CPG/balance unit checks also passed without changing those
packages.
