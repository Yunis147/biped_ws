# Simple quasi-static walking

The executable now plays precomputed joint positions in a **50 Hz timed loop**.
It has no subscriptions to `/imu`, `/joint_states` or `/diagnostics`. It does
not pause for servo tracking, bridge health, sensor timestamps, measured
settling, contact guesses or a landing-height budget. The old feedback and
online placement modules have been removed.

Default commands are a **3 cm step** and **1 cm foot lift** for either leg in
both directions. The planned torso roll remains **2.29 degrees**. The robot
still shifts its pelvis sideways to put its mass over the supporting foot.
Only `biped_gait` is changed; bringup, hardware, CPG and balance packages are
unchanged.

## Run

From the workspace root:

```bash
colcon build --packages-select biped_gait --symlink-install
source install/setup.bash
```

Use the existing simulator or hardware launch:

```bash
ros2 launch biped_mujoco mujoco.launch.py walk:=true
```

```bash
ros2 launch biped_bringup hardware.launch.py walk:=true imu:=false
```

`walk:=true` launches the node. It still waits for a motion command:

```bash
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
```

| Command | Action |
|---|---|
| `start` / `forward` | Repeat forward steps |
| `backward` | Repeat backward steps |
| `step` | Take one right-foot step, then centre the body |
| `stand` | Move from the startup pose to the bent standing pose |
| `stop` | Finish any airborne step, then centre the body |
| `hold` | Pause where the command is now; continue publishing that position |
| `start` after `hold` | Resume the same phase |
| `status` | Publish the current phase and settings |

Read status with:

```bash
ros2 topic echo --once /walk/status
```

To run the walker separately, start bringup with `walk:=false`, then use
`ros2 launch biped_gait walk.launch.py`. For a separate simulator node, add
`sim:=true`. Start from the calibrated startup pose: this loop does not read
actual joint positions to find out where the robot is.

## What the loop does

The sequence is:

```text
bend into standing
shift weight left
lift right foot 10 mm
move right foot forward 30 mm
lower right foot to the model floor
shift weight right
lift left foot 10 mm
move left foot ahead of the right foot by 30 mm
lower left foot to the model floor
repeat
```

The controller reads one interpolated row of twelve joint angles every 20 ms
and publishes it to `/position_controller/commands`. Each movement runs for
its planned duration. A short fixed rest follows each phase, then the next
phase starts. During these rests it keeps publishing the last row. There is
no test of measured settling and no automatic fault state.

`landings` in status counts completed **commanded** landing phases. It is not
a measurement that a physical foot has contacted the floor.

Normal startup loads the saved tables rather than solving the whole gait.
After stopping in a different stance, restarting may need a new entry path.
That is computed in the background while the held pose continues publishing.

## The model used to calculate the positions

The source of dimensions and masses is
`robotv2_description/urdf/robot.urdf.xacro`. Bottom contact outlines come from
the scaled foot meshes. The total model mass is:

```text
base       = 0.3251 kg
right leg  = 0.4832 kg
left leg   = 0.4832 kg
total      = 1.2915 kg
weight     = 1.2915 × 9.81 = about 12.67 N
```

Each leg's six link masses are 0.0865, 0.0222, 0.1571, 0.0921, 0.0228 and
0.1025 kg. All link centres of mass are included, including the lifted leg.

For each pose, calculate the location of each link by adding its joint offset
and applying its joint rotation. Repeating this from the torso through all
six leg joints gives the foot position and angle. This is **forward
kinematics**. Finding the joint angles needed for a chosen foot position and
angle is **inverse kinematics**.

## Balance and lean

A mass centre is a weighted average. For two masses, 1 kg at 0 cm and 2 kg at
3 cm, it is:

```text
centre = (1 × 0 + 2 × 3) / (1 + 2) = 2 cm
```

The robot uses the same calculation for all links in three directions:

```text
centre of mass = sum(link mass × link mass-centre position) / total mass
```

For slow walking, a vertical line down from this centre should lie inside the
supporting sole. With both feet planted the available area includes the space
between them; with one foot lifted only the other sole counts.

The soles are about 90 mm long and 37 mm wide. The desired mass centre is
10 mm inward from the loaded sole centre. On the narrower sole:

```text
half width = 36.97798 / 2 = 18.48899 mm
inward-edge margin = 18.48899 - 10 = 8.48899 mm
```

The complete paths are checked offline against an 8 mm minimum margin. The
fore/aft mass-centre target follows the midpoint between the feet. A 30 mm
step places this target only 15 mm from the loaded sole centre, within its
45 mm half length.

The pelvis moves sideways. Bending a knee can help move the pelvis, but simply
lowering the torso does not move its mass over one foot. A small torso roll
helps fit this weight transfer inside the leg's joint ranges:

```text
roll = 0.040 radians × 180 / pi = 2.2918 degrees
```

The first lifted right foot also moves inward by about 19.346 mm, changing
foot-centre spacing from 109.346 mm to 90 mm. Later steps keep that spacing.
The feet are never pulled sideways while planted.

## Bent knees and the larger lift

The zero-angle model height of the torso origin above the soles is
273.882675 mm. Body height is planned as:

```text
centred standing = 273.882675 - 18 = 255.882675 mm
single support   = 273.882675 - 21 = 252.882675 mm
```

The body is up to 3 mm higher in the middle of a transfer. For a transfer
fraction `u` from 0 to 1, the added height is:

```text
3 mm × 16 × u² × (1-u)²
```

It is zero at each end and 3 mm at the middle. The previous 24 mm crouch put
the ankle outside its joint range when combined with a 10 mm lift. Using
21 mm keeps this larger lift reachable without increasing the torso roll.
These heights are selected model settings, not unique answers forced by
physics. The full path is checked after selecting them.

The ready stance has about 52.7 degrees of knee bend. The smallest checked
walking knee bend is about 14.62 degrees, during the initial weight transfer;
the legs remain bent. The solver uses all six joints, so it does not estimate
height from knee angle alone or command the straight leg length for landing.

## Landing and the floor

The lift phase raises the sole by 10 mm relative to the supporting sole. Swing
keeps that height. Landing lowers it vertically to **zero clearance** in the
model's ground frame. All bottom corners are considered and the soles remain
flat. The landing endpoint has no extra downward extension or search motion.

To convert a world foot target to a target relative to the torso:

```text
relative position = world foot position - torso position
body-frame target = inverse torso rotation × relative position
```

Inverse kinematics then finds the joint angles that put the sole there while
keeping it level. Torso height, torso roll, hip offsets, ankle offsets, mesh
thickness and knee bend are all included in this calculation.

Landing now plays these calculated rows directly. It never waits for an
encoder-based ground estimate, runs out a measured-height budget, or switches
to `placement_hold`. It does not finish the landing by guessing an additional
leg extension.

The floor is an assumption of this open-loop method. **Commanded lift is
10 mm; actual lift depends on servo tracking and frame flex.** There is no
feedback correction for an uneven floor, slip or body tilt. Likewise, zero
model clearance cannot guarantee zero extra physical ground pressure when the
real calibration or geometry differs from the model.

## Timing

Movement uses a smooth fraction:

```text
s(u) = 10u³ - 15u⁴ + 6u⁵
position = start + s(u) × (end - start)
```

Speed and acceleration are zero at the ends. There are 81 calculated rows per
phase, with smooth interpolation between them. Durations keep nominal joint
speed below 0.18 rad/s and acceleration below 0.30 rad/s². These are offline
trajectory calculations, not live tracking gates.

For landing, the maximum slope of `s` is 1.875. Lowering 10 mm at a maximum
planned 1.5 mm/s therefore takes at least:

```text
T = 1.875 × 10 / 1.5 = 12.5 seconds
```

After those 12.5 seconds the landing phase finishes and the loop moves on
after its fixed rest. It keeps sending targets throughout.

## Settings, cached paths and tests

[config/walk.yaml](config/walk.yaml) contains the eleven active settings.
[model_walk.py](biped_gait/model_walk.py) holds the path settings and simple
phase player. [walking_node.py](biped_gait/walking_node.py) publishes the rows.
The shared geometry helper retains older defaults for CPG/balance callers;
the walking executable always supplies its explicit current profile.

After changing the URDF or path settings, compile matching tables once:

```bash
ros2 run biped_gait compile_walk
```

For custom settings, compile with `--config /path/to/walk.yaml` and launch
with `config:=/path/to/walk.yaml`. Compiling only generates tables; it sends
no commands. Existing tables are checked against the current model and profile.

Run the unit and optional MuJoCo physics tests from the source directory:

```bash
python3 -B -m unittest discover -s biped_gait/test -v
```

Physics tests require MuJoCo and xacro. They use the real simulated joint and
body positions to check outcomes, without feeding any measurements to the
walking controller. They check forward/backward walking, stopping, longer
runs with weaker motors, actual foot clearance and uneven control intervals.
Other tests independently verify masses, foot geometry, the 10 mm planned lift,
30 mm step, bent knees, floor contact, phase continuity and joint ranges.

To test the actual ROS launch in private localhost-only domains:

```bash
python3 -B biped_gait/test/run_mujoco_walk.py --landings 6 --domain 223
python3 -B biped_gait/test/run_mujoco_walk.py --direction backward --landings 6 --domain 224
```

The harness verifies that the walker subscribes to none of the three feedback
topics. Simulation results do not establish physical hardware walking.

Validated results for this version: all **25 tests passed**. Both actual ROS
launches completed six steps and stopped, with zero feedback subscriptions.
The twelve-step weaker-motor tests also completed and stopped without falling.
Peak torso tilt was about 2.46° with default simulated motors and 2.61° with
weaker motors. The lowest sole corners reached about 8.0 mm actual clearance
with default motors and 6.4 mm with weaker motors, despite the exact 10 mm
planned lift; this illustrates the servo-tracking limitation of open-loop
walking. These are simulated measurements, not physical robot test results.
