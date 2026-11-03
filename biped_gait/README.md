# Quasi-static walking: measured leg placement and early transfer backoff

The current node adds three changes for the reported physical failures:

- **Weight transfer:** a smaller 3.44° planned torso angle, posture feedback, and
  a brief pause for a rotation-rate spike, with early reversal if it persists
  or the measured/predicted tilt error is too large.
- **Placement:** supporting-leg encoders and IMU tilt determine the estimated
  floor relative to the bent body. The swing foot stops at that estimate instead
  of finishing a fixed extension. Its landing position is relative to the
  supporting foot, so small position errors do not widen the stance each step.
- **Startup:** default forward/backward paths are shipped as precomputed tables.
  Launch loads and checks them; it does not rebuild all walking paths.

Only `biped_gait` is changed. CPG, hardware drivers, bringup and the URDF are
unchanged. The bridge communication issue remains a separate unresolved issue.

**Physical walking is not established by the simulation tests.** There are no
contact/force sensors: computed floor height assumes the supporting sole is on
flat ground and the kinematic model/calibration are correct. IMU and encoder
feedback cannot guarantee that a foot is touching or measure its pressure.

## 1. Build and run

In your workspace, using the ROS/MuJoCo environment you already use:

```bash
cd ~/biped_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to biped_gait
source install/setup.bash
```

For MuJoCo, launch its existing simulation and this walker:

```bash
ros2 launch biped_mujoco mujoco.launch.py walk:=true
```

For hardware, start your existing robot bringup with its walker disabled
(`walk:=false`) and IMU enabled, then run:

```bash
ros2 launch biped_gait walk.launch.py
```

Do not run another gait or actuator-enabled balance node on the same command
topic. This node never enables torque or opens a servo bus. No commands are
published before a motion request. After `Ready`, allow fresh encoder/IMU data
to arrive; check `/walk/status` for `hardware_ready: true`.

Use another sourced terminal:

```bash
# Optional: lower into the bent standing stance, with posture feedback.
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stand'}"
# One step and return to centred standing:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'step'}"
# Continuous forward walking:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
# Finish the airborne step and centre:
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
ros2 topic echo /walk/status --once
```

Wait for `standing` before starting again or requesting `backward`. A quiet
one-second gyro calibration is automatic; your earlier IMU direction tests do
not need repeating. Start requires stationary joints near calibrated zero or
the last held stance. `hold` latches a hard stop; `calibrate` recalibrates the
stationary gyro bias. Neither performs fall recovery or switches torque off.

A `placement_hold` or `transfer_hold` needs physical support and a restart from
a known stance. These states retain bounded posture feedback and prohibit new
steps. A hard `fault` also requires restart. Do not repeatedly send `start` to
clear a latched condition. A simulation reset should restart the walker too.

Source layout: [feedback_walk.py](biped_gait/feedback_walk.py) handles phases;
[placement.py](biped_gait/placement.py) handles measured floor geometry and landed
anchors; [plan_store.py](biped_gait/plan_store.py) handles offline tables;
[config/walk.yaml](config/walk.yaml) contains the active settings. Legacy pure
math defaults remain in [quasi_static_walk.py](biped_gait/quasi_static_walk.py)
for CPG/balance import compatibility. They do not set this node's active lean.

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

## 6. Chosen heights and the remaining sideways motion

The URDF zero-angle body height is 273.882675 mm. The active settings give:

```text
centred body height = 273.882675 - 18 = 255.882675 mm
single-support body height = 273.882675 - 26 = 247.882675 mm
planned torso angle = 0.06 radians = 3.44 degrees
first lift = 4 mm; nominal swing clearance = 6 mm
```

These are chosen motion settings, checked against the complete joint path. Mass
alone does not determine them. Zero torso angle with the original stance/path
exceeds a hip-roll limit. A smaller angle that passes ideal geometry can also
leave too little joint margin for real tracking error. The selected profile
keeps more joint room than the attempted 2.29° profile.

The narrower sole is about 36.977978 mm wide. The CoM target is 10 mm inward
from the supporting sole centre:

```text
nominal sideways margin = 36.977978 / 2 - 10 = 8.488989 mm
```

The compiler checks at least 8 mm nominal margin. The previous profile targeted
8 mm inward and left about 10.49 mm margin. This is a 2 mm reduction in nominal
margin to reduce required travel; it is not an increase in physical stability
proved by calculation. Before lifting a foot, the measured-pose model must still
have at least 5 mm margin and the body/joints must settle.

The model still needs about 73 mm sideways pelvis movement on the first transfer.
A smaller torso angle does not remove the need to shift the whole mass. The
current stance and joint limits do not support a promise of zero sway.

## 7. How the bent knees determine landing height

A bent leg is shorter than a straight leg. For illustration, two equal 100 mm
links with a 60° knee bend have this hip-to-ankle distance:

```text
2 * 100 * cos(60 / 2) = 173.2 mm, rather than 200 mm
```

The real calculation uses all six measured joints in each leg, not that
simplified two-link example. For each link, rotate its offset using the angles
of the preceding joints, then add it to the previous position. Add the scaled
sole-bottom corners from the URDF too. This gives the four corners of each foot
relative to the body, including the current knee bend.

Rotate those corners using IMU roll/pitch so Z points vertically upward. Then:

```text
estimated floor Z = lowest supporting-foot corner Z
estimated body height = -estimated floor Z
swing clearance = lowest swing-foot corner Z - estimated floor Z
```

The body origin is Z=0 in these calculations, so a foot below it has negative Z.
The body height is recalculated from the measured supporting leg during
placement. It is not assumed to be the straight-leg height or the nominal
247.882675 mm planned height.

Placement works in these stages:

1. Align the foot above the intended landing X/Y, relative to the supporting
   sole's heading. Horizontal target changes are limited to 3 mm/s. If needed,
   use a 4 mm alignment-clearance target, with vertical changes at most 2 mm/s.
2. Once aligned, start a finite downward-travel budget from the **then-measured**
   clearance. A lift during alignment is therefore accounted for explicitly.
3. Move the lowest commanded sole corner toward the estimated plane, at most
   2 mm/s. All corners and the current foot rotation count; sole-centre height
   alone can miss a toe pressing below the plane.
4. When measured clearance is within 0.3 mm of the plane, capture the measured
   bent stance once. Do not finish an unused nominal extension or keep chasing
   changing encoder readings under gravity. A small estimated penetration is
   corrected upward to the estimated plane, never with another downward search.
5. After settling, continue the next body motion from these landed foot anchors.
   The body path is adapted smoothly while planted feet keep their positions.
   There is no joint-space blend back to an obsolete leg length.

Online placement enters its horizontal, vertical, foot-level and feedback
correction motion with a one-second smooth ramp. It does not jump straight to
the full alignment speeds after the swing hold. Capturing the measured stance
also uses a smooth transition rather than an immediate joint-target replacement.
For that base capture curve, duration is at least:

```text
sqrt(5.774 * largest joint-angle change / 0.30)
```

The 0.30 value is the nominal joint acceleration in rad/s². The duration also
respects the joint-speed limit and is at least 0.4 seconds. Posture feedback and
geometric floor corrections remain additional bounded control actions, so this
formula is not a claim that every final feedback command has that acceleration.
The capture plane and target are fixed once observed; new encoder deflection
does not start another downward extension. Placement and subsequent settling
use the same half-strength feedback, avoiding a gain change at touchdown.

The inverse leg calculation finds the hip, knee and ankle angles needed for the
sole target. The swing knee must retain at least 0.12 rad = 6.88° of bend; an
unreachable target is refused, not clipped into a straight leg. No calculated
placement target intentionally goes below the estimated floor. Exhausting the
height budget prohibits further downward searching; failure to complete within
the placement timeout interrupts the step.

This is **estimated touchdown**, not measured contact. A floor bump, an incorrect
sole offset or physical flex can cause contact before the estimate. A tilt
error over 1.5°, horizontal gyro rate over 0.12 rad/s, tracking error over
0.035 rad, estimated penetration over 1 mm, or a sole tilted over 2° within
2 mm of the plane interrupts placement. Reaching the plane before horizontal
alignment is complete also interrupts it.

On interruption, `placement_hold` captures the swing leg's measured pose and
requests only a 1 mm upward relief, within the joint speed limit. The supporting
leg keeps its load-bearing target. Posture feedback remains active; no new
step is permitted. This reduces continued position error pressing into the
floor. It cannot measure pressure or guarantee that every contact is detected.

## 8. Balance feedback and early transfer reversal

For roll and pitch, the requested small body correction is:

```text
error = measured tilt - planned tilt
correction = -0.30 * error - 0.04 * (filtered gyro rate - planned rotation rate)
```

Gyro units are radians/second. The correction is limited to ±1.5°, rate-limited
to 0.05 rad/s, and reduced during placement or near a straight knee. All leg
joints work together to retain intended sole poses. Full joint commands are
limited to 0.25 rad/s; individual joints are not clipped independently.
The planned attitude is taken from the supporting leg, including during left
foot placement; the lifted foot does not define body attitude.

The gyro rate used for damping is smoothed over 0.04 seconds. This reduces
quickly changing corrections when sensor messages arrive late or a rate reading
jumps. For a new sensor sample:

```text
a = 1 - exp(-time since previous sample / 0.04 seconds)
filtered rate = previous filtered rate + a * (new rate - previous filtered rate)
```

Here `exp(x)` means the exponential function. At a 0.02-second interval,
`a` is about 0.393: use about 39% of the new reading and 61% of the previous
filtered value. Reading the same sensor timestamp twice does not advance this
filter. The interruption checks below still use the original gyro reading;
smoothing the damping command does not hide a dangerous rate or tilt.

During a double-support transfer, look a short time ahead:

```text
error rate = measured gyro rate - planned body rotation rate
predicted tilt error = current error + 0.25 seconds * error rate
```

If the measured or predicted error exceeds 1.25°, reverse progress immediately
along that transfer's own path. A rate-only warning works differently:

1. When the horizontal rate error exceeds 0.10 rad/s, pause the transfer's
   progress. Keep both feet planted and keep posture correction active.
2. Resume once the rate error drops below 0.08 rad/s. The lower release value
   prevents repeated stop/start decisions around 0.10 rad/s.
3. If it does not settle within 0.10 seconds of new IMU samples, reverse the
   transfer. Repeated control callbacks using one old IMU sample cannot confirm
   this duration. Tilt checks remain immediate during the pause.

The 0.10-second confirmation affects only a rate-only warning. For example,
one reading of 0.101 rad/s with a predicted error of 0.99° pauses rather than
reversing the entire transfer. A continuing problem still causes recovery.

During reversal, posture feedback continues. Return to the previous stance and wait for another explicit
request, or remain in `transfer_hold` when that stance is not centred. This acts
before the 8° hard limit; it is not a guarantee of catching an established fall.

The planned rate comes from accepted body targets divided by the actual control
time interval. Intended rotation during weight transfer is therefore not counted
as disturbance growth. For example, a steady 1.10° tracking error while measured
and commanded rotation rates are both 0.02 rad/s stays a 1.10° predicted error.
The old raw-gyro prediction incorrectly increased it to 1.386°, triggering a
false backoff. No error/rate limit is raised by this correction.

Transfer backoffs print roll/pitch error, planned rotation rate, rate error,
its duration and the triggering check. Their first `interruption_snapshot` records those values and the measured
and commanded joints before returning, so a later healthy pose does not hide
the original event.

The absolute 8° tilt limit, large tracking error, stale inputs and bridge health
checks remain. A hard fault holds the last target, or suppresses output for
unhealthy/stale inputs so the bridge watchdog takes over. The small feedback
controller is not a general fall-recovery controller. No limit is raised merely
to let an unstable step continue.

The IMU filter uses each message's source timestamp for integration. Freshness
and gyro calibration use a separate receipt clock, monotonic on hardware. Valid
samples sharing one `/clock` update do not cause a false timestamp fault. Stale,
repeated and backwards source timestamps are still rejected.

## 9. Precomputed paths and startup

The default forward/backward tables live in `biped_gait/data`. Launch loads them
and checks their checksums, joint bounds, model/settings fingerprint and array
shapes. It does not run the full numerical gait solver on each startup.

The fingerprint covers masses, joint geometry/axes/limits, scaled sole corners,
active geometry/timing settings and walking-math source. If these change, the
old tables are rejected. Compile matching paths offline, with no robot commands:

```bash
ros2 run biped_gait compile_walk
# For a custom ROS parameter YAML:
ros2 run biped_gait compile_walk --config /path/to/walk.yaml
```

This is the explicit calculation step, and can take tens of seconds on slower
machines. Files go into `$XDG_CACHE_HOME/biped_gait` or `~/.cache/biped_gait`.
They are reused on the next launch. Do not change the URDF simply to bypass a
cache mismatch; regenerate the tables for the intended model.

A restart from a different held stride may need a different path. It is prepared
in a background worker with current stance feedback and command publication
continuing, then cached. `preparing_path` reports this wait. `stop` cancels the
request. Neither launch nor the background calculation automatically starts walking.

Nominal paths use 81 joint poses per segment, smooth progress
`s = 10*u^3 - 15*u^4 + 6*u^5`, and cubic interpolation. Timing respects
0.18 rad/s and 0.30 rad/s². Placement uses measured geometry rather than replaying
the nominal descent table. Between phases, posture and tracking must settle;
servo lag over 0.04 rad pauses progress and persistent lag still latches a fault.

## 10. Status, verification and limits

`/walk/status` includes the phase, planned/measured tilt, correction, worst
tracking joint, transfer-backoff count, interruption reason and placement data:
measured body height, entry clearance, remaining downward budget and alignment
error. `hardware_ready` describes current input health; it does not clear a
latched interruption/fault. `contact_measured` always remains `false`.
`interruption_snapshot` freezes the first transfer or placement interruption, including gyro
rates, measured and commanded joints, alignment/capture stage, clearance and
IMU/joint-state source-time skew. Record it before restarting; current healthy
readings do not explain a transient that has already passed.

Run offline checks without opening a servo bus:

```bash
python3 -B biped_gait/biped_gait/quasi_static_walk.py --check
python3 -B -m unittest discover -s biped_gait/test -v
```

For an actual ROS/MuJoCo launch, source the built workspace and run these
commands from the workspace directory, one at a time:

```bash
python3 -B src/biped_gait/test/run_mujoco_walk.py --landings 12
python3 -B src/biped_gait/test/run_mujoco_walk.py --landings 6 --stress --domain 210
```

They start their own headless simulation in a private ROS domain restricted to
localhost, request continuous walking, then request a centred stop. They fail on
a fault, transfer reversal, tilt over 6°, no forward movement or insufficient
landings. Launch logs and a JSON result are saved in the directory printed by
the script. No joint commands are sent to the normal robot domain.

The stress run adds 20 ms of sensor delivery delay and a 40 ms gyro-only burst
in the transfer after the first landing. It checks that the controller pauses,
resumes and completes the following steps. This is a deliberately constructed
test, not a recording of the physical robot's IMU.

Physics tests require MuJoCo and xacro. They use simulated IMU/encoders, the
current URDF, 50 Hz control, assumed stiffness 50 N·m/rad and 50% of assumed stall
torque. The controller completes three landings in each direction and returns
to standing. Peak torso tilt is about 3.92° forward and 3.82° backward. A raised
patch test interrupts placement and checks bounded relief with active feedback.
These are simulation results, not identified hardware gains or torque ratings.

All 62 gait tests pass. They include a brief rate-only burst after landing,
six landings with 20 ms sensor delay and uneven control intervals, repeated
sensor timestamps, and sustained-rate/immediate-tilt recovery checks.
A 3 N sideways pulse for 0.1 seconds during double-support
transfer triggers an early return to standing with no foot lift; peak tilt in
that test is about 2.60°. This checks one recoverable disturbance, not every fall.
Continuous-walking regressions also read the actual MuJoCo launch parameter
YAML, including its 100 N·m/rad motor stiffness and full configured torque limit,
and require six landings in each direction before stopping. This covers the
post-landing transfer that a single-step-and-centre ROS test does not exercise.

The rate-only physics test reproduces the earlier controller's reversal after
the first landing: rate error 0.101 rad/s, predicted tilt error about 1.01°.
The updated controller completes six landings with the same constructed burst.
The real-time ROS stress check also completes six landings and a centred stop,
including one 40 ms transfer pause followed by resumed walking. With 20 ms
added sensor delivery delay, its peak torso tilt is about 3.75°, forward travel
about 117 mm, and largest command gap about 32 ms on the development machine.
The earlier version failed to settle in this delayed ROS test; smoothing the
damping rate removes that failure in the tested run. This is a headless
simulation result and a constructed sensor burst, not a recording of the user's
graphical run or proof of physical walking.

The default real-time ROS/MuJoCo launch also completes twelve landings followed
by a centred stop: peak torso tilt about 3.76°, forward travel about 236 mm,
and no transfer pause or reversal. The test observer's largest wall-time gap
between received commands in that run is 334 ms, compared with 32 ms in the
delayed-sensor run. These are receipt-time measurements; they do not establish
publisher timing or hardware command deadlines.

The source description still notes that its mass/mesh articulated alignment has
not been physically validated. The walker cannot measure force, detect every
slip, identify bracket flex or prove that cached bridge encoder values came from
a fresh physical read. Hardware validation with support and recorded phase/IMU/
encoder data remains necessary. Passing these checks does not establish that the
physical robot will walk successfully.
