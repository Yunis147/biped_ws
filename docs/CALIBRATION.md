# Calibrating the real robot

Do these in order. Each step builds on the one before it. Nothing here moves the robot with any real
torque until "7. Enable torque" — everything up to that point is either torque OFF (you move the legs by
hand) or a low, hand-held torque limit for a one-joint test.

Build and source the workspace first:
```bash
cd ~/biped_ws && colcon build --symlink-install && source install/setup.bash
```

## 0. Before you touch any Python

- Power the servo bus from a supply that can give all 12 servos enough current (a few amps peak; a USB
  port is not enough). Confirm the voltage reads correctly once servos answer (step 2).
- Plug the USB-to-serial (or Jetson UART) adapter in.
- `config/hardware.yaml` → `bus.port: auto` finds `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/ttyTHS1`
  automatically. Set it explicitly if you know the port.
- Put the robot where it can safely fall over or have a leg move unexpectedly (a stand, or lying on
  something soft) until you've done the direction check in step 4.
- Every tool below is a plain command — run it directly, not inside `servo_bridge` (stop that node first,
  the tools and the bridge must not use the bus at the same time).

## 1. Give every servo its ID

Brand-new ST3215 servos are all ID 1. They must be assigned one at a time, each connected alone to the
bus.

```bash
ros2 run biped_hardware biped_set_id --guided
```

It prompts you through all 12 joints, in the order listed in `config/servos.yaml` (built from the ID
table in AsterisCrack/BipedRobotJetson — see below). Connect only the one servo it asks for, press Enter,
and it sets that servo's ID and verifies it took.

| joint | id | | joint | id |
|---|---|---|---|---|
| l_hip_yaw | 13 | | r_hip_yaw | 7 |
| l_hip_roll | 2 | | r_hip_roll | 8 |
| l_hip_pitch | 3 | | r_hip_pitch | 9 |
| l_knee | 4 | | r_knee | 10 |
| l_ankle_roll | 5 | | r_ankle_roll | 11 |
| l_ankle_pitch | 6 | | r_ankle_pitch | 12 |

If you already assigned IDs another way, skip this and just make sure they match this table (or edit
`config/servos.yaml` if you'd rather keep different IDs).

## 2. Scan the bus

With everything reconnected:
```bash
ros2 run biped_hardware biped_scan
```
Expect `OK: all 12 expected servos answer and nothing else is on the bus.` If something is missing, check
power and the connector at that joint before continuing. If NOTHING answers, try
`ros2 run biped_hardware biped_scan --try-bauds` in case a servo shipped at a different baud rate.

## 3. Set the zero pose (mid-points)

This tells each servo "whatever angle the leg is at right now is 0 rad" — the pose the URDF and the
walking gait call neutral/standing.

```bash
ros2 run biped_hardware biped_calibrate zero
```

Torque is OFF, so you move the robot by hand. The tool prints exactly what the zero pose looks like:

- both legs straight, both feet flat and pointing forward, parallel
- thighs vertical, hips level, torso upright and not twisted
- left/right symmetric (no lean)

It reads all 12 servos, shows you the table, and asks you to confirm before saving. It defaults to
`--method servo`: it stores the mid-point inside each servo's own EEPROM (register-level `TORQUE_ENABLE =
128`, the standard Feetech "set here as centre" command), so the servo itself reports 2048 at this pose
afterwards. If that command doesn't take for some reason, it falls back to a software offset in
`calibration.yaml` instead and tells you.

One joint at a time, nudge it with `ros2 run biped_hardware biped_jog` and type `set0` if you want to
fine-tune a single joint's zero by eye afterwards, without redoing the whole-body pose.

## 4. Check every joint's direction

This is the step that matters most for this gait — it's what the whole document
`walking_math_explained.md` depends on (hip-roll signs, knee sign, ankle math).

```bash
ros2 run biped_hardware biped_calibrate directions
```

**The robot must be supported with the legs free to move** (on a stand, or lying on its back) — not
standing on its own. It turns torque on at a low limit (300/1000 by default) and holds the current pose.
For each joint it moves it 10° in the direction the URDF calls positive and tells you what to expect, for
example:

> `r_hip_pitch_joint`: EXPECTED: the leg swings backward (foot moves behind the robot)

You answer yes/no. If no, it tries the opposite sign; if that also doesn't match, it leaves that joint
alone and tells you something else is wrong (wrong ID on that connector, or the servo horn mounted on a
different joint than expected). Any direction it had to flip goes into `calibration.yaml`.

What "positive" means for each joint is explained with the reasoning in
`walking_math_explained.md`, section 4.3 — this tool is just that logic turned into a hands-on test.

## 5. (Optional) Record real joint ranges

The defaults in `servos.yaml` are the URDF's limits. If your real robot's wiring, horns or 3D-printed
parts can't quite reach them (or can safely go further before anything binds), record the real range:

```bash
ros2 run biped_hardware biped_calibrate ranges
```

Torque OFF, move each joint by hand to each end before anything touches or binds, press Enter at each end.
It only ever narrows the URDF limits, never widens them.

Check the recorded ranges against what the gait actually needs (from
`walking_math_explained.md`): hip roll must reach about ±0.40 rad, ankle pitch about −0.59 rad. If a
range comes out narrower than that, the gait's margin will be smaller than modelled — rather not walk
until that's sorted.

## 6. Check the calibration

```bash
ros2 run biped_hardware biped_calibrate show
```

Prints the direction, zero_steps and limits currently in effect (file defaults + anything saved in
`calibration.yaml`).

## 7. Enable torque (the first real test)

Support the robot (hands or a stand — not standing freely yet) and start the bridge:

```bash
ros2 launch biped_hardware hardware.launch.py
```

Watch `/diagnostics` for a few seconds (temperature, voltage, any ERROR/WARN). Then:

```bash
ros2 service call /servo_bridge/torque std_srvs/srv/SetBool "{data: true}"
```

It holds exactly the pose it's already in (no jump), at `safety.torque_limit` (500/1000 by default). Check
each leg resists a gentle push without the servos straining or overheating, then try a small manual test
move:
```bash
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0,0,0,0,0,0, 0,0,0,0,0,0]}"     # neutral - it should already be there
```
then a small lean (see the walking document for the numbers) before trusting it with the full gait.

## 8. Run the walking gait

```bash
ros2 run biped_gait quasi_static_walk
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'start'}"
ros2 topic pub --once /walk_cmd std_msgs/msg/String "{data: 'stop'}"
```

Same node, same topic, same gait as in Gazebo. If it doesn't balance as well as simulation, re-check step
4 first (a flipped sign anywhere shows up as a fall) and step 5 (a joint hitting a real mechanical limit
sooner than the URDF says).

## Doing it all without hardware first (dry run)

Every tool and launch file above has a mock mode that behaves like a real bus at the packet level (servos
answer pings, hold position, obey speed limits, etc.), so you can rehearse the whole sequence with no
servos connected:

```bash
ros2 run biped_hardware biped_scan --mock
ros2 launch biped_hardware hardware.launch.py mock:=true rviz:=true
```

## If something won't move / answer

```bash
ros2 run biped_hardware biped_scan
```
tells you which servo IDs are missing. Check, in order: power to that servo, the data connector, that
`biped_set_id --guided` actually completed for it (step 1), and that nothing else is drawing the bus at
the same time (another script, a second `biped_scan`, `servo_bridge` still running).

## Where these commands came from

`servo_array.py` / `biped_hardware/st3215/` implement the standard Feetech/SCS packet protocol
(ping/read/write/sync-read/sync-write) and the mid-point and ID-change commands used above; the register
map and the serial transport are adapted from **AsterisCrack/BipedRobotJetson** (MIT licence — see
`biped_hardware/NOTICE.md`). The servo IDs and direction signs in `config/servos.yaml` are the starting
point from that same repo's `config/robot.yaml`; `biped_calibrate directions` is how you confirm (or
correct) them on your own robot rather than trusting them blindly.
