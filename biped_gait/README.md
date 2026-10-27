# Walking node

`quasi_static_walk` publishes the same 12-joint position commands as before.
Commands remain `start` / `forward`, `backward`, and `stop` on `/walk_cmd`.

The gait now plans foot positions relative to an upright torso, then solves leg
IK. It assumes a flat floor and does not use IMU or contact feedback.

- Start lowers smoothly from the initial zero pose into a bent-knee stance.
- The torso origin stays 265 mm above the soles during walking. Lateral transfer
  therefore changes hip and knee angles together, rather than changing leg reach
  independently for each foot.
- The whole swing sole lifts 12 mm and stays level and forward-facing. Placement
  lowers it vertically to the supporting foot's ground plane, with no forced
  straight-knee endpoint or joint-limit clipping.
- With both feet planted, their separation and heading stay fixed as the torso
  moves. Settling holds follow weight shifts and landings; these are timed holds,
  not confirmation that the robot has balanced or made contact.
- Stop completes the step and centres the torso while preserving the current
  foot spacing. It holds a bent-knee stance instead of pulling the planted feet
  together into the original all-zero pose. Restart uses that held stance.

`BODY_HEIGHT` and `SWING_CLEARANCE` are in metres. `LEAN_ANGLE`, `STEP_PITCH`, and
`LIFTOFF_SHIFT` set Cartesian distances through `LEG_REACH * sin(angle)`; actual
joint angles come from IK. `TIME_SCALE` scales motion and settling times. A slower
time scale does not repair a negative static balance margin. The copied link
masses match the current `robot.urdf.xacro`; the tests check this agreement.

From the source directory, check the gait without ROS or a robot connection:

```bash
python3 -B biped_gait/biped_gait/quasi_static_walk.py --check
python3 -B -m unittest discover -s biped_gait/test -v
```

The default rigid-model checks give at least 8.2 mm static support margin in
both directions, 12 mm swing clearance, and under 0.01 mm loaded-foot height
error, including interpolation between table rows. Double support is checked
as well as single support. Unreachable poses or failed startup checks prevent
walking. These checks do not measure actual servo tracking, body tilt, contact
forces or dynamic stability.

Rebuild `biped_gait` from the ROS workspace root and source `install/setup.bash`
before using the existing launch commands. Begin physical validation with the
robot supported against a fall: check lowering and weight shift, then one lift
and landing, before repeated steps. No physical or simulator run was performed
for this change. The older `docs/walking_math_explained.md` explains the previous
joint-keyframe gait; its knee, lean, clearance and timing examples are historical.
