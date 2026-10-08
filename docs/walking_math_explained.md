# How the biped's walking numbers were calculated — from scratch

**Robot:** AsterisCrack BipedRobot V2 · **Gait:** open-loop quasi-static walking (no RL)
**Constants explained here:** `LEAN_ANGLE 0.40` · `KNEE_BEND −0.80` · `STEP_PITCH 0.12` · `LIFTOFF_SHIFT 0.06` · segment times 1.5–2.5 s · the ankle angles

> **Note added after this document was written:** `base_link`'s mass and centre of mass were then updated,
> in both `robotv2_description` and `biped_gait`, to the values from the AsterisCrack/BipedRobotJetson
> hardware repo (mass 0.622478 kg, CoM (3.01, −2.16, 31.32) mm, vs. 0.633299 kg / (2.43, −1.74, 38.18) mm
> used throughout the worked examples below). The change is about 1.6% of body mass and moves the overall
> robot CoM well under 1 mm — re-running `check_gait()` gives 9.7 mm instead of 10.0 mm worst-case margin,
> same conclusion. The worked numbers below were not redone; use them for *how the calculation works*, not
> as the exact current constants (those are in `biped_gait/biped_gait/quasi_static_walk.py`, which runs
> `--check` against whatever `BASE` currently contains).

You do not need to know any formula before reading this. Every formula is introduced, explained in plain words, and (where it matters) derived. Every number in a "worked example" was computed from your robot's URDF file, not made up.

---

## 0. How to read this

- **Part 1** — what problem we are solving, in one page.
- **Part 2** — the maths toolkit, from zero (angles → rotation → centre of mass → tipping → solving equations).
- **Part 3** — the robot's own data (lengths, masses).
- **Part 4** — the actual calculations, one constant at a time.
- **Part 5** — a table saying honestly which numbers were *calculated*, which were *judged*, and which were *tested*.
- **Part 6** — what this method cannot guarantee.
- **Part 7** — glossary.

Symbols you will see: `×` multiply, `·` multiply, `Σ` "add up all of these", `θ α β γ` angles, `ℓ` a length, `m` mass, `g = 9.81 m/s²` (gravity).

---

## 1. The problem in one page

The robot has 12 motors. A **pose** is just 12 angles — one per motor. Walking, in this project, is:

1. Choose a short list of key poses (lean left → lift right foot → swing it forward → put it down → lean right → …).
2. Check that **in every pose the robot would not tip over**.
3. Move from pose to pose **slowly**, so that motion itself adds no extra tipping force.

So the maths has to answer four questions:

| # | Question | Maths needed |
|---|---|---|
| Q1 | For a given pose, **where is the robot's centre of mass (CoM)?** | angles → positions (forward kinematics), then a weighted average |
| Q2 | **Is the CoM over the foot** (so it does not tip)? | support polygon + distance to an edge |
| Q3 | **How slow is slow enough?** | the ZMP formula |
| Q4 | **How do I keep the feet flat** when the legs are bent and tilted? | solving equations numerically (Newton's method) |

Everything below is built to answer these four.

---

## 2. The maths toolkit, from zero

### 2.1 Angles: degrees and radians

A full turn is 360°. Robotics software measures angles in **radians**, where a full turn is 2π radians.

```
2π radians = 360°          →   1 radian = 360° / 2π = 57.2958°
degrees = radians × 57.2958          radians = degrees / 57.2958
```

**Why radians?** Take a circle of radius 1. If a point moves a distance θ along the edge, it has turned by θ radians. So *angle = arc length ÷ radius*. This makes the maths of small movements very simple.

Angles in this project:

| radians | degrees | what it is |
|---|---|---|
| 0.12 | 6.9° | step pitch (forward) |
| 0.40 | 22.9° | the lean |
| 0.436 | 25.0° | the hip-roll **limit** (URDF) |
| 0.611 | 35.0° | the ankle-pitch limit (URDF, negative side) |

### 2.2 Sine and cosine (right triangles)

Take a stick of length ℓ hanging straight down, then tilt it by an angle θ away from vertical. It makes a right triangle:

```
        hip (pivot)
         *
         |\
         | \  ℓ  (the stick)
  height |  \
 ℓ·cos θ |   \
         |    \
         |_____* foot
        sideways = ℓ·sin θ
```

```
sideways distance = ℓ × sin θ
height below pivot = ℓ × cos θ
```

That is all sine and cosine mean here. **Worked example** — the whole leg from the hip-roll joint to the ankle-roll joint is ℓ = 178.136 mm long (Part 3). Tilt it by 0.40 rad:

```
sin 0.40 = 0.38942   cos 0.40 = 0.92106
sideways = 178.136 × 0.38942 = 69.4 mm
height   = 178.136 × 0.92106 = 164.1 mm   (so the leg "gets shorter" by 14.0 mm)
```

**Small-angle shortcut (for intuition only).** When θ is small (in radians), `sin θ ≈ θ`. Why: the stick's tip travels along a tiny arc of length ℓθ, and for a tiny arc the arc and the straight sideways distance are almost the same. How good is it?

| θ (rad) | sin θ | error of "sin θ ≈ θ" |
|---|---|---|
| 0.05 | 0.04998 | 0.04 % |
| 0.10 | 0.09983 | 0.17 % |
| 0.20 | 0.19867 | 0.67 % |
| 0.40 | 0.38942 | 2.72 % |

The robot script never uses this shortcut — it uses exact sine and cosine — but it explains why "a tilt of θ moves a point ℓ·θ sideways".

**Inverse (arcsin).** If you know `sin θ = 0.168`, the angle is `θ = arcsin(0.168) = 0.169 rad`. That is how the very first version turned "30 mm out of 178 mm" into a pitch angle.

### 2.3 Coordinates: which way is x, y, z?

We describe every position as three numbers `(x, y, z)` in millimetres, measured from the middle of the robot's torso (`base_link`):

```
x = forward      y = left      z = up
```

So the left foot is at positive y, the right foot at negative y, and the floor is at about z = −284.5 mm (the feet are 284.5 mm below the origin of the torso).

### 2.4 Rotating a point (this is where all the "angle → position" maths comes from)

Take a point at distance r from the origin, at angle φ:

```
x = r·cos φ        y = r·sin φ
```

Rotate it by an extra angle θ. It is still at distance r, now at angle φ + θ:

```
x' = r·cos(φ+θ)        y' = r·sin(φ+θ)
```

Geometry of triangles gives the **angle-addition formulas**:

```
cos(φ+θ) = cos φ·cos θ − sin φ·sin θ
sin(φ+θ) = sin φ·cos θ + cos φ·sin θ
```

Substitute them, and notice `r·cos φ = x` and `r·sin φ = y`:

```
x' = x·cos θ − y·sin θ
y' = x·sin θ + y·cos θ                 ← the rotation formula
```

That single pair of formulas is the foundation of everything in forward kinematics.

**Worked example — why a positive hip-pitch swings the foot *backward*.** The hip-pitch joint rotates about the **y axis**. The relevant plane is (x, z), and for rotation about y the formula is

```
x' =  x·cos θ + z·sin θ
z' = −x·sin θ + z·cos θ
```

A foot hanging straight below the hip at (x, z) = (0, −178.1 mm). Rotate by θ = +0.12 rad:

```
x' = 0·cos 0.12 + (−178.1)·sin 0.12 = −178.1 × 0.11971 = −21.3 mm
z' = −0·sin 0.12 + (−178.1)·cos 0.12 = −178.1 × 0.99281 = −176.8 mm
```

`x'` is **negative** = the foot moved **backward**. So with this robot's joint axis, *a positive hip-pitch value moves the foot backward; walking forward needs negative values.* This is exactly why, in the first test, `STEP_PITCH = +0.169` made the robot walk backward.

### 2.5 Rotation matrices (just a tidy way to write the formulas)

A **matrix** is a table of numbers. "Matrix × point" means: each row of the table gives one coordinate of the answer. The three basic 3D rotations (about the x, y and z axes) are:

```
        ⎡1    0       0   ⎤             ⎡ cos θ   0   sin θ⎤            ⎡cos θ  −sin θ   0⎤
Rx(θ) = ⎢0  cos θ  −sin θ ⎥     Ry(θ) = ⎢   0     1     0  ⎥    Rz(θ) = ⎢sin θ   cos θ   0⎥
        ⎣0  sin θ   cos θ ⎦             ⎣−sin θ   0   cos θ⎦            ⎣  0       0     1⎦
```

Look at `Rx`: row 2 says `y' = y·cos θ − z·sin θ`, row 3 says `z' = y·sin θ + z·cos θ` — the same rotation formula from 2.4, applied in the (y, z) plane.

**Negative axes.** Some URDF joints have axis `(0, −1, 0)` (the knee) or `(−1, 0, 0)` (right hip roll). Rotating by θ about −y is the same as rotating by −θ about +y. That is all a "reversed axis" means.

**Chaining.** To apply rotation A and then B, you multiply the matrices: `B × A`. Order matters: rotating about x then y is generally *not* the same as y then x. (This matters in 4.6.)

### 2.6 Forward kinematics: from joint angles to positions

The robot is a **chain**: torso → hip yaw → hip roll → hip pitch → knee → ankle roll → ankle pitch → foot. For each joint the URDF gives:

- an **offset** `(x, y, z)` from the previous link (a fixed length, set by how the robot was built), and
- an **axis** the joint rotates about.

Walking down the chain, keep two things: the running position `p` and the running rotation `R` (how the current link is turned relative to the torso). For each joint:

```
p ← p + R × offset          (move along the next bone, using the current orientation)
R ← R × Rot(axis, joint angle)     (then apply this joint's own rotation)
```

After the last joint you know where the foot is. A fully worked example is in 4.4.

### 2.7 Centre of mass (CoM)

**Idea.** The CoM is the single point where the whole robot "balances" — where you could hold it on one fingertip.

**Derivation (the seesaw).** Put mass m₁ at position x₁ and mass m₂ at x₂ on a plank. Support it at point p. Each mass pulls down with force m·g, and a force at distance d from the support makes a turning effect (torque) of force × d. Balance means the two turning effects cancel:

```
m₁·g·(p − x₁) = m₂·g·(x₂ − p)
```

`g` cancels. Solve for p:

```
m₁·p − m₁·x₁ = m₂·x₂ − m₂·p
(m₁ + m₂)·p = m₁·x₁ + m₂·x₂
p = (m₁·x₁ + m₂·x₂) / (m₁ + m₂)
```

With many masses it is the same idea — a **weighted average of positions, weighted by mass**:

```
CoM = Σ (mᵢ × rᵢ)  /  Σ mᵢ
```

done separately for x, y and z. (`rᵢ` is the position of the i-th link's own centre of mass; the URDF gives it for every link.)

### 2.8 Why "CoM over the foot" means "does not tip"

Gravity pulls down at the CoM. The ground can only push up where the foot touches it. Pretend the robot starts to tip about the edge of the foot. Gravity's turning effect about that edge is `M·g × (horizontal distance from the CoM to the edge)`:

- CoM is **over the foot** (inside the edge) → gravity presses the robot onto the foot → the ground pushes back → **stable**.
- CoM is **beyond the edge** → gravity's torque rotates the robot over the edge → **tips**.

**Support polygon.** The contact patch(es) with the ground. With both feet down it is the shape you get by stretching a rubber band around both soles (the **convex hull**). With one foot down it is just that foot.

**Margin.** To make this a number: project the CoM straight down onto the floor (just use its x and y). The **margin** is the distance from that point to the nearest edge of the support polygon: positive = inside (safe), negative = outside (tips).

**Distance from a point to an edge (derivation).** Take an edge from corner `a` to corner `b`. Its direction is `e = b − a`. A vector perpendicular to `e` (turned 90°) is `(−e_y, e_x)`; dividing by its length `|e|` gives a **unit normal** `n` that points to the inside of the polygon. The **dot product** of two vectors, `u·v = u_x·v_x + u_y·v_y`, measures how much of `u` points along `v`. So the signed distance of the point `p` from the edge is

```
distance = (p − a) · n          where n = (−e_y, e_x) / |e|
```

The margin is the smallest such distance over all edges.

**Worked example — standing still on both feet** (all joints at 0). The soles are rectangles:

```
left foot : x from −29.8 to +60.2 mm,  y from +34.4 to +71.6 mm
right foot: x from −29.8 to +60.2 mm,  y from −71.6 to −34.4 mm
```

The rubber band around both is the rectangle x ∈ [−29.8, 60.2], y ∈ [−71.6, 71.6]. The CoM (computed in 4.1) is at (x, y) = (1.7, −0.7) mm. Distances to the four edges: back 31.5, front 58.5, sides ≈ 71 → **margin = 31.5 mm** (safe).

Now pretend only the **left** foot is on the ground. Its nearest edge to the CoM is the inner edge at y = 34.4 mm, and the CoM is at y = −0.7 mm: **margin = −35.1 mm** (the CoM is 35 mm *outside* the foot — it would tip). So to lift the right foot, the CoM must first be moved 35 mm toward the left foot. *This single number is the reason the robot has to lean.*

### 2.9 Newton's method (how to solve an equation by repeated guessing)

Sometimes we need an unknown angle `x` such that `f(x) = 0`, and there is no neat formula. **Newton's method:** guess `x`, then draw the tangent line to the curve at that point and see where the tangent crosses zero. That crossing is a better guess.

The tangent at `x` has slope `f′(x)`. It reaches zero after moving a distance `f(x) / f′(x)`, so

```
x_new = x − f(x) / f′(x)
```

**Worked example** — solve `cos x = x`, i.e. `f(x) = cos x − x = 0`, with `f′(x) = −sin x − 1`. Start at x = 1:

| step | x | f(x) | f′(x) |
|---|---|---|---|
| 0 | 1.000000 | −0.459698 | −1.841471 |
| 1 | 0.750364 | −0.018923 | −1.681905 |
| 2 | 0.739113 | −0.000046 | −1.673633 |
| 3 | 0.739085 | 0.000000 | — |

The answer 0.739085 appeared in three steps. Notice how the error shrinks super-fast.

**Two unknowns.** The ankle solver (4.6) has two unknowns (ankle roll and ankle pitch) and two equations (the foot must not tilt in either direction). Then `f` is a pair of numbers and `f′` becomes a 2×2 table of slopes called the **Jacobian** `J` (how much each equation changes when each unknown changes). One step solves the small system `J · Δ = −f` for the correction `Δ`, and then `x_new = x + Δ`. Same idea.

---

## 3. The robot's own data (from the URDF file)

The URDF is the robot's blueprint. The numbers below are copied from it (left leg shown; the right leg is the mirror image, with the sign of y flipped).

**Joint offsets** (each is measured from the *previous* link, in millimetres):

| Joint | offset x | y | z | axis |
|---|---|---|---|---|
| hip yaw (from torso) | −1.59 | +21.672 | −0.15 | z (r: −z) |
| hip roll | 19.1 | 30.829 | −40.776 | x (r: **−x**) |
| hip pitch | −18.625 | −20.625 | −33.9 | y |
| knee | 0 | 2.0 | −72.052 | **−y** |
| ankle roll | 18.625 | 19.101 | −72.184 | x (r: −x) |
| ankle pitch | −18.625 | −20.625 | −33.9 | y |

**Lengths we use again and again:**

```
thigh (hip pitch → knee)                      ℓ₁ = 72.052 mm
calf  (knee → ankle roll)                        = 72.184 mm
ankle roll → ankle pitch                         = 33.9 mm
ankle pitch → sole of the foot                   = 31.57 mm

hip roll → ankle roll (vertical)           ℓ  = 33.9 + 72.052 + 72.184 = 178.136 mm
hip pitch → ankle pitch (vertical)         ℓ  = 72.052 + 72.184 + 33.9 = 178.136 mm
hip pitch → sole (vertical)                    = 209.706 mm
```

**Joint limits (from the URDF)** that decide what values we are *allowed* to command:

| joint | lower | upper |
|---|---|---|
| hip roll (both legs) | −0.436 | +1.571 |
| ankle pitch | **−0.611** | +1.484 |
| knee, hip pitch | ±2.09 / ±1.57 | |

**Masses (13 links, total 1.661 kg).** The torso is 0.633 kg; each leg is about 0.514 kg. Roughly 62 % of the mass is "below the torso" in the legs.

---

## 4. The calculations, one at a time

### 4.1 The centre of mass of the whole robot (Q1)

Apply the weighted-average formula from 2.7. Positions are measured at the standing pose (all joints = 0). The y positions of left and right links are equal and opposite, so they cancel in the sum — only the torso contributes to Σ m·y. Left and right links are combined as pairs ("×2").

| link (left+right) | mass (kg) | x (mm) | z (mm) | m·x | m·z |
|---|---|---|---|---|---|
| torso (`base_link`) | 0.633299 | 2.43 | 38.18 | 1.537 | 24.179 |
| hip upper ×2 | 0.201298 | −1.22 | −25.22 | −0.244 | −5.078 |
| hip lower ×2 | 0.042678 | −0.72 | −57.88 | −0.030 | −2.470 |
| thigh ×2 | 0.337394 | −1.11 | −110.85 | −0.374 | −37.400 |
| calf ×2 | 0.210060 | −1.17 | −199.40 | −0.246 | −41.886 |
| ankle ×2 | 0.042678 | −0.72 | −236.01 | −0.030 | −10.072 |
| foot ×2 | 0.193590 | 11.05 | −262.91 | 2.140 | −50.898 |
| **total** | **1.660997** | | | **2.753** | **−123.625** |

(the torso's y: −1.74 mm, so Σ m·y = 0.633299 × (−1.74) = −1.103)

```
CoM x = 2.753   / 1.660997 =   1.66 mm
CoM y = −1.103  / 1.660997 =  −0.66 mm
CoM z = −123.625/ 1.660997 = −74.43 mm
```

The floor is at z = −284.5 mm, so **the CoM is 210.1 mm above the floor**, essentially in the middle, 0.7 mm to the right of centre. That is what 2.8 used.

### 4.2 Why the robot must lean (the 35 mm problem)

From 2.8: with only the left foot down, the CoM is **35.1 mm outside** the foot. The left foot's inner edge is at y = +34.4 mm and the CoM is at y ≈ 0. The lean has to move the CoM by 35 mm *plus a safety margin* toward the stance foot.

### 4.3 The sign rules (derived, not guessed)

These cost us a lot of falls the first time. All three follow from the rotation formulas.

**(a) Hip roll — "lean left = right +L, left −L".** The URDF gives the right hip-roll axis as `−x` and the left as `+x`. Rotating by θ about `−x` equals rotating by `−θ` about `+x`. So

```
right roll = +L  →  rotates by −L about +x
left  roll = −L  →  rotates by −L about +x
```

Both legs rotate by the **same** angle about the same axis — they tilt together like a parallelogram. If you commanded the same sign on both (`+L, +L`) the legs would rotate in opposite directions and splay into a V. That was the "splits" bug in the first script.

**(b) Hip pitch — forward is negative.** Shown in 2.4: a positive pitch (rotation about +y) moves a hanging foot backward by `x = −ℓ·sin α`. So the script defines `F = −STEP` as the value that sends a leg forward.

**(c) Knee — negative angle is the normal bend.** The knee axis is `−y`, so a knee value β rotates the shin by `−β` about +y. A negative β = a positive rotation about +y = the same direction as a positive hip pitch = the shin swings **backward** (the normal way a knee bends). A positive knee value would bend it the wrong way.

### 4.4 Forward kinematics, one leg, with real numbers

Pose: **lean left**, left leg. The left hip roll is −0.40 rad, the ankle roll the solver gave is +0.40 rad, everything else 0. Follow the algorithm of 2.6 (`p ← p + R·offset`, then `R ← R·Rot`):

```
cos(−0.40) = 0.92106    sin(−0.40) = −0.38942
```

| joint | offset in its parent's frame (mm) | offset after applying R (mm) | position in the torso frame (mm) |
|---|---|---|---|
| hip yaw | (−1.6, 21.7, −0.2) | (−1.6, 21.7, −0.2) | (−1.6, 21.7, −0.2) |
| hip roll | (19.1, 30.8, −40.8) | (19.1, 30.8, −40.8) | (17.5, 52.5, −40.9) |
| hip pitch | (−18.6, −20.6, −33.9) | (−18.6, **−32.2**, **−23.2**) | (−1.1, 20.3, −64.1) |
| knee | (0, 2.0, −72.1) | (0, −26.2, −67.1) | (−1.1, −5.9, −131.3) |
| ankle roll | (18.6, 19.1, −72.2) | (18.6, −10.5, −73.9) | (17.5, −16.4, −205.2) |
| ankle pitch | (−18.6, −20.6, −33.9) | (−18.6, −20.6, −33.9) | (−1.1, −37.1, −239.1) |

How the bold numbers were obtained: after the hip-roll joint, `R = Rx(−0.40)`. Apply it to the hip-pitch offset (−18.6, −20.6, −33.9) with the `Rx` formulas from 2.5:

```
y' = y·cos θ − z·sin θ = (−20.6)(0.92106) − (−33.9)(−0.38942) = −18.97 − 13.20 = −32.2
z' = y·sin θ + z·cos θ = (−20.6)(−0.38942) + (−33.9)( 0.92106) =   8.02 − 31.22 = −23.2
```

The rotation tilts every bone below the hip. After the ankle roll (+0.40) the two rotations cancel (`Rx(−0.40)·Rx(+0.40) = identity`), so the last offset is not rotated: the **foot is flat again**.

The result: before the lean the foot was at y = +32.4 mm; after it, y = −37.1 mm. It moved **69.5 mm**, and the shortcut `ℓ·sin L = 178.136 × sin 0.40 = 69.4 mm` predicts the same thing. The pelvis has slid 69 mm sideways *relative to the feet*, with the torso still upright.

### 4.5 The lean angle `LEAN_ANGLE = 0.40` (Q1 + Q2)

**The first attempt — and why it was wrong.** The first version used a simple rigid-body idea: "tilt the whole robot like a pole about the foot; a pole of height h tilted by θ has its top move h·tan θ". With `d = 22.3 mm` needed and `h = 188.5 mm`:

```
θ = arctan(22.3 / 188.5) = 0.1178 rad       × 1.2 safety = 0.1413 ≈ 0.142
```

Three mistakes, all found later with the proper model:

| mistake | what the first calculation used | the real value |
|---|---|---|
| where the foot is | y = 21.7 mm (the hip-yaw joint) | foot centre y = **53.0 mm**; inner edge **34.4 mm** |
| CoM height above floor | 188.5 mm | **210.1 mm** |
| how the body moves | whole robot rotates like a pole | the *pelvis slides* ℓ·sin L sideways, the legs mostly stay with the feet |

**The correct method.** For each candidate lean `L`: (1) build the pose, (2) do the forward kinematics of **every** link, (3) compute the CoM (weighted average), (4) compute the margin over the stance foot (2.8). Do that for several L and look at the numbers. For the pose where the right foot is lifted (knee −0.80) and the weight is on the left foot:

| L (rad) | L (deg) | pelvis slides ℓ·sin L | CoM moves toward the foot | margin over left foot |
|---|---|---|---|---|
| 0.142 | 8.1° | 25.2 mm | 17.2 mm | **−18.7 mm** (tips) |
| 0.200 | 11.5° | 35.4 | 24.1 | −11.7 |
| 0.300 | 17.2° | 52.6 | 35.9 | **+0.1** (on the edge) |
| 0.350 | 20.1° | 61.1 | 41.6 | +5.8 |
| **0.400** | **22.9°** | **69.4** | **47.3** | **+11.5** |
| 0.436 | 25.0° | 75.2 | 51.4 | +15.5 (hip-roll limit) |

**Why the CoM moves less than the pelvis.** At L = 0.40 the pelvis slides 69 mm, but the CoM only moves 47 mm. The reason is that the *legs stay near the feet*. Link by link, how far each part moved (relative to the stance foot):

| part | mass (kg) | moved (mm) | m × moved |
|---|---|---|---|
| torso + 2 hip-upper links (the "pelvis group") | 0.834 | 69.3 | 58.8 |
| 2 hip-lower links | 0.043 | 62.7 | 2.7 |
| 2 thighs | 0.337 | 42.1 | 14.2 |
| 2 calves | 0.210 | 14.7 and 7.6 | 2.4 |
| 2 ankles | 0.043 | 14.5 and 0.0 | 0.3 |
| 2 feet | 0.194 | 15.3 and −0.3 | 1.5 |
| **total** | **1.661** | | **78.8** |

`CoM movement = 78.8 / 1.661 = 47.4 mm`. This is the CoM formula (2.7) applied to *movements*. Only half the mass (the pelvis group) moves the full 69 mm, so the CoM moves only about two-thirds as far as the pelvis.

**Choosing 0.40.** The margin grows with L. We want it to be large (the foot is only 37 mm wide, half-width 18.6 mm; aim for at least ~8–10 mm), but the hip-roll joint is limited to 0.436 rad. `L = 0.40` is 0.036 rad (2.1°) below that limit and gives +11.5 mm at the key pose (10.0 mm at the worst moment of the whole motion). Going to 0.436 would gain only ~4 mm and leave zero room before the joint limit.

### 4.6 Flat feet: the ankle angles (Q4)

**The easy case (one direction only).** Rotations about the *same* axis simply add. All pitch joints share the y axis, but the knee axis is −y (see 4.3), so the foot's pitch relative to the torso is

```
foot pitch = hip − knee + ankle
```

For the foot to be flat: `ankle = knee − hip`. This is the rule used in the first versions. It works only when the leg has **no roll**.

**The real case.** The joint order in the leg is **hip roll → hip pitch → knee → ankle roll → ankle pitch**. The foot's total rotation is

```
R_foot = Rx(hip roll) · Ry(hip pitch − knee) · Rx(ankle roll) · Ry(ankle pitch)
```

Because rotations do not commute (2.5), a roll *before* the pitches and a roll *after* them do not cancel when the pitches are non-zero. What we need is that the foot's own "up" direction (its z axis, the third column of `R_foot`) points straight up, i.e. its x and y components are zero. That is **two equations** (`up_x = 0`, `up_y = 0`) in **two unknowns** (ankle roll, ankle pitch). We solve them with Newton's method (2.9), starting from (0, 0).

**Worked example (right leg when the right foot is first lifted):** hip roll +0.40, hip pitch +0.06, knee −0.80. Error = (up_x, up_y):

| iteration | ankle roll | ankle pitch | error (up_x, up_y) | size |
|---|---|---|---|---|
| 0 | 0 | 0 | (0.758, 0.254) | 0.799 |
| 1 | −0.6480 | −1.1616 | (−0.358, 0.130) | 0.381 |
| 2 | −0.5060 | −0.7533 | (0.037, 0.038) | 0.0535 |
| 3 | −0.5738 | −0.7738 | (−0.0007, 0.0013) | 0.0015 |
| 4 | −0.57498 | −0.77263 | (−7.6e−7, −7.8e−7) | 1e−6 |
| 5 | −0.57498 | −0.77263 | (1e−14, 4e−13) | ≈ 0 |

The first Jacobian was `J = [[0, 0.652], [0.921, −0.295]]` — the table of slopes in 2.9. Five steps are enough; the code runs it for every pose.

What the "easy rule" would have given here: ankle roll −0.40 and ankle pitch −0.86, which leaves the foot **tilted by 8.5°**. The solved values leave it at 0.00°.

**Joint limits.** The solved ankle pitch (−0.773) is beyond the ankle-pitch limit (−0.611), so the script clips it to −0.591 (the limit plus a 0.02 rad safety gap). That is acceptable *only for the foot that is in the air* — a lifted foot may tilt a few degrees. The foot on the ground must always be flat, and in every pose in the cycle the loaded foot comes out at 0.00° tilt.

### 4.7 The knee bend `KNEE_BEND = −0.80` and foot clearance

**Formula (derived).** Say the thigh is tilted forward by `a` from vertical and the knee bends by an amount `k` (positive here), so the shin is tilted `a − k` from vertical. With a flat foot, the height of the ankle joint below the hip is (using 2.2 for each bone):

```
height of ankle below hip = ℓ₁·cos(a) + ℓ₂·cos(a − k)       where ℓ₁ = 72.052 mm (thigh),
                                                                   ℓ₂ = 72.184 + 33.9 = 106.084 mm (knee → ankle pitch)
```

For a straight leg (k = 0) the height is `(ℓ₁+ℓ₂)·cos a`. The foot rises by the difference:

```
rise = (ℓ₁+ℓ₂)·cos(a) − [ ℓ₁·cos(a) + ℓ₂·cos(a − k) ]
```

**Worked example** (a = 0.12 rad, k = 0.80):

```
straight: 178.136 × cos 0.12 = 178.136 × 0.99281 = 176.86 mm
bent    : 72.052 × 0.99281 + 106.084 × cos(−0.68) = 71.53 + 106.084 × 0.77757 = 71.53 + 82.49 = 154.02... 
```

(Use the more precise table below; it was computed by the program.)

**Planar formula versus the full model:**

| knee K | formula | full 3D model | full model with no lean | ankle command |
|---|---|---|---|---|
| −0.50 | 6.8 mm | 6.5 mm | 6.8 mm | −0.349 |
| −0.65 | 13.8 | 12.7 | 13.8 | −0.484 |
| **−0.80** | 22.8 | **19.1** | 17.5 | −0.591 (clipped) |
| −0.95 | 33.7 | 21.2 | 20.1 | −0.591 (clipped) |

Read the table: the formula matches the model **exactly** (6.8, 13.8) while the lean is zero and the ankle is not at its limit. At K = −0.80 the ankle pitch has hit its limit (the clip), the foot tilts, and the clearance grows more slowly than the formula says; at −0.95 it hardly grows at all (+2 mm for 0.15 rad more bend). So **−0.80** was chosen: 19 mm of clearance (above the ~15 mm target), and further bending gains almost nothing. The small gap between "formula" and "full 3D model with lean" (6.8 vs 6.5) is the lean's cos 0.40 = 0.92 shrinking vertical motion slightly.

*The very first version used the same formula with the calf length only and a 15 mm target and gave −0.656; the real clearance there was only about 7–10 mm.*

### 4.8 The step size `STEP_PITCH = 0.12`

**Formula (derived from 2.2 / 2.4).** With a hip-pitch angle S, the ankle sits `ℓ·sin S` in front of (or behind) the hip, ℓ = 178.136 mm. In a step, the stance leg is pitched back by S and the swing leg forward by S, so the swing foot lands

```
landing distance = 2 · ℓ · sin S           (between the two feet, front-to-back)
```

and the body moves `ℓ · sin S` forward over the stance foot during the swing. (The "30 mm step" in the first version was this second number, not the step length.)

| S | formula 2ℓ sin S | measured by the model | worst CoM margin over one foot |
|---|---|---|---|
| 0.08 | 28.5 mm | 28.0 mm | +10.0 mm |
| 0.10 | 35.6 | 35.0 | +6.6 |
| 0.12 | 42.7 | 42.0 | +3.1 |
| 0.12 **with lift-off shift 0.06** | 42.7 | 42.0 | **+10.1** |
| 0.16 | 56.8 | 55.9 | −3.7 |
| 0.169 (first version) | 59.9 | 59.0 | **−5.3** |

**The heel problem.** In the stride stance the stance foot is `ℓ·sin S` in front of the body. Its heel is 30 mm behind the ankle. When the other foot lifts, the CoM must already be in front of that heel. For S = 0.169 the heel is at `178.136 × sin 0.169 − 30 = 0 mm` ahead of the CoM — the CoM sits on the edge, and the model says −5 mm. A shorter step puts the heel behind the CoM. But shorter steps are slow, which is why the next trick matters.

### 4.9 The lift-off shift `LIFTOFF_SHIFT = 0.06`

Just before lifting a foot, both hips are pitched back by d = 0.06 rad, which moves the whole body forward over the planted feet by

```
ℓ · sin d = 178.136 × sin 0.06 = 10.7 mm
```

That pushes the CoM away from the heel and over the stance foot before the lift. Look at the table above: with d = 0.06 the margin at S = 0.12 jumps from +3.1 to +10.1 mm, with the same step length.

**Side effect.** The two legs now have different pitch angles, so their effective vertical lengths differ by

```
Δh = ℓ · (cos a₁ − cos a₂) = 178.136 × (cos(−0.06) − cos(0.18)) = 2.56 mm
```

The leg that is about to lift is 2.6 mm shorter, so its foot already hangs 2.6 mm off the ground. That is harmless (that foot is about to lift anyway), and `d` was kept small for exactly this reason. A bigger shift would need real leg inverse kinematics to keep both feet on the ground.

### 4.10 The timing: why 1.5–2.5 s per move (Q3)

**Smooth motion (derivation).** We want a movement `s(t)` that goes from 0 to 1 with **zero speed at both ends** (no jerks). Try a cubic `s = a·t³ + b·t² + c·t + e`:

```
s(0) = 0   →  e = 0
s′(0) = 0  →  c = 0                 (speed zero at start)
s(1) = 1   →  a + b = 1
s′(1) = 0  →  3a + 2b = 0           (speed zero at the end)
solve:  b = −3a/2,  a − 3a/2 = 1  →  a = −2, b = 3
```

```
s(t) = 3t² − 2t³          ("smoothstep")        s(0.25)=0.156  s(0.5)=0.5  s(0.75)=0.844
```

Acceleration is the second derivative: `s′ = 6t − 6t²`, `s″ = 6 − 12t`, whose biggest size is **6** (at the start and end). A move of size D over time T is `D·s(t/T)`, so its peak acceleration is

```
a_peak = 6 · D / T²
```

**The ZMP formula (derivation).** Treat the robot as a point mass m at height z_c above the floor, at horizontal position x_c, accelerating forward with `a`. The floor pushes with an upward force N at the point p, and a horizontal friction force f:

```
vertical:    N = m·g                   (it is not moving up or down)
horizontal:  f = m·a                   (Newton: force = mass × acceleration)
```

Take the turning effects (torques) about the CoM; the robot is not rotating, so they must sum to zero. N acts at horizontal distance (p − x_c) from the CoM, f acts at height z_c below the CoM:

```
(p − x_c)·N + z_c·f = 0
p = x_c − z_c·f / N = x_c − (z_c / g)·a
```

So **the point where the ground effectively pushes (the "ZMP") is the CoM position shifted by (z_c/g)·a**. If the motion is slow (a ≈ 0) the ZMP equals the CoM's floor projection — the pure "CoM over the foot" test of 2.8. If the motion is fast, the ZMP moves away from the CoM, and the real requirement becomes "the ZMP over the foot". The shift is `z_c·a/g` with z_c = 0.2101 m:

**Worked example.** A 40 mm CoM move:

```
T = 2.0 s:  a = 6 × 0.040 / 2.0² = 0.060 m/s²    shift = 0.2101 × 0.060 / 9.81 = 1.3 mm
T = 0.4 s:  a = 6 × 0.040 / 0.4² = 1.5  m/s²     shift = 0.2101 × 1.5   / 9.81 = 32.1 mm   ← bigger than the whole margin
```

That is why this gait *must* be slow. Doubling the speed makes the effect four times larger.

**The table used to choose the times.** For each move, the program tracked the CoM through the whole smoothstep motion and found the peak acceleration (relative to the planted foot):

| move | time T | CoM travel | peak accel | ZMP shift | feet on the ground |
|---|---|---|---|---|---|
| neutral → lean left | 2.5 s | 46.0 mm | 0.044 m/s² | 0.94 mm | both |
| lean → lift right (1st) | 1.5 | 1.3 | 0.011 | 0.23 | one |
| lift → swing right | 2.0 | 12.9 | 0.019 | 0.40 | one |
| swing → place right | 1.5 | 7.9 | 0.021 | 0.45 | one |
| place → lean right | 2.5 | 91.1 | 0.080 | 1.72 | both |
| lean → lift left | 1.5 | 3.2 | 0.012 | 0.26 | one |
| lift → swing left | 2.0 | 33.4 | 0.048 | 1.04 | one |
| swing → place left | 1.5 | 7.9 | 0.021 | 0.45 | one |
| place → lean left | 2.5 | 91.1 | 0.080 | 1.72 | both |

**Rule used:** keep the ZMP shift below about a quarter of the stability margin (≈ 2.5 mm), then round to convenient times. All values are 1.7 mm or less; the moves with only one foot down are 1.0 mm or less. (The weight-shift moves are slower than they need to be only because they cross the centre of the robot with both feet down, where margins are big.)

### 4.11 Why `stop` finishes the step (Q2 again)

If you blend straight from a pose with a foot in the air to the standing pose, the CoM has to travel back to the middle while one foot is still up. Computed worst margin along that path:

| stop started from | worst margin |
|---|---|
| lift right (1st) | −25.8 mm |
| swing right | −24.3 |
| lift left | −25.9 |
| swing left | −23.0 |
| lift right | −27.3 |
| lean left / place right / lean right / place left / lean left (cycle) | both feet stay down — safe |

So `stop` keeps walking until it reaches one of the "both feet down" poses, and only then returns to standing.

### 4.12 The backward gait numbers

For backward walking the same step size gave a weaker margin: with `S = 0.12, d = 0.06` the margin is **+3.1 mm** (K = −0.80) or **+5.4 mm** (K = −0.50). The reason is a mirrored geometry: the foot is longer in front of the ankle (60 mm) than behind it (30 mm), so going backward the CoM has less foot to stand on. A search over S, d and K found:

```
S = 0.08, d = 0.04, K = −0.65   →   margin +10.0 mm,  clearance 19.9 mm
```

so backward uses those. They were checked in the model and the stop logic was tested, but backward has had far less testing than forward.

### 4.13 The final set of poses, and the self-check

```
neutral         → standing
lean_left       → weight over the left foot (roll ±0.40)
lift_right_1st  → right knee −0.80, both hips pitched back 0.06
swing_right     → right leg forward 0.12, left leg back 0.12, right knee −0.80
place_right     → right knee straight, foot flat on the floor, 42 mm ahead
lean_right      → weight over the right foot
…and the mirror image on the other side, repeating.
```

The script contains a short version of exactly the calculations in this document. When it starts it:

1. solves the ankles for every sample of every move (4.6),
2. computes the CoM and the sole corners with forward kinematics (4.1, 4.4),
3. finds the margin whenever only one foot is down (2.8),
4. prints the worst margin and the swing-foot clearance. Run it without ROS with:

```bash
python3 quasi_static_walk.py --check
```

Expected output: `worst one-foot stability margin 10.0 mm … swing-foot clearance 19.1 mm … OK`.

---

## 5. Which numbers were calculated, judged or tested?

| Constant | How it was obtained | Honest label |
|---|---|---|
| Hip roll, hip pitch, knee **signs** | derived from the URDF axes (4.3) | calculated, and confirmed by tests (the original wrong signs made it split, walk backward or fall) |
| Ankle angles | solved numerically for a flat foot at every moment (4.6) | **calculated** |
| `LEAN_ANGLE 0.40` | margin versus L from the model (4.5), capped by the hip-roll limit | **calculated**, with a *judgement* about how much margin to keep |
| `KNEE_BEND −0.80` | clearance formula and model (4.7), stopped where the ankle hits its limit | calculated, **judgement** on the target (~15 mm) |
| `STEP_PITCH 0.12` | margin versus step size (4.8) | calculated; the trade-off (bigger step vs margin) is a **judgement** |
| `LIFTOFF_SHIFT 0.06` | searched in the model (4.9) | found by **searching**, then explained |
| Segment times 1.5–2.5 s | ZMP table (4.10) with a "quarter of the margin" rule, rounded | calculated + **rule of thumb** |
| Backward values | searched in the model (4.12) | calculated; **lightly tested** |
| "It actually works in Gazebo" | you ran it | **tested** — this is the real proof; the maths only gives the reasons and the starting values |

---

## 6. What this method cannot guarantee

Be ready to say these, because an expert will ask:

- **It is a static model.** Links are rigid, the torso is assumed upright, joints are assumed to reach their commanded angles exactly and instantly (no lag, no sag, no motor limits). Real servos and the simulator's joint controller are softer.
- **The masses and centres of mass come from the URDF** (CAD data). They have not been measured on the real robot.
- **Open loop.** There is no feedback. If something is a little off (floor friction, a bump, a motor offset) the robot cannot correct for it. This is why the margin is a few millimetres and not zero — and why a few millimetres is not a lot.
- **Collisions between the legs** were checked only with a simple test (the two legs stay at least 22 mm apart at every height), not a full collision check.
- **Flat floor only.** The foot is modelled as a flat rectangle.
- **The first version's lean calculation was wrong** (4.5). It happened to walk in simulation anyway, which shows that this model and the simulator disagree somewhere; this model is *conservative*. Use it as a design tool, not as a proof.

---

## 7. Glossary

| word | meaning |
|---|---|
| **URDF** | the file describing the robot: links (parts), joints, lengths, masses |
| **link** | one rigid part (thigh, calf, foot, …) |
| **joint** | a motor/hinge connecting two links |
| **frame** | a set of x, y, z axes attached to a part |
| **forward kinematics (FK)** | joint angles → positions of every part |
| **CoM** | centre of mass: the mass-weighted average position of the whole robot |
| **support polygon** | the shape of the ground contact (rubber band around the soles) |
| **margin** | distance from the CoM's floor projection to the nearest edge of the support polygon (+ safe, − tips) |
| **quasi-static** | moving so slowly that only gravity matters, not momentum |
| **ZMP** | zero-moment point: where the ground's push effectively acts; equals the CoM's projection when moving slowly |
| **smoothstep** | the curve `3t² − 2t³`, which starts and ends with zero speed |
| **Newton's method** | repeated "tangent-line" guesses for solving equations numerically |
| **Jacobian** | the table of slopes used in Newton's method with several unknowns |
| **stance foot** | the foot carrying the weight; **swing foot** is the lifted one |
| **clearance** | how high the lifted foot is above the floor |
| **open loop** | commands are sent without measuring the result |

---

## 8. Where each piece lives in `quasi_static_walk.py`

| In this document | In the script |
|---|---|
| 2.5 rotation matrices | `_rot()` |
| 2.9 + 4.6 Newton, flat foot | `solve_ankles()` |
| 2.7 + 4.4 CoM and forward kinematics | `_com_and_feet()` |
| 2.8 support polygon and margin | `_hull()`, `_margin()` |
| 4.3, 4.5, 4.7–4.9 the poses and constants | `build_keyframes()` + the `TUNING` block |
| 4.10 smoothstep | `blend()` |
| 4.10 times | `SEGMENT_TIME` |
| 4.11 stop | `SAFE_STOP` and the stop logic in the node |
| 4.13 self-check | `check_gait()` / `--check` |
