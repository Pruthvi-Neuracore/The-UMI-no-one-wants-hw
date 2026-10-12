# Retargeting PG Chopstick demos onto a robot

## What a demo gives you

For each hand and each frame, a demo gives two things:

- `T_world_camera`: the pose of the gripper's camera frame in the demo world, from any SLAM or visual-inertial
  odometry that tracks the X5 or D405.
- θ: the jaw angle, from the red dots.

## Frames

All of these are measured from the CAD by `cad/kinematics.py` and stored in
`retarget/pg_chopstick/pg_chopstick_kinematics.json`. The same frames appear in every URDF.

| Frame | Where | Gripper-frame position |
|---|---|---|
| `chopstick_base` (gripper frame) | centre of the mounting face; +X along the jaws, +Y to the left jaw, +Z up | – |
| `chopstick_camera` (X5) | midway between the lens domes, pitched 10° nose-down | (34.4, 0, 57.0) mm |
| `chopstick_camera` (D405) | centre of the front face, view axis 35° down | (39.4, 0, 47.4) mm |
| `chopstick_tcp` | where the TPU tips meet when closed | (204.7, 0, −3.7) mm |
| `chopstick_flange` (robot builds) | the robot flange (OpenArm link7, or the tool flange) | – |

**Mounts** (`T_flange_gripper`):

- `openarm`: the gripper's +X along link7 +Z, +Y along −Y, +Z along +X, at link7 z = 109.5 mm.
- `iso50`: the same axes on the tool flange, 16 mm out from the flange face.

The camera → TCP transform depends only on the camera:

- **X5:** 178.3 mm forward, 30.2 mm down.
- **D405:** 164.7 mm forward, 53.0 mm "up" in its pitched frame.

## The mapping

**PG Chopstick robot gripper** (`end_effector="chopstick"`). The robot carries the same jaws and the same camera
mount, so its camera goes exactly where the demo camera was:

```
T_base_flange = T_base_world · T_world_camera · (T_flange_gripper · T_gripper_camera)^-1
θ_robot       = θ_demo  (clipped to 72°),   servo_steps = zero + sign · θ · 4096 / 2π
```

**Stock OpenArm parallel hand** (`end_effector="stock"`, OpenArm only). The TCPs are matched instead, and the tip gap
becomes the finger opening (`clip(gap / 2, 0, 0.044)`).

`T_base_world` places the demo scene in the arm's base frame. Use one retargeter per arm.

## Jaw angle

- **From the red dots:** `kin.theta_from_dots(left, right, frame="camera")` takes (N, 3) dot centres in metres, with
  NaN rows for unseen dots. It returns θ within 0.6° at 0.5 mm of noise per dot. `kin.dots_at(θ, frame="camera")`
  gives the template for a detector.
- **From a tip gap:** `kin.theta_from_gap(gap)`. Reference values: 10° → 48 mm, 20° → 96 mm, 30° → 141 mm.

## IK

- **OpenArm:** `python urdf.py --openarm-urdf .../openarm.urdf --out openarm_pg_chopstick.urdf` swaps the stock hands
  for the gripper on both arms.
- **ISO-flange cobots:** add a fixed joint from the robot's flange link (for example UR or FANUC `tool0`) to
  `chopstick_flange` in `urdf/pg_chopstick_iso50.urdf`.

## CLI

```bash
pip install -e retarget
pg-chopstick-retarget demo_right.csv -o right_robot.csv --camera x5 --mount iso50 --base-world T_base_world.npy
```

Input columns: `t, x, y, z, qx, qy, qz, qw, theta` (or `gap` instead of `theta`), the camera pose in the demo world.
Output columns: the flange pose (named `link7_*`) and TCP pose in the arm base frame, plus `theta, servo_steps`, or
`finger` for `--ee stock`.
