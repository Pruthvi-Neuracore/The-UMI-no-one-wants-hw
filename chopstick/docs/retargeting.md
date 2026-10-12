# Retargeting chopstick demos onto OpenArm

## What a demo gives you

For each hand and each frame, KIWI's pipeline (or any SLAM that tracks the X5) gives two things:

- `T_world_camera`: the pose of the X5's **camera frame** in the demo world. That frame is midway between the two lens
  domes, with +X out of the front lens and +Z up.
- The **jaw angle** θ, recovered from the red dots on the jaws.

## Frames

All of these are measured from the CAD by `cad/kinematics.py` and stored in
`retarget/chopstick_retarget/chopstick_kinematics.json`. The same frames appear in the URDFs.

| Frame | Where | From the gripper frame |
|---|---|---|
| `chopstick_base` (gripper frame) | centre of the mounting face; +X along the jaws, +Y to the left jaw, +Z up | – |
| `chopstick_camera` | midway between the X5 lens domes, pitched 10° nose-down | (34.4, 0, 57.0) mm, pitch 10° |
| `chopstick_tcp` | where the TPU tips meet when closed | (204.7, 0, −3.7) mm |
| `chopstick_tip_left/right` | the TPU tip ends, which move with the jaws | – |
| `chopstick_flange` (OpenArm URDF only) | OpenArm link7 frame | gripper = link7 · `T_LINK7` |

`T_LINK7` (in `cad/openarm_adapter.py`) puts the gripper's +X along link7 +Z, +Y along link7 −Y and +Z along link7 +X,
at link7 z = 109.5 mm. The camera → TCP transform is the same on the handheld and the robot gripper:
178.3 mm forward and 30.2 mm down in the camera frame, with a 10° pitch.

## The mapping

**Chopstick robot gripper** (`end_effector="chopstick"`). The robot carries the same jaws and the same camera mount, so
the robot's camera goes exactly where the demo camera was:

```
T_base_link7 = T_base_world · T_world_camera · (T_link7_gripper · T_gripper_camera)^-1
θ_robot      = θ_demo                    (clipped to the clash-free 72°)
servo_steps  = zero + sign · θ · 4096 / 2π
```

Since the robot's camera ends up in the same place, the robot sees what the demo camera saw. That keeps policies trained
on the X5 video valid on the robot.

**Stock OpenArm parallel hand** (`end_effector="stock"`). The TCPs are matched instead. Both TCP frames approach along
+X and close along Y, so no extra rotation is needed. The tip gap becomes the finger opening:

```
T_base_link7 = T_base_world · T_world_camera · T_camera_tcp · (T_link7_stock_tcp)^-1
finger       = clip(tip_gap(θ) / 2, 0, 0.044)
```

`T_base_world` places the demo scene relative to the arm's base. Use the identity if the demo is already expressed in
the robot base frame. For a bimanual setup, use one retargeter per arm with that arm's base frame.

## Jaw angle

- **From the red dots:** `kin.theta_from_dots(left, right, frame="camera")` takes the 3D dot centres, as (N, 3) arrays
  in metres. Mark unseen dots with NaN rows. It fits the rotation of each dot about its jaw's pivot and returns the
  weighted circular mean over both jaws. With 0.5 mm of noise per dot it recovers θ within 0.6° (see the tests).
  `kin.dots_at(θ, frame="camera")` gives the template for a detector.
- **From a measured tip gap:** `kin.theta_from_gap(gap)`.
- **Reference values:** 10° → 48 mm, 20° → 96 mm, 30° → 141 mm.

## IK

Generate an OpenArm URDF with the chopstick gripper on both arms:

```bash
cd cad && python urdf.py --openarm-urdf /path/to/openarm_description/openarm.urdf --out openarm_chopstick.urdf
```

This removes the stock hand, its TCP and its fingers, then attaches `openarm_{left,right}_chopstick_flange` to link7.
That adds the jaw joints `openarm_{left,right}_chopstick_jaw`, with the right jaw as a mimic. Solve IK for
`openarm_{side}_link7` → `T_base_link7`, or for `openarm_{side}_chopstick_camera` / `_tcp` directly.

## Offline CLI

```bash
pip install -e retarget
chopstick-retarget demo_right.csv -o right_openarm.csv --ee chopstick --base-world T_base_world.npy \
                   --servo-zero 2048 --servo-sign 1
```

Input columns: `t, x, y, z, qx, qy, qz, qw, theta`, or `gap` instead of `theta`. This is the camera pose in the demo
world. Output columns: link7 and TCP poses in the arm base frame, plus `theta, servo_steps` (or `finger` for
`--ee stock`).
