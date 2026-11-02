# Scissor-UMI · Robotiq 2F-140

<p align="center"><img src="img/hero.png" alt="Scissor-UMI for the Robotiq 2F-140: scissor handles, 2F-140 replica jaws, the D405 on the robot's own camera tab and a Quest Touch Pro above it" width="720"></p>

A scissor-handled data-collection gripper whose **jaws are printed copies of a Robotiq 2F-140's jaws**: its outer
knuckle and outer finger, inner knuckle, inner finger and pad. They are pinned at the 2F-140's own joint positions, so
they move exactly like the robot's. **The D405 sits where it sits on the robot**: the handheld carries an identical
copy of the tab of our UR camera plate, the plate that mounts the D405 on the real 2F-140. A **Feetech STS3215**
on the scissor axle reads the jaw angle, as on HandUMI, and the **Quest Touch Pro** stands above the camera. On the
robot you use the real 2F-140 with the camera plate and the D405 as they are.

The design follows [FOMI](https://robbyant-research.github.io/FOMI/): scissor handles, meshed sector gears that keep
the jaws symmetric, and force felt through the handles.

## Handheld and robot

<p align="center"><img src="img/twins.png" alt="The handheld and the Robotiq 2F-140 on its camera plate, seen from behind at the same q" width="880"></p>

| Handheld | Robot (2F-140 + UR camera plate + D405) |
|---|---|
| <img src="img/hero.png" width="420"> | <img src="img/robot_2f140.png" width="420"> |

### What the D405 sees

<p align="center"><img src="img/camera_views.png" alt="The D405's view on the handheld (left) and on the 2F-140 (right) at the same q: identical jaws and pads" width="880"></p>

The D405 on the handheld (left) and on the robot (right), at the same q. The camera is in the same place on its tab,
78 mm off the axis at the flange, looking forward and 8° in towards the axis. The jaws and pads are identical; only
what sits behind them at the bottom edge of the image differs.

## Mechanism

| Open (q = 0) | Closed (q = 0.7) |
|---|---|
| <img src="img/top_open.png" width="380"> | <img src="img/top_closed.png" width="380"> |

<p align="center"><img src="img/stroke.png" alt="The stroke from open to closed" width="880"></p>

The parts are layered along x. The axis convention is the 2F-140's `robotiq_arg2f_base_link`: z is the approach
direction, y is the opening direction and x points up.

| Layer | x (mm) | What's there |
|---|---|---|
| electronics box | −71 … −25.5 | STS3215 (on the scissor axle), Pico 2, IMU, USB 3 hub |
| bottom plate | −25.5 … −20.5 | standoffs, pivot spacers, arc slot for the stop screw, record button pod |
| 2F-140 jaws | −19.6 … 19.6 | the replica parts at their own x, as on the gripper |
| L1 / L2 / L3 | −11.5 … 11.5 | handle A + left gear's drive teeth / the sync teeth / handle B + right gear's drive teeth |
| top plate | 20.5 … 25.5 | pivot spacers; the mast's foot |
| mast, hood, Quest arm | above | the camera tab copy, the D405, the Touch Pro |

* **Handles.** Handle A (lower) has a two-finger bow (26 × 46 mm inside) with a finger rest. Handle B (upper) has a
  thumb ring (Ø26 inside). Both are 14 mm tall with 7 mm walls and every edge rounded (R 3.4), so nothing sharp
  presses into the fingers. Each arm leaves its gear as a 16 mm wide web and is lofted, rounded, up to the ring's full
  height, so the section is largest where the bending load is. The loops turn only 20° over the full stroke: about
  96 mm apart open and 38 mm apart closed.
* **Jaws.** The 2F-140's parallelogram, built from replicas of its parts. The outer link (outer knuckle + outer
  finger, printed as one piece) and the inner knuckle pivot on Ø4 dowels through both plates. Their far ends pin to the
  inner finger, which carries the pad. The right jaw uses the same parts turned 180°, as on the gripper. Only hidden
  features differ from Robotiq's parts: bores, the modelled pins removed, the drive lever inside the body cut off, the
  base clevis emptied for the gear (2 × M3 key it), 0.3 mm joint clearance, and the pad split into a PETG core and a
  2 mm TPU face.
* **Gears.** The two knuckle gears (20 teeth, m 1.27) sit in the inner knuckles' base clevis on the 2F-140's inner
  pivots and mesh with each other. Each handle has a 40-tooth sector on one shared axle (2:1). The flanks are thinned
  by 0.08 mm so printed teeth run free.
* **Encoder.** A Feetech STS3215 hangs under the bottom plate on the scissor axle, inside the electronics box. Its
  horn is screwed (4 × M2) to `servo_coupler`, which takes handle A's hex end. It runs with torque off, as on HandUMI,
  and q = 2 × its angle: about 228 counts over the stroke, roughly 0.6 mm of pad travel per count.
* **Stops.** An M3 screw in handle A runs in an arc slot in the bottom plate. The slot's ends are q = 0 and q = 0.7.
* **Camera.** `tab_mast` stands on the top plate and puts a copy of the camera plate's tab face, with its six holes,
  at exactly the same pose in G. The D405 fixes to it the way it fixes to the robot plate. `d405_hood` is an open
  frame round the camera with no floor, so the camera sits straight on the tab. It is screwed from behind into the
  tab's two outer holes.
* **Quest.** `quest_arm` closes the hood's top and rises, as one piece, to a split collar bored to the Touch Pro's
  handle and clamped with one M3. The controller stands upright on the centre plane right above the camera, its face
  looking forward over the jaws and its cameras clear. For the left hand, use `quest_arm_left`.
* **Robot side.** The real 2F-140 on `ur_camera_plate` (our part, unchanged; it puts the coupling 7 mm further from
  the UR flange) with the D405 on its tab. There is no Quest on the robot.

## Verification

`python cad/verify.py` checks the following:

| Check | Result |
|---|---|
| Gear meshes over the whole stroke (sync, handle A, handle B), from sections of the real parts | engaged throughout, 0 tooth interference, backlash 1.8° (sync) and 0.9° (handles) |
| Handheld jaws vs Robotiq's 2F-140 meshes on the robot at q = 0, 0.35, 0.7 | median surface distance 0.00 mm for every part; pads within 0.43 mm |
| Interference between every pair of parts over the stroke (handles, gears, jaws, servo, box, mast, hood, D405, Quest arm and controller), and the robot's plate and D405 vs the gripper | none; the pads meet at q = 0.7 |

The 2F-140's pad faces are 128.6 mm apart at q = 0 in Robotiq's URDF (the 140 mm on the datasheet is measured
elsewhere on the finger). Both builds match the URDF.

## Files

| | |
|---|---|
| [`STEP/`](STEP/) | every part (STEP only). The `*_2f140` parts are the jaw replicas. `ur_camera_plate` is the robot's camera plate |
| [`bom.md`](bom.md) | printed and bought parts, handheld and robot |
| [`electronics.md`](electronics.md) | Pico 2, STS3215, IMU, button, USB hub |
| [`urdf/scissor_umi_2f-140.urdf`](urdf/scissor_umi_2f-140.urdf) | handheld: jaws (`finger_joint` + mimics), pads, `d405_optical_frame`, `quest_ring_frame` |
| [`urdf/2f-140_camera.urdf.xacro`](urdf/2f-140_camera.urdf.xacro) | the same `d405_optical_frame` as a macro on the robot's `robotiq_arg2f_base_link` |
| [`NOTICE.md`](NOTICE.md) | where the jaw geometry and the reference models come from |

## Status

This version is for review and has not been printed yet. The jaw replicas come from Robotiq's ROS visual meshes, so
their STEP is faceted. They are dimensionally exact to those meshes, which may lack small fillets of the production
parts.
