# Scissor-UMI · Robotiq 2F-140

<p align="center"><img src="img/hero.png" alt="Scissor-UMI for the Robotiq 2F-140: scissor handles, 2F-140 replica jaws, a D405 on a tilting hinge and a Quest Touch Pro on top" width="720"></p>

A scissor-handled data-collection gripper whose **jaws are printed copies of a Robotiq 2F-140's jaws**: its outer
knuckle and outer finger, inner knuckle, inner finger and pad. They are pinned at the 2F-140's own joint positions, so
they move exactly like the robot's. On the robot side you use the real 2F-140, with the same D405 + Quest head on a
flange collar. The camera, the controller and the jaws end up in the same place on both, and the handheld's jaw angle
is the robot's `finger_joint`, so a recorded q can be sent straight to the gripper.

The design follows [FOMI](https://robbyant-research.github.io/FOMI/): scissor handles, meshed sector gears that keep
the jaws symmetric, and force felt through the handles. The changes here:

* **The jaws are the 2F-140's.** They are Robotiq's jaw-part geometry at the URDF's joint frames
  ([`cad/replica.py`](../cad/replica.py), [`cad/kin.py`](../cad/kin.py)). They close at the same q = 0.7, and the pads
  follow the robot's arc, including its forward travel as it closes.
* **The camera is a RealSense D405** in HandUMI's cup on a tilting knuckle (30° down by default). FOMI uses a fisheye.
* **The tracker is a Meta Quest Touch Pro** in HandUMI's controller ring, on top behind the camera. FOMI uses a Vive
  tracker.
* **There is a 2:1 handle gear.** The two knuckle gears (20 teeth) sit inside the inner knuckles' base clevis, on the
  2F-140's inner pivots, and mesh with each other. Each handle has a 40-tooth sector on one shared axle. The loops turn
  only 20° over the full stroke: about 96 mm apart open and 38 mm apart closed.

## Handheld and robot

<p align="center"><img src="img/twins.png" alt="The handheld and the Robotiq 2F-140 with the same head, seen from behind at the same q" width="880"></p>

| Handheld | Robot twin |
|---|---|
| <img src="img/hero.png" width="420"> | <img src="img/robot_2f140.png" width="420"> |

### What the D405 sees

<p align="center"><img src="img/camera_views.png" alt="The D405's view on the handheld (left) and on the 2F-140 (right) at the same q: identical jaws and pads" width="880"></p>

These are the D405 views on the handheld (left) and on the robot (right) at the same q. The jaws and pads are
identical; only the base at the bottom edge of the image differs.

## Mechanism

| Open (q = 0) | Closed (q = 0.7) |
|---|---|
| <img src="img/top_open.png" width="380"> | <img src="img/top_closed.png" width="380"> |

<p align="center"><img src="img/stroke.png" alt="The stroke from open to closed" width="880"></p>

The parts are layered along x. The axis convention is the 2F-140's `robotiq_arg2f_base_link`: z is the approach
direction, y is the opening direction and x points up.

| Layer | x (mm) | What's there |
|---|---|---|
| bottom plate | −25.5 … −20.5 | standoffs, pivot spacers, arc slot for the stop screw, record button pod; the electronics box hangs under it |
| 2F-140 jaws | −19.6 … 19.6 | the replica parts at their own x, as on the gripper |
| L1 | −11.5 … −4.2 | handle A + its sector, the left knuckle gear's drive teeth |
| L2 | −3.7 … 3.7 | the two knuckle gears' sync teeth |
| L3 | 4.2 … 11.5 | handle B + its sector, the right knuckle gear's drive teeth |
| top plate | 20.5 … 25.5 | pivot spacers, the mast to the head |

* **Jaws.** The 2F-140's parallelogram: the outer link (outer knuckle + outer finger, printed as one piece; on the
  gripper they are bolted together) and the inner knuckle pivot on Ø4 dowels through both plates. Their far ends pin
  to the inner finger, which carries the pad. The right jaw uses the same parts turned 180°, as on the gripper.
* **What changed from Robotiq's parts.** Only features that are hidden on the gripper:
  * Ø4.2 bores on the joint axes, with the pins Robotiq's meshes model as part of the links removed.
  * The outer knuckle's drive lever, which sits inside the gripper body, is cut off.
  * The inner knuckle's base clevis is emptied for the gear, with 2 × M3 holes through one prong that key the gear to
    it.
  * The inner finger's joint faces are shaved by 0.3 mm for running clearance.
  * The pad (the URDF's 27 × 65 × 7.5 mm box) is split into a PETG core screwed to the inner finger and a 2 mm TPU
    face. It sits flush with the finger, 0.43 mm off the URDF box.
* **Gears.** The flanks are thinned by 0.08 mm so the printed teeth run free.
* **Stops.** An M3 screw in handle A runs in an arc slot in the bottom plate. The slot's ends are q = 0 and q = 0.7.
* **Sensing.** A diametric magnet sits in handle A's lower hub, over an AS5047P in the electronics box. q is twice the
  handle angle. See [`electronics.md`](electronics.md).
* **Head.** The head bolts onto a 4 × M3 interface at x = 46 mm: the top of the mast on the handheld, the collar's
  arm on the robot. It carries the D405 cup's knuckle (M4, tilts 15–65°) and a copy of HandUMI's main-support arm end,
  so HandUMI's Quest ring bolts on as it does on HandUMI. For the left hand, use `head_left` and the mirrored ring.
* **Robot collar.** A 6 mm ISO 9409-1-50-4-M6 spacer between the flange and the 2F-140's coupling. Use M6 screws 6 mm
  longer than the coupling's own, and turn the collar to keep the arm clear of the coupling's cable exit.

## Verification

`python cad/verify.py` checks the following:

| Check | Result |
|---|---|
| Gear meshes over the whole stroke (sync, handle A, handle B), from sections of the real parts | engaged throughout, 0 tooth interference, backlash 1.8° (sync) and 0.9° (handles) |
| Handheld jaws vs Robotiq's 2F-140 meshes on the robot at q = 0, 0.35, 0.7 | median surface distance 0.00 mm for every part; pads within 0.43 mm |
| Interference between all parts over the stroke; head, D405 and Quest vs both builds | none; the pads meet at q = 0.7 |

The 2F-140's pad faces are 128.6 mm apart at q = 0 in Robotiq's URDF (the 140 mm on the datasheet is measured
elsewhere on the finger). Both builds match the URDF.

## Files

| | |
|---|---|
| [`STEP/`](STEP/) | every printed part (STEP only); the `*_2f140` parts are the jaw replicas |
| [`bom.md`](bom.md) | printed and bought parts, handheld and robot |
| [`electronics.md`](electronics.md) | Pico 2, AS5047P, button, USB hub |
| [`urdf/scissor_umi_2f-140.urdf`](urdf/scissor_umi_2f-140.urdf) | handheld: jaws (`finger_joint` + mimics), pads, `d405_optical_frame`, `quest_ring_frame` |
| [`urdf/2f-140_head.urdf.xacro`](urdf/2f-140_head.urdf.xacro) | the same two frames as a macro on the robot's `robotiq_arg2f_base_link` |
| [`NOTICE.md`](NOTICE.md) | where the jaw geometry comes from |

Camera pose in G (mm): optical centre (71.8, 0, 48.1), looking (−0.5, 0, 0.866), i.e. 30° below the approach axis.

## Status

This version is for review and has not been printed yet. The jaw replicas come from Robotiq's ROS visual meshes, so
their STEP is faceted. They are dimensionally exact to those meshes, which may lack small fillets of the production
parts.
