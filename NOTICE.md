# Notice

The chopstick jaws, chopstick bases, clips and Insta360 X5 frames are derived from TToTMooN/KIWI_hardware
(commit `626ad7b`), licensed under Apache-2.0; the licence text is in `LICENSE`. The unmodified sources are in
`cad/<kit>/source/upstream/`.

The parallel grippers are HandUMI from Pruthvi-Neuracore/The-UMI-no-one-wants (its parts, tip sets and assembly code,
vendored unchanged in `cad/<kit>/handumi/`; itself derived from murobotics-ai/handumi-hw, Apache-2.0, with the Open-ENPIRE
tips from pgeedh/Open-ENPIRE-Gripper, Apache-2.0; see `cad/<kit>/handumi/hardware/NOTICE.md`). The robot renders in
the parallel folders are HandUMI's. The one new part there is `x5_wrist_mount`, the X5 cradle on HandUMI's camera hinge.

The D405 cup (`d405_cup`) is the HandUMI D405 wrist mount from Pruthvi-Neuracore/The-UMI-no-one-wants, itself built on
murobotics-ai/handumi-hw (Apache-2.0). It is used unchanged, from `cad/d405/source/handumi/`.

Changes and new work in this repository:

- The upstream parts were split, renamed, moved to the closed pose, and engraved with the PG badge.
- The handheld jaws got elastic finger-sleeve pads and a record-button pocket.
- New parts: the PG pod and wrist saddles (also for robot forearms), the X5 cradle for HandUMI, the D405 frame with its tilt knuckle, the
  electronics box, its lid and wrist saddle, the Quest controller cradle, the OpenArm adapter and the ISO 50 flange plate.
- Also new: the kinematics, URDFs, assemblies, renders, tag sheet and documentation.

The OpenArm, UR, ABB and FANUC geometry used for checks and renders comes from the robot descriptions in the Neuracore
robots repository, and is not redistributed here.

Render-only reference models:

- **Shadow Hand** (`cad/<kit>/source/shadow_hand/`, used for `hand_robot.png`): from google-deepmind/mujoco_menagerie,
  Apache-2.0 (its `LICENSE` is alongside), posed and exported to PLY.
- **Human hand** (`hand.png`): Meta's rigged hand from the Quest hand-tracking samples. Not redistributed; see
  `cad/<kit>/source/hand/README.md`.
- **Quest Touch Pro controller** (D405 kit): Meta's model. The raw file is not redistributed (see
  `cad/d405/source/quest/README.md`), but its shape appears in the D405 renders, assembly GLB/STL and URDF meshes for
  visualisation only. It is Meta's property and is not covered by this repository's licence.
