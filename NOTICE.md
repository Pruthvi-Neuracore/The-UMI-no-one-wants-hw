# Notice

The chopstick jaws, chopstick bases, clips and Insta360 X5 frames are derived from TToTMooN/KIWI_hardware
(commit `626ad7b`), licensed under Apache-2.0; the licence text is in `LICENSE`. The unmodified sources are in
`cad/<kit>/source/upstream/`.

The D405 cup (`d405_cup`) is the HandUMI D405 wrist mount from Pruthvi-Neuracore/The-UMI-no-one-wants, itself built on
murobotics-ai/handumi-hw (Apache-2.0). It is used unchanged, from `cad/d405/source/handumi/`.

Changes and new work in this repository:

- The upstream parts were split, renamed, moved to the closed pose, and engraved with the PG badge.
- The handheld jaws got elastic finger-sleeve pads and a record-button pocket.
- New parts: the PG pod and wrist saddle, the parallel-jaw module, the D405 frame with its tilt knuckle, the
  electronics box, its lid and wrist saddle, the Quest 3S cradle, the OpenArm adapter and the ISO 50 flange plate.
- Also new: the kinematics, URDFs, assemblies, renders, tag sheet and documentation.

The OpenArm, UR, ABB and FANUC geometry used for checks and renders comes from the robot descriptions in the Neuracore
robots repository, and is not redistributed here.
