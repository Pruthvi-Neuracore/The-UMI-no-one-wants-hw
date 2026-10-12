# Notice

PG Chopstick's jaws, bases, clips and Insta360 X5 frames are derived from TToTMooN/KIWI_hardware (commit `626ad7b`),
which is licensed under Apache-2.0. The licence text is in `LICENSE`. The unmodified source files are in
`cad/source/upstream/` (`data_gripper.step`, `robot_gripper.step`, `print.3mf`).

Changes in this repository:

- The parts were split out of the assemblies, renamed, and moved to the closed jaw pose.
- Every visible part carries an engraved PG badge (`cad/restyle.py`).
- New parts: `pg_pod`, `pg_pod_lid`, `pg_pod_button`, `d405_frame`, `openarm_adapter` and `iso_flange_plate`.
- Also new: the kinematics, URDFs, assemblies, renders, tag sheet, documentation and the `retarget/` package.

The OpenArm link7 and FANUC CRX geometry used for checks and renders comes from the robot descriptions in the
Neuracore robots repository, and is not redistributed here.
