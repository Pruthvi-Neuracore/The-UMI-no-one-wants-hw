# Robotiq 2F-140 (renders and checks only)

`assembly.robotiq()` poses Robotiq's 2F-140 visual meshes (`robotiq_arg2f_coupling.stl`, `robotiq_arg2f_base_link.stl`,
`robotiq_arg2f_140_{outer_knuckle,outer_finger,inner_knuckle,inner_finger}.stl`, in metres) to draw the robot twin.
They come from Robotiq's ROS description (`robotiq_2f_140_gripper_visualization`, ros-industrial/robotiq). They are not
copied into this repository: put the STLs in this folder to rebuild the renders and the clearance checks.

The kinematics in `kin.py` (pivots, link frames, pad offset, range) come from the same URDF and need no meshes.
