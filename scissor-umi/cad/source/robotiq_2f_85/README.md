# Robotiq 2F-85 (jaw replicas, renders and checks)

`replica.py` turns Robotiq's 2F-85 visual meshes into the handheld's printed jaw parts, and `assembly.robotiq()` poses
them (with `robotiq_arg2f_base_link.stl` and `robotiq_arg2f_coupling.stl`) to draw the robot twin. They come from
Robotiq's ROS description (`robotiq_arg2f_85_*` visual meshes, ros-industrial/robotiq), converted to STL in mm with their
vertices welded so each part is one closed solid. They are not copied into this repository: put the STLs in this folder
to rebuild the jaw parts, the renders and the checks.

The kinematics in `kin.py` (pivots, link offsets, pad, range) come from the same URDF (`robotiq_arg2f_85_model`).
