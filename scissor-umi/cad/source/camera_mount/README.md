# UR camera plate (the team's part)

`ur_camera_plate.step` is the plate that mounts the D405 on the robot's 2F-140. It is a 6 mm spacer between the UR
flange and the gripper's coupling, with a tab bent up 8° that carries the camera. It is used unchanged:

* the robot renders show it as it is;
* `head.py` measures its tab (pose and the 6 × M3 pattern), so the handheld's `tab_mast` puts an identical tab face at
  the same place in the gripper frame.
