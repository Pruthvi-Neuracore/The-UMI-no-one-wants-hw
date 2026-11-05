# Notice

* **Robotiq 2F-85 jaw geometry.** The parts `outer_link_2f85`, `inner_knuckle_2f85`, `inner_finger_2f85`,
  `pad_core_2f85` and `pad_skin_2f85` are derived from Robotiq's 2F-85 gripper description for ROS: the visual
  meshes `robotiq_arg2f_85_outer_knuckle`, `_outer_finger`, `_inner_knuckle`, `_inner_finger`, the URDF's joint
  frames and its `inner_finger_pad` box. Robotiq and 2F-85 are trademarks of Robotiq Inc.; this project is not
  affiliated with or endorsed by Robotiq. Check the licence of the Robotiq description you build from before
  redistributing derived parts. The source meshes themselves are not included (see
  `scissor-umi/cad/source/robotiq_2f_85/README.md`).
* **Feetech STS3215 model.** From TheRobotStudio/SO-ARM100 (Apache-2.0). Used for renders, checks and hole positions.
* **UR camera plate.** Our own part (`ur_camera_plate`), used unchanged.
* **Meta Quest Touch Pro.** Meta's model is used for renders and to measure the handle for the collar. It is not
  included.
