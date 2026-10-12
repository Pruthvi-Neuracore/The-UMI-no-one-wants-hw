# Notice

The base mechanism in `hardware/` is derived from [murobotics-ai/handumi-hw](https://github.com/murobotics-ai/handumi-hw)
at commit `fc7462a`, licensed under Apache-2.0 (see `hardware/LICENSE`). The original parts the build scripts start from
are kept in `cad/source/`. `hardware/reference/STS3215_03a.step` is from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0).

`hardware/STL/gripper_tips/Open-ENPIRE/` comes from
[pgeedh/Open-ENPIRE-Gripper](https://github.com/pgeedh/Open-ENPIRE-Gripper) (Apache-2.0, license copied to
`cad/source/enpire/`). The device jaws are the I2RT finger with its mount tab replaced by a flange plate
(`cad/enpire_tip.py`). `Open-ENPIRE/robot/OpenArm_Hard.stl` is the project's OpenArm finger, converted to millimetres.

`hardware/{STL,STEP}/gripper_tips/AgileX-Piper/` are unmodified from handumi-hw.

`hardware/STL/gripper_tips/Robotiq-Hand-E/` is built from the Robotiq Hand-E finger visual mesh
(`cad/source/hand_e/finger.dae`, from the Robotiq Hand-E robot description) with a flange plate added (`cad/hande_tip.py`).

README robot renders (`docs/img/robot_*.png`) are made from the ABB CRB 15000, FANUC CRX-5iA and UR5 dual-arm
descriptions with Robotiq Hand-E grippers in the Neuracore robots repository, and from the AgileX Piper and OpenArm models
in [murobotics-ai/handumi-sw](https://github.com/murobotics-ai/handumi-sw) `assets/` (Apache-2.0). The models are not
redistributed here.

Changes in this repository:

- Renamed the part folders to `right/` and `left/` (STEP and STL).
- `main_support`: restyled `fisheye_camera_main_support` (blended outline, lightening slots, rounded edges), with a new
  braced camera post, Ø12 / M4 hinge and two M3 holes for the electronics box. Source: `cad/restyle.py`.
- `end_cover`: `main_support_cover_plate` restyled to match.
- `electronics_box` and `electronics_lid`: a new enclosure for a Raspberry Pi Pico 2, an IMU and a USB 3 hub, replacing
  the servo controller support and cover. Source: `cad/electronics_box.py`.
- Finger links (`*_thumb_link`, `*_index_middle_finger_link`): finger sleeves, a universal tip flange (M2/M3/M4
  patterns) and a record-button pocket with a TPU cover (`button_cover_tpu`). Source: `cad/finger_link.py`.
- `d405_wrist_mount`: a D405 cup with Ø12 / M4 hinge lugs and gussets, replacing `camera_mount`. Source: `cad/d405_wrist_mount.py`.
- Every printed part carries a small engraved PG logo (`cad/watermark.py`); the gripper tips do not.
- The original bill of materials, hardware README and the gripper tips for other robots were removed; `docs/bom.md` is new.
