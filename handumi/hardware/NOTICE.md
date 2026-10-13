# Notice

The base mechanism in `hardware/` is derived from [murobotics-ai/handumi-hw](https://github.com/murobotics-ai/handumi-hw)
at commit `fc7462a`, licensed under Apache-2.0 (see `hardware/LICENSE`). The scripts that generate the parts, and the originals
they start from, are on the repository's `cad` branch.

`hardware/STL/gripper_tips/Open-ENPIRE/` comes from
[pgeedh/Open-ENPIRE-Gripper](https://github.com/pgeedh/Open-ENPIRE-Gripper) (Apache-2.0, license in `Open-ENPIRE/LICENSE`). The device jaws are the I2RT finger with its mount tab replaced by
a flange plate. `Open-ENPIRE/robot/OpenArm_Hard.stl` is the project's OpenArm finger, converted to millimetres.

`hardware/{STL,STEP}/gripper_tips/AgileX-Piper/` are unmodified from handumi-hw.

`hardware/STL/gripper_tips/Robotiq-Hand-E/` is built from the Robotiq Hand-E finger visual mesh
from the Robotiq Hand-E robot description, with a flange plate added.

README robot renders (`docs/img/robot_*.png`) are made from the ABB CRB 15000, FANUC CRX-5iA and UR5 dual-arm
descriptions with Robotiq Hand-E grippers in the Neuracore robots repository, and from the AgileX Piper and OpenArm models
in [murobotics-ai/handumi-sw](https://github.com/murobotics-ai/handumi-sw) `assets/` (Apache-2.0). The models are not
redistributed here.

Changes in this repository:

- Renamed the part folders to `right/` and `left/` (STEP and STL).
- `main_support`: restyled `fisheye_camera_main_support` (blended outline, lightening slots, rounded edges), with a new
  braced camera post, Ø12 / M4 hinge and two M3 holes for the electronics box.
- `end_cover`: `main_support_cover_plate` restyled to match.
- `electronics_box` and `electronics_lid`: a new enclosure for a Raspberry Pi Pico 2, an IMU and a USB 3 hub, replacing
  the servo controller support and cover.
- Finger links (`*_thumb_link`, `*_index_middle_finger_link`): finger sleeves, a universal tip flange (M2/M3/M4
  patterns) and a record-button pocket with a TPU cover (`button_cover_tpu`).
- `d405_wrist_mount`: a D405 cup with Ø12 / M4 hinge lugs and gussets, replacing `camera_mount`.
- Every printed part carries a small engraved PG logo; the gripper tips do not.
- The original bill of materials, hardware README and the gripper tips for other robots were removed; `docs/bom.md` is new.
