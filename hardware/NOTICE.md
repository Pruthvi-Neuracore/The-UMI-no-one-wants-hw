# Notice

The base mechanism in `hardware/` is derived from [murobotics-ai/handumi-hw](https://github.com/murobotics-ai/handumi-hw)
at commit `fc7462a`, licensed under Apache-2.0 (see `hardware/LICENSE`). The original parts the build scripts start from
are kept in `cad/source/`. `hardware/reference/STS3215_03a.step` is from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) (Apache-2.0).

Gripper tips (`hardware/{STL,STEP}/gripper_tips/`):

- `AgileX-Piper/`: unmodified from handumi-hw.
- `Robotiq-Hand-E/Hand-E-Finger`: built from the Robotiq Hand-E finger visual mesh (`cad/source/hand_e/finger.dae`, from
  the Robotiq Hand-E robot description), remeshed watertight, with the Hand-E's own finger mount from Robotiq's drawing
  (Hand-E manual, 5.1.4) and its rack groove filled (`cad/hande_tip.py`). `Hand-E-Rack-Shim` is new.
- `Open-ENPIRE/`: from [pgeedh/Open-ENPIRE-Gripper](https://github.com/pgeedh/Open-ENPIRE-Gripper) (Apache-2.0, license
  copied to `cad/source/enpire/`), `grippers-stl/enpire_i2rt_yam_stl`: Wenli Xiao's ENPIRE finger for the I2RT YAM,
  unmodified, with its mirror image for the second jaw (`cad/enpire_tip.py`). `Open-ENPIRE/robot/OpenArm_Hard.stl` is
  the project's OpenArm finger, converted to millimetres.
- `OpenArm/`: the hard-shell fingers from this repository's `openarm-hardshell-fingers` branch (Apache-2.0, copied to
  `cad/source/openarm/`), with the rib behind the base plate cut off (`cad/openarm_tip.py`). Their base re-models
  OpenArm v1's printed finger base from its description; OpenArm's description is not redistributed here.

The robot finger interfaces in `cad/finger_interfaces.py` are measured from the Robotiq Hand-E manual (fingers and
fingertip holder drawings), the AgileX `piper_description` finger mesh (`link7`), the
[i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt) YAM tip mesh (MIT) and the Open-ENPIRE YAM finger, and
OpenArm v1's finger base.

README robot renders (`docs/img/robot_*.png`) are made from the ABB CRB 15000, FANUC CRX-5iA and UR5 dual-arm
descriptions with Robotiq Hand-E grippers in the Neuracore robots repository, from the AgileX Piper and OpenArm models
in [murobotics-ai/handumi-sw](https://github.com/murobotics-ai/handumi-sw) `assets/` (Apache-2.0), and from the I2RT YAM
model in i2rt-robotics/i2rt (MIT). `robot_openarm.png` is the `openarm-hardshell-fingers` branch's render of OpenArm v1.
The models are not redistributed here.

Changes in this repository:

- Renamed the part folders to `right/` and `left/` (STEP and STL).
- `main_support`: restyled `fisheye_camera_main_support` (blended outline, lightening slots, rounded edges), with a new
  braced camera post ending in a fixed tab for the D405, and two M3 holes for the electronics box. Source:
  `cad/restyle.py`, `cad/d405_mount.py`.
- `end_cover`: `main_support_cover_plate` restyled to match.
- `electronics_box` and `electronics_lid`: a new enclosure for a Raspberry Pi Pico 2, an IMU and a USB 3 hub, replacing
  the servo controller support and cover. Source: `cad/electronics_box.py`.
- Finger links (`*_thumb_link`, `*_index_middle_finger_link`): finger sleeves, a flange carrying the AgileX Piper,
  Robotiq Hand-E, I2RT YAM and OpenArm v1 finger mounts, and a record-button pocket with a TPU cover
  (`button_cover_tpu`). Source: `cad/finger_link.py`, `cad/finger_interfaces.py`.
- `d405_hood`: an open frame round the D405 on the main support's tab, replacing `camera_mount`. Source:
  `cad/d405_mount.py`.
- Every printed part carries a small engraved PG logo (`cad/watermark.py`); the gripper tips do not.
- The original bill of materials, hardware README and the gripper tips for other robots were removed; `docs/bom.md` is new.
