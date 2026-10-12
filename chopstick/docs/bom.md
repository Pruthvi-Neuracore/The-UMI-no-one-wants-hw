# Bill of materials

Quantities are **per gripper**. A bimanual kit is two of each.

## Filament (PG blue and white)

| Colour | Material | Parts |
|---|---|---|
| **Blue** | PLA (PETG for the robot plates) | bases, clips, X5 frames, `d405_frame`, `openarm_adapter`, `iso_flange_plate`, `x5_button_*` |
| **White** | PLA | jaws (`jaw_left_dc`, `jaw_left_sa`, `jaw_right`), `pg_pod`, `pg_pod_lid` |
| **Red** | PLA, second filament | the dots on the jaws: add `hardware/STL/<jaw>_dots.stl` as a part or modifier on the jaw and give it red. Red on white is what the vision tracks |
| **White** | TPU 95A | `tip_tpu_left`, `tip_tpu_right`, `pg_pod_button` (or blue TPU for the button) |

## Handheld, Insta360 X5 (`data`)

| Item | Qty | Notes |
|---|---|---|
| Insta360 X5 | 1 | Voice control on, auto power-off off, 4K 120 fps, battery cover removed |
| 624 ball bearing (4 × 13 × 5) | 4 | Two per jaw |
| Shoulder screw M3, Ø4 × 18 mm shoulder | 2 | Jaw pivots |
| Socket screw M3 × 55 (button head) + M3 nut + M3 × 9 mm OD washer | 2 | Camera frame to base, through the clip |
| Heat-set insert M2 × 4 | 2 | Back of the base |
| Heat-set insert M3 × 4 | 2 | Underside of the base |
| Socket cap screw M2 × 4 | 3 | Locating screws, camera button |
| **Button head screw 1/4"-20 × 5/8"** | **1** | **Pod + frame + camera (replaces the 3/8" countersunk)** |
| Rubber band | 1 | Reopens the jaws |
| Finger straps | 2 | One loop per jaw |
| Wrist strap, foam pad, grip tape | – | To taste |
| ArUco tag, DICT_4X4_50, 14 mm | 1 | `hardware/tags/pg_tags_4x4_50_14mm.pdf`, one id per gripper |
| Super glue | – | TPU tips onto the jaws |

### PG pod electronics

| Item | Qty | Notes |
|---|---|---|
| Seeed XIAO ESP32-S3 | 1 | 21 × 17.5 mm, USB-C, LiPo charger, Wi-Fi/BLE |
| IMU breakout (ICM-42688-P, LSM6DSOX or BNO085 class) | 1 | Up to about 26 × 18 mm, I²C. Tape or epoxy it to the pod's top wall |
| 1S LiPo, 402535 (300 mAh, 4 × 25 × 35 mm) with protection | 1 | JST-PH or soldered to the XIAO's BAT pads |
| Slide switch SS12D00 (or similar 8.5 × 3.6 mm) | 1 | In the battery lead |
| Tactile switch 6 × 6 mm, about 160–250 gf | 1 | On the post behind the button |
| M2 × 6 self-tapping screw | 4 | Lid |
| Thin silicone wire, double-sided foam tape | – | |

## Handheld, RealSense D405 (`data_d405`)

As the X5 build, without the X5, the pod or the 1/4"-20 screw, plus:

| Item | Qty | Notes |
|---|---|---|
| Intel RealSense D405 | 1 | |
| M3 × 6 socket screw | 2 | Camera to the cup floor (its back holes, 20 mm apart) |
| USB-C to USB-A cable, USB 3 | 1 | To the laptop |

## Robot gripper (`openarm`, `iso50`, and their `_d405` versions)

| Item | Qty | Notes |
|---|---|---|
| Insta360 X5 **or** RealSense D405 | 1 | Same camera as the demos |
| Feetech SM40BL servo set | 1 | Drives the left jaw through its rotor plate |
| 624 ball bearing | 2 | Driven (right) jaw |
| Shoulder screw M3, Ø4 × 18 mm shoulder | 2 | |
| Socket screw M3 × 55 + nut + 9 mm OD washer | 4 | |
| Socket screw M3 × 12 | 4 | Rotor plate to the driver jaw |
| Heat-set insert M2 × 4 / M3 × 4 | 4 / 1 | Base |
| Socket screw M2 × 4 | 5 | |
| Heat-set insert M3, Ø4.0 hole, up to 6 mm long | 4 | In the mount plate's front, for the camera frame |
| Socket head screw M3 × 10 | 4 | Camera frame to the plate, from inside the frame |
| **OpenArm:** socket head screw M3 × 16 | 6 | `openarm_adapter` to link7 (PCD 50) |
| **ISO 50:** socket head screw M6 × 14, dowel pin Ø6 × 10 | 4, 1 | `iso_flange_plate` to the tool flange |
