# Bill of materials

Quantities are **per hand**. A bimanual kit is two of each. Purchased parts follow KIWI's
[assembly guide](https://github.com/TToTMooN/KIWI_hardware/blob/main/chopsticks/chopsticks_assembly.pdf); the OpenArm
adapter rows are ours.

## Data gripper (handheld)

| Item | Qty | Notes |
|---|---|---|
| Insta360 X5 | 1 | 360° camera. Voice control on, auto power-off off, 4K 120 fps video, battery cover removed |
| Shoulder screw M3, Ø4 × 18 mm shoulder | 2 | Jaw pivots |
| 624 ball bearing (4 × 13 × 5) | 4 | Two per jaw |
| Socket screw M3 × 55 (button head recommended) | 2 | Camera frame to base, through the clip |
| Washer M3, 9 mm OD | 2 | |
| M3 hex nut | 2 | |
| Countersunk screw 1/4"-20 × 3/8" | 1 | Camera into the frame |
| Socket cap screw M2 × 4 | 3 | Locating screws, camera button |
| Heat-set insert M2 × 4 | 2 | Back of the base |
| Heat-set insert M3 × 4 | 2 | Underside of the base |
| Rubber band | 1 | Reopens the jaws, hooked through the centre cavity |
| Finger straps | 2 | One loop per jaw |
| Wrist strap + safety pin | 1 | Under the base |
| Foam pad | – | Under the base and jaws, to taste |
| 3M grip tape, 1" | – | To taste |
| Double-sided tape | – | For the ArUco tag |
| ArUco tag, DICT_4X4_50, 14 mm | 1 | `hardware/tags/gripper_tags_4x4_50_14mm.pdf`, one id per gripper |
| Super glue | – | TPU tips onto the jaws |

| Printed part | Material | Notes |
|---|---|---|
| `chopstick_base_dc` | PLA | |
| `chopstick_clip_dc` | PLA | |
| `x5_lite_frame` | PLA | |
| `jaw_left_dc`, `jaw_right` | PLA, 2 colours | Black, with the red dots as a second filament (painted in the 3MF) |
| `tip_tpu_left`, `tip_tpu_right` | TPU 95A | |
| `x5_button_cap`, `x5_button_shaft` | PLA | |

## Robot gripper on OpenArm

| Item | Qty | Notes |
|---|---|---|
| Insta360 X5 | 1 | |
| Feetech SM40BL servo set | 1 | Drives the left jaw through its rotor plate |
| Shoulder screw M3, Ø4 × 18 mm shoulder | 2 | |
| 624 ball bearing | 2 | Right (driven) jaw |
| Socket screw M3 × 55 | 4 | |
| Washer M3, 9 mm OD | 4 | |
| M3 hex nut | 4 | |
| Countersunk screw 1/4"-20 × 3/8" | 1 | |
| Socket screw M2 × 4 | 5 | |
| Socket screw M3 × 12 | 4 | Rotor plate to driver jaw |
| Heat-set insert M2 × 4 | 4 | |
| Heat-set insert M3 × 4 | 1 | |
| **Socket head screw M3 × 16** | **6** | **Adapter to the link7 flange (PCD 50)** |
| **Heat-set insert M3, Ø4.0 hole, ≤ 6 mm long** | **4** | **In the adapter front, for the camera frame** |
| **Socket head screw M3 × 10** | **4** | **Camera frame to the adapter, from inside the frame** |
| 3M grip tape, super glue, double-sided tape, ArUco tag | – | |

| Printed part | Material | Notes |
|---|---|---|
| `chopstick_base_sa` | PLA | |
| `chopstick_clip_sa`, `x5_clip_b` | PLA | |
| `x5_frame` | PLA | Reinforced frame. Drill its four rear-wall Ø2.8 holes out to Ø3.2 |
| `jaw_left_sa`, `jaw_right` | PLA, 2 colours | |
| `tip_tpu_left`, `tip_tpu_right` | TPU 95A | |
| `x5_button_cap`, `x5_button_shaft` | PLA | |
| **`openarm_adapter`** | **PETG or PLA, 100 % infill near the holes** | **Print back face down; no supports needed** |
