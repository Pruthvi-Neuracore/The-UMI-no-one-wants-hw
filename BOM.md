# Bill of materials

Everything needed to build the UMI no one wants. Quantities are given **per device** (one hand) and **per pair** (one
right-hand and one left-hand device). The same list is in [`bom.csv`](bom.csv) for ordering.

## Printed parts

Print one set from `hardware/STL/right/` for the right hand and one from `hardware/STL/left/` for the left hand.
STEP files for every part are in `hardware/STEP/`. Roughly 200 g of filament per device.

| Part | File | Material | Per device | Per pair |
|---|---|---|---|---|
| Main support (T-plate with camera post) | `main_support.stl` | PLA / PETG | 1 | 2 |
| End cover | `end_cover.stl` | PLA / PETG | 1 | 2 |
| D405 wrist mount (cup on the M4 hinge) | `d405_wrist_mount.stl` | PLA / PETG | 1 | 2 |
| Electronics box | `electronics_box.stl` | PLA / PETG | 1 | 2 |
| Electronics lid | `electronics_lid.stl` | PLA / PETG | 1 | 2 |
| Thumb link | `right_thumb_link.stl` / `left_thumb_link.stl` | PLA / PETG | 1 | 2 |
| Index/middle finger link (record button pocket) | `right_index_middle_finger_link.stl` / `left_index_middle_finger_link.stl` | PLA / PETG | 1 | 2 |
| Crank mechanism plate | `crank_mechanism_plate.stl` | PLA / PETG | 1 | 2 |
| Connecting link 1 | `connecting_link_1.stl` | PLA / PETG | 1 | 2 |
| Connecting link 2 | `connecting_link_2.stl` | PLA / PETG | 1 | 2 |
| Hand support base | `hand_support_base.stl` | PLA / PETG | 1 | 2 |
| Controller support | `right_controller_support.stl` / `left_controller_support.stl` | PLA / PETG | 1 | 2 |
| Record-button cover | `button_cover_tpu.stl` | **TPU 95A** | 1 | 2 |

### Gripper tips (pick one set)

From `hardware/STL/gripper_tips/`, matched to the robot you'll deploy on.

| Tip set | Files | Material | Per device | Per pair |
|---|---|---|---|---|
| AgileX Piper | `AgileX-Piper/Agile-X-Piper-LEFT-Gripper.stl` + `-RIGHT-Gripper.stl` | PLA / PETG | 1 + 1 | 2 + 2 |
| Open-ENPIRE (OpenArm) | `Open-ENPIRE/ENPIRE-LEFT-Jaw.stl` + `ENPIRE-RIGHT-Jaw.stl` | PLA / PETG | 1 + 1 | 2 + 2 |
| Open-ENPIRE soft inserts | `Open-ENPIRE/ENPIRE-LEFT-Soft-Insert.stl` + `ENPIRE-RIGHT-Soft-Insert.stl` | TPU 95A | 1 + 1 | 2 + 2 |
| Robotiq Hand-E (ABB, FANUC, UR) | `Robotiq-Hand-E/Hand-E-Finger.stl` (index jaw is the same part turned 180°) | PLA / PETG | 2 | 4 |

## Mechanical hardware

| Part | Used for | Per device | Per pair |
|---|---|---|---|
| MR63ZZ bearing, 3 × 6 × 2.5 mm | Crank and links | 4 | 8 |
| LM4UU linear bearing, 4 × 8 × 12 mm | Finger links on the rods | 4 | 8 |
| Ø4 mm × 135 mm stainless rod | Linear guide | 2 | 4 |
| Feetech STS3215 servo | Jaw-width encoder (torque off) | 1 | 2 |
| Hook-and-loop strap, 15 cm | Hand and finger straps | 4 | 8 |

## Fasteners

| Part | Used for | Per device | Per pair |
|---|---|---|---|
| M4×30 bolt + M4 nyloc nut | Camera hinge | 1 + 1 | 2 + 2 |
| M3×5 screw | D405 into its cup (rear holes, 4 mm max thread depth) | 2 | 4 |
| M3×20 screw | Mechanism | 2 | 4 |
| M3×12 screw | Mechanism | 4 | 8 |
| M3×15 screw | Mechanism | 4 | 8 |
| M3×8 screw | Electronics box to the arm (counterbored) | 2 | 4 |
| M3×6 screw | Lid to the box | 4 | 8 |
| M3 heat-set insert, Ø4.0 × 5 | Box (2) and lid (4) | 6 | 12 |
| M2×4 self-tapping screw | Pico to the lid standoffs | 4 | 8 |
| M2 self-tapping screw | Tips to the flange (8 × 12 mm pattern) | 8 | 16 |
| M2 screws + nuts, assorted | Mechanism | ~12 | ~24 |

Optional, only for tips that use the larger flange patterns:

| Part | Used for | Per device | Per pair |
|---|---|---|---|
| M3 heat-set insert, Ø4.0 × 5 | M3 tip pattern (6 per finger link) | 12 | 24 |
| M4 heat-set insert, Ø5.6 × 7 | M4 tip pattern (2 per finger link) | 4 | 8 |

## Electronics

Wiring and power are in [docs/electronics.md](docs/electronics.md).

| Part | Notes | Per device | Per pair |
|---|---|---|---|
| Intel RealSense D405 | Wrist depth camera | 1 | 2 |
| Raspberry Pi Pico 2 | IMU, button and servo bus | 1 | 2 |
| BNO085 IMU breakout | In the box floor pocket (28 × 24 mm); check your board's outline | 1 | 2 |
| USB 3 hub board, USB-C upstream | Bay is 38 × 42 mm, up to ~14 mm tall (VL817-type 4-port); check the size | 1 | 2 |
| 6 × 6 mm tactile switch, stiff (~250–320 gf) | Record start / stop | 1 | 2 |
| 74LVC1G125 buffer, or a 1 kΩ resistor | Pico TX/RX onto the servo's single-wire bus | 1 | 2 |
| Thin flexible 2-core wire, ~30 cm | Button to the Pico | 1 | 2 |

## Cables

| Part | Per device | Per pair |
|---|---|---|
| USB-C to USB-A, USB 3, ~15 cm (D405 to hub) | 1 | 2 |
| Micro-USB to USB-A, ~10 cm, right-angle helps (Pico to hub) | 1 | 2 |
| USB-C cable to the laptop, USB 3 | 1 | 2 |

## Consumables and tools

- Double-sided tape or epoxy (IMU in its pocket)
- Solder, heat-shrink, a few short jumper wires (IMU, servo bus, button)
- Soldering iron with a heat-set insert tip
- M2, M3 and M4 hex keys / drivers
