# Bill of materials

Quantities are for a **pair** (one right-hand and one left-hand device).

## Printed parts

| Part | Material | Qty |
|---|---|---|
| Everything in `hardware/STL/right/` and `hardware/STL/left/` except the button cover | PLA or PETG, ~200 g per device | 1 set each |
| `button_cover_tpu.stl` | TPU 95A | 2 |
| Tips from `hardware/STL/gripper_tips/` (Piper, Hand-E, Open-ENPIRE or OpenArm), or the robot's own fingers | PLA / PETG (ENPIRE and OpenArm soft parts in TPU 95A) | 2 pairs |
| `Hand-E-Rack-Shim.stl`, only to mount real Robotiq Hand-E fingers | PLA / PETG | 4 |
| Robot side: `hardware/STL/robot/hande_camera_plate.stl`, one per Hand-E arm, with a D405, `d405_hood.stl`, 2× M3×6, 2× M3×8 and 4× M6 flange screws 6 mm longer than the Hand-E's own | PETG-CF / PA-CF | 1 per arm |

## Mechanics

| Part | Qty |
|---|---|
| MR63ZZ bearing, 3 × 6 × 2.5 mm | 8 |
| LM4UU linear bearing, 4 × 8 × 12 mm | 8 |
| Ø4 mm × 135 mm stainless rod | 4 |
| Feetech STS3215 servo (used as the jaw-width encoder, torque off) | 2 |
| M3×6 screws (D405 to the main support's tab, from behind) | 4 |
| M3×8 screws (D405 hood, from behind the tab) | 4 |
| M3 screws: 4× M3×20, 8× M3×12, 8× M3×15 (mechanism); 4× M3×8 (box to arm); 8× M3×6 (lid) | 32 |
| M3 heat-set inserts, Ø4.0 × 5 (box, lid) | ~16 |
| M2 screws + nuts (mechanism, Pico) | ~24 |
| Tip screws, per pair of devices, for the tip set you fit: Piper 8× M2×10; Hand-E 4× M3×10 (+ 4× Ø2×6 dowel pins for real Robotiq fingers); YAM 4× M4×10; OpenArm 8× M2.5×8 | 1 set |
| Hook-and-loop straps, 15 cm | 8 |

## Electronics

| Part | Qty |
|---|---|
| Intel RealSense D405 | 2 |
| Raspberry Pi Pico 2 | 2 |
| BNO085 IMU breakout | 2 |
| USB 3 hub board, USB-C upstream (fits a 38 × 42 × 14 mm bay) | 2 |
| 6 × 6 mm tactile switch, stiff (~250–320 gf) | 2 |
| Short USB cables (D405 to hub, Pico to hub) and one USB-C cable to the laptop per device | 2 sets |
| 74LVC1G125 buffer or 1 kΩ resistor (servo bus) | 2 |
| Thin 2-core wire, ~30 cm | 2 |

Wiring and power are in [electronics.md](electronics.md).
