# Bill of materials: Insta360 X5 · Parallel

Per device. A bimanual kit is one right hand and one left hand. This is HandUMI's list with the D405 swapped for the
X5.

## Printed parts

| Part | Material | Qty |
|---|---|---|
| Everything in `STL/right/` or `STL/left/` except `button_cover_tpu` | blue / white PLA or PETG, about 200 g | 1 set |
| `x5_wrist_mount` (in both folders) | PETG or PLA | 1 |
| `button_cover_tpu` | TPU 95A: green (right hand, record) or red (left hand, cancel) | 1 |
| Tips from `STL/gripper_tips/` (Piper, Open-ENPIRE or Hand-E) | PLA / PETG (ENPIRE soft inserts in TPU) | 1 pair |

## Mechanics

| Part | Qty |
|---|---|
| MR63ZZ bearing, 3 × 6 × 2.5 mm | 4 |
| LM4UU linear bearing, 4 × 8 × 12 mm | 4 |
| Ø4 mm × 135 mm stainless rod | 2 |
| Feetech STS3215 servo (jaw-width encoder, torque off) | 1 |
| M4 × 30 bolt + nyloc nut (camera hinge) | 1 |
| 1/4"-20 × 3/8" low-head camera screw (X5 to the cradle) | 1 |
| M3 screws: 2× M3×20, 4× M3×12, 4× M3×15 (mechanism); 2× M3×8 (box to arm); 4× M3×6 (lid) | 16 |
| M3 heat-set inserts, Ø4.0 × 5 (box, lid, M3 tip pattern) | ~12 |
| M2 screws + nuts (mechanism, tips, Pico) | ~20 |
| Hook-and-loop straps, 15 cm | 4 |

## Electronics

| Part | Qty |
|---|---|
| Insta360 X5 | 1 |
| Raspberry Pi Pico 2 | 1 |
| BNO085 IMU breakout | 1 |
| USB hub board, USB-C upstream (fits the 38 × 42 × 14 mm bay; USB 2 is enough here) | 1 |
| 6 × 6 mm tactile switch, stiff (~250–320 gf) + ~30 cm thin 2-core wire | 1 |
| Short USB cable (Pico to hub) and one USB-C cable to the laptop | 1 set |
| 74LVC1G125 buffer or 1 kΩ resistor (servo bus) | 1 |

Wiring: [electronics.md](electronics.md).
