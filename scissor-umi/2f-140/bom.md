# Scissor-UMI · 2F-140: bill of materials

## Printed: handheld

STEP for every part is in [`STEP/`](STEP/). The material shown is a suggestion: PETG or PLA+ for the mechanism and
TPU 95A for the pads.

| Part | Qty | Material | Notes |
|---|---|---|---|
| `base_bottom` | 1 | PETG | Five standoff columns (M3 heat-set inserts in their tops). Also carries the AS5047P pocket, the stop slot and the button pod. Print upside down. |
| `base_top` | 1 | PETG | The mast to the head (4× M3 inserts at its top) and the floor of the electronics box. |
| `knuckle_gear_left` | 1 | PETG | Inner link + 20-tooth sector gear (m 1.27). Teeth in L1 and L2. |
| `knuckle_gear_right` | 1 | PETG | Teeth in L2 and L3, half a tooth out of phase with the left gear. |
| `outer_link` | 4 | PETG | Guide links: two per jaw, one above and one below. |
| `finger_left`, `finger_right` | 1 + 1 | PETG | The couplers. A T-slot runs along each for the pad. |
| `pad_tpu` | 2 | TPU 95A | The robot pad's size (27 × 65 × 7.5 mm) and position. Slide it into the finger's T-slot from the tip and add a drop of CA. |
| `handle_a` | 1 | PETG | L1. 40-tooth sector meshing the left gear, the loop on the −y side, the encoder magnet in its lower hub, and an M3 insert for the stop screw. |
| `handle_b` | 1 | PETG | L3. 40-tooth sector meshing the right gear, the loop on the +y side. |
| `ebox_cover` | 1 | PETG | Covers the Pico 2 and the USB hub on the top plate, with the USB-C cable exit on the +y side. |
| `head_right` (or `head_left`) | 1 | PETG | D405 tilt knuckle plus a copy of HandUMI's main-support arm end for the Quest ring. Shared with the robot. |

## Printed: robot twin (on the real Robotiq 2F-140)

| Part | Qty | Material | Notes |
|---|---|---|---|
| `robot_collar_iso50` | 1 | PETG / PA-CF | 6 mm spacer between the ISO 9409-1-50-4-M6 flange and the 2F-140's coupling. An arm runs up the gripper's +x side to the head interface. |
| `head_right` (or `head_left`) | 1 | PETG | The same part as on the handheld. |

## Reused from HandUMI (in [`handumi/hardware/STEP/`](../../handumi/hardware/STEP/))

| Part | Qty | Notes |
|---|---|---|
| `d405_wrist_mount` (the D405 cup) | 1 per build | Hangs on the head's knuckle with an M4 × 30 bolt and a nyloc nut; tilts 15–65°. The default tilt is 30°. |
| `right_controller_support` (mirror it for the left hand) | 1 per build | The Quest Touch Pro ring. It bolts to the head's arm end with 2× M3 × 25 + nuts, exactly as on HandUMI. |

## Bought

| Item | Qty (handheld) | Qty (robot) | Notes |
|---|---|---|---|
| Intel RealSense D405 | 1 | 1 | |
| Meta Quest Touch Pro controller | 1 | 1 | Tracking only. Right or left. |
| Robotiq 2F-140 + its coupling | – | 1 | |
| Ø4 × 32 mm steel dowel pins | 4 | – | At the outer and inner pivots: press fit in both plates, running fit (Ø4.2) in the links. |
| Ø4 × 20 mm steel dowel pins | 4 | – | Finger joints (B, E). |
| M4 × 35 bolt + M4 heat-set insert | 1 | – | Handle axle. Through the top plate and handle B into handle A's insert, with threadlocker. |
| M3 × 8 button head + M3 heat-set insert | 1 | – | Stop pin. Fits from below through the bottom plate's arc slot into handle A. The slot ends are the open and closed stops. |
| M3 × 10 + M3 heat-set inserts | 5 + 5 | – | Plates (standoffs). |
| M3 × 10 + M3 heat-set inserts | 4 + 4 | 4 + 4 | Head onto the mast or the collar. |
| M3 × 8 + M3 heat-set inserts | 4 + 4 | – | Ebox cover. |
| M3 × 25 + nuts | 2 | 2 | Quest ring to the head. |
| M4 × 30 + nyloc | 1 | 1 | D405 tilt hinge. |
| M6 screws, 6 mm longer than the coupling's own | – | 4 | Coupling + collar to the flange. |
| Ø6 × 10 dowel | – | 1 | ISO 9409 locating pin (the collar's 0° hole). |
| AS5047P breakout (or AS5600) + Ø6 × 2.5 diametric magnet | 1 | – | Handle angle (q = 2 × handle angle). |
| Raspberry Pi Pico 2 | 1 | – | Reads the encoder and the button. USB to the PC. |
| USB 3 hub board (2+ ports, USB-C upstream) | 1 | – | One cable to the PC for the D405 and the Pico. |
| 12 × 12 mm tactile switch + green cap | 1 | – | Record. In the pod under the base, at the index finger. |
