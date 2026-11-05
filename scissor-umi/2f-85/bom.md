# Scissor-UMI · 2F-85: bill of materials

## Printed: handheld

STEP for every part is in [`STEP/`](STEP/). The material shown is a suggestion: PETG or PLA+ for the mechanism and
TPU 95A for the pad faces.

| Part | Qty | Material | Notes |
|---|---|---|---|
| `outer_link_2f85` | 2 | PETG | 2F-85 outer knuckle + outer finger, one piece. The same part on both jaws (turned 180°). |
| `inner_knuckle_2f85` | 2 | PETG | 2F-85 inner knuckle. The knuckle gear sits in its base fork, keyed with 2 × M3 through one prong. |
| `inner_finger_2f85` | 2 | PETG | 2F-85 inner finger. 2 × M3 counterbored holes for the pad core. |
| `pad_core_2f85` | 2 | PETG | The back 3.5 mm of the 2F-85 pad (20 × 35 mm), with two Ø1.6 pilots for the M2 screws. |
| `pad_skin_2f85` | 2 | TPU 95A | The 1.5 mm grip face, glued to the core. |
| `knuckle_gear_left`, `knuckle_gear_right` | 1 + 1 | PETG | 20-tooth sector gears (m 1.27), half a tooth out of phase with each other. |
| `handle_a_rh` / `handle_a_lh` | 1 | PETG | 40-tooth sector (meshes the left gear), hub with a hex end for the servo, inserts for the axle bolt and the stop screw. Right hand: the four-finger loop; left hand: the palm bar. |
| `handle_b_rh` / `handle_b_lh` | 1 | PETG | 40-tooth sector (meshes the right gear). Right hand: the palm bar; left hand: the four-finger loop. |
| `grip_tpu_a_*`, `grip_tpu_b_*` | 1 + 1 | TPU 95A | Soft inserts for the same hand's handles: the finger bar's inner face and the palm bar's outer face. Glue them in. |
| `servo_coupler` | 1 | PETG | Between handle A's hex end and the STS3215 horn (4 × M2). |
| `base_bottom` | 1 | PETG | Five standoff columns, pivot spacers, the stop slot, the button pod. |
| `base_top` | 1 | PETG | Pivot spacers; 4 × M3 inserts for the mast's foot. |
| `ebox_cover` | 1 | PETG | Box under the bottom plate: the STS3215 on its floor (4 × M2), Pico 2 standoffs, the hub bay, vents. |
| `tab_mast` | 1 | PETG / PA-CF | Copy of the camera plate's tab face, with the same six holes and pose; extends up behind the camera for the Quest arm; foot and gussets. |
| `d405_hood` | 1 | PETG | Open frame round the D405 on the tab. |
| `quest_arm_right` / `quest_arm_left` | 1 | PETG | Pad on the mast's back + split collar for the Touch Pro's handle, behind the camera. |

Print the `_rh` handles and inserts with `quest_arm_right` for the right-hand handheld, the `_lh` ones with
`quest_arm_left` for the left. Every other part is the same for both hands.

## Robot side

| Part | Qty | Notes |
|---|---|---|
| `ur_camera_plate` | 1 | Our existing camera plate, unchanged. The D405 mounts on its tab. Use M6 screws 7 mm longer than the coupling's own. |

## Bought

| Item | Qty (handheld) | Qty (robot) | Notes |
|---|---|---|---|
| Intel RealSense D405 | 1 | 1 | Fixed to the tab as on the robot plate. |
| Meta Quest Touch Pro controller | 1 | – | Tracking only. Right or left. |
| Robotiq 2F-85 + its UR coupling | – | 1 | |
| Feetech STS3215 servo | 1 | – | Jaw encoder, torque off. 4 × M2 × 8 into its bottom holes, horn with 4 × M2 × 6 to the coupler. |
| BNO085-class IMU breakout | 1 | – | In the box, as on HandUMI. |
| Raspberry Pi Pico 2 | 1 | – | Servo bus, IMU, button. USB to the hub. |
| USB 3 hub board (USB-C upstream) | 1 | – | One cable to the PC for the D405 and the Pico. |
| 74LVC1G125 or 1 kΩ resistor | 1 | – | Pico TX/RX onto the servo's single-wire bus. |
| 12 × 12 mm tactile switch + green cap | 1 | – | Record. In the pod under the base, at the index finger. |
| Ø4 × 50 mm steel dowel pins | 4 | – | Base pivots: press fit in both plates. |
| Ø4 × 25 mm steel dowel pins | 4 | – | Finger joints. |
| M4 × 35 bolt + M4 heat-set insert | 1 | – | Handle axle: through the top plate and handle B into handle A, with threadlocker. |
| M3 × 20 button head + M3 heat-set insert | 1 | – | Stop screw. |
| M3 × 10 + M3 heat-set inserts | 4 + 4 | – | Gear keys. |
| M2 × 8 self-tapping | 4 | – | Pads: from the inner finger's back into the pad core. |
| M3 × 10 + M3 heat-set inserts | 5 + 5 | – | Plates (standoffs). |
| M3 × 12 + M3 heat-set inserts | 4 + 4 | – | Mast foot to the top plate. |
| M3 × 12 self-tapping | 2 | – | Hood to the tab, from behind. |
| M3 × 16 + M3 heat-set inserts | 4 + 4 | – | Quest arm to the mast, from the mast's front into the pad. |
| M3 × 20 + M3 nut | 1 | – | Quest collar clamp. |
| M3 × 50 + M3 heat-set inserts | 4 + 4 | – | Electronics box to the bottom plate. |
| M6 screws, 7 mm longer than the coupling's own | – | 4 | Coupling + camera plate to the UR flange. |
