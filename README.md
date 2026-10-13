# OpenArm hard-shell fingers

<p align="center"><img src="docs/img/openarm_open.png" alt="Two fingers on OpenArm's link7, half open: OpenArm's own finger base flowing into a longer finger, with a black soft part flush in each gripping face" width="560"></p>

Printed fingers for **OpenArm v1's parallel gripper**, in place of its stock fingers. Each finger is one hard part and
one soft part sharing one outer surface, so it reads as one seamless form.

<p align="center"><img src="docs/img/finger.png" alt="One finger: the hard part with the soft part's seat and tongue slot (left), the soft part (middle) and the two together (right)" width="820"></p>
<p align="center"><img src="docs/img/finger_side.png" alt="Side view: the stock base flaring into the finger; the soft part flush along the gripping face; the hard tip" width="700"></p>

## What you print

| File | Material | Per gripper |
|---|---|---|
| `hardware/STL/hardshell_finger_left.stl`, `hardshell_finger_right.stl` | PETG (or PLA) | 1 + 1 |
| `hardware/STL/hardshell_tpu_left.stl`, `hardshell_tpu_right.stl` | TPU 95A | 1 + 1 |

The STEP files are in `hardware/STEP/`. The right parts are the mirror images of the left ones, as OpenArm's own
fingers are.

## How it's made

- **Base: OpenArm's own.** It's re-modelled from OpenArm v1's finger: the mounting plate with its four M2.5 holes, and
  the keying features behind it (centre block, two prongs, rib and the round boss). So it fits the carriage exactly
  as the stock finger does.
- **Flow:** up to midway (35 mm), the finger follows the stock finger's own sections. From there one smooth loft through
  rounded sections carries it into a longer finger, with a flat gripping plane and a rounded tip.
- **Soft part (TPU 95A):** the front 6 mm of that same section along the gripping length, so its sides are flush with
  the hard part's.
  - Its face has three shallow transverse grooves, like the PG Chopstick's pad.
  - A tongue on its back keys into a slot in the hard part. Glue it in.
- **Hard part (PETG):** everything else, including a 1.5 mm **hard tip** over the soft part's end. The tip's face sits
  0.6 mm behind the soft face, so the soft part touches first, and a thin object at the very tip also bears on the
  hard tip.
- **Closing:** fully closed, the soft faces are about 0.2 mm apart. Fully open, they are about 88 mm apart.

## Strength and weight

`cad/check_strength.py` slices the hard part every 5 mm and checks its bending strength against **a 100 N knock at the
fingertip**, with a 1.5 safety factor on PETG's ~45 MPa. The weakest section has **1.9×** what's needed, and a normal
grasp is a few tens of newtons. The hard part is 53 cm³ (about 67 g in PETG) and the soft part 9 cm³.

## The screws

The four M2.5 screws are the stock finger's, put in from the finger side:
- **Upper two:** each has a straight access bore that comes out on the finger's sloping back, so you reach them
  straight on, also with the soft part glued in.
- **Lower two:** these sit just under the base's gripping face, in open channels. Reach them with a ball-end hex key
  from the gripping side.

<p align="center"><img src="docs/img/openarm_closed.png" alt="The fingers closed on OpenArm's link7" width="560"></p>

## Checked

On OpenArm v1's description, closed, half open and fully open:
- The fingers never touch each other: neither the hard parts, nor the soft parts, nor one finger's hard part with the
  other's soft part.
- The hard and soft parts don't overlap, and each prints as one solid.
- The STL files are watertight.

## Print and fit

1. **Hard part:** print it standing on the base. Use 4 walls and 30% infill, with supports under the base's keying
   features.
2. **Soft part:** print in TPU 95A, lying on its back, with the tongue up.
3. **Fit:**
   1. Take off the stock finger.
   2. Screw the new hard part on with the same four screws.
   3. Glue the soft part in with flexible CA, or a TPU-rated contact adhesive.

## Files

| Path | What |
|---|---|
| `hardware/STEP/`, `hardware/STL/` | The four printable parts |
| `cad/openarm_hardshell.py` | The generator (build123d). `python openarm_hardshell.py --openarm <handumi-sw assets>` also renders and checks on OpenArm |
| `cad/check_strength.py` | The bending check along the hard part |
| `docs/img/` | Renders |

Licences and sources: [NOTICE.md](NOTICE.md).
