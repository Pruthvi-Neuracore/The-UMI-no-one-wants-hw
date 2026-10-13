# OpenArm hard-shell fingers

<p align="center"><img src="docs/img/openarm_open.png" alt="Two fingers on OpenArm's link7, half open: OpenArm's own finger base flowing into chopstick-style jaws, a white hard spine over a black scalloped soft body, with chisel tips" width="560"></p>

Printed fingers for **OpenArm v1's parallel gripper**, in place of its stock fingers. Each finger is one hard part and
one soft part sharing one outer surface, so it reads as one seamless form.

<p align="center"><img src="docs/img/finger.png" alt="One finger: the hard part with the soft part's seat and tongue slot (left), the soft part (middle) and the two together (right)" width="820"></p>
<p align="center"><img src="docs/img/finger_side.png" alt="Side view: the stock base flowing into the faceted jaw, the scalloped soft body below the hard spine, the chisel nose with its hard tip" width="700"></p>

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
- **Flow:** up to midway (35 mm), the finger follows the stock finger's own sections. A smooth loft then carries it into
  the jaw.
- **Jaw, in the PG Chopstick's design language:**
  - flat facets and lightly rounded long edges;
  - in plan, straight sides that chamfer to a point;
  - in side view, a flat gripping edge and a straight back that breaks into a **chisel nose**;
  - **three big scallops** along the gripping edge, as on the chopstick's TPU tip.
- **Soft part (TPU 95A):** the jaw's lower body below a flat parting line, flush with the hard part's faces, so the
  finger reads as one form: soft below, hard spine above, as on the chopstick.
  - The parting line steps down along the chisel, so the hard spine stays whole into the tip.
  - A tongue on the soft part's back keys into a slot in the hard part. Glue it in.
- **Hard part (PETG):** the base, the flow, the spine and a 1.5 mm **hard tip** at the end of the chisel. Its gripping
  face sits 0.6 mm behind the soft face, so the soft part touches first.
- **Closing:** fully closed, the soft faces are about 0.2 mm apart, and the hard faces 1.4 mm. Fully open, the soft
  faces are about 88 mm apart.

## Strength and weight

`cad/check_strength.py` slices the hard part every 5 mm and checks its bending strength against **a 100 N knock at the
fingertip**, with a 1.5 safety factor on PETG's ~45 MPa:
- Every section passes, with at least 1.3× what's needed.
- The tightest is just behind the chisel tip, where the bending is smallest.
- A normal grasp is a few tens of newtons.

The hard part is 56 cm³ (about 70 g in PETG) and the soft part 8 cm³.

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
