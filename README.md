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

## Spoon fingers

<p align="center"><img src="docs/img/spoon_held.png" alt="The spoon fingers on OpenArm's link7 holding the 50 cm spoon by its handle near the neck" width="700"></p>

A second pair, `hardware/spoon/`, made for holding a **large spoon or fork by its flat, wide handle** without it
slipping or turning. It is designed around a 50 cm spoon whose handle is 7.7 mm thick and 22–43 mm wide, with rounded
edges.

**Why flat pads aren't enough:** picked from above, the fingers close across the handle's width. The spoon's weight,
about 100 mm out from the grasp, then tries to turn it about the closing axis. Flat pads resist that only by friction.
At a 20 N grip that's about 0.04 N·m, far below what a heavy spoon needs, so it tips.

**What these fingers do instead:** they hold it by **form closure**.
- Each fingertip has a **slot** that takes the handle's edge: 8.4 mm high for the 7.7 mm handle, 4.5 mm deep,
  round-bottomed like the edge, with lead-in chamfers.
- The slot is lined with 1.2 mm of TPU, one piece with the soft gripping face above it.
- When the spoon tries to tip, the handle bears on the slot's lips across the whole 31 mm width of the wide,
  square-ended jaw. The moment is carried by the lips, not by friction.

<p align="center"><img src="docs/img/spoon_slots.png" alt="Looking down the handle: its rounded edges seated in the two slots, wrapped by the black TPU liners" width="560"></p>
<p align="center"><img src="docs/img/spoon_finger.png" alt="Side view of a spoon finger: OpenArm's base, the wide jaw, the soft face and the slot at the tip" width="700"></p>

**Checked with the spoon's own model** (`cad/openarm_spoon.py --spoon <stl>`), held 200 mm along the handle, where it is
38.5 mm wide (14.5 mm of the 44 mm finger stroke):
- **Fit:** the handle sits in both slots, touching only the TPU liners, which it presses slightly. No hard part
  touches it.
- **Form closure:** tilting the spoon 3° either way about the closing axis drives it into the slot lips, so it can't
  turn.
- **Load:** a 0.5 kg spoon, with its centre of mass 108 mm from the grasp, puts 0.53 N·m on the grasp. The lips carry
  that as ±17 N across the 31 mm width. That's about 1.5 MPa in the 3.5 mm PETG lip, far below its strength.
- **Picking it up:** the lower lip at the tip is 4.7 mm thick, liner included. A spoon lying on a table rests on its
  bowl and the end of its handle, so near the neck the handle is 10–18 mm clear of the table and the lip slides under
  it.
- **Strength and clearance:** every section of the hard part has at least 2.2× what a 100 N knock at the tip needs.
  Over the whole stroke the fingers never touch each other.

| File | Material | Per gripper |
|---|---|---|
| `hardware/spoon/STL/spoon_finger_left.stl`, `spoon_finger_right.stl` | PETG | 1 + 1 |
| `hardware/spoon/STL/spoon_tpu_left.stl`, `spoon_tpu_right.stl` | TPU 95A | 1 + 1 |

They fit OpenArm the same way as the fingers above, with the same base and screws.

## Files

| Path | What |
|---|---|
| `hardware/STEP/`, `hardware/STL/` | The four printable parts |
| `cad/openarm_hardshell.py` | The generator (build123d). `python openarm_hardshell.py --openarm <handumi-sw assets>` also renders and checks on OpenArm |
| `cad/check_strength.py` | The bending check along the hard part |
| `cad/openarm_spoon.py` | The spoon fingers' generator, with the fit, form-closure and load check against a spoon's STL |
| `hardware/spoon/` | The spoon fingers' STEP and STL |
| `docs/img/` | Renders |

Licences and sources: [NOTICE.md](NOTICE.md).
