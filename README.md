# OpenArm hard-shell fingers

<p align="center"><img src="docs/img/openarm_open.png" alt="Two fingers on OpenArm's link7, half open: white hard jaws on Open-ENPIRE's mount, each carrying the black PG Chopstick soft pad" width="560"></p>

Printed fingers for **OpenArm v1's parallel gripper**, in place of its stock fingers. Each finger is one soft part and
one hard part, as in Open-ENPIRE:

- **Soft part:** the PG Chopstick's TPU pad, its grooved face and soft fingertip.
- **Hard part:** a straight jaw on Open-ENPIRE's OpenArm mount, with a pocket the pad drops into.

<p align="center"><img src="docs/img/finger.png" alt="One finger: the hard jaw with its pocket (left), the soft pad (middle) and the pad in the jaw (right)" width="820"></p>
<p align="center"><img src="docs/img/finger_side.png" alt="Side view: the straight jaw on its base block, the pad's face just proud of the rim" width="700"></p>

## What you print

| File | Material | Per gripper |
|---|---|---|
| `hardware/STL/hardshell_finger_left.stl`, `hardshell_finger_right.stl` | PETG (or PLA) | 1 + 1 |
| `hardware/STL/hardshell_tpu_left.stl`, `hardshell_tpu_right.stl` | TPU 95A | 1 + 1 |

The STEP files are in `hardware/STEP/`. The right parts are the mirror images of the left ones, as OpenArm's own
fingers are.

## How it's made

- **Mount:** Open-ENPIRE's OpenArm mount, unchanged: a 5 mm plate with four M3 holes (with head seats) and the key
  rib behind it. It goes where the stock finger's base goes, with the same screws.
- **Hard jaw:** a straight, prismatic part.
  - **Outline:** in side view and in plan, it's the straight-sided envelope of the pad with a 2.5–3 mm margin. It has
    flat faces, straight tapers to a chamfered tip, and rounded long edges.
  - **Base:** a solid block, square to the mount plate.
  - **Pocket:** the pad's exact shape, with straight walls up to its face. The whole soft face is free to grip, and
    the pad's face stands 1.5 mm proud of the rim.
  - **Tip:** past the pad's end, a 1.5 mm **hard tip** comes down to 0.6 mm behind the soft face, so the very tip is
    hard and soft together.
- **Soft part:** the chopstick pad's straight part. The chopstick closes on a pivot, so there its pad is angled 9°;
  here it's turned straight, so the two pads close flat on each other.
- **Closing:** fully closed, the pads are 0.6 mm apart, so they can hold thin things. Fully open, they are about 89 mm
  apart.

## Strength and weight

Lean, but sized to survive a knock. `cad/check_strength.py` slices the jaw every 5 mm and checks its bending strength
(section modulus) against **a 100 N knock at the fingertip**, with a 1.5 safety factor on PETG's ~45 MPa:

| Where | Needed | Has (closing direction) |
|---|---|---|
| Base block | 413 mm³ | 1388 mm³ |
| Just past the base (the weakest point) | 313 mm³ | 360 mm³ |
| Middle of the finger | 196 mm³ | 392 mm³ |
| Near the tip | 80 mm³ | 357 mm³ |

So even at the weakest point, a 100 N knock at the tip keeps a 1.7 safety factor. A normal grasp is a few tens of
newtons. Across the jaws the jaw is about 5× stiffer still. The hard part is 45 cm³, about 57 g in PETG, including
the mount plate.

## The screws

The four M3 screws go in from the finger side:
- **Upper two:** they sit above the base block, so you reach them straight on, also with the pad glued in.
- **Lower two:** these sit in line with the jaw, in straight bores through the base block that open into the pad's
  pocket. Fit them before you glue the pad in.

<p align="center"><img src="docs/img/openarm_closed.png" alt="The fingers closed on OpenArm's link7" width="560"></p>

## Checked

On OpenArm v1's description, closed, half open and fully open:
- The jaws never touch each other, and neither do the pads.
- The jaw and the pad don't overlap.
- Fully closed, the mount's key rib sits in link7's slot the same way Open-ENPIRE's does.
- The STL files are watertight.

## Print and fit

1. **Jaw:** print it standing on the mount plate's back face. Use 4 walls and 30% infill, with supports only under the
   plate.
2. **Pad:** print in TPU 95A, lying on its back.
3. **Fit:**
   1. Take off the stock finger.
   2. Screw the jaw on with its four M3 screws.
   3. Glue the pad in with flexible CA or a TPU-rated contact adhesive.

## Files

| Path | What |
|---|---|
| `hardware/STEP/`, `hardware/STL/` | The four printable parts |
| `cad/openarm_hardshell.py` | The generator (build123d). `python openarm_hardshell.py --openarm <handumi-sw assets>` also renders and checks on OpenArm |
| `cad/check_strength.py` | The bending check along the jaw |
| `cad/source/` | The chopstick TPU pad it starts from |
| `docs/img/` | Renders |

Licences and sources: [NOTICE.md](NOTICE.md) (Apache-2.0).
