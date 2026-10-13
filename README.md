# OpenArm hard-shell fingers

<p align="center"><img src="docs/img/openarm_open.png" alt="Two fingers on OpenArm's link7, half open: yellow hard shells on Open-ENPIRE's mount, each wrapping a black soft part" width="560"></p>

Printed fingers for **OpenArm v1's parallel gripper**, in place of its stock fingers. Each finger is a **soft TPU part
inside a hard shell**: the shell follows the soft part all round its back and sides, and closes over its end as a thin
hard tip, so the fingertip is hard and soft together.

<p align="center"><img src="docs/img/finger.png" alt="One finger: the hard shell (left), the soft part (middle) and the two together (right)" width="820"></p>

## What you print

| File | Material | Per gripper |
|---|---|---|
| `hardware/STL/hardshell_finger_left.stl`, `hardshell_finger_right.stl` | PETG (or PLA) | 1 + 1 |
| `hardware/STL/hardshell_tpu_left.stl`, `hardshell_tpu_right.stl` | TPU 95A | 1 + 1 |

The STEP files are in `hardware/STEP/`. The right parts are the mirror images of the left ones, as OpenArm's own
fingers are. The renders show a yellow shell and a black soft part; any colours work.

## How it's made

- **Mount:** Open-ENPIRE's OpenArm mount, unchanged: a 5 mm plate with four M3 holes and the key rib behind it. It
  goes where the stock finger's base goes, with the same screws.
- **Hard shell:** a 2.5 mm wall that follows the soft part's back and sides along its whole length.
  - At the fingertip it closes over the soft part's end as a **1.5 mm hard tip**, coming down to 0.6 mm behind the
    soft face.
  - Three ribs join it to the plate.
- **Soft part:** a tapered finger in TPU 95A.
  - Its contact face stands 1.5 mm proud of the shell's rim and has three shallow transverse grooves, like the PG
    Chopstick's pad.
  - It curves 1.5 mm inward over the last part of the finger, so the tips pinch first.
  - It's glued into the shell along its whole back and sides.
- **Hard and soft at the tip:** a light grasp is on the soft part. A firm one, or a thin object at the very tip, also
  bears on the hard tip.
- **Closing:** fully closed, the soft tips are 0.6 mm apart and the hard tips 1.8 mm. Fully open, they are about
  89 mm apart.

## The screws, with the finger glued

The shell starts 18 mm in front of the mount plate. The ribs run between the screw heads, so all four screws stay
reachable after the soft part is glued in:
- The upper two screws are open from above.
- The lower two take a 2.5 mm ball-end hex key, between the ribs and over the shell's base.

<p align="center"><img src="docs/img/openarm_closed.png" alt="The fingers closed on OpenArm's link7" width="560"></p>

## Checked

On OpenArm v1's description, closed, half open and fully open:
- The shells never touch each other, and neither do the soft parts.
- The shell and the soft part don't overlap.
- Fully closed, the mount's key rib sits in link7's slot the same way Open-ENPIRE's does.

## Print and fit

1. **Shell:** print it standing on the mount plate's back face, with supports only under the plate's overhang. Use 4
   walls and 30% infill. Or print it lying on its back, with tree supports under the rim.
2. **Soft part:** print in TPU 95A, lying on its back, with no supports.
3. **Glue:** glue the soft part into the shell. For TPU to PETG or PLA, use flexible CA, or a TPU-rated contact
   adhesive.
4. **Fit:**
   1. Take off the stock finger.
   2. Screw the finger on with its four M3 screws: the upper two from above, the lower two with the ball-end key
      between the ribs.

## Files

| Path | What |
|---|---|
| `hardware/STEP/`, `hardware/STL/` | The four printable parts |
| `cad/openarm_hardshell.py` | The generator (build123d). `python openarm_hardshell.py --openarm <handumi-sw assets>` also renders and checks on OpenArm |
| `docs/img/` | Renders |

Licences and sources: [NOTICE.md](NOTICE.md) (Apache-2.0).
