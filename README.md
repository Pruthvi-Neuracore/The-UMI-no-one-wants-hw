# OpenArm hard-shell fingers

<p align="center"><img src="docs/img/openarm_open.png" alt="Two fingers on OpenArm's link7, half open: white hard holders on Open-ENPIRE's mount, each carrying the black PG Chopstick soft pad" width="560"></p>

Printed fingers for **OpenArm v1's parallel gripper**, in place of its stock fingers. Each finger is one soft part and
one hard part, as in Open-ENPIRE:

- **Soft part:** the PG Chopstick's TPU pad, its grooved face and soft fingertip.
- **Hard part:** a holder the pad drops into, on Open-ENPIRE's OpenArm mount.

<p align="center"><img src="docs/img/finger.png" alt="One finger: the hard holder with its pocket (left), the soft pad (middle) and the pad in the holder (right)" width="820"></p>
<p align="center"><img src="docs/img/finger_side.png" alt="Side view: the pad's face stands just proud of the holder's rim" width="700"></p>

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
- **Holder (hard):** the pad's side outline grown by 2.5 mm, as one constant width 2.5 mm wider than the pad on each
  side, with rounded edges.
  - The pad's exact shape is cut into it, so the pad drops in and is glued along its back and sides.
  - The rim follows the pad's face, 1.5 mm behind it, so the whole soft face is free to grip.
  - Past the pad's end, a 1.5 mm **hard tip** comes down to 0.6 mm behind the soft face, so the very tip is hard and
    soft together.
- **Soft part:** the chopstick pad's straight part. The chopstick closes on a pivot, so there its pad is angled 9°;
  here it's turned straight, so the two pads close flat on each other.
- **Closing:** fully closed, the pads are 0.6 mm apart, so they can hold thin things. Fully open, they are about 89 mm
  apart.

## The screws, with the finger glued

The holder starts 18 mm in front of the mount plate, joined to it by three ribs that run between the screw heads. So
all four screws stay reachable after the pad is glued in:
- The upper two are open from above.
- The lower two take a 2.5 mm ball-end hex key, between the ribs.

<p align="center"><img src="docs/img/openarm_closed.png" alt="The fingers closed on OpenArm's link7" width="560"></p>

## Checked

On OpenArm v1's description, closed, half open and fully open:
- The holders never touch each other, and neither do the pads.
- The holder and the pad don't overlap.
- Fully closed, the mount's key rib sits in link7's slot the same way Open-ENPIRE's does.

## Print and fit

1. **Holder:** print it standing on the mount plate's back face. Use 4 walls and 30% infill, with supports only under
   the plate.
2. **Pad:** print in TPU 95A, lying on its back.
3. **Glue:** glue the pad into the holder with flexible CA, or a TPU-rated contact adhesive.
4. **Fit:**
   1. Take off the stock finger.
   2. Screw the finger on with its four M3 screws: the upper two from above, the lower two with the ball-end key
      between the ribs.

## Files

| Path | What |
|---|---|
| `hardware/STEP/`, `hardware/STL/` | The four printable parts |
| `cad/openarm_hardshell.py` | The generator (build123d). `python openarm_hardshell.py --openarm <handumi-sw assets>` also renders and checks on OpenArm |
| `cad/source/` | The chopstick TPU pad it starts from |
| `docs/img/` | Renders |

Licences and sources: [NOTICE.md](NOTICE.md) (Apache-2.0).
