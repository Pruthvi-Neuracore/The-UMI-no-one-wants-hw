"""PG palm button: a soft record button that sits in your palm, on the thumb strap, for the wrist-worn (hand) builds.

Curl your middle and ring fingers into the palm, like Spider-Man firing a web, and they press it: once to start
recording, again to stop. It's one TPU 95A part, threaded on the 20 mm thumb strap, so it stays put and gives with
the hand:

  * a rounded pad, 32 mm along the hand x 26 mm across, 10 mm thick, its top a gentle dome with three shallow rings
    (you find it by feel),
  * a tunnel through its base for the strap (21 x 2.4 mm, across the hand), so it slides along the strap to fit
    your palm,
  * inside, a pocket for a 6 x 6 mm tactile switch (a soft one, ~160 gf: the pad already takes the grasp) under a
    1.2 mm membrane with a nub on its underside that presses the switch, put in through a slit in the pad's wrist end,
  * a 3 mm channel from the pocket out of the wrist end for the 2-core wire, which runs along the strap to the
    box (D405) or pod (X5) and in through its button-wire hole.

Part frame (mm): origin at the centre of the base (the side on your palm), +x towards the fingers, +y across the hand
(the strap's direction), +z out of the palm (the dome, towards the curled fingers).

    python palm_button.py
"""
from build123d import Axis, Box, Cylinder, Pos, Rot, Torus, fillet

from parts import save_part

L, W, H = 32.0, 26.0, 10.0         # along the hand, across, thick
CORNER_R, TOP_R = 9.0, 3.5
STRAP = (21.0, 2.4)                # strap tunnel: width along the hand x thickness, through along y
STRAP_Z = 1.4                      # tunnel floor above the base
SWITCH = (6.4, 6.4, 4.2)           # pocket for a 6 x 6 x ~3.5 mm switch
MEMBRANE, NUB = 1.2, (2.6, 1.0)    # membrane over the switch; nub (dia, length) under it
WIRE_D = 3.0


def build():
    pad = Pos(0, 0, H / 2) * Box(L, W, H)
    pad = fillet(pad.edges().filter_by(Axis.Z), CORNER_R)
    pad = fillet(pad.edges().group_by(Axis.Z)[-1], TOP_R)                 # soft top edge
    pad -= Pos(0, 0, STRAP_Z + STRAP[1] / 2) * Box(STRAP[0], W + 2, STRAP[1])  # strap tunnel
    z_top = H - MEMBRANE
    z_sw = z_top - SWITCH[2]
    pad -= Pos(0, 0, (z_sw + z_top) / 2) * Box(*SWITCH)                     # switch pocket
    pad += Pos(0, 0, z_top - NUB[1] / 2) * Cylinder(NUB[0] / 2, NUB[1])      # nub under the membrane
    pad -= Pos(-L / 4 - 2.0, 0, (z_sw + z_top - 1.5) / 2) * Box(L / 2 + 4.0, 1.2, SWITCH[2] - 1.5)  # insertion slit
    pad -= Pos(-L / 4 - 2.0, 0, z_sw + WIRE_D / 2) * Rot(0, 90, 0) * Cylinder(WIRE_D / 2, L / 2 + 4.0)  # wire channel
    for r in (4.0, 7.0, 10.0):                                              # three rings to find it by feel
        pad -= Pos(0, 0, H + 0.1) * Torus(r, 0.45)
    return pad


def switch_envelope():
    """The tactile switch in its pocket (for renders)."""
    z_sw = H - MEMBRANE - SWITCH[2]
    return Pos(0, 0, z_sw + 1.75) * Box(6.0, 6.0, 3.5)


def main():
    p = build()
    save_part("palm_button", p)
    bb = p.bounding_box()
    print(f"palm_button {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
          f"{len(p.solids())} solid(s)")


if __name__ == "__main__":
    main()
