"""Finger sleeve with a record button, on each handheld jaw.

The finger rides under the jaw's dotted shelf, against the jaw's outer wall, and pushes that wall inward to close
the jaws. The sleeve is a ring around the finger there, merged into the jaw:

  * a 5.5 mm pad on the outer wall (the wall is only ~4 mm thick), which is what the finger pad rests on and pushes,
  * a tube round the finger (Ø20 bore, 2.4 mm wall, 26 mm long), open at both ends, staying below the shelf so the
    red dots on top stay visible,
  * a 6 x 6 mm tactile switch in a pocket in the pad, under a domed TPU cover (`sleeve_button_cover`) set flush in
    a 12 x 9 mm oval recess, with a wire channel out of the sleeve's back end.

Both handheld jaws get a sleeve, so the gripper works in either hand: fit the switch on your index-finger side and a
cover with no switch (or with one, as a second button) on the other.
Pressing the wall also closes the jaws, so use a stiff switch (about 250-320 gf); the firmware tells a deliberate
press from a grasp by its duration.

The sleeve is placed from the jaw's own faces (outer wall and shelf underside), so the same code fits both jaws.
"""
import numpy as np
from build123d import Box, Cylinder, GeomType, Plane, Pos, Rot

R_IN, WALL, LENGTH, PAD = 10.0, 2.4, 26.0, 5.5
SHELF_CLEAR = 0.6
SWITCH, SWITCH_DEPTH = 6.4, 4.3
COVER = (12.0, 9.0, 1.0)              # oval recess: length (along the finger), width, depth
WIRE = (2.4, 2.0)                     # channel width, depth
ALONG = 0.0                           # sleeve centre along the jaw, relative to the outer wall's centre (mm)


def _faces(jaw, side):
    sy = -1.0 if side == "right" else 1.0
    planes = [f for f in jaw.faces() if f.geom_type == GeomType.PLANE]
    outer = max((f for f in planes if np.dot(tuple(f.normal_at()), (0.19, 0.98 * sy, 0)) > 0.98), key=lambda f: f.area)
    shelf = max((f for f in planes if np.dot(tuple(f.normal_at()), (0.34, -0.07 * sy, -0.94)) > 0.98),
                key=lambda f: f.area)
    return outer, shelf


def frame(jaw, side):
    """Sleeve frame: origin on the outer wall at the finger's height, x along the jaw, z out of the wall."""
    outer, shelf = _faces(jaw, side)
    n = np.array(tuple(outer.normal_at()))
    d = np.array([-n[1], n[0], 0.0]) if side == "right" else np.array([n[1], -n[0], 0.0])
    d /= np.linalg.norm(d)
    if d[0] < 0:
        d = -d
    c = np.array(tuple(outer.center())) + d * ALONG
    nb = np.array(tuple(shelf.normal_at()))
    pb = np.array(tuple(shelf.center()))

    def shelf_z(x, y):                 # z of the shelf's underside above (x, y)
        return pb[2] - (nb[0] * (x - pb[0]) + nb[1] * (y - pb[1])) / nb[2]

    # finger axis: PAD + R_IN out from the wall; its top (plus the tube wall) must stay under the shelf over the
    # whole sleeve length
    axis_xy = c[:2] + n[:2] * (PAD + R_IN)
    z_top = min(shelf_z(*(axis_xy + d[:2] * s + n[:2] * t)) for s in (-LENGTH / 2, LENGTH / 2) for t in (-R_IN, R_IN))
    zc = z_top - SHELF_CLEAR - R_IN
    origin = np.array([c[0], c[1], zc])
    return Plane(origin=tuple(origin), x_dir=tuple(d), z_dir=tuple(n))


def sleeve_cutters(pl):
    """(solid to add, solid to remove) in the gripper frame."""
    pad = Pos(0, 0, PAD / 2) * Box(LENGTH, 2 * (R_IN + WALL) - 2.0, PAD)
    tube = Pos(0, 0, PAD + R_IN) * Rot(0, 90, 0) * Cylinder(R_IN + WALL, LENGTH)
    add = pad + tube
    bore = Pos(0, 0, PAD + R_IN) * Rot(0, 90, 0) * Cylinder(R_IN, LENGTH + 2)
    pocket = Pos(0, 0, PAD - SWITCH_DEPTH / 2 + 0.01) * Box(SWITCH, SWITCH, SWITCH_DEPTH)
    L, W, D = COVER
    recess = Pos(0, 0, PAD - D / 2 + 1.0) * _oval(L, W, D + 2.0)   # also clears the tube's wall around the dome
    # wire channel: from the pocket to the sleeve's back end, along the bottom of the pocket
    chan = Pos(-LENGTH / 4 - 1.0, 0, PAD - SWITCH_DEPTH + WIRE[1] / 2) * Box(LENGTH / 2 + 2.0, WIRE[0], WIRE[1])
    chan += Pos(-LENGTH / 2, 0, PAD - SWITCH_DEPTH / 2) * Box(2.0, WIRE[0], SWITCH_DEPTH)   # exit at the back end
    remove = bore + pocket + recess + chan
    return pl.location * add, pl.location * remove


def _oval(L, W, H):
    """Stadium-shaped prism, length along x, centred."""
    r = W / 2
    s = Box(L - W, W, H)
    for sx in (-1, 1):
        s += Pos(sx * (L - W) / 2, 0, 0) * Cylinder(r, H)
    return s


def add_sleeve(jaw, side):
    pl = frame(jaw, side)
    add, remove = sleeve_cutters(pl)
    return (jaw + add) - remove, pl


def cover(pl):
    """TPU button cover in the gripper frame: oval cap with a 0.8 mm dome on top and a nub under it for the switch."""
    L, W, D = COVER
    cap = Pos(0, 0, PAD - D / 2) * _oval(L - 0.3, W - 0.3, D)
    dome = Pos(0, 0, PAD + 0.4) * _oval(L - 2.0, W - 2.0, 0.8)
    nub = Pos(0, 0, PAD - D - 0.4) * Cylinder(1.5, 0.8)
    part = cap + dome + nub
    return pl.location * part

