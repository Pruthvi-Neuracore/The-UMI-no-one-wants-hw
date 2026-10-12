"""Finger sleeves on the handheld jaws: a printed pad with two strap fins, and an elastic hook-and-loop strap.

The finger rides under the jaw's dotted shelf, against the jaw's outer wall, and pushes that wall inward to close
the jaws. At that spot each handheld jaw gets:

  * a 5.5 mm pad on the outer wall (the wall itself is only ~4 mm thick): the surface the finger pad rests on and
    pushes,
  * two strap fins along the pad's long sides, 2.4 mm thick and 8 mm tall, that cradle the sides of the finger; each
    has a 17 x 2.5 mm slot for the strap,
  * an elastic hook-and-loop strap, 16 mm wide (about 12 cm per finger, not printed): one end is looped through one
    fin's slot, it wraps over the finger, goes through the other fin's slot and folds back onto itself. It stretches
    and adjusts to any finger and comes off without tools,
  * on the index-finger jaw only, the record button: a 6 x 6 mm tactile switch in a pocket in the pad, under a domed
    TPU cover (`sleeve_button_cover_<side>`) set flush in a 12 x 9 mm oval recess, with a wire channel out of the back.

The pad is placed from the jaw's own faces (outer wall and shelf underside), so the same code fits both jaws; the
fins and the strap stay under the shelf so the red dots on top stay visible.
"""
import numpy as np
from build123d import Box, Cylinder, GeomType, Plane, Pos, Rot

LENGTH, PAD, HALF_W = 26.0, 5.5, 10.0         # pad length (along the finger), thickness, half-width
SHELF_CLEAR = 0.6
SWITCH, SWITCH_DEPTH = 6.4, 4.3
COVER = (12.0, 9.0, 1.0)                      # oval recess: length (along the finger), width, depth
WIRE = (2.4, 2.0)                             # channel width, depth
ALONG = 0.0                                   # sleeve centre along the jaw, relative to the outer wall's centre (mm)
FIN_T, FIN_H, FIN_L = 2.4, 8.0, 22.0          # strap fins: thickness, height above the pad, length along the finger
STRAP_W, STRAP_T = 16.0, 1.2                  # elastic hook-and-loop strap
SLOT = (STRAP_W + 1.0, 2.5)                   # slot in each fin: length along the finger, height
SLOT_Z = PAD + 4.75                           # slot centre, out from the wall
FINGER = 19.0                                 # finger height the reference strap is drawn round (16-22 fits)


def _faces(jaw, side):
    sy = -1.0 if side == "right" else 1.0
    planes = [f for f in jaw.faces() if f.geom_type == GeomType.PLANE]
    outer = max((f for f in planes if np.dot(tuple(f.normal_at()), (0.19, 0.98 * sy, 0)) > 0.98), key=lambda f: f.area)
    shelf = max((f for f in planes if np.dot(tuple(f.normal_at()), (0.34, -0.07 * sy, -0.94)) > 0.98),
                key=lambda f: f.area)
    return outer, shelf


def frame(jaw, side, finger=22.0):
    """Sleeve frame: origin on the outer wall, x along the jaw, z out of the wall, y up (across the finger).

    Its height is chosen so that the upper fin and a strap round a `finger` mm finger stay under the shelf."""
    outer, shelf = _faces(jaw, side)
    n = np.array(tuple(outer.normal_at()))
    d = np.array([-n[1], n[0], 0.0]) if side == "right" else np.array([n[1], -n[0], 0.0])
    d /= np.linalg.norm(d)
    if d[0] < 0:
        d = -d
    c = np.array(tuple(outer.center())) + d * ALONG
    nb = np.array(tuple(shelf.normal_at()))
    pb = np.array(tuple(shelf.center()))

    def shelf_z(xy):                      # z of the shelf's underside above (x, y)
        return pb[2] - (nb[0] * (xy[0] - pb[0]) + nb[1] * (xy[1] - pb[1])) / nb[2]

    reach = PAD + finger + STRAP_T         # how far the strap stands out from the wall
    z_top = min(shelf_z(c[:2] + d[:2] * s + n[:2] * t)
                for s in (-LENGTH / 2, LENGTH / 2) for t in np.linspace(0, reach, 6))
    zc = z_top - SHELF_CLEAR - (HALF_W + FIN_T)
    return Plane(origin=(c[0], c[1], zc), x_dir=tuple(d), z_dir=tuple(n))


def _oval(L, W, H):
    """Stadium-shaped prism, length along x, centred."""
    s = Box(L - W, W, H)
    for sx in (-1, 1):
        s += Pos(sx * (L - W) / 2, 0, 0) * Cylinder(W / 2, H)
    return s


def pad_cutters(pl, button):
    """(solid to add to the jaw, solid to remove from it), in the gripper frame."""
    add = Pos(0, 0, PAD / 2) * Box(LENGTH, 2 * HALF_W, PAD)
    remove = None
    for sy in (-1, 1):
        fin_y = sy * (HALF_W + FIN_T / 2)
        add += Pos(0, fin_y, (PAD + FIN_H) / 2) * Box(FIN_L, FIN_T, PAD + FIN_H)
        add += Pos(0, fin_y, PAD + FIN_H) * Rot(0, 90, 0) * Cylinder(FIN_T / 2, FIN_L)     # rounded top edge
        slot = Pos(0, fin_y, SLOT_Z) * Box(SLOT[0], FIN_T + 2, SLOT[1])
        remove = slot if remove is None else remove + slot
    if button:
        L, W, D = COVER
        remove += Pos(0, 0, PAD - SWITCH_DEPTH / 2 + 0.01) * Box(SWITCH, SWITCH, SWITCH_DEPTH)
        remove += Pos(0, 0, PAD - D / 2 + 0.5) * _oval(L, W, D + 1.0)
        remove += Pos(-LENGTH / 4 - 1.0, 0, PAD - SWITCH_DEPTH + WIRE[1] / 2) * Box(LENGTH / 2 + 2.0, WIRE[0], WIRE[1])
        remove += Pos(-LENGTH / 2, 0, PAD - SWITCH_DEPTH / 2) * Box(2.0, WIRE[0], SWITCH_DEPTH)
    return pl.location * add, pl.location * remove


def add_sleeve(jaw, side, button):
    pl = frame(jaw, side)
    add, remove = pad_cutters(pl, button)
    return (jaw + add) - remove, pl


def strap(pl, finger=FINGER):
    """Reference body for the elastic strap round a `finger` mm finger, in the gripper frame (not printed): up the
    inside of each fin from its slot, over the top of the finger, and the folded-back tail outside one fin."""
    r_out = HALF_W
    zc = PAD + finger - r_out             # centre of the arch over the finger
    body = None
    for sy in (-1, 1):
        leg = Pos(0, sy * (r_out - STRAP_T / 2), (SLOT_Z + zc) / 2) * Box(STRAP_W, STRAP_T, zc - SLOT_Z)
        body = leg if body is None else body + leg
    arch = Pos(0, 0, zc) * Rot(0, 90, 0) * (Cylinder(r_out, STRAP_W) - Cylinder(r_out - STRAP_T, STRAP_W + 1))
    arch -= Pos(0, 0, zc - r_out / 2 - 0.01) * Box(STRAP_W + 2, 2 * r_out + 2, r_out)
    tail = Pos(0, -(HALF_W + FIN_T + 0.3 + STRAP_T / 2), SLOT_Z + 3.0) * Box(STRAP_W, STRAP_T, 9.0)
    return pl.location * (body + arch + tail)


def cover(pl):
    """TPU button cover in the gripper frame: oval cap with a 0.8 mm dome on top and a nub under it for the switch."""
    L, W, D = COVER
    cap = Pos(0, 0, PAD - D / 2) * _oval(L - 0.3, W - 0.3, D)
    dome = Pos(0, 0, PAD + 0.4) * _oval(L - 2.0, W - 2.0, 0.8)
    nub = Pos(0, 0, PAD - D - 0.4) * Cylinder(1.5, 0.8)
    return pl.location * (cap + dome + nub)
