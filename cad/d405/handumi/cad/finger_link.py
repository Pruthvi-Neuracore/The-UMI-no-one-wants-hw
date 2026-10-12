"""Finger links with a universal tip flange.

The original links carry tips on a 12 x 16 x 3 mm plate on a 12 x 6 mm neck. Here the plate and neck are
replaced by a lofted neck flaring into a 20 x 30 x 5 mm flange with filleted edges. The front face stays at
y = 0, so tips sit exactly where they did. Hole patterns on the flange (link frame: x across, z along the finger,
holes along Y):

    M2   4x  8 x 12 mm   existing tip library (self-tapping, as before)
    M3   6x  12 x 24 mm rectangle + centre pair at z = ±12 (heat-set inserts, Ø4.0 x 5)
    M4   2x  12 mm apart at z = 0 (heat-set inserts, Ø5.6 x 7); same spacing as Robotiq's M4 accessory holes

Flange width is set so the two flanges keep a 3.6 mm gap at full close.

The index/middle link has a 6 x 6 mm tactile switch set flush into the inner wall of its sleeve, where the index finger
pad rests: press with the finger to click (record start / stop, wired to the Pico).
A finger hood arches over the finger channel on each link (one channel on the thumb link, a double one on the
index/middle link) so the finger stays put; print the links in PETG or TPU-friendly settings if the hood feels tight.
"""
from pathlib import Path

from build123d import Axis, Box, Cylinder, Face, Polyline, Pos, Rot, Wire, fillet, import_step, loft

FLANGE = (20.0, 5.0, 30.0)          # x, y (thickness), z
NECK_ROOT = (-6.0, 6.0, -9.0, 7.0)  # x0, x1, z0, z1 on the link body at y = NECK_Y
NECK_Y = -12.4
M2 = [(sx * 4.0, sz * 6.0) for sx in (-1, 1) for sz in (-1, 1)]
M3 = [(x, sz * 12.0) for x in (-6.0, 0.0, 6.0) for sz in (-1, 1)]
M4 = [(-6.0, 0.0), (6.0, 0.0)]


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def rect_y(y, x0, x1, z0, z1):
    return Face(Wire(Polyline((x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1), close=True).edges()))


def hole_y(x, z, d, depth):
    return Pos(x, -depth / 2 + 0.01, z) * Rot(90, 0, 0) * Cylinder(d / 2, depth + 0.02)


HOOD_Y = (-43.0, -21.0)        # along the finger (link y)
HOOD_CLEAR, HOOD_T = 0.5, 2.5   # skin clearance over the Ø23 channel, shell thickness


def finger_hood(thumb, sx):
    """Shell arching over the finger channel so the finger slides in like a ring and can't slip out sideways.
    Thumb: one Ø23 channel centred at (x, z) = (10·sx, -18). Index/middle: two Ø23 channels at (10·sx, -26 / -18).
    sx is the side of the channel: -1 for the right thumb and left index, +1 for the right index and left thumb."""
    from build123d import Circle, Plane, SlotCenterToCenter, extrude
    y0, y1 = HOOD_Y
    r_in = 11.5 + HOOD_CLEAR
    pl = Plane(origin=(0, y0, 0), x_dir=(1, 0, 0), z_dir=(0, 1, 0))     # sketch in XZ (local y = -Z), extrude +Y
    if thumb:
        outer = pl * Pos(10.0 * sx, 18.0) * Circle(r_in + HOOD_T)
        inner = pl * Pos(10.0 * sx, 18.0) * Circle(r_in)
    else:
        outer = pl * Pos(10.0 * sx, 22.0) * Rot(0, 0, 90) * SlotCenterToCenter(8.0, 2 * (r_in + HOOD_T))
        inner = pl * Pos(10.0 * sx, 22.0) * Rot(0, 0, 90) * SlotCenterToCenter(8.0, 2 * r_in)
    shell = extrude(outer, y1 - y0) - extrude(inner, y1 - y0)
    keep = box(-30, -5.5, y0 - 1, y1 + 1, -60, 20) if sx < 0 else box(5.5, 30, y0 - 1, y1 + 1, -60, 20)
    hood = shell & keep                                                  # only the part outside the paddle face
    try:
        hood = fillet(hood.edges().filter_by(Axis.Y, reverse=True), 1.0)   # soft ends
    except Exception:
        pass
    return hood


BUTTON = 6.0                     # 6 x 6 mm tactile switch (use a stiff one, ~250-320 gf), body 3.5 mm
BUTTON_Y, BUTTON_Z = -32.0, -22.0   # middle of the sleeve, between the two finger channels
BUTTON_FACE_X = -0.8             # depth of the channel wall there (index channel centred at x = 10·sx)
COVER, COVER_T = 10.0, 1.0       # TPU button cover thickness (COVER kept for the switch footprint)
COVER_L, COVER_W, DOME = 12.0, 9.0, 0.8   # oval pad: 12 mm along the finger, 9 mm across, dome 0.8 mm into the channel
NUB = 0.6                        # nub under the cover that presses the switch
SWITCH_STACK = COVER_T + NUB + 0.8 + 3.5   # cover + nub + actuator + body: depth of the switch pocket


def button_centre(sx):
    """Centre of the switch face (link frame): flush in the channel wall the finger presses, facing the finger."""
    return (BUTTON_FACE_X * sx, BUTTON_Y, BUTTON_Z)


def button_parts(sx):
    """(add, cut) for a 6 x 6 tactile switch set flush into the inner wall of the index sleeve, where the finger pad
    rests. A 3 mm boss on the other face of the paddle backs the pocket; the wires leave through it.
    sx: channel side (+1 right index, -1 left index)."""
    cx, cy, cz = button_centre(sx)
    x_back = -6.0 * sx                                                   # other face of the paddle
    from build123d import Cylinder as _Cyl, Rot as _Rot                  # round, soft-edged boss: nothing for the hand to catch on
    boss = Pos(x_back - 1.5 * sx, cy, cz) * _Rot(0, 90, 0) * _Cyl(8.0, 3.0)
    try:
        boss = fillet([e for e in boss.edges() if abs(e.center().X - (x_back - 3.0 * sx)) < 0.01], 1.4)
    except Exception:
        pass
    depth = SWITCH_STACK                                                 # switch sits under the TPU cover
    x0, x1 = sorted((cx + 0.6 * sx, cx - depth * sx))
    pocket = box(x0, x1, cy - BUTTON / 2 - 0.2, cy + BUTTON / 2 + 0.2, cz - BUTTON / 2 - 0.2, cz + BUTTON / 2 + 0.2)
    l0, l1 = sorted((cx - (depth - 0.1) * sx, x_back - 4.0 * sx))    # legs + solder joints; open at the back for access
    legs = box(l0, l1, cy - 4.5, cy + 4.5, cz - 3.5, cz + 3.5)
    w0, w1 = sorted((cx - depth * sx, x_back - 3.5 * sx))
    wires = box(w0, w1, cy - 1.5, cy + 1.5, cz - 2.0, cz + 2.0)          # out through the boss
    from build123d import Ellipse, Plane, extrude                        # oval recess for the TPU cover, flush with the wall
    pl = Plane(origin=(cx - COVER_T * sx, cy, cz), x_dir=(0, 1, 0), z_dir=(sx, 0, 0))
    recess = extrude(pl * Ellipse(COVER_L / 2 + 0.15, COVER_W / 2 + 0.15), COVER_T + 3.0)
    return boss, pocket + legs + wires + recess


def button_cover():
    """Ergonomic TPU cover for the record button (print in TPU 95A): an oval pad, 12 mm along the finger and 9 mm across,
    that sits flush in the sleeve wall with a soft 0.8 mm dome rising into the channel, so the finger finds it by touch
    and presses it with a natural squeeze. A nub underneath presses the switch; a rim locates it in the recess.
    Modelled flat (dome up along +z, nub down) for printing; x runs along the finger."""
    from build123d import Cylinder, Ellipse, Sphere, extrude
    a, b = COVER_L / 2, COVER_W / 2
    pad = extrude(Ellipse(a, b), COVER_T)
    R = (b * b + DOME * DOME) / (2 * DOME)                              # spherical cap over the short axis
    dome = Pos(0, 0, COVER_T + DOME - R) * Sphere(R)
    dome &= extrude(Ellipse(a, b), COVER_T + DOME)
    pad = pad + dome
    try:
        pad = fillet(pad.edges().group_by(Axis.Z)[0], 0.4)
    except Exception:
        pass
    nub = Pos(0, 0, -NUB / 2) * Cylinder(1.6, NUB)                      # presses the 3.5 mm actuator
    rim = Pos(0, 0, -0.6) * (extrude(Ellipse(a - 0.3, b - 0.3), 0.6) - extrude(Ellipse(a - 1.3, b - 1.3), 0.6))
    return pad + nub + rim


def finger_link(src):
    s = import_step(str(src))
    s -= box(-6.6, 6.6, NECK_Y + 0.2, 0.6, -8.6, 8.6)                   # old 12 x 16 plate and 12 x 6 neck
    fx, fy, fz = FLANGE
    flange = box(-fx / 2, fx / 2, -fy, 0.0, -fz / 2, fz / 2)
    flange = fillet(flange.edges().filter_by(Axis.Y), 3.0)
    flange = fillet(flange.edges().group_by(Axis.Y)[-1], 1.0)          # soften the front edges
    x0, x1, z0, z1 = NECK_ROOT
    neck = loft([rect_y(NECK_Y - 1.0, x0, x1, z0, z1), rect_y(NECK_Y + 2.5, x0 - 0.5, x1 + 0.5, z0 - 1.5, z1 + 1.5),
                 rect_y(-fy + 0.01, -fx / 2 + 1, fx / 2 - 1, -fz / 2 + 3, fz / 2 - 3)], ruled=False)
    name = Path(str(src)).name
    thumb = "thumb" in name
    sx = (-1 if thumb else 1) * (-1 if name.startswith("left") else 1)
    s = s + neck + flange + finger_hood(thumb, sx)
    if not thumb:
        add, cut = button_parts(sx)
        s = s + add - cut
    for x, z in M2:
        s -= hole_y(x, z, 2.1, 6.0)
    for x, z in M3:
        s -= hole_y(x, z, 4.0, 5.0)
    for x, z in M4:
        s -= hole_y(x, z, 5.6, 7.0)
    return s
