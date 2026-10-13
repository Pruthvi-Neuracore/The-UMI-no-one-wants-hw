"""OpenArm chest camera: the head camera holder (head camera holder v4.stl) turned into a mast that bolts into the
T-slots of OpenArm's core (the 60 x 60 extrusion), with a separate D435 mount on top of it.

Holder (chest_camera_holder): the source holder is a 250 mm column with a forward arm at its top and a tilted foot
at its bottom. Here:
  * the foot is cut off and the column stands on a block on a mounting plate (PLATE): 60 mm wide like the core's
    face, two keys on its back that sit in the face's two 8 mm T-slots (30 mm apart), 4x M5 bolts into 8 mm-slot
    hammer nuts. The plate runs down below the column (BOLTS), so all four heads are open from the front;
  * the arm at the top, pointing away from the core, gets 4x M3 holes (ARM_HOLES), counterbored from below, for the
    D435 mount.

D435 mount (d435_mount), a separate part: a base that sits on the arm (M3 heat-set inserts from below), a seat
tilted TILT down for the camera, a rear wall with the D435's two rear M3 holes (45 mm apart), a low front lip.

Holder frame as the source STL (mm): z up, the core behind the column at +x (its face at X_FACE), the arm and the
camera looking towards -x, y across (the column's centre at Y_C).
The source STL is read from cad/source/head_camera/ (or --src); it is not published.

    python openarm_chest_camera.py [--src DIR]
"""
import argparse
import math
from pathlib import Path

from build123d import (Axis, Box, Cylinder, Mesher, Plane, Pos, Rot, export_step, export_stl, fillet)

import openarm_hardshell as H

OUT = H.OUT / "chest_camera"
IMG = H.IMG
SRC = Path(__file__).resolve().parent / "source" / "head_camera"
Y_C = 41.0                 # the column's centre across
CUT_Z = 50.0               # the source's foot and its bend are below this
X_FACE = 23.0              # the core's face (the plate's back); the column's back is at x <= 19.9
# the mounting plate: (x from, z from, z to), 60 wide; the block under the column (x from, z from, z to)
PLATE = (15.0, -45.0, 80.0)
BLOCK = (4.9, 44.0, 62.0)
SLOT_Y = (-15.0, 15.0)     # the core's two T-slots on a face, from its centre
KEY = (7.6, 1.8)           # keys in the slots: width, height (the slots are 8 mm, their lips 2 mm deep)
BOLTS_Z = (-30.0, 0.0)     # M5 bolt rows, under the column
M5 = (5.5, 10.0, 3.0)      # clearance, counterbore dia, counterbore depth
# the arm: its top, and the camera mount's four M3 holes (x, y from Y_C), counterbored from below
ARM_TOP = 262.8
ARM_HOLES = [(-27.0, -8.0), (-27.0, 8.0), (-11.0, -8.0), (-11.0, 8.0)]
M3 = (3.4, 6.2, 3.0)       # clearance, counterbore dia, counterbore depth
MOUNT_X = -19.0            # the mount's centre on the arm
# the D435 mount
D435 = (90.0, 25.0, 25.0)  # width, depth, height
TILT = 45.0                # the seat's tilt down (deg): the camera looks 45 deg down
FIT = 0.4
SEAT_T, WALL_T, WALL_H, LIP = 4.0, 4.0, 18.0, (2.5, 3.0)
REAR_M3 = (45.0, 12.5)     # the D435's rear M3 holes: spacing, height above its bottom face
INSERT = (4.0, 6.0)        # M3 heat-set insert holes in the mount's base: dia, depth
BASE = (32.0, 4.0)         # the mount's base: depth (x), thickness


def _xbox(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def holder(src):
    s = Mesher().read(str(Path(src) / "head_camera_holder_v4.stl"))[0]
    s = s & _xbox(-200, 200, -200, 200, CUT_Z, 400)
    x0, z0, z1 = PLATE
    plate = fillet(_xbox(x0, X_FACE, Y_C - 30, Y_C + 30, z0, z1).edges().filter_by(Axis.X), 4.0)
    bx0, bz0, bz1 = BLOCK
    block = _xbox(bx0, X_FACE - 1, Y_C - 16.6, Y_C + 16.6, bz0, bz1)
    h = s + block + plate
    for dy in SLOT_Y:                                                   # keys into the T-slots
        h += _xbox(X_FACE - 0.5, X_FACE + KEY[1], Y_C + dy - KEY[0] / 2, Y_C + dy + KEY[0] / 2, z0 + 3, z1 - 3)
        for z in BOLTS_Z:
            h -= Pos(X_FACE, Y_C + dy, z) * Rot(0, 90, 0) * Cylinder(M5[0] / 2, 40)
            h -= Pos(x0 + M5[2] / 2 - 0.01, Y_C + dy, z) * Rot(0, 90, 0) * Cylinder(M5[1] / 2, M5[2] + 0.02)
    arm_bottom = ARM_TOP - 19.3
    for x, dy in ARM_HOLES:
        h -= Pos(x, Y_C + dy, ARM_TOP - 10) * Cylinder(M3[0] / 2, 30)
        h -= Pos(x, Y_C + dy, arm_bottom + M3[2] / 2 - 0.01) * Cylinder(M3[1] / 2, M3[2] + 0.02)
    if len(h.solids()) > 1:
        h = max(h.solids(), key=lambda q: q.volume)
    return h


def _cam_frame():
    """The camera's frame on the mount (mount frame: base at z = 0, x as the holder's, centred on MOUNT_X):
    origin O at the camera's back-bottom edge, f forward (towards -x, tilted down), n up from the seat."""
    t = math.radians(TILT)
    f = (-math.cos(t), -math.sin(t))
    n = (-math.sin(t), math.cos(t))
    D = D435[1] + FIT
    # place it so the seat's front-bottom corner sits over the base's front edge, 3 mm up
    u, v = D + LIP[0], -SEAT_T
    xo = -BASE[0] / 2 + 1.0 - (u * f[0] + v * n[0])
    zo = BASE[1] - (u * f[1] + v * n[1])
    return (xo, zo), f, n


def d435_mount():
    from shapely.geometry import MultiPoint, Polygon
    from shapely.ops import unary_union
    (xo, zo), f, n = _cam_frame()
    D = D435[1] + FIT

    def P(u, v):
        return (xo + u * f[0] + v * n[0], zo + u * f[1] + v * n[1])
    seat = Polygon([P(-WALL_T, -SEAT_T), P(D + LIP[0], -SEAT_T), P(D + LIP[0], 0), P(-WALL_T, 0)])
    wall = Polygon([P(-WALL_T, 0), P(0, 0), P(0, WALL_H), P(-WALL_T, WALL_H)])
    lip = Polygon([P(D, 0), P(D + LIP[0], 0), P(D + LIP[0], LIP[1]), P(D, LIP[1])])
    hb = BASE[0] / 2
    under = MultiPoint([(-hb, 0), (hb, 0), P(-WALL_T, -SEAT_T), P(D + LIP[0], -SEAT_T),
                        (-hb, BASE[1]), (hb, BASE[1])]).convex_hull
    prof = unary_union([seat, wall, lip, under]).buffer(0.6, join_style=1).buffer(-0.6, join_style=1)
    pts = [(round(x, 3), round(z, 3)) for x, z in list(prof.simplify(0.02).exterior.coords)[:-1]]
    w = D435[0] + 2 * FIT + 4.0
    m = H._poly_prism(pts, (0, 2), -w / 2, w / 2)
    try:
        m = fillet(m.edges().filter_by(Axis.Y), 0.8)
    except Exception:
        pass
    for dy in (-REAR_M3[0] / 2, REAR_M3[0] / 2):                          # the D435's rear M3 holes, through the wall
        cx, cz = P(-WALL_T / 2, REAR_M3[1])
        m -= Pos(cx, dy, cz) * Rot(0, 90 - TILT, 0) * Cylinder(M3[0] / 2, WALL_T + 4)
    for x, dy in ARM_HOLES:                                                 # heat-set inserts, from below
        m -= Pos(x - MOUNT_X, dy, INSERT[1] / 2 - 0.01) * Cylinder(INSERT[0] / 2, INSERT[1] + 0.02)
    return m


def camera_box():
    """The D435 on the mount (for the renders), in the mount frame."""
    (xo, zo), f, n = _cam_frame()
    t = TILT
    c = (xo + (D435[1] / 2 + FIT / 2) * f[0] + D435[2] / 2 * n[0], zo + (D435[1] / 2 + FIT / 2) * f[1] + D435[2] / 2 * n[1])
    return Pos(c[0], 0, c[1]) * Rot(0, t, 0) * Box(D435[1], D435[0], D435[2])


def core(z0=-120.0, z1=120.0):
    """A stub of OpenArm's 60 x 60 core behind the plate (for the renders)."""
    b = _xbox(X_FACE, X_FACE + 60, Y_C - 30, Y_C + 30, z0, z1)
    for dy in SLOT_Y:
        b -= _xbox(X_FACE - 1, X_FACE + 2, Y_C + dy - 4, Y_C + dy + 4, z0 - 1, z1 + 1)
        b -= _xbox(X_FACE + 2, X_FACE + 7, Y_C + dy - 8.25, Y_C + dy + 8.25, z0 - 1, z1 + 1)
    return b


def render(h, mount):
    import pyvista as pv
    place = Pos(MOUNT_X, Y_C, ARM_TOP)
    items = [(H._mesh(h, 0.05), "#2b2e33"), (H._mesh(place * mount, 0.05), "#e9edf1"),
             (H._mesh(place * camera_box(), 0.1), "#55595f"), (H._mesh(core(), 0.1), "#c9ccd1")]

    def plot(path, items, view, up=(0, 0, 1), size=(1300, 1500), zoom=1.05):
        p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
        p.set_background("white")
        p.enable_anti_aliasing("ssaa")
        for m, c in items:
            p.add_mesh(pv.wrap(m), color=c, smooth_shading=True, split_sharp_edges=True, specular=0.35)
        p.view_vector(view, viewup=up)
        p.reset_camera()
        p.camera.zoom(zoom)
        p.screenshot(str(path))
        p.close()
        print("wrote", path.name)
    plot(IMG / "chest_camera.png", items, (-1.0, -0.8, 0.35))
    plot(IMG / "chest_camera_back.png", items[:2] + items[2:3], (1.0, -0.6, -0.25))
    plot(IMG / "d435_mount.png", [(H._mesh(mount, 0.03), "#e9edf1")], (-0.8, -1.0, 0.6), size=(1400, 1000), zoom=1.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(SRC), help="folder with head_camera_holder_v4.stl")
    a = ap.parse_args()
    h, mount = holder(a.src), d435_mount()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("chest_camera_holder", h), ("d435_mount", mount)):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    render(h, mount)


if __name__ == "__main__":
    main()
