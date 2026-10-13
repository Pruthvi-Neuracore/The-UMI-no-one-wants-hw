"""OpenArm V5 fingers: a flat, triangular finger with a ribbed back, after the flat black fingers common on compact
parallel grippers, on OpenArm v1's printed finger base. One plain printed part per finger.

  * the gripping face: one flat triangle (GRIP_Z, the fingers close 0.2 mm apart), 61 mm wide at the mounting plate,
    tapering to a rounded tip (PLAN); a friction pad (1 mm rubber or grip tape) can be stuck on it, as in the photo;
  * the back: a wedge, the full height of the mounting plate at the root, curving down to TIP_T at the tip (BACK);
  * the cells: at the root the wedge is opened into four cells (CELLS), open to the back and running out through its
    slope, separated by ribs (RIB_T) and closed by the side walls (WALL_T) that follow the outline. The two middle
    cells are over the screw columns: their floor is just under the lower screw heads, so all four heads are open at
    the bottom of the cells, and a hex key reaches them straight along the cell;
  * base: OpenArm v1's printed finger base (openarm_hardshell.base(): the plate, its four M2.5 holes and the rib),
    its plate carried down to the gripping face, so the plate's end and the flat gripping face are one plane.

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).

    python openarm_v5.py [--openarm /path/to/handumi-sw/assets]
"""
import argparse

import numpy as np
from build123d import Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, fillet, mirror

import openarm_hardshell as H

OUT = H.OUT / "v5"
IMG = H.IMG
GRIP_Z = 24.8              # the gripping face: 0.2 mm apart when closed, as the other fingers
TIP_X = 95.0
# the gripping face's half-width along x: the plate's width at the root, straight sides, a rounded tip
PLAN = [(9.0, 30.5), (TIP_X - 4.0, 7.5), (TIP_X, 4.5)]
# the back (z along x): the top of the mounting plate at the root, a concave curve down to TIP_T above the face
BACK_ROOT, BACK_FLAT, TIP_T, BACK_POW = 60.2, 10.0, 4.0, 1.25
# the cells: (y from, y to) bands across the finger, open above CELL_FLOOR; the middle two are over the screw columns
# (y = +-7.5), the outer two lighten the root
RIB_T, WALL_T = 3.0, 3.0
CELLS = [(RIB_T / 2, 14.5), (14.5 + RIB_T, 40.0)]           # one side; mirrored to the other
CELL_FLOOR = 37.6          # just under the lower screw heads' seats (z 37.8)
ROUND = (4.0, 1.4)
EDGE_R = 1.2


def _rounded(pts, r):
    from shapely.geometry import Polygon
    pl = Polygon(pts).buffer(-r, join_style=1).buffer(r, join_style=1)
    return [(round(x, 3), round(y, 3)) for x, y in list(pl.simplify(0.02).exterior.coords)[:-1]]


def _plan(x0, inset=0.0):
    from shapely.geometry import Polygon
    pts = [(x0, -30.5)] + [(x, -h) for x, h in PLAN] + [(x, h) for x, h in PLAN[::-1]] + [(x0, 30.5)]
    pts = _rounded(pts, ROUND[0])
    if inset:
        pts = [(round(x, 3), round(y, 3)) for x, y in
               list(Polygon(pts).buffer(-inset, join_style=2).simplify(0.02).exterior.coords)[:-1]]
    return pts


def _back(x):
    t = np.clip((x - BACK_FLAT) / (TIP_X - BACK_FLAT), 0.0, 1.0)
    return GRIP_Z + TIP_T + (BACK_ROOT - GRIP_Z - TIP_T) * (1 - t) ** BACK_POW


def wedge():
    """The solid finger: the plan triangle x the side wedge, its long edges rounded."""
    x0 = H.PLATE["x"][1] - 0.5
    xb = x0 - 10.0
    xs = np.linspace(xb, TIP_X + 2, 80)
    side = [(xb, GRIP_Z), (TIP_X + 2, GRIP_Z)] + [(x, float(_back(x))) for x in xs[::-1]]
    w = H._poly_prism(_plan(xb), (0, 1)) & H._poly_prism(_rounded(side, ROUND[1]), (0, 2))
    w = w & H._poly_prism([(x0, -100), (200, -100), (200, 100), (x0, 100)], (0, 1))
    for r in (EDGE_R, 0.8):
        try:
            f = fillet(w.edges().filter_by(Axis.X, reverse=True).filter_by(lambda e: e.center().X > x0 + 1), r)
            if f.is_valid:
                return f
        except Exception:
            pass
    return w


def cells():
    """The four cells: the outline inset by the side walls, cut to the bands between the ribs, above the floor."""
    x1 = H.PLATE["x"][1]
    inner = H._poly_prism(_plan(x1 - 20.0, WALL_T), (0, 1))
    inner = inner & H._poly_prism([(x1, -100), (200, -100), (200, 100), (x1, 100)], (0, 1))
    out = None
    for a, b in CELLS:
        for sy in (1, -1):
            band = Pos(100, sy * (a + b) / 2, CELL_FLOOR + 50) * Box(200, b - a, 100)
            out = band if out is None else out + band
    return inner & out


def plate_to_face():
    """The mounting plate carried down to the gripping face (the stock plate ends 5.4 mm above it)."""
    (x0, x1), (y0, y1), (z0, _) = H.PLATE["x"], H.PLATE["y"], H.PLATE["z"]
    h = z0 - GRIP_Z + 2.0 * H.PLATE["r"]
    b = Pos((x0 + x1) / 2, (y0 + y1) / 2, GRIP_Z + h / 2) * Box(x1 - x0, y1 - y0, h)
    return fillet(b.edges().filter_by(Axis.X), H.PLATE["r"])


def build():
    v5 = H.base() + plate_to_face() + (wedge() - cells())
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        v5 -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        v5 -= Pos(x1 - H.CBORE[1] / 2 + 0.01, y, z) * Rot(0, 90, 0) * Cylinder(H.CBORE[0] / 2, H.CBORE[1] + 0.02)
    if len(v5.solids()) > 1:
        v5 = max(v5.solids(), key=lambda q: q.volume)
    return v5


def render(assets, finger):
    import pyvista as pv
    import trimesh
    fm = H._mesh(finger, 0.03)
    col = {"finger": "#2b2e33", "link7": "#c9ccd1"}

    def plot(path, items, view, up=(0, 0, 1), size=(1600, 1100), zoom=1.1):
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
    plot(IMG / "v5_finger.png", [(fm, col["finger"])], (0.45, -0.8, 0.7), up=(1, 0, 0), size=(1500, 1300), zoom=1.15)
    if not assets:
        return
    M = {"hardshell_finger_left": fm, "hardshell_tpu_left": trimesh.creation.box((0.1, 0.1, 0.1))}
    _, (lo, hi) = H.on_openarm(assets, M, 0.0)
    for name, frac in (("v5_open", 0.5), ("v5_closed", 0.0)):
        o, _ = H.on_openarm(assets, M, lo + frac * (hi - lo))
        items = [(o["link7"], col["link7"]), (o["finger_left"], col["finger"]), (o["finger_right"], col["finger"])]
        plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        gap = trimesh.proximity.closest_point(o["finger_left"], o["finger_right"].vertices)[1].min()
        hit = round(o["finger_left"].intersection(o["finger_right"]).volume, 1) if o["finger_left"].is_volume else -1
        print(f"  {name}: fingers {gap:.1f} mm apart at the nearest point; overlap {hit} mm3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the renders and checks on OpenArm)")
    a = ap.parse_args()
    v5 = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("v5_finger_left", v5), ("v5_finger_right", mirror(v5, Plane.XZ))):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    render(a.openarm, v5)


if __name__ == "__main__":
    main()
