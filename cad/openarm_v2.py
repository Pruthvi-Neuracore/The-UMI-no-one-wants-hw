"""OpenArm v2 fingers: a solid, flat triangular finger for OpenArm v1, with a soft strip on its gripping edge.

The finger is a flat plate (FLAT_T thick near the mount, thicker over the middle: THICKNESS), drawn as a triangle: as tall as the mounting plate where it leaves it,
tapering to the tip, with a straight gripping edge so the two fingers close flat on each other. It is solid (not a
truss). Three gusset ribs at its root stiffen it sideways; they sit between the screw heads, so all four heads of the
mounting plate stay open from the front.

  * base: OpenArm v1's printed finger base (openarm_hardshell.base(): the plate, its four M2.5 holes and the rib);
  * the hard plate: the triangle, its long edges rounded, its gripping edge TIP_BACK behind the soft strip's face;
  * `v2_tpu_<side>` (TPU 95A): a strip along the gripping edge, STRIP_T deep, with a tongue keyed into the plate
    (glue it in); the last TIP_T of the edge is the hard tip.

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).

    python openarm_v2.py [--openarm /path/to/handumi-sw/assets]
"""
import argparse
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, fillet, mirror

import openarm_hardshell as H

OUT = H.OUT / "v2"
IMG = H.IMG
GRIP_Z = 24.8              # the soft strip's face: the fingers close to 0.2 mm apart, as the other fingers
FLAT_T = 10.0              # the plate's thickness (across the finger): the screw heads at y = +-7.5 stay clear
# root ribs: three horizontal gussets at the root, ROOT_HALF each side of centre at the mounting plate, tapering into
# the triangle over ROOT_LEN; they sit in the bands between the screw heads (z 37.8-42.6 and 47.8-52.6), so the heads
# stay open from the front, and they carry the sideways load where it is largest
ROOT_HALF, ROOT_LEN = 12.0, 34.0
# the plate's half-thickness along x: FLAT_T / 2 for the first 26 mm (an L-key's short arm reaches the screw heads
# straight on), then thicker over the middle where the bending is largest, thinning again to the tip
THICKNESS = [(6.1, 5.0), (26.0, 5.0), (45.0, 8.0), (80.0, 8.0), (115.0, 5.5)]
ROOT_BANDS = [(None, 37.0), (43.4, 47.0), (53.4, 59.6)]   # z ranges (None: from the gripping edge)
TIP_X = 115.0
TIP_H = 11.0               # the tip's height (from the gripping edge)
ROOT_TOP = 59.6            # the triangle's root runs up the mounting plate to its top
STRIP = (40.0, TIP_X - 1.5)  # the soft strip's length (the last TIP_T is the hard tip)
STRIP_T = 4.0
TIP_T, TIP_BACK = 1.5, 0.6
TONGUE = dict(half=2.0, h=2.5)
EDGE_R = 1.5


def plate():
    """The flat triangle (hard)."""
    x0 = H.PLATE["x"][1] - 0.5                                  # into the mounting plate
    tri = [(x0, GRIP_Z), (TIP_X, GRIP_Z), (TIP_X, GRIP_Z + TIP_H), (x0, ROOT_TOP)]
    plan = [(x, -h) for x, h in THICKNESS] + [(x, h) for x, h in THICKNESS[::-1]]
    plan[0] = (x0, plan[0][1])
    plan[-1] = (x0, plan[-1][1])
    from shapely.geometry import Polygon
    pl = Polygon(plan).buffer(-3.0, join_style=1).buffer(3.0, join_style=1)     # round the taper's corners
    pts = [(round(x, 3), round(y, 3)) for x, y in list(pl.simplify(0.02).exterior.coords)[:-1]]
    p = H._poly_prism(tri, (0, 2)) & H._poly_prism(pts, (0, 1))
    for r in (EDGE_R, 1.0):
        try:
            b = fillet(p.edges().filter_by(Axis.Y, reverse=True).filter_by(lambda e: e.center().X > x0 + 1), r)
            if b.is_valid:
                return b
        except Exception:
            pass
    return p


def root_ribs():
    """The three root gussets: a wedge in plan (ROOT_HALF at the mounting plate to the plate's half-thickness), cut to
    the bands between the screw heads."""
    x0 = H.PLATE["x"][1] - 0.5
    wedge = H._poly_prism([(x0, -ROOT_HALF), (x0 + ROOT_LEN, -FLAT_T / 2), (x0 + ROOT_LEN, FLAT_T / 2), (x0, ROOT_HALF)],
                          (0, 1))
    ribs = None
    for z0, z1 in ROOT_BANDS:
        z0 = GRIP_Z + TIP_BACK if z0 is None else z0
        band = wedge & (Pos(0, 0, (z0 + z1) / 2) * Box(600, 100, z1 - z0))
        ribs = band if ribs is None else ribs + band
    for r in (1.0, 0.6):
        try:
            b = fillet(ribs.edges().filter_by(Axis.Y, reverse=True).filter_by(lambda e: e.center().X > x0 + 1), r)
            if b.is_valid:
                return b
        except Exception:
            pass
    return ribs


def build():
    """(hard, soft)."""
    tri = plate()
    s0, s1 = STRIP
    strip = tri & (Pos((s0 + s1) / 2, 0, GRIP_Z + STRIP_T / 2) * Box(s1 - s0, 40, STRIP_T))
    t = TONGUE
    tongue = Pos((s0 + s1) / 2, 0, GRIP_Z + STRIP_T + t["h"] / 2 - 0.05) * Box(s1 - s0 - 12, 2 * t["half"], t["h"] + 0.1)
    soft = strip + tongue
    hard = tri - soft
    hard -= Pos(0, 0, GRIP_Z - 5 + TIP_BACK) * Box(600, 100, 10)        # hard edge just behind the soft face
    hard = H.base() + hard + root_ribs()
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:                                               # through holes, head seats in the plate
        hard -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        hard -= Pos(x1 - H.CBORE[1] / 2 + 0.01, y, z) * Rot(0, 90, 0) * Cylinder(H.CBORE[0] / 2, H.CBORE[1] + 0.02)
    if len(hard.solids()) > 1:
        hard = max(hard.solids(), key=lambda q: q.volume)
    return hard, soft


def render(assets, parts):
    import pyvista as pv
    import trimesh
    meshes = {n: H._mesh(p, 0.03) for n, p in parts.items()}
    col = {"hard": "#2b2e33", "soft": "#9aa3ad", "link7": "#c9ccd1"}         # black plate, grey TPU strip

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
    hm, sm = meshes["v2_finger_left"], meshes["v2_tpu_left"]
    plot(IMG / "v2_finger.png", [(hm, col["hard"]), (sm, col["soft"])], (0.3, -1.0, 0.35), up=(0, 0, -1),
         size=(1800, 1000), zoom=1.25)
    if not assets:
        return
    M = {"hardshell_finger_left": hm, "hardshell_tpu_left": sm}
    _, (lo, hi) = H.on_openarm(assets, M, 0.0)
    for name, frac in (("v2_open", 0.5), ("v2_closed", 0.0)):
        o, _ = H.on_openarm(assets, M, lo + frac * (hi - lo))
        items = [(o["link7"], col["link7"])] + [(o[k], col["hard" if k.startswith("finger") else "soft"])
                                                for k in o if k != "link7"]
        plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        gap = trimesh.proximity.closest_point(o["tpu_left"], o["tpu_right"].vertices)[1].min()
        hits = [round(o[a].intersection(o[b]).volume, 1) for a, b in
                (("finger_left", "finger_right"), ("tpu_left", "tpu_right"), ("finger_left", "tpu_right"))]
        print(f"  {name}: soft strips {gap:.1f} mm apart; overlaps hard/hard, soft/soft, hard/soft: {hits} mm3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the renders and checks on OpenArm)")
    a = ap.parse_args()
    hard, soft = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    parts = {"v2_finger_left": hard, "v2_tpu_left": soft,
             "v2_finger_right": mirror(hard, Plane.XZ), "v2_tpu_right": mirror(soft, Plane.XZ)}
    for n, p in parts.items():
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    print(f"hard / soft overlap {(hard & soft).volume if (hard & soft).solids() else 0:.1f} mm3")
    render(a.openarm, parts)


if __name__ == "__main__":
    main()
