"""OpenArm V4 fingers: a solid, flat, triangular blade after the ARX-X5 gripper finger (murobotics-ai/handumi-hw,
`gripper_tips/ARX-X5-2023`, Apache-2.0), on OpenArm v1's printed finger base, one plain printed part per finger.

The ARX-X5 finger is a flat blade, 61 mm wide at its root, tapering to an 11 mm tip, its broad face the gripping face.
The original is a pocketed tray (hollow under its top skin) and sits off-centre. Here it is:
  * solid: the blade is the ARX's plan outline (PLAN, measured from the STL in cad/source/arx_x5/) intersected with
    a side profile (SIDE), with no pockets;
  * centred on the mounting plate (the ARX outline drifts up to 4.6 mm to one side; here it is symmetric);
  * centred on the plate like OpenArm's own finger: its root runs between the two rows of screws (z 40.2-50.2, the
    plate's middle), then it sweeps down in a smooth S into the flat gripping face, from x 38 to the tip, tapering
    into the ARX's own thickness;
  * screw cutouts, as OpenArm's finger has: a rounded slot through the root over each column of screws (SLOT), so
    both heads are open from above and a ball-end hex key reaches them;
  * its broad flat face is the gripping face, at GRIP_Z (the fingers close 0.2 mm apart);
  * fused to OpenArm's base (the plate, its four holes and head seats, the rib).

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).

    python openarm_v4.py [--openarm /path/to/handumi-sw/assets]
"""
import argparse
import math
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, fillet, mirror

import openarm_hardshell as H

OUT = H.OUT / "v4"
IMG = H.IMG
SRC = Path(__file__).resolve().parent / "source" / "arx_x5" / "ARX-X5-2023-LEFT-Gripper.stl"
GRIP_Z = 24.8              # the gripping face: 0.2 mm apart when closed, as the other fingers
# the blade's half-width along x (the ARX-X5 finger's width, centred), to its rounded tip
PLAN = [(11.0, 29.6), (15.0, 26.9), (19.0, 24.2), (23.0, 21.5), (27.0, 19.7), (79.0, 5.35), (80.6, 4.0)]
# side profile (z along x): the root between the screw rows (ROOT_Z), the back easing down into the ARX's taper from
# BACK_X, the underside easing down into the flat gripping face (GRIP_Z) from FACE_X
ROOT_Z = (40.2, 50.2)
BACK_X = (14.0, 47.0)
BACK_TAPER = [(47.0, 34.8), (59.0, 32.3), (80.6, 28.0)]    # the ARX's back, 3.2 mm above the gripping face at the tip
FACE_X = (12.0, 38.0)
# the screw cutouts: a slot through the root over each column of screws (y = +-7.5), (width, length from the plate,
# floor z): it clears both head seats (4.8 mm) and gives a ball-end key ~25 deg of swing
SLOT = (5.6, 24.0, 37.6)
ROUND = (4.0, 1.4)         # plan and side corners rounded with these radii (the tip is only 3.2 mm thick)
EDGE_R = 1.2


def _rounded(pts, r):
    from shapely.geometry import Polygon
    pl = Polygon(pts).buffer(-r, join_style=1).buffer(r, join_style=1)
    return [(round(x, 3), round(y, 3)) for x, y in list(pl.simplify(0.02).exterior.coords)[:-1]]


def _ease(x, xa, xb, za, zb):
    t = min(max((x - xa) / (xb - xa), 0.0), 1.0)
    return za + (zb - za) * (1 - math.cos(math.pi * t)) / 2


def blade():
    """The solid blade: plan outline x side profile, its long edges rounded."""
    x0 = H.PLATE["x"][1] - 0.5                                  # into the mounting plate
    xb = x0 - 10.0                                              # (cut back to the plate after rounding)
    plan = [(xb, -30.5)] + [(x, -h) for x, h in PLAN] + [(x, h) for x, h in PLAN[::-1]] + [(xb, 30.5)]
    xs = np.linspace(xb, BACK_X[1], 40)
    top = [(x, _ease(x, *BACK_X, ROOT_Z[1], BACK_TAPER[0][1])) for x in xs] + BACK_TAPER[1:]
    xs = np.linspace(xb, PLAN[-1][0], 60)
    bottom = [(x, _ease(x, *FACE_X, ROOT_Z[0], GRIP_Z)) for x in xs]
    side = top + bottom[::-1]
    b = H._poly_prism(_rounded(plan, ROUND[0]), (0, 1)) & H._poly_prism(_rounded(side, ROUND[1]), (0, 2))
    b = b & H._poly_prism([(x0, -100), (200, -100), (200, 100), (x0, 100)], (0, 1))
    for r in (EDGE_R, 0.8):
        try:
            f = fillet(b.edges().filter_by(Axis.X, reverse=True).filter_by(lambda e: e.center().X > x0 + 1), r)
            if f.is_valid:
                return f
        except Exception:
            pass
    return b


def slots():
    """The screw cutouts: a round-ended slot through the root over each column of screws."""
    (_, x1) = H.PLATE["x"]
    w, length, z0 = SLOT
    out = None
    for y in sorted({y for y, _ in H.HOLES}):
        xa, xe = x1 - 0.01, x1 + length - w / 2
        s = Pos((xa + xe) / 2, y, z0 + 50) * Box(xe - xa, w, 100) + Pos(xe, y, z0 + 50) * Cylinder(w / 2, 100)
        out = s if out is None else out + s
    return out


def build():
    v4 = H.base() + (blade() - slots())
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        v4 -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        v4 -= Pos(x1 - H.CBORE[1] / 2 + 0.01, y, z) * Rot(0, 90, 0) * Cylinder(H.CBORE[0] / 2, H.CBORE[1] + 0.02)
    if len(v4.solids()) > 1:
        v4 = max(v4.solids(), key=lambda q: q.volume)
    return v4


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
    plot(IMG / "v4_finger.png", [(fm, col["finger"])], (0.3, -1.0, 0.35), up=(0, 0, -1), size=(1800, 1000), zoom=1.25)
    if not assets:
        return
    M = {"hardshell_finger_left": fm, "hardshell_tpu_left": trimesh.creation.box((0.1, 0.1, 0.1))}
    _, (lo, hi) = H.on_openarm(assets, M, 0.0)
    for name, frac in (("v4_open", 0.5), ("v4_closed", 0.0)):
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
    v4 = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("v4_finger_left", v4), ("v4_finger_right", mirror(v4, Plane.XZ))):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    render(a.openarm, v4)


if __name__ == "__main__":
    main()
