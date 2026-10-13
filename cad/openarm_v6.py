"""OpenArm V6 fingers: the "fine tip" flat finger (fine_tip_flange_left/right.stp) on OpenArm v1's printed finger base,
one plain printed part per finger.

The fine-tip finger is a flat triangle, 78 mm long, its straight edge the gripping face (49 x 33.6 mm at the root),
its curved back opened into ribbed cells, on a 3.5 mm flange with slotted holes for its own carriage. Here:
  * its flange is cut off (everything below the cells' floor) and the finger stands on OpenArm's base instead: the
    plate, its four M2.5 holes and the rib, the plate carried down to the gripping face (one plane with it, as V5's)
    and out under the whole finger root, so the cells have a floor;
  * its gripping edge is the gripping face (GRIP_Z, the fingers close 0.2 mm apart);
  * across the finger it sits SHIFT off the plate's centre, so OpenArm's four screw heads land in the cells (clear of
    the web through the middle of the finger); where the rib between the cells crosses the upper two heads, a
    key slot (KEY) is cut through it.

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).
The source STEP files are read from cad/source/fine_tip/ (or --src).

    python openarm_v6.py [--src DIR] [--openarm /path/to/handumi-sw/assets]
"""
import argparse
from pathlib import Path

from build123d import Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, fillet, import_step, mirror

import openarm_hardshell as H

OUT = H.OUT / "v6"
IMG = H.IMG
SRC = Path(__file__).resolve().parent / "source" / "fine_tip"
GRIP_Z = 24.8              # the gripping face: 0.2 mm apart when closed, as the other fingers
FLANGE_Y = 3.5             # in the source frame: the flange is y < 3.5 (the cells' floor); the finger starts there
WIDTH_X = 24.5             # in the source frame: the gripping edge is the plane x = -24.5
SHIFT = 15.2               # source z of the finger frame's y = 0: the lower screw heads' row in the lower cells
KEY = (5.0, 26.0)          # key slots through the rib over the upper screw heads: (diameter, length from the plate)


def fine_tip(src):
    """The fine-tip finger without its flange, in the finger frame: source y -> x (along), z -> y (across),
    x -> z (through, the gripping edge at GRIP_Z)."""
    s = import_step(str(Path(src) / "fine_tip_flange_left.stp"))
    s = s & (Pos(0, FLANGE_Y + 100, 0) * Box(400, 200, 400))
    x1 = H.PLATE["x"][1]
    loc = Plane(origin=(x1 - FLANGE_Y, -SHIFT, GRIP_Z + WIDTH_X), x_dir=(0, 0, 1), z_dir=(0, 1, 0)).location
    return loc * s


def base_to_finger(finger):
    """OpenArm's plate carried down to the gripping face and out under the finger's root, so the cells have a floor."""
    (x0, x1), (y0, y1), (z0, z1) = H.PLATE["x"], H.PLATE["y"], H.PLATE["z"]
    root = (finger & (Pos(x1 + 0.5, 0, 0) * Box(1.0, 400, 400))).bounding_box()
    ya, yb = min(y0, root.min.Y), max(y1, root.max.Y)
    za, zb = GRIP_Z, max(z1, root.max.Z)
    b = Pos((x0 + x1) / 2, (ya + yb) / 2, (za + zb) / 2) * Box(x1 - x0, yb - ya, zb - za)
    return fillet(b.edges().filter_by(Axis.X), H.PLATE["r"])


def build(src):
    finger = fine_tip(src)
    v6 = H.base() + base_to_finger(finger) + finger
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        v6 -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        v6 -= Pos(x1 - H.CBORE[1] / 2 + 0.01, y, z) * Rot(0, 90, 0) * Cylinder(H.CBORE[0] / 2, H.CBORE[1] + 0.02)
        v6 -= Pos(x1 + KEY[1] / 2, y, z) * Rot(0, 90, 0) * Cylinder(KEY[0] / 2, KEY[1])
    if len(v6.solids()) > 1:
        v6 = max(v6.solids(), key=lambda q: q.volume)
    return v6


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
    plot(IMG / "v6_finger.png", [(fm, col["finger"])], (0.45, -0.8, 0.7), up=(1, 0, 0), size=(1500, 1300), zoom=1.15)
    if not assets:
        return
    M = {"hardshell_finger_left": fm, "hardshell_tpu_left": trimesh.creation.box((0.1, 0.1, 0.1))}
    _, (lo, hi) = H.on_openarm(assets, M, 0.0)
    for name, frac in (("v6_open", 0.5), ("v6_closed", 0.0), (None, 1.0)):
        o, _ = H.on_openarm(assets, M, lo + frac * (hi - lo))
        if name:
            items = [(o["link7"], col["link7"]), (o["finger_left"], col["finger"]), (o["finger_right"], col["finger"])]
            plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        gap = trimesh.proximity.closest_point(o["finger_left"], o["finger_right"].vertices)[1].min()
        hit = round(o["finger_left"].intersection(o["finger_right"]).volume, 1) if o["finger_left"].is_volume else -1
        arm = min(trimesh.proximity.closest_point(o["link7"], o[f"finger_{s}"].vertices)[1].min() for s in ("left", "right"))
        print(f"  {name or 'fully open'}: fingers {gap:.1f} mm apart at the nearest point; overlap {hit} mm3; "
              f"{arm:.1f} mm from link7")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=str(SRC), help="folder with fine_tip_flange_left.stp")
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the renders and checks on OpenArm)")
    a = ap.parse_args()
    v6 = build(a.src)
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("v6_finger_left", v6), ("v6_finger_right", mirror(v6, Plane.XZ))):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    render(a.openarm, v6)


if __name__ == "__main__":
    main()
