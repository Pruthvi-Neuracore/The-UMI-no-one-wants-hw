"""OpenArm V4 fingers: the ARX-X5 gripper finger (murobotics-ai/handumi-hw, `gripper_tips/ARX-X5-2023`, Apache-2.0)
on OpenArm v1's printed finger base, one plain printed part per finger.

The HandUMI tip is the ARX-X5 finger, a flat, tapering triangular blade, on a 61 x 29 mm flange plate for HandUMI's tip
flange. Here the flange plate is cut off and the finger is put on OpenArm's base:
  * turned so its broad flat gripping face faces the other finger, and tilted 6.5 deg so that face runs exactly along
    the approach: the two fingers close flat on each other (GRIP_Z, 0.2 mm apart when the gripper is closed);
  * its root (61 mm across, like OpenArm's plate) fused to the base: the plate, its four M2.5 holes, the rib;
  * straight access bores from the four screw heads along the finger, until they come out of it.

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).

    python openarm_v4.py [--openarm /path/to/handumi-sw/assets]
"""
import argparse
import math
from pathlib import Path

import numpy as np
from build123d import Box, Cylinder, Mesher, Plane, Pos, Rot, export_step, export_stl, mirror

import openarm_hardshell as H

OUT = H.OUT / "v4"
IMG = H.IMG
SRC = Path(__file__).resolve().parent / "source" / "arx_x5" / "ARX-X5-2023-LEFT-Gripper.stl"
GRIP_Z = 24.8              # the gripping face: 0.2 mm apart when closed, as the other fingers
FLANGE_X = 0.0             # in the tip's own frame, the HandUMI flange plate is x < 0; the finger starts here
TILT_DEG = None            # the gripping face's tilt to the finger's length (measured from the mesh when None)
BORE_D, BORE_LEN = 5.0, 90.0


def arx_finger():
    """The ARX-X5 finger without its flange plate, in the finger frame (a build123d solid)."""
    import trimesh
    m = trimesh.load(SRC)
    n, a = m.face_normals, m.area_faces
    top = n[:, 2] > 0.8                                         # the broad flat gripping face (+z in the tip frame)
    nt = (n[top] * a[top][:, None]).sum(0)
    nt /= np.linalg.norm(nt)
    tilt = TILT_DEG if TILT_DEG is not None else math.degrees(math.atan2(-nt[0], nt[2]))
    solid = Mesher().read(str(SRC))[0]
    solid = solid & (Pos(FLANGE_X + 100, 0, 0) * Box(200, 400, 400))
    # flip it 180 deg about x (the gripping face now faces -z, towards the other finger), then level the face
    solid = Rot(0, -tilt, 0) * Rot(180, 0, 0) * solid
    bb = solid.bounding_box()
    x1 = H.PLATE["x"][1] - 0.3
    solid = Pos(x1 - bb.min.X, -(bb.min.Y + bb.max.Y) / 2, GRIP_Z - bb.min.Z) * solid
    return solid, tilt


def build():
    finger, tilt = arx_finger()
    v4 = H.base() + finger
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        v4 -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        v4 -= Pos(x1 - H.CBORE[1] + BORE_LEN / 2, y, z) * Rot(0, 90, 0) * Cylinder(max(H.CBORE[0], BORE_D) / 2, BORE_LEN)
    if len(v4.solids()) > 1:
        v4 = max(v4.solids(), key=lambda q: q.volume)
    return v4, tilt


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
    v4, tilt = build()
    print(f"ARX-X5 finger: gripping face levelled by {tilt:.1f} deg")
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
