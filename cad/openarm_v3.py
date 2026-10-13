"""OpenArm V3 fingers: V1 (openarm_hardshell.py) in one material. The same outer shape (OpenArm's printed base, the
stock flow, the chopstick-style jaw with its chisel nose and scallops), but what was the TPU soft part is solid PLA,
part of the same print: one plain part per finger, no gluing.

    python openarm_v3.py [--openarm /path/to/handumi-sw/assets]
"""
import argparse

from build123d import Plane, export_step, export_stl, mirror

import openarm_hardshell as H

OUT = H.OUT / "v3"
IMG = H.IMG


def build():
    """V1's hard and soft parts, fused into one solid."""
    hard, soft = H.build()
    finger = hard + soft
    if len(finger.solids()) > 1:
        finger = max(finger.solids(), key=lambda q: q.volume)
    return finger


def render(assets, finger):
    import pyvista as pv
    import trimesh
    fm = H._mesh(finger, 0.03)
    col = {"finger": "#e9edf1", "link7": "#c9ccd1"}

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
    plot(IMG / "v3_finger.png", [(fm, col["finger"])], (0.15, -1.0, 0.25), up=(0, 0, -1), size=(1800, 900), zoom=1.2)
    if not assets:
        return
    M = {"hardshell_finger_left": fm, "hardshell_tpu_left": trimesh.creation.box((0.1, 0.1, 0.1))}
    _, (lo, hi) = H.on_openarm(assets, M, 0.0)
    for name, frac in (("v3_open", 0.5), ("v3_closed", 0.0)):
        o, _ = H.on_openarm(assets, M, lo + frac * (hi - lo))
        items = [(o["link7"], col["link7"]), (o["finger_left"], col["finger"]), (o["finger_right"], col["finger"])]
        plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        gap = trimesh.proximity.closest_point(o["finger_left"], o["finger_right"].vertices)[1].min()
        hit = round(o["finger_left"].intersection(o["finger_right"]).volume, 1)
        print(f"  {name}: fingers {gap:.1f} mm apart at the nearest point; overlap {hit} mm3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the renders and checks on OpenArm)")
    a = ap.parse_args()
    finger = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("v3_finger_left", finger), ("v3_finger_right", mirror(finger, Plane.XZ))):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    render(a.openarm, finger)


if __name__ == "__main__":
    main()
