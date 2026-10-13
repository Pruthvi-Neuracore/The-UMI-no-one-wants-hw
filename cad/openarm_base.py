"""OpenArm finger base on its own: OpenArm v1's printed finger base (the mounting plate with its four M2.5 holes,
counterbored on the finger side, and the rib behind it), as used by every finger here. Useful as a starting block for
your own fingers.

    python openarm_base.py
"""
from build123d import Cylinder, Plane, Pos, Rot, export_step, export_stl, mirror

import openarm_hardshell as H

OUT = H.OUT / "base"


def build():
    b = H.base()
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        b -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        b -= Pos(x1 - H.CBORE[1] / 2 + 0.01, y, z) * Rot(0, 90, 0) * Cylinder(H.CBORE[0] / 2, H.CBORE[1] + 0.02)
    return b


def main():
    b = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in (("openarm_finger_base_left", b), ("openarm_finger_base_right", mirror(b, Plane.XZ))):
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")


if __name__ == "__main__":
    main()
