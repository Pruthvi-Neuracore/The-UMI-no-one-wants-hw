"""Insta360 X5 mount for the HandUMI camera post: the only part that differs from the D405 build.

It hangs on the same hinge as `d405_wrist_mount` (hinge.py): two Ø12 lugs on an M4 x 30 bolt + nyloc nut, straddling
the main support's 10 mm centre knuckle. Above them, a cradle holds the X5 upright, its lenses facing forward and back
and clear above everything, fixed by the camera's own 1/4"-20 socket:

  * base plate 6 mm thick with a Ø6.6 hole and a Ø12.5 x 4 counterbore from below for a low-head 1/4"-20 x 3/8"
    camera screw (the head sits between the lugs, clear of the knuckle),
  * front and back walls 20 mm high (the lens faces), side walls with a 24 mm slot for the ports and the battery door,
  * the hinge lets you tilt the camera; upright (TILT_DEG = 0) its weight sits over the pivot.

Hinge frame "Hf" (this file): origin on the hinge axis at the knuckle's centre, +z up (in use), y along the hinge axis,
-x forward (the fingers' side). hinge_to_m() places it in HandUMI's main-support frame M (-Z up).

    python x5_wrist_mount.py
"""
import sys
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Pos, Rot, export_step, export_stl, fillet

sys.path.append(str(Path(__file__).resolve().parent))
import hinge as H  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
X5 = (37.6, 46.0, 124.0)            # Insta360 X5 body: thickness (lens axis), width, height (mm)
CLEAR, WALL, CORNER_R = 0.4, 2.5, 5.0
PLATE_Z, PLATE_T = 9.0, 6.0          # base plate underside above the hinge axis, thickness
WALL_H, PORT_SLOT = 20.0, 24.0
SCREW_D, CBORE = (6.6, (12.5, 4.0))  # 1/4"-20 clearance, counterbore (dia, depth) from below
TILT_DEG = 0.0                       # upright


def build():
    r = H.KNUCKLE_OD / 2
    x_in = H.CENTRE_W / 2 + H.GAP
    hx, hy = X5[0] / 2 + CLEAR + WALL, X5[1] / 2 + CLEAR + WALL
    z0, z1 = PLATE_Z, PLATE_Z + PLATE_T
    body = Pos(0, 0, (z0 + z1 + WALL_H) / 2) * Box(2 * hx, 2 * hy, z1 - z0 + WALL_H)
    body = fillet(body.edges().filter_by(Axis.Z), CORNER_R)
    pocket = Pos(0, 0, z1 + WALL_H / 2 + 1) * Box(X5[0] + 2 * CLEAR, X5[1] + 2 * CLEAR, WALL_H + 2)
    body -= fillet(pocket.edges().filter_by(Axis.Z), 3.0)
    for sy in (-1, 1):                                            # port / battery-door slots in the side walls
        body -= Pos(0, sy * hy, z1 + WALL_H / 2 + 1) * Box(PORT_SLOT, 2 * WALL + 2, WALL_H + 2)
    # lugs: Ø12 on the hinge axis, each with a neck up into the plate
    for s in (-1, 1):
        y0, y1 = sorted((s * x_in, s * (x_in + H.LUG_W)))
        w = y1 - y0
        lug = Pos(0, (y0 + y1) / 2, 0) * Rot(90, 0, 0) * Cylinder(r, w)
        lug += Pos(0, (y0 + y1) / 2, (z0 + 1) / 2) * Box(2 * r, w, z0 + 1)
        body += lug
    for s in (-1, 1):                                             # M4 pivot through both lugs
        body -= Pos(0, s * (x_in + H.LUG_W / 2), 0) * Rot(90, 0, 0) * Cylinder(H.BOLT_D / 2, H.LUG_W + 2)
    body -= Pos(0, 0, (z0 + z1) / 2) * Cylinder(SCREW_D / 2, PLATE_T + 2)
    body -= Pos(0, 0, z0 + CBORE[1] / 2 - 0.5) * Cylinder(CBORE[0] / 2, CBORE[1] + 1)
    return body


def camera_in_hinge(x5_shape_upright_bottom_centred):
    """The X5 (in its upright frame: bottom centre at the origin, lens axis x, +z up) placed in the cradle."""
    return Pos(0, 0, PLATE_Z + PLATE_T) * x5_shape_upright_bottom_centred


def hinge_to_m(tilt_deg=TILT_DEG):
    """4x4 taking hinge-frame points into the main-support frame M (-Z up). Tilt: positive leans the camera forward."""
    a = np.radians(tilt_deg)
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)    # Hf z (up) -> M -Z, keeps right-handedness
    Ry = np.array([[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]])  # lean towards -x
    T = np.eye(4)
    T[:3, :3] = R @ Ry
    T[:3, 3] = [H.AXIS_M[0], H.Y_CENTRE, H.AXIS_M[1]]
    return T


def main(out=ROOT / "hardware"):
    part = build()
    for side in ("right", "left"):                                # same part for both hands
        (out / "STEP" / side).mkdir(parents=True, exist_ok=True)
        (out / "STL" / side).mkdir(parents=True, exist_ok=True)
        export_step(part, str(out / "STEP" / side / "x5_wrist_mount.step"))
        export_stl(part, str(out / "STL" / side / "x5_wrist_mount.stl"), tolerance=0.02, angular_tolerance=0.1)
    bb = part.bounding_box()
    print(f"x5_wrist_mount {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {part.volume / 1e3:.1f} cm3, "
          f"{len(part.solids())} solid(s)")
    return part


if __name__ == "__main__":
    main()
