"""PG D405 frame: an Intel RealSense D405 on the same tilting camera mount as the HandUMI build.

The camera sits in the HandUMI D405 cup (`d405_cup`, used unchanged from the main branch: a cup with two Ø12 hinge lugs,
triangulated gussets, side windows and a cable window in its top wall so the USB cable leaves from the top). The cup
hangs on this frame's centre knuckle, the same Ø12 x 10 mm knuckle with an M4 bolt as on the HandUMI main support,
so the camera tilts: loosen the M4 x 30 bolt, set the angle, tighten the nyloc nut.

  front: the X5 frame's base interface, unchanged (the 8 mm slab and tongue that key into the base, with the two
         M3 x 55 clamp screws at (x, z) = (48.05, ±12.5)), so it fits the handheld and the robot base alike,
  back:  a plate with the robot mounts' four M3 holes at (y, z) = (±9, ±26.9), so on the robot the OpenArm adapter or
         the ISO 50 plate bolts straight on; a central web joins it to the front,
  top:   the hinge knuckle, on top of the back plate, axis along the gripper's Y at HINGE.

TILT_DEG is the camera's view direction below horizontal (35° default: the jaws and tips sit in the D405's 7-50 cm
depth range, the tips about 190 mm away). The cup clears the base and jaws from TILT_RANGE[0] to TILT_RANGE[1]
(checked by assembly.py --check).

    python d405_frame.py
"""
import math

import numpy as np
import trimesh
from build123d import Box, Cylinder, Pos, Rot, export_step, export_stl

from parts import HERE, HW, load_raw

CUP_STL = HERE / "source" / "handumi" / "d405_cup.stl"   # HandUMI d405_wrist_mount, its camera-mount frame
CUP_AXIS = (-29.5, 1.5)          # hinge axis in the cup's frame: along x, at (y, z) (HandUMI hinge.AXIS_CAM)
CUP_FLOOR, CAM_DEPTH = 2.0, 23.0  # inner floor of the cup; the D405 sits on it and looks along the cup's +z
KNUCKLE_OD, KNUCKLE_W, BOLT_D = 12.0, 10.0, 4.4         # HandUMI hinge: centre knuckle, M4 clearance
HINGE = (2.4, 37.2)              # knuckle axis in the gripper frame (x, z), along y
TILT_DEG = 35.0
TILT_RANGE = (15.0, 65.0)
BACK = dict(x=(0.0, 6.0), y=15.5, z=(-33.0, 31.0))
INSERTS = [(9.0, 26.9), (-9.0, 26.9), (9.0, -26.9), (-9.0, -26.9)]
WEB_Y = 4.0
WEB_TOP = 8.0                    # kept low so the cup's floor and gussets swing clear over the whole tilt range


def cam_matrix(tilt_deg=TILT_DEG):
    """4x4 from the cup's frame to the gripper frame (mm) at a tilt: cup z (view) pitched tilt_deg below +X, cup x
    along +Y (the hinge), the hinge axis on HINGE."""
    p = math.radians(tilt_deg)
    R = np.column_stack([[0, 1.0, 0], [math.sin(p), 0, math.cos(p)], [math.cos(p), 0, -math.sin(p)]])
    T = np.eye(4)
    T[:3, :3] = R
    hinge = np.array([HINGE[0], 0.0, HINGE[1]])
    T[:3, 3] = hinge - R @ np.array([0.0, *CUP_AXIS])
    return T


def view_dir(tilt_deg=TILT_DEG):
    p = math.radians(tilt_deg)
    return np.array([math.cos(p), 0.0, -math.sin(p)])


def camera_frame(tilt_deg=TILT_DEG):
    """D405 camera frame in the gripper frame (mm): front-face centre, +X along the view axis, +Z up."""
    C = cam_matrix(tilt_deg)
    v = view_dir(tilt_deg)
    z = np.cross(v, [0, 1, 0])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([v, [0, 1, 0], z / np.linalg.norm(z)])
    T[:3, 3] = (C @ [0, 0, CUP_FLOOR + CAM_DEPTH, 1])[:3]
    return T


def cup(tilt_deg=TILT_DEG):
    """The HandUMI D405 cup (mesh) placed on the hinge at a tilt, in the gripper frame."""
    return trimesh.load(CUP_STL).apply_transform(cam_matrix(tilt_deg))


def camera_envelope(tilt_deg=TILT_DEG):
    """The D405 body (42 x 42 x 23 mm) as a mesh in the gripper frame."""
    box = trimesh.creation.box(extents=[42.0, 42.0, CAM_DEPTH])
    box.apply_translation([0, 0, CUP_FLOOR + CAM_DEPTH / 2])
    return box.apply_transform(cam_matrix(tilt_deg))


def build():
    lite = load_raw()["x5_lite_frame"]
    interface = lite & (Pos(39.5 + 5, 0, 0) * Box(10, 80, 120))              # slab + tongue, x >= 39.5
    x0, x1 = BACK["x"]
    z0, z1 = BACK["z"]
    back = Pos((x0 + x1) / 2, 0, (z0 + z1) / 2) * Box(x1 - x0, 2 * BACK["y"], z1 - z0)
    web = Pos((x1 + 40.0) / 2, 0, (z0 + 4 + WEB_TOP) / 2) * Box(40.0 - x1 + 0.5, 2 * WEB_Y, WEB_TOP - z0 - 4)
    hx, hz = HINGE
    r = KNUCKLE_OD / 2
    knuckle = Pos(hx, 0, hz) * Rot(90, 0, 0) * Cylinder(r, KNUCKLE_W)
    neck = Pos(hx, 0, (z1 + hz) / 2) * Box(KNUCKLE_OD, KNUCKLE_W, hz - z1 + 0.5)     # knuckle onto the back plate
    body = interface + back + web + neck + knuckle
    body -= Pos(hx, 0, hz) * Rot(90, 0, 0) * Cylinder(BOLT_D / 2, KNUCKLE_W + 2)
    for y, z in INSERTS:
        body -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(1.7, x1 - x0 + 2)
    return body


def main():
    from restyle import restyle
    p = restyle("d405_frame", build())
    export_step(p, str(HW / "STEP" / "d405_frame.step"))
    export_stl(p, str(HW / "STL" / "d405_frame.stl"), tolerance=0.02, angular_tolerance=0.15)
    cup().export(HW / "STL" / "d405_cup.stl")                 # placed at TILT_DEG, gripper frame
    ref = HW / "reference"
    ref.mkdir(exist_ok=True)
    camera_envelope().export(ref / "d405_envelope.stl")
    bb = p.bounding_box()
    print(f"d405_frame {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
          f"{len(p.solids())} solid(s); hinge at x={HINGE[0]}, z={HINGE[1]}; cup at {TILT_DEG}°")


if __name__ == "__main__":
    main()
