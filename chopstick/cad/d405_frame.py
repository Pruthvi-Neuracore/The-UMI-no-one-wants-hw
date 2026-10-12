"""PG D405 frame: an Intel RealSense D405 in place of the Insta360 X5, on either base.

  front: the X5 frame's base interface, unchanged (the 8 mm slab and tongue that key into the base, with the two
         M3 x 55 clamp screws at (x, z) = (48.05, ±12.5)), so it fits the handheld and the robot base alike,
  top:   a cup for the D405 (42 x 42 x 23 mm), pitched 35° nose-down so the jaws and tips sit in its 7-50 cm depth
         range (tips about 190 mm away), held by 2 x M3 x 6 into the camera's back holes (20 mm apart), with cable
         windows in both side walls,
  back:  a plate with the OpenArm adapter's four M3 holes at (y, z) = (±9, ±26.9), so on the robot the adapter bolts
         straight on; a single central web joins it to the front, leaving the screw heads open from the sides for a
         hex key. On the handheld the back plate is just the PG-badged spine.

    python d405_frame.py
"""
import math

import numpy as np
from build123d import Box, Cylinder, Polygon, Pos, Rot, export_step, export_stl, extrude

from parts import HW, load_raw

CAM = (42.0, 42.0, 23.0)          # D405 body: width (y), height, depth (along view)
CLEAR = 0.4
WALL, FLOOR = 2.5, 3.0
PITCH = 35.0                      # view axis, degrees below horizontal
CENTRE = (30.0, 0.0, 54.0)        # camera body centre, gripper frame (mm)
SCREWS = 20.0                     # D405 back M3 holes, spacing along y
BACK = dict(x=(0.0, 6.0), y=15.5, z=(-33.0, 31.0))
INSERTS = [(9.0, 26.9), (-9.0, 26.9), (9.0, -26.9), (-9.0, -26.9)]
WEB_Y = 4.0


def cam_loc():
    """Location of the camera body frame: origin at its centre, local +Z the view axis, local Y = gripper Y."""
    phi = 90.0 + PITCH            # turns local +Z to (cos p, 0, -sin p)
    return Pos(*CENTRE) * Rot(0, phi, 0)


def view_dir():
    p = math.radians(PITCH)
    return np.array([math.cos(p), 0.0, -math.sin(p)])


def camera_envelope():
    return cam_loc() * Box(CAM[1], CAM[0], CAM[2])


def cup():
    w, h, d = CAM[0] + 2 * CLEAR, CAM[1] + 2 * CLEAR, CAM[2]
    outer = Pos(0, 0, -d / 2 - FLOOR + (d + FLOOR) / 2 - 2.0) * Box(h + 2 * WALL, w + 2 * WALL, d + FLOOR - 4.0)
    pocket = Pos(0, 0, 5.0) * Box(h, w, d + 10.0)                              # open at the front
    c = outer - pocket
    for s in (-1, 1):                                                         # camera screws through the floor
        c -= Pos(0, s * SCREWS / 2, -d / 2 - FLOOR / 2) * Cylinder(1.7, FLOOR + 2)
        c -= Pos(-4.0, s * (w / 2 + WALL / 2), 0) * Box(16.0, WALL + 2, 12.0)   # cable windows
    return cam_loc() * c


def cup_corner(lx, lz):
    """(x, z) of a point given in the camera body frame's (X, Z), on the y = 0 plane."""
    p = cam_loc() * Pos(lx, 0, lz)
    return p.position.X, p.position.Z


def support(width=28.0):
    """Solid wedge under the cup: from the slab's top and the back plate up to the cup's bottom face.

    It stays below the cup's bottom outer face and behind the camera's front face, so it is outside the D405's view.
    """
    h = CAM[1] / 2 + CLEAR + WALL
    fb = cup_corner(h, CAM[2] / 2 - 4.0)          # cup bottom face, front edge
    bb = cup_corner(h, -CAM[2] / 2 - FLOOR)       # cup bottom face, back edge
    pts = [(BACK["x"][0], 22.0), (39.5, 22.0), (39.5, 30.5), (fb[0], fb[1] + 0.5), (bb[0], bb[1] + 0.5),
           (BACK["x"][0], BACK["z"][1])]
    return Pos(0, width / 2, 0) * Rot(90, 0, 0) * extrude(Polygon(*pts, align=None), width)


def build():
    lite = load_raw()["x5_lite_frame"]
    interface = lite & (Pos(39.5 + 5, 0, 0) * Box(10, 80, 120))              # slab + tongue, x >= 39.5
    x0, x1 = BACK["x"]
    z0, z1 = BACK["z"]
    back = Pos((x0 + x1) / 2, 0, (z0 + z1) / 2) * Box(x1 - x0, 2 * BACK["y"], z1 - z0)
    web = Pos((x1 + 40.0) / 2, 0, (z0 + 4 + 31.0) / 2) * Box(40.0 - x1 + 0.5, 2 * WEB_Y, 31.0 - z0 - 4)
    body = interface + back + web + support() + cup()
    body -= cam_loc() * Pos(0, 0, 5.0) * Box(CAM[1] + 2 * CLEAR, CAM[0] + 2 * CLEAR, CAM[2] + 10.0)  # camera space
    for y, z in INSERTS:
        body -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(1.7, x1 - x0 + 2)
    return body


def main():
    from restyle import restyle
    p = restyle("d405_frame", build())
    export_step(p, str(HW / "STEP" / "d405_frame.step"))
    export_stl(p, str(HW / "STL" / "d405_frame.stl"), tolerance=0.02, angular_tolerance=0.15)
    bb = p.bounding_box()
    print(f"d405_frame {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
          f"{len(p.solids())} solid(s)")


if __name__ == "__main__":
    main()
