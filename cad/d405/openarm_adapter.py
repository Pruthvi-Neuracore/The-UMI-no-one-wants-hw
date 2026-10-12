"""OpenArm flange adapter: bolts the chopstick robot gripper's Insta360 X5 frame to OpenArm link7.

The upstream OpenArm plate (4 holes on a 26.9 mm square) does not match the link7 flange in the OpenArm description we
use (openarm_description/meshes/arm/v10/link7: 6 x M3 on a 50 mm PCD, face at link7 z = 94.5 mm, with a 1 mm high
Ø35 boss), which is also the pattern the stock OpenArm hand bolts to. So this plate is drawn against that flange:

  back  (x = -15): sits on the link7 face, Ø36.5 x 1.5 pocket over the boss, 6 x Ø3.4 holes on PCD 50 with Ø5.8
                   counterbores from the front (M3 x 16 socket head, installed first; the four counterbores at the
                   tongue's edges open to the side),
  front (x = 0):   a 31.6 mm wide tongue that fits between the X5 frame's two rear legs (which reach 4.8 mm behind the
                   mounting face), with 4 x M3 heat-set inserts under the frame's rear-wall holes at (y, z) = (±9, ±26.9).
                   The frame is screwed on from inside the camera cavity (camera out) with M3 x 10; open the frame's
                   four Ø2.8 holes to Ø3.2 first.

Orientation: like the stock OpenArm hand, the jaws open along link7 Y: the gripper's +X is link7 +Z, +Y is link7 -Y and
+Z (camera up) is link7 +X, which points up when the elbow is bent 90° from the zero pose (T_LINK7 below).

    python openarm_adapter.py
"""
import math

import numpy as np
from build123d import Axis, Box, Cylinder, Pos, Rot, export_step, export_stl, fillet

from parts import save_part

T = 15.0              # total thickness, back face at x = -T
TONGUE = 5.0          # tongue depth in front of the body (frame legs reach 4.8 mm behind x = 0)
TONGUE_W = 31.6       # between the legs at |y| = 16.3, 0.5 mm clearance each side
TONGUE_H = (-33.0, 31.0)
BODY_R = 31.0
PCD = 50.0
# (y, z) on PCD 50, every 60 deg starting on the Y axis
FLANGE_HOLES = [(round(PCD / 2 * math.cos(math.radians(a)), 2), round(PCD / 2 * math.sin(math.radians(a)), 2))
                for a in range(0, 360, 60)]
INSERTS = [(9.0, 26.9), (-9.0, 26.9), (9.0, -26.9), (-9.0, -26.9)]
CBORE_D, CBORE_FLOOR_X = 5.8, -6.0
LINK7_FACE_Z = 0.0945  # m, link7 frame

# Pose of the gripper frame (plate front face centre) in the OpenArm link7 frame, metres.
T_LINK7 = np.array([[0.0, 0.0, 1.0, 0.0],
                    [0.0, -1.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, LINK7_FACE_Z + T / 1000.0],
                    [0.0, 0.0, 0.0, 1.0]])


def along_x(shape):
    """Turn a Z-axis primitive so its axis runs along X."""
    return Rot(0, 90, 0) * shape


def build():
    body = Pos(-T + (T - TONGUE) / 2, 0, 0) * along_x(Cylinder(BODY_R, T - TONGUE))
    body = fillet(body.edges().filter_by(Axis.X, reverse=True).group_by(Axis.X)[0], 2.0)
    zc, zh = (TONGUE_H[0] + TONGUE_H[1]) / 2, TONGUE_H[1] - TONGUE_H[0]
    tongue = Pos(-TONGUE / 2 - 0.5, 0, zc) * Box(TONGUE + 1.0, TONGUE_W, zh)
    tongue = fillet(tongue.edges().filter_by(Axis.X), 3.0)
    plate = body + tongue
    # boss pocket on the back
    plate -= Pos(-T + 0.75, 0, 0) * along_x(Cylinder(36.5 / 2, 1.5))
    # flange screws: through holes and counterbores down to x = CBORE_FLOOR_X
    for y, z in FLANGE_HOLES:
        plate -= Pos(-T / 2, y, z) * along_x(Cylinder(1.7, T + 2))
        plate -= Pos(CBORE_FLOOR_X / 2 + 0.5, y, z) * along_x(Cylinder(CBORE_D / 2, -CBORE_FLOOR_X + 1.0))
        if abs(y) + CBORE_D / 2 > TONGUE_W / 2 - 2.0 and abs(y) < TONGUE_W / 2:
            # too close to the tongue's edge to leave a wall: open the counterbore to the side
            w = TONGUE_W / 2 - abs(y) + 1.0
            plate -= Pos(CBORE_FLOOR_X / 2 + 0.5, math.copysign(abs(y) + w / 2, y), z) * Box(-CBORE_FLOOR_X + 1.0, w, CBORE_D)
    # heat-set inserts for the X5 frame (M3 x 5.7: Ø4.0 x 6 deep)
    for y, z in INSERTS:
        plate -= Pos(-3.0 + 0.5, y, z) * along_x(Cylinder(2.0, 7.0))
    return plate


def main():
    from restyle import restyle
    p = restyle("openarm_adapter", build())
    save_part("openarm_adapter", p)
    bb = p.bounding_box()
    print(f"openarm_adapter {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3")


if __name__ == "__main__":
    main()
