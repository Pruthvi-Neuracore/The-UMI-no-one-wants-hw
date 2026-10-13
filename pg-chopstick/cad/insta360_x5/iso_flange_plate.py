"""ISO 9409-1-50-4-M6 flange plate: the PG Chopstick robot gripper on any cobot with the standard 50 mm tool flange
(UR e-series, ABB GoFa CRB 15000, FANUC CRX, and many more).

  back  (x = -T): sits on the tool flange. A Ø31.45 x 3 mm centring spigot for the flange's Ø31.5 H7 centring bore,
                  4 x Ø6.6 holes on PCD 50 at 45° to the gripper axes, counterbored Ø11 from the front for
                  M6 x 14 socket heads (installed first), and a Ø6 dowel hole on the PCD (Ø6 x 10 pin) that sets the
                  orientation. Counterbores that land on the tongue's edges are opened to the side.
  front (x = 0):  the same interface as openarm_adapter: a 31.6 mm tongue between the X5 frame's rear legs (or flat on
                  the D405 frame's back plate) with 4 x M3 heat-set inserts at (y, z) = (±9, ±26.9); frame screwed on
                  from inside with M3 x 10.

Orientation (T_FLANGE below): gripper +X along the flange +Z (tool approach), +Y along flange +Y, +Z (camera up) along
flange -X. With the usual tool0 convention (UR, FANUC, ABB descriptions), flange +X points down when the arm reaches
forward, so this puts the camera on top. The bolt pattern repeats every 90°, so the plate can be fitted in four rolls;
the dowel picks one. If your tool frame differs, change T_FLANGE (it is the only robot-specific transform the
retargeting needs).

    python iso_flange_plate.py
"""
import math

import numpy as np
from build123d import Axis, Box, Cylinder, Pos, export_step, export_stl, fillet

from openarm_adapter import INSERTS, TONGUE, TONGUE_H, TONGUE_W, along_x
from parts import save_part

T = 16.0
BODY_R = 32.0                       # Ø64, just over the flange's Ø63
PCD = 50.0
BOLTS = [(PCD / 2 * math.cos(math.radians(a)), PCD / 2 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
BOLT_D, CBORE_D, CBORE_FLOOR_X = 6.6, 11.0, -T + 8.0          # 8 mm clamped under the head
SPIGOT_D, SPIGOT_H = 31.45, 3.0
DOWEL = (PCD / 2, 0.0)              # on the PCD, on the +Y axis (between two bolts)
DOWEL_D, DOWEL_DEPTH = 6.1, 6.0

T_FLANGE = np.array([[0.0, 0.0, -1.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [1.0, 0.0, 0.0, T / 1000.0],
                     [0.0, 0.0, 0.0, 1.0]])


def build():
    body = Pos(-T + (T - TONGUE) / 2, 0, 0) * along_x(Cylinder(BODY_R, T - TONGUE))
    body = fillet(body.edges().filter_by(Axis.X, reverse=True).group_by(Axis.X)[-1], 2.0)
    zc, zh = (TONGUE_H[0] + TONGUE_H[1]) / 2, TONGUE_H[1] - TONGUE_H[0]
    tongue = Pos(-TONGUE / 2 - 0.5, 0, zc) * Box(TONGUE + 1.0, TONGUE_W, zh)
    tongue = fillet(tongue.edges().filter_by(Axis.X), 3.0)
    spigot = Pos(-T - SPIGOT_H / 2, 0, 0) * along_x(Cylinder(SPIGOT_D / 2, SPIGOT_H))
    spigot = fillet(spigot.edges().group_by(Axis.X)[0], 0.6)
    plate = body + tongue + spigot
    for y, z in BOLTS:
        plate -= Pos(-T / 2, y, z) * along_x(Cylinder(BOLT_D / 2, T + 2))
        plate -= Pos(CBORE_FLOOR_X / 2 + 0.5, y, z) * along_x(Cylinder(CBORE_D / 2, -CBORE_FLOOR_X + 1.0))
        if abs(y) - CBORE_D / 2 < TONGUE_W / 2 + 1.0 and abs(y) + CBORE_D / 2 > TONGUE_W / 2 - 2.0:
            w = abs(abs(y) + CBORE_D / 2 - (TONGUE_W / 2 - 2.0)) + 3.0
            plate -= Pos(CBORE_FLOOR_X / 2 + 0.5, math.copysign(TONGUE_W / 2 - 2.0 + w / 2, y), z) * Box(
                -CBORE_FLOOR_X + 1.0, w, CBORE_D)
    dy, dz = DOWEL
    plate -= Pos(-T + DOWEL_DEPTH / 2 - 0.5, dy, dz) * along_x(Cylinder(DOWEL_D / 2, DOWEL_DEPTH + 1))
    for y, z in INSERTS:                                      # M3 heat-set inserts for the camera frame
        plate -= Pos(-3.0 + 0.5, y, z) * along_x(Cylinder(2.0, 7.0))
    return plate


def main():
    from restyle import restyle
    p = restyle("iso_flange_plate", build())
    save_part("iso_flange_plate", p)
    bb = p.bounding_box()
    print(f"iso_flange_plate {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
          f"{len(p.solids())} solid(s)")


if __name__ == "__main__":
    main()
