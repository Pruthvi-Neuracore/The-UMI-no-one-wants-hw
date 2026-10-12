"""Enclosed electronics box: Raspberry Pi Pico 2 + IMU + USB 3 hub, one USB-C cable out.

Local frame: floor outer face at z=0, box rises toward +z, x along the arm, y across it.
In the assembly (main-support frame M) local z maps to -Z (the box hangs under the arm, on the
same side as the servo) and the box centre sits at BOX_CENTRE_M.

Inside:
  * USB 3 hub board on the floor (bay sized for common 4-port VL817-type boards: VERIFY)
  * IMU breakout (BNO085-class) in a floor pocket, rigid to the frame (outline: VERIFY)
  * Raspberry Pi Pico 2 on four M2 standoffs under the lid (51 x 21 mm, holes 47 x 11.4 mm)
Openings: D405 cable and servo cable on the camera side, USB-C to the laptop on one side wall,
vents in the lid. Two M3 screws come up through the arm into heat-set inserts in the floor.
"""
from build123d import (Box, Cylinder, Pos, Rot, SlotCenterToCenter, extrude, fillet, Axis)

BOX = (43.0, 90.0, 30.0)                 # outer x, y, z (z without lid)
WALL, FLOOR, LID_T = 2.5, 3.0, 3.0
BOX_CENTRE_M = (40.0, 72.5)              # between the servo (x <= 17.2) and the controller support (x >= 63)
ARM_SCREWS = [(-10.0, -11.0), (10.0, -11.0)]   # local (x, y), between the IMU and the hub
PICO_HOLES = [(sx * 5.7, sy * 23.5) for sx in (-1, 1) for sy in (-1, 1)]
PICO_STANDOFF_H = 5.0                    # under the lid; the hub (<= 14 mm tall) sits below
IMU_POCKET = (28.0, 24.0, 2.0)           # VERIFY against your breakout
IMU_AT = (0.0, -28.0)
HUB_BAY = (38.0, 42.0)                   # VERIFY against your hub board
HUB_AT = (0.0, 16.0)
LID_POSTS = [(sx * 17.0, sy * 41.0) for sx in (-1, 1) for sy in (-1, 1)]
M3_INSERT, M3_CLR, M2_PILOT = 4.0, 3.4, 1.9


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def shell():
    bx, by, bz = BOX
    b = fillet(box(-bx / 2, bx / 2, -by / 2, by / 2, 0, bz).edges().filter_by(Axis.Z), 5.0)
    inner = fillet(box(-bx / 2 + WALL, bx / 2 - WALL, -by / 2 + WALL, by / 2 - WALL, FLOOR, bz + 1).edges().filter_by(Axis.Z), 3.0)
    b -= inner
    # floor features
    ix, iy, iz = IMU_POCKET
    b -= box(IMU_AT[0] - ix / 2, IMU_AT[0] + ix / 2, IMU_AT[1] - iy / 2, IMU_AT[1] + iy / 2, FLOOR - iz, FLOOR + 0.1)
    hx, hy = HUB_BAY                                                    # 1.5 mm locating rails around the hub
    rails = box(HUB_AT[0] - hx / 2 - 1.5, HUB_AT[0] + hx / 2 + 1.5, HUB_AT[1] - hy / 2 - 1.5, HUB_AT[1] + hy / 2 + 1.5, FLOOR - 0.01, FLOOR + 1.5)
    rails -= box(HUB_AT[0] - hx / 2, HUB_AT[0] + hx / 2, HUB_AT[1] - hy / 2, HUB_AT[1] + hy / 2, FLOOR - 1, FLOOR + 2)
    b += rails
    for x, y in ARM_SCREWS:                                             # M3 inserts from the inside
        b += Pos(x, y, FLOOR + 2.5 - 0.01) * Cylinder(4.5, 5.0)
        b -= Pos(x, y, 3.0) * Cylinder(M3_INSERT / 2, 10)
    for x, y in LID_POSTS:
        b += Pos(x, y, (FLOOR + bz) / 2) * Cylinder(4.0, bz - FLOOR)
        b -= Pos(x, y, bz - 3) * Cylinder(M3_INSERT / 2, 8)
    # openings
    b -= box(-bx / 2 - 1, -bx / 2 + WALL + 1, -9, 9, 13.5, 23.5)          # D405 cable (USB plug passes)
    b -= box(-bx / 2 - 1, -bx / 2 + WALL + 1, -34, -26, 6, 11)            # servo cable
    b -= box(4, 17, by / 2 - WALL - 1, by / 2 + 1, 6, 14)                  # USB-C to the laptop (hub upstream)
    return _soften(b, [e for e in b.edges() if e.bounding_box().max.Z < 0.01], (2.5, 2.0, 1.5))   # rounded underside


def lid():
    bx, by, bz = BOX
    l = fillet(box(-bx / 2, bx / 2, -by / 2, by / 2, bz, bz + LID_T).edges().filter_by(Axis.Z), 5.0)
    for x, y in LID_POSTS:
        l -= Pos(x, y, bz + LID_T / 2) * Cylinder(M3_CLR / 2, LID_T + 2)
    for i in range(5):                                                  # vents end short of the Pico standoffs (y ±23.5)
        l -= Pos(-12 + i * 6, 0, bz - 1) * extrude(Rot(0, 0, 90) * SlotCenterToCenter(30.0, 3.0), LID_T + 2)
    for x, y in PICO_HOLES:                                             # Pico hangs under the lid
        l += Pos(x, y, bz - PICO_STANDOFF_H / 2 + 0.01) * Cylinder(2.6, PICO_STANDOFF_H)
        l -= Pos(x, y, bz - PICO_STANDOFF_H + 3) * Cylinder(M2_PILOT / 2, 8)
    top = [e for e in l.edges() if e.bounding_box().min.Z > bz + LID_T - 0.01 and abs(abs(e.center().X) - 0) >= 0
           and (abs(e.center().X) > bx / 2 - 6 or abs(e.center().Y) > by / 2 - 6)]
    return _soften(l, top, (1.5, 1.0))                                  # rounded top edge


def _soften(shape, edges, radii):
    from build123d import fillet
    for r in radii:
        try:
            out = fillet(list(edges), r)
            if out.is_valid:
                return out
        except Exception:
            pass
    return shape


def to_m(x, y):
    """Local (x, y) -> main-support frame (the box is flipped about X when mounted under the arm)."""
    return BOX_CENTRE_M[0] + x, BOX_CENTRE_M[1] - y
