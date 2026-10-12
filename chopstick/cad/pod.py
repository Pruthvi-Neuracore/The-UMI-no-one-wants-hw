"""PG pod: a small sealed electronics pod under the handheld gripper's camera frame.

It holds a Seeed XIAO ESP32-S3 (LiPo charger, Wi-Fi/BLE, USB-C), an IMU breakout, a 300 mAh 1S LiPo (402535),
a slide power switch and a record button, so the handheld gripper stays wireless. The IMU is screwed rigidly to the
camera, so its gyro can be cross-correlated with the X5's own IMU to put both on one clock.

Mounting: the pod's top has a 3 mm deep pocket that takes the bottom of the X5 frame (it can't rotate), and the
camera's 1/4"-20 screw is replaced by a longer one (1/4"-20 x 5/8" button head) that goes up from inside the pod,
through the pod's top wall and the frame, into the camera. Same 10° axis as the original screw.

The record buttons in the jaws' finger sleeves (sleeve.py) are wired in through two Ø3.5 holes in the front wall.

Parts: pg_pod (body), pg_pod_lid (bottom lid, 4 x M2 x 6 self-tapping), pg_pod_button (TPU cap over a 6 x 6 mm
tactile switch, front face, under the palm's heel / ring finger).

    python pod.py
"""
import math

from build123d import (Axis, Box, Cylinder, Pos, Rot, export_step, export_stl, fillet)

from parts import HW

# frame underside (gripper frame): flat face at z = -50.7; the frame's lower block is within this box over the 3 mm
# the pocket covers (measured from the lite frame)
FRAME_BOTTOM_Z = -50.7
FRAME_BLOCK = dict(x=(0.9, 27.7), y=24.0)
POCKET_CLEAR, POCKET_D = 0.4, 3.0
SCREW_AT = (15.43, 0.0)                      # where the 1/4"-20 axis leaves the frame's bottom face
SCREW_DIR = (0.174, 0.0, 0.985)
WALL, TOP, H = 2.2, 4.0, 22.0                 # side walls, top wall under the pocket, inner height
X0 = FRAME_BLOCK["x"][0] - POCKET_CLEAR - WALL
X1 = 33.0
Y = FRAME_BLOCK["y"] + POCKET_CLEAR + WALL
Z_TOP = FRAME_BOTTOM_Z + POCKET_D             # top of the pocket's lip
Z_BOT = FRAME_BOTTOM_Z - TOP - H              # bottom of the body, where the lid goes
LID = 2.0
CORNER_R = 6.0
BUTTON = dict(y=0.0, z=FRAME_BOTTOM_Z - TOP - 8.0, d=11.0)
USB = dict(y=-6.0, z=Z_BOT + 5.0, w=10.0, h=4.6)       # XIAO USB-C, back wall
SWITCH = dict(x=14.0, z=Z_BOT + 6.0, w=9.0, h=4.0)     # slide switch, -y wall
BOSS_IN = 4.5                                          # lid screw bosses, inset from the inner corners
CABLE = dict(y=17.0, z=FRAME_BOTTOM_Z - TOP - 4.0, d=3.5)  # sleeve-button wires, front wall, one hole per side


def shell():
    cx, w = (X0 + X1) / 2, X1 - X0
    outer = Pos(cx, 0, (Z_BOT + Z_TOP) / 2) * Box(w, 2 * Y, Z_TOP - Z_BOT)
    outer = fillet(outer.edges().filter_by(Axis.Z), CORNER_R)
    outer = fillet(outer.edges().group_by(Axis.Z)[-1], 1.5)
    inner = Pos(cx, 0, (Z_BOT - 1 + FRAME_BOTTOM_Z - TOP) / 2) * Box(w - 2 * WALL, 2 * (Y - WALL),
                                                                         FRAME_BOTTOM_Z - TOP - Z_BOT + 1)
    inner = fillet(inner.edges().filter_by(Axis.Z), CORNER_R - WALL)
    (fx0, fx1), fy = FRAME_BLOCK["x"], FRAME_BLOCK["y"]
    pocket = Pos((fx0 + fx1) / 2, 0, FRAME_BOTTOM_Z + POCKET_D / 2 + 0.5) * Box(
        fx1 - fx0 + 2 * POCKET_CLEAR, 2 * (fy + POCKET_CLEAR), POCKET_D + 1)
    return outer - inner - pocket


def build_pod():
    body = shell()
    # camera screw: Ø6.6 clearance along the original 10° axis, and a flat seat inside for the button head
    sx, sy = SCREW_AT
    tilt = math.degrees(math.atan2(SCREW_DIR[0], SCREW_DIR[2]))
    body -= Pos(sx, sy, FRAME_BOTTOM_Z) * Rot(0, tilt, 0) * Cylinder(3.3, 30)
    # lid screw bosses in the inner corners (Ø6, Ø1.8 pilot for M2 self-tapping)
    for bx in (X0 + WALL + BOSS_IN, X1 - WALL - BOSS_IN):
        for by in (-(Y - WALL - BOSS_IN), Y - WALL - BOSS_IN):
            hgt = FRAME_BOTTOM_Z - TOP - Z_BOT
            body += Pos(bx, by, Z_BOT + hgt / 2) * Cylinder(3.0, hgt)
            body -= Pos(bx, by, Z_BOT + 5) * Cylinder(0.9, 10.5)
    # record button: Ø11 through the front wall, and a post behind it that carries the 6 x 6 switch
    b = BUTTON
    body -= Pos(X1 - WALL / 2, b["y"], b["z"]) * Rot(0, 90, 0) * Cylinder(b["d"] / 2, WALL + 2)
    post_x1 = X1 - WALL - 1.4 - 4.3                      # flange 1.4 + switch height 4.3 (incl. plunger)
    post = Pos(post_x1 - 3.0, b["y"], (b["z"] + FRAME_BOTTOM_Z - TOP) / 2 - 2.0) * Box(
        6.0, 9.0, FRAME_BOTTOM_Z - TOP - b["z"] + 6.0)
    body += post
    body -= Pos(post_x1 - 0.5, b["y"], b["z"]) * Box(1.2, 6.4, 6.4)   # switch seat
    # front wall: wire entries for the sleeve buttons on the jaws (one per side, so either hand works)
    for sy in (-1, 1):
        body -= Pos(X1 - WALL / 2, sy * CABLE["y"], CABLE["z"]) * Rot(0, 90, 0) * Cylinder(CABLE["d"] / 2, WALL + 2)
    # back wall: USB-C for the XIAO, and a Ø2 light pipe for its charge LED
    u = USB
    usb = Pos(X0 + WALL / 2, u["y"], u["z"]) * Box(WALL + 2, u["w"], u["h"])
    body -= fillet(usb.edges().filter_by(Axis.X), 1.5)
    body -= Pos(X0 + WALL / 2, u["y"] + 10.0, u["z"]) * Rot(0, 90, 0) * Cylinder(1.0, WALL + 2)
    # -y wall: slide switch
    sw = SWITCH
    body -= Pos(sw["x"], -Y + WALL / 2, sw["z"]) * Box(sw["w"], WALL + 2, sw["h"])
    return body


def build_lid():
    cx, w = (X0 + X1) / 2, X1 - X0
    lid = Pos(cx, 0, Z_BOT - LID / 2) * Box(w, 2 * Y, LID)
    lid = fillet(lid.edges().filter_by(Axis.Z), CORNER_R)
    for bx in (X0 + WALL + BOSS_IN, X1 - WALL - BOSS_IN):
        for by in (-(Y - WALL - BOSS_IN), Y - WALL - BOSS_IN):
            lid -= Pos(bx, by, Z_BOT - LID / 2) * Cylinder(1.1, LID + 2)
            lid -= Pos(bx, by, Z_BOT - LID + 0.6) * Cylinder(2.1, 1.3)     # shallow head recess
    # locating rim that sits just inside the walls
    rim = Pos(cx, 0, Z_BOT + 0.75) * Box(w - 2 * WALL - 0.6, 2 * (Y - WALL) - 0.6, 1.5)
    rim = fillet(rim.edges().filter_by(Axis.Z), CORNER_R - WALL - 0.3)
    rim -= Pos(cx, 0, Z_BOT + 0.75) * Box(w - 2 * WALL - 4.6, 2 * (Y - WALL) - 4.6, 2.0)
    for bx in (X0 + WALL + BOSS_IN, X1 - WALL - BOSS_IN):          # keep the rim off the screw bosses
        for by in (-(Y - WALL - BOSS_IN), Y - WALL - BOSS_IN):
            rim -= Pos(bx, by, Z_BOT + 0.75) * Cylinder(3.6, 2.0)
    return lid + rim


def build_button():
    """TPU cap: a dome that stands 1 mm proud of the front wall, a flange inside, and a nub for the switch."""
    b = BUTTON
    d = b["d"] - 0.4
    cap = Pos(X1 - WALL / 2 + 0.5, b["y"], b["z"]) * Rot(0, 90, 0) * Cylinder(d / 2, WALL + 1.0)
    cap = fillet(cap.edges().group_by(Axis.X)[-1], 1.2)
    cap += Pos(X1 - WALL - 0.7, b["y"], b["z"]) * Rot(0, 90, 0) * Cylinder(d / 2 + 1.6, 1.4)
    cap += Pos(X1 - WALL - 1.4 - 0.5, b["y"], b["z"]) * Rot(0, 90, 0) * Cylinder(1.6, 1.0)
    return cap


def main():
    from restyle import restyle
    for name, part in (("pg_pod", restyle("pg_pod", build_pod())), ("pg_pod_lid", build_lid()),
                       ("pg_pod_button", build_button())):
        export_step(part, str(HW / "STEP" / f"{name}.step"))
        export_stl(part, str(HW / "STL" / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = part.bounding_box()
        print(f"{name:14s} {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm, {part.volume / 1e3:5.1f} cm3, "
              f"{len(part.solids())} solid(s)")


if __name__ == "__main__":
    main()
