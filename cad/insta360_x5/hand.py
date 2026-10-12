"""PG Hand: the Insta360 X5 on the back of the wrist, for bare-hand demonstrations.

The X5 sits in its usual frame, clamped to the PG pod by the camera's own 1/4"-20 screw (see pod.py). For the wrist,
the pod's flat lid is swapped for `pg_wrist_saddle`:

  * its top is the lid: the same footprint, locating rim and four M2 screw holes,
  * under it, a saddle that sits on the back of the forearm just behind the wrist: concave across the arm (R 38 mm,
    fits most forearms with a little foam), 60 mm long along the arm,
  * the saddle is pitched 15° so the device leans forward: with the X5's own 10° in its frame, the camera looks about
    25° down, over the back of the hand at what it grasps,
  * two side wings with 27 x 3.5 mm slots for a 25 mm hook-and-loop strap round the forearm,
  * access holes from below for the lid screws.

    python hand.py
"""
import math

from build123d import Box, Cylinder, Pos, Rot

import pod
from parts import save_part

R_ARM = 38.0                 # forearm radius the saddle fits (concave across the arm)
PITCH = 15.0                 # device nose-down relative to the forearm
LENGTH = 60.0                # along the forearm
WING = 7.0                   # side wings beyond the pod outline, for the strap slots
STRAP_SLOT = (27.0, 3.5)     # along the arm, across
MIN_T = 3.0                  # thinnest saddle under the lid (at the front centre line)


def build():
    lid = pod.build_lid()
    z_top = pod.Z_BOT - pod.LID                       # underside of the lid
    xc = (pod.X0 + pod.X1) / 2
    x0, x1 = xc - LENGTH / 2, xc + LENGTH / 2
    half_w = pod.Y + WING
    depth = 45.0
    body = Pos(xc, 0, z_top - depth / 2) * Box(LENGTH, 2 * half_w, depth)
    # the forearm: a cylinder along the arm, tilted PITCH up towards the front, whose top line runs MIN_T under the lid
    # at the front end
    a = math.radians(PITCH)
    z_front = z_top - MIN_T
    centre_front = (x1, 0.0, z_front - R_ARM)
    arm = Cylinder(R_ARM, 4 * LENGTH)                 # along z, then turned to run along the arm
    arm = Pos(*centre_front) * Rot(0, 90 - PITCH, 0) * Pos(0, 0, -LENGTH) * arm
    body -= arm
    # strap slots through the wings, top to bottom
    for sy in (-1, 1):
        body -= Pos(xc, sy * (pod.Y + WING / 2), z_top - depth / 2) * Box(STRAP_SLOT[0], STRAP_SLOT[1], depth + 2)
    # screwdriver access to the lid screws
    for bx in (pod.X0 + pod.WALL + pod.BOSS_IN, pod.X1 - pod.WALL - pod.BOSS_IN):
        for by in (-(pod.Y - pod.WALL - pod.BOSS_IN), pod.Y - pod.WALL - pod.BOSS_IN):
            body -= Pos(bx, by, z_top - depth / 2) * Cylinder(2.6, depth + 2)
    saddle = lid + body
    return saddle


def main():
    from restyle import restyle
    s = restyle("pg_wrist_saddle", build())
    save_part("pg_wrist_saddle", s)
    bb = s.bounding_box()
    print(f"pg_wrist_saddle {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {s.volume / 1e3:.1f} cm3, "
          f"{len(s.solids())} solid(s)")


if __name__ == "__main__":
    main()
