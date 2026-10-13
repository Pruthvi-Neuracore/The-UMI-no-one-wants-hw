"""PG Hand: the Insta360 X5 on the back of the wrist, for bare-hand demonstrations.

The X5 sits in its usual frame, clamped to the PG pod by the camera's own 1/4"-20 screw (see pod.py). For the wrist,
the pod's flat lid is swapped for `pg_wrist_saddle`:

  * its top is the lid: the same footprint, locating rim and four M2 screw holes,
  * under it, a saddle that sits on the back of the forearm just behind the wrist: concave across the arm (R 38 mm,
    fits most forearms with a little foam), 60 mm long along the arm,
  * the saddle is pitched 15° so the device leans forward: with the X5's own 10° in its frame, the camera looks about
    25° down, over the back of the hand at what it grasps,
  * two side wings with 27 x 3.5 mm slots for a 25 mm hook-and-loop strap round the forearm,
  * access holes from below for the lid screws,
  * a front tab: a curved bar 3 mm off the skin ahead of the saddle, for the 20 mm thumb strap. The strap runs from
    the tab along the back of the hand, between thumb and index finger and round the palm, carrying the palm button
    (palm_button.py), and closes on itself; it stops the saddle sliding or turning without moving it from where it
    sits.
`pg_wrist_saddle_robot` is the same part for a robot hand's round forearm (R 68.5 mm, e.g. the Shadow Hand's).

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
R_ROBOT = 68.5               # pg_wrist_saddle_robot: round robot forearms up to Ø137 (Shadow Hand: Ø135)


THUMB_STRAP = (21.0, 3.0)    # front tab: the 20 mm thumb strap loops round its bar (slot width, slot length along the arm)
TAB_GAP, TAB_T, TAB_L = 3.0, 4.0, 12.0   # the bar stands TAB_GAP off the skin, TAB_T thick; the tab reaches TAB_L forward


def front_tab(x1, z_front, r_arm, pitch):
    """The front tab for the thumb strap: a curved bar ahead of the saddle, standing off the skin so the strap passes
    under it, with a slot between it and the saddle where the strap comes back up and folds over onto itself.
    Returns (solid to add, solid to cut)."""
    def arm(r, extra=0.0):
        c = Cylinder(r, 4 * TAB_L + 200)
        return Pos(x1, 0.0, z_front - r_arm) * Rot(0, 90 - pitch, 0) * Pos(0, 0, -100) * c
    half = THUMB_STRAP[0] / 2 + 3.0
    reach = Pos(x1 + TAB_L / 2 - 1.0, 0, z_front - r_arm) * Box(TAB_L + 2.0, 2 * half, 2 * r_arm + 40)
    band = (arm(r_arm + TAB_GAP + TAB_T) - arm(r_arm + TAB_GAP)) & reach
    band &= Pos(0, 0, z_front - 6.0) * Box(400, 400, 30.0)             # only the top of the arm
    slot = Pos(x1 + 1.0 + THUMB_STRAP[1] / 2, 0, z_front - 5.0) * Box(THUMB_STRAP[1], THUMB_STRAP[0], 40.0)
    return band, slot


def build(r_arm=R_ARM, tab=True):
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
    centre_front = (x1, 0.0, z_front - r_arm)
    arm = Cylinder(r_arm, 4 * LENGTH)                 # along z, then turned to run along the arm
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
    if tab:                                           # the thumb strap's tab (not on the robot saddle)
        band, slot = front_tab(x1, z_front, r_arm, PITCH)
        saddle = saddle + band - slot
    return saddle


def main():
    from restyle import restyle
    for name, r, tab in (("pg_wrist_saddle", R_ARM, True), ("pg_wrist_saddle_robot", R_ROBOT, False)):
        s = restyle("pg_wrist_saddle", build(r, tab))
        save_part(name, s)
        bb = s.bounding_box()
        print(f"{name} {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {s.volume / 1e3:.1f} cm3, "
              f"{len(s.solids())} solid(s)")


if __name__ == "__main__":
    main()
