"""Scissor-UMI: a scissor-handled data-collection gripper with a Robotiq gripper's jaw linkage, a RealSense D405 and a
Meta Quest Touch Pro controller on top. The robot twin is the real Robotiq gripper with the same D405 + Quest head on a
flange collar, so the camera, the controller and the pads are in the same place on both.

The mechanism follows FOMI (robbyant-research.github.io/FOMI): scissor handles on the operator's side, meshed sector
gears that keep the two jaws symmetric, and parallelogram finger links, so the operator feels the grasp directly
through the handles. Here the finger links copy the Robotiq gripper's own four-bar (kin.py), so the handheld's pads
move exactly like the robot's for the same q, and the jaws themselves are printed copies of the 2F-140's jaw parts
(replica.py): outer knuckle + outer finger, inner knuckle, inner finger and pad.

Frame and layers (mm): gripper frame G from kin.py (z approach, y opening, x up). The mechanism is a sandwich of
layers stacked along x:

  bottom plate  x -25.5 .. -20.5  encoder, stop slot, record button; the electronics box hangs under it
  L1            x -11.5 .. -4.2   handle A + its sector gear, the left knuckle gear's drive teeth
  L2            x -3.7 .. 3.7     the two knuckle gears' sync teeth
  L3            x 4.2 .. 11.5     handle B + its sector gear, the right knuckle gear's drive teeth
  top plate     x 20.5 .. 25.5    the mast to the head

The 2F-140 jaw parts sit at their own x (centred, up to ±19.6 mm for the inner knuckle) between the plates. The
knuckle gears sit inside the inner knuckles' base clevis (its prongs at |x| 12.2 .. 19.5) and are screwed to one prong
each, so gear and knuckle turn as one.

Gear train: the inner (knuckle) links carry 20-tooth sector gears (module 1.27, pitch radius 12.7 mm: they mesh with
each other exactly on the robot gripper's inner pivots, 25.4 mm apart). Both scissor handles turn on one axle H behind
them, each with a 40-tooth sector that meshes with one knuckle gear, so a handle turns half as far as the jaws (20°
for the full stroke) and the two loops travel ~65 mm, a comfortable scissor span.

    python scissor.py            # writes every part as STEP to ../build/<gripper>/STEP
"""
import math
from pathlib import Path

import numpy as np
from bd_warehouse.gear import SpurGearPlan
from build123d import (Axis, Box, Circle, Cylinder, Location, Plane, Polygon, Pos, Rectangle, Rot, Sketch, Solid,
                       export_step, extrude, fillet, import_step, make_hull, mirror)

import kin

HERE = Path(__file__).resolve().parent
SRC = HERE / "source"
BUILD = HERE.parent / "build" / kin.GRIPPER
STEP_DIR = BUILD / "STEP"

# ---- layers (G x) --------------------------------------------------------------------------------------------------
BOT = (-25.5, -20.5)
L1 = (-11.5, -4.2)
L2 = (-3.7, 3.7)
L3 = (4.2, 11.5)
TOP = (20.5, 25.5)
CLEVIS = 12.2                     # the inner knuckle's prongs start at |x| = 12.2 (its base end)
CLR = 0.5                         # radial clearance around moving parts

PIN_D = 4.0                       # Ø4 x 50 mm steel dowels at the base pivots
PIN_BORE = 4.2                    # running fit in the moving parts
P1_SPACER = 5.2                   # the plates' bosses on the outer pivot reach in to the outer knuckle's hub (±4.9)

# ---- gears ---------------------------------------------------------------------------------------------------------
MODULE = 1.27
KNUCKLE_TEETH = 20                # pitch r = 12.7: the two knuckle gears mesh on the inner pivots (25.4 apart)
HANDLE_TEETH = 40                 # pitch r = 25.4: 2:1, the handles turn q / 2
R_K = MODULE * KNUCKLE_TEETH / 2
R_H = MODULE * HANDLE_TEETH / 2
HUB_R = 10.0                      # knuckle gear hub (below the root circle, clear of the outer pivot's boss)


def pivots():
    j = kin.joints(0.0)
    P1, P2 = j["P1"], j["P2"]
    # handle axle on the centre line, R_K + R_H from both inner pivots
    zH = P2[1] - math.sqrt((R_K + R_H) ** 2 - P2[0] ** 2)
    return P1, P2, np.array([0.0, zH])


P1L, P2L, H = pivots()
P1R, P2R = P1L * [-1, 1], P2L * [-1, 1]

# ---- handles -------------------------------------------------------------------------------------------------------
LOOP_DIST = 90.0                  # axle -> loop centre
LOOP_ID, LOOP_OD = 24.0, 36.0
LOOP_GAP_CLOSED = 38.0            # loop centre to centre with the jaws closed (2 mm between the rings)
HANDLE_W = 11.0
STOP_R = 18.0                     # handle A's stop pin (M3) from the axle; it runs in an arc slot in the bottom plate
HUB_H_R = 5.0                     # handle A's lower hub (bearing in the bottom plate, holds the encoder magnet)

# ---- plates, mast, head --------------------------------------------------------------------------------------------
STANDOFFS = [(0.0, 77.0), (-44.0, 40.0), (44.0, 40.0), (-30.0, -2.0), (30.0, -2.0)]
STANDOFF_R = 5.0
MAST = dict(y=18.0, z=(-16.0, 20.0), x=(TOP[1], 46.0))          # column on the top plate up to the head interface
HEAD_X = (46.0, 52.0)             # head plate; its back face x = 46 is the interface (robot bracket or mast)
HEAD_HOLES = [(12.0, -9.0), (-12.0, -9.0), (12.0, 13.0), (-12.0, 13.0)]   # (y, z), M3 into heat-set inserts
HINGE = (58.0, 13.0)              # D405 tilt hinge axis (x, z), along y: HandUMI's Ø12 x 10 knuckle, M4 bolt
TILT_DEG = 30.0                   # view below the approach axis; the pads sit in the middle of the image
KNUCKLE_OD, KNUCKLE_W = 12.0, 10.0
EBOX = dict(x=(BOT[0] - 22.0, BOT[0]), y=29.0, z=(4.0, 72.0), wall=2.4)   # under the bottom plate
# The Quest Touch Pro stands behind the camera in HandUMI's controller support (its ring), the controller's face looking
# forward, like on the D405 PG Chopstick builds.
QUEST_ARM_AT = (-34.0, 64.0)      # (z, x) of the support's arm-end centre in G (on the centre line)
SUPPORT_YAW = 55.0                # see pg-chopstick d405 ebox.py: the Touch Pro's handle fits the ring best at 55°

# ---- robot collar (ISO 9409-1-50-4-M6) -----------------------------------------------------------------------------
COLLAR_T = 6.0
COLLAR_OD = 75.0
ISO_PCD, ISO_HOLE, ISO_PILOT = 50.0, 6.6, 31.5


# ====================================================================================================================
# helpers: 2D sketches in the (y, z) plane, extruded along x
# ====================================================================================================================
def _loc(m):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*m[0], *m[1], *m[2])
    return Location(t)


YZ = _loc(((0, 0, 1, 0), (1, 0, 0, 0), (0, 1, 0, 0)))   # sketch X -> G y, sketch Y -> G z, extrude Z -> G x


def slab(sk, x):
    """Extrude a (y, z) sketch over x = (x0, x1)."""
    return (Pos(0, 0, x[0]) * extrude(sk, x[1] - x[0])).moved(YZ)


def C(p, r):
    return Pos(float(p[0]), float(p[1])) * Circle(r)


def hull(*sks):
    s = sks[0]
    for k in sks[1:]:
        s = s + k
    return make_hull(s.edges())


def bar(a, b, w):
    return hull(C(a, w / 2), C(b, w / 2))


def rect(y0, y1, z0, z1):
    return Pos((y0 + y1) / 2, (z0 + z1) / 2) * Rectangle(y1 - y0, z1 - z0)


def wedge(c, r, a0, a1):
    """Pie slice at c, radius r, from angle a0 to a1 (deg, from +y towards +z)."""
    n = max(4, int(abs(a1 - a0) / 5) + 1)
    pts = [tuple(c)] + [(c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a)))
                        for a in np.linspace(a0, a1, n)]
    return Polygon(*pts, align=None)


def xcyl(p, r, x):
    """Cylinder along x through the (y, z) point p."""
    return Pos(0, float(p[0]), float(p[1])) * Pos(x[0], 0, 0) * Rot(0, 90, 0) * Pos(0, 0, (x[1] - x[0]) / 2) * \
        Cylinder(r, x[1] - x[0])


def rot_about(p, deg):
    """Rotation about the G x axis through the (y, z) point p (a positive angle turns +y towards +z)."""
    return Pos(0, float(p[0]), float(p[1])) * Rot(deg, 0, 0) * Pos(0, -float(p[0]), -float(p[1]))


FLANK_CLR = 0.08                  # each tooth flank thinned by this (printed gears: ~0.16 mm backlash per mesh)


def gear_sketch(teeth, centre, phase_deg, clearance=False):
    g = Sketch() + SpurGearPlan(MODULE, teeth, 20)
    if clearance:                     # thin both flanks: the gear turned +d and -d, intersected (d = FLANK_CLR at the
        d = math.degrees(FLANK_CLR / (MODULE * teeth / 2))       # pitch circle); keeps the involutes exact
        g = (Rot(0, 0, d) * g) & (Rot(0, 0, -d) * g)
    return Pos(float(centre[0]), float(centre[1])) * Rot(0, 0, phase_deg) * g


def _deg(v):
    return math.degrees(math.atan2(v[1], v[0]))


# ====================================================================================================================
# gear phases: found numerically (the two knuckle gears are mirror images in layout, so their teeth are half a pitch
# apart; each handle's phase follows its knuckle gear's)
# ====================================================================================================================
def _best_phase(fixed, teeth, centre):
    pitch = 360.0 / teeth
    best = None
    for ph in np.linspace(0, pitch, 37)[:-1]:
        a = (fixed & gear_sketch(teeth, centre, ph)).area
        if best is None or a < best[1]:
            best = (ph, a)
    return best[0]


def phases():
    kl = 0.0                                                     # left knuckle gear: a tooth along +y at q = 0
    gl = gear_sketch(KNUCKLE_TEETH, P2L, kl)
    kr = _best_phase(gl, KNUCKLE_TEETH, P2R)
    ha = _best_phase(gl, HANDLE_TEETH, H)
    hb = _best_phase(gear_sketch(KNUCKLE_TEETH, P2R, kr), HANDLE_TEETH, H)
    return dict(kl=kl, kr=kr, ha=ha, hb=hb)


# ====================================================================================================================
# motion: each part's rigid motion at q (applied to the part as built at q = 0)
# ====================================================================================================================
def motion(name, q):
    """build123d Location for a part at q (parts as built at q = 0; the right jaw's replica parts are turned 180°
    about z first, see jaw_base())."""
    j0, j = kin.joints(0.0), kin.joints(q)
    a = math.degrees(q)
    side = "left" if name.endswith("_left") else "right" if name.endswith("_right") else None
    sgn = -1 if side == "left" else 1
    if name.startswith(("knuckle_gear", "inner_knuckle")):
        return rot_about(P2L if side == "left" else P2R, sgn * a)
    if name.startswith("outer_link"):
        return rot_about(P1L if side == "left" else P1R, sgn * a)
    if name.startswith(("inner_finger", "pad_core", "pad_skin")):
        d = j["B"] - j0["B"]
        return Pos(0, float(d[0]) * (1 if side == "left" else -1), float(d[1]))
    if name in ("handle_a", "stop_pin"):
        return rot_about(H, a * R_K / R_H)
    if name == "handle_b":
        return rot_about(H, -a * R_K / R_H)
    return Location()


def jaw_base(name):
    """4 x 4 placing a jaw part's STEP (built for the left jaw) for `name`: identity, or 180° about z for the right."""
    T = np.eye(4)
    if name.endswith("_right") and name.startswith(("outer_link", "inner_knuckle", "inner_finger", "pad_")):
        T[0, 0] = T[1, 1] = -1.0
    return T


# ====================================================================================================================
# parts (each built in G at q = 0)
# ====================================================================================================================
KEY_INSERT = 6.0                  # M3 heat-set inserts in the gear's lobe, depth


def knuckle_gear(side="left"):
    """Sector gear in the inner knuckle's base clevis, screwed to one prong (2 x M3 through the prong into inserts).
    Left: drive teeth in L1 (handle A), sync teeth in L2, keyed to the lower prong. Right: L3 / L2, upper prong."""
    import replica
    ph = phases()
    s = 1 if side == "left" else -1
    P2 = P2L if side == "left" else P2R
    phase = ph["kl"] if side == "left" else ph["kr"]
    full = gear_sketch(KNUCKLE_TEETH, P2, phase, clearance=True)
    qd = math.degrees(kin.g()["q_max"])

    def arc(world_deg):
        lo, hi = sorted((world_deg, world_deg + s * qd))
        return wedge(P2, R_K + 3 * MODULE, lo - 12, hi + 12)
    to_other = _deg(P2R - P2L) if side == "left" else _deg(P2L - P2R)
    to_handle = _deg(H - P2)
    sync = (full & arc(to_other)) + C(P2, HUB_R)
    drive = (full & arc(to_handle)) + C(P2, HUB_R)
    # lobe along the knuckle's arm (inside the clevis, up to where its web starts), carrying the key inserts
    a0 = kin.link_angle(0.0)
    arm = np.array([math.cos(a0), math.sin(a0)]) * np.array([s, 1])        # local +y of the knuckle, in (y, z)
    lobe = bar(P2, P2 + arm * (replica.KEY_AT[1] + 2.0), 11.0)   # ends 1.3 mm short of the knuckle's web
    lay_drive = L1 if side == "left" else L3
    xr = (-CLEVIS + 0.25, L2[1]) if side == "left" else (L2[0], CLEVIS - 0.25)
    part = slab(sync, L2) + slab(drive, lay_drive) + slab(C(P2, HUB_R) + lobe, xr)
    part -= xcyl(P2, PIN_BORE / 2, (-30, 30))
    face = xr[0] if side == "left" else xr[1]
    for k in replica.KEY_AT:
        c = P2 + arm * k
        part -= xcyl(c, 2.0, (face - s * 0.01, face + s * KEY_INSERT) if side == "left" else (face - KEY_INSERT, face + 0.01))
    return part


def handle_angles():
    """World angles (deg) of handle A's tail at q = 0 (open) and q_max (closed); handle B is its mirror."""
    half = LOOP_GAP_CLOSED / 2
    closed = -180 + math.degrees(math.acos(half / LOOP_DIST))      # loop on the -y side, behind the axle
    opened = closed - math.degrees(kin.g()["q_max"]) * R_K / R_H
    return opened, closed


def handle(which="a"):
    """Scissor handle with its 40-tooth sector. A: L1, meshes the left knuckle gear, loop on the -y side.
    B: L3, the right knuckle gear, loop on +y. Squeezing the loops together closes the jaws."""
    ph = phases()
    lay = L1 if which == "a" else L3
    s = 1 if which == "a" else -1
    opened, _ = handle_angles()
    tail = opened if which == "a" else 180 - opened
    loop = H + LOOP_DIST * np.array([math.cos(math.radians(tail)), math.sin(math.radians(tail))])
    full = gear_sketch(HANDLE_TEETH, H, ph["ha"] if which == "a" else ph["hb"], clearance=True)
    to_k = _deg((P2L if which == "a" else P2R) - H)
    turn = math.degrees(kin.g()["q_max"]) * R_K / R_H * s       # handle turns +s*q/2; contact arc runs backwards
    lo, hi = sorted((to_k, to_k - turn))
    sector = full & wedge(H, R_H + 3 * MODULE, lo - 10, hi + 10)
    sk = sector + C(H, 9.0) + bar(H, loop, HANDLE_W)
    p = slab(sk, lay)
    # the loop is taller than the layer (outside the plates), for the fingers
    lx = (BOT[0], L1[1]) if which == "a" else (L3[0], TOP[1])
    ring = C(loop, LOOP_OD / 2)
    p += slab(ring, lx)
    p -= slab(C(loop, LOOP_ID / 2), (lx[0] - 1, lx[1] + 1))
    p -= xcyl(H, PIN_BORE / 2 if which == "b" else 1.6, (-20, 20))  # A: M4 heat-set insert from the top (Ø5.6 x 8)
    if which == "a":
        p -= xcyl(H, 2.8, (L1[1] - 8, L1[1] + 0.1))                  # M4 heat-set insert, bolt from the top
        p += xcyl(H, HUB_H_R, (BOT[0] + 0.3, L1[0] + 0.1))             # lower hub runs in the bottom plate
        p -= xcyl(H, 3.1, (BOT[0], BOT[0] + 2.8))                       # Ø6 x 2.5 diametric magnet (AS5047P)
        sp = H + STOP_R * np.array([math.cos(math.radians(opened + 8)), math.sin(math.radians(opened + 8))])
        p -= xcyl(sp, 2.0, (L1[0] - 0.1, L1[0] + 5))                    # M3 heat-set insert for the stop screw
    return p


def stop_arc():
    """The arc handle A's stop screw runs in: its ends are the open and closed stops."""
    opened, closed = handle_angles()
    a0, a1 = opened + 8, closed + 8
    pts = [H + STOP_R * np.array([math.cos(math.radians(a)), math.sin(math.radians(a))]) for a in (a0, a1)]
    sk = Sketch()
    for a in np.linspace(a0, a1, 12):
        sk += C(H + STOP_R * np.array([math.cos(math.radians(a)), math.sin(math.radians(a))]), 1.7)
    return sk, pts


def plate_outline():
    pts = [C(P1L, 11), C(P1R, 11), C(P2L, 11), C(P2R, 11), C(H, 16)]
    pts += [C(s, STANDOFF_R + 3) for s in STANDOFFS]
    return hull(*pts)


def _plate_bosses(top):
    """Spacer bosses from a plate inwards: to the outer knuckle's hub at P1, to the inner knuckle's prongs at P2 and
    to the handles at H (top plate only; the bottom plate's is handle A's hub bearing)."""
    sg = 1 if top else -1
    face = TOP[0] if top else BOT[1]
    b = None
    for c, reach, r in ((P1L, P1_SPACER, 5.5), (P1R, P1_SPACER, 5.5), (P2L, 19.9, 5.5), (P2R, 19.9, 5.5),
                        (H, (L3[1] + 0.3) if top else (-L1[0] + 0.3), 8.0)):
        x = (reach, face + 0.01) if top else (face - 0.01, -reach)
        bb = xcyl(c, r, x)
        b = bb if b is None else b + bb
    return b


def base_bottom():
    """Bottom plate with the standoff columns (M3 heat-set inserts in their tops), the pivot spacers, the stop slot,
    the record button pod and the electronics box under it (AS5047P under handle A's hub)."""
    out = plate_outline() + rect(-EBOX["y"], EBOX["y"], EBOX["z"][0], EBOX["z"][1])
    p = slab(out, BOT) + _plate_bosses(top=False)
    for s in STANDOFFS:
        p += slab(C(s, STANDOFF_R), (BOT[1], TOP[0]))
        p -= xcyl(s, 2.0, (TOP[0] - 7, TOP[0] + 0.1))                   # M3 insert
    for c in (P1L, P1R, P2L, P2R):
        p -= xcyl(c, PIN_D / 2 - 0.05, (BOT[0] - 1, 30))                # press fit for the dowels
    p -= xcyl(H, HUB_H_R + 0.25, (BOT[0] - 1, 0))                      # handle A's hub
    for dy in (-10.5, 10.5):                                            # AS5047P board: 2 x M2 inserts, under the hub
        p -= xcyl((H[0] + dy, H[1]), 1.6, (BOT[0] - 0.01, BOT[0] + 4))
    arc, _ = stop_arc()
    p -= slab(arc, (BOT[0] - 1, BOT[1] + 1))
    for y, z in _ebox_holes():
        p -= xcyl((y, z), 2.0, (BOT[0] - 0.01, BOT[0] + 4.5))
    # record button: a 12 x 12 tactile switch in a pod under the plate's back edge, the index finger's place
    bp = np.array([0.0, -10.0])
    pod = slab(rect(bp[0] - 9, bp[0] + 9, bp[1] - 9, bp[1] + 9), (BOT[0] - 9, BOT[0]))
    pod -= slab(rect(bp[0] - 6.2, bp[0] + 6.2, bp[1] - 6.2, bp[1] + 6.2), (BOT[0] - 9.1, BOT[0] - 4))
    pod -= slab(rect(-2.5, 2.5, bp[1], EBOX["z"][0] + 6), (BOT[0] - 5, BOT[0] - 1.5))   # wire into the box
    p += pod
    return p


def base_top():
    """Top plate, the pivot spacers and the mast to the head (M3 inserts in its top face at x = 46)."""
    out = plate_outline() + rect(-MAST["y"], MAST["y"], MAST["z"][0], MAST["z"][1])
    p = slab(out, TOP) + _plate_bosses(top=True)
    mast = slab(rect(-MAST["y"], MAST["y"], *MAST["z"]), MAST["x"])
    mast -= slab(rect(-MAST["y"] + 3, MAST["y"] - 3, MAST["z"][0] + 3, MAST["z"][1] - 3), (TOP[1], MAST["x"][1] - 4))
    mast -= slab(rect(-7, 7, MAST["z"][0] - 1, MAST["z"][0] + 3.01), (TOP[1] + 3, MAST["x"][1] - 6))  # cable out the back
    p += mast
    for y, z in HEAD_HOLES:
        p -= xcyl((y, z), 2.0, (MAST["x"][1] - 7, MAST["x"][1] + 0.1))
    for s in STANDOFFS:
        p -= xcyl(s, 1.7, (TOP[0] - 1, TOP[1] + 1))
        p -= xcyl(s, 3.1, (TOP[1] - 1.5, TOP[1] + 1))                    # head recess
    for c in (P1L, P1R, P2L, P2R):
        p -= xcyl(c, PIN_D / 2 - 0.05, (-30, TOP[1] + 1))
    p -= xcyl(H, 2.2, (0, TOP[1] + 1))                                 # M4 axle bolt
    p -= xcyl(H, 4.2, (TOP[1] - 3, TOP[1] + 1))
    return p


def _ebox_holes():
    (z0, z1), y = EBOX["z"], EBOX["y"]
    return [(y - 4, z0 + 4), (-y + 4, z0 + 4), (y - 4, z1 - 4), (-y + 4, z1 - 4)]


def ebox_cover():
    """Box under the bottom plate for the Pico 2, the USB hub and the AS5047P. Open towards the plate, screwed up into
    it at the corners."""
    (x0, x1), y, (z0, z1), w = EBOX["x"], EBOX["y"], EBOX["z"], EBOX["wall"]
    outer = slab(rect(-y, y, z0, z1), (x0, x1))
    outer = fillet(outer.edges().filter_by(Axis.X), 4.0)
    inner = slab(rect(-y + w, y - w, z0 + w, z1 - w), (x0 + w, x1 + 1))
    box = outer - inner
    for yy, zz in _ebox_holes():
        box += slab(C((yy, zz), 4.0), (x0 + w, x1))
        box -= xcyl((yy, zz), 1.7, (x0 - 1, x1 + 1))
        box -= xcyl((yy, zz), 3.0, (x0 - 1, x0 + 2.0))
    box -= Pos((x0 + x1) / 2 + 2, y, (z0 + z1) / 2) * Box(8.0, 3 * w, 13.0)       # USB-C out (to the PC), +y side
    box -= Pos((x0 + x1) / 2 + 2, 0, z0) * Box(8.0, 10.0, 3 * w)                  # D405 cable in (from the head)
    return box


# ---- head: D405 knuckle + Quest arm end ----------------------------------------------------------------------------
HANDUMI = HERE.parents[1] / "handumi" / "hardware" / "STEP" / "right"
# HandUMI's controller support in its STEP frame S (see pg-chopstick/cad/d405/ebox.py): ring axis z, arm end box,
# and HandUMI's main support -> S. These are the D405 build's numbers, unchanged.
RING_C = (20.45, 50.95)
T_S = ((0.0, -1.0, 0.0, -32.5), (-1.0, 0.0, 0.0, RING_C[0]), (0.0, 0.0, -1.0, 0.0))
M_TO_S = ((0.0, -1.0, 0.0, 95.4), (1.0, 0.0, 0.0, -53.0), (0.0, 0.0, 1.0, 0.0))
ARM_END_S = ((-1.0, 46.0), (11.0, 22.0), (-0.5, 8.5))
ARM_C_S = (22.9, 16.0, 4.0)
# kit frame (X forward, Y left, Z up) -> G (x up, y, z forward)
K = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])


def _m4(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def t_support():
    """4 x 4: HandUMI's support frame S -> G."""
    a = math.radians(SUPPORT_YAW)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    R_kit = Rz @ np.array(T_S)[:, :3]
    R = K @ R_kit
    at = np.array([QUEST_ARM_AT[1], 0.0, QUEST_ARM_AT[0]])
    return _m4(R, at - R @ ARM_C_S)


def _loc4(T):
    return _loc(tuple(map(tuple, T[:3, :4])))


def quest_support(side="right"):
    s = import_step(str(HANDUMI / "right_controller_support.step")).moved(_loc4(t_support()))
    return s if side == "right" else mirror(s, Plane.XZ)


def head(side="right"):
    """Head plate on the x = 46 interface, the D405 tilt knuckle on top and a copy of HandUMI's main-support arm end,
    so HandUMI's controller support (the Quest ring) bolts on exactly as on HandUMI (2 x M3 x 25 + nuts)."""
    plate = slab(rect(-18, 18, -16, 20), HEAD_X)
    plate = fillet(plate.edges().filter_by(Axis.X), 3.0)
    hx, hz = HINGE
    knuckle = Pos(hx, 0, hz) * Rot(90, 0, 0) * Cylinder(KNUCKLE_OD / 2, KNUCKLE_W)
    neck = Pos((HEAD_X[1] + hx) / 2, 0, hz) * Box(hx - HEAD_X[1] + 0.5, KNUCKLE_W, KNUCKLE_OD)
    body = plate + neck + knuckle
    body -= Pos(hx, 0, hz) * Rot(90, 0, 0) * Cylinder(2.2, KNUCKLE_W + 2)
    # Quest arm end: HandUMI's main support, clipped to its arm end, in G
    ms = import_step(str(HANDUMI / "main_support.step"))
    (bx0, bx1), (by0, by1), (bz0, bz1) = ARM_END_S
    clip = Pos((bx0 + bx1) / 2, (by0 + by1) / 2, (bz0 + bz1) / 2) * Box(bx1 - bx0, by1 - by0, bz1 - bz0)
    T = t_support()
    end = (ms.moved(_loc(M_TO_S)) & clip).moved(_loc4(T))
    # neck: from the arm end's inner edge to the head plate's back, a tapered 8 mm thick strut along -z
    foot = [(T @ [x, y, 0, 1])[:3] for x in (4.9, 40.9) for y in (by0 + 0.5, by1 - 1.0)]
    foot_b = [(T @ [x, y, 8, 1])[:3] for x in (4.9, 40.9) for y in (by0 + 0.5, by1 - 1.0)]
    pts = np.array(foot + foot_b)
    plate_pts = [(x, y, -16.0) for x in HEAD_X for y in (-16.0, 16.0)] + \
                [(x, y, -6.0) for x in HEAD_X for y in (-16.0, 16.0)]
    strut = _hull3d(np.vstack([pts, plate_pts]))
    arm = end + strut
    for hxs in (14.9, 30.9):                                        # the arm end's two M3 holes
        c = (T @ [hxs, 16.0, 4.0, 1])[:3]
        ax = T[:3, 2]
        arm -= Solid.make_cylinder(1.6, 60, Plane(origin=tuple(c - ax * 30), z_dir=tuple(ax)))
    body += arm
    for y, z in HEAD_HOLES:
        body -= xcyl((y, z), 1.7, (HEAD_X[0] - 1, HEAD_X[1] + 1))
        body -= xcyl((y, z), 3.0, (HEAD_X[1] - 3.0, HEAD_X[1] + 30))
    return body if side == "right" else mirror(body, Plane.XZ)


def _hull3d(pts):
    """Convex hull of points as a solid (via trimesh -> faces)."""
    import trimesh
    from build123d import Face, Shell, Wire
    h = trimesh.convex.convex_hull(pts)
    faces = []
    for f in h.faces:
        v = [tuple(map(float, h.vertices[i])) for i in f]
        faces.append(Face(Wire.make_polygon(v)))
    return Solid(Shell(faces))


# ---- D405 cup (HandUMI) and its pose ------------------------------------------------------------------------------
CUP_AXIS = (-29.5, 1.5)           # hinge axis in the cup's frame: along x, at (y, z) (HandUMI hinge.AXIS_CAM)
CUP_FLOOR, CAM_DEPTH = 2.0, 23.0


def cup_matrix(tilt_deg=TILT_DEG):
    """4 x 4: the HandUMI D405 cup's frame -> G at a tilt (the cup's +z is the view)."""
    p = math.radians(tilt_deg)
    R_kit = np.column_stack([[0, 1.0, 0], [math.sin(p), 0, math.cos(p)], [math.cos(p), 0, -math.sin(p)]])
    R = K @ R_kit
    hinge = np.array([HINGE[0], 0.0, HINGE[1]])
    return _m4(R, hinge - R @ np.array([0.0, *CUP_AXIS]))


def cup(tilt_deg=TILT_DEG):
    return import_step(str(HANDUMI / "d405_wrist_mount.step")).moved(_loc4(cup_matrix(tilt_deg)))


def camera_pose(tilt_deg=TILT_DEG):
    """D405 optical frame in G (OpenCV convention: z view, x right in the image, y down), origin on the front glass."""
    Tc = cup_matrix(tilt_deg)
    view = Tc[:3, 2]
    down = -np.array([1.0, 0, 0]) + view * view[0]
    down /= np.linalg.norm(down)
    right = np.cross(down, view)
    T = _m4(np.column_stack([right, down, view]), (Tc @ [0, 0, CUP_FLOOR + CAM_DEPTH, 1])[:3])
    return T


# ---- robot collar --------------------------------------------------------------------------------------------------
def flange_z():
    p = kin.g()
    return -p["coupling_to_base"] - p["coupling_back"]          # the robot flange face in G (no collar)


def robot_collar():
    """Spacer ring between the robot's ISO 50 flange and the gripper's coupling, with an arm up the gripper's +x side to
    the head interface (x = 46, M3 inserts). The gripper moves COLLAR_T further from the flange: use M6 screws
    COLLAR_T longer."""
    zf = flange_z()                                              # the coupling's back face, now on the collar
    z0, z1 = zf - COLLAR_T, zf
    ring = Pos(0, 0, (z0 + z1) / 2) * Cylinder(COLLAR_OD / 2, COLLAR_T)
    ring -= Pos(0, 0, z1 - 1.5) * Cylinder(ISO_PILOT / 2 + 0.1, 3.01)    # recess for the coupling's pilot
    ring += Pos(0, 0, z0 - 1.5) * Cylinder(ISO_PILOT / 2 - 0.1, 3.0)      # pilot into the robot flange's recess
    ring -= Pos(0, 0, (z0 + z1) / 2 - 1.5) * Cylinder(10.0, COLLAR_T + 4)   # Ø20 through (centre, cables)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        ring -= Pos(ISO_PCD / 2 * math.cos(a), ISO_PCD / 2 * math.sin(a), (z0 + z1) / 2) * Cylinder(ISO_HOLE / 2, 20)
    ring -= Pos(ISO_PCD / 2, 0, z0) * Cylinder(3.05, 8)            # Ø6 dowel (ISO 9409 locating pin) at 0°
    # wall up the +x side, x 39 .. 46, to the head's interface
    wall = Pos(42.5, 0, (z0 + 20) / 2) * Box(7.0, 36.0, 20 - z0)
    foot = Pos((30 + 46) / 2, 0, (z0 + z1) / 2) * Box(16.0, 36.0, COLLAR_T)
    body = ring + wall + foot
    for y in (-15.0, 15.0):                                      # gussets: wall to collar, 6 mm thick
        pts = [(x, y + dy, z) for dy in (-3.0, 3.0) for x, z in ((26.0, z1 - 0.5), (39.5, z1 - 0.5), (39.5, z1 + 22))]
        body += _hull3d(np.array(pts))
    for y, z in HEAD_HOLES:
        body -= xcyl((y, z), 2.0, (46 - 7, 46.1))
    return body


# ====================================================================================================================
PARTS = {
    "base_bottom": base_bottom, "base_top": base_top,
    "knuckle_gear_left": lambda: knuckle_gear("left"), "knuckle_gear_right": lambda: knuckle_gear("right"),
    "handle_a": lambda: handle("a"), "handle_b": lambda: handle("b"),
    "ebox_cover": ebox_cover,
    "head_right": lambda: head("right"), "head_left": lambda: head("left"),
    "robot_collar_iso50": robot_collar,
}


def _replica(name):
    import replica
    return lambda: replica.part(name)


for _n in ("outer_link", "inner_knuckle", "inner_finger", "pad_core", "pad_skin"):
    PARTS[f"{_n}_2f140"] = _replica(_n)


def build_all(names=None):
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    out = {}
    for n, f in PARTS.items():
        if names and n not in names:
            continue
        s = f()
        export_step(s, str(STEP_DIR / f"{n}.step"))
        bb = s.bounding_box()
        print(f"{n:22s} {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm  {s.volume / 1e3:6.1f} cm3  "
              f"{len(s.solids())} solid(s)")
        out[n] = s
    return out


if __name__ == "__main__":
    import sys
    print(f"H = {H.round(2)}, gear phases {phases()}, handle tail open/closed {handle_angles()}")
    build_all(sys.argv[1:] or None)
