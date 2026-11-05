"""Scissor-UMI: a scissor-handled data-collection gripper with a Robotiq gripper's jaw linkage, a RealSense D405 and a
Meta Quest Touch Pro controller on top. The robot twin is the real Robotiq gripper with the same D405 + Quest head on a
flange collar, so the camera, the controller and the pads are in the same place on both.

The mechanism: scissor handles on the operator's side, meshed sector
gears that keep the two jaws symmetric, and parallelogram finger links, so the operator feels the grasp directly
through the handles. Here the finger links copy the Robotiq gripper's own four-bar (kin.py), so the handheld's pads
move exactly like the robot's for the same q, and the jaws themselves are printed copies of the 2F-140's jaw parts
(replica.py): outer knuckle + outer finger, inner knuckle, inner finger and pad.

Frame and layers (mm): gripper frame G from kin.py (z approach, y opening, x up). The mechanism is a sandwich of
layers stacked along x:

  bottom plate  x -25.5 .. -20.5  stop slot, record button; the electronics box (STS3215 encoder) hangs under it
  L1            x -11.5 .. -4.2   handle A + its sector gear, the left knuckle gear's drive teeth
  L2            x -3.7 .. 3.7     the two knuckle gears' sync teeth
  L3            x 4.2 .. 11.5     handle B + its sector gear, the right knuckle gear's drive teeth
  top plate     x 20.5 .. 25.5    the mast (camera tab copy) stands on it

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
LOOP_DIST = 90.0                  # with LOOP_GAP_CLOSED: sets the handles' angles (where the old loops met when closed)
LOOP_GAP_CLOSED = 38.0
# Grips, after a spring-action garden scissor (Fiskars Easy Action): handle B is a solid palm bar that swells in the
# middle, handle A a long D-loop the four fingers go through and wrap round. Both are 26 mm tall (x), every edge
# rounded, with a soft TPU face where the hand pushes and pulls. Hold it from below, palm up: the palm on the palm
# bar, the fingers up through the loop's window and round its outer bar, the index finger on the record button under
# the base. The grip is handed: the fingers point across the handheld from the palm bar to the loop, so the thumb and
# index finger end up forward (at the button) only if the loop is on the side the fingers point to. Right hand: palm
# bar on handle B (+y), loop on handle A (-y). Left hand: the mirror, palm bar on A, loop on B. The gears, hub, servo
# drive and stop are the same in both (they are on handle A whatever its grip).
GRIP_X = (-13.0, 13.0)
GRIP_R = 5.0                      # edge rounding of the grips
TPU_X = (-9.5, 9.5)               # the soft inserts' height
# (s along the handle from the axle, n outwards, away from the other handle), mm
PALM_OUT = [(34, 6.5), (50, 10.0), (65, 14.0), (80, 16.5), (95, 16.0), (110, 13.5), (121, 9.5)]
PALM_IN = [(121, -5.0), (100, -5.5), (75, -6.0), (50, -6.0), (34, -6.0)]
PALM_TIP = (127.0, 2.0)
PALM_TPU = 3.0                    # the palm insert's thickness, on the outer face, s 50 .. 116
LOOP_OUT = [(34, 6.5), (46, 16.0), (56, 30.0), (68, 39.0), (85, 43.0), (105, 43.0), (122, 40.0), (134, 32.0)]
LOOP_IN = [(140, 4.0), (128, -5.0), (100, -6.0), (70, -6.0), (34, -6.0)]
LOOP_TIP = (143.0, 18.0)
WINDOW = dict(s=(58.0, 128.0), n=(7.0, 31.0))   # the finger window (rounded slot): 70 x 24 mm
FINGER_TPU = dict(s=(70.0, 118.0), t=2.6)      # soft strip on the finger bar's inner face
STOP_R = 18.0                     # handle A's stop pin (M3) from the axle; it runs in an arc slot in the bottom plate
HUB_H_R = 5.0                     # handle A's lower hub: bearing in the bottom plate, hex end into the servo coupler
HEX_AF = 7.5

# ---- plates, electronics, servo ------------------------------------------------------------------------------------
STANDOFFS = [(0.0, 77.0), (-44.0, 40.0), (44.0, 40.0), (-35.0, 2.0), (35.0, 2.0)]
STANDOFF_R = 5.0
EBOX = dict(x=(BOT[0] - 45.5, BOT[0]), y=34.5, z=(4.0, 72.0), wall=2.4)   # under the bottom plate
# Feetech STS3215 as the jaw encoder (torque off), coaxial with the scissor axle under the bottom plate: its horn is
# screwed to `servo_coupler`, which takes handle A's hex end. Handle A turns q / 2, so the servo reads q / 2.
SERVO_STEP = SRC / "feetech" / "STS3215_03a.step"
HORN_X = BOT[0] - 3.5             # the servo horn's face (G x)
COUPLER_X = (HORN_X, BOT[0] - 0.5)



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
    if name in ("handle_a", "stop_pin", "servo_coupler", "grip_tpu_a"):
        return rot_about(H, a * R_K / R_H)
    if name in ("handle_b", "grip_tpu_b"):
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


def _unit(deg):
    return np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])


def _section(c2, d2, x_mid, w, h, r):
    """A rounded-rectangle section on the plane through the (y, z) point c2 normal to the in-plane direction d2:
    width w across the handle (in the plane), height h along G x."""
    from build123d import RectangleRounded
    o = (float(x_mid), float(c2[0]), float(c2[1]))
    zdir = (0.0, float(d2[0]), float(d2[1]))
    xdir = (0.0, -float(d2[1]), float(d2[0]))
    return Plane(origin=o, x_dir=xdir, z_dir=zdir) * RectangleRounded(w, h, r)


def _handle_frame(which):
    """Handle-local (s, n) -> G (y, z) at q = 0: s along the handle from the axle, n outwards."""
    opened, _ = handle_angles()
    tail = opened if which == "a" else 180 - opened
    t = _unit(tail)
    o = _unit(tail - 90.0) if which == "a" else _unit(tail + 90.0)
    return lambda sn: H + sn[0] * t + sn[1] * o, t


def _sn_face(f, outer, tip, inner):
    """Closed face from an outer spline, a rounded tip and an inner spline, in handle-local (s, n)."""
    from build123d import Line, Spline, Wire, make_face
    P = lambda pts: [tuple(map(float, f(p))) for p in pts]      # noqa: E731
    po, pi_ = P(outer), P(inner)
    pt = tuple(map(float, f(tip)))
    w = Wire([Spline(*po), Spline(po[-1], pt, pi_[0]), Spline(*pi_), Line(pi_[-1], po[0])])
    return make_face(w)


def _round_all(solid, r, only=None):
    """Round the top and bottom edges of an extruded grip (largest radius from r down that works), except the flat
    joint face where the arm joins. only: round just these edges."""
    from build123d import GeomType
    joint = [f for f in solid.faces() if f.geom_type == GeomType.PLANE and abs(f.normal_at().X) < 1e-6]
    es = only if only is not None else [e for e in solid.edges() if abs(e.tangent_at(0.5).X) < 1e-6
                                        and not any(e.is_same(x) for f in joint for x in f.edges())]
    for rr in np.arange(r, 1.4, -0.5):
        try:
            return fillet(es, float(rr))
        except Exception:                               # noqa: BLE001
            continue
    return solid


HANDS = {"right": "rh", "left": "lh"}


def grip_kind(which, hand="right"):
    """'loop' or 'palm' for handle `which` of the `hand` build (the loop goes on the side the fingers point to)."""
    return "loop" if (which == "a") == (hand == "right") else "palm"


def grip(which, hand="right"):
    """The grip body of a handle (without the web and the gear), full and its TPU insert, at q = 0."""
    f, _ = _handle_frame(which)
    if grip_kind(which, hand) == "palm":
        body = slab(_sn_face(f, PALM_OUT, PALM_TIP, PALM_IN), GRIP_X)
        body = _round_all(body, GRIP_R)
        from build123d import Line, Spline, Wire, make_face
        P = lambda pts: [tuple(map(float, f(p))) for p in pts]      # noqa: E731
        lo = P([(50, 0.0)] + [(sv, nv - PALM_TPU) for sv, nv in PALM_OUT[1:-1]] + [(116, 0.0)])
        out = P([(50, 40.0), (116, 40.0)])
        w = Wire([Spline(*lo[1:-1]), Line(lo[-2], out[1]), Line(out[1], out[0]), Line(out[0], lo[1])])
        tpu = body & slab(make_face(w), TPU_X)
    else:
        body = _round_all(slab(_sn_face(f, LOOP_OUT, LOOP_TIP, LOOP_IN), GRIP_X), 6.0)
        (s0, s1), (n0, n1) = WINDOW["s"], WINDOW["n"]
        from build123d import SlotCenterToCenter
        c = f(((s0 + s1) / 2, (n0 + n1) / 2))
        _, t = _handle_frame(which)
        win = Pos(*c) * Rot(0, 0, _deg(t)) * SlotCenterToCenter(s1 - s0 - (n1 - n0), n1 - n0)
        cut = slab(win, (GRIP_X[0] - 1, GRIP_X[1] + 1))
        body = body - cut
        wedges = [e for e in body.edges() if abs(e.tangent_at(0.5).X) < 1e-6
                  and (cut & Pos(*e.center()) * Box(0.5, 0.5, 0.5)).volume > 0]   # the window's rims
        body = _round_all(body, 4.0, only=wedges)
        (a0, a1), tt = FINGER_TPU["s"], FINGER_TPU["t"]
        cc = f(((a0 + a1) / 2, n1 + tt / 2 - 0.5))
        strip = Pos(*cc) * Rot(0, 0, _deg(t)) * SlotCenterToCenter(a1 - a0 - tt - 1, tt + 1)
        tpu = body & slab(strip, TPU_X)
    return body, tpu


def handle(which="a", hand="right"):
    """Scissor handle: gear sector and web in its layer, a rounded loft up to the grip, and the grip (grip()), less
    its soft insert. A (L1, -y side) meshes the left knuckle gear, carries the servo's hex drive and the stop screw;
    B (L3, +y) meshes the right knuckle gear. Right hand: A is the finger loop, B the palm bar; left hand: the other
    way round. Squeezing closes the jaws."""
    from build123d import loft
    ph = phases()
    lay = L1 if which == "a" else L3
    sg = 1 if which == "a" else -1
    opened, _ = handle_angles()
    f, t = _handle_frame(which)
    full = gear_sketch(HANDLE_TEETH, H, ph["ha"] if which == "a" else ph["hb"], clearance=True)
    to_k = _deg((P2L if which == "a" else P2R) - H)
    turn = math.degrees(kin.g()["q_max"]) * R_K / R_H * sg
    lo, hi = sorted((to_k, to_k - turn))
    sector = full & wedge(H, R_H + 3 * MODULE, lo - 10, hi + 10)
    D_WEB = 30.0
    web = sector + C(H, 9.5) + hull(C(H, 8.0), C(H + t * D_WEB, 7.0))
    p = slab(web, lay)
    arm = loft([_section(f((D_WEB - 6.0, 0.0)), t, sum(lay) / 2, 14.0, lay[1] - lay[0], 2.0),
                _section(f((44.0, 1.0)), t, sum(GRIP_X) / 2, 14.0, GRIP_X[1] - GRIP_X[0] - 6.0, 4.0)])
    body, tpu = grip(which, hand)
    p = p + arm + body - tpu
    p -= xcyl(H, PIN_BORE / 2 if which == "b" else 1.6, (-30, 30))
    if which == "a":
        p -= xcyl(H, 2.8, (L1[1] - 8, L1[1] + 0.1))                      # M4 heat-set insert, axle bolt from the top
        p += xcyl(H, HUB_H_R, (BOT[0] + 0.2, L1[0] + 0.1))                # hub: bearing in the bottom plate
        p += slab(_hex(HEX_AF), (COUPLER_X[1] - 2.4, BOT[0] + 0.3))        # hex end into the servo coupler
        sp = H + STOP_R * _unit(opened + 8)
        p -= xcyl(sp, 2.0, (L1[0] - 0.1, L1[0] + 5))                      # M3 heat-set insert for the stop screw
    return p


def grip_tpu(which, hand="right"):
    return grip(which, hand)[1]


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
    the record button pod, and the electronics box + servo under it."""
    out = plate_outline() + rect(-EBOX["y"], EBOX["y"], EBOX["z"][0], EBOX["z"][1])
    p = slab(out, BOT) + _plate_bosses(top=False)
    for s in STANDOFFS:
        p += slab(C(s, STANDOFF_R), (BOT[1], TOP[0]))
        p -= xcyl(s, 2.0, (TOP[0] - 7, TOP[0] + 0.1))                   # M3 insert
    for c in (P1L, P1R, P2L, P2R):
        p -= xcyl(c, PIN_D / 2 - 0.05, (BOT[0] - 1, 30))                # press fit for the dowels
    p -= xcyl(H, HUB_H_R + 0.25, (BOT[0] - 1, 0))                      # handle A's hub
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
    """Top plate and the pivot spacers; the mast's foot screws down into it (4 x M3 inserts)."""
    import head
    out = plate_outline() + rect(-head.MAST_V, head.MAST_V, head.FOOT["z"][0], head.FOOT["z"][1])
    p = slab(out, TOP) + _plate_bosses(top=True)
    for s in STANDOFFS:
        p -= xcyl(s, 1.7, (TOP[0] - 1, TOP[1] + 1))
        p -= xcyl(s, 3.1, (TOP[1] - 1.5, TOP[1] + 1))                    # head recess
    for c in (P1L, P1R, P2L, P2R):
        p -= xcyl(c, PIN_D / 2 - 0.05, (-30, TOP[1] + 1))
    p -= xcyl(H, 2.2, (0, TOP[1] + 1))                                 # M4 axle bolt
    p -= xcyl(H, 4.2, (TOP[1] - 3, TOP[1] + 1))
    for y, z in head.mast_foot_holes():
        p -= xcyl((y, z), 2.0, (TOP[1] - 4.5, TOP[1] + 0.1))
    return p


def tab_mast():
    import head
    return head.tab_mast(TOP[1])


def _ebox_holes():
    (z0, z1), y = EBOX["z"], EBOX["y"]
    return [(y - 4, z0 + 4), (-y + 4, z0 + 4), (y - 4, z1 - 4), (-y + 4, z1 - 4)]


PICO = dict(holes=(47.0, 11.4), standoff=5.0)


def ebox_cover():
    """Box under the bottom plate: the STS3215 on its floor (4 x M2 into its bottom holes), the Pico 2 on standoffs
    on the +y wall and the USB 3 hub on the -y side. Open towards the plate, screwed up into it at the corners."""
    (x0, x1), y, (z0, z1), w = EBOX["x"], EBOX["y"], EBOX["z"], EBOX["wall"]
    outer = slab(rect(-y, y, z0, z1), (x0, x1))
    outer = fillet(outer.edges().filter_by(Axis.X), 5.0)
    outer = fillet(outer.edges().group_by(Axis.X)[0], 3.0)
    inner = slab(rect(-y + w, y - w, z0 + w, z1 - w), (x0 + w, x1 + 1))
    box = outer - inner
    for yy, zz in _ebox_holes():
        box += slab(C((yy, zz), 4.0), (x0 + w, x1))
        box -= xcyl((yy, zz), 1.7, (x0 - 1, x1 + 1))
        box -= xcyl((yy, zz), 3.0, (x0 - 1, x0 + 2.0))
    holes, xb = servo_holes_bottom()
    for yy, zz in holes:                                                # servo: M2 x 8 through the floor
        box += xcyl((yy, zz), 3.0, (x0 + w - 0.01, xb))
        box -= xcyl((yy, zz), 1.1, (x0 - 1, xb + 1))
        box -= xcyl((yy, zz), 2.0, (x0 - 1, x0 + 1.4))
    # Pico 2 on four M2 standoffs on the +y wall (board in the x-z plane)
    pc = (sum(EBOX["x"]) / 2, sum(EBOX["z"]) / 2)
    for dx in (-PICO["holes"][1] / 2, PICO["holes"][1] / 2):
        for dz in (-PICO["holes"][0] / 2, PICO["holes"][0] / 2):
            st = Pos(pc[0] + dx, y - w - PICO["standoff"] / 2 + 0.01, pc[1] + dz) * Rot(90, 0, 0) * \
                Cylinder(2.6, PICO["standoff"])
            box += st
            box -= Pos(pc[0] + dx, y - w - 2.0, pc[1] + dz) * Rot(90, 0, 0) * Cylinder(0.8, 8.0)
    box -= Pos(x1 - 14.0, y, (z0 + z1) / 2 + 10) * Box(9.0, 3 * w, 13.0)        # USB-C out (to the PC), +y side
    box -= Pos(x1 - 4.0, 0, z1) * Box(6.0, 12.0, 3 * w)                         # D405 + button cables in (front)
    box -= Pos(x1 - 4.0, 0, z0) * Box(6.0, 12.0, 3 * w)
    for zz in np.arange(z0 + 14, z1 - 10, 6.0):                                 # vents in the floor
        box -= Pos(x0 + w / 2, -y + 12.0, zz) * Box(3 * w, 14.0, 2.5)
    return box


# ---- servo ---------------------------------------------------------------------------------------------------------
def servo_matrix():
    """4 x 4: the STS3215 model's frame -> G. Its horn axis (model x = 12.5, y = 0, along +z) on the scissor axle,
    horn up against the coupler, body forward under the bottom plate."""
    R = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])           # model x -> -G z, y -> G y, z -> G x
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [HORN_X - 20.2, 0.0, H[1] + 12.5]
    return T


def servo_holes_bottom():
    """The servo's four bottom mounting holes (model (4.2 | -16.5, ±10.25)) in G (y, z), and its bottom face's x."""
    T = servo_matrix()
    pts = [(T @ [x, y, -19.4, 1])[:3] for x in (4.2, -16.5) for y in (-10.25, 10.25)]
    return [(p[1], p[2]) for p in pts], pts[0][0]


def servo_coupler():
    """Disc between handle A's hex end and the servo horn (4 x M2 into the horn's holes on r 7 mm)."""
    x0, x1 = COUPLER_X
    c = xcyl(H, 10.5, (x0, x1))
    c -= slab(_hex(HEX_AF + 0.3), (x1 - 2.6, x1 + 0.1))
    for k in range(4):
        a = math.radians(45 + 90 * k)
        c -= xcyl((H[0] + 7.0 * math.cos(a), H[1] + 7.0 * math.sin(a)), 1.1, (x0 - 1, x1 + 1))
        c -= xcyl((H[0] + 7.0 * math.cos(a), H[1] + 7.0 * math.sin(a)), 2.1, (x1 - 1.4, x1 + 0.1))
    c -= xcyl(H, 3.0, (x0 - 0.1, x0 + 1.2))                              # the horn's centre screw head
    return c


def _hex(af):
    r = af / 2 / math.cos(math.pi / 6)
    return Polygon(*[(H[0] + r * math.cos(math.radians(30 + 60 * k)), H[1] + r * math.sin(math.radians(30 + 60 * k)))
                     for k in range(6)], align=None)


def camera_pose():
    import head
    return head.camera_pose()


def flange_z():
    import head
    return head.flange_z()


# ====================================================================================================================
PARTS = {
    "base_bottom": base_bottom, "base_top": base_top, "tab_mast": tab_mast,
    "knuckle_gear_left": lambda: knuckle_gear("left"), "knuckle_gear_right": lambda: knuckle_gear("right"),
    "servo_coupler": servo_coupler,
    "ebox_cover": ebox_cover,
}


def _head(name, *a):
    import head
    return lambda: getattr(head, name)(*a)


for _hand, _h in HANDS.items():
    for _w in "ab":
        PARTS[f"handle_{_w}_{_h}"] = (lambda w, hd: lambda: handle(w, hd))(_w, _hand)
        PARTS[f"grip_tpu_{_w}_{_h}"] = (lambda w, hd: lambda: grip_tpu(w, hd))(_w, _hand)

PARTS.update({"d405_hood": _head("d405_hood"), "quest_arm_right": _head("quest_arm", "right"),
              "quest_arm_left": _head("quest_arm", "left")})


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
