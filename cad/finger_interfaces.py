"""Robot finger interfaces on HandUMI's finger links. This table is the one place to edit: the flange holes
(finger_link.py), the tip placement (tips.py, assembly.py) and the Hand-E rack shim (hande_tip.py) are all computed
from it, and `python finger_interfaces.py` checks the combined layout.

Each interface is a robot's own finger mount, i.e. what its carriage offers the finger, measured from the
manufacturer's drawing or CAD (see `source`). Both finger links carry every interface at once, so the robot's real
finger, or a printed copy of it, bolts straight onto the link with its own screws.

Pattern frame (one per interface, for its reference finger file): origin at the hole-pattern centre on the mating face;
  u  along the closing axis, pointing at the other jaw (toward the gripping face),
  w  out of the mating face into the finger (the way the finger grows; the screws run along it),
  v  = w x u.
For a mirror pair (left and right fingers are mirror images) the mirrored file has every feature at (u, -v). For a
`same` pair (one part, turned 180 degrees for the other jaw) both jaws use (u, v).

Finger-link frame: x along the closing axis, y = w (flange face at y = 0, fingers at +y), z across. On each link u
points at the other link, and the pattern is placed so the finger's centre plane lands on z = 0: that is where both
jaws meet, whatever robot they come from. A robot whose finger sits off its own screw line (the Hand-E) therefore gets
its pattern at opposite ends of the flange on the two links, exactly as its two racks sit on the robot.
"""
from dataclasses import dataclass

import numpy as np

# screw: (tapped pilot in the printed link, clearance, head diameter for a counterbore), mm. The pilot holes are sized
# for cutting the thread straight into PETG / PLA with the screw itself (or a tap); use a drop of threadlocker.
SCREW = {
    "M2": (1.7, 2.4, 4.2),
    "M2.5": (2.1, 2.9, 5.0),
    "M3": (2.6, 3.4, 6.2),
    "M4": (3.4, 4.5, 7.8),
}
DOWEL_FIT = 0.05            # press fit for a steel dowel: printed hole = dowel + this
MIN_WALL = 0.8              # thinnest printed wall allowed between two holes, or a hole and the flange edge


@dataclass(frozen=True)
class Interface:
    key: str
    name: str
    robots: str
    source: str
    screw: str
    holes: tuple                 # (u, v) hole centres
    screw_from: str = "finger"   # "finger": heads on the finger, thread cut in the link; "link": screws come from behind
                                 #   the flange into the finger's own threads / inserts (clearance + counterbore)
    depth: float = 6.0           # thread depth in the link ("finger") / screw-head access depth behind the face ("link")
    dowels: tuple = ()           # (u, v, diameter, depth)
    pockets: tuple = ()          # (u0, u1, v0, v1, depth): recesses in the face for keys standing proud of the finger
    reliefs: tuple = ()          # (u0, u1, v0, v1, w0, w1): finger material behind its mating face, cleared from the link
    centre_v: float = 0.0        # v of the finger's centre plane
    pair: str = "mirror"         # "mirror" or "same" (see above)
    du: float = 0.0              # shift of the whole finger toward the other jaw, mm
    dv: float = 0.0              # shift of the whole finger across (along v), off the jaws' centre plane, mm
    note: str = ""


INTERFACES = {
    "piper": Interface(
        "piper", "AgileX Piper", "AgileX Piper (and handumi-hw's tip library: ARX X5 and others)",
        "piper_description link7.STL: the fingertip seat on the Piper's finger, 4x M2 on 8 x 12 mm; the Piper jaw "
        "(handumi-hw) has the matching 4x 2.4 mm holes",
        "M2", tuple((su * 4.0, sv * 6.0) for su in (-1, 1) for sv in (-1, 1))),
    "hande": Interface(
        "hande", "Robotiq Hand-E", "Robotiq Hand-E on UR, ABB GoFa, FANUC CRX, ...",
        "Robotiq Hand-E manual, 5.1.4 (fingers, fingertip holder): a finger fixes to its rack with 2x M3 9 mm apart "
        "along the rack and a 2 mm dowel between them; the finger's centre sits 14.2 mm off that line "
        "(robotiq_hande_description finger mesh)",
        "M3", ((-4.5, 0.0), (4.5, 0.0)), depth=7.0, dowels=((0.0, 0.0, 2.0, 4.0),), centre_v=-14.2, pair="same"),
    "yam": Interface(
        "yam", "I2RT YAM", "I2RT YAM (linear gripper)",
        "Open-ENPIRE i2rt_UCG_Hard.stl (Wenli Xiao's YAM finger) and i2rt linear_4310 tip mesh: 2 screws 21 mm "
        "apart across the finger, into M3 heat-set inserts in the finger, plus an 18.7 x 3 x 5 mm key on the "
        "gripping side",
        "M4", ((0.0, -10.5), (0.0, 10.5)), depth=5.0,
        pockets=((5.9, 13.0, -8.6, 10.7, 5.4),), du=2.0,
        note="The YAM's own finger mesh carries M4 socket screws (7 mm heads) put in from the finger side into the "
             "carriage, 21 mm apart; the ENPIRE finger has the same 4.6 mm holes. The finger's centre is 1.3 mm off its "
             "screw line; the pattern is centred instead, so the finger sits 1.3 mm off the jaws' centre plane. du = 2 keeps "
             "its holes clear of OpenArm's."),
    "openarm": Interface(
        "openarm", "OpenArm v1", "OpenArm v1",
        "OpenArm v1 printed finger base (openarm_description finger, as re-modelled in openarm-hardshell-fingers): "
        "4x M2.5 on 10 x 15 mm",
        "M2.5", tuple((su * 5.0, sv * 7.5) for su in (-1, 1) for sv in (-1, 1)), du=-5.0,
        note="OpenArm's finger has a 5 x 7.6 mm rib behind its plate, beside the carriage (10.1-15.1 mm off centre). "
             "On HandUMI that space is the flange, which carries the other robots' holes there, so the rib is left "
             "off: HandUMI's OpenArm finger is printed without it, and a stock OpenArm finger needs it trimmed flush "
             "with the plate. The flange backs the whole plate instead. du = -5 keeps its holes clear of the Piper "
             "and YAM holes (OpenArm's fingers bend in toward each other, so they still meet)."),
}

# ------------------------------------------------------------------ the flange that carries them
FLANGE_IN = 10.0     # from the pattern axis to the flange edge facing the other jaw (x); keeps the 3.6 mm gap the
                     # two flanges have at full close
FLANGE_OUT = 12.0    # to the outer edge (away from the other jaw); room for OpenArm's outer column
FLANGE_T = 5.0       # thickness (y)
FLANGE_EDGE = 2.5    # along z the flange ends this far past the outermost hole on each side (so each link's flange
                     # is as long as its own layout needs: the Hand-E pattern sits 14.2 mm off centre, on opposite
                     # ends on the two links)


# ------------------------------------------------------------------ placement
HANDS = ("right", "left")
LINKS = ("thumb", "index")


def u_sign(hand, link):
    """Sign of u along link x: on the right hand the thumb sits at -x of the index, so its u points to +x."""
    s = 1.0 if link == "thumb" else -1.0
    return s if hand == "right" else -s


def uses_mirror(itf, hand, link):
    """Does this link carry the mirrored finger file? Right thumb: the reference file; the left hand mirrors it."""
    if itf.pair == "same":
        return False
    return (link == "index") == (hand == "right")


def frame(itf, hand, link):
    """(R, o): link point = o + R @ (u, v, w) for this interface's finger file on this link."""
    su = u_sign(hand, link)
    sv = -su                                       # v = w x u with w = +y, u = su * x
    vm = -1.0 if uses_mirror(itf, hand, link) else 1.0
    R = np.array([[su, 0, 0], [0, 0, 1.0], [0, sv * vm, 0]])
    o = np.array([su * itf.du, 0.0, -sv * vm * itf.centre_v + itf.dv])
    return R, o


def to_link(itf, hand, link, u, v, w=0.0):
    R, o = frame(itf, hand, link)
    return o + R @ np.array([u, v, w], float)


def flange_x(hand, link):
    """(x_min, x_max) of the flange on this link: FLANGE_IN toward the other jaw, FLANGE_OUT away from it."""
    su = u_sign(hand, link)
    return (-FLANGE_OUT, FLANGE_IN) if su > 0 else (-FLANGE_IN, FLANGE_OUT)


def flange_z(hand, link):
    """(z_min, z_max) of the flange on this link: FLANGE_EDGE past the outermost hole on each side."""
    r = [(f[3] - f[4] / 2, f[3] + f[4] / 2) for f in features(hand, link) if f[0] in ("hole", "dowel")]
    return min(a for a, _ in r) - FLANGE_EDGE, max(b for _, b in r) + FLANGE_EDGE


def features(hand, link, interfaces=None):
    """Every cut the flange needs on this link, in link coordinates:
    ("hole", key, x, z, d, depth, kind) with kind "tap" / "clear"; ("cbore", key, x, z, d, depth);
    ("dowel", key, x, z, d, depth); ("pocket", key, x0, x1, z0, z1, depth); ("relief", key, x0, x1, z0, z1, y0, y1)."""
    out = []
    for itf in (interfaces or INTERFACES.values()):
        tap, clear, head = SCREW[itf.screw]
        for u, v in itf.holes:
            x, _, z = to_link(itf, hand, link, u, v)
            if itf.screw_from == "finger":
                out.append(("hole", itf.key, x, z, tap, max(itf.depth, FLANGE_T), "tap"))
            else:
                out.append(("hole", itf.key, x, z, clear, FLANGE_T + 1.0, "clear"))
                out.append(("cbore", itf.key, x, z, head, itf.depth))
        for u, v, d, depth in itf.dowels:
            x, _, z = to_link(itf, hand, link, u, v)
            out.append(("dowel", itf.key, x, z, d + DOWEL_FIT, depth))
        for u0, u1, v0, v1, depth in itf.pockets:
            c = np.array([to_link(itf, hand, link, u, v) for u in (u0, u1) for v in (v0, v1)])
            out.append(("pocket", itf.key, c[:, 0].min(), c[:, 0].max(), c[:, 2].min(), c[:, 2].max(), depth))
        for u0, u1, v0, v1, w0, w1 in itf.reliefs:
            c = np.array([to_link(itf, hand, link, u, v, w) for u in (u0, u1) for v in (v0, v1) for w in (w0, w1)])
            out.append(("relief", itf.key, c[:, 0].min(), c[:, 0].max(), c[:, 2].min(), c[:, 2].max(),
                        c[:, 1].min(), c[:, 1].max()))
    return out


def check(hand, link, verbose=True):
    """Walls between round features of different interfaces, and to the flange edge and to pockets / reliefs. Returns
    the list of problems (empty = the layout prints and every interface keeps its own holes)."""
    feats = features(hand, link)
    rounds = [(f[1], f[0], f[2], f[3], f[4] / 2) for f in feats if f[0] in ("hole", "dowel")]
    boxes = [(f[1], f[0], f[2], f[3], f[4], f[5]) for f in feats if f[0] in ("pocket", "relief")]
    bad, worst = [], (np.inf, None)
    x_lo, x_hi = flange_x(hand, link)
    z_lo, z_hi = flange_z(hand, link)
    for i, (ka, ta, xa, za, ra) in enumerate(rounds):
        edge = min(xa - x_lo, x_hi - xa, za - z_lo, z_hi - za) - ra
        if edge < MIN_WALL:
            bad.append(f"{ka} {ta} at ({xa:.1f}, {za:.1f}): {edge:.2f} mm to the flange edge")
        for kb, tb, xb, zb, rb in rounds[i + 1:]:
            if ka == kb:
                continue
            wall = np.hypot(xa - xb, za - zb) - ra - rb
            worst = min(worst, (wall, f"{ka}/{kb}"))
            if wall < MIN_WALL:
                bad.append(f"{ka} {ta} ({xa:.1f}, {za:.1f}) / {kb} {tb} ({xb:.1f}, {zb:.1f}): wall {wall:.2f} mm")
        for kb, tb, x0, x1, z0, z1 in boxes:
            if ka == kb:
                continue
            dx = max(x0 - xa, 0.0, xa - x1)
            dz = max(z0 - za, 0.0, za - z1)
            wall = np.hypot(dx, dz) - ra
            worst = min(worst, (wall, f"{ka}/{kb} {tb}"))
            if wall < MIN_WALL:
                bad.append(f"{ka} {ta} ({xa:.1f}, {za:.1f}) / {kb} {tb}: wall {wall:.2f} mm")
    if verbose:
        print(f"{hand:5s} {link:5s}: thinnest wall between interfaces {worst[0]:.2f} mm ({worst[1]})"
              + ("" if not bad else "  PROBLEMS:\n    " + "\n    ".join(bad)))
    return bad


def table():
    """Markdown table of the patterns, for the README."""
    rows = ["| Interface | Screws | Pattern (along closing x across) | Extras | Robots |", "|---|---|---|---|---|"]
    for itf in INTERFACES.values():
        us = sorted({round(u, 2) for u, _ in itf.holes})
        vs = sorted({round(v, 2) for _, v in itf.holes})
        pat = f"{us[-1] - us[0]:g} x {vs[-1] - vs[0]:g} mm" if len(us) > 1 and len(vs) > 1 else \
            (f"{us[-1] - us[0]:g} mm along closing" if len(us) > 1 else f"{vs[-1] - vs[0]:g} mm across")
        extra = []
        if itf.dowels:
            extra.append(f"Ø{itf.dowels[0][2]:g} dowel")
        if itf.pockets:
            extra.append("key slot")
        if itf.reliefs:
            extra.append("rib relief")
        if itf.screw_from == "link":
            extra.append("screws from behind the flange")
        rows.append(f"| {itf.name} | {len(itf.holes)}x {itf.screw} | {pat} | {', '.join(extra) or '-'} | {itf.robots} |")
    return "\n".join(rows)


if __name__ == "__main__":
    problems = sum((check(h, l) for h in HANDS for l in LINKS), [])
    for h in HANDS:
        for l in LINKS:
            print(f"\n{h} {l}:")
            for f in features(h, l):
                print("  ", f[0], f[1], " ".join(f"{v:.2f}" if isinstance(v, float) else str(v) for v in f[2:]))
    print("\n" + table())
    raise SystemExit(1 if problems else 0)
