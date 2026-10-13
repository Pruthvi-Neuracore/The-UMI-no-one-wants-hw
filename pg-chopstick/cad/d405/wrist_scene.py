"""The scenery for the wrist renders: a forearm and hand under the wrist saddle, with its strap, and a dexterous robot
hand carrying the same head.

Everything is built in the saddle's arm frame and returned in the gripper frame (mm):
  arm frame: origin on the saddle's concave top line at its front edge, +s along the forearm towards the hand,
  +n up out of the arm (the saddle's top line is n = 0), y as in the gripper frame.
"""
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.geometry import LineString

SKIN, STRAP = "#e3b48f", "#2c5aa0"
HERE = Path(__file__).resolve().parent
SHADOW = HERE / "source" / "shadow_hand"
# forearm cross-sections behind the wrist: (s, half-width, half-height)
FOREARM = [(16, 24.5, 16.5), (2, 26.5, 18.0), (-45, 30.0, 21.0), (-110, 34.5, 26.0), (-190, 37.5, 29.0),
           (-260, 37.0, 28.5)]
HAND_WRIST = (12.0, 1.0, 13.0)   # centre of the hand model's wrist section (hand frame), joined to the first section
STRAP_W, STRAP_T = 25.0, 1.6


def arm_frame(x1, z_top, min_t, pitch):
    """Gripper-frame pose of the arm frame for a saddle whose front edge is at x = x1, lid underside z_top."""
    a = math.radians(pitch)
    u, n = np.array([math.cos(a), 0, math.sin(a)]), np.array([-math.sin(a), 0, math.cos(a)])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, [0, 1, 0], n])        # columns: s, y, n
    T[:3, 3] = [x1, 0, z_top - min_t]
    return T


def _section(s, w, h, k=72, drop=1.0):
    t = np.linspace(0, 2 * np.pi, k, endpoint=False)
    # a slightly squarer ellipse (superellipse, exponent 2.4) reads more like a forearm than a plain ellipse
    c, sn = np.cos(t), np.sin(t)
    e = 2 / 2.4
    y = w * np.sign(c) * np.abs(c) ** e
    n = -h - drop + h * np.sign(sn) * np.abs(sn) ** e
    return np.column_stack([np.full(k, s), y, n])


def forearm(sections=FOREARM):
    rings = [_section(*sec) for sec in sections]
    k = len(rings[0])
    V = np.vstack(rings)
    F = []
    for i in range(len(rings) - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            c, d = a + k, b + k
            F += [[a, c, b], [b, c, d]]
    # caps
    for i, flip in ((0, True), (len(rings) - 1, False)):
        ctr = len(V)
        V = np.vstack([V, rings[i].mean(0)])
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            F.append([ctr, b, a] if flip else [ctr, a, b])
    m = trimesh.Trimesh(V, np.array(F), process=True)
    m.fix_normals()
    return m


def _profile(s):
    s_, w_, h_ = np.array(FOREARM).T
    return np.interp(-s, -s_, w_), np.interp(-s, -s_, h_)


def strap(s_c, slot_y, r_saddle, n_top):
    """A 25 mm strap round the forearm at s = s_c: up through both wing slots and folded over the wings' tops."""
    w, h = _profile(s_c)
    off = STRAP_T / 2 + 0.6
    n_slot = -(r_saddle - math.sqrt(max(r_saddle ** 2 - slot_y ** 2, 0.0)))   # where the wing meets the concave
    t = np.linspace(np.pi, 2 * np.pi, 40)
    e = 2 / 2.4
    c, sn = np.cos(t), np.sin(t)
    ring = np.column_stack([(w + off) * np.sign(c) * np.abs(c) ** e,
                            -h - 1.0 + (h + off) * np.sign(sn) * np.abs(sn) ** e])
    pts = [(-slot_y - 9, n_top + off), (-slot_y, n_top + off), (-slot_y, n_slot), (-w - off, -h - 1.0)]
    pts += [tuple(p) for p in ring[1:-1]]
    pts += [(w + off, -h - 1.0), (slot_y, n_slot), (slot_y, n_top + off), (slot_y + 9, n_top + off)]
    poly = LineString(pts).buffer(STRAP_T / 2, join_style=2, cap_style=2)
    m = trimesh.creation.extrude_polygon(poly, STRAP_W)          # (y, n) extruded along +z
    m.apply_translation([0, 0, -STRAP_W / 2])
    R = np.eye(4)
    R[:3, :3] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])      # (y, n, z) -> (s, y, n): s <- z, y <- x, n <- y
    m.apply_transform(R)
    m.apply_translation([s_c, 0, 0])
    return m


def human(T_arm, saddle_len, slot_y, r_saddle, n_top, droop=8.0):
    """[(mesh, colour)] in the gripper frame: the posed right hand, the forearm and the strap."""
    out = [(forearm(), SKIN), (strap(-saddle_len / 2, slot_y, r_saddle, n_top), STRAP)]
    try:
        import human_hand
        hand = human_hand.hand()
    except Exception as e:                                      # no hand model: forearm only
        print("  (no hand model:", e, ")")
        hand = None
    if hand is not None:
        # the hand's wrist section centred on the forearm's first section; fingers along +s, back of the hand up (+n)
        hand = hand.subdivide_loop(2)
        s0, _, h0 = FOREARM[0]
        T = trimesh.transformations.rotation_matrix(math.radians(droop), [0, 1, 0], [s0, 0, -h0 - 1.0])
        hand.apply_translation(np.array([s0, 0, -h0 - 1.0]) - HAND_WRIST)
        hand.apply_transform(T)
        out.insert(0, (hand, SKIN))
    for m, _ in out:
        m.apply_transform(T_arm)
    return out


def shadow(T_arm, r_saddle, saddle_len, slot_y, n_top, forearm_colour="#2b2b2e", hand_colour="#e9e9ec",
           metal="#b9bcc2"):
    """[(mesh, colour)] in the gripper frame: the Shadow Hand (MuJoCo Menagerie, Apache-2.0) with its round forearm
    under the saddle (its top line on n = 0), wrist and palm towards +s, back of the hand up, and the strap; plus the
    pose of the hand's mounting plate (for putting the whole thing on a robot flange)."""
    out = []
    T_sh = np.eye(4)
    # Shadow forearm frame: z along the arm (mounting plate at z = 0, wrist at z ~ 213), palm towards +x
    #   -> arm frame: s = z - z_front, y = y_sh, n = -x - r_forearm
    z_front = SADDLE_FRONT_Z
    T_sh[:3, :3] = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    T_sh[:3, 3] = [-z_front, 0, -R_SHADOW]
    for f in sorted(SHADOW.glob("shadow_*.ply")):
        m = trimesh.load(f, process=False)
        tag = f.stem.split("_", 1)[1]
        col = {"forearm": forearm_colour, "hand": hand_colour, "metal": metal}.get(tag, hand_colour)
        out.append((m.apply_transform(T_sh), col))
    out.append((strap_round(-saddle_len / 2, slot_y, r_saddle, n_top), STRAP))
    for m, _ in out:
        m.apply_transform(T_arm)
    return out, T_arm @ T_sh


R_SHADOW = 68.0           # Shadow Hand forearm radius
SADDLE_FRONT_Z = 182.0    # the saddle's front edge, along the Shadow forearm (its top cap ends at z ~ 200)


def strap_round(s_c, slot_y, r, n_top):
    """A strap round a round forearm of radius r whose top line is n = 0 (the robot version)."""
    off = STRAP_T / 2 + 0.4
    n_slot = -(r - math.sqrt(max(r ** 2 - slot_y ** 2, 0.0)))
    t = np.linspace(math.atan2(n_slot + r, slot_y), -np.pi - math.atan2(n_slot + r, slot_y), 80)
    ring = np.column_stack([(r + off) * np.cos(t), -r + (r + off) * np.sin(t)])
    pts = [(-slot_y - 9, n_top + off), (-slot_y, n_top + off)] + [tuple(p) for p in ring[::-1]] + \
          [(slot_y, n_top + off), (slot_y + 9, n_top + off)]
    poly = LineString(pts).buffer(STRAP_T / 2, join_style=2, cap_style=2)
    m = trimesh.creation.extrude_polygon(poly, STRAP_W)
    m.apply_translation([0, 0, -STRAP_W / 2])
    R = np.eye(4)
    R[:3, :3] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    m.apply_transform(R)
    m.apply_translation([s_c, 0, 0])
    return m
