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

SKIN, STRAP, SLEEVE, WIRE, TPU = "#e3b48f", "#2c5aa0", "#4a5361", "#2b2e33", "#dfe5ec"
HERE = Path(__file__).resolve().parent
SHADOW = HERE / "source" / "shadow_hand"
HAND_WRIST = (12.0, 1.0, 13.0)   # where the hand model's wrist sits (hand frame), on the forearm's top line
S0, H0 = 16.0, 16.5              # ... placed at s = S0, n = -H0 - 1 in the arm frame
STRAP_W, STRAP_T = 25.0, 1.6
THUMB_W = 18.0                   # the thumb strap (20 mm hook-and-loop, drawn a little narrower where it bends)
WRIST_CUT = 8.0                  # the forearm takes over from the hand model this far into the hand (hand x)
# forearm, anatomical: (distance back from the wrist cut, half-width, half-height). Flat and tendon-y at the wrist,
# muscle bellies swelling in the upper half (added as bulges below), under a shirt sleeve from SLEEVE_D back.
FOREARM = [(30, 29.5, 19.5), (60, 31.0, 21.5), (100, 34.0, 25.0), (150, 37.0, 28.5), (200, 39.0, 30.5),
           (260, 40.0, 31.0), (330, 40.5, 31.0), (440, 41.0, 31.0), (760, 41.0, 31.0)]
SLEEVE_D = 150.0
K = 96                           # points round each ring


def arm_frame(x1, z_top, min_t, pitch):
    """Gripper-frame pose of the arm frame for a saddle whose front edge is at x = x1, lid underside z_top."""
    a = math.radians(pitch)
    u, n = np.array([math.cos(a), 0, math.sin(a)]), np.array([-math.sin(a), 0, math.cos(a)])
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, [0, 1, 0], n])        # columns: s, y, n
    T[:3, 3] = [x1, 0, z_top - min_t]
    return T


def _smooth(a, b, x):
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def _profile_d(d):
    d_, w_, h_ = np.array(FOREARM).T
    return np.interp(d, d_, w_), np.interp(d, d_, h_)


def _anatomical(d, theta):
    """Radius of the forearm's section at distance d behind the wrist cut, at polar angle theta (from +y towards +n),
    about the section's centre; and the centre's n (the top line stays 1 mm under the saddle's)."""
    w, h = _profile_d(d)
    p = 2.4
    r = (np.abs(np.cos(theta) / w) ** p + np.abs(np.sin(theta) / h) ** p) ** (-1 / p)
    def bump(c, width):
        dt = np.angle(np.exp(1j * (theta - c)))
        return np.exp(-(dt / width) ** 2)
    r = r * (1 + 0.10 * _smooth(80, 210, d) * bump(math.radians(40), 0.55)      # extensors / brachioradialis
             + 0.07 * _smooth(70, 220, d) * bump(math.radians(-115), 0.7)      # flexors
             - 0.04 * _smooth(40, 120, d) * bump(math.radians(105), 0.35))     # the ulna's flat, top inner side
    return r, -1.0 - h


def _polar(loop, centre, theta):
    """A closed (y, n) loop as radius at each angle theta about centre (assumes it is star-shaped)."""
    d = loop - centre
    a = np.arctan2(d[:, 1], d[:, 0])
    r = np.hypot(d[:, 0], d[:, 1])
    o = np.argsort(a)
    a, r = a[o], r[o]
    return np.interp(np.angle(np.exp(1j * theta)), np.r_[a - 2 * np.pi, a, a + 2 * np.pi], np.r_[r, r, r])


def _tube(rings, cap_start=False, cap_end=True):
    """Mesh through rings (each K x 3, same angular order)."""
    k = len(rings[0])
    V = np.vstack(rings)
    F = []
    for i in range(len(rings) - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            F += [[a, b, a + k], [b, b + k, a + k]]
    for i, flip, on in ((0, False, cap_start), (len(rings) - 1, True, cap_end)):
        if not on:
            continue
        c = len(V)
        V = np.vstack([V, rings[i].mean(0)])
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            F.append([c, a, b] if flip else [c, b, a])
    m = trimesh.Trimesh(V, np.array(F), process=False)
    if m.volume < 0:
        m.invert()
    return m


def forearm(wrist_loop=None, s_cut=S0 - 4.0):
    """The forearm from the wrist cut (s = s_cut, matching the hand model's section there, if given) back to the elbow,
    and the shirt sleeve over its upper half. Returns (arm, sleeve)."""
    theta = np.linspace(-np.pi, np.pi, K, endpoint=False)
    if wrist_loop is not None:
        c0 = wrist_loop.mean(0)
        r0 = _polar(wrist_loop, c0, theta)
    else:
        r0, n0 = _anatomical(0.0, theta)
        c0 = np.array([0.0, n0])
    ds = np.r_[0, 2, 5, 9, 14, 20, 28, 38, 50, 65, 80, 100, 125, 150, 180, 215, 255, 300, 350, 400, 460, 540, 640, 760]
    rings, arm_r = [], []
    for d in ds:
        ra, nc = _anatomical(d, theta)
        t = _smooth(0, 32, d)
        r = (1 - t) * r0 + t * ra
        c = (1 - t) * c0 + t * np.array([0.0, nc])
        rings.append(np.column_stack([np.full(K, s_cut - d), c[0] + r * np.cos(theta), c[1] + r * np.sin(theta)]))
        arm_r.append((d, c, r))
    arm = _tube(rings)
    # sleeve: a loose cotton sleeve, rolled once at the cuff, with soft folds
    srings_out, srings_in = [], []
    for d, c, r in arm_r:
        if d < SLEEVE_D - 1:
            continue
        fold = 1.3 * np.sin(5 * theta + d / 23.0) * _smooth(SLEEVE_D, SLEEVE_D + 50, d)
        cuff = 2.2 * np.exp(-((d - SLEEVE_D - 6) / 6.0) ** 2)
        ro = r + 5.0 + fold + cuff
        srings_out.append(np.column_stack([np.full(K, s_cut - d), c[0] + ro * np.cos(theta), c[1] + ro * np.sin(theta)]))
    for d in (SLEEVE_D, SLEEVE_D + 14):
        ra, nc = _anatomical(d, theta)
        ri = ra + 1.2
        srings_in.append(np.column_stack([np.full(K, s_cut - d), ri * np.cos(theta), nc + ri * np.sin(theta)]))
    # the cuff's edge: the inner face comes out to the outer one at d = SLEEVE_D
    sleeve = _tube(srings_in[::-1] + srings_out, cap_start=False, cap_end=False)
    return arm, sleeve


def _profile(s):
    return _profile_d(S0 - 4.0 - s)


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


# --- the thumb strap and the palm button ------------------------------------------------------------------------------
# the strap's loop round the hand (hand frame: x to the fingers, y to the thumb, z out of the back of the hand):
# across the back of the hand, through the web between thumb and index finger, back across the palm a little nearer
# the wrist, and up the outer edge. (x, y, side): side +1 back of the hand, -1 palm, 0 the web / outer edge
THUMB_LOOP = [(92, -34, 1), (98, 0, 1), (106, 22, 1), (116, 32, 0), (104, 19, -1), (90, 0, -1), (82, -32, -1),
              (87, -45, 0)]
BUTTON_AT = 0.70                 # where the palm button sits along the loop (0..1 from the first point): under the
                                 # middle and ring fingers


def _hit(mesh, origin, direction):
    loc, _, _ = mesh.ray.intersects_location([origin], [direction], multiple_hits=True)
    if not len(loc):
        return None
    return loc[np.argmin(np.linalg.norm(loc - origin, axis=1))]


def _surface_point(hand, x, y, side):
    if side:
        return _hit(hand, np.array([x, y, 150.0 * side]), np.array([0, 0, -float(side)]))
    sy = 1.0 if y > 0 else -1.0
    top = _hit(hand, np.array([x, y, 150.0]), np.array([0, 0, -1.0]))
    bot = _hit(hand, np.array([x, y, -150.0]), np.array([0, 0, 1.0]))
    if top is not None and bot is not None:                     # a thin web: its edge
        return (top + bot) / 2 + np.array([0, 0.0, 0])
    return _hit(hand, np.array([x, y + 150.0 * sy, 0.0]), np.array([0, -sy, 0.0]))


def _closed_spline(P, n=240):
    from scipy.interpolate import splev, splprep
    P = np.vstack([P, P[:1]])
    tck, _ = splprep(P.T, s=0, per=True)
    u = np.linspace(0, 1, n, endpoint=False)
    return np.array(splev(u, tck)).T, u


def _ribbon(C, N, width, thick, closed=True):
    """A flat strap along centreline C (n x 3) lying on the surface with outward normals N, width (scalar or n)."""
    T = np.roll(C, -1, 0) - np.roll(C, 1, 0) if closed else np.gradient(C, axis=0)
    T /= np.linalg.norm(T, axis=1)[:, None]
    N = N - (N * T).sum(1)[:, None] * T
    N /= np.linalg.norm(N, axis=1)[:, None]
    B = np.cross(T, N)
    w = np.broadcast_to(np.asarray(width, float), (len(C),))[:, None] / 2
    q = [C - B * w, C + B * w, C + B * w + N * thick, C - B * w + N * thick]
    n = len(C)
    V = np.vstack(q)
    F = []
    segs = n if closed else n - 1
    for i in range(segs):
        j = (i + 1) % n
        for a in range(4):
            b = (a + 1) % 4
            F += [[a * n + i, b * n + i, a * n + j], [b * n + i, b * n + j, a * n + j]]
    if not closed:
        V = np.vstack([V, C[:1], C[-1:]])
        m = trimesh.Trimesh(V, np.array(F), process=True)
        return m
    return trimesh.Trimesh(V, np.array(F), process=True)


def _normals_at(mesh, P):
    _, _, tri = trimesh.proximity.closest_point(mesh, P)
    return mesh.face_normals[tri]


def _tube_along(P, r=1.1, k=12):
    """A round wire along the polyline P."""
    T = np.gradient(P, axis=0)
    T /= np.linalg.norm(T, axis=1)[:, None]
    a = np.cross(T[0], [0, 0, 1.0])
    if np.linalg.norm(a) < 1e-6:
        a = np.cross(T[0], [0, 1.0, 0])
    U = [a / np.linalg.norm(a)]
    for t in T[1:]:                                             # parallel transport
        u = U[-1] - np.dot(U[-1], t) * t
        U.append(u / np.linalg.norm(u))
    U = np.array(U)
    W = np.cross(T, U)
    ang = np.linspace(0, 2 * np.pi, k, endpoint=False)
    rings = [p + r * (np.cos(ang)[:, None] * u + np.sin(ang)[:, None] * w) for p, u, w in zip(P, U, W)]
    return _tube(rings, cap_start=True, cap_end=True)


def thumb_strap(hand):
    """The thumb strap's loop and the palm button's pose, on the (posed) hand in the hand frame.
    Returns (loop mesh, centreline, normals, button 4x4 in the hand frame, button's index on the centreline)."""
    pts = []
    for x, y, side in THUMB_LOOP:
        p = _surface_point(hand, x, y, side)
        pts.append(p)
    P = np.array(pts)
    C, u = _closed_spline(P)
    C, _, tri = trimesh.proximity.closest_point(hand, C)       # back onto the skin
    N = hand.face_normals[tri]
    N = np.array([np.mean(N[max(0, i - 3):i + 4], 0) for i in range(len(N))])
    C = C + N * 0.6
    web = np.argmin(np.linalg.norm(C - P[3], axis=1))
    idx = np.arange(len(C))
    dist = np.minimum(np.abs(idx - web), len(C) - np.abs(idx - web))
    width = THUMB_W - 7.0 * np.exp(-(dist / 14.0) ** 2)
    loop = _ribbon(C, N, width, STRAP_T)
    i = int(BUTTON_AT * len(C))
    t = C[(i + 1) % len(C)] - C[i - 1]
    z = N[i] / np.linalg.norm(N[i])
    y = t - t.dot(z) * z
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    B = np.eye(4)
    B[:3, :3] = np.column_stack([x, y, z])
    B[:3, 3] = C[i] - z * 1.7                                   # the strap runs in the pad's tunnel
    return loop, C, N, B, i


def _palm_button():
    import palm_button as PB
    from build123d import export_stl
    from parts import ROOT
    p = ROOT / "cache" / "palm_button.stl"
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        export_stl(PB.build(), str(p), tolerance=0.02, angular_tolerance=0.15)
    return trimesh.load(p)


def human(T_arm, saddle_len, slot_y, r_saddle, n_top, droop=8.0, tab_s=7.5, wire_to=None):
    """[(mesh, colour)] in the gripper frame: the posed right hand, the forearm and its sleeve, the forearm strap, the
    thumb strap from the saddle's front tab (at s = tab_s) with the palm button, and its wire to `wire_to` (a point in
    the gripper frame: the box's or pod's button-wire hole)."""
    out = [(strap(-saddle_len / 2, slot_y, r_saddle, n_top), STRAP)]
    try:
        import human_hand
        hand0 = human_hand.hand()
    except Exception as e:                                      # no hand model: forearm only
        print("  (no hand model:", e, ")")
        hand0 = None
    if hand0 is None:
        arm, sleeve = forearm()
        out += [(arm, SKIN), (sleeve, SLEEVE)]
    else:
        hand0 = hand0.subdivide_loop(2)
        # the hand frame -> the arm frame: wrist section on the forearm's top line; fingers along +s, back of the hand
        # up (+n), drooping a little at the wrist
        P = trimesh.transformations.rotation_matrix(math.radians(droop), [0, 1, 0], [S0, 0, -H0 - 1.0]) @ \
            trimesh.transformations.translation_matrix(np.array([S0, 0, -H0 - 1.0]) - HAND_WRIST)
        loop, C, N, B, ib = thumb_strap(hand0)
        hand = hand0.copy().apply_transform(P)
        s_cut = (P @ [WRIST_CUT, 0, 0, 1])[0]
        sec = hand.section([1, 0, 0], [s_cut, 0, 0])
        wl = max(sec.discrete, key=len)[:, 1:] if sec is not None else None
        hand = hand.slice_plane([s_cut, 0, 0], [1, 0, 0], cap=False)
        arm, sleeve = forearm(wl, s_cut)
        out += [(hand, SKIN), (arm, SKIN), (sleeve, SLEEVE)]
        out.append((loop.apply_transform(P), STRAP))
        out.append((_palm_button().apply_transform(P @ B), TPU))
        # the connector: from under the tab's bar, along the back of the hand to the loop, and its free end folded back
        # over it (hook-and-loop)
        Cw = trimesh.transform_points(C, P)
        Nw = (P[:3, :3] @ N.T).T
        top = np.argmax(Cw[:, 2] - 0.02 * np.abs(Cw[:, 1]))
        skin = trimesh.util.concatenate([hand, arm])
        ss = np.linspace(tab_s, Cw[top, 0] - 4.0, 50)
        ys = np.interp(ss, [tab_s, Cw[top, 0]], [0.0, Cw[top, 1]])
        cl = []
        for s_, y_ in zip(ss, ys):
            h = _hit(skin, np.array([s_, y_, 80.0]), np.array([0, 0, -1.0]))
            cl.append(h if h is not None else np.array([s_, y_, -1.0]))
        cl = np.array(cl) + [0, 0, 0.7]
        cl = np.vstack([cl, Cw[top] + [0, 0, STRAP_T + 0.2]])
        nn = np.tile([0, 0, 1.0], (len(cl), 1))
        out.append((_ribbon(cl, nn, THUMB_W, STRAP_T, closed=False), STRAP))
        out.append((_ribbon(cl[:-8] + [0, 0, STRAP_T + 0.1], nn[:-8], THUMB_W, STRAP_T, closed=False), STRAP))
        # the wire: out of the button, round the outer (little-finger) edge of the hand on the strap, back along the
        # connector, and up into the box
        n = len(Cw)
        path = [Cw[(ib + k) % n] + Nw[(ib + k) % n] * (STRAP_T + 1.2) for k in range(0, (top - ib) % n + 1, 2)]
        path += [p + [0, -7.0, STRAP_T * 2 + 1.2] for p in cl[::-3]]
        if wire_to is not None:
            w = np.linalg.inv(T_arm) @ [*wire_to, 1]
            a, b = path[-1], w[:3]
            for t in np.linspace(0.1, 1.0, 10):
                q = a + (b - a) * t
                q[2] += 6.0 * math.sin(math.pi * t)
                path.append(q)
        out.append((_tube_along(np.array(path)), WIRE))
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
