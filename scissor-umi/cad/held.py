"""The handheld in a hand, for the renders: Meta's rigged hand (human_hand.py) posed on the grips, with a forearm.

Held from below, palm up: the four fingers curl up through the loop's window and pull its far bar, the heel of the hand
sits under the palm bar, the thumb and index finger forward towards the record button. The right hand holds the
right-hand build; the left hand (the mirror of the right) holds the left-hand build.
"""
import math

import numpy as np
import trimesh

import assembly as A
import scissor as S

SKIN, SLEEVE = "#e3b48f", "#4a5361"
# flexion (deg) per joint: the fingers hook up through the window, the thumb lies along the base
CURLS = {"index1": 62, "index2": 78, "index3": 30, "middle1": 64, "middle2": 80, "middle3": 30,
         "ring1": 66, "ring2": 80, "ring3": 30, "pinky1": 68, "pinky2": 78, "pinky3": 30,
         "thumb1": -15, "thumb2": 0, "thumb3": 5}
FOREARM = [(0, 27.0, 19.0), (30, 29.5, 19.5), (60, 31.0, 21.5), (100, 34.0, 25.0), (150, 37.0, 28.5),
           (200, 39.0, 30.5), (260, 40.0, 31.0)]
SLEEVE_D = 105.0
K = 72


def _tube(rings):
    k = len(rings[0])
    V = np.vstack(rings)
    F = []
    for i in range(len(rings) - 1):
        for j in range(k):
            a, b = i * k + j, i * k + (j + 1) % k
            F += [[a, b, a + k], [b, b + k, a + k]]
    c = len(V)
    V = np.vstack([V, rings[-1].mean(0)])
    for j in range(k):
        a, b = (len(rings) - 1) * k + j, (len(rings) - 1) * k + (j + 1) % k
        F.append([c, b, a])
    m = trimesh.Trimesh(V, np.array(F), process=False)
    if m.volume < 0:
        m.invert()
    return m


def forearm(x0=6.0):
    """Forearm and sleeve in the hand frame, from x = x0 (just inside the hand's wrist) back along -x."""
    th = np.linspace(-np.pi, np.pi, K, endpoint=False)
    d_, w_, h_ = np.array(FOREARM).T
    arm, sl = [], []
    for d in np.r_[0, 5, 12, 22, 35, 50, 70, 95, 120, 150, 170, 174]:
        w, h = np.interp(d, d_, w_), np.interp(d, d_, h_)
        r = (np.abs(np.cos(th) / w) ** 2.4 + np.abs(np.sin(th) / h) ** 2.4) ** (-1 / 2.4)
        ring = np.column_stack([np.full(K, x0 - d), r * np.cos(th), r * np.sin(th)])
        if d <= 170:
            arm.append(ring)
        if d >= SLEEVE_D:
            ro = r + 5.0 + 1.2 * np.sin(5 * th + d / 23.0)
            sl.append(np.column_stack([np.full(K, x0 - d), ro * np.cos(th), ro * np.sin(th)]))
    return _tube(arm), _tube(sl)


PITCH = 35.0                      # the fingers tilt up (and the wrist drops) this much from horizontal
DROP = 14.0                       # the knuckles' centre this far under the grips' bottom face
N_KNUCKLE = 18.0                  # the knuckles under the window: this far out from the handle line


def hand_on_grips(q, side="right", pitch=None, drop=None):
    """[(mesh, colour)] in G for the `side` build at q."""
    import human_hand
    j = {}
    hand = human_hand.hand(curls=CURLS, joints_out=j)
    arm, sleeve = forearm()
    # handle A (right hand: the loop) at q: its outward direction and the window's inner edge
    w = "a" if side == "right" else "b"
    f, t = S._handle_frame(w)
    name = "handle_a" if w == "a" else "handle_b"
    T = A._T(S.motion(name, q))
    s0, s1 = S.WINDOW["s"]
    p_in = T @ np.array([0.0, *f(((s0 + s1) / 2, N_KNUCKLE)), 1.0])
    p_out = T @ np.array([0.0, *f(((s0 + s1) / 2, 10.0)), 1.0])
    o = (p_out - p_in)[:3]
    o /= np.linalg.norm(o)
    if side == "left":                         # build the right hand's pose for the mirrored grip, then mirror
        o = o * [1, -1, 1]
        p_in = p_in * [1, -1, 1, 1]
    up = np.array([1.0, 0, 0])
    a = math.radians(PITCH if pitch is None else pitch)
    xh = math.cos(a) * o + math.sin(a) * up
    zh = -math.cos(a) * up + math.sin(a) * o
    R = np.column_stack([xh, np.cross(zh, xh), zh])          # hand x (fingers), y (thumb), z (back of the hand)
    mcp = np.mean([j[k] for k in ("index1", "middle1", "ring1", "pinky1")], 0)
    target = np.array([S.GRIP_X[0] - (DROP if drop is None else drop), p_in[1], p_in[2]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = target - R @ mcp
    out = []
    for m, c in ((hand, SKIN), (arm, SKIN), (sleeve, SLEEVE)):
        m = m.copy()
        m.apply_transform(M)
        if side == "left":
            m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
            m.invert()
        m.visual.face_colors = A._hex(c)
        out.append(m)
    return {"hand": out[0], "forearm": out[1], "sleeve": out[2]}
