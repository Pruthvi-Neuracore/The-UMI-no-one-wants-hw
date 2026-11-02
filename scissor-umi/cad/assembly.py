"""Assemblies as trimesh meshes in G: the handheld at any q, and the robot twin (the Robotiq gripper posed at q with the
collar and the same head). Also clearance checks.

    python assembly.py --check     # interference between moving parts over the stroke, and the head on both builds
"""
import argparse
import math
from functools import lru_cache

import numpy as np
import trimesh

import kin
import scissor as S

CACHE = S.BUILD / "cache"
QUEST_DIR = S.SRC / "quest"
ROBOTIQ_DIR = S.SRC / "robotiq_2f_140"

COLOURS = {"blue": "#1f5fae", "white": "#f4f6f8", "tpu": "#2b2e33", "camera": "#1d1f22", "steel": "#b9bec5",
           "quest": "#3b3f45", "robotiq": "#2a2c30", "robotiq_metal": "#9aa1a9", "record": "#2e9e44"}

# part name -> (STEP name, colour)
HANDHELD = {
    "base_bottom": ("base_bottom", "blue"), "base_top": ("base_top", "blue"),
    "knuckle_gear_left": ("knuckle_gear_left", "white"), "knuckle_gear_right": ("knuckle_gear_right", "white"),
    "outer_link_left_lo": ("outer_link", "white"), "outer_link_left_hi": ("outer_link", "white"),
    "outer_link_right_lo": ("outer_link", "white"), "outer_link_right_hi": ("outer_link", "white"),
    "finger_left": ("finger_left", "blue"), "finger_right": ("finger_right", "blue"),
    "pad_left": ("pad_tpu", "tpu"), "pad_right": ("pad_tpu", "tpu"),
    "handle_a": ("handle_a", "white"), "handle_b": ("handle_b", "white"),
    "ebox_cover": ("ebox_cover", "white"),
}
HEAD = {"head": ("head_{side}", "blue")}
ROBOT = {"robot_collar_iso50": ("robot_collar_iso50", "blue")}


def _hex(c):
    c = COLOURS.get(c, c).lstrip("#")
    return [int(c[i:i + 2], 16) for i in (0, 2, 4)] + [255]


@lru_cache(None)
def step_mesh(name):
    """A part's STEP as a mesh (cached as STL under build/, never shipped)."""
    from build123d import export_stl, import_step
    p = CACHE / f"{name}.stl"
    src = S.STEP_DIR / f"{name}.step"
    if not p.exists() or p.stat().st_mtime < src.stat().st_mtime:
        CACHE.mkdir(parents=True, exist_ok=True)
        export_stl(import_step(str(src)), str(p), tolerance=0.03, angular_tolerance=0.15)
    return trimesh.load(p)


def _T(loc):
    """build123d Location -> 4 x 4."""
    from OCP.gp import gp_Trsf
    t = loc.wrapped.Transformation()
    return np.array([[t.Value(i, j) for j in range(1, 5)] for i in range(1, 4)] + [[0, 0, 0, 1.0]])


def _place(name, step, colour, T):
    m = step_mesh(step).copy()
    if name == "pad_right":                                     # one pad part, turned over for the right jaw
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))
    if name.endswith(("_lo", "_hi")):                          # the outer link is built in L1; _hi sits in L3
        if name.endswith("_hi"):
            m.apply_translation([S.L3[0] - S.L1[0], 0, 0])
        if "_right_" in name:
            m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
            m.invert()
    m.apply_transform(T)
    m.visual.face_colors = _hex(colour)
    return m


def handheld(q, side="right", quest=True):
    """{name: mesh} for the handheld at q. side: which hand's Quest controller (the mechanism is the same)."""
    out = {}
    for n, (step, col) in HANDHELD.items():
        out[n] = _place(n, step, col, _T(S.motion(n, q)))
    out.update(head_meshes(side, quest))
    return out


def head_meshes(side="right", quest=True):
    out = {"head": _place("head", f"head_{side}", "blue", np.eye(4))}
    cup = step_mesh("_d405_cup").copy() if (S.STEP_DIR / "_d405_cup.step").exists() else None
    if cup is not None:
        cup.visual.face_colors = _hex("blue")
        out["d405_cup"] = cup
    out["d405"] = d405_envelope()
    if quest:
        sup = step_mesh(f"_quest_support_{side}").copy()
        sup.visual.face_colors = _hex("blue")
        out["quest_support"] = sup
        out["quest_controller"] = controller(side)
    return out


def d405_envelope(tilt=S.TILT_DEG):
    from shapely.geometry import box as rect
    outline = rect(-16.75, -16.75, 16.75, 16.75).buffer(4.0)
    m = trimesh.creation.extrude_polygon(outline, S.CAM_DEPTH - 0.6)
    m.apply_translation([0, 0, S.CUP_FLOOR + 0.6])
    m.apply_transform(S.cup_matrix(tilt))
    m.visual.face_colors = _hex("camera")
    return m


def _ring_mesh(side):
    return step_mesh(f"_quest_support_{side}")


def controller(side="right"):
    """The Touch Pro standing in HandUMI's ring, face forward (as in pg-chopstick d405 ebox.controller_envelope, here
    in G: the ring's axis is G x)."""
    f = QUEST_DIR / "MetaQuestTouchPro_Right.stl"
    if f.exists():
        m = trimesh.load(f)
        v = m.vertices
        h = v[v[:, 1] > 15.0] - v[v[:, 1] > 15.0].mean(0)
        a = np.linalg.eigh(h.T @ h)[1][:, -1]
        a = a if a[2] > 0 else -a
        fwd = np.array([0.0, 1.0, 0.0])
        fwd = fwd - fwd.dot(a) * a
        fwd /= np.linalg.norm(fwd)
        R = np.eye(4)
        R[:3, :3] = np.column_stack([fwd, np.cross(a, fwd), a]).T     # -> kit frame: X forward, Z up the handle
        m.apply_transform(R)
        z0 = m.bounds[0, 2]
        c = []
        for dz in np.arange(20.0, 51.0, 5.0):
            loop = max(m.section([0, 0, 1], [0, 0, z0 + dz]).discrete, key=lambda l: np.ptp(l[:, 0]) * np.ptp(l[:, 1]))
            c.append([z0 + dz, (loop[:, 0].max() + loop[:, 0].min()) / 2, (loop[:, 1].max() + loop[:, 1].min()) / 2])
        c = np.array(c)
        d = np.array([np.polyfit(c[:, 0], c[:, 1], 1)[0], np.polyfit(c[:, 0], c[:, 2], 1)[0], 1.0])
        m.apply_transform(trimesh.geometry.align_vectors(d / np.linalg.norm(d), [0, 0, 1.0]))
    else:
        m = trimesh.creation.capsule(height=70.0, radius=17.0)
        m.apply_translation([0, 0, 35])
    K4 = np.eye(4)
    K4[:3, :3] = S.K
    m.apply_transform(K4)                                              # kit -> G (handle along G x)
    ring = step_mesh("_quest_support_right")
    T = S.t_support()
    cy, cz = (T @ [*S.RING_C, 0, 1])[[1, 2]]
    x_lo, x_hi = ring.bounds[:, 0]
    m.apply_translation([x_lo - 12.0 - m.vertices[:, 0].min(), 0, 0])
    for _ in range(3):
        b = m.vertices[(m.vertices[:, 0] > x_lo) & (m.vertices[:, 0] < x_hi)]
        m.apply_translation([0, cy - (b[:, 1].min() + b[:, 1].max()) / 2, cz - (b[:, 2].min() + b[:, 2].max()) / 2])
    if side == "left":
        m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
        m.invert()
    m.visual.face_colors = _hex("quest")
    return m


# ---- the robot gripper (Robotiq meshes, reference only) -------------------------------------------------------------
def _rx(a):
    return trimesh.transformations.rotation_matrix(a, [1, 0, 0])


def _tr(p):
    return trimesh.transformations.translation_matrix(p)


def robotiq(q):
    """The Robotiq 2F-140 posed at q in G (meshes from source/robotiq_2f_140, in metres -> mm)."""
    g = kin.g()

    def load(n):
        m = trimesh.load(ROBOTIQ_DIR / f"robotiq_arg2f_{n}.stl")
        if m.extents.max() < 1:
            m.apply_scale(1000.0)
        return m

    out = {}
    cpl = load("coupling")
    cpl.apply_translation([0, 0, -g["coupling_to_base"]])
    out["robotiq_coupling"] = cpl
    out["robotiq_base"] = load("base_link")
    a = kin.link_angle(q)
    af = kin.link_angle(0.0) + g["finger_angle"]
    for side, Rs in (("left", np.eye(4)), ("right", trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))):
        j = kin.joints(q)
        To = _tr([0, *g["outer_pivot"]]) @ _rx(a)
        out[f"robotiq_outer_knuckle_{side}"] = load("140_outer_knuckle").apply_transform(Rs @ To)
        out[f"robotiq_outer_finger_{side}"] = load("140_outer_finger").apply_transform(
            Rs @ To @ _tr([0, *g["outer_to_finger"][0]]))
        out[f"robotiq_inner_knuckle_{side}"] = load("140_inner_knuckle").apply_transform(
            Rs @ _tr([0, *g["inner_pivot"]]) @ _rx(a))
        out[f"robotiq_inner_finger_{side}"] = load("140_inner_finger").apply_transform(Rs @ _tr([0, *j["B"]]) @ _rx(af))
        pw, pl, pt = g["pad_size"]
        pad = trimesh.creation.box([pw, pl, pt])
        out[f"robotiq_pad_{side}"] = pad.apply_transform(
            Rs @ _tr([0, *j["B"]]) @ _rx(af) @ _tr([0, *g["pad_offset"]]))
    for n, m in out.items():
        m.visual.face_colors = _hex("robotiq_metal" if "knuckle" in n or "outer_finger" in n else "robotiq")
    return out


def robot(q, side="right", quest=True):
    out = robotiq(q)
    out["robot_collar_iso50"] = _place("robot_collar_iso50", "robot_collar_iso50", "blue", np.eye(4))
    out.update(head_meshes(side, quest))
    return out


def export_refs():
    """Placed reference parts (HandUMI cup, controller support) as STEP so the assemblies can mesh them."""
    from build123d import export_step
    S.STEP_DIR.mkdir(parents=True, exist_ok=True)
    export_step(S.cup(), str(S.STEP_DIR / "_d405_cup.step"))
    for side in ("right", "left"):
        export_step(S.quest_support(side), str(S.STEP_DIR / f"_quest_support_{side}.step"))


# ---- checks --------------------------------------------------------------------------------------------------------
def _overlap(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold")
        return float(i.volume) if i is not None and not i.is_empty else 0.0
    except Exception as e:                                         # noqa: BLE001
        return float("nan")


MOVING_PAIRS_SKIP = set()


def check(steps=8):
    names = list(HANDHELD)
    bad = []
    for q in np.linspace(0, kin.g()["q_max"], steps):
        a = handheld(q, quest=False)
        a = {k: v for k, v in a.items() if k in names or k == "head"}
        keys = list(a)
        for i in range(len(keys)):
            for k in keys[i + 1:]:
                v = _overlap(a[keys[i]], a[k])
                if v > 1.0:
                    bad.append((round(q, 3), keys[i], k, round(v, 1)))
    for b in bad:
        print("overlap", b)
    print(f"{len(bad)} overlapping pairs over {steps} poses")
    # head and controller vs the robot gripper and the handheld
    for build, f in (("robot", robot), ("handheld", handheld)):
        for q in (0.0, kin.g()["q_max"]):
            a = f(q)
            extras = [k for k in a if k.startswith(("quest", "d405"))]
            for e in extras:
                for k, m in a.items():
                    if k in extras or k in ("head",):
                        continue
                    v = _overlap(a[e], m)
                    if v > 1.0:
                        print(f"{build} q={q}: {e} overlaps {k} by {v:.0f} mm3")
    gaps = {q: kin.pad_gap(q) for q in (0.0, kin.g()["q_max"])}
    print("pad gap open / closed:", {k: round(v, 1) for k, v in gaps.items()})
    print("camera pose (G, mm):\n", S.camera_pose().round(3))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--refs", action="store_true")
    a = ap.parse_args()
    if a.refs or not (S.STEP_DIR / "_d405_cup.step").exists():
        export_refs()
    if a.check:
        check()
