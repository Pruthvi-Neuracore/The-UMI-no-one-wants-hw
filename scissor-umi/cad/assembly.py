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
ROBOTIQ_DIR = S.SRC / f"robotiq_2f_{kin.g()['mesh']}"

COLOURS = {"blue": "#1f5fae", "white": "#f4f6f8", "tpu": "#1b1c1f", "camera": "#1d1f22", "steel": "#b9bec5",
           "quest": "#3b3f45", "robotiq": "#2a2c30", "robotiq_metal": "#9aa1a9", "record": "#2e9e44"}

# part name -> (STEP name, colour)
HANDHELD = {
    "base_bottom": ("base_bottom", "blue"), "base_top": ("base_top", "blue"),
    "knuckle_gear_left": ("knuckle_gear_left", "white"), "knuckle_gear_right": ("knuckle_gear_right", "white"),
    "handle_a": ("handle_a", "white"), "handle_b": ("handle_b", "white"),
    "ebox_cover": ("ebox_cover", "white"),
}
# the 2F-140 jaw replicas (same colours as the robot render: metal-grey links, dark finger and pad)
TAG = kin.GRIPPER.replace("-", "")                   # 2f140 / 2f85
for _s in ("left", "right"):
    HANDHELD.update({f"outer_link_{_s}": (f"outer_link_{TAG}", "robotiq_metal"),
                     f"inner_knuckle_{_s}": (f"inner_knuckle_{TAG}", "robotiq_metal"),
                     f"inner_finger_{_s}": (f"inner_finger_{TAG}", "robotiq"),
                     f"pad_core_{_s}": (f"pad_core_{TAG}", "robotiq"),
                     f"pad_skin_{_s}": (f"pad_skin_{TAG}", "tpu")})
HANDHELD.update({"servo_coupler": ("servo_coupler", "white"), "tab_mast": ("tab_mast", "blue"),
                 "grip_tpu_a": ("grip_tpu_a", "blue"), "grip_tpu_b": ("grip_tpu_b", "blue")})


HANDED = ("handle_a", "handle_b", "grip_tpu_a", "grip_tpu_b")


def handed(step, side="right"):
    """The STEP of a handed part for the `side` hand's build (handle_a -> handle_a_rh / handle_a_lh)."""
    return f"{step}_{S.HANDS[side]}" if step in HANDED else step


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
    m.apply_transform(S.jaw_base(name))                        # right jaw: the same part turned 180° about z
    m.apply_transform(T)
    m.visual.face_colors = _hex(colour)
    return m


def handheld(q, side="right", quest=True):
    """{name: mesh} for the handheld at q. side: which hand it is built for: the grips (palm bar / finger loop swap
    sides) and the Quest controller; the mechanism is the same."""
    out = {}
    for n, (step, col) in HANDHELD.items():
        out[n] = _place(n, handed(step, side), col, _T(S.motion(n, q)))
    out["servo"] = servo_mesh()
    out.update(electronics())
    out["d405_hood"] = _place("d405_hood", "d405_hood", "blue", np.eye(4))
    out["d405"] = d405_mesh()
    if quest:
        out["quest_arm"] = _place("quest_arm", f"quest_arm_{side}", "blue", np.eye(4))
        out["quest_controller"] = controller(side)
    return out


def d405_mesh():
    import head
    m = head.d405_mesh()
    m.visual.face_colors = _hex("camera")
    return m


def _board(size, centre, colour, axis_normal):
    """A board as a box: size (along G x, along the board's other in-plane axis, thickness), placed with its normal
    along G y (axis_normal = 'y') or G x ('x')."""
    sx, sl, t = size
    m = trimesh.creation.box([sx, t, sl] if axis_normal == "y" else [t, sl, sx])
    m.apply_translation(centre)
    m.visual.face_colors = _hex(colour)
    return m


def electronics():
    """Pico 2, USB 3 hub and IMU in the box (stand-ins at their places, for the renders and the checks)."""
    (x0, x1), y, (z0, z1), w = S.EBOX["x"], S.EBOX["y"], S.EBOX["z"], S.EBOX["wall"]
    xm, zm = (x0 + x1) / 2, (z0 + z1) / 2
    yp = y - w - S.PICO["standoff"] - 0.8                       # the Pico's board on its standoffs (+y wall)
    pico = _board((21.0, 51.0, 1.6), [xm, yp, zm], "#1b6e3a", "y")
    chip = _board((7.0, 7.0, 1.0), [xm, yp - 1.3, zm + 4.0], "#202020", "y")
    usb = _board((8.0, 6.0, 3.0), [xm, yp - 2.3, zm + 25.5 - 3.0], "#b9bec5", "y")
    hub = _board((36.0, 42.0, 1.6), [xm, -(y - w - 6.0), zm], "#24364d", "y")
    hub_parts = _board((24.0, 30.0, 8.0), [xm, -(y - w - 6.0) + 4.8, zm], "#3a4a62", "y")
    imu = trimesh.creation.box([1.6, 11.0, 20.0])
    imu.apply_translation([x0 + w + 0.8 + 0.6, 19.0, 30.0])
    imu.visual.face_colors = _hex("#5b3f8c")
    return {"pico": trimesh.util.concatenate([pico, chip, usb]),
            "usb_hub": trimesh.util.concatenate([hub, hub_parts]), "imu": imu}


def servo_mesh():
    m = step_mesh("_sts3215").copy()
    m.apply_transform(S.servo_matrix())
    m.visual.face_colors = _hex("#26282c")
    return m


def controller(side="right"):
    import quest
    m = quest.controller_mesh(side)
    m.visual.face_colors = _hex("quest")
    return m


# ---- the robot gripper (Robotiq meshes, reference only) -------------------------------------------------------------
def _rx(a):
    return trimesh.transformations.rotation_matrix(a, [1, 0, 0])


def _tr(p):
    return trimesh.transformations.translation_matrix(p)


def robotiq(q):
    """The robot's Robotiq gripper posed at q in G (meshes from source/robotiq_2f_<140|85>, in mm)."""
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
    M, F = g["mesh"], (np.diag([-1.0, -1.0, 1.0, 1.0]) if g["mesh_mirror"] else np.eye(4))
    for side, Rs in (("left", np.eye(4)), ("right", trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))):
        j = kin.joints(q)
        To = _tr([0, *g["outer_pivot"]]) @ _rx(a) @ F
        out[f"robotiq_outer_knuckle_{side}"] = load(f"{M}_outer_knuckle").apply_transform(Rs @ To)
        out[f"robotiq_outer_finger_{side}"] = load(f"{M}_outer_finger").apply_transform(
            Rs @ To @ _tr([0, *kin.local(g["outer_to_finger"][0])]))
        out[f"robotiq_inner_knuckle_{side}"] = load(f"{M}_inner_knuckle").apply_transform(
            Rs @ _tr([0, *g["inner_pivot"]]) @ _rx(a) @ F)
        Tf = Rs @ _tr([0, *j["B"]]) @ _rx(af) @ F
        out[f"robotiq_inner_finger_{side}"] = load(f"{M}_inner_finger").apply_transform(Tf)
        (c, size) = g["pad_local"]
        pad = trimesh.creation.box(size)
        out[f"robotiq_pad_{side}"] = pad.apply_transform(Tf @ _tr(c))
    for n, m in out.items():
        m.visual.face_colors = _hex("robotiq_metal" if "knuckle" in n or "outer_finger" in n else "robotiq")
    return out


def robot(q, side="right", quest=False):
    """The real gripper with the team's UR camera plate and the D405 straight on its tab (no Quest on the robot)."""
    out = robotiq(q)
    m = step_mesh("_ur_camera_plate").copy()
    m.visual.face_colors = _hex("#8d96a3")
    out["ur_camera_plate"] = m
    out["d405"] = d405_mesh()
    return out


def export_refs():
    """Reference parts as STEP in build/ so the assemblies can mesh them: the camera plate (placed in G) and the
    STS3215 (its own frame)."""
    import head
    from build123d import export_step, import_step
    S.STEP_DIR.mkdir(parents=True, exist_ok=True)
    export_step(head.robot_plate(), str(S.STEP_DIR / "_ur_camera_plate.step"))
    export_step(import_step(str(S.SERVO_STEP)), str(S.STEP_DIR / "_sts3215.step"))


# ---- checks --------------------------------------------------------------------------------------------------------
def _overlap(a, b):
    try:
        i = trimesh.boolean.intersection([a, b], engine="manifold")
        return float(i.volume) if i is not None and not i.is_empty else 0.0
    except Exception as e:                                         # noqa: BLE001
        return float("nan")


MOVING_PAIRS_SKIP = set()


# the pads meet at q_max; the soft grip inserts are bonded into their handles (coincident curved faces)
CONTACT_OK = {frozenset(p) for p in (("pad_skin_left", "pad_skin_right"), ("handle_a", "grip_tpu_a"),
                                     ("handle_b", "grip_tpu_b"))}


def check(steps=8):
    """Interference between every pair of parts of the handheld (Quest included) over the stroke, and of the robot
    build's camera and plate with the gripper."""
    bad = []
    for side in ("right", "left"):
        for q in np.linspace(0, kin.g()["q_max"], steps):
            a = handheld(q, side)
            keys = list(a)
            for i in range(len(keys)):
                for k in keys[i + 1:]:
                    v = _overlap(a[keys[i]], a[k])
                    if v > 1.0:
                        bad.append((side, round(float(q), 3), keys[i], k, round(v, 1)))
    for q in (0.0, kin.g()["q_max"]):
        r = robot(q)
        for e in ("d405", "ur_camera_plate"):
            for k, m in r.items():
                if k != e and not (e == "ur_camera_plate" and k == "d405"):
                    v = _overlap(r[e], m)
                    if v > 1.0:
                        bad.append(("robot", q, "robot:" + e, k, round(v, 1)))
    real = [b for b in bad if frozenset(b[2:4]) not in CONTACT_OK]
    for b in bad:
        print("overlap", b, "(expected contact)" if b not in real else "")
    print(f"{len(real)} unexpected overlapping pairs over {steps} poses, right- and left-hand builds")
    gaps = {q: kin.pad_gap(q) for q in (0.0, kin.g()["q_max"])}
    print("pad gap open / closed:", {k: round(float(v), 1) for k, v in gaps.items()})
    return real


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--refs", action="store_true")
    a = ap.parse_args()
    if a.refs or not (S.STEP_DIR / "_sts3215.step").exists():
        export_refs()
    if a.check:
        check()
