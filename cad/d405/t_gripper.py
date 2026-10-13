"""PG Parallel: the HandUMI T-plate gripper, unchanged, with this kit's camera on its post.

  * Insta360 X5 kit: the X5 stands on the post in `x5_wrist_mount`, the only part that differs from HandUMI.
  * D405 kit: HandUMI exactly, with its D405 cup (`d405_wrist_mount`) on the same hinge.

Every part is HandUMI's (Pruthvi-Neuracore/The-UMI-no-one-wants, vendored in handumi/: main support, end cover, finger
links with the sleeves, record button and universal tip flange, crank and connecting links, rods, servo encoder,
electronics box, hand support base, controller support, and the tip sets). The one new part is `x5_wrist_mount`
(handumi/cad/x5_wrist_mount.py), which takes the D405 cup's place on the same Ø12 / M4 hinge.

    python t_gripper.py [--check]

Writes, under build/insta360_x5/t_gripper/:
  hardware/STEP|STL/{right,left}/   every printed part (HandUMI's + x5_wrist_mount), STL exported from the STEP
  hardware/STEP|STL/gripper_tips/   the tip sets (AgileX Piper, Open-ENPIRE, Robotiq Hand-E)
  assembly/x5_parallel_right_<tips>.glb/.stl   the right hand, half open, once per tip set
and the renders to build/insta360_x5/docs/img/ (parallel*.png, tips_*.png, camera_mount.png).
"""
import argparse
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
sys.path.append(str(HERE / "handumi" / "cad"))         # after the kit's own modules: HandUMI has an assembly.py too


def _load(name, alias):
    import importlib.util
    spec = importlib.util.spec_from_file_location(alias, HERE / "handumi" / "cad" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[alias] = mod
    path = list(sys.path)
    spec.loader.exec_module(mod)
    sys.path[:] = path                                  # HandUMI's modules put their folder first; keep ours first
    return mod


A = _load("assembly", "handumi_assembly")
KIT = HERE.name
X5 = KIT == "insta360_x5"
if X5:
    import x5_wrist_mount as XM  # noqa: E402
MOUNT = "x5_wrist_mount" if X5 else "d405_wrist_mount"
from occ import place  # noqa: E402
from tips import TIP_SETS  # noqa: E402

from parts import ROOT as KIT_BUILD, load_raw  # noqa: E402

OUT = KIT_BUILD / "t_gripper"
SRC_HW = HERE / "handumi" / "hardware"
X5_COLOUR, MOUNT_COLOUR = "#1b1d21", A.BLUE
UP_Z = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])      # HandUMI M (-Z up) -> +Z up
UP_Y = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])  # -Z up -> +Y up (glTF)
SLUG = {"AgileX Piper": "agilex-piper", "Robotiq Hand-E": "robotiq-hand-e", "Open-ENPIRE (UCG)": "open-enpire"}


def x5_upright():
    """The X5 body from the KIWI envelope, upright: bottom centre at the origin, lens axis along x, +z up."""
    m = A_to_mesh(load_raw()["insta360_x5_envelope"])
    T, ext = trimesh.bounds.oriented_bounds(m)
    m.apply_transform(T)                                             # centred, axes = (thickness, width, height)
    if np.argmax(m.extents) != 2:
        raise RuntimeError(f"unexpected X5 envelope axes {m.extents}")
    m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0]))   # its top was at -z
    m.apply_translation([0, 0, -m.bounds[0][2]])
    return m


def A_to_mesh(shape, tol=0.05):
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def paint(m, colour):
    rgba = np.array([int(colour[i:i + 2], 16) for i in (1, 3, 5)] + [255], np.uint8)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
    return m


def build(opening=0.5, tipset=None, tilt=None):
    """{name: (trimesh in M, mm, colour)} for the right hand with the kit's camera on the post."""
    ts = tipset or TIP_SETS[0]
    parts = A.assemble(opening, camera="none" if X5 else "wrist", tipset=(ts, A.best_orientation(ts)))
    out = {n: (A_to_mesh(sh), col) for n, (sh, col) in parts.items()}
    if not X5:
        return out
    T = XM.hinge_to_m(XM.TILT_DEG if tilt is None else tilt)
    out["x5_wrist_mount"] = (A_to_mesh(XM.build()).apply_transform(T), MOUNT_COLOUR)
    x5 = x5_upright()
    x5.apply_translation([0, 0, XM.PLATE_Z + XM.PLATE_T])
    out["insta360_x5"] = (x5.apply_transform(T), X5_COLOUR)
    return out


def check():
    """Clash check (exact solids): the X5 mount and camera against every HandUMI part, closed / half / fully open,
    with every tip set, and the hinge's tilt range. D405 kit: HandUMI's own check."""
    from build123d import Box, Pos
    from occ import overlap
    if not X5:
        for ts in TIP_SETS:
            for op in (0.0, 0.5, 1.0):
                hits = A.interferences(A.assemble(op, tipset=(ts, A.best_orientation(ts))))
                print(f"  {ts.name}, opening {op}: {hits or 'no interferences'}")
        return
    mount = XM.build()
    body = Pos(0, 0, XM.PLATE_Z + XM.PLATE_T + XM.X5[2] / 2) * Box(*XM.X5)
    worst = 0.0
    for ts in TIP_SETS:
        for op in (0.0, 0.5, 1.0):
            parts = A.assemble(op, camera="none", tipset=(ts, A.best_orientation(ts)))
            T = XM.hinge_to_m()
            for nm, s in (("x5_wrist_mount", place(mount, T)), ("insta360_x5", place(body, T))):
                for n, (sh, _) in parts.items():
                    v = overlap(s, sh)
                    worst = max(worst, v)
                    if v > 0.01:
                        print(f"  CLASH {nm} x {n}: {v:.2f} mm3 ({ts.name}, opening {op})")
    # tilt range: lean forward / back until the cradle or camera touches the frame
    parts = A.assemble(0.0, camera="none")
    fixed = [sh for n, (sh, _) in parts.items() if n in ("main_support", "electronics_box", "electronics_lid",
                                                          "controller_support", "servo_sts3215")]
    lim = {}
    for sgn in (1, -1):
        a = 0.0
        while abs(a) < 90:
            T = XM.hinge_to_m(a + sgn * 1.0)
            if any(overlap(place(s, T), f) > 0.01 for s in (mount, body) for f in fixed):
                break
            a += sgn * 1.0
        lim["forward" if sgn > 0 else "back"] = abs(a)
    print(f"x5 mount: worst overlap {worst:.2f} mm3; tilt range {lim['back']:.0f}° back to {lim['forward']:.0f}° forward")
    return worst, lim


def export_parts():
    """Copy HandUMI's STEP parts and tips, add x5_wrist_mount, and export STL for every STEP part."""
    hw = OUT / "hardware"
    if hw.exists():
        shutil.rmtree(hw)
    for sub in ("STEP", "STL"):
        src = SRC_HW / sub
        if src.exists():
            shutil.copytree(src, hw / sub, dirs_exist_ok=True)
    if X5:
        XM.main(hw)
    from build123d import import_step
    n = 0
    for side in ("right", "left"):
        (hw / "STL" / side).mkdir(parents=True, exist_ok=True)
        for st in sorted((hw / "STEP" / side).glob("*.step")):
            stl = hw / "STL" / side / (st.stem + ".stl")
            if stl.exists():
                continue
            A_to_mesh(import_step(str(st)), 0.03).export(stl)
            n += 1
    print(f"parts: {n} STL exported; tips from HandUMI")


def export_assemblies(opening=0.5):
    d = OUT / "assembly"
    d.mkdir(parents=True, exist_ok=True)
    for ts in TIP_SETS:
        parts = build(opening, ts)
        stem = f"{'x5' if X5 else 'd405'}_parallel_right_{SLUG[ts.name]}"
        meshes = {n: paint(m.copy(), c) for n, (m, c) in parts.items()}
        merged = trimesh.util.concatenate(list(meshes.values()))
        merged.apply_transform(UP_Z)
        merged.export(d / f"{stem}.stl")
        scene = trimesh.Scene()
        S = np.diag([1e-3, 1e-3, 1e-3, 1.0]) @ UP_Y
        for n, m in meshes.items():
            scene.add_geometry(m, node_name=n, geom_name=n, transform=S)
        scene.export(d / f"{stem}.glb")
        print(f"  {stem}: {len(meshes)} parts")


# --- renders ---------------------------------------------------------------------------------------------------------
def _scene(p, parts, T=np.eye(4)):
    from render_docs import add
    for n, (m, c) in parts.items():
        add(p, m.copy().apply_transform(T @ UP_Z), c)


HERO_DIR = np.array([-190.0, 330.0, 270.0])    # the chopstick hero's view (from front-right, above), turned for tips
                                               # along -X: every parallel render is seen the same way


def _view(focal, dist):
    f = np.array(focal, float)
    return tuple(f + HERO_DIR / np.linalg.norm(HERO_DIR) * dist), tuple(f)


def render_all():
    from render_docs import IMG, plotter, shot
    IMG.mkdir(parents=True, exist_ok=True)
    # hero: right hand, Piper tips, half open (Z up after UP_Z: fingers towards -X, closing along Y)
    p = plotter((1700, 1150))
    _scene(p, build(0.45, TIP_SETS[0]))
    shot(p, IMG / "parallel.png", *_view((-40, -72, 30), 620), zoom=1.1)
    # the other side
    p = plotter((1700, 1150))
    _scene(p, build(0.45, TIP_SETS[0]))
    shot(p, IMG / "parallel_back.png", (420, -330, 260), (0, -70, 40), zoom=1.1)          # the hero's far side
    # opening: closed, half, full
    p = plotter((2100, 900))
    for i, op in enumerate((0.0, 0.5, 1.0)):                       # side by side across the view, seen from the front
        _scene(p, build(op, TIP_SETS[0]), trimesh.transformations.translation_matrix([0, -i * 300.0, 0]))
    shot(p, IMG / "parallel_opening.png", (-1400, -370, 520), (-40, -370, 20), zoom=1.6, aspect=2.4)
    # one image per tip set, close on the jaws
    for ts in TIP_SETS:
        p = plotter((1500, 1100))
        _scene(p, build(0.35, ts))
        shot(p, IMG / f"tips_{SLUG[ts.name]}.png", *_view((-70, -72, 10), 560), zoom=1.05)
    # the X5 mount on the post, close up
    parts = build(0.4, TIP_SETS[0])
    keep = {k: v for k, v in parts.items() if k in ("main_support", "x5_wrist_mount", "insta360_x5", "camera_mount", "d405",
                                                    "electronics_box", "electronics_lid", "end_cover")}
    p = plotter((1500, 1100))
    _scene(p, keep)
    shot(p, IMG / "camera_mount.png", *_view((-30, -72, 90), 360), zoom=1.3)


def parts_sheet():
    """Every printed part of the right hand laid out flat, the X5 mount first."""
    from build123d import import_step
    from render_docs import IMG, add, plotter, shot
    hw = OUT / "hardware" / "STEP" / "right"
    names = [MOUNT, "main_support", "end_cover", "right_thumb_link", "right_index_middle_finger_link",
             "button_cover_tpu", "crank_mechanism_plate", "connecting_link_1", "connecting_link_2",
             "hand_support_base", "right_controller_support", "electronics_box", "electronics_lid"]
    p = plotter((2000, 1100))
    x = y = 0.0
    row_h = 0.0
    for n in names:
        m = A_to_mesh(import_step(str(hw / f"{n}.step")), 0.08)
        m.apply_translation(-m.bounds[0])
        if x + m.extents[0] > 520:
            x, y, row_h = 0.0, y - row_h - 30, 0.0
        m.apply_translation([x, y - m.extents[1], 0])
        col = MOUNT_COLOUR if n == MOUNT else (
            "#2e9e44" if n == "button_cover_tpu" else (A.WHITE if "link" in n and "connecting" not in n
                                                      or n in ("crank_mechanism_plate", "hand_support_base",
                                                               "electronics_box") else A.BLUE))
        add(p, m, col)
        x += m.extents[0] + 30
        row_h = max(row_h, m.extents[1])
    shot(p, IMG / "parallel_parts.png", (260, -900, 1300), (260, -150, 0), zoom=1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--render-only", action="store_true")
    a = ap.parse_args()
    if a.check:
        check()
    if not a.render_only:
        export_parts()
        export_assemblies()
    if not a.no_render:
        render_all()
        parts_sheet()


if __name__ == "__main__":
    main()
