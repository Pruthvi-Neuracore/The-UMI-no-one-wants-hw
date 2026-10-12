"""Render the README images to chopstick/docs/img/.

    python render_docs.py [--openarm /path/to/openarm_description]

Without --openarm the OpenArm images are skipped (the arm meshes are not redistributed here).
"""
import argparse
import math
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

import openarm_adapter
from assembly import assemble, link7_in_gripper
from kinematics import to_mesh
from kiwi import ROOT, load_parts

IMG = ROOT / "docs" / "img"
BG = "white"


def add(p, m, colour=None, **kw):
    pm = pv.wrap(m)
    if colour is None:
        colour = m.visual.face_colors[0, :3] / 255.0
    p.add_mesh(pm, color=colour, smooth_shading=True, split_sharp_edges=True, specular=0.25, specular_power=20, **kw)


def plotter(size=(1600, 1100)):
    p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
    p.set_background(BG)
    p.enable_anti_aliasing("ssaa")
    return p


def shot(p, path, pos, focal, up=(0, 0, 1), zoom=1.0):
    """Look from pos towards focal, fit everything in view, then zoom in by `zoom`."""
    p.camera_position = [pos, focal, up]
    p.reset_camera()
    p.camera.zoom(zoom)
    p.screenshot(str(path))
    p.close()
    print("wrote", path.relative_to(ROOT) if ROOT in path.parents else path)


def gripper(build, deg, parts):
    return assemble(build, math.radians(deg), parts)


def hero(parts):
    p = plotter()
    for m in gripper("data", 18, parts).values():
        add(p, m)
    shot(p, IMG / "hero.png", (300, -330, 260), (110, 0, 10), zoom=1.3)


def kit(parts):
    """The handheld data gripper and the OpenArm robot gripper side by side."""
    p = plotter((2000, 1000))
    for m in gripper("data", 18, parts).values():
        add(p, m)
    view = np.array([320.0, -490.0, 330.0])
    side = np.array([-view[1], view[0], 0.0]) / np.hypot(view[0], view[1])   # across the view, to the right
    off = trimesh.transformations.translation_matrix(side * 300)
    for m in gripper("openarm", 25, parts).values():
        add(p, m.copy().apply_transform(off))
    c = np.array([100.0, 0, 0]) + side * 150
    shot(p, IMG / "kit.png", tuple(c + view), tuple(c), zoom=1.5)


def opening(parts):
    p = plotter((2000, 700))
    for i, deg in enumerate((0, 15, 30, 45)):
        off = trimesh.transformations.translation_matrix([0, -i * 330, 0])
        for m in gripper("data", deg, parts).values():
            add(p, m.copy().apply_transform(off))
        p.add_point_labels([[235, -i * 330, 0]], [f"{deg}°"], font_size=36, shape=None, text_color="#333333",
                           show_points=False, always_visible=True)
    shot(p, IMG / "opening.png", (110, -495, 1300), (110, -495, 0), up=(1, 0, 0), zoom=2.1)


def robot_gripper(parts, openarm):
    p = plotter()
    for m in gripper("openarm", 25, parts).values():
        add(p, m)
    if openarm:
        add(p, link7_in_gripper(openarm), "#c8ccd2")
    shot(p, IMG / "robot_gripper_openarm.png", (260, -360, 240), (70, 0, 0), zoom=1.35)


def adapter(openarm):
    p = plotter((1400, 1000))
    ad = to_mesh(openarm_adapter.build(), 0.03)
    add(p, ad.copy().apply_translation([30, 0, 0]), "#5b6068")
    if openarm:
        l7 = link7_in_gripper(openarm).apply_translation([-25, 0, 0])
        add(p, l7.slice_plane([-75, 0, 0], [1, 0, 0], cap=True), "#c8ccd2")      # just the wrist end of link7
    shot(p, IMG / "openarm_adapter.png", (-160, -190, 120), (0, 0, 0), zoom=1.2)
    p = plotter((1400, 1000))
    add(p, ad, "#5b6068")
    shot(p, IMG / "openarm_adapter_front.png", (160, -150, 110), (-7, 0, 0), zoom=1.2)


def exploded(parts):
    """The printed parts of both builds laid out flat (no camera or servo)."""
    p = plotter((2000, 1000))
    names = ["chopstick_base_dc", "chopstick_clip_dc", "x5_lite_frame", "jaw_left_dc", "jaw_right", "tip_tpu_left",
             "tip_tpu_right", "chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b", "jaw_left_sa"]
    x = 0.0
    for row, group in enumerate((names[:7], names[7:] + ["openarm_adapter"])):
        x = 0.0
        for n in group:
            m = to_mesh(openarm_adapter.build() if n == "openarm_adapter" else parts[n], 0.05)
            m.apply_translation(-m.bounds[0] + [x, -row * 150.0, 0])
            add(p, m, "#9aa0a8" if n.startswith("tip") else "#3c4046")
            x += m.extents[0] + 25
    shot(p, IMG / "parts.png", (450, -700, 900), (450, -75, 0), zoom=1.6)


BIMANUAL_CFG = {"left_joint4": 1.6, "right_joint4": 1.6,
                "openarm_left_chopstick_jaw": 0.3, "openarm_right_chopstick_jaw": 0.45}


def bimanual(openarm, cfg=BIMANUAL_CFG, path=None, direction=(1.0, -0.75, 0.45), size=(1600, 1200)):
    """Both OpenArm arms with the chopstick robot gripper, from the spliced URDF."""
    import yourdfpy
    from urdf import URDF as URDF_DIR, splice_openarm
    tmp = ROOT / "assembly" / "_openarm_chopstick.urdf"
    mesh_dir = (URDF_DIR / "meshes").resolve()
    splice_openarm(Path(openarm) / "openarm.urdf", tmp, (URDF_DIR / "chopstick_openarm.urdf").read_text(), str(mesh_dir))
    r = yourdfpy.URDF.load(str(tmp), mesh_dir=str(openarm), build_scene_graph=True, load_meshes=True)
    tmp.unlink()
    r.update_cfg({k: v for k, v in cfg.items() if k in r.actuated_joint_names})
    p = plotter(size)
    g = r.scene.graph
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for nd in g.nodes_geometry:
        T, name = g[nd]
        m = r.scene.geometry[name].copy().apply_transform(T)
        ours = "chopstick" in name or "adapter" in name or not name.startswith("openarm_")  # visuals named by part
        col = ("#9aa0a8" if "tip_tpu" in name else "#1d1f22" if "envelope" in name else "#3c4046") if ours else "#d5d8dc"
        add(p, m, col)
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
    c = (lo + hi) / 2
    d = np.asarray(direction) / np.linalg.norm(direction)
    shot(p, path or IMG / "openarm_bimanual.png", tuple(c + d * 3.5 * np.linalg.norm(hi - lo)), tuple(c), zoom=1.35)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="path to openarm_description")
    a = ap.parse_args()
    IMG.mkdir(parents=True, exist_ok=True)
    parts = load_parts()
    hero(parts)
    kit(parts)
    opening(parts)
    exploded(parts)
    robot_gripper(parts, a.openarm)
    adapter(a.openarm)
    if a.openarm:
        bimanual(a.openarm)


if __name__ == "__main__":
    main()
