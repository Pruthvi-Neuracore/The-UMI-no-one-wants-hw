"""Render the README images to chopstick/docs/img/.

    python render_docs.py [--openarm /path/to/openarm_description] [--fanuc /path/to/crx5ia/arm.urdf]

Without --openarm / --fanuc the robot images are skipped (the arm meshes are not redistributed here).
"""
import argparse
import math
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

from assembly import COLOURS, assemble, colour_of
from iso_flange_plate import T_FLANGE
from kinematics import to_mesh
from parts import ROOT, load_parts

IMG = ROOT / "docs" / "img"
BG = "white"
ARM = "#d9dde2"


def add(p, m, colour=None, **kw):
    pm = pv.wrap(m)
    if colour is None:
        colour = m.visual.face_colors[0, :3] / 255.0
    p.add_mesh(pm, color=colour, smooth_shading=True, split_sharp_edges=True, specular=0.3, specular_power=25, **kw)


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


def place(p, meshes, T=np.eye(4)):
    for m in meshes.values():
        add(p, m.copy().apply_transform(T))


def side_by_side(parts, builds, path, degs, view=(320.0, -490.0, 330.0), gap=300.0, size=(2000, 1000), zoom=2.2):
    p = plotter(size)
    view = np.array(view)
    side = np.array([-view[1], view[0], 0.0]) / np.hypot(view[0], view[1])   # across the view, to the right
    for i, (b, d) in enumerate(zip(builds, degs)):
        place(p, gripper(b, d, parts), trimesh.transformations.translation_matrix(side * gap * i))
    c = np.array([100.0, 0, 0]) + side * gap * (len(builds) - 1) / 2
    shot(p, path, tuple(c + view), tuple(c), zoom=zoom)


def hero(parts):
    p = plotter()
    place(p, gripper("data", 18, parts))
    shot(p, IMG / "hero.png", (300, -330, 260), (110, 0, -10), zoom=1.3)


def opening(parts):
    p = plotter((2000, 760))
    for i, deg in enumerate((0, 15, 30, 45)):
        place(p, gripper("data", deg, parts), trimesh.transformations.translation_matrix([0, -i * 330, 0]))
        p.add_point_labels([[235, -i * 330, 0]], [f"{deg}°"], font_size=36, shape=None, text_color="#1f5fae",
                           show_points=False, always_visible=True)
    shot(p, IMG / "opening.png", (110, -495, 1300), (110, -495, 0), up=(1, 0, 0), zoom=2.1)


def pod(parts):
    """The PG pod from below and behind: USB-C, switch, button, lid."""
    p = plotter((1400, 1000))
    place(p, gripper("data", 10, parts))
    shot(p, IMG / "pod.png", (-260, -380, -260), (40, 0, -30), zoom=1.25)


def plates(parts):
    """OpenArm adapter and ISO 50 plate, front and back."""
    p = plotter((2000, 900))
    for i, n in enumerate(("openarm_adapter", "iso_flange_plate")):
        m = to_mesh(parts[n], 0.03)
        for j, rot in enumerate((0, 180)):
            T = trimesh.transformations.translation_matrix([0, -i * 160 - j * 75, 0]) @ \
                trimesh.transformations.rotation_matrix(math.radians(rot), [0, 0, 1])
            add(p, m.copy().apply_transform(T), COLOURS["blue"])
    shot(p, IMG / "plates.png", (260, -440, 160), (0, -117, 0), zoom=1.6)


def exploded(parts):
    """Printed parts: handheld row, robot row, new PG parts row."""
    rows = [["chopstick_base_dc", "chopstick_clip_dc", "x5_lite_frame", "jaw_left_dc", "jaw_right", "tip_tpu_left",
             "tip_tpu_right"],
            ["chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b", "jaw_left_sa"],
            ["pg_pod", "pg_pod_lid", "pg_pod_button", "d405_frame", "openarm_adapter", "iso_flange_plate"]]
    p = plotter((2000, 1200))
    for r, group in enumerate(rows):
        x = 0.0
        for n in group:
            m = to_mesh(parts[n], 0.05)
            m.apply_translation(-m.bounds[0] + [x, -r * 150.0, 0])
            add(p, m, colour_of(n))
            x += m.extents[0] + 25
    shot(p, IMG / "parts.png", (450, -800, 900), (450, -150, 0), zoom=1.5)


BIMANUAL_CFG = {"left_joint4": 1.6, "right_joint4": 1.6,
                "openarm_left_chopstick_jaw": 0.3, "openarm_right_chopstick_jaw": 0.45}


def urdf_scene(r, p, ours):
    """Add every visual of a loaded yourdfpy URDF; `ours(name)` says which visuals are PG Chopstick parts."""
    g = r.scene.graph
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for nd in g.nodes_geometry:
        T, name = g[nd]
        m = r.scene.geometry[name].copy().apply_transform(T)
        part = name.split("_jawframe")[0]
        add(p, m, colour_of(part) if ours(name) else ARM)
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
    return lo, hi


def bimanual(openarm, path=None, direction=(1.0, -0.75, 0.45)):
    """Both OpenArm arms with the PG Chopstick robot gripper, from the spliced URDF."""
    import yourdfpy
    from urdf import URDF as URDF_DIR, splice_openarm
    tmp = ROOT / "assembly" / "_openarm_chopstick.urdf"
    mesh_dir = (URDF_DIR / "meshes").resolve()
    splice_openarm(Path(openarm) / "openarm.urdf", tmp, (URDF_DIR / "pg_chopstick_openarm.urdf").read_text(),
                   str(mesh_dir))
    r = yourdfpy.URDF.load(str(tmp), mesh_dir=str(openarm), build_scene_graph=True, load_meshes=True)
    tmp.unlink()
    r.update_cfg({k: v for k, v in BIMANUAL_CFG.items() if k in r.actuated_joint_names})
    p = plotter((1600, 1200))
    lo, hi = urdf_scene(r, p, lambda n: not n.startswith("openarm_") or "adapter" in n)
    c = (lo + hi) / 2
    d = np.asarray(direction) / np.linalg.norm(direction)
    shot(p, path or IMG / "openarm_bimanual.png", tuple(c + d * 3.5 * np.linalg.norm(hi - lo)), tuple(c), zoom=1.35)


def fanuc(urdf_path, parts):
    """FANUC CRX-5iA with the ISO 50 build on tool0."""
    import yourdfpy
    r = yourdfpy.URDF.load(str(urdf_path), build_scene_graph=True, load_meshes=True)
    cfg = {"J2": 0.35, "J3": -0.35, "J5": -1.2}
    r.update_cfg({k: v for k, v in cfg.items() if k in r.actuated_joint_names})
    p = plotter((1600, 1200))
    lo, hi = urdf_scene(r, p, lambda n: False)
    T = r.get_transform("tool0", r.base_link) @ T_FLANGE @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
    for m in gripper("iso50", 25, parts).values():
        mm = m.copy().apply_transform(T)
        add(p, mm)
        lo, hi = np.minimum(lo, mm.bounds[0]), np.maximum(hi, mm.bounds[1])
    c = (lo + hi) / 2
    d = np.array([1.0, -1.1, 0.55])
    shot(p, IMG / "fanuc_crx_iso50.png", tuple(c + d / np.linalg.norm(d) * 3.5 * np.linalg.norm(hi - lo)), tuple(c),
         zoom=1.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="path to openarm_description")
    ap.add_argument("--fanuc", help="path to a FANUC CRX URDF with a tool0 frame (e.g. crx5ia/arm.urdf)")
    a = ap.parse_args()
    IMG.mkdir(parents=True, exist_ok=True)
    for old in IMG.glob("*.png"):
        old.unlink()
    parts = load_parts()
    hero(parts)
    side_by_side(parts, ["data", "openarm"], IMG / "kit.png", (18, 25))
    side_by_side(parts, ["data_d405", "iso50_d405"], IMG / "d405.png", (18, 25))
    opening(parts)
    pod(parts)
    plates(parts)
    exploded(parts)
    if a.openarm:
        bimanual(a.openarm)
    if a.fanuc:
        fanuc(a.fanuc, parts)


if __name__ == "__main__":
    main()
