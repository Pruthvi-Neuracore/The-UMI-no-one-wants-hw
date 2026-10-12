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


def plotter(size=(1600, 1100), scale=1.0):
    """scale: scene units per mm (1 for grippers in mm, 1e-3 for robots in metres), for the ambient occlusion."""
    p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
    p.set_background(BG)
    p.enable_anti_aliasing("ssaa")
    p.enable_ssao(radius=6.0 * scale, bias=0.3 * scale, kernel_size=128, blur=True)
    return p


def tidy(path, aspect=1.6, margin=0.07):
    """Crop the image to what was drawn, then pad it back to `aspect` with an even margin, so every render is
    centred and framed the same way."""
    from PIL import Image, ImageChops
    im = Image.open(path).convert("RGB")
    bg = Image.new("RGB", im.size, (255, 255, 255))
    box = ImageChops.difference(im, bg).convert("L").point(lambda v: 255 if v > 8 else 0).getbbox()
    if not box:
        return
    im = im.crop(box)
    w, h = im.size
    W = max(w * (1 + 2 * margin), h * (1 + 2 * margin) * aspect)
    H = W / aspect
    out = Image.new("RGB", (int(W), int(H)), (255, 255, 255))
    out.paste(im, ((int(W) - w) // 2, (int(H) - h) // 2))
    out.save(path)


def shot(p, path, pos, focal, up=(0, 0, 1), zoom=1.0, aspect=1.6):
    """Look from pos towards focal, fit everything in view, zoom in by `zoom`, save, then crop and centre it."""
    p.camera_position = [pos, focal, up]
    p.reset_camera()
    p.camera.zoom(zoom)
    p.screenshot(str(path))
    p.close()
    tidy(path, aspect)
    print("wrote", path.relative_to(ROOT) if ROOT in path.parents else path)


def gripper(build, frac, parts):
    """A build opened `frac` of a typical working opening (0 closed, 1 = 45 deg per chopstick jaw / full stroke)."""
    return assemble(build, frac, parts)


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
    place(p, gripper("chopstick_right", 0.4, parts))
    shot(p, IMG / "hero.png", (300, -330, 260), (110, 0, -10), zoom=1.3)


def opening(parts, build="chopstick_right", path="opening.png"):
    p = plotter((2000, 760))
    labels = ("0°", "15°", "30°", "45°") if build.startswith("chopstick") else ("0 mm", "20 mm", "40 mm", "60 mm")
    for i, (frac, lab) in enumerate(zip((0, 1 / 3, 2 / 3, 1), labels)):
        place(p, gripper(build, frac, parts), trimesh.transformations.translation_matrix([0, -i * 330, 0]))
        p.add_point_labels([[245, -i * 330, 0]], [lab], font_size=36, shape=None, text_color="#1f5fae",
                           show_points=False, always_visible=True)
    shot(p, IMG / path, (110, -495, 1300), (110, -495, 0), up=(1, 0, 0), zoom=2.1)


def pod(parts):
    """The PG pod from below and behind: USB-C, switch, button, lid."""
    p = plotter((1400, 1000))
    place(p, gripper("chopstick_right", 0.2, parts))
    shot(p, IMG / "pod.png", (-260, -380, -260), (40, 0, -30), zoom=1.25)


def sleeve_closeup(parts):
    """Right handheld jaw, looking down the finger sleeve from behind (the way the finger goes in): the strap fins and
    the elastic strap, and the button cover on the pad, the wall the finger pushes."""
    import sleeve
    from parts import load_raw
    pl = sleeve.frame(load_raw()["jaw_right"], "right")
    o, d, n = (np.array(tuple(v)) for v in (pl.origin, pl.x_dir, pl.z_dir))
    axis = o + n * (sleeve.PAD + sleeve.HALF_W)
    p = plotter((1400, 1000))
    for name in ("jaw_right_dc_button", "tip_tpu_right", "sleeve_button_cover_right", "sleeve_strap_right"):
        add(p, to_mesh(parts[name], 0.03), colour_of(name))
    shot(p, IMG / "sleeve.png", tuple(axis - 90 * d + 32 * n - np.array([0, 0, 8.0])), tuple(axis + 6 * d - 5 * n),
         zoom=1.5)


def hand(parts):
    """The X5 on the wrist saddle, on a forearm (a simple stand-in), seen from the side and front."""
    import hand as H
    import pod as PD
    p = plotter((1600, 1100))
    place(p, assemble("hand", 0.0, parts))
    xc = (PD.X0 + PD.X1) / 2
    a = np.radians(H.PITCH)
    arm = trimesh.creation.cylinder(radius=H.R_ARM - 1.0, height=260, sections=64)
    arm.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2 - a, [0, 1, 0]))
    z_top = PD.Z_BOT - PD.LID - H.MIN_T
    x1 = xc + H.LENGTH / 2
    arm.apply_translation([x1 - 40 * np.cos(a), 0, z_top - H.R_ARM - 40 * np.sin(a) + 1.0])
    add(p, arm, "#e8c9b0")
    shot(p, IMG / "hand.png", (320, -380, 120), (20, 0, -60), zoom=1.3)


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
    rows = [["chopstick_base_dc", "chopstick_clip_dc", "x5_lite_frame", "jaw_left_dc", "jaw_right_dc_button",
             "tip_tpu_left", "tip_tpu_right", "sleeve_button_cover_right"],
            ["chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b", "jaw_left_sa", "jaw_right"],
            ["parallel_base_dc", "parallel_carriage_left", "parallel_carriage_right", "parallel_pinion",
             "parallel_finger_left_sleeve", "parallel_finger_right_button"],
            ["pg_pod", "pg_pod_lid", "pg_wrist_saddle", "openarm_adapter", "iso_flange_plate"]]
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
    tmp = ROOT / "assemblies" / "_openarm_chopstick.urdf"
    mesh_dir = (URDF_DIR / "meshes").resolve()
    splice_openarm(Path(openarm) / "openarm.urdf", tmp, (URDF_DIR / "pg_chopstick_openarm.urdf").read_text(),
                   str(mesh_dir))
    r = yourdfpy.URDF.load(str(tmp), mesh_dir=str(openarm), build_scene_graph=True, load_meshes=True)
    tmp.unlink()
    r.update_cfg({k: v for k, v in BIMANUAL_CFG.items() if k in r.actuated_joint_names})
    p = plotter((1600, 1200), scale=1e-3)
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
    for m in gripper("chopstick_iso50", 0.55, parts).values():
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
    ap.add_argument("--fanuc", help="optional: single FANUC CRX URDF with tool0 (crx5ia/arm.urdf) for a close-up")
    a = ap.parse_args()
    IMG.mkdir(parents=True, exist_ok=True)
    for old in IMG.glob("*.png"):
        if not old.name.startswith("robot_"):            # robot_*.png come from robots.py
            old.unlink()
    parts = load_parts()
    hero(parts)
    side_by_side(parts, ["chopstick_left", "chopstick_right"], IMG / "pair.png", (0.4, 0.4), gap=320.0)
    side_by_side(parts, ["chopstick_right", "chopstick_openarm"], IMG / "kit.png", (0.4, 0.55))
    side_by_side(parts, ["parallel_right", "parallel_iso50"], IMG / "parallel.png", (0.5, 0.5), gap=360.0)
    opening(parts, "parallel_right", "parallel_opening.png")
    hand(parts)
    opening(parts)
    pod(parts)
    sleeve_closeup(parts)
    plates(parts)
    exploded(parts)
    if a.openarm:
        bimanual(a.openarm)
    if a.fanuc:
        fanuc(a.fanuc, parts)
    # the dual-arm UR / ABB / FANUC renders: robots.py


if __name__ == "__main__":
    main()
