"""Render the robots the tip sets reproduce (for the README). Robot descriptions are not vendored here:

    python robots.py /path/to/Neuracore_Robots        # ABB GoFa CRB 15000, FANUC CRX-5iA, UR5 duals with Robotiq Hand-E
    python robots.py --sw /path/to/handumi-sw/assets  # AgileX Piper; OpenArm with Open-ENPIRE fingers
    python robots.py --yam /path/to/i2rt              # I2RT YAM with the Open-ENPIRE fingers (i2rt-robotics/i2rt, MIT)
"""
import re
import sys
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh
import yourdfpy

pv.OFF_SCREEN = True
IMG = Path(__file__).resolve().parents[1] / "docs/img"
BG = "#e6eaef"
BLUE = "#1d4f9c"

ROBOTS = {
    # name: (urdf, Hand-E mesh folder (to add at {side}_tool0) or None if the URDF has one, pose {joint regex: value}, view)
    "abb_gofa": ("example_abb/descriptions/crb15000_dual_description/robot.urdf",
                 "example_fanuc/fanuc_decription/crx5ia/meshes/hand_e",
                 {r"joint_2$": 0.35, r"joint_3$": 0.25, r"joint_5$": 0.95}, (1.6, -0.9, 0.8)),
    "fanuc_crx5ia": ("example_fanuc/fanuc_decription/crx5ia/dual_hand_e.urdf", None,
                     {r"J2$": 0.35, r"J3$": -0.25, r"J5$": -0.95}, (1.6, -0.9, 0.8)),
    "ur5": ("example_ur/descriptions/ur5_dual_description/robot.urdf", None,
            {r"left_shoulder_pan": np.pi, r"shoulder_lift": -1.3, r"elbow": 1.6, r"wrist_1": -1.87,
             r"wrist_2": -1.57}, (1.6, -0.9, 0.8)),
}
DROP = re.compile(r"2f140|arg2f")        # the ABB description ships a 2F-140; it is swapped for the Hand-E


def hande_meshes(folder):
    """Hand-E on a Z-out tool flange, as in the Robotiq Hand-E description: coupler, body +11 mm, fingers +99 mm."""
    def load(n):
        m = trimesh.load(folder / n, force="mesh")
        return m
    Tz = lambda z, rz=0.0: trimesh.transformations.translation_matrix([0, 0, z]) @ \
        trimesh.transformations.rotation_matrix(rz, [0, 0, 1])
    return [(load("io_coupler.dae"), Tz(0.0)), (load("hande.dae"), Tz(0.011)),
            (load("finger.dae"), Tz(0.110)), (load("finger.dae"), Tz(0.110, np.pi))]


def render(root, name):
    rel, hande, pose, view = ROBOTS[name]
    path = root / rel
    robot = yourdfpy.URDF.load(str(path), load_meshes=True, build_collision_scene_graph=False)
    cfg = {}
    for j in robot.actuated_joint_names:
        for pat, v in pose.items():
            if re.search(pat, j):
                cfg[j] = v
    robot.update_cfg(cfg)
    meshes = []
    for link in robot.link_map.values():
        for vis in link.visuals:
            g = vis.geometry.mesh
            if g is None or DROP.search(g.filename):
                continue
            m = trimesh.load(path.parent / g.filename, force="mesh")
            if g.scale is not None:
                m.apply_scale(g.scale)
            Tv = vis.origin if vis.origin is not None else np.eye(4)
            m.apply_transform(robot.get_transform(link.name) @ Tv)
            meshes.append((m, "finger" in link.name or "hande" in link.name))
    if hande:
        for side in ("left", "right"):
            for m, T in hande_meshes(root / hande):
                m = m.copy()
                m.apply_transform(robot.get_transform(f"{side}_tool0") @ T)
                meshes.append((m, True))
    pl = pv.Plotter(off_screen=True, window_size=(1300, 1000))
    pl.enable_anti_aliasing("ssaa")
    pl.set_background(BG)
    for m, grip in meshes:
        col = "#2b2d31" if grip else (0.9, 0.91, 0.93)
        pl.add_mesh(pv.wrap(m), color=col, smooth_shading=True, split_sharp_edges=True, specular=0.3)
    pl.view_vector(view, viewup=(0, 0, 1))
    pl.reset_camera()
    pl.camera.zoom(1.15)
    out = IMG / f"robot_{name}.png"
    pl.screenshot(str(out))
    print(out, "parts:", len(meshes))


SW_ROBOTS = {
    # name: (urdf relative to assets, package-root map, joint pose {joint-name regex: value}, view)
    "piper": ("piper/piper.urdf", {"piper_description": "piper"},
              {r"joint2$": 1.2, r"joint3$": -1.1, r"joint5$": 0.8, r"joint7$": 0.03, r"joint8$": -0.03}, (1.0, -1.2, 0.7)),
    "openarm": ("openarm/urdf/openarm_v1.urdf", {"openarm_description": "openarm/openarm_description"},
                {r"joint2$": 0.4, r"joint4$": 1.4, r"joint6$": 0.5}, (1.0, -1.2, 0.6)),
}


def handler(assets, root, pkgs):
    def fn(fname):
        m = re.match(r"package://([^/]+)/(.*)", fname)
        if m:
            base = assets / pkgs.get(m.group(1), m.group(1))
            return str(base / m.group(2))
        return str((root / fname).resolve())
    return fn


ENPIRE_OPENARM = Path(__file__).resolve().parent / "source/enpire/open-arm_enpire_left-jaw.stl"


def enpire_fingers(robot):
    """Open-ENPIRE OpenArm jaws in place of the stock OpenArm fingers (world-frame meshes).
    Jaw frame (mm): length +x from the mount, height y, thickness z (gripping side at low z). Stock finger mesh frame
    (mm): x across, y thickness (gripping side at low y), z along the finger; mount section starts near z = 670."""
    jaw = trimesh.load(ENPIRE_OPENARM)
    jaw.apply_scale(1000.0)
    M = np.array([[0, 1, 0, 0.0], [0, 0, 1, 17.3], [1, 0, 0, 676.0], [0, 0, 0, 1]], float)
    out = []
    for link in robot.link_map:
        if not link.endswith("_finger"):
            continue
        sy = -1.0 if link.endswith("right_finger") else 1.0           # the URDF mirrors the right finger mesh in y
        S = np.diag([0.001, 0.001 * sy, 0.001, 1.0])
        S[:3, 3] = [0.0, -0.05 * sy, -0.673001]                        # the URDF visual origin of the finger (mirrored for the right)
        m = jaw.copy()
        m.apply_transform(robot.get_transform(link) @ S @ M)
        if sy < 0:
            m.invert()
        out.append(m)
    return out


def render_sw(assets, name, enpire=False):
    rel, pkgs, pose, view = SW_ROBOTS[name]
    path = assets / rel
    robot = yourdfpy.URDF.load(str(path), filename_handler=handler(assets, path.parent, pkgs), load_meshes=True)
    cfg = {}
    for j in robot.actuated_joint_names:
        for pat, v in pose.items():
            if re.search(pat, j):
                lim = robot.joint_map[j].limit
                cfg[j] = float(np.clip(v, lim.lower, lim.upper)) if lim is not None and lim.lower is not None else v
    robot.update_cfg(cfg)
    scene = robot.scene
    pl = pv.Plotter(off_screen=True, window_size=(1100, 1000))
    pl.enable_anti_aliasing("ssaa")
    pl.set_background(BG)
    for node in scene.graph.nodes_geometry:
        T, gname = scene.graph[node]
        g = scene.geometry[gname]
        if not isinstance(g, trimesh.Trimesh) or (enpire and "finger" in str(gname).lower()):
            continue
        g = g.copy()
        g.apply_transform(T)
        col = g.visual.main_color[:3] / 255.0 if hasattr(g.visual, "main_color") else (0.75, 0.77, 0.8)
        if np.allclose(col, 0) or np.allclose(col, 1):
            col = (0.72, 0.74, 0.78)
        pl.add_mesh(pv.wrap(g), color=col, smooth_shading=True, split_sharp_edges=True, specular=0.3)
    if enpire:
        for m in enpire_fingers(robot):
            pl.add_mesh(pv.wrap(m), color="#1d4f9c", smooth_shading=True, specular=0.3)
    pl.view_vector(view, viewup=(0, 0, 1))
    pl.reset_camera()
    pl.camera.zoom(1.2)
    out = IMG / f"robot_{name}{'_enpire' if enpire else ''}.png"
    pl.screenshot(str(out))
    print(out, "joints posed:", len(cfg))


ENPIRE_YAM = Path(__file__).resolve().parent / "source/enpire/i2rt_UCG_Hard.stl"
ENPIRE_YAM_SOFT = Path(__file__).resolve().parent / "source/enpire/I2RT_UCG_Soft.stl"
YAM_FACE_Z = -145.7          # stock YAM tip mesh (mm): the finger's mating face under the carriage
YAM_MEET_Y = 46.4            # the plane the two stock fingertips meet on (tip_right is tip_left mirrored about it)


def yam_enpire(side):
    """Open-ENPIRE YAM finger (hard, soft) in the stock YAM tip mesh frame (mm). The finger's two 4.6 mm holes go on the
    carriage's M4 pair, at x = 1.2 / 22.2, y = 29.3 (tip_left), its key toward the other finger (+y on tip_left)."""
    M = np.array([[1, 0, 0, 21.9], [0, 0, -1, 43.6], [0, 1, 0, -137.8], [0, 0, 0, 1]], float)   # ENPIRE -> tip_left
    if side == "right":
        F = np.diag([1.0, -1.0, 1.0, 1.0])
        F[1, 3] = 2 * YAM_MEET_Y
        M = F @ M
    out = []
    for src in (ENPIRE_YAM, ENPIRE_YAM_SOFT):
        m = trimesh.load(src)
        m.apply_transform(M)
        out.append(m)
    return out


def render_yam(i2rt):
    """I2RT YAM (yam/v1 URDF) with the stock fingers swapped for the Open-ENPIRE YAM fingers HandUMI carries."""
    path = Path(i2rt) / "i2rt/robot_models/arm/yam/v1/yam.urdf"
    robot = yourdfpy.URDF.load(str(path), load_meshes=True, build_collision_scene_graph=False)
    pose = {r"joint2$": 1.0, r"joint3$": 1.2, r"joint5$": -0.6, r"joint7$": 0.018, r"joint8$": 0.018}
    cfg = {j: v for j in robot.actuated_joint_names for pat, v in pose.items() if re.search(pat, j)}
    robot.update_cfg(cfg)
    meshes = []
    for link in robot.link_map.values():
        for vis in link.visuals:
            g = vis.geometry.mesh
            if g is None:
                continue
            m = trimesh.load(path.parent / g.filename, force="mesh")
            Tv = vis.origin if vis.origin is not None else np.eye(4)
            Tw = robot.get_transform(link.name) @ Tv
            if link.name.startswith("tip_"):
                keep = trimesh.creation.box(extents=[1, 1, 1])
                keep.apply_translation([0, 0, 0.5 + YAM_FACE_Z / 1000.0])        # the carriage only
                m = trimesh.boolean.intersection([m, keep], engine="manifold") if m.is_volume else m.slice_plane(
                    [0, 0, YAM_FACE_Z / 1000.0], [0, 0, 1])
                for k, f in enumerate(yam_enpire(link.name.split("_")[1])):
                    f.apply_scale(0.001)
                    f.apply_transform(Tw)
                    meshes.append((f, "#e07020" if k else BLUE))
            m.apply_transform(Tw)
            meshes.append((m, (0.72, 0.74, 0.78)))
    pl = pv.Plotter(off_screen=True, window_size=(1100, 1000))
    pl.enable_anti_aliasing("ssaa")
    pl.set_background(BG)
    for m, col in meshes:
        pl.add_mesh(pv.wrap(m), color=col, smooth_shading=True, split_sharp_edges=True, specular=0.3)
    pl.view_vector((1.0, -1.2, 0.6), viewup=(0, 0, 1))
    pl.reset_camera()
    pl.camera.zoom(1.2)
    out = IMG / "robot_yam_enpire.png"
    pl.screenshot(str(out))
    print(out, "joints posed:", len(cfg))


if __name__ == "__main__":
    if sys.argv[1] == "--yam":
        render_yam(sys.argv[2])
    elif sys.argv[1] == "--sw":
        for n in (sys.argv[3:] or ["piper", "openarm+enpire"]):
            render_sw(Path(sys.argv[2]), n.split("+")[0], enpire=n.endswith("+enpire"))
    else:
        for n in (sys.argv[2:] or ROBOTS):
            render(Path(sys.argv[1]), n)
