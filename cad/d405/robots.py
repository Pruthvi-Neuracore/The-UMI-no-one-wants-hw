"""Render bimanual cobots in their own colours with the PG Chopstick ISO 50 build on both flanges.

    python robots.py /path/to/Neuracore_Robots [ur5 abb_gofa fanuc_crx]

Uses robot descriptions from the Neuracore robots repository (not redistributed here):
  * UR5 and ABB GoFa CRB 15000: their dual-arm descriptions, with the stock grippers (Robotiq Hand-E / 2F) left out,
  * FANUC CRX-5iA: the clean single-arm description (crx5ia/arm.urdf), placed twice side by side.
The PG Chopstick goes on each arm's flange: the approach axis runs from the flange towards the arm's tool0, and the
camera is kept on top. Both arms are posed by a small IK so their tips meet in front of the robot.
Writes docs/img/robot_<name>.png.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

from assembly import assemble
from parts import load_parts
from render_docs import IMG, plotter, shot

ROBOTS = {
    "ur5": dict(urdf="example_ur/descriptions/ur5_dual_description/robot.urdf", dual=True, elbow="forearm_link",
                tcp={"left": (0.60, -0.17, 0.22), "right": (0.60, 0.17, 0.22)}, approach=(0.6, 0.0, -0.8),
                seed={"shoulder_lift_joint": -1.6, "elbow_joint": 1.6, "wrist_1_joint": -1.6, "wrist_2_joint": -1.57},
                view=(1.6, -0.75, 0.8)),
    "abb_gofa": dict(urdf="example_abb/descriptions/crb15000_dual_description/robot.urdf", dual=True, elbow="link_3",
                     mirror=(-1, 1, 1, -1, 1, -1),
                     tcp={"left": (0.62, -0.17, 0.30), "right": (0.62, 0.17, 0.30)}, approach=(0.6, 0.0, -0.8),
                     seed={"joint_2": 0.4, "joint_3": 0.4, "joint_5": 0.9},
                     view=(1.6, -0.75, 0.8)),
    "fanuc_crx": dict(urdf="example_fanuc/fanuc_decription/crx5ia/arm.urdf", dual=False, elbow="J3_link",
                      bases={"left": (0.0, -0.45, 0.0), "right": (0.0, 0.45, 0.0)}, mirror=(-1, 1, 1, -1, 1, -1),
                      tcp={"left": (0.58, -0.17, 0.22), "right": (0.58, 0.17, 0.22)}, approach=(0.6, 0.0, -0.8),
                      seed={"J2": 0.3, "J3": -0.3, "J5": -1.2},
                      view=(1.6, -0.75, 0.8)),
}
TCP_FROM_FLANGE = 0.016 + 0.2047          # ISO plate thickness + gripper-frame TCP x (m)
STOCK_GRIPPER = ("robotiq", "hande", "finger", "knuckle", "coupler", "eef", "pad", "arg2f")


class Arm:
    """One arm: a yourdfpy model, the pose of its base in the scene, and the prefix its links carry."""

    def __init__(self, r, prefix="", T_base=np.eye(4)):
        self.r, self.prefix, self.T = r, prefix, np.asarray(T_base, float)

    def link(self, name):
        return self.T @ self.r.get_transform(self.prefix + name, self.r.base_link)

    def joints(self):
        return [j for j in self.r.actuated_joint_names if j.startswith(self.prefix)
                and self.r.joint_map[j].type == "revolute" and not any(s in j for s in STOCK_GRIPPER)]


def link_meshes(r, root_dir, T_scene=np.eye(4), keep=None):
    """[(trimesh in the scene frame, rgb 0-1)] for every visual of every link, keeping each material's colour.

    COLLADA files carry one sub-mesh per material (UR's blue ring, FANUC's green bands); STL visuals take the URDF
    <material> colour. Links of the stock grippers are left out.
    """
    out = []
    for name, link in r.link_map.items():
        if any(s in name.lower() for s in STOCK_GRIPPER) or (keep and not keep(name)):
            continue
        T_link = T_scene @ r.get_transform(name, r.base_link)
        for v in link.visuals:
            if v.geometry is None or v.geometry.mesh is None:
                continue
            fn = (root_dir / v.geometry.mesh.filename.replace("package://", "")).resolve()
            if not fn.exists():
                continue
            T = T_link @ (v.origin if v.origin is not None else np.eye(4))
            sc = v.geometry.mesh.scale
            S = np.diag([*(np.broadcast_to(sc, 3) if sc is not None else (1, 1, 1)), 1.0])
            urdf_rgb = None
            if v.material is not None and v.material.color is not None:
                urdf_rgb = np.array(v.material.color.rgba[:3])
            loaded = trimesh.load(str(fn))
            if isinstance(loaded, trimesh.Scene):
                for node in loaded.graph.nodes_geometry:
                    T_node, gname = loaded.graph[node]
                    g = loaded.geometry[gname].copy()
                    rgb = urdf_rgb
                    try:
                        rgb = np.array(g.visual.material.main_color[:3]) / 255.0
                    except Exception:
                        pass
                    out.append((g.apply_transform(T @ S @ T_node), rgb if rgb is not None else np.array([0.85] * 3)))
            else:
                out.append((loaded.apply_transform(T @ S), urdf_rgb if urdf_rgb is not None else np.array([0.85] * 3)))
    return out


def approach_in_flange(arm):
    """The tool approach direction in the flange frame: from the flange towards tool0, or, where tool0 sits on the
    flange, tool0's own +Z (the ROS-Industrial tool0 convention)."""
    F, t = arm.link("flange"), arm.link("tool0")
    a = t[:3, 3] - F[:3, 3]
    a = t[:3, 2].copy() if np.linalg.norm(a) < 1e-6 else a / np.linalg.norm(a)
    return F[:3, :3].T @ a


def mount_frame(arm, a_f):
    """Flange pose with +X along the approach and +Z as close to world up as possible (camera on top)."""
    F = arm.link("flange")
    x = F[:3, :3] @ a_f
    up = np.array([0, 0, 1.0]) - x * x[2]
    if np.linalg.norm(up) < 1e-3:                    # tool pointing straight up or down: keep the camera towards -X
        up = np.array([-1.0, 0, 0]) - x * x[0]
    z = up / np.linalg.norm(up)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = F[:3, 3]
    return T


def solve(arm, tcp, approach, seed, elbow, start_q=None):
    """Joint values for one arm putting its TCP at `tcp` (scene frame) with the gripper approach along `approach`."""
    from scipy.optimize import minimize
    joints = arm.joints()
    a_f = approach_in_flange(arm)
    want_a = np.asarray(approach, float) / np.linalg.norm(approach)
    lims = [(arm.r.joint_map[j].limit.lower, arm.r.joint_map[j].limit.upper) for j in joints]
    x0 = np.array([next((v for k, v in seed.items() if j.endswith(k)), 0.0) for j in joints])
    if start_q is not None:
        x0 = np.asarray(start_q, float)

    def cost(q):
        arm.r.update_cfg(dict(zip(joints, q)))
        F = arm.link("flange")
        a = F[:3, :3] @ a_f
        p = F[:3, 3] + a * TCP_FROM_FLANGE
        z_elbow = arm.link(elbow)[2, 3] if elbow else 0.0
        return (np.sum((p - tcp) ** 2) * 100 + np.sum((a - want_a) ** 2) + 1e-4 * np.sum((q - x0) ** 2)
                - 0.02 * z_elbow)                  # prefer elbow-up solutions

    best = None
    for k in range(16):
        start = x0.copy()
        if k:
            start = x0 + np.random.default_rng(k).normal(0, 0.9, len(x0))
            if k % 2:
                start[0] += np.pi                   # try the first joint turned round as well
        start = np.clip(start, *np.array(lims).T)
        res = minimize(cost, start, bounds=lims, method="L-BFGS-B")
        if best is None or res.fun < best.fun:
            best = res
    arm.r.update_cfg(dict(zip(joints, best.x)))
    return best.x, best.fun, a_f


def render(root, name, spec, parts):
    import yourdfpy
    path = Path(root) / spec["urdf"]
    if spec["dual"]:
        r = yourdfpy.URDF.load(str(path), build_scene_graph=True, load_meshes=False)
        arms = {s: Arm(r, f"{s}_") for s in ("left", "right")}
        models = [(r, np.eye(4))]
    else:
        arms = {s: Arm(yourdfpy.URDF.load(str(path), build_scene_graph=True, load_meshes=False), "",
                       trimesh.transformations.translation_matrix(spec["bases"][s])) for s in ("left", "right")}
        models = [(a.r, a.T) for a in arms.values()]
    q_left, a_f = None, {}
    for side in ("left", "right"):
        start = np.array(spec["mirror"]) * q_left if side == "right" and spec.get("mirror") else None
        q, err, a_f[side] = solve(arms[side], np.array(spec["tcp"][side]), spec["approach"], spec["seed"],
                                  spec["elbow"], start)
        q_left = q if side == "left" else q_left
        print(f"  {name} {side}: IK cost {err:.2e}")
    p = plotter((1600, 1200), scale=1e-3)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for r, T in models:
        for m, rgb in link_meshes(r, path.parent, T):
            p.add_mesh(pv.wrap(m), color=rgb, smooth_shading=True, specular=0.35, specular_power=30)
            lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
    for side, frac in (("left", 0.4), ("right", 0.65)):
        T = mount_frame(arms[side], a_f[side]) @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
        for m in assemble(spec.get("gripper", "chopstick_iso50"), frac, parts).values():
            mm = m.copy().apply_transform(T)
            p.add_mesh(pv.wrap(mm), color=mm.visual.face_colors[0, :3] / 255.0, smooth_shading=True,
                       split_sharp_edges=True, specular=0.3, specular_power=25)
            lo, hi = np.minimum(lo, mm.bounds[0]), np.maximum(hi, mm.bounds[1])
    c = (lo + hi) / 2
    d = np.asarray(spec["view"]) / np.linalg.norm(spec["view"])
    shot(p, IMG / f"robot_{name}.png", tuple(c + d * 3.0 * np.linalg.norm(hi - lo)), tuple(c), zoom=1.3)


def robot_hand(root, parts):
    """The hand variant's robot twin: a Shadow Hand on a UR5e with the same D405 head strapped to its forearm
    (ebox_wrist_saddle_robot), close up. Writes docs/img/hand_robot.png."""
    import yourdfpy
    import wrist_scene as W
    from render_docs import add, wrist
    from assembly import colour_of, to_mesh
    global TCP_FROM_FLANGE
    spec = ROBOTS["ur5"]
    path = Path(root) / spec["urdf"]
    r = yourdfpy.URDF.load(str(path), build_scene_graph=True, load_meshes=False)
    arm = Arm(r, "right_")
    saved, TCP_FROM_FLANGE = TCP_FROM_FLANGE, 0.33              # flange to the palm
    solve(arm, np.array([0.70, 0.20, 0.30]), (1.0, 0.0, 0.18), spec["seed"], spec["elbow"])
    TCP_FROM_FLANGE = saved
    M = mount_frame(arm, approach_in_flange(arm))                 # x along the approach, z up
    T_sh = np.eye(4)
    T_sh[:3, :3] = M[:3, :3] @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])   # Shadow: z forward, back of the hand up
    T_sh[:3, 3] = M[:3, 3]
    T_arm, k = wrist(parts)
    scene, T_sh_in_g = W.shadow(T_arm, r_saddle=68.5, **k)
    T_g = T_sh @ np.diag([1e-3, 1e-3, 1e-3, 1.0]) @ np.linalg.inv(T_sh_in_g)   # gripper frame (mm) -> world (m)
    p = plotter((1600, 1200), scale=1e-3)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    close = lambda n: n.startswith("right_") and any(k in n for k in ("forearm", "wrist", "flange", "tool"))
    for m, rgb in link_meshes(r, path.parent, keep=close):          # the arm from the elbow on, for the close-up
        p.add_mesh(pv.wrap(m), color=rgb, smooth_shading=True, specular=0.35, specular_power=30)
    meshes = assemble("hand", 0.0, parts)
    meshes.pop("ebox_wrist_saddle", None)
    sad = to_mesh(parts["ebox_wrist_saddle_robot"], 0.05)
    sad.visual.face_colors = trimesh.visual.color.hex_to_rgba(colour_of("ebox_wrist_saddle"))
    meshes["ebox_wrist_saddle_robot"] = sad
    for m in list(meshes.values()):
        mm = m.copy().apply_transform(T_g)
        p.add_mesh(pv.wrap(mm), color=mm.visual.face_colors[0, :3] / 255.0, smooth_shading=True,
                   split_sharp_edges=True, specular=0.3, specular_power=25)
        lo, hi = np.minimum(lo, mm.bounds[0]), np.maximum(hi, mm.bounds[1])
    for m, col in scene:
        mm = m.copy().apply_transform(T_g)
        p.add_mesh(pv.wrap(mm), color=col, smooth_shading=True, specular=0.4, specular_power=30)
        lo, hi = np.minimum(lo, mm.bounds[0]), np.maximum(hi, mm.bounds[1])
    c = (lo + hi) / 2
    d = np.array([0.35, 1.0, 0.35])
    shot(p, IMG / "hand_robot.png", tuple(c + d / np.linalg.norm(d) * 2.5), tuple(c), zoom=1.6)


def main():
    root = sys.argv[1]
    only = sys.argv[2:] or [*ROBOTS, "hand"]
    parts = load_parts()
    for name in only:
        if name == "hand":
            robot_hand(root, parts)
        else:
            render(root, name, ROBOTS[name], parts)


if __name__ == "__main__":
    main()
