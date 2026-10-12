"""Render bimanual cobots in their own colours with the PG Chopstick ISO 50 build on both flanges.

    python robots.py /path/to/Neuracore_Robots

Uses the dual-arm descriptions in the Neuracore robots repository (not redistributed here): UR5, ABB GoFa CRB 15000
and FANUC CRX-10iA. Their stock grippers (Robotiq Hand-E / 2F) are left out, and the PG Chopstick goes on each
`<side>_flange`. The approach axis is taken from the flange towards the stock tool0, and the camera is kept on top.
Writes docs/img/robot_<name>.png.
"""
import math
import sys
from pathlib import Path

import numpy as np
import pyvista as pv

from assembly import assemble
from parts import load_parts
from render_docs import IMG, plotter, shot

# targets for the left / right TCP (base frame, m), approach direction, and a seed for the IK
ROBOTS = {
    "ur5": dict(elbow="forearm_link", urdf="example_ur/descriptions/ur5_dual_description/robot.urdf",
                tcp={"left": (0.60, -0.17, 0.22), "right": (0.60, 0.17, 0.22)}, approach=(0.6, 0.0, -0.8),
                seed={"shoulder_lift_joint": -1.6, "elbow_joint": 1.6, "wrist_1_joint": -1.6, "wrist_2_joint": -1.57},
                view=(1.6, -0.75, 0.8)),
    "abb_gofa": dict(elbow="link_3", mirror=(-1, 1, 1, -1, 1, -1), urdf="example_abb/descriptions/crb15000_dual_description/robot.urdf",
                     tcp={"left": (0.62, -0.17, 0.30), "right": (0.62, 0.17, 0.30)}, approach=(0.6, 0.0, -0.8),
                     seed={"joint_2": 0.4, "joint_3": 0.4, "joint_5": 0.9},
                     view=(1.6, -0.75, 0.8)),
    "fanuc_crx": dict(elbow=None, mirror=(-1, 1, 1, -1, 1, -1), urdf="assets/fanuc_crx10ia_dual/robot.urdf",
                      tcp={"left": (0.55, -0.17, -0.55), "right": (0.55, 0.17, -0.55)}, approach=(0.6, 0.0, -0.8),
                      seed={"J2": 0.6, "J3": -0.4, "J5": -1.0},
                      view=(1.6, -0.75, 0.5)),
}
TCP_FROM_FLANGE = 0.016 + 0.2047          # ISO plate thickness + gripper-frame TCP x (m)
STOCK_GRIPPER = ("robotiq", "hande", "finger", "knuckle", "coupler", "eef", "pad", "arg2f")


def link_meshes(r, root_dir):
    """[(trimesh in the base frame, rgb 0-1)] for every visual of every link, keeping each material's colour.

    COLLADA files carry one sub-mesh per material (UR's blue ring, FANUC's accents); STL visuals take the URDF
    <material> colour. Links of the stock grippers are left out.
    """
    import trimesh
    out = []
    for name, link in r.link_map.items():
        if any(s in name.lower() for s in STOCK_GRIPPER):
            continue
        T_link = r.get_transform(name, r.base_link)
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
                for gname, T_node in ((n, loaded.graph[n][0]) for n in loaded.graph.nodes_geometry):
                    g = loaded.geometry[loaded.graph[gname][1]].copy()
                    rgb = urdf_rgb
                    try:
                        rgb = np.array(g.visual.material.main_color[:3]) / 255.0
                    except Exception:
                        pass
                    out.append((g.apply_transform(T @ S @ T_node), rgb if rgb is not None else np.array([0.85] * 3)))
            else:
                out.append((loaded.apply_transform(T @ S), urdf_rgb if urdf_rgb is not None else np.array([0.85] * 3)))
    return out


def mount_frame(r, side):
    """Flange pose with +X along the approach (flange -> stock tool0) and +Z as close to world up as possible."""
    F = r.get_transform(f"{side}_flange", r.base_link)
    tool = r.get_transform(f"{side}_tool0", r.base_link)
    x = (tool[:3, 3] - F[:3, 3]).copy()
    if np.linalg.norm(x) < 1e-6:                     # tool0 at the flange: use the flange's own z
        x = F[:3, 2].copy()
    x /= np.linalg.norm(x)
    up = np.array([0, 0, 1.0]) - x * x[2]
    if np.linalg.norm(up) < 1e-3:
        up = np.cross(x, [0, 1.0, 0])
    z = up / np.linalg.norm(up)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = F[:3, 3]
    return T


def approach_in_flange(r, side):
    F = r.get_transform(f"{side}_flange", r.base_link)
    t = r.get_transform(f"{side}_tool0", r.base_link)
    a = t[:3, 3] - F[:3, 3]
    a = F[:3, 2] if np.linalg.norm(a) < 1e-6 else a / np.linalg.norm(a)
    return F[:3, :3].T @ a


def solve(r, side, tcp, approach, seed, elbow, start_q=None):
    """Joint values for one arm putting its TCP at `tcp` with the gripper approach along `approach`."""
    from scipy.optimize import minimize
    joints = [j for j in r.actuated_joint_names if j.startswith(side) and r.joint_map[j].type == "revolute"
              and not any(s in j for s in STOCK_GRIPPER)]
    a_f = approach_in_flange(r, side)
    want_a = np.asarray(approach, float) / np.linalg.norm(approach)
    lims = [(r.joint_map[j].limit.lower, r.joint_map[j].limit.upper) for j in joints]
    x0 = np.array([next((v for k, v in seed.items() if j.endswith(k)), 0.0) for j in joints])
    if side == "right":
        x0 = np.array([-v if j.endswith(("shoulder_pan_joint", "joint_1", "J1")) else v for j, v in zip(joints, x0)])
    if start_q is not None:
        x0 = np.asarray(start_q, float)

    def cost(q):
        r.update_cfg(dict(zip(joints, q)))
        F = r.get_transform(f"{side}_flange", r.base_link)
        a = F[:3, :3] @ a_f
        p = F[:3, 3] + a * TCP_FROM_FLANGE
        z_elbow = r.get_transform(f"{side}_{elbow}", r.base_link)[2, 3] if elbow else 0.0
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
    cfg = dict(zip(joints, best.x))
    r.update_cfg(cfg)
    return cfg, best.fun


def render(root, name, spec, parts):
    import yourdfpy
    path = Path(root) / spec["urdf"]
    r = yourdfpy.URDF.load(str(path), build_scene_graph=True, load_meshes=False)
    cfg = {}
    left_q = None
    for side in ("left", "right"):
        start = None
        if side == "right" and spec.get("mirror") is not None:
            start = np.array(spec["mirror"]) * left_q          # mirror image of the left arm's solution
        c, err = solve(r, side, np.array(spec["tcp"][side]), spec["approach"], spec["seed"], spec["elbow"], start)
        left_q = np.array(list(c.values())) if side == "left" else left_q
        cfg.update(c)
        print(f"  {name} {side}: IK residual {err:.2e}")
    r.update_cfg(cfg)
    p = plotter((1600, 1200))
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for m, rgb in link_meshes(r, path.parent):
        p.add_mesh(pv.wrap(m), color=rgb, smooth_shading=True, specular=0.35, specular_power=30)
        lo, hi = np.minimum(lo, m.bounds[0]), np.maximum(hi, m.bounds[1])
    for side, deg in (("left", 22), ("right", 35)):
        T = mount_frame(r, side) @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
        for m in assemble("iso50", math.radians(deg), parts).values():
            mm = m.copy().apply_transform(T)
            p.add_mesh(pv.wrap(mm), color=mm.visual.face_colors[0, :3] / 255.0, smooth_shading=True,
                       split_sharp_edges=True, specular=0.3, specular_power=25)
            lo, hi = np.minimum(lo, mm.bounds[0]), np.maximum(hi, mm.bounds[1])
    c = (lo + hi) / 2
    d = np.asarray(spec["view"]) / np.linalg.norm(spec["view"])
    shot(p, IMG / f"robot_{name}.png", tuple(c + d * 3.0 * np.linalg.norm(hi - lo)), tuple(c), zoom=1.3)


def main():
    root = sys.argv[1]
    only = sys.argv[2:] or list(ROBOTS)
    parts = load_parts()
    for name in only:
        render(root, name, ROBOTS[name], parts)


if __name__ == "__main__":
    main()
