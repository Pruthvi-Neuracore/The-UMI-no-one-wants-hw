"""Robot camera posts for the parallel (HandUMI) gripper: the same camera, in the same place relative to the jaws, on
each robot the tip sets reproduce.

On the handheld gripper the camera hangs on HandUMI's Ø12 / M4 hinge. Each robot gets a printed post that carries the
same hinge knuckle at the same pose relative to its fingertips, so the same camera mount (this kit's `x5_wrist_mount`
with the X5, or HandUMI's `d405_wrist_mount` with the D405) bolts on and the robot sees what the device saw:

  * `camera_post_hande`: Robotiq Hand-E on an ISO 9409-1-50-4-M6 flange (UR5 / UR5e, ABB GoFa, FANUC CRX). A 6 mm
    plate between the flange and the Hand-E's coupler (4 x M6 x 16 -> longer screws by 6 mm, the coupler's dowel goes
    through), with the post up the side of the Hand-E.
  * `camera_post_piper`: AgileX Piper. A split collar round the gripper's base (2 x M3 x 16 + nuts), with the post.
  * `camera_post_openarm`: OpenArm v1 with Open-ENPIRE fingers. A split collar round link7's gripper housing.

"The same pose": the fingertip frame (origin midway between the two fingertip pads, x along the approach, y from one
jaw to the other) is found on the device with each tip set and on the robot with its own fingers; the hinge sits in
the same place in that frame. The jaws are symmetric, so the camera may go on either side; the side that clears the
robot is used. Checked for clashes against the robot's gripper and wrist and the fingers over their stroke.

    python robot_cameras.py /path/to/Neuracore_Robots /path/to/handumi-sw/assets [--no-render]

Writes the posts (STEP / STL) to build/<kit>/t_gripper/hardware/{STEP,STL}/robot/ and the renders robot_camera_*.png.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import trimesh

import t_gripper as TG
from tips import TIP_SETS

A = TG.A
H = sys.modules.get("hinge") or __import__("hinge")
HW_OUT = TG.OUT / "hardware"
IMG = TG.KIT_BUILD / "docs" / "img"
KNUCKLE_OD, KNUCKLE_W, BOLT_D = 12.0, 10.0, 4.4          # HandUMI's hinge (hinge.py): centre knuckle and M4
BAR = (12.0, 9.0)                                        # the post's section: across the jaws x away from the gripper
NECK = 10.0
CLEAR = 3.0                                              # minimum gap from the post to the robot
ROBOT_GREY, GRIPPER_DARK, POST_BLUE = "#d6d9de", "#2b2e33", "#1f5fae"


# --- the fingertip frame ------------------------------------------------------------------------------------------
def tip_frame(tA, tB, a, depth=8.0):
    """4x4: origin midway between the two fingertip pads (each the centroid of its mesh's last `depth` mm along the
    approach a), x = a, y from tA to tB, z = x cross y."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    cs = []
    for m in (tA, tB):
        v = m.vertices
        d = v @ a
        cs.append(v[d > d.max() - depth].mean(0))
    o = (cs[0] + cs[1]) / 2
    y = cs[1] - cs[0]
    y = y - y.dot(a) * a
    y /= np.linalg.norm(y)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([a, y, np.cross(a, y)])
    T[:3, 3] = o
    return T


def device(tipset, opening=0.5):
    """The handheld gripper with a tip set: its meshes in HandUMI's frame M, its fingertip frame, and the camera parts
    (mount + camera) and the hinge knuckle's centre and axis in M."""
    parts = TG.build(opening, tipset)
    T_tip = tip_frame(parts["tip_thumb"][0], parts["tip_index"][0], [-1.0, 0, 0])
    cam = {n: parts[n] for n in parts if n in ("x5_wrist_mount", "insta360_x5", "camera_mount", "d405")}
    k = np.array([H.AXIS_M[0], H.Y_CENTRE, H.AXIS_M[1]])
    return parts, T_tip, cam, k, np.array([0, 1.0, 0])


# --- the robots' grippers -----------------------------------------------------------------------------------------
def _urdf(path, pkgs=None):
    import yourdfpy

    def handler(fname):
        m = re.match(r"package://([^/]+)/(.*)", fname)
        if m and pkgs is not None:
            return str(pkgs[m.group(1)] / m.group(2))
        return str((path.parent / fname).resolve())
    return yourdfpy.URDF.load(str(path), filename_handler=handler, load_meshes=True, build_scene_graph=True)


def _link_meshes(r, root, names):
    """{link: trimesh in mm, in `root`'s frame} for the visuals of the named links."""
    out = {}
    T0 = np.linalg.inv(r.get_transform(root, r.base_link))
    for name in names:
        link = r.link_map[name]
        ms = []
        for v in link.visuals:
            g = v.geometry.mesh
            if g is None:
                continue
            m = trimesh.load(r._filename_handler(g.filename), force="mesh")
            S = np.diag([*(np.broadcast_to(g.scale, 3) if g.scale is not None else (1, 1, 1)), 1.0])
            Tv = v.origin if v.origin is not None else np.eye(4)
            m.apply_transform(T0 @ r.get_transform(name, r.base_link) @ Tv @ S)
            if np.linalg.det(S[:3, :3]) < 0:
                m.invert()
            ms.append(m)
        if ms:
            out[name] = trimesh.util.concatenate(ms).apply_scale(1000.0)
    return out


def hande(nr_root, plate_t=6.0):
    """Robotiq Hand-E on an ISO 50 flange, in the flange frame (mm, z out of the flange): coupler, body and fingers
    (closed), lifted by the post's plate. Approach +z."""
    d = Path(nr_root) / "example_fanuc/fanuc_decription/crx5ia/meshes/hand_e"
    # the description's coupler origin is 3 mm above its bottom face: lift the stack so that face sits on the plate
    lift = plate_t - trimesh.load(d / "io_coupler.dae", force="mesh").bounds[0][2] * 1000.0

    def load(n, z, rz=0.0):
        m = trimesh.load(d / n, force="mesh").apply_scale(1000.0)
        m.apply_transform(trimesh.transformations.rotation_matrix(rz, [0, 0, 1]))
        return m.apply_translation([0, 0, z + lift])
    fixed = {"hande_coupler": load("io_coupler.dae", 0.0), "hande_body": load("hande.dae", 11.0)}
    fingers = [load("finger.dae", 110.0), load("finger.dae", 110.0, np.pi)]
    flange = trimesh.creation.cylinder(radius=31.5, height=20.0).apply_translation([0, 0, -10.0])
    return dict(fixed=fixed, fingers=fingers, approach=np.array([0, 0, 1.0]), wrist={"flange": flange},
                stroke=_stroke(fingers, 25.0))


def piper(sw):
    """AgileX Piper's gripper in link6's frame (mm, z = approach)."""
    sw = Path(sw)
    path = sw / "piper/piper.urdf"
    r = _urdf(path, {"piper_description": sw / "piper"})
    r.update_cfg({j: 0.0 for j in r.actuated_joint_names})
    pre = next(n for n in r.link_map if n.endswith("link6")).replace("link6", "")
    ms = _link_meshes(r, pre + "link6", [pre + n for n in ("link6", "gripper_base", "link7", "link8")])
    fingers = [ms.pop(pre + "link7"), ms.pop(pre + "link8")]
    wrist = {"link6": ms.pop(pre + "link6")}
    return dict(fixed={"piper_gripper_base": ms[pre + "gripper_base"]}, fingers=fingers, approach=np.array([0, 0, 1.0]),
                wrist=wrist, stroke=_stroke(fingers, 35.0))


def openarm(sw):
    """OpenArm v1's right link7 (with its gripper housing) and Open-ENPIRE jaws, in link7's frame (mm)."""
    sw = Path(sw)
    path = sw / "openarm/urdf/openarm_v1.urdf"
    r = _urdf(path, {"openarm_description": sw / "openarm/openarm_description"})
    r.update_cfg({j: 0.0 for j in r.actuated_joint_names})
    root = "openarm_right_link7"
    ms = _link_meshes(r, root, [root])
    jaws = _enpire(r, root)
    tcp = r.get_transform("openarm_right_hand_tcp", root)[:3, 3]
    a = tcp / np.linalg.norm(tcp)
    return dict(fixed={"openarm_link7": ms[root]}, fingers=jaws, approach=a, wrist={}, stroke=_stroke(jaws, 30.0))


ENPIRE_JAW = Path(__file__).resolve().parent / "handumi" / "hardware" / "STL" / "gripper_tips" / "Open-ENPIRE" / \
    "robot" / "OpenArm_Hard.stl"


def _enpire(r, root):
    """The Open-ENPIRE OpenArm jaws on link7's two finger links, as HandUMI's robot render places them."""
    jaw = trimesh.load(ENPIRE_JAW)
    M = np.array([[0, 1, 0, 0.0], [0, 0, 1, 17.3], [1, 0, 0, 676.0], [0, 0, 0, 1]], float)
    out = []
    T0 = np.linalg.inv(r.get_transform(root, r.base_link))
    for link in (root.replace("link7", "left_finger"), root.replace("link7", "right_finger")):
        sy = -1.0 if link.endswith("right_finger") else 1.0
        S = np.diag([0.001, 0.001 * sy, 0.001, 1.0])
        S[:3, 3] = [0.0, -0.05 * sy, -0.673001]
        m = jaw.copy()
        m.apply_transform(T0 @ r.get_transform(link, r.base_link) @ S @ M)
        m.apply_scale(1000.0)
        if sy < 0:
            m.invert()
        out.append(m)
    return out


def _stroke(fingers, half):
    """The fingers' travel: each moves along the line between them, up to `half` mm outwards."""
    c = [f.centroid for f in fingers]
    d = (c[1] - c[0]) / np.linalg.norm(c[1] - c[0])
    return [(-d * half, d * half)]


ROBOTS = {
    # name: (loader, tip set, the robots it stands for)
    "hande": (hande, "Robotiq Hand-E", "UR5 / UR5e, ABB GoFa, FANUC CRX"),
    "piper": (piper, "AgileX Piper", "AgileX Piper"),
    "openarm": (openarm, "Open-ENPIRE (UCG)", "OpenArm"),
}


def overlap_mm3(a, b):
    try:
        return float(a.intersection(b).volume)
    except Exception:
        return 0.0


# --- the post ------------------------------------------------------------------------------------------------------
BASES = {
    # robot: the post's base. plate: ISO 50 disc between the flange and the coupler; collar: split clamp round a
    # cylinder of radius r over z0..z1 (link frame)
    "hande": dict(kind="plate", t=6.0, d=63.0, pcd=50.0, holes=6.6),
    "piper": dict(kind="collar", r=28.5, z0=12.0, z1=38.0),
    "openarm": dict(kind="collar", r=28.5, z0=58.0, z1=86.0),
}
COLLAR_T, COLLAR_GAP = 5.0, 2.0


def placement(name, g, tipset):
    """G: 4x4 taking HandUMI's frame M (the device) into the robot's frame, for both sides; the one whose camera and
    post clear the robot is chosen later."""
    _, T_dev, cam, k, kaxis = device(tipset)
    T_rob = tip_frame(g["fingers"][0], g["fingers"][1], g["approach"])
    out = []
    for flip in (np.eye(4), np.diag([1.0, -1.0, -1.0, 1.0])):
        G = T_rob @ flip @ np.linalg.inv(T_dev)
        out.append((G, (G @ [*k, 1])[:3], G[:3, :3] @ kaxis))
    return out, cam


def _robot_parts(g, hull=False):
    """The robot's gripper and wrist, with each finger at both ends of its stroke. hull=True: each part's convex hull
    (watertight, a little bigger than the part: a safe stand-in for clash checks, since the robots' meshes are open)."""
    ms = list(g["fixed"].values()) + list(g["wrist"].values())
    for f, (d0, d1) in zip(g["fingers"], g["stroke"] * 2):
        sgn = -1 if f is g["fingers"][0] else 1
        ms += [f, f.copy().apply_translation(sgn * d1)]
    return [m.convex_hull for m in ms] if hull else ms


def clash_points(g, solids, n=60000, inset=0.3):
    """How many points sampled on the robot's surface (gripper, wrist, fingers over their stroke) lie inside any of
    the given watertight solids (the post, the camera mount, the camera), more than `inset` mm deep."""
    pts = np.vstack([trimesh.sample.sample_surface_even(m, n)[0] for m in _robot_parts(g)])
    hits = 0
    for sld in solids:
        lo, hi = sld.bounds
        cand = pts[np.all((pts > lo - 1) & (pts < hi + 1), axis=1)]
        if len(cand):
            inside = sld.contains(cand)
            if inside.any():
                d = trimesh.proximity.closest_point(sld, cand[inside])[1]
                hits += int((d > inset).sum())
    return hits


def _robot_mesh(g):
    return trimesh.util.concatenate(_robot_parts(g))


def build_post(name, K, kaxis, robot):
    """The post (build123d solid, in the robot's link frame, mm) carrying the hinge knuckle at K, axis kaxis."""
    from build123d import Box, Cylinder, Location, Plane, Pos, Rot
    base = BASES[name]
    z_hat = np.array([0, 0, 1.0])
    u = np.array([K[0], K[1], 0.0])
    u /= np.linalg.norm(u)
    t = np.cross(z_hat, u)
    plane = Plane(origin=(0, 0, 0), x_dir=tuple(u), z_dir=(0, 0, 1))      # local x = u (out), y = t, z = approach
    Ku, Kt, Kz = K @ u, K @ t, K[2]
    # how far the robot reaches out towards the post, round the bar's band
    v = robot.vertices
    z_lo = base["z0"] if base["kind"] == "collar" else 0.0
    band = v[(np.abs(v @ t - Kt) < BAR[0] / 2 + 4) & (v[:, 2] > z_lo - 2) & (v[:, 2] < Kz + KNUCKLE_OD)]
    r_in = max(band @ u) + CLEAR if len(band) else 20.0
    if base["kind"] == "collar":
        r_in = max(r_in, base["r"] + 0.3 + COLLAR_T - 1.0)
    r_out = r_in + BAR[1]
    if r_out > Ku - KNUCKLE_OD / 2 - 1.0:
        raise RuntimeError(f"{name}: no room for the post between the robot (r {r_in:.1f}) and the knuckle ({Ku:.1f})")
    z_top = Kz + KNUCKLE_OD / 2
    parts = []
    bar = Pos((r_in + r_out) / 2, Kt, (z_lo + z_top) / 2) * Box(BAR[1], BAR[0], z_top - z_lo)
    neck = Pos((r_out - 1 + Ku) / 2, Kt, Kz) * Box(Ku - r_out + 1, NECK, KNUCKLE_OD)
    knuckle = Pos(Ku, Kt, Kz) * Rot(90, 0, 0) * Cylinder(KNUCKLE_OD / 2, KNUCKLE_W)
    post = bar + neck + knuckle
    if base["kind"] == "plate":
        disc = Pos(0, 0, base["t"] / 2) * Cylinder(base["d"] / 2, base["t"])
        tab = Pos(r_out / 2, Kt, base["t"] / 2) * Box(r_out, BAR[0] + 10, base["t"])
        post = post + disc + tab
        for i in range(4):
            a = math.radians(45 + 90 * i)
            post -= Pos(base["pcd"] / 2 * math.cos(a), base["pcd"] / 2 * math.sin(a), base["t"] / 2) * \
                Cylinder(base["holes"] / 2, base["t"] + 2)
        post -= Pos(0, 0, base["t"] / 2) * Cylinder(6.1 / 2, base["t"] + 2)        # the coupler's centre dowel
    else:
        r0, r1 = base["r"] + 0.3, base["r"] + 0.3 + COLLAR_T
        h = base["z1"] - base["z0"]
        zc = (base["z0"] + base["z1"]) / 2
        ring = Pos(0, 0, zc) * (Cylinder(r1, h) - Cylinder(r0, h + 2))
        ears = Pos(-(r1 + 4.0), 0, zc) * Box(12.0, 2 * COLLAR_GAP + 12.0, h)
        ring = ring + Pos(0, 0, 0) * (ears - Pos(0, 0, zc) * Cylinder(r0, h + 2))
        ring -= Pos(-(r0 + 8.0), 0, zc) * Box(20.0, COLLAR_GAP, h + 2)             # the split
        for dz in (-h / 4, h / 4):                                                 # 2 x M3 across the split
            ring -= Pos(-(r1 + 5.0), 0, zc + dz) * Rot(90, 0, 0) * Cylinder(1.7, 2 * COLLAR_GAP + 14)
        # turn the ring so its split is opposite the post (local -x), in the post's own frame
        post = post + ring
    post -= Pos(Ku, Kt, Kz) * Rot(90, 0, 0) * Cylinder(BOLT_D / 2, KNUCKLE_W + 2)    # M4 pivot
    # from the post's frame (x = u, y = t) to the link frame
    R = np.column_stack([u, t, z_hat])
    from OCP.gp import gp_Trsf
    tr = gp_Trsf()
    tr.SetValues(*R[0], 0.0, *R[1], 0.0, *R[2], 0.0)
    return post.moved(Location(tr)), dict(r_in=r_in, Ku=Ku, Kz=Kz)


def solve(name, g, tipset):
    """Place the camera on the side that clears; returns (post solid, post mesh, camera meshes, G, info)."""
    robot = _robot_mesh(g)
    (sides, cam) = placement(name, g, tipset)
    best = None
    for side, (G, K, kaxis) in enumerate(sides):
        # build in a frame where K has no tangential offset: rotate the link frame so the knuckle sits on the post's
        # centre plane (the band test uses t = K's tangential coordinate)
        try:
            post, info = build_post(name, K, kaxis, robot)
        except RuntimeError as e:
            print("  ", e)
            continue
        pm = TG.A_to_mesh(post, 0.05)
        cams = {n: m.copy().apply_transform(G) for n, (m, _) in cam.items()}
        clash = clash_points(g, [pm, *cams.values()])
        print(f"  {name} side {side}: knuckle {np.round(K, 1)}, post from r {info['r_in']:.1f}, "
              f"robot points inside the post / camera: {clash}")
        if best is None or clash < best[-1]:
            best = (post, pm, cams, G, side, clash)
    return best


def side_by_side(name, g, tipset, post_mesh, cams, G, side, path):
    """The handheld gripper and the robot's gripper side by side, each in its own fingertip frame (same view), so the
    camera can be seen to sit in the same place."""
    import pyvista as pv
    from render_docs import add, plotter, shot
    parts, T_dev, _, _, _ = device(tipset)
    flip = np.eye(4) if side == 0 else np.diag([1.0, -1.0, -1.0, 1.0])
    T_rob = tip_frame(g["fingers"][0], g["fingers"][1], g["approach"]) @ flip
    p = plotter((2000, 1100))
    Ld = np.linalg.inv(T_dev)
    for n, (m, col) in parts.items():
        add(p, m.copy().apply_transform(Ld), col)
    Lr = trimesh.transformations.translation_matrix([0, 260.0, 0]) @ np.linalg.inv(T_rob)
    for m in list(g["fixed"].values()) + list(g["wrist"].values()):
        add(p, m.copy().apply_transform(Lr), ROBOT_GREY)
    for i, m in enumerate(g["fingers"]):                     # part open, like the device
        d = g["stroke"][0][1] * 0.6 * (-1 if i == 0 else 1)
        add(p, m.copy().apply_translation(d).apply_transform(Lr), GRIPPER_DARK)
    add(p, post_mesh.copy().apply_transform(Lr), POST_BLUE)
    for n, m in cams.items():
        col = dict(parts)[n][1] if n in parts else GRIPPER_DARK
        add(p, m.copy().apply_transform(Lr), col)
    # tip frame: x along the approach (out of the tips), y across the jaws (the robot 260 mm along it), the camera
    # towards -z; look from in front of the tips, a little to the side and above
    c = np.array([-90.0, 130.0, -40.0])
    shot(p, path, tuple(c + np.array([520.0, -330.0, -330.0])), tuple(c), up=(0, 0, -1.0), zoom=1.15, aspect=1.9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("neuracore_robots")
    ap.add_argument("sw_assets")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    from build123d import export_step, export_stl
    args = {"hande": a.neuracore_robots, "piper": a.sw_assets, "openarm": a.sw_assets}
    for d in ("STEP", "STL"):
        (HW_OUT / d / "robot").mkdir(parents=True, exist_ok=True)
    for name, (loader, tsname, robots) in ROBOTS.items():
        tipset = next(ts for ts in TIP_SETS if ts.name == tsname)
        g = loader(args[name])
        post, pm, cams, G, side, clash = solve(name, g, tipset)
        export_step(post, str(HW_OUT / "STEP" / "robot" / f"camera_post_{name}.step"))
        export_stl(post, str(HW_OUT / "STL" / "robot" / f"camera_post_{name}.stl"), tolerance=0.02,
                   angular_tolerance=0.15)
        bb = post.bounding_box()
        print(f"camera_post_{name} ({robots}): {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm, "
              f"{post.volume / 1e3:.1f} cm3, {len(post.solids())} solid(s); camera side {side}, robot points inside: {clash}")
        if not a.no_render:
            side_by_side(name, g, tipset, pm, cams, G, side, IMG / f"robot_camera_{name}.png")


if __name__ == "__main__":
    main()
