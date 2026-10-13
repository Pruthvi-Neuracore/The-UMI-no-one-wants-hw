"""OpenArm hard-shell fingers: a printed hard shell with the PG Chopstick's soft TPU pad, replacing OpenArm v1's stock
parallel-gripper fingers.

Each finger is two prints:
  * `hardshell_finger_<side>` (PETG / PLA): the OpenArm mount from Open-ENPIRE (pgeedh/Open-ENPIRE-Gripper, the
    `OpenArm_Hard` jaw: a 5.1 mm plate with four M3 holes and the key rib behind it, which sit where the stock finger's
    base does), a three-rib neck, and the hard back of the PG Chopstick jaw (TToTMooN/KIWI_hardware), whose inner face
    carries the soft pad,
  * `hardshell_tpu_<side>` (TPU 95A): the PG Chopstick's TPU tip, unchanged in section, from the jaw's straight part
    to the fingertip. It wraps the hard shell's inner face and runs on past it to form the soft fingertip.
The chopstick jaw closes on a pivot, so its pad is turned until its face is parallel to the finger, and the pad's
contact face is put where Open-ENPIRE's is (finger frame z = 28 mm), so the OpenArm closes on these fingers as it does
on Open-ENPIRE's. The shell's back is cut flat at z = 61 mm (inside Open-ENPIRE's depth, and a flat face to print on).

Finger frame (mm; Open-ENPIRE's OpenArm jaw frame): x along the finger from the mount plate's front face, y across the
plate (its height), z through the finger, the gripping face at low z. The right finger is the mirror image in y (as in
OpenArm's description).

    python openarm_hardshell.py [--openarm /path/to/handumi-sw/assets] [--no-render]
"""
import argparse
import math
from pathlib import Path

import numpy as np
from build123d import (Axis, Box, Cylinder, Location, Plane, Pos, Rot, export_step, export_stl, fillet, import_step,
                       mirror)

HERE = Path(__file__).resolve().parent
SRC = HERE / "source"
OUT = HERE.parent / "hardware"
IMG = HERE.parent / "docs" / "img"

# Open-ENPIRE's OpenArm mount (measured from OpenArm_Hard.stl)
PLATE = dict(x=(-5.1, 0.0), y=(-30.8, 30.2), z=(30.0, 59.6), r=2.0)
RIB = dict(x=(-18.0, -5.1), y=(9.8, 13.8), z=(30.0, 59.6))
HOLES = [(7.2, 55.7), (7.2, 34.1), (-7.9, 34.0), (-7.9, 55.6)]       # (y, z), M3
HOLE_D, HEAD_D, HEAD_FROM = 3.4, 6.0, -1.5                          # through, screw-head bore from x = HEAD_FROM on
# the chopstick jaw and pad (PG Chopstick gripper frame), and where they go
X_CUT = 100.0              # keep the jaw's and pad's straight part, x >= X_CUT
GRIP_Z = 25.0              # the pad's contact face in the finger frame: 3 mm inside Open-ENPIRE's (28), so the pads just meet
BACK_Z = 61.0              # the shell's flat back
X_START = 12.0             # where the jaw part starts along the finger (the neck is 0..X_START)
NECK_RIBS = [(-4.2, 4.2), (10.6, 14.6), (-15.4, -11.4)]             # y spans, clear of the screw-head bores


def _loc(R, t):
    from OCP.gp import gp_Trsf
    tr = gp_Trsf()
    tr.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
    return Location(tr)


def _mesh(shape, tol=0.05):
    import trimesh
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def chopstick_parts():
    """The chopstick's right jaw and TPU tip (gripper frame, closed), cut to their straight part."""
    jaw = import_step(str(SRC / "jaw_right.step"))
    tip = import_step(str(SRC / "tip_tpu_right.step"))
    keep = Pos(X_CUT + 100, 0, 0) * Box(200, 200, 200)
    return jaw & keep, tip & keep


def placement(tip):
    """Rotation + translation (3 x 4) taking the chopstick gripper frame to the finger frame: the pad's face turned
    parallel to the finger, its contact face on z = GRIP_Z, its straight part starting at x = X_START, centred in y."""
    m = _mesh(tip)
    xs, ys = [], []
    for x in np.arange(X_CUT + 15, m.bounds[1][0] - 15, 5.0):
        s = m.section([1, 0, 0], [x, 0, 0])
        if s is not None:
            xs.append(x)
            ys.append(s.vertices[:, 1].max())
    k = np.polyfit(xs, ys, 1)[0]
    th = -math.atan(k)                                  # turn the pad's face parallel to x
    Rz = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1.0]])
    M = np.array([[1, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # chopstick (x, y, z) -> finger (x, z, -y)
    R = M @ Rz
    v = m.vertices @ R.T
    face = v[v[:, 0] > X_CUT + 15][:, 2].min()          # the contact face (lowest z on the straight part)
    t = np.array([X_START - v[:, 0].min(), -(v[:, 1].min() + v[:, 1].max()) / 2, GRIP_Z - face])
    return R, t, math.degrees(th)


def build():
    jaw, tip = chopstick_parts()
    R, t, th = placement(tip)
    L = _loc(R, t)
    jaw, tip = jaw.moved(L), tip.moved(L)
    (x0, x1), (y0, y1), (z0, z1) = PLATE["x"], PLATE["y"], PLATE["z"]
    plate = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    plate = fillet(plate.edges().filter_by(Axis.X), PLATE["r"])
    (rx0, rx1), (ry0, ry1), (rz0, rz1) = RIB["x"], RIB["y"], RIB["z"]
    rib = Pos((rx0 + rx1) / 2 + 0.25, (ry0 + ry1) / 2, (rz0 + rz1) / 2) * Box(rx1 - rx0 + 0.5, ry1 - ry0, rz1 - rz0)
    # the neck: three ribs from the plate into the jaw's back, behind the pad
    jb = jaw.bounding_box()
    zr0 = max(z0, GRIP_Z + 9.0)
    neck = None
    for a, b in NECK_RIBS:
        r = Pos(X_START / 2 + 4.0, (a + b) / 2, (zr0 + z1) / 2) * Box(X_START + 8.0, b - a, z1 - zr0)
        neck = r if neck is None else neck + r
    shell = plate + rib + neck + jaw
    shell &= Pos(0, 0, BACK_Z - 100) * Box(600, 600, 200)                       # flat back
    shell -= tip                                                                # the pad's seat, exact
    for y, z in HOLES:
        shell -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HOLE_D / 2, x1 - x0 + 2)
        shell -= Pos(HEAD_FROM + 100, y, z) * Rot(0, 90, 0) * Cylinder(HEAD_D / 2, 200)   # head + driver bore
    return shell, tip, th


def on_openarm(assets, meshes, q):
    """{name: trimesh in link7's frame (mm)}: OpenArm v1's right link7 and both fingers at finger-joint value q (m),
    placed as HandUMI's robot render places Open-ENPIRE's OpenArm jaws (same frame as this finger)."""
    import re
    import trimesh
    import yourdfpy
    assets = Path(assets)
    path = assets / "openarm/urdf/openarm_v1.urdf"
    pk = assets / "openarm/openarm_description"

    def handler(fname):
        m = re.match(r"package://([^/]+)/(.*)", fname)
        return str(pk / m.group(2)) if m else str((path.parent / fname).resolve())
    r = yourdfpy.URDF.load(str(path), filename_handler=handler, load_meshes=True, build_scene_graph=True)
    joints = [j for j in r.actuated_joint_names if "right_finger" in j]
    r.update_cfg({j: q for j in joints})
    root = "openarm_right_link7"
    T0 = np.linalg.inv(r.get_transform(root, r.base_link))
    out = {}
    for v in r.link_map[root].visuals:
        g = v.geometry.mesh
        m = trimesh.load(handler(g.filename), force="mesh")
        S = np.diag([*(np.broadcast_to(g.scale, 3) if g.scale is not None else (1, 1, 1)), 1.0])
        m.apply_transform(T0 @ r.get_transform(root, r.base_link) @ (v.origin if v.origin is not None else np.eye(4)) @ S)
        out["link7"] = m.apply_scale(1000.0)
    Mj = np.array([[0, 1, 0, 0.0], [0, 0, 1, 17.3], [1, 0, 0, 676.0], [0, 0, 0, 1]], float)
    for link in (root.replace("link7", "left_finger"), root.replace("link7", "right_finger")):
        side = "right" if link.endswith("right_finger") else "left"
        sy = -1.0 if side == "right" else 1.0
        S = np.diag([0.001, 0.001 * sy, 0.001, 1.0])
        S[:3, 3] = [0.0, -0.05 * sy, -0.673001]
        T = T0 @ r.get_transform(link, r.base_link) @ S @ Mj
        for n in ("finger", "tpu"):
            m = meshes[f"hardshell_{n}_left"].copy().apply_transform(T)       # the URDF mirrors the right one
            m.apply_scale(1000.0)
            if sy < 0:
                m.invert()
            out[f"{n}_{side}"] = m
    lim = r.joint_map[joints[0]].limit
    return out, (lim.lower, lim.upper)


def render(assets, parts):
    import pyvista as pv
    import trimesh
    meshes = {n: _mesh(p, 0.03) for n, p in parts.items()}
    IMG.mkdir(parents=True, exist_ok=True)
    col = {"finger": "#1f5fae", "tpu": "#e9edf1", "link7": "#c9ccd1"}

    def plot(path, items, view, up=(0, 0, 1), size=(1600, 1100), zoom=1.2):
        p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
        p.set_background("white")
        p.enable_anti_aliasing("ssaa")
        for m, c in items:
            p.add_mesh(pv.wrap(m), color=c, smooth_shading=True, split_sharp_edges=True, specular=0.3)
        p.view_vector(view, viewup=up)
        p.reset_camera()
        p.camera.zoom(zoom)
        p.screenshot(str(path))
        p.close()
        print("wrote", path.name)
    # the two prints, side by side, and the finger assembled
    sh, pad = meshes["hardshell_finger_left"], meshes["hardshell_tpu_left"]
    a = [(sh, col["finger"]), (pad.copy().apply_translation([0, 75.0, 0]), col["tpu"]),
         (sh.copy().apply_translation([0, 150.0, 0]), col["finger"]),
         (pad.copy().apply_translation([0, 150.0, 0]), col["tpu"])]
    plot(IMG / "finger.png", a, (0.35, -0.25, -1.0), up=(1, 0, 0), size=(1800, 1000), zoom=1.15)
    if not assets:
        return
    for name, frac in (("openarm_open", 0.5), ("openarm_closed", 0.0)):
        o, (lo, hi) = on_openarm(assets, meshes, 0.0)
        o, _ = on_openarm(assets, meshes, lo + frac * (hi - lo))
        items = [(o["link7"], col["link7"])] + [(o[k], col[k.split("_")[0]]) for k in o if k != "link7"]
        plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        if frac == 0.0:
            gap = trimesh.proximity.closest_point(o["tpu_left"], o["tpu_right"].vertices)[1].min()
            print(f"  closed (joint {lo:.3f} m): pads {gap:.1f} mm apart at the nearest point")
        else:
            gap = trimesh.proximity.closest_point(o["tpu_left"], o["tpu_right"].vertices)[1].min()
            print(f"  half open: pads {gap:.1f} mm apart (fully open, {hi:.3f} m per finger: about 89 mm)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the render on OpenArm's link7)")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    shell, tip, th = build()
    parts = {"hardshell_finger_left": shell, "hardshell_tpu_left": tip,
             "hardshell_finger_right": mirror(shell, Plane.XZ), "hardshell_tpu_right": mirror(tip, Plane.XZ)}
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in parts.items():
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    print(f"pad turned {th:.1f} deg to lie parallel to the finger")
    ov = (shell & tip).volume if (shell & tip).solids() else 0.0
    print(f"shell / pad overlap: {ov:.1f} mm3")
    if not a.no_render:
        render(a.openarm, parts)


if __name__ == "__main__":
    main()
