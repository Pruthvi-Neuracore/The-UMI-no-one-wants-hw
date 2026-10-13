"""OpenArm hard-shell fingers: printed fingers for OpenArm v1's parallel gripper, in place of its stock fingers.

Each finger is two prints:
  * `hardshell_finger_<side>` (PETG / PLA): Open-ENPIRE's OpenArm mount (pgeedh/Open-ENPIRE-Gripper: a 5.1 mm plate
    with four M3 holes and the key rib behind it, where the stock finger's base goes), three neck ribs, and a hard
    shell that follows the soft part all round its back and sides at SHELL_T, closing over its end as a 1.5 mm
    **hard tip**.
  * `hardshell_tpu_<side>` (TPU 95A): the soft part, a tapered finger whose contact face has three shallow transverse
    grooves like the PG Chopstick's pad. It is glued into the shell along its whole back and sides.
The screws stay reachable with the two glued together: the shell starts 18 mm in front of the mount plate, and the
neck ribs pass between the screw heads (the upper two are open from above; the lower two take a ball-end hex key
between the ribs).
The tip is hard and soft together: the soft face stands NAIL_BACK proud of the hard tip, so a light grasp is on the soft
part, and a firm one, or a thin object at the very tip, also bears on the 1.5 mm hard tip.

Finger frame (mm; Open-ENPIRE's OpenArm jaw frame): x along the finger from the mount plate's front face, y across the
plate (its height), z through the finger, the gripping face at low z. The right finger is the mirror image in y (as in
OpenArm's description).

    python openarm_hardshell.py [--openarm /path/to/handumi-sw/assets] [--no-render]
"""
import argparse
from pathlib import Path

import numpy as np
from build123d import (Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, extrude, fillet, loft,
                       mirror)

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "hardware"
IMG = HERE.parent / "docs" / "img"

# Open-ENPIRE's OpenArm mount (measured from its OpenArm_Hard jaw)
PLATE = dict(x=(-5.1, 0.0), y=(-30.8, 30.2), z=(30.0, 59.6), r=2.0)
RIB = dict(x=(-18.0, -5.1), y=(9.8, 13.8), z=(30.0, 59.6))
HOLES = [(7.2, 55.7), (7.2, 34.1), (-7.9, 34.0), (-7.9, 55.6)]       # (y, z), M3
HOLE_D, HEAD_D, HEAD_FROM = 3.4, 6.0, -1.5                          # through, screw-head bore from x = HEAD_FROM on
# the finger
GRIP_Z = 25.0              # the soft part's contact face at the tip (the pads just meet there when closed); along the
                           # finger it sits HOOK further back, so the tips pinch first
SHELL_T = 2.5              # the hard shell's wall: it follows the soft part's back and sides all the way, at this thickness
PROUD = 1.5                # the soft face stands this far proud of the shell's rim
NAIL_BACK = 0.6            # the hard tip's face is this far behind the soft face: the soft part touches first
HOOK = 1.5
# the soft part's sections along x: (x, half-height in y, contact face z, back z)
SOFT = [(18.0, 12.0, GRIP_Z + HOOK, 37.0), (60.0, 11.0, GRIP_Z + HOOK, 34.0), (95.0, 9.8, GRIP_Z + 0.6, 32.0),
        (110.0, 8.0, GRIP_Z, 29.5)]
R_SOFT, R_SHELL = 2.0, 3.5         # corner radii of the sections
TIP_T = 1.5                # the hard tip: the shell closes over the soft part's end with this thickness, down to just
                           # behind the soft face (NAIL_BACK), then the soft part starts
GROOVES = [40.0, 62.0, 84.0]       # x of the soft face's transverse grooves (like the chopstick pad's)
GROOVE_R, GROOVE_D = 7.0, 1.2
# the neck: three ribs from the plate to the shell, clear of the four screw heads (Ø5.5 at y = 7.2 / -7.9)
NECK_RIBS = [(-4.2, 4.2), (10.6, 14.6), (-15.6, -11.6)]
NECK_Z = (27.0, 48.0)      # below the upper screws (z 55.7), so those are open from above


def _section(x, half, z0, z1, r=R_SOFT):
    """A rounded rectangle in the plane x = const, across y (half-height) and z (z0..z1)."""
    from build123d import RectangleRounded
    pl = Plane(origin=(x, 0, (z0 + z1) / 2), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return pl * RectangleRounded(2 * half, z1 - z0, min(r, half - 0.1, (z1 - z0) / 2 - 0.1))


def _face_z(x):
    xs, zs = [s[0] for s in SOFT], [s[2] for s in SOFT]
    return float(np.interp(x, xs, zs))


def _mesh(shape, tol=0.05):
    import trimesh
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def build():
    """(shell, soft): the hard shell with its mount and hard tip, and the soft part it wraps."""
    (x0, x1), (y0, y1), (z0, z1) = PLATE["x"], PLATE["y"], PLATE["z"]
    plate = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    plate = fillet(plate.edges().filter_by(Axis.X), PLATE["r"])
    (rx0, rx1), (ry0, ry1), (rz0, rz1) = RIB["x"], RIB["y"], RIB["z"]
    rib = Pos((rx0 + rx1) / 2 + 0.25, (ry0 + ry1) / 2, (rz0 + rz1) / 2) * Box(rx1 - rx0 + 0.5, ry1 - ry0, rz1 - rz0)
    # the soft part: a tapered finger, its contact face flat at GRIP_Z with three shallow grooves
    soft = loft([_section(x, h, zf, zb) for x, h, zf, zb in SOFT])
    for x in GROOVES:
        soft -= Pos(x, 0, _face_z(x) - GROOVE_R + GROOVE_D) * Rot(90, 0, 0) * Cylinder(GROOVE_R, 40)
    # the hard shell: the same sections grown by SHELL_T round the back and sides (and a wall at the base end), open on
    # the gripping side down to the rim
    t = SHELL_T
    x0s, h0, zf0, zb0 = SOFT[0]
    outer = [_section(x0s - t, h0 + t, zf0 + PROUD, zb0 + t, R_SHELL)]
    outer += [_section(x, h + t, zf + PROUD, zb + t, R_SHELL) for x, h, zf, zb in SOFT[1:]]
    shell = loft(outer) - loft([_section(x, h, zf - 5, zb) for x, h, zf, zb in SOFT])
    xe, he, zfe, ze = SOFT[-1]                                                  # the hard tip

    cap = _section(xe, he + t, zfe + NAIL_BACK, ze + t, R_SHELL)
    shell += extrude(cap, TIP_T)
    neck = None
    for a, b in NECK_RIBS:
        xr = SOFT[0][0] - SHELL_T + 1.0                                         # into the shell's base wall only
        r = Pos(xr / 2, (a + b) / 2, sum(NECK_Z) / 2) * Box(xr, b - a, NECK_Z[1] - NECK_Z[0])
        neck = r if neck is None else neck + r
    shell = plate + rib + neck + shell
    for y, z in HOLES:
        shell -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HOLE_D / 2, x1 - x0 + 2)
        shell -= Pos(HEAD_FROM / 2, y, z) * Rot(0, 90, 0) * Cylinder(HEAD_D / 2, -HEAD_FROM + 0.02)   # head seat
    return shell, soft


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
    col = {"finger": "#f2b705", "tpu": "#2b2e33", "link7": "#c9ccd1"}      # hard yellow, soft black

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
    shell, tip = build()
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
    ov = (shell & tip).volume if (shell & tip).solids() else 0.0
    print(f"shell / pad overlap: {ov:.1f} mm3")
    if not a.no_render:
        render(a.openarm, parts)


if __name__ == "__main__":
    main()
