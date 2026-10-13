"""OpenArm hard-shell fingers: printed fingers for OpenArm v1's parallel gripper, in place of its stock fingers.

One soft part and one hard part per finger, as in Open-ENPIRE:
  * `hardshell_tpu_<side>` (TPU 95A), the soft part: the PG Chopstick's TPU pad (from TToTMooN/KIWI_hardware), its
    straight part, turned so its face lies parallel to the finger.
  * `hardshell_finger_<side>` (PETG / PLA), the hard part:
      - Open-ENPIRE's OpenArm mount (pgeedh/Open-ENPIRE-Gripper): a 5.1 mm plate with four M3 holes and the key rib
        behind it, where the stock finger's base goes;
      - three ribs from the plate to the holder, between the screw heads;
      - the holder: the soft part's side outline grown by SHELL_T, across a constant width SHELL_T wider than the soft
        part each side, edges rounded, with the soft part's exact shape cut into it. The soft face stands PROUD of
        its rim; past the soft part's end a TIP_T hard tip.
The soft part drops into the holder and is glued in. The holder starts NECK_L in front of the plate, so the screws
stay reachable with the finger glued: the upper two from above, the lower two with a ball-end hex key between the
ribs.

Finger frame (mm; Open-ENPIRE's OpenArm jaw frame): x along the finger from the mount plate's front face, y across the
plate (its height), z through the finger, the gripping face at low z. The right finger is the mirror image in y (as in
OpenArm's description).

    python openarm_hardshell.py [--openarm /path/to/handumi-sw/assets] [--no-render]
"""
import argparse
import math
from pathlib import Path

import numpy as np
from build123d import (Axis, Box, Cylinder, Location, Plane, Polyline, Pos, Rot, export_step, export_stl, extrude,
                       fillet, import_step, make_face, mirror)

HERE = Path(__file__).resolve().parent
SRC = HERE / "source"
OUT = HERE.parent / "hardware"
IMG = HERE.parent / "docs" / "img"

# Open-ENPIRE's OpenArm mount (measured from its OpenArm_Hard jaw)
PLATE = dict(x=(-5.1, 0.0), y=(-30.8, 30.2), z=(30.0, 59.6), r=2.0)
RIB = dict(x=(-18.0, -5.1), y=(9.8, 13.8), z=(30.0, 59.6))
HOLES = [(7.2, 55.7), (7.2, 34.1), (-7.9, 34.0), (-7.9, 55.6)]       # (y, z), M3
HOLE_D, HEAD_D, HEAD_SEAT = 3.4, 6.0, 1.5                           # through hole; screw-head seat in the plate
# the soft part: the chopstick pad's straight part, placed in the finger frame
X_CUT = 100.0              # keep the chopstick pad from x >= X_CUT (its straight part)
GRIP_Z = 25.0              # its contact face (3 mm inside Open-ENPIRE's 28, so the pads just meet when closed)
NECK_L = 18.0              # the soft part starts this far in front of the plate (the screws are reached in between)
# the hard holder
SHELL_T = 2.5              # wall round the soft part's back and sides
PROUD = 1.5                # the soft face stands this far proud of the shell's rim
TIP_T = 1.5                # the hard tip over the soft part's end
TIP_BACK = 0.6             # the hard tip's face is this far behind the soft face: the soft part touches first


def _loc(R, t):
    from OCP.gp import gp_Trsf
    tr = gp_Trsf()
    tr.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
    return Location(tr)


def _mesh(shape, tol=0.05):
    import trimesh
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def soft_part():
    """The chopstick pad's straight part in the finger frame: its face turned parallel to the finger (the chopstick
    closes on a pivot, so there it is angled), on z = GRIP_Z, starting NECK_L in front of the plate, centred in y."""
    tip = import_step(str(SRC / "tip_tpu_right.step")) & (Pos(X_CUT + 100, 0, 0) * Box(200, 200, 200))
    m = _mesh(tip)
    xs, ys = [], []
    for x in np.arange(X_CUT + 15, m.bounds[1][0] - 15, 5.0):
        s = m.section([1, 0, 0], [x, 0, 0])
        if s is not None:
            xs.append(x)
            ys.append(s.vertices[:, 1].max())
    th = -math.atan(np.polyfit(xs, ys, 1)[0])
    Rz = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1.0]])
    M = np.array([[1, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # chopstick (x, y, z) -> finger (x, z, -y)
    R = M @ Rz
    v = m.vertices @ R.T
    face = v[v[:, 0] > X_CUT + 15][:, 2].min()
    t = np.array([NECK_L - v[:, 0].min(), -(v[:, 1].min() + v[:, 1].max()) / 2, GRIP_Z - face])
    return tip.moved(_loc(R, t)), math.degrees(th)


N_SECTIONS, N_POINTS = 30, 96


def _ring(poly, x, scale=1.0, centre=None):
    """A closed spline in the plane x through `poly`'s outline (optionally scaled about `centre`), resampled to
    N_POINTS from the top of its back (max z), the same way round, so successive sections loft cleanly."""
    from build123d import Spline, Wire
    from shapely.geometry.polygon import orient
    ring = orient(poly, 1.0).exterior
    pts = np.array([ring.interpolate(d).coords[0] for d in np.linspace(0, ring.length, 600, endpoint=False)])
    if centre is not None:
        pts = centre + (pts - centre) * scale
    k = np.argmax(pts[:, 1] - 0.05 * np.abs(pts[:, 0] - pts[:, 0].mean()))
    pts = np.roll(pts, -k, axis=0)
    seg = np.linalg.norm(np.diff(np.vstack([pts, pts[:1]]), axis=0), axis=1)
    d = np.r_[0, np.cumsum(seg)]
    s = np.linspace(0, d[-1], N_POINTS, endpoint=False)
    ys = np.interp(s, d, np.r_[pts[:, 0], pts[0, 0]])
    zs = np.interp(s, d, np.r_[pts[:, 1], pts[0, 1]])
    return Wire([Spline(*[(x, y, z) for y, z in zip(ys, zs)], periodic=True)])


def _soft_sections(m):
    """The soft part's cross-sections (x, polygon in (y, z)) along its length."""
    from shapely.geometry import Polygon
    x0, x1 = m.bounds[0][0], m.bounds[1][0]
    out = []
    for x in np.linspace(x0 + 0.3, x1 - 0.3, N_SECTIONS):
        sec = m.section([1, 0, 0], [x, 0, 0])
        loop = max(sec.discrete, key=lambda l: Polygon(l[:, 1:]).area)[:, 1:]
        out.append((x, Polygon(loop).buffer(0)))
    return out


def _rim(m):
    """The rim along x: PROUD behind the soft face, following its deepest points so it never covers the face,
    smoothed; over the hard tip, TIP_BACK behind the face."""
    xs, xe = m.bounds[0][0], m.bounds[1][0]
    grid = np.arange(xs - SHELL_T - 1, xe + TIP_T + 1.01, 1.0)
    face = np.array([m.section([1, 0, 0], [float(np.clip(x, xs + 0.3, xe - 0.3)), 0, 0]).vertices[:, 2].min()
                     for x in grid])
    face = np.array([face[max(0, i - 7):i + 8].max() for i in range(len(face))])
    face = np.convolve(np.pad(face, 4, mode="edge"), np.ones(9) / 9, mode="valid")
    return grid, np.array([f + (TIP_BACK if x > xe - 0.5 else PROUD) for x, f in zip(grid, face)])


def holder(soft):
    """The hard shell: every cross-section of the soft part grown by SHELL_T and lofted, closed at the base and over
    the tip (TIP_T past the soft part's end), the gripping side trimmed to the rim; and the solid base from the mount
    plate to the shell. (The soft part is cut out in build().)"""
    from build123d import Polyline, loft, make_face
    from shapely.geometry import Polygon
    m = _mesh(soft, 0.03)
    grown = [(x, p.buffer(SHELL_T, join_style=1)) for x, p in _soft_sections(m)]
    xb, pb = grown[0]
    xe, pe = grown[-1]
    c = np.array(pe.centroid.coords[0])
    wires = [_ring(pb, xb - 0.3 - SHELL_T)] + [_ring(p, x) for x, p in grown] + \
            [_ring(pe, xe + 0.3 + TIP_T, 0.8, c)]
    body = loft([make_face(w) for w in wires])
    grid, rim = _rim(m)
    keep = Polygon(list(zip(grid, rim)) + [(grid[-1], 200), (grid[0], 200)]).buffer(0).simplify(0.1)
    pts = [(x, -60.0, z) for x, z in list(keep.exterior.coords)[:-1]]
    body &= extrude(make_face(Polyline(*pts, close=True)), 120.0, dir=(0, 1, 0))
    # the solid base: a loft from the plate's front face (nearly its full size) into the shell's base section, so the
    # finger grows out of the plate as one solid
    from shapely.geometry import box as sbox
    (py0, py1), (pz0, pz1) = PLATE["y"], PLATE["z"]
    face0 = sbox(py0 + 3.0, max(pz0, float(rim[0])), py1 - 3.0, pz1 - 1.0).buffer(-3.0).buffer(3.0, join_style=1)
    x_join = xb - 0.3 - SHELL_T
    base = loft([make_face(_ring(face0, 0.0)), make_face(_ring(pb, x_join + 0.8))])
    return body + base


def build():
    """(shell, soft, turn)."""
    soft, th = soft_part()
    xb = soft.bounding_box().min.X
    shell = holder(soft)
    # the mount
    (x0, x1), (y0, y1), (z0, z1) = PLATE["x"], PLATE["y"], PLATE["z"]
    plate = Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)
    plate = fillet(plate.edges().filter_by(Axis.X), PLATE["r"])
    (rx0, rx1), (ry0, ry1), (rz0, rz1) = RIB["x"], RIB["y"], RIB["z"]
    rib = Pos((rx0 + rx1) / 2 + 0.25, (ry0 + ry1) / 2, (rz0 + rz1) / 2) * Box(rx1 - rx0 + 0.5, ry1 - ry0, rz1 - rz0)
    shell = plate + rib + shell - soft
    xc = xb + 0.5                     # the screw-head bores run through the solid base (the lower two into the seat)
    for y, z in HOLES:
        shell -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HOLE_D / 2, x1 - x0 + 2)
        shell -= Pos((xc - HEAD_SEAT) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HEAD_D / 2, xc + HEAD_SEAT)
    return shell, soft, th


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
            m.fix_normals()
            out[f"{n}_{side}"] = m
    lim = r.joint_map[joints[0]].limit
    return out, (lim.lower, lim.upper)


COLOURS = {"finger": "#e9edf1", "tpu": "#2b2e33", "link7": "#c9ccd1"}     # white hard shell, black soft part


def render(assets, parts):
    import pyvista as pv
    import trimesh
    meshes = {n: _mesh(p, 0.03) for n, p in parts.items()}
    IMG.mkdir(parents=True, exist_ok=True)

    def plot(path, items, view, up=(0, 0, 1), size=(1600, 1100), zoom=1.2):
        p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
        p.set_background("white")
        p.enable_anti_aliasing("ssaa")
        p.enable_ssao(radius=4.0, bias=0.2, kernel_size=128, blur=True)
        for m, c in items:
            p.add_mesh(pv.wrap(m), color=c, smooth_shading=True, split_sharp_edges=True, specular=0.35,
                       specular_power=30)
        p.view_vector(view, viewup=up)
        p.reset_camera()
        p.camera.zoom(zoom)
        p.screenshot(str(path))
        p.close()
        print("wrote", path.name)
    sh, pad = meshes["hardshell_finger_left"], meshes["hardshell_tpu_left"]
    a = [(sh, COLOURS["finger"]), (pad.copy().apply_translation([0, 75.0, 0]), COLOURS["tpu"]),
         (sh.copy().apply_translation([0, 150.0, 0]), COLOURS["finger"]),
         (pad.copy().apply_translation([0, 150.0, 0]), COLOURS["tpu"])]
    plot(IMG / "finger.png", a, (0.35, -0.25, -1.0), up=(1, 0, 0), size=(1800, 1000), zoom=1.15)
    plot(IMG / "finger_side.png", [(sh, COLOURS["finger"]), (pad, COLOURS["tpu"])], (0.15, -1.0, 0.25),
         up=(0, 0, -1), size=(1800, 900), zoom=1.2)
    if not assets:
        return
    for name, frac in (("openarm_open", 0.5), ("openarm_closed", 0.0)):
        o, (lo, hi) = on_openarm(assets, meshes, 0.0)
        o, _ = on_openarm(assets, meshes, lo + frac * (hi - lo))
        items = [(o["link7"], COLOURS["link7"])] + [(o[k], COLOURS[k.split("_")[0]]) for k in o if k != "link7"]
        plot(IMG / f"{name}.png", items, (1.0, -0.55, 0.3), zoom=1.0)
        gap = trimesh.proximity.closest_point(o["tpu_left"], o["tpu_right"].vertices)[1].min()
        hits = [round(o[a].intersection(o[b]).volume, 1) for a, b in
                (("finger_left", "finger_right"), ("tpu_left", "tpu_right"), ("finger_left", "tpu_right"))]
        print(f"  {name}: soft parts {gap:.1f} mm apart; overlaps shell/shell, soft/soft, shell/soft: {hits} mm3")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the renders and checks on OpenArm's link7)")
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    shell, soft, th = build()
    parts = {"hardshell_finger_left": shell, "hardshell_tpu_left": soft,
             "hardshell_finger_right": mirror(shell, Plane.XZ), "hardshell_tpu_right": mirror(soft, Plane.XZ)}
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    for n, p in parts.items():
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    print(f"pad turned {th:.1f} deg; shell / soft overlap {(shell & soft).volume if (shell & soft).solids() else 0:.1f} mm3")
    if not a.no_render:
        render(a.openarm, parts)


if __name__ == "__main__":
    main()
