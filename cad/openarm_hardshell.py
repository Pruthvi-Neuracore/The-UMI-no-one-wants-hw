"""OpenArm hard-shell fingers: printed fingers for OpenArm v1's parallel gripper, in place of its stock fingers.

Each finger is one hard part and one soft part sharing one outer surface, so the finger reads as one seamless form:
  * the base is OpenArm v1's own finger base (measured from its description): the 5.2 mm mounting plate with its
    four M2.5 holes (counterbored on the finger side) and the keying features behind it (centre block, two prongs,
    rib and the round boss), so it fits the carriage exactly as the stock finger does;
  * from the plate to midway (x = 35) the finger follows the stock finger's own sections, then a smooth loft carries
    it into the jaw, drawn in the PG Chopstick's language: flat facets; in plan, straight sides chamfered to a point;
    in side view, a flat gripping edge and a straight back breaking into a chisel nose; three big scallops along the
    gripping edge;
  * `hardshell_tpu_<side>` (TPU 95A), the soft part: the jaw's lower body below a flat parting line (falling along the
    chisel so the hard spine stays whole), flush with the hard part's faces, with the scallops and a tongue on its
    back that keys into the hard part (glue it in);
  * `hardshell_finger_<side>` (PETG / PLA), the hard part: everything else (base, flow, spine and a TIP_T hard tip),
    its gripping face TIP_BACK behind the soft face so the soft part touches first, with straight access bores for
    the four screw heads.

Finger frame (mm; Open-ENPIRE's OpenArm jaw frame, which places the stock finger where OpenArm puts it): x along the
finger from the carriage, y across, z through the finger, the gripping face at low z. The right finger is the mirror
image in y (as in OpenArm's description). Strength: check_strength.py.

    python openarm_hardshell.py [--openarm /path/to/handumi-sw/assets] [--no-render]
"""
import argparse
from pathlib import Path

import numpy as np
from build123d import (Axis, Box, Cylinder, Location, Plane, Pos, Rot, export_step, export_stl, extrude, fillet,
                       loft, make_face, mirror)

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "hardware"
IMG = HERE.parent / "docs" / "img"

# --- OpenArm v1's finger base (measured from its finger mesh, in this frame) ----------------------------------------
PLATE = dict(x=(1.4, 6.6), y=(-30.5, 30.5), z=(30.2, 60.2), r=2.0)
HOLES = [(7.5, 40.2), (-7.5, 40.2), (7.5, 50.2), (-7.5, 50.2)]     # (y, z), M2.5
HOLE_D, CBORE = 3.0, (4.8, 1.6)                                    # through; counterbore (dia, depth) from the front
CENTRE = dict(x=(-2.9, 1.5), y=(-10.0, 10.0))                      # behind the plate, full plate height
PRONGS = dict(x=(-6.2, -2.8), y=(4.0, 10.0), chamfer=1.0)          # two, mirrored in y
RIB = dict(x=(-6.2, 1.5), y=(10.1, 15.1))
BOSS = dict(yz=(-18.1, 45.2), small=(8.0, (-7.4, -3.8)), big=(13.6, (-3.9, 1.5)), hole=4.2)
# --- the finger: sections (x, y_min, y_max, z_face, z_back); up to x = 35 they are the stock finger's -------------------
STOCK = [(6.4, -30.2, 30.2, 42.3, 59.2), (11, -26.3, 26.7, 41.8, 57.6), (17, -22.1, 24.3, 41.0, 55.4),
         (23, -19.0, 23.3, 40.3, 53.3), (29, -16.9, 21.7, 39.4, 51.1), (35, -15.6, 19.2, 38.4, 48.9)]
GRIP_Z = 24.8              # the soft face: the pads close to a fraction of a millimetre apart
FLOW = [(46, -15.0, 15.0, 31.0, 46.5), (57, -14.6, 14.6, GRIP_Z, 45.0)]   # from the stock sections into the jaw
SECTION_R = 4.0
# the distal jaw, in the PG Chopstick's language: flat facets, a pointed (hexagonal) plan, a chisel nose, big scallops
JAW_X0 = 53.0              # it starts inside the flow's last section
JAW_SIDE = [(JAW_X0, 45.2), (96.0, 41.5), (120.0, None)]  # back line (x, z); the chisel runs from (96, 41.5) to the tip
JAW_PLAN = [(JAW_X0, 14.8), (94.0, 13.5), (120.0, 3.5)]   # half-width (x, y): straight sides, then the point
JAW_EDGE_R = 1.2
SCALLOPS = [(66.0, 7.5, 2.6), (83.0, 7.5, 2.6), (99.0, 6.0, 2.0)]   # (x, radius, depth) along the gripping edge
SOFT_X = (59.0, 120.0 - 1.5)   # the soft part's length (the last 1.5 mm is the hard tip)
SPLIT = [(59.0, 8.0), (92.0, 7.0), (108.0, 3.0), (118.5, 0.9)]   # the parting line above the soft face, along x:
                                                                # it falls with the chisel so the hard spine stays
                                                                # whole all the way into the hard tip
TIP_T, TIP_BACK = 1.5, 0.6
TONGUE = dict(x=(64.0, 104.0), half=4.0, h=2.5)
BORE_D = 5.0
# how far each screw's access bore runs: the upper two until they come out of the sloping back; the lower two as open
# channels under the base's gripping face (a ball-end key reaches them from the gripping side)
BORE_LEN = {50.2: 30.0, 40.2: 20.0}


def _mesh(shape, tol=0.05):
    import trimesh
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def _section(x, y0, y1, z0, z1, r=SECTION_R):
    from build123d import RectangleRounded
    pl = Plane(origin=(x, (y0 + y1) / 2, (z0 + z1) / 2), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return make_face(pl * RectangleRounded(y1 - y0, z1 - z0, min(r, (y1 - y0) / 2 - 0.2, (z1 - z0) / 2 - 0.2)).wire())


def base():
    """OpenArm v1's finger base: the plate (holes added later) and the keying features behind it."""
    (x0, x1), (y0, y1), (z0, z1) = PLATE["x"], PLATE["y"], PLATE["z"]
    zc, h = (z0 + z1) / 2, z1 - z0
    b = fillet((Pos((x0 + x1) / 2, (y0 + y1) / 2, zc) * Box(x1 - x0, y1 - y0, h)).edges().filter_by(Axis.X), PLATE["r"])
    (a0, a1), (c0, c1) = CENTRE["x"], CENTRE["y"]
    b += Pos((a0 + a1) / 2, (c0 + c1) / 2, zc) * Box(a1 - a0, c1 - c0, h)
    (p0, p1), (q0, q1) = PRONGS["x"], PRONGS["y"]
    for s in (1, -1):
        pr = Pos((p0 + p1) / 2, s * (q0 + q1) / 2, zc) * Box(p1 - p0 + 0.1, q1 - q0, h)
        b += pr
    (r0, r1), (t0, t1) = RIB["x"], RIB["y"]
    b += Pos((r0 + r1) / 2, (t0 + t1) / 2, zc) * Box(r1 - r0, t1 - t0, h)
    by, bz = BOSS["yz"]
    for d, (u0, u1) in (BOSS["small"], BOSS["big"]):
        b += Pos((u0 + u1) / 2, by, bz) * Rot(0, 90, 0) * Cylinder(d / 2, u1 - u0)
    b -= Pos(-2.0, by, bz) * Rot(0, 90, 0) * Cylinder(BOSS["hole"] / 2, 20)
    return b


def _poly_prism(pts, axes, lo=-200.0, hi=200.0):
    """Extrude a polygon drawn on two axes (0 = x, 1 = y, 2 = z) along the third, from lo to hi."""
    from build123d import Polyline
    third = ({0, 1, 2} - set(axes)).pop()
    q3 = []
    for a, b in pts:
        q = [0.0, 0.0, 0.0]
        q[axes[0]], q[axes[1]], q[third] = a, b, lo
        q3.append(tuple(q))
    d = [0, 0, 0]
    d[third] = 1
    return extrude(make_face(Polyline(*q3, close=True)), hi - lo, dir=tuple(d))


def jaw():
    """The distal jaw: side outline (flat gripping edge, straight back, chisel nose) and plan outline (straight sides
    to a point), intersected, long edges lightly rounded."""
    (x0, z0), (xc, zc), (xt, _) = JAW_SIDE
    zt = GRIP_Z + TIP_BACK + 0.8                                   # the chisel ends just above the gripping edge
    side = [(x0, GRIP_Z), (xt, GRIP_Z), (xt, zt), (xc, zc), (x0, z0)]
    (p0, h0), (p1, h1), (pt, ht) = JAW_PLAN
    plan = [(p0, -h0), (p1, -h1), (pt, -ht), (pt, ht), (p1, h1), (p0, h0)]
    j = _poly_prism(side, (0, 2)) & _poly_prism(plan, (0, 1))
    for r in (JAW_EDGE_R, 0.8):
        try:
            b = fillet(j.edges().filter_by(Axis.X), r)
            if b.is_valid:
                return b
        except Exception:
            pass
    return j


def finger_body():
    """The whole finger's outer form: the stock flow (a smooth loft from the plate through the stock finger's sections)
    joined to the faceted distal jaw."""
    flow = loft([_section(*s) for s in STOCK + FLOW])
    return flow + jaw()


def split(body):
    """(hard, soft): the soft part is the jaw's lower body below the parting line along SOFT_X, with the scallops and a
    tongue; the hard part is the rest, with the 1.5 mm hard tip (its face TIP_BACK behind the soft face)."""
    (s0, s1) = SOFT_X
    xs, hs = zip(*SPLIT)
    pts = [(s0, GRIP_Z - 10)] + [(x, GRIP_Z + h) for x, h in SPLIT] + [(s1, GRIP_Z - 10)]   # SPLIT ends at s1
    region = _poly_prism(pts, (0, 2))
    soft = body & region
    t = TONGUE
    zt = GRIP_Z + float(np.interp(sum(t["x"]) / 2, xs, hs))
    tongue = Pos(sum(t["x"]) / 2, 0, zt + t["h"] / 2 - 0.05) * Box(t["x"][1] - t["x"][0], 2 * t["half"], t["h"] + 0.1)
    soft = soft + tongue
    hard = body - soft
    for x, r, d in SCALLOPS:
        soft -= Pos(x, 0, GRIP_Z - r + d) * Rot(90, 0, 0) * Cylinder(r, 80)
    hard -= Pos(0, 0, GRIP_Z - 5 + TIP_BACK) * Box(600, 100, 10)       # the hard face sits TIP_BACK behind the soft
    hard = max(hard.solids(), key=lambda q: q.volume) if len(hard.solids()) > 1 else hard
    return hard, soft


def build():
    """(hard, soft)."""
    body = finger_body()
    hard, soft = split(body)
    hard = base() + hard
    (x0, x1) = PLATE["x"]
    for y, z in HOLES:
        hard -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HOLE_D / 2, x1 - x0 + 2)
        L = BORE_LEN[z]                                                # head seat and key access, to where it opens
        hard -= Pos(x1 - CBORE[1] + L / 2, y, z) * Rot(0, 90, 0) * Cylinder(max(CBORE[0], BORE_D) / 2, L)
    hard -= soft                                                       # (the bores never reach the soft part)
    return hard, soft


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
    shell, soft = build()
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
    print(f"hard / soft overlap {(shell & soft).volume if (shell & soft).solids() else 0:.1f} mm3")
    if not a.no_render:
        render(a.openarm, parts)


if __name__ == "__main__":
    main()
