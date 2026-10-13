"""OpenArm spoon fingers: a hard/soft finger pair for OpenArm v1 that holds a large spoon (or fork) by its flat handle
without slipping or turning.

The spoon (spoon 50 cm): a flat handle 22-43 mm wide and 7.7 mm thick nearly all along, rounded edges. Picked from
above, the fingers close across the handle's width, edge to edge. The spoon's weight, with its centre of mass about
100 mm out from the grasp, then tries to turn the spoon about the closing axis; flat pads resist that only by friction,
so the spoon tips. These fingers hold it by **form closure**: each fingertip has a TPU-lined **V-groove** that takes
the handle's edge. When the spoon tries to tip, the handle bears on the groove's flanks across the whole finger width,
so the moment is carried by the flanks' normal force instead of by friction.

  * base and flow: OpenArm v1's own finger base and the stock finger's sections up to midway (openarm_hardshell.py);
  * the jaw: widening to 2 x JAW_HALF (40 mm), so the flanks have a long lever against tipping;
  * the V-groove: 90 deg, V_DEPTH deep, round-bottomed. A rounded edge of any thickness from about 3 to 10 mm seats
    where it touches both flanks, and the 14 mm mouth catches it several millimetres off centre and guides it in as
    the fingers close;
  * the spatula lip below the groove: LIP + liner at the face, its corner chamfered, thickening towards its root, so it
    slides under a handle raised off the table;
  * the soft part (TPU 95A): the groove's liner and the gripping face above it, in one piece, with fine ribs in the
    liner against sliding along the handle, keyed into the hard part.

Finger frame as in openarm_hardshell.py (x along the finger, y across it, z through it, the gripping face at low z).

    python openarm_spoon.py [--openarm /path/to/handumi-sw/assets] [--spoon /path/to/spoon.stl] [--mass 0.5]
"""
import argparse
import math
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Plane, Pos, Rot, export_step, export_stl, fillet, loft, mirror

import openarm_hardshell as H

HANDLE_R = None            # the handle edge's radius for the grasp placement (None: the spoon's, 7.7 / 2)

OUT = H.OUT / "spoon"
IMG = H.IMG
FACE_Z = 24.8              # the jaw's face (as the chopstick finger)
# the V-groove: across the finger, opening at the face, 90 deg, round-bottomed; lined with TPU
V_HALF = 45.0              # half-angle of the V (deg)
V_DEPTH = 7.0              # apex depth from the face, inside the liner
V_APEX_R = 0.8
LINER = 1.2
RIB = (0.5, 1.0, 3.0)      # ribs in the liner's face: depth, gap width, pitch (along the handle, across the finger)
LIP = 2.5                  # the spatula lip at the tip, at the face (it thickens towards its root)
TIP_CHAMFER = 1.2          # the lip's leading corner, chamfered so it slides under a raised handle
JAW_HALF = 20.0            # 40 mm wide: more lever against tipping
JAW_X0, JAW_X1 = 53.0, 70.0  # the jaw widens from the flow's width to JAW_HALF over this length
JAW_BACK = (45.8, 41.0)    # the back's height at JAW_X0 and at the tip
SOFT_X0 = 60.0
SOFT_T = 5.0               # the soft face's thickness above the groove


def groove_x():
    """The groove's centre and the tip's x: the lower flank meets the face LIP + LINER short of the tip."""
    mouth = V_DEPTH * math.tan(math.radians(V_HALF))           # half-height of the mouth (inside the liner)
    tip = 112.0
    xc = tip - LIP - LINER - mouth
    return xc, mouth, tip


def seat_depth(r):
    """How deep (from the face) a rounded edge of radius r sits in the V: where it touches both flanks."""
    a = math.radians(V_HALF)
    return V_DEPTH - r / math.sin(a) + r


def vgroove(clear=0.0):
    """The groove (the open space for the handle), grown by `clear`: a V across the full width, opening at the face,
    apex rounded."""
    from build123d import Polyline, make_face, extrude
    xc, mouth, tip = groove_x()
    a = math.radians(V_HALF)
    d = V_DEPTH + clear / math.sin(a)                          # the grown V's apex depth
    m = d * math.tan(a) + 2.0                                   # extend past the face
    zf = FACE_Z - 2.0 / math.tan(a)
    pts = [(xc - m, zf), (xc, FACE_Z + d), (xc + m, zf)]
    q3 = [(x, -60.0, z) for x, z in pts]
    v = extrude(make_face(Polyline(*q3, close=True)), 120.0, dir=(0, 1, 0))
    try:
        v = fillet(v.edges().filter_by(Axis.Y).sort_by(Axis.Z)[-1:], V_APEX_R + clear)
    except Exception:
        pass
    return v


def jaw():
    """The wide jaw: widens from the flow to JAW_HALF, straight to a square tip; the tip's lip bevelled underneath."""
    xc, mouth, tip = groove_x()
    z0, z1 = JAW_BACK
    side = [(JAW_X0, FACE_Z), (tip - 0.01, FACE_Z), (tip, FACE_Z + 0.01), (tip, z1), (JAW_X0, z0)]
    plan = [(JAW_X0, -15.2), (JAW_X1, -JAW_HALF), (tip, -JAW_HALF), (tip, JAW_HALF), (JAW_X1, JAW_HALF), (JAW_X0, 15.2)]
    j = H._poly_prism(side, (0, 2)) & H._poly_prism(plan, (0, 1))
    for r in (2.0, 1.2):
        try:
            bb = fillet(j.edges().filter_by(Axis.X), r)
            if bb.is_valid:
                j = bb
                break
        except Exception:
            pass
    return j


def build():
    """(hard, soft)."""
    flow = loft([H._section(*s) for s in H.STOCK + [(46, -15.2, 15.2, 29.5, 47.6), (57, -15.2, 15.2, FACE_Z, 45.8)]])
    body = flow + jaw()
    xc, mouth, tip = groove_x()
    # the soft part: the liner in the V and the face above it, one piece; ribs in the liner's face
    liner = (vgroove(LINER) - vgroove(0.0)) & body
    lo = xc - mouth - LINER
    face = Pos((SOFT_X0 + lo) / 2 + 0.5, 0, FACE_Z + SOFT_T / 2) * Box(lo - SOFT_X0 + 1.0, 2 * JAW_HALF + 2, SOFT_T)
    soft = (liner + face) & body
    depth, gap, pitch = RIB
    skin = vgroove(depth) - vgroove(0.0)
    for y in np.arange(-JAW_HALF + pitch / 2, JAW_HALF, pitch):
        soft -= skin & (Pos(xc, y, FACE_Z + 5) * Box(40, gap, 30))
    tongue = Pos((SOFT_X0 + 6 + lo - 6) / 2, 0, FACE_Z + SOFT_T + 1.2) * Box(lo - SOFT_X0 - 12, 8.0, 2.5)
    soft = soft + tongue
    hard = body - (liner + face + tongue) - vgroove(0.0)
    # the spatula lip: chamfer its leading corner so the edge that slides under the handle is thin
    hard -= Pos(tip, 0, FACE_Z) * Rot(0, 45, 0) * Box(TIP_CHAMFER * math.sqrt(2), 120, TIP_CHAMFER * math.sqrt(2))
    hard = H.base() + hard
    (x0, x1) = H.PLATE["x"]
    for y, z in H.HOLES:
        hard -= Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(H.HOLE_D / 2, x1 - x0 + 2)
        L = H.BORE_LEN[z]
        hard -= Pos(x1 - H.CBORE[1] + L / 2, y, z) * Rot(0, 90, 0) * Cylinder(max(H.CBORE[0], H.BORE_D) / 2, L)
    hard -= Pos(0, 0, FACE_Z - 5 + H.TIP_BACK) * Box(600, 100, 10)    # hard faces just behind the soft face
    if len(hard.solids()) > 1:
        hard = max(hard.solids(), key=lambda q: q.volume)
    soft -= vgroove(0.0)
    return hard, soft


def finger_transforms(assets, q):
    """{side: 4x4 finger frame (mm) -> OpenArm right link7 frame (mm)} at finger-joint value q (m)."""
    import re
    import yourdfpy
    assets = Path(assets)
    path = assets / "openarm/urdf/openarm_v1.urdf"
    pk = assets / "openarm/openarm_description"

    def handler(fname):
        m = re.match(r"package://([^/]+)/(.*)", fname)
        return str(pk / m.group(2)) if m else str((path.parent / fname).resolve())
    r = yourdfpy.URDF.load(str(path), filename_handler=handler, load_meshes=False, build_scene_graph=True)
    r.update_cfg({j: q for j in r.actuated_joint_names if "right_finger" in j})
    root = "openarm_right_link7"
    T0 = np.linalg.inv(r.get_transform(root, r.base_link))
    Mj = np.array([[0, 1, 0, 0.0], [0, 0, 1, 17.3], [1, 0, 0, 676.0], [0, 0, 0, 1]], float)
    out = {}
    for side in ("left", "right"):
        sy = -1.0 if side == "right" else 1.0
        S = np.diag([0.001, 0.001 * sy, 0.001, 1.0])
        S[:3, 3] = [0.0, -0.05 * sy, -0.673001]
        T = T0 @ r.get_transform(root.replace("link7", f"{side}_finger"), r.base_link) @ S @ Mj
        out[side] = np.diag([1000.0, 1000.0, 1000.0, 1.0]) @ T
    return out


def spoon_frame(stl):
    """The spoon as a mesh in its own frame: x along it (handle end at 0, bowl at +x), y across, z through, centred."""
    import trimesh
    m = trimesh.load(stl)
    T, _ = trimesh.bounds.oriented_bounds(m)
    m.apply_transform(T)
    order = np.argsort(m.extents)[::-1]
    P = np.eye(4)
    P[:3, :3] = np.eye(3)[order]
    if np.linalg.det(P[:3, :3]) < 0:
        P[2, :3] *= -1
    m.apply_transform(P)
    lo, hi = m.bounds[:, 0]
    w = lambda x: np.ptp(m.section([1, 0, 0], [x, 0, 0]).vertices[:, 1])
    if w(lo + 0.15 * (hi - lo)) > w(hi - 0.15 * (hi - lo)):
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 0, 1]))
    m.apply_translation([-m.bounds[0][0], -(m.bounds[0][1] + m.bounds[1][1]) / 2, 0])
    return m


def grasp(assets, spoon, x_grip, preload=0.3):
    """Place the spoon in the gripper, its handle's edges seated in the slots at handle station x_grip. Returns
    (finger transforms, spoon in link7 frame, finger-joint value q, handle width there)."""
    s = spoon.section([1, 0, 0], [x_grip, 0, 0]).vertices
    width = np.ptp(s[:, 1])
    yc, zc = (s[:, 1].min() + s[:, 1].max()) / 2, (s[:, 2].min() + s[:, 2].max()) / 2
    r = HANDLE_R if HANDLE_R else 3.85
    q = (width - preload - 0.2 - 2 * seat_depth(r)) / 2000.0
    T = finger_transforms(assets, q)
    xs, _, _ = groove_x()
    bottom = {k: (T[k] @ [xs, 0.0, FACE_Z + seat_depth(r), 1])[:3] for k in T}
    c = (bottom["left"] + bottom["right"]) / 2
    yv = bottom["right"] - bottom["left"]
    yv /= np.linalg.norm(yv)                                   # spoon y: across the handle, from left to right finger
    zv = T["left"][:3, 0]                                      # spoon z: the handle's thickness, along the approach
    zv -= zv.dot(yv) * yv
    zv /= np.linalg.norm(zv)
    xv = np.cross(yv, zv)                                      # spoon x: along the handle (across the fingers)
    R = np.column_stack([xv, yv, zv])
    G = np.eye(4)
    G[:3, :3] = R
    G[:3, 3] = c - R @ [x_grip, yc, zc]
    return T, spoon.copy().apply_transform(G), q, width, G


def spoon_check(assets, stl, mass, x_grip):
    """Fit, form closure and load for the spoon held at x_grip."""
    import trimesh
    from openarm_hardshell import _mesh
    hard, soft = build()
    hm, sm = _mesh(hard, 0.05), _mesh(soft, 0.05)
    spoon = spoon_frame(stl)
    T, sp, q, width, G = grasp(assets, spoon, x_grip)
    parts = {f"{n}_{k}": m.copy().apply_transform(T[k]) for k in T for n, m in (("hard", hm), ("soft", sm))}
    print(f"grasp at {x_grip:.0f} mm along the handle: handle {width:.1f} mm wide; finger joint {q * 1000:.1f} mm "
          f"(of 44)")
    for n, m in parts.items():
        v = m.intersection(sp).volume if m.is_volume else float("nan")
        print(f"  spoon overlap with {n}: {v:.2f} mm3 (the liner's preload)" if "soft" in n else
              f"  spoon overlap with {n}: {v:.2f} mm3")
    # form closure: tilt the spoon about the closing axis (its y) by +-3 deg about the grasp point
    cg = G[:3, 3] + G[:3, :3] @ spoon.section([1, 0, 0], [x_grip, 0, 0]).vertices.mean(0)
    yv = G[:3, 1]
    for a in (-3.0, 3.0):
        R = trimesh.transformations.rotation_matrix(math.radians(a), yv, cg)
        st = sp.copy().apply_transform(R)
        hit = sum(m.intersection(st).volume for m in parts.values())
        print(f"  tilted {a:+.0f} deg about the closing axis: {hit:.0f} mm3 into the fingers (blocked if > 0)")
    # the load: weight times its lever from the grasp, carried as a couple by the slot lips over the finger width
    com = spoon.center_mass
    lever = abs(com[0] - x_grip) / 1000.0
    M = mass * 9.81 * lever
    span = 2 * JAW_HALF / 1000.0
    print(f"  {mass:.2f} kg spoon, centre of mass {lever * 1000:.0f} mm from the grasp: {M:.2f} N m; the lips carry it "
          f"as +-{M / span:.1f} N per finger pair over {span * 1000:.0f} mm (friction alone, mu 0.5 and 20 N grip, "
          f"would hold about {0.5 * 20 * 0.004:.2f} N m)")
    return T, sp, parts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm", help="handumi-sw assets folder (for the checks and renders on OpenArm)")
    ap.add_argument("--spoon", help="the spoon's STL")
    ap.add_argument("--mass", type=float, default=0.5)
    ap.add_argument("--grip", type=float, default=200.0, help="where along the handle to hold it (mm from its end)")
    a = ap.parse_args()
    hard, soft = build()
    for d in ("STEP", "STL"):
        (OUT / d).mkdir(parents=True, exist_ok=True)
    parts = {"spoon_finger_left": hard, "spoon_tpu_left": soft,
             "spoon_finger_right": mirror(hard, Plane.XZ), "spoon_tpu_right": mirror(soft, Plane.XZ)}
    for n, p in parts.items():
        export_step(p, str(OUT / "STEP" / f"{n}.step"))
        export_stl(p, str(OUT / "STL" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = p.bounding_box()
        print(f"{n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, {p.volume / 1e3:.1f} cm3, "
              f"{len(p.solids())} solid(s)")
    if a.openarm and a.spoon:
        render(a.openarm, a.spoon, a.mass, a.grip)



def render(assets, stl, mass, x_grip):
    import pyvista as pv
    import trimesh
    from openarm_hardshell import COLOURS, _mesh, on_openarm
    T, sp, parts = spoon_check(assets, stl, mass, x_grip)
    hard, soft = build()
    hm, sm = _mesh(hard, 0.03), _mesh(soft, 0.03)
    # link7 for context
    q = None
    o, _ = on_openarm(assets, {"hardshell_finger_left": hm, "hardshell_tpu_left": sm}, 0.0)
    link7 = o["link7"]
    col = {"hard": "#e9edf1", "soft": "#2b2e33"}

    def plot(path, items, view, focal=None, zoom=1.0, size=(1800, 1100), up=(0, 0, 1)):
        p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
        p.set_background("white")
        p.enable_anti_aliasing("ssaa")
        for m, c in items:
            p.add_mesh(pv.wrap(m), color=c, smooth_shading=True, split_sharp_edges=True, specular=0.35)
        p.view_vector(view, viewup=up)
        p.reset_camera()
        if focal is not None:
            p.camera.focal_point = focal
        p.camera.zoom(zoom)
        p.screenshot(str(path))
        p.close()
        print("wrote", path.name)
    fingers = [(m, col[n.split("_")[0]]) for n, m in parts.items()]
    plot(IMG / "spoon_held.png", [(link7, COLOURS["link7"]), (sp, "#b9bec5")] + fingers, (1.0, -0.8, 0.45), zoom=1.0)
    # close-up: look down the handle at a cut through the grasp, so the handle's section shows seated in both slots
    import trimesh as tm
    xv = T["left"][:3, 1] if False else None
    G = grasp(assets, spoon_frame(stl), x_grip)[4]
    xa, c = G[:3, 0], G[:3, 3] + G[:3, :3] @ spoon_frame(stl).section([1, 0, 0], [x_grip, 0, 0]).vertices.mean(0)
    cut = sp.slice_plane(c, -xa, cap=True)                                    # the bowl side removed
    fing = [(m.slice_plane(c - 0.0 * xa, -xa, cap=True), cc) for m, cc in fingers]
    p = pv.Plotter(off_screen=True, window_size=(1600, 1100), lighting="three lights")
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    for m, cc in [(cut, "#b9bec5")] + fing:
        p.add_mesh(pv.wrap(m), color=cc, smooth_shading=True, split_sharp_edges=True, specular=0.35)
    p.camera_position = [tuple(c + xa * 160 + G[:3, 2] * 20), tuple(c), tuple(-G[:3, 2])]
    p.camera.view_angle = 28
    p.screenshot(str(IMG / "spoon_slots.png"))
    p.close()
    print("wrote spoon_slots.png")
    side = [(hm, col["hard"]), (sm, col["soft"])]
    plot(IMG / "spoon_finger.png", side, (0.25, -1.0, 0.2), up=(0, 0, -1), size=(1800, 900), zoom=1.3)


if __name__ == "__main__":
    main()
