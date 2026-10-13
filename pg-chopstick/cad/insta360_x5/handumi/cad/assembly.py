"""Reconstruct the HandUMI (right hand) assembly from the part files, check it, and render it.

HandUMI publishes parts only, each in its own frame. Every placement here comes from matching
mating features (hole patterns, bores, pins), noted on each part. Frame "M" is the
fisheye_camera_main_support STEP frame: rods along Y, tips toward -X, hand toward +Z, so in use
-Z is up. All parts are loaded from STEP because some HandUMI STLs use a different frame.

    python assembly.py [--opening 0..1] [--support STEP] [--check]
"""
import argparse
import sys
import itertools
from functools import lru_cache
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Pos, Rot, fillet, import_step

from occ import overlap, place

sys.path.insert(0, str(Path(__file__).resolve().parent))
import electronics_box as EB  # noqa: E402
from tips import TIP_SETS, link_transform, orientations  # noqa: E402
import finger_link as FL  # noqa: E402
import hinge as H  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
HW = ROOT / "hardware"
R = HW / "STEP/right"

CRANK_C = np.array([-18.0, 72.5])        # servo axis (Ø20.4 hole) in M
CRANK_R, ROD_LEN = 21.5, 36.0            # crank pin radius, connecting-link centre distance
LINK_PIN_X = {"thumb": -30.5, "index": -5.5}
CRANK_Z, CONN_Z = 10.0, 15.0             # crank centre plane / connecting-link plane (from --solve)
BLUE, WHITE, METAL, DARK, PCB = "#1d4f9c", "#f4f5f7", "#c9ccd1", "#2b2e33", "#2f7d4f"   # frame, accents, steel, bought parts, boards


def T(R3, t):
    m = np.eye(4)
    m[:3, :3] = np.array(R3, float)
    m[:3, 3] = t
    return m


@lru_cache(None)
def step(path):
    """STEP part; mesh-derived parts (gripper tips) ship as STL only and are read from hardware/STL instead."""
    path = Path(path)
    if not path.exists():
        stl = Path(str(path).replace("/STEP/", "/STL/")).with_suffix(".stl")
        from build123d import Mesher
        return Mesher().read(str(stl))[0]
    return import_step(str(path))


def link_positions(opening):
    """Crank angle 0 = closed, 90 deg = fully open. Crossed linkage: each crank pin drives the far link."""
    th = np.radians(90 * opening)
    u = np.array([np.cos(th), np.sin(th)])
    p1, p2 = CRANK_C + CRANK_R * u, CRANK_C - CRANK_R * u
    yt = p1[1] + np.sqrt(ROD_LEN**2 - (LINK_PIN_X["thumb"] - p1[0]) ** 2)
    yi = p2[1] - np.sqrt(ROD_LEN**2 - (LINK_PIN_X["index"] - p2[0]) ** 2)
    return th, p1, p2, yt, yi


def d405_dummy(floor):
    b = fillet((Pos(0, 0, floor + 11.5) * Box(42, 42, 23)).edges().filter_by(Axis.Z), 7)
    for x in (-9, 9):
        b -= Pos(x, 0, floor + 23) * Cylinder(5, 1.2)
    return b


def assemble(opening=0.6, camera="wrist", tipset=None, support=None, crank_z=CRANK_Z, conn_z=CONN_Z, redesign=True):
    parts = {}

    def add(name, shape, mat, colour):
        parts[name] = (place(shape, mat), colour)

    I3 = np.eye(3)
    SRC = ROOT / "cad/source"
    support = support or (R / "main_support.step" if redesign else SRC / "fisheye_camera_main_support.step")
    add("main_support", step(support), np.eye(4), BLUE)
    # cover plate: rod Ø4.1 (x -28/-8, z 29) and M3 (z 4) line up with the bar; nuts in the bar's slots, y 140..150
    add("main_support_cover_plate", step(R / "end_cover.step" if redesign else SRC / "main_support_cover_plate.step"),
        T(I3, [0, 150, 0]), BLUE)
    for x in (-28.0, -8.0):                                   # Ø4 x 135 rods
        add(f"rod_{x:+.0f}", Cylinder(2.0, 135.0), T([[1, 0, 0], [0, 0, 1], [0, -1, 0]], [x, 75.0, 29.0]), METAL)

    th, p1, p2, yt, yi = link_positions(opening)
    # finger links: Ø8.2 LM4UU bores (link y -40.5/-20.5, z 1.5) on the rods; flipped so the pin faces the bar
    RL = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
    add("thumb_link", step(R / "right_thumb_link.step"), T(RL, [-48.5, yt, 30.5]), WHITE)
    add("index_link", step(R / "right_index_middle_finger_link.step"), T(RL, [-48.5, yi, 30.5]), WHITE)
    # record start/stop button: 6 x 6 tactile switch under a TPU cover, flush in the index sleeve's wall (right index: sx = +1)
    bx_, by_, bz_ = FL.button_centre(1)
    top = bx_ - FL.COVER_T - FL.NUB                                   # actuator top, under the cover's nub
    sw = Pos(top - 0.8 - 1.75, by_, bz_) * Box(3.5, 6.0, 6.0) + Pos(top - 0.4, by_, bz_) * Rot(0, 90, 0) * Cylinder(1.75, 0.8)
    add("record_button", sw, T(RL, [-48.5, yi, 30.5]), "#2b2e33")
    cover = Pos(bx_ - FL.COVER_T, by_, bz_) * Rot(0, 90, 0) * FL.button_cover()
    add("button_cover", cover, T(RL, [-48.5, yi, 30.5]), "#d23b2a")
    for nm, yc in (("thumb", yt), ("index", yi)):
        for x in (-28.0, -8.0):
            add(f"lm4uu_{nm}_{x:+.0f}", Cylinder(4.0, 12.0) - Cylinder(2.0, 13.0),
                T([[1, 0, 0], [0, 0, 1], [0, -1, 0]], [x, yc, 29.0]), METAL)

    # STS3215 (SO-ARM100 model): holes (4.2,±10.2)/(-16.5,±10.2) -> M (-9.7/11, 72.5±10.3); top face z 17 -> 5.2 (pocket ceiling)
    add("servo_sts3215", step(HW / "reference/STS3215_03a.step"), T([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], [-5.5, 72.5, -11.8]), DARK)
    c, s = np.cos(th), np.sin(th)
    add("crank_mechanism_plate", step(R / "crank_mechanism_plate.step"),
        T([[c, 0, -s], [s, 0, c], [0, -1, 0]], [CRANK_C[0], CRANK_C[1], crank_z]), WHITE)
    for nm, pc, key, yl in (("1", p1, "thumb", yt), ("2", p2, "index", yi)):
        pl = np.array([LINK_PIN_X[key], yl])
        u = (pc - pl) / np.linalg.norm(pc - pl)
        mid = (pc + pl) / 2
        add(f"connecting_link_{nm}", step(R / f"connecting_link_{nm}.step"),
            T([[u[0], 0, -u[1]], [u[1], 0, u[0]], [0, -1, 0]], [mid[0], mid[1], conn_z]), BLUE)

    # arm end stack on the two M3 holes (69, 64.5/80.5): controller support channel | arm | hand support base
    add("hand_support_base", step(R / "hand_support_base.step"), T(I3, [31.0, 54.5, 8.0]), WHITE)
    add("controller_support", step(R / "right_controller_support.step"),
        T([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], [53.0, 95.4, 0.0]), BLUE)

    # electronics box under the arm (-Z side): Pico 2 on the lid, IMU in the floor, USB 3 hub on the floor;
    # two M3 screws come up through the arm's counterbored holes (30, 72.5) / (50, 72.5)
    RB = np.diag([1.0, -1.0, -1.0])                                    # box local z -> -Z
    cxy = EB.BOX_CENTRE_M
    add("electronics_box", step(R / "electronics_box.step"), T(RB, [cxy[0], cxy[1], 0.0]), WHITE)
    add("electronics_lid", step(R / "electronics_lid.step"), T(RB, [cxy[0], cxy[1], 0.0]), BLUE)
    bz = EB.BOX[2]
    boards = {"pico2": Pos(0, 0, bz - EB.PICO_STANDOFF_H - 0.5) * Box(21.0, 51.0, 1.0),
              "imu": Pos(*EB.IMU_AT, EB.FLOOR - EB.IMU_POCKET[2] + 0.8) * Box(25.0, 22.0, 1.6),
              "usb3_hub": Pos(*EB.HUB_AT, EB.FLOOR + 5.0) * Box(EB.HUB_BAY[0] - 1, EB.HUB_BAY[1] - 1, 10.0)}
    for nm, b in boards.items():
        add(nm, b, T(RB, [cxy[0], cxy[1], 0.0]), PCB)

    # D405 mount on the main support's camera post: Ø12 / M4 hinge, camera at the 65° pose in hinge.py
    if camera == "wrist":
        Rc, tc = H.camera_pose()
        add("camera_mount", step(R / "d405_wrist_mount.step"), T(Rc, tc), WHITE)
        add("d405", d405_dummy(2.0), T(Rc, tc), DARK)

    # gripper tips on the finger-link flange (link y = 0 face); orientation from tips.fit_tipset()
    ts, choice = tipset if tipset is not None else (TIP_SETS[0], None)
    Rs, dus, thumb_side = choice or best_orientation(ts)
    side_files = {"left": (ts.left, ts.centre_l), "right": (ts.right, ts.centre_r)}
    index_side = "right" if thumb_side == "left" else "left"
    for k, (nm, (files, centre), yc) in enumerate((("thumb", side_files[thumb_side], yt), ("index", side_files[index_side], yi))):
        Tt = link_transform(Rs[k], centre, ts.axis, ts.face, dus[k])
        TL = T(RL, [-48.5, yc, 30.5])
        for k, fn in enumerate(files):
            col = "#e07020" if "Soft" in fn else DARK                     # compliant inserts in orange
            add(f"tip_{nm}" + ("" if k == 0 else f"_{k}"), step(HW / "STEP/gripper_tips" / ts.folder / fn), TL @ Tt, col)
    return parts


def best_orientation(ts, cache={}):
    """Pick the shared jaw rotation where the jaws face each other: they get closest (smallest opening without
    overlap, then smallest gap). Checked with meshes, so it's fast."""
    import trimesh
    if ts.name in cache:
        return cache[ts.name]
    RL = [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
    files = {"left": (ts.left, ts.centre_l), "right": (ts.right, ts.centre_r)}
    best = None
    for Rs, dus, thumb_side in orientations(ts):
        index_side = "right" if thumb_side == "left" else "left"
        for op in np.arange(0.0, 0.5, 0.05):
            _, _, _, yt, yi = link_positions(op)
            placed = []
            for k, (side, yc) in enumerate(((thumb_side, yt), (index_side, yi))):
                fl, centre = files[side]
                m = _tip_mesh(ts.folder, fl[0]).copy()
                m.apply_transform(T(RL, [-48.5, yc, 30.5]) @ link_transform(Rs[k], centre, ts.axis, ts.face, dus[k]))
                placed.append(m)
            if all(m.is_volume for m in placed):
                ov = trimesh.boolean.intersection(placed, engine="manifold").volume
            else:                                                        # exact solids for non-watertight meshes
                sol = [place(step(HW / "STEP/gripper_tips" / ts.folder / files[sd][0][0]),
                             T(RL, [-48.5, yc, 30.5]) @ link_transform(Rs[k], files[sd][1], ts.axis, ts.face, dus[k]))
                       for k, (sd, yc) in enumerate(((thumb_side, yt), (index_side, yi)))]
                ov = overlap(sol[0], sol[1])
            if ov < 0.5:
                from scipy.spatial import cKDTree
                gap = float(cKDTree(placed[1].vertices).query(placed[0].vertices)[0].min())
                if best is None or (op, gap) < (best[0], best[1]):
                    best = (op, gap, (Rs, dus, thumb_side))
                break
    cache[ts.name] = best[2]
    return best[2]


def _tip_mesh(folder, fname, cache={}):
    import trimesh
    key = (folder, fname)
    if key not in cache:
        stl = HW / "STL/gripper_tips" / folder / fname.replace(".step", ".stl")
        if stl.exists():
            cache[key] = trimesh.load(stl)
        else:
            sh = step(HW / "STEP/gripper_tips" / folder / fname)
            v, f = sh.tessellate(0.1, 0.3)
            cache[key] = trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f))
    return cache[key]


ALLOWED = [{"rod", "lm4uu"}, {"lm4uu", "thumb_link"}, {"lm4uu", "index_link"}, {"rod", "thumb_link"},
           {"rod", "index_link"}, {"rod", "main_support"}, {"rod", "main_support_cover_plate"},
           # pin joints: Ø3 pins run through MR63 bearings in the connecting links (bearings not modelled)
           {"crank_mechanism_plate", "connecting_link_1"}, {"crank_mechanism_plate", "connecting_link_2"},
           {"connecting_link_1", "thumb_link"}, {"connecting_link_2", "index_link"},
           {"camera_mount", "d405"}, {"imu", "electronics_box"},   # IMU sits in its floor pocket
           {"thumb_link", "tip_thumb"}, {"index_link", "tip_index"},   # bolted face contact (0.03 mm sliver)
           {"tip_thumb", "tip_thumb_1"}, {"tip_index", "tip_index_1"},  # pads / socks sit on their jaws
           {"record_button", "index_link"}, {"button_cover", "index_link"}, {"record_button", "button_cover"},
           # the jaws meet before the mechanism's own stop; the closing point is where the tips touch
           {"tip_thumb", "tip_index"}, {"tip_thumb", "tip_index_1"}, {"tip_thumb_1", "tip_index"}, {"tip_thumb_1", "tip_index_1"}]          # the D405 is a visual stand-in (body only, no fillet at the floor)


def interferences(parts, tol=1.0):
    hits = []
    for (a, (sa, _)), (b, (sb, _)) in itertools.combinations(parts.items(), 2):
        ka, kb = a.split("_")[0] if a.startswith(("rod", "lm4uu")) else a, b.split("_")[0] if b.startswith(("rod", "lm4uu")) else b
        if {ka, kb} in ALLOWED or ka == kb:
            continue
        v = overlap(sa, sb)
        if v > tol:
            hits.append((a, b, round(v, 1)))
    return hits


def render(parts, out, views=((-0.9, -1.0, -0.75), (-0.2, 1.0, -0.6)), size=(1900, 1000), zoom=1.25):
    import pyvista as pv
    pv.OFF_SCREEN = True
    meshes = []
    for sh, col in parts.values():
        v, f = sh.tessellate(0.08, 0.25)
        meshes.append((pv.PolyData(np.array([(q.X, q.Y, q.Z) for q in v]),
                                   np.hstack([np.full((len(f), 1), 3), np.array(f)]).ravel()), col))
    pl = pv.Plotter(off_screen=True, shape=(1, len(views)), window_size=size)
    for i, v in enumerate(views):
        pl.subplot(0, i)
        pl.set_background("white")
        for m, col in meshes:
            pl.add_mesh(m, color=col, specular=0.35)
        pl.view_vector(v, viewup=(0, 0, -1))      # in use, -Z is up
        pl.reset_camera()
        pl.camera.zoom(zoom)
    pl.screenshot(str(out))
    return out


def solve_heights(opening=0.0):
    """Find crank / connecting-link planes with no interference (closed position is the tightest)."""
    best = None
    for cz in np.arange(9.0, 13.01, 0.5):
        for lz in np.arange(13.0, 18.01, 0.5):
            p = assemble(opening, crank_z=cz, conn_z=lz)
            keys = ["crank_mechanism_plate", "connecting_link_1", "connecting_link_2", "thumb_link",
                    "index_link", "main_support", "servo_sts3215"]
            tot = sum(overlap(p[a][0], p[b][0]) for a, b in itertools.combinations(keys, 2)
                      if not {a, b} <= {"thumb_link", "index_link", "main_support"})
            if best is None or tot < best[0]:
                best = (tot, cz, lz)
    return best


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--opening", type=float, default=0.6)
    ap.add_argument("--camera", default="wrist")
    ap.add_argument("--support", default=None)
    ap.add_argument("--out", default=str(ROOT / "docs/img/assembly.png"))
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--solve", action="store_true")
    ap.add_argument("--original", action="store_true", help="stock HandUMI parts instead of the redesign")
    a = ap.parse_args()
    if a.solve:
        print("best (overlap, crank_z, conn_z):", solve_heights())
    else:
        parts = assemble(a.opening, a.camera, support=a.support, redesign=not a.original)
        print(render(parts, a.out))
        if a.check:
            print("interferences:", interferences(parts) or "none")


def cables(opening=0.55):
    """Render-only cables (pyvista tubes): D405 -> electronics box, servo -> box, box -> laptop (the one USB-C)."""
    import pyvista as pv
    Rc, tc = H.camera_pose()
    cam = lambda p: tc + Rc @ np.array(p, float)
    bx = lambda x, y, z: np.array([EB.BOX_CENTRE_M[0] + x, EB.BOX_CENTRE_M[1] - y, -z])   # box local -> M
    wall = -EB.BOX[0] / 2
    _, _, _, yt, yi = link_positions(opening)
    RL = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
    index = lambda p: RL @ np.array(p, float) + np.array([-48.5, yi, 30.5])          # index-link frame -> M
    routes = {
        "record_button": ([index((-9.5, FL.BUTTON_Y + 2.0, FL.BUTTON_Z)), index((-12.0, FL.BUTTON_Y + 2.0, FL.BUTTON_Z)),
                           index((-11.0, -20.0, 6.0)), index((-6.0, -30.0, 14.0)), (-20.0, yi - 4.0, 12.0),
                           (5.0, 60.0, -6.0), (12.0, 95.0, -8.0), bx(wall - 6, -26.0, 8.5), bx(wall + 5, -26.0, 8.5)], 1.0),
        "d405_usb": ([cam((0.0, 21.0, 9.0)), cam((0.0, 34.0, 9.0)), cam((0.0, 44.0, 2.0)), (-20.0, 84.0, -58.0),
                      (5.0, 80.0, -40.0), bx(wall - 8, 0.0, 18.5), bx(wall + 6, 0.0, 18.5)], 2.0),    # out of the top window
        "servo": ([(-24.0, 86.0, -24.0), (-8.0, 96.0, -22.0), (8.0, 104.0, -12.0), bx(wall - 6, -30.0, 8.5),
                   bx(wall + 5, -30.0, 8.5)], 1.2),
        "usb_c_to_laptop": ([bx(10.5, EB.BOX[1] / 2 - 4, 10.0), bx(10.5, EB.BOX[1] / 2 + 10, 10.0),
                             bx(14.0, EB.BOX[1] / 2 + 40, 4.0), bx(30.0, EB.BOX[1] / 2 + 80, -10.0),
                             bx(60.0, EB.BOX[1] / 2 + 120, -30.0)], 2.2),
    }
    out = []
    for name, (pts, r) in routes.items():
        sp = pv.Spline(np.array(pts, float), 200).tube(radius=r, n_sides=20)
        out.append((name, sp, "#16181c"))
    return out
