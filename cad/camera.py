"""The D405 mount as a matched pair: the same tab, at the same pose relative to the Hand-E fingers, on HandUMI and on
the robot. This is Scissor-UMI's approach (a plate between the robot flange and the gripper's coupling, with a tab
that carries the camera, and an identical tab on the handheld), with a pose chosen so the camera sees the jaws on both.

  Pose      set by POSE below, in HandUMI's main-support frame M: where the D405's glass is and what it looks at
            (the grasp point between the Hand-E fingertips). It is moved into the robot's frame through the Hand-E
            finger frame, which both builds share (tips.py places the Hand-E fingers on HandUMI's flanges).
  Tab       Scissor-UMI's tab: the D405's back flat on it, its 6x M3 pattern (a 20 mm diamond for the camera, an outer
            pair at +-22.5 mm for the hood), counterbored from behind so the screws go in from the back.
  HandUMI   main_support gets the mast (restyle.py adds post_m()): a stem lofted out of the T-plate into the tab.
  Robot     hande_camera_plate: the team plate's ring (Scissor-UMI's ur_camera_plate: the recess over the UR flange,
            the ISO 9409-1-50-4-M6 holes and dowel, the pilot for the Robotiq coupling; Robotiq uses the same coupling
            on the Hand-E) with a swan-neck arm that runs down beside the Hand-E to the same tab.
  Hood      d405_hood: Scissor-UMI's hood, unchanged, on both.

Frames. M: rods along Y, tips toward -X, hand toward +Z (-Z is up in use). Tool: the robot flange face, z out of it,
x the Hand-E's closing axis, the camera on the +y side. Tab frame (u, v, w): origin on the tab face at the diamond's
centre, w the view (the tab's normal), u up the image (away from the jaws), v = w x u (across the jaws).

    python camera.py      # -> ../hardware/{STEP,STL}/{right,left}/d405_hood.*, ../hardware/{STEP,STL}/robot/hande_camera_plate.*
"""
import math
from functools import lru_cache
from pathlib import Path

import numpy as np
from build123d import (Axis, Box, Cylinder, Face, Location, Polyline, Pos, Wire, export_step, export_stl, fillet,
                       import_step, loft)

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "hardware"
PLATE_STEP = HERE / "source/camera_mount/ur_camera_plate.step"

# ---- pose (M) -------------------------------------------------------------------------------------------------------
POSE = dict(glass=(-26.0, 72.5, -54.0),     # centre of the D405's front glass: over the encoder, 54 mm above the
                                            # T-plate's top, 96 mm from what it looks at
            target=(-72.0, 72.5, 30.5),      # the middle of the Hand-E fingers, on the jaws' centre plane: the jaws
                                             # stay in view from closed to fully open
            aim_up=0.0)                      # degrees: tilt the view up from the target

# ---- the D405, the tab and the hood (Scissor-UMI: scissor-umi/cad/head.py) -----------------------------------------
CAM = (42.0, 42.0, 23.0)                    # D405 body u x v x depth
TAB_HOLES_UV = ((0.0, 10.0), (0.0, -10.0), (10.0, 0.0), (-10.0, 0.0), (2.0, 22.5), (2.0, -22.5))
CAM_CLR, HOOD_WALL = 0.3, 2.6
HOOD_H, HOOD_FLANGE, HOOD_V = CAM[2] - 4.0, 4.0, 27.5
HOOD_U = CAM[0] / 2 + CAM_CLR + HOOD_WALL
M3_CLEAR, M3_PILOT, CBORE, TAB_SEAT = 3.4, 2.6, 6.2, 3.0   # TAB_SEAT: plate under a head (M3 x 6 -> 3 mm in the D405)
TAB_T = 8.0                                 # tab plate thickness behind the face (Scissor-UMI's mast: 8)
TAB_V = 30.0                                # tab half-width across
TAB_U = (-HOOD_U - 9.0, HOOD_U + 3.0)       # tab extent up the image (u); below the hood it is the arch's crossbar
TAB_R = 8.0                                 # tab corner radius

# ---- HandUMI mast: an arch over the encoder ---------------------------------------------------------------------------
CHIN = 9.0                                  # the tab runs this far below the hood, as the crossbar the legs carry
LEG = dict(v=23.0, w=11.0, t=TAB_T)         # legs meet the crossbar at v = +-23 (outside the encoder), 11 wide
LEG_ROOT = dict(x=-6.0, len=8.0, width=13.0)   # leg feet on the T-plate's bar (centre x; size along x, along y): on the
                                            # solid strip between its lightening slots (x <= -11) and its back edge
SERVO_Y = (60.1, 84.9)                      # the encoder between the legs (M y)

# ---- robot plate -------------------------------------------------------------------------------------------------------
PLATE_RING_C = (0.0, -35.81)                # flange axis in the team plate's frame (x, y)
PLATE_FLANGE_Z, PLATE_BOSS_Z = 3.0, 10.0    # robot flange face / coupling seat in the team plate's frame (z)
RING_R = 37.5                               # the team plate's ring radius (kept as it is)
HANDE_FINGERS_Z = 110.0                     # Hand-E description: coupling seat -> finger frames
HANDE_HALF = 37.5                           # Hand-E body and coupling: 75 x 75 mm
LOBE = dict(y=52.0, x=30.0, z=(-3.0, 3.0), r=10.0)   # flat lobe out of the ring on the camera side, under the coupling
                                            # (the coupling starts 4 mm above the flange face)
ROBOT_LEG = dict(v=21.0, w=12.0, foot=(41.0, 51.0), web=3.0)   # the robot arch's legs: at v = +-21, 12 wide, feet on
                                            # the lobe (y); a 3 mm web joins them into one ribbed blade (nothing passes
                                            # between them on the robot, and a robot arm shakes the camera far more)


# ======================================================================================================================
# frames
def _T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _loc(T):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*T[0, :4], *T[1, :4], *T[2, :4])
    return Location(t)


@lru_cache(None)
def jaw_frame_m():
    """4x4: the Hand-E finger frame (finger.dae: x closing, z along the finger, origin between the fingers) in M."""
    from tips import TIP_SETS, link_transform
    ts = next(t for t in TIP_SETS if t.interface == "hande")
    Tl = _T(np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float), [-48.5, 72.5, 30.5])   # both links on the centre line
    Tt, Ti = Tl @ link_transform(ts, "right", "thumb"), Tl @ link_transform(ts, "right", "index")
    J = Tt.copy()
    J[:3, 3] = (Tt[:3, 3] + Ti[:3, 3]) / 2
    if (J[:3, :3] @ [0, 1, 0])[2] > 0:                                    # the camera side (+y) is up (-M z)
        J[:3, :3] = J[:3, :3] @ np.diag([-1.0, -1.0, 1.0])                # the Hand-E is symmetric under a half turn
    return J


def jaw_frame_tool():
    """4x4: the same finger frame on the robot (tool frame): the coupling on the plate's boss, fingers 110 mm on."""
    return _T(np.eye(3), [0, 0, (PLATE_BOSS_Z - PLATE_FLANGE_Z) + HANDE_FINGERS_Z])


def m_to_tool():
    return jaw_frame_tool() @ np.linalg.inv(jaw_frame_m())


@lru_cache(None)
def tab_frame():
    """4x4: tab frame -> M, from POSE."""
    g, tgt = np.array(POSE["glass"], float), np.array(POSE["target"], float)
    w = (tgt - g) / np.linalg.norm(tgt - g)
    v = np.array([0, -1.0, 0])                                           # across the jaws
    v -= w * (v @ w)
    v /= np.linalg.norm(v)
    u = np.cross(v, w)                                                   # up the image (u x v = w)
    a = math.radians(POSE["aim_up"])                                     # tilt the view up about v
    w, u = math.cos(a) * w + math.sin(a) * u, math.cos(a) * u - math.sin(a) * w
    return _T(np.column_stack([u, v, w]), g - w * CAM[2])


def tab_frame_tool():
    return m_to_tool() @ tab_frame()


def camera_pose():
    """(R, t): D405 body frame -> M. Body: back on the tab face (z = 0), looking +z, x = -v (image right), y = u."""
    T = tab_frame()
    return T[:3, :3] @ np.array([[0, 1.0, 0], [-1.0, 0, 0], [0, 0, 1.0]]), T[:3, 3].copy()


def optical_pose():
    """4x4: D405 optical frame (OpenCV: z the view, x right, y down) on the glass, in M."""
    T = tab_frame()
    u, v, w = T[:3, 0], T[:3, 1], T[:3, 2]
    return _T(np.column_stack([-v, -u, w]), T[:3, 3] + w * CAM[2])


def in_tab(shape, T=None):
    return shape.moved(_loc(tab_frame() if T is None else T))


def to_m(p):
    return (tab_frame() @ [*p, 1.0])[:3]


# ======================================================================================================================
# geometry
def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _soft(shape, edges, radii):
    """Fillet with the first radius that works, and that leaves the part whole (OCC sometimes returns a valid but
    truncated solid)."""
    v0 = shape.volume
    for r in radii:
        try:
            out = fillet(list(edges), r)
            if out.is_valid and len(out.solids()) == len(shape.solids()) and abs(out.volume - v0) < 0.05 * v0:
                return out
        except Exception:
            pass
    return shape


def _rect(pts):
    return Face(Wire(Polyline(*[tuple(map(float, p)) for p in pts], close=True).edges()))


def tab_local():
    """Scissor-UMI's tab as a plate (tab frame): face at w = 0, TAB_T behind, rounded, the six holes counterbored
    from behind."""
    t = _box(TAB_U[0], TAB_U[1], -TAB_V, TAB_V, -TAB_T, 0.0)
    t = _soft(t, t.edges().filter_by(Axis.Z), (TAB_R, 5.0))
    t = _soft(t, [e for e in t.edges() if e.center().Z < -TAB_T + 0.01], (2.0, 1.5))
    return t


def tab_holes():
    cut = None
    for u, v in TAB_HOLES_UV:
        c = Pos(u, v, -20.0) * Cylinder(M3_CLEAR / 2, 40.0) + Pos(u, v, -TAB_SEAT - 20.0) * Cylinder(CBORE / 2, 40.0)
        cut = c if cut is None else cut + c
    return cut


def hood_space():
    """Everything in front of the tab that the hood and the camera need (tab frame)."""
    return _box(-HOOD_U - 0.5, HOOD_U + 0.5, -HOOD_V - 0.5, HOOD_V + 0.5, 0.0, CAM[2] + 6.0)


def leg_feet():
    """The two legs' feet on the T-plate's top (M x, y): outside the encoder in y."""
    T = tab_frame()
    feet = []
    for sgn in (-1, 1):
        top = (T @ [TAB_U[0] + LEG["w"] / 2, sgn * LEG["v"], -TAB_T / 2, 1.0])[:3]
        feet.append((LEG_ROOT["x"], top[1]))
    return feet


def post_m():
    """HandUMI's mast (M): the tab, its lower edge a crossbar under the hood, carried by two legs that straddle the
    encoder and land on the T-plate's bar. Each leg is lofted from a rounded foot on the plate into the crossbar."""
    from build123d import Plane, Rot, SlotCenterToCenter
    T = tab_frame()
    body = in_tab(tab_local())
    u0 = TAB_U[0]
    for (fx, fy), sgn in zip(leg_feet(), (-1, 1)):
        r = LEG_ROOT
        hx, hy = r["len"] / 2, r["width"] / 2
        foot = _rect([(fx + hx, fy + hy, 0.0), (fx + hx, fy - hy, 0.0), (fx - hx, fy - hy, 0.0), (fx - hx, fy + hy, 0.0)])
        v0, v1 = sgn * LEG["v"] - LEG["w"] / 2, sgn * LEG["v"] + LEG["w"] / 2     # v runs along -M y
        top = _rect([(T @ [u0 + 3.0, v, w, 1.0])[:3] for v, w in ((v0, -LEG["t"] + 2.5), (v1, -LEG["t"] + 2.5),
                                                                       (v1, -1.5), (v0, -1.5))])   # ends inside the crossbar
        leg = loft([foot, top], ruled=True)
        long_edges = [e for e in leg.edges() if e.length > 30.0]           # the four edges running up the leg
        leg = _soft(leg, long_edges, (3.0, 2.0, 1.5))
        body += leg
    body -= in_tab(hood_space())
    body -= in_tab(tab_holes())
    return body


def hood_local():
    """Scissor-UMI's d405_hood, unchanged (tab frame)."""
    hu, hv = CAM[0] / 2 + CAM_CLR, CAM[1] / 2 + CAM_CLR
    ou, ov = hu + HOOD_WALL, hv + HOOD_WALL
    flange = _box(-ou, ou, -HOOD_V, HOOD_V, 0, HOOD_FLANGE)
    flange = fillet(flange.edges().filter_by(lambda e: abs(e.tangent_at(0).Z) > 0.99), 4.0)
    walls = _box(-ou, ou, -ov, ov, 0, HOOD_H)
    body = flange + walls - _box(-hu, hu + 10, -hv, hv, -1, HOOD_H + 1)
    for u, v in TAB_HOLES_UV:
        if abs(v) > 20:                                                  # the outer pair: M3 self-tapping from behind
            body -= Pos(u, v, 0) * Cylinder(1.25, 2 * 8.0)
    for sgn in (-1, 1):                                                  # USB-C either side, near the back
        body -= _box(-6.5, 6.5, sgn * hv - 5, sgn * hv + 5, 3.0, 13.0)
    return body


def d405_local():
    """Stand-in D405 body (tab frame), back on the tab face."""
    b = fillet(_box(-CAM[0] / 2, CAM[0] / 2, -CAM[1] / 2, CAM[1] / 2, 0, CAM[2]).edges().filter_by(Axis.Z), 7.0)
    for v in (-9.0, 9.0):
        b -= Pos(0, v, CAM[2]) * Cylinder(5.0, 1.2)
    return b


# ---- robot plate -------------------------------------------------------------------------------------------------------
def plate_to_tool():
    """4x4: the team plate's frame -> tool frame (the ring's axis on the flange axis, flange face z = 0)."""
    return _T(np.eye(3), [-PLATE_RING_C[0], -PLATE_RING_C[1], -PLATE_FLANGE_Z])


def ring_tool():
    """The team plate's ring (its tab cut away), in the tool frame."""
    p = import_step(str(PLATE_STEP)).moved(_loc(plate_to_tool()))
    keep = Pos(0, 0, 0) * Cylinder(RING_R, 100.0)
    return p & keep


def lobe_tool():
    """A flat lobe that carries the ring out to the camera side (tool frame), as the team plate's own tab root does."""
    z0, z1 = LOBE["z"]
    lobe = _box(-LOBE["x"], LOBE["x"], 0.0, LOBE["y"], z0, z1)
    lobe = _soft(lobe, [e for e in lobe.edges().filter_by(Axis.Z) if e.center().Y > 1.0], (LOBE["r"], 6.0))
    return lobe - Pos(0, 0, 0) * Cylinder(RING_R - 1.0, 40.0)                # the ring itself stays the team plate's


def legs_tool():
    """The robot arch: two legs from the lobe's outer edge into the tab's crossbar, outside the Hand-E's sides."""
    Tt = tab_frame_tool()
    out = None
    u0 = TAB_U[0]
    y0, y1 = ROBOT_LEG["foot"]
    for sgn in (-1, 1):
        v0, v1 = sgn * ROBOT_LEG["v"] - ROBOT_LEG["w"] / 2, sgn * ROBOT_LEG["v"] + ROBOT_LEG["w"] / 2
        xs = sorted((Tt @ [u0, v, 0, 1.0])[0] for v in (v0, v1))
        foot = _rect([(xs[0], y0, 0.0), (xs[1], y0, 0.0), (xs[1], y1, 0.0), (xs[0], y1, 0.0)])
        top = _rect([(Tt @ [u0 + 3.0, v, w, 1.0])[:3] for v, w in ((v0, -TAB_T + 2.5), (v1, -TAB_T + 2.5),
                                                                        (v1, -1.5), (v0, -1.5))])
        leg = loft([foot, top], ruled=True)
        leg = _soft(leg, [e for e in leg.edges() if e.length > 30.0], (3.0, 2.0, 1.5))
        out = leg if out is None else out + leg
    # the web: a thin blade between the legs, on their mid-plane
    v = ROBOT_LEG["v"]
    xs = sorted((Tt @ [u0, s * v, 0, 1.0])[0] for s in (-1, 1))
    wt = ROBOT_LEG["web"]
    ym = (y0 + y1) / 2
    foot = _rect([(xs[0], ym - wt / 2, 0.0), (xs[1], ym - wt / 2, 0.0), (xs[1], ym + wt / 2, 0.0), (xs[0], ym + wt / 2, 0.0)])
    wm = -TAB_T / 2
    top = _rect([(Tt @ [u0 + 3.0, vv, ww, 1.0])[:3] for vv, ww in ((-v, wm - wt / 2), (v, wm - wt / 2), (v, wm + wt / 2),
                                                                     (-v, wm + wt / 2))])
    return out + loft([foot, top], ruled=True)


def hande_camera_plate():
    Tt = tab_frame_tool()
    body = ring_tool() + lobe_tool() + legs_tool() + in_tab(tab_local(), Tt)
    body -= in_tab(hood_space(), Tt)
    body -= in_tab(tab_holes(), Tt)
    # keep the coupling's bore and seat as the team plate has them: nothing of the arm inside the ring's pilot
    body -= Pos(0, 0, PLATE_BOSS_Z - PLATE_FLANGE_Z + 50.0) * Cylinder(RING_R - 0.01, 100.0)
    return body


def hande_meshes_tool(root):
    """Hand-E (robotiq_hande_description meshes from the Neuracore robots repo) in the tool frame, on the plate."""
    import trimesh
    base = Path(root) / "example_fanuc/fanuc_decription/crx5ia/meshes/hand_e"
    z0 = PLATE_BOSS_Z - PLATE_FLANGE_Z
    out = []
    for n, z, rz in (("io_coupler.dae", 0.0, 0.0), ("hande.dae", 11.0, 0.0), ("finger.dae", 110.0, 0.0),
                     ("finger.dae", 110.0, math.pi)):
        m = trimesh.load(base / n, force="mesh")
        m.apply_scale(1000.0)
        m.apply_transform(trimesh.transformations.rotation_matrix(rz, [0, 0, 1]))
        m.apply_translation([0, 0, z0 + z])
        out.append((n, m))
    return out


def report():
    T = tab_frame()
    g, tgt = np.array(POSE["glass"]), np.array(POSE["target"])
    tilt = math.degrees(math.asin(T[2, 2]))
    Tt = tab_frame_tool()
    print(f"D405 glass {g} looks {tilt:.1f} deg below the plate, {np.linalg.norm(tgt - g):.0f} mm to the grasp point")
    print(f"robot: tab centre in the tool frame {np.round(Tt[:3, 3], 1)} (z from the flange, y off the axis)")


def main():
    report()
    h = in_tab(hood_local())
    for side in ("right", "left"):
        export_step(h, str(OUT / "STEP" / side / "d405_hood.step"))
        export_stl(h, str(OUT / "STL" / side / "d405_hood.stl"), tolerance=0.02, angular_tolerance=0.15)
    p = hande_camera_plate()
    print(f"hande_camera_plate valid={p.is_valid} solids={len(p.solids())} vol={p.volume:.0f}")
    for t in ("STEP", "STL"):
        (OUT / t / "robot").mkdir(parents=True, exist_ok=True)
    export_step(p, str(OUT / "STEP/robot/hande_camera_plate.step"))
    export_stl(p, str(OUT / "STL/robot/hande_camera_plate.stl"), tolerance=0.02, angular_tolerance=0.15)


if __name__ == "__main__":
    main()
