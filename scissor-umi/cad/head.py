"""The camera + Quest head, mounted the way the D405 is mounted on the robot's 2F-140.

Robot: `ur_camera_plate` (source/camera_mount/, the team's own part, used unchanged). It is a 6 mm spacer between the UR
flange and the 2F-140's coupling (a Ø63 recess over the flange, a Ø62.8 pilot for the coupling, ISO 50 M6 holes and
a dowel hole). A tab bent up 8° carries the camera ~78 mm off the axis, behind the jaws, looking forward and 8° in
towards the axis. The tab has a 6 x M3 pattern (a Ø20 diamond plus two at ±22.5 mm).

Handheld: `tab_mast` stands on the top plate and puts an identical tab face, with the same holes, at the same pose
in G. Both builds then carry the same two parts:

  the D405, straight on the tab face (its back on the tab, looking along the tab's normal), fixed the way it is on
  the robot plate. On the robot that is all: the plate and the camera.

The handheld adds two parts on its tab:
  d405_hood    an open frame round the camera (no floor, so the camera sits on the tab exactly as on the robot),
               screwed from behind the mast into the tab's two outer holes (±22.5 mm). The camera itself is fixed to
               the tab as on the robot plate; the hood guards it and carries the Quest arm.
  quest_arm    a pad on the back of the mast, above the camera (4 x M3 from the mast's front into inserts in the
               pad), and a split collar bored to the Touch Pro's handle (quest.py), clamped with one M3. The
               controller stands upright on the centre plane right behind the camera, face forward, above the hand.

Frames: M = the plate's STEP frame (z from the flange, the tab along +y). G = kin.py's gripper frame
(robotiq_arg2f_base_link). The tab frame T has u along the tab (away from the axis), v = G y and w = the tab's
normal (forward), with its origin on the tab face at the Ø20 diamond's centre.
"""
import math
from pathlib import Path

import numpy as np
from build123d import Box, Cylinder, Location, Plane, Polygon, Pos, Rot, Solid, extrude, fillet, import_step

import kin
import quest as Q

HERE = Path(__file__).resolve().parent
PLATE_STEP = HERE / "source" / "camera_mount" / "ur_camera_plate.step"

# ---- the robot plate, measured from its STEP -----------------------------------------------------------------------
RING_C_M = (0.0, -35.81)          # flange axis in M (x, y)
BOSS_TOP_M = 10.0                 # the coupling sits on the pilot boss (M z = 10)
FLANGE_M = 3.0                    # the robot flange's face in M (top of the Ø63 recess)
TAB_O_M = np.array([0.0, 42.025, 11.372])          # tab face, at the diamond's centre
TAB_N_M = np.array([0.0, -math.sin(math.radians(8.0)), math.cos(math.radians(8.0))])   # tab normal (8.0° tilt)
TAB_HOLES_UV = [(0.0, 10.0), (0.0, -10.0), (10.0, 0.0), (-10.0, 0.0), (2.0, 22.5), (2.0, -22.5)]   # (u, v), Ø3.4

# M -> G: M y (the tab) -> G x (up), M x -> -G y, M z -> G z; the boss top on the coupling's back face
R_MG = np.array([[0, 1.0, 0], [-1.0, 0, 0], [0, 0, 1.0]])


def m_to_g():
    zc = -kin.g()["coupling_to_base"] - kin.g()["coupling_back"]       # coupling's back face in G
    T = np.eye(4)
    T[:3, :3] = R_MG
    T[:3, 3] = [-RING_C_M[1], 0.0, zc - BOSS_TOP_M]
    return T


def flange_z():
    """The robot flange face in G (the plate adds BOSS_TOP_M - FLANGE_M = 7 mm)."""
    return (m_to_g() @ [0, 0, FLANGE_M, 1])[2]


def tab_frame():
    """4 x 4: the tab frame T (u, v, w) -> G."""
    M = m_to_g()
    w = R_MG @ TAB_N_M
    u = R_MG @ np.array([0.0, TAB_N_M[2], -TAB_N_M[1]])      # along the tab, away from the axis
    v = np.cross(w, u)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([u, v, w])
    T[:3, 3] = (M @ [*TAB_O_M, 1])[:3]
    return T


def _loc(T):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*T[0, :4], *T[1, :4], *T[2, :4])
    return Location(t)


def in_tab(shape):
    T = tab_frame()
    return shape.moved(Location(Plane(origin=tuple(T[:3, 3]), x_dir=tuple(T[:3, 0]), z_dir=tuple(T[:3, 2]))))


def robot_plate():
    return import_step(str(PLATE_STEP)).moved(_loc(m_to_g()))


# ---- D405 hood ------------------------------------------------------------------------------------------------------
CAM = (42.0, 42.0, 23.0)          # D405 body: u x v x depth
CAM_CLR = 0.3
WALL = 2.6
BASE_T = 0.0                      # the camera's back sits on the tab face itself
WALL_H = CAM[2] - 4.0             # walls stop 4 mm short of the camera's face, so the glass stands proud
FLANGE = 4.0                      # the hood's flange on the tab face, outside the camera
HOOD_U = CAM[0] / 2 + CAM_CLR + WALL      # the hood's top edge (u)


def _box(u, v, w):
    return Pos((u[0] + u[1]) / 2, (v[0] + v[1]) / 2, (w[0] + w[1]) / 2) * Box(u[1] - u[0], v[1] - v[0], w[1] - w[0])


def hood_local():
    hu, hv = CAM[0] / 2 + CAM_CLR, CAM[1] / 2 + CAM_CLR
    ou, ov = hu + WALL, hv + WALL
    flange = _box((-ou, ou), (-27.5, 27.5), (0, FLANGE))
    flange = fillet(flange.edges().filter_by(lambda e: abs(e.tangent_at(0).Z) > 0.99), 4.0)
    walls = _box((-ou, ou), (-ov, ov), (0, WALL_H))
    body = flange + walls - _box((-hu, hu + 10), (-hv, hv), (-1, WALL_H + 1))            # no floor: open to the tab
    for u, v in TAB_HOLES_UV:
        if abs(v) > 20:                                                 # the outer pair: M3 self-tapping from behind
            body -= Pos(u, v, 0) * Cylinder(1.25, 2 * 8.0)
    for s in (-1, 1):                                                   # USB-C either side, near the back
        body -= _box((-6.5, 6.5), (s * hv - 5, s * hv + 5), (3.0, 13.0))
    return body


def d405_hood():
    return in_tab(hood_local())


def camera_pose():
    """D405 optical frame in G (OpenCV: z along the view, x right in the image, y down), on the front glass. The
    image's width runs across the jaws (G y)."""
    T = tab_frame()
    u, v, w = T[:3, 0], T[:3, 1], T[:3, 2]
    o = T[:3, 3] + w * (BASE_T + CAM[2])
    R = np.column_stack([-v, -u, w])                                  # x right (= -G y seen from behind), y down
    F = np.eye(4)
    F[:3, :3] = R
    F[:3, 3] = o
    return F


def d405_mesh():
    import trimesh
    from shapely.geometry import box as rect
    outline = rect(-CAM[0] / 2 + 4, -CAM[1] / 2 + 4, CAM[0] / 2 - 4, CAM[1] / 2 - 4).buffer(4.0)
    m = trimesh.creation.extrude_polygon(outline, CAM[2])
    m.apply_translation([0, 0, BASE_T])
    return m.apply_transform(tab_frame())


# ---- Quest arm + collar --------------------------------------------------------------------------------------------
COLLAR_WALL = 4.0
CLAMP_GAP = 1.6
PAD = dict(u=(HOOD_U + 2.0, 40.0), v=17.0, t=6.0)       # the Quest arm's pad on the mast's back (tab frame); its top stays below the collar's
PAD_HOLES = [(29.5, 11.0), (29.5, -11.0), (37.0, 11.0), (37.0, -11.0)]   # (u, v): M3 from the mast's front


def _poly_solid(pts_yz, x0, x1):
    """A prism along G x from a (y, z) outline."""
    from build123d import Polyline, make_face
    w = Polyline(*[(float(y), float(z)) for y, z in pts_yz], close=True)
    f = make_face(w)
    YZ = np.array([[0, 0, 1.0, 0], [1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 0, 1.0]])
    return (Pos(0, 0, x0) * extrude(f, x1 - x0)).moved(_loc(YZ))


def _hull3d(pts):
    import trimesh
    from build123d import Face, Shell, Wire
    h = trimesh.convex.convex_hull(np.asarray(pts, float))
    return Solid(Shell([Face(Wire.make_polygon([tuple(map(float, h.vertices[i])) for i in f])) for f in h.faces]))


def quest_arm(side="right"):
    """Pad on the mast's back + split collar for the Touch Pro's handle, right behind the camera."""
    x0 = Q.HANDLE_X0 + Q.COLLAR_DZ[0]
    x1 = Q.HANDLE_X0 + Q.COLLAR_DZ[1]
    bore = Q.outline_g(Q.FIT)
    outer = Q.outline_g(Q.FIT + COLLAR_WALL)
    collar = _poly_solid(outer, x0, x1)
    T = tab_frame()
    w0, w1 = -MAST_T - PAD["t"], -MAST_T
    pad = in_tab(_box(PAD["u"], (-PAD["v"], PAD["v"]), (w0, w1)))
    # bridge: the hull of the pad's back face and the collar's front half
    back = [(T @ [u, v, w0 + 0.5, 1])[:3] for u in PAD["u"] for v in (-PAD["v"] + 2, PAD["v"] - 2)]
    zc = Q.HANDLE_Z
    front = [(x, y, z) for x in (x0 + 0.5, x1 - 0.5) for y, z in outer if z > zc]
    arm = pad + _hull3d(np.vstack([back, front])) + collar
    arm -= _poly_solid(bore, Q.HANDLE_X0 - 3.0, x1 + 20.0)
    for u, v in PAD_HOLES:                                               # M3 heat-set inserts, from the pad's face
        arm -= in_tab(Pos(u, v, w1) * Cylinder(2.0, 12.0))
    # split clamp at the back (-z) with two ears and an M3 across
    zb = min(z for _, z in outer)
    xc = (x0 + x1) / 2
    arm += Pos(xc, 0, zb - 5.0) * Box(x1 - x0, 2 * 7.0 + CLAMP_GAP, 12.0)
    arm -= Pos(xc, 0, zb - 2.0) * Box(x1 - x0 + 2, CLAMP_GAP, 22.0)
    arm -= Pos(xc, 0, zb - 6.0) * Rot(90, 0, 0) * Cylinder(1.7, 40)
    arm -= Pos(xc, 7.5 + CLAMP_GAP / 2, zb - 6.0) * Rot(90, 0, 0) * Cylinder(3.1, 3.0)
    arm -= Pos(xc, -(7.5 + CLAMP_GAP / 2), zb - 6.0) * Rot(90, 0, 0) * Cylinder(3.2, 3.0)   # nut side (M3 nut)
    if side == "left":
        from build123d import mirror
        arm = mirror(arm, Plane.XZ)
    return arm


def quest_frame():
    return Q.collar_frame()


# ---- the handheld's tab ------------------------------------------------------------------------------------------
MAST_T = 8.0                      # the mast's plate (the tab copy) is 8 mm thick behind the tab face
MAST_V = 30.0
FOOT = dict(z=(-36.0, -4.0))      # foot on the top plate (G z): 4 x M3 down into inserts in the top plate


def mast_foot_holes():
    return [(-21.0, -32.5), (21.0, -32.5), (-21.0, -8.0), (21.0, -8.0)]   # (y, z)


def tab_mast(top_x):
    """The handheld's copy of the robot tab: a plate in the tab's plane from the top plate (x = top_x) up past the
    hood, its face on the tab face, with the tab's six holes (M3 clearance, counterbored behind: the camera and the hood
    fix from behind as on the robot plate); a foot on the top plate and two gussets behind it."""
    T = tab_frame()
    u_lo = (top_x - T[0, 3]) / T[0, 0] - 2.0                          # where the plane meets the top plate
    u_hi = PAD["u"][1] + 2.0
    plate = in_tab(_box((u_lo, u_hi), (-MAST_V, MAST_V), (-MAST_T, 0.0)))
    for u, v in TAB_HOLES_UV:
        plate -= in_tab(Pos(u, v, 0) * Cylinder(1.7, 4 * MAST_T))
        plate -= in_tab(Pos(u, v, -MAST_T) * Cylinder(3.1, 2 * 3.0))
    for u, v in PAD_HOLES:                                             # the Quest arm: M3 from the front
        plate -= in_tab(Pos(u, v, 0) * Cylinder(1.7, 4 * MAST_T))
        plate -= in_tab(Pos(u, v, 0) * Cylinder(3.1, 2 * 3.0))
    # cut the plate flat on the top plate
    plate = plate & Pos(top_x + 200, 0, 0) * Box(400, 400, 400)
    foot = Pos(top_x + 3.0, 0, sum(FOOT["z"]) / 2) * Box(6.0, 2 * MAST_V, FOOT["z"][1] - FOOT["z"][0])
    body = plate + foot
    for s in (-1, 1):                                                  # gussets behind the plate
        y = s * (MAST_V - 3.0)
        pts = [(top_x, y + d, FOOT["z"][0]) for d in (-3, 3)]
        pts += [(top_x, y + d, -14.0) for d in (-3, 3)]
        pts += [(top_x + 55.0, y + d, (T @ [(top_x + 55 - T[0, 3]) / T[0, 0], 0, -MAST_T + 0.5, 1])[2])
                for d in (-3, 3)]
        body += _hull3d(pts)
    for y, z in mast_foot_holes():
        body -= Pos(top_x, y, z) * Rot(0, 90, 0) * Cylinder(1.7, 40.0)  # M3 down into the top plate's inserts
        body -= Pos(top_x + 6.0 + 10.0, y, z) * Rot(0, 90, 0) * Cylinder(3.1, 20.0)
    return body
