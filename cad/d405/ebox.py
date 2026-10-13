"""Electronics box, lid, wrist saddle and Quest Touch Pro controller cradle for the D405 head.

ebox: bolts to the back of `d405_frame`'s back plate with the same interface as the robot mount plates (four M3
heat-set inserts in its front wall at (y, z) = (±9, ±26.9) mm, M3 x 10 from the front through the back plate). It
holds a Raspberry Pi Pico 2, an IMU breakout and a USB 3 hub, so the D405 and the Pico share one USB-C cable to the
laptop (the HandUMI electronics):
  * USB 3 hub board standing against the front wall (bay 42 x 38 mm, up to 14 mm thick),
  * Pico 2 on the back wall, IMU flat under the top wall (rigid to the camera's frame),
  * a 12 x 6 mm slot in the top wall for the D405's cable (from the cup's top window), a USB-C slot low in the back
    wall for the cable to the laptop, and two Ø3.5 holes low in the side walls for the record button's wire (one per
    side, so the same box serves either hand),
  * open at the bottom for `ebox_lid` (4 x M2 x 6 self-tapping) or, on the wrist, `ebox_wrist_saddle`.
Quest mount: HandUMI's own controller support (the ring at the back of HandUMI), unchanged, on `quest_arm`: a plate
bolted to the box's back wall (4 x M3 x 10 into heat-set inserts) that carries a copy of HandUMI's main-support arm
end, so the support bolts on exactly as on HandUMI. The Touch Pro's handle stands upright in the ring with its face
forward, over the gripper; the left hand uses the mirror parts and the left controller. Tracking only.

    python ebox.py
"""
import math
from pathlib import Path

from build123d import Axis, Box, Cylinder, Pos, Rot, fillet

from parts import save_part

X0, X1 = -38.0, 0.0                # box, gripper frame (front wall on the back plate's back face at x = 0)
Y = 30.0
Z0, Z1 = -40.0, 24.0
WALL, FRONT = 2.2, 4.0
CORNER_R = 5.0
INSERTS = [(9.0, 26.9), (-9.0, 26.9), (9.0, -26.9), (-9.0, -26.9)]
LID = 2.0
BOSS_IN = 2.6                      # bosses merge into the walls
CRADLE_INSERTS = [(12.0, 14.0), (-12.0, 14.0), (12.0, -16.0), (-12.0, -16.0)]
STRAP = (21.0, 3.0)                 # 20 mm strap slots
R_ARM, PITCH, SADDLE_L, WING, MIN_T = 38.0, 15.0, 60.0, 7.0, 3.0
R_ROBOT = 68.5                      # ebox_wrist_saddle_robot: round robot forearms up to Ø137 (Shadow Hand)


THUMB_STRAP = (21.0, 3.0)    # front tab: the 20 mm thumb strap loops round its bar (slot width, slot length along the arm)
TAB_GAP, TAB_T, TAB_L = 3.0, 4.0, 12.0   # the bar stands TAB_GAP off the skin, TAB_T thick; the tab reaches TAB_L forward


def front_tab(x1, z_front, r_arm, pitch):
    """The front tab for the thumb strap: a curved bar ahead of the saddle, standing off the skin so the strap passes
    under it, with a slot between it and the saddle where the strap comes back up and folds over onto itself.
    Returns (solid to add, solid to cut)."""
    def arm(r, extra=0.0):
        c = Cylinder(r, 4 * TAB_L + 200)
        return Pos(x1, 0.0, z_front - r_arm) * Rot(0, 90 - pitch, 0) * Pos(0, 0, -100) * c
    half = THUMB_STRAP[0] / 2 + 3.0
    reach = Pos(x1 + TAB_L / 2 - 1.0, 0, z_front - r_arm) * Box(TAB_L + 2.0, 2 * half, 2 * r_arm + 40)
    band = (arm(r_arm + TAB_GAP + TAB_T) - arm(r_arm + TAB_GAP)) & reach
    band &= Pos(0, 0, z_front - 6.0) * Box(400, 400, 30.0)             # only the top of the arm
    slot = Pos(x1 + 1.0 + THUMB_STRAP[1] / 2, 0, z_front - 5.0) * Box(THUMB_STRAP[1], THUMB_STRAP[0], 40.0)
    return band, slot


def _box(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _x(shape):
    return Rot(0, 90, 0) * shape


def bosses():
    pts = []
    for bx in (X0 + WALL + BOSS_IN, X1 - FRONT - BOSS_IN):
        for by in (-(Y - WALL - BOSS_IN), Y - WALL - BOSS_IN):
            pts.append((bx, by))
    return pts


def build_box():
    outer = _box((X0, X1), (-Y, Y), (Z0, Z1))
    outer = fillet(outer.edges().filter_by(Axis.Z), CORNER_R)
    inner = _box((X0 + WALL, X1 - FRONT), (-Y + WALL, Y - WALL), (Z0 - 1, Z1 - WALL))
    inner = fillet(inner.edges().filter_by(Axis.Z), CORNER_R - WALL)
    b = outer - inner
    for y, z in INSERTS:                                       # M3 heat-set inserts in the front wall
        b -= Pos(X1 - 3.0, y, z) * _x(Cylinder(2.0, 7.0))
    for bx, by in bosses():                                    # lid screw bosses
        h = 14.0
        b += Pos(bx, by, Z0 + h / 2) * Cylinder(3.0, h)
        b -= Pos(bx, by, Z0 + 5) * Cylinder(0.9, 10.5)
    b -= _box((-22.0, -10.0), (-3.0, 3.0), (Z1 - WALL - 1, Z1 + 1))          # D405 cable, top wall
    b -= _box((X0 - 1, X0 + WALL + 1), (-5.0, 5.0), (Z0 + 4.0, Z0 + 8.6))   # USB-C to the laptop, back wall
    for sy in (-1, 1):                                          # record-button wire, side walls
        b -= Pos(-8.0, sy * (Y - WALL / 2), Z0 + 10.0) * Rot(90, 0, 0) * Cylinder(1.75, WALL + 2)
    for y, z in CRADLE_INSERTS:                                 # inserts for the Quest cradle, back wall
        b -= Pos(X0 + 3.0, y, z) * _x(Cylinder(2.0, 7.0))
    b += _box((X0 + WALL, X0 + WALL + 4.0), (-Y + WALL, Y - WALL), (Z1 - WALL - 4.0, Z1 - WALL))  # insert ribs
    for y, z in CRADLE_INSERTS:
        b -= Pos(X0 + 3.0, y, z) * _x(Cylinder(2.0, 7.0))
    return b


def build_lid():
    lid = _box((X0, X1), (-Y, Y), (Z0 - LID, Z0))
    lid = fillet(lid.edges().filter_by(Axis.Z), CORNER_R)
    for bx, by in bosses():
        lid -= Pos(bx, by, Z0 - LID / 2) * Cylinder(1.1, LID + 2)
        lid -= Pos(bx, by, Z0 - LID + 0.6) * Cylinder(2.1, 1.3)
    return lid


def build_wrist_saddle(r_arm=R_ARM, tab=True):
    """The lid with a saddle under it: concave across the forearm (R 38), pitched 15° so the D405 looks forward-down
    over the hand, with slotted wings for a 25 mm strap, access holes for the lid screws, and a front tab for the
    20 mm thumb strap that carries the palm button."""
    lid = build_lid()
    z_top = Z0 - LID
    xc = (X0 + X1) / 2
    x0, x1 = xc - SADDLE_L / 2, xc + SADDLE_L / 2
    half_w = Y + WING
    depth = 45.0
    body = Pos(xc, 0, z_top - depth / 2) * Box(SADDLE_L, 2 * half_w, depth)
    arm = Pos(x1, 0, z_top - MIN_T - r_arm) * Rot(0, 90 - PITCH, 0) * Pos(0, 0, -SADDLE_L) * Cylinder(r_arm, 4 * SADDLE_L)
    body -= arm
    for sy in (-1, 1):
        body -= Pos(xc, sy * (Y + WING / 2), z_top - depth / 2) * Box(27.0, 3.5, depth + 2)
    for bx, by in bosses():
        body -= Pos(bx, by, z_top - depth / 2) * Cylinder(2.6, depth + 2)
    saddle = lid + body
    if tab:                                           # the thumb strap's tab (see hand README; not on the robot saddle)
        band, slot = front_tab(x1, z_top - MIN_T, r_arm, PITCH)
        saddle = saddle + band - slot
    return saddle


HANDUMI = Path(__file__).resolve().parent / "handumi" / "hardware"
# HandUMI's controller support in its own STEP frame (S): ring axis z (ring z -18..4), opening 40.3 x 45.5 centred at
# RING_C, hung on the main support's arm end (8 mm thick at z 0..8, end at y = 21, M3 holes at (14.9, 16) and
# (30.9, 16)). T_S puts S in the gripper frame: S y -> -X (the ring behind the box), S z -> -Z (HandUMI's -Z is up),
# S x -> -Y, the ring centred on y = 0 and its clip just clear of quest_arm's plate.
RING_C = (20.45, 50.95)
T_S = ((0.0, -1.0, 0.0, -32.5), (-1.0, 0.0, 0.0, RING_C[0]), (0.0, 0.0, -1.0, 0.0))
# HandUMI's main support (frame M) into S: the inverse of where HandUMI's assembly hangs the support
M_TO_S = ((0.0, -1.0, 0.0, 95.4), (1.0, 0.0, 0.0, -53.0), (0.0, 0.0, 1.0, 0.0))
ARM_END_S = ((-1.0, 46.0), (11.0, 22.0), (-0.5, 8.5))           # the arm end's box in S
QUEST_PLATE = dict(x=(-44.0, -38.0), y=24.0, z=(-24.0, 22.0))  # quest_arm's plate on the box's back wall
QUEST_UNDER = 12.0                 # the handle's end hangs this far below the ring
# The ring's oval opening is set at an angle to HandUMI's arm. The Touch Pro's handle fits it best turned 55° to the
# arm (0.4 mm all round, no overlap; searched over yaw and depth), so the support is turned 55° about the ring's axis
# and the controller's face still looks straight forward.
SUPPORT_YAW = 55.0


def _matrix(m):
    import numpy as np
    T = np.eye(4)
    T[:3, :4] = np.array(m)
    return T


ARM_C_S = (22.9, 16.0, 4.0)        # the arm end's centre in S (between the two M3 holes, mid-thickness)
ARM_AT = (-63.5, 0.0, -4.0)        # where it sits in the gripper frame: straight behind the box, on the centre line


def t_support():
    """3 x 4: HandUMI's support frame S into the gripper frame (right hand): T_S's axes turned SUPPORT_YAW about z,
    with the arm end's centre on ARM_AT."""
    import numpy as np
    a = math.radians(SUPPORT_YAW)
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
    R = Rz @ np.array(T_S)[:, :3]
    t = np.array(ARM_AT) - R @ ARM_C_S
    return tuple(map(tuple, np.column_stack([R, t])))


def _loc(m):
    """build123d Location from a 3 x 4 matrix (rotation + mm)."""
    from build123d import Location
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*m[0], *m[1], *m[2])
    return Location(t)


def _mirror_y(shape):
    from build123d import Plane, mirror
    return mirror(shape, Plane.XZ)


def quest_support(side="right"):
    """HandUMI's right controller support, unchanged, on quest_arm's arm end (left hand: its mirror image)."""
    from build123d import import_step
    s = import_step(str(HANDUMI / "STEP" / "right" / "right_controller_support.step")).moved(_loc(t_support()))
    return s if side == "right" else _mirror_y(s)


def build_quest_arm(side="right"):
    """A plate on the box's back wall (4 x M3 x 10 into its heat-set inserts, counterbored from the back) and a copy of
    HandUMI's main-support arm end on an 8 mm neck, so HandUMI's controller support bolts on exactly as it does on
    HandUMI (2 x M3 x 25 + nuts). The arm end sits on the centre line behind the box, turned SUPPORT_YAW; the neck is the
    hull of its footprint and a patch on the plate, with a rib under it."""
    from build123d import import_step
    (x0, x1), y, (z0, z1) = QUEST_PLATE["x"], QUEST_PLATE["y"], QUEST_PLATE["z"]
    plate = fillet(_box((x0, x1), (-y, y), (z0, z1)).edges().filter_by(Axis.X), 4.0)
    ms = import_step(str(HANDUMI / "STEP" / "right" / "main_support.step"))
    (bx0, bx1), (by0, by1), (bz0, bz1) = ARM_END_S
    clip = Pos((bx0 + bx1) / 2, (by0 + by1) / 2, (bz0 + bz1) / 2) * Box(bx1 - bx0, by1 - by0, bz1 - bz0)
    end = ms.moved(_loc(M_TO_S)) & clip
    end = end.moved(_loc(t_support()))
    # neck: the hull of the arm end's footprint and a patch on the plate, 8 mm thick like the arm, and a rib under it
    import numpy as np
    from build123d import Polygon, extrude
    from shapely.geometry import MultiPoint
    T = _matrix(t_support())
    foot = [(T @ [x, yy, 0, 1])[:2] for x in (4.9, 40.9) for yy in (by0 + 0.5, by1 - 1.0)]
    hull = MultiPoint(foot + [(x0 + 0.5, -16.0), (x0 + 0.5, 16.0)]).convex_hull
    zt = (T @ [0, 0, 0, 1])[2]                                          # arm's top face (S z = 0) in the gripper frame
    neck = Pos(0, 0, zt - 8.0) * extrude(Polygon(*list(hull.exterior.coords)[:-1], align=None), 8.0)
    spine = MultiPoint([(T @ [22.9, by1 - 4.0, 0, 1])[:2], (x0 + 0.5, 0.0)])
    (sx0, sy0), (sx1, sy1) = spine.geoms[0].coords[0], spine.geoms[1].coords[0]
    L, ang = math.hypot(sx1 - sx0, sy1 - sy0), math.degrees(math.atan2(sy1 - sy0, sx1 - sx0))
    rib = Pos((sx0 + sx1) / 2, (sy0 + sy1) / 2, zt - 8.0 - 6.0) * Rot(0, 0, ang) * Box(L, 6.0, 12.5)
    arm = end + neck + rib
    a = plate + arm
    for hx in (14.9, 30.9):                                              # the arm end's two M3 holes, through the neck
        hx_, hy_, _ = (T @ [hx, 16.0, 0, 1])[:3]
        a -= Pos(hx_, hy_, zt - 4.0) * Cylinder(1.6, 40.0)
    for yy, zz in CRADLE_INSERTS:
        a -= Pos((x0 + x1) / 2, yy, zz) * _x(Cylinder(1.7, x1 - x0 + 2))
        a -= Pos(x0 - 5.0, yy, zz) * _x(Cylinder(3.1, 12.0))             # counterbore + screwdriver access from the back
    return a if side == "right" else _mirror_y(a)


def _ring_mesh():
    """HandUMI's controller support as a mesh in the gripper frame (cached as STL in build/)."""
    import trimesh
    from build123d import export_stl, import_step
    from parts import ROOT
    p = ROOT / "cache" / "right_controller_support.stl"
    if not p.exists():
        p.parent.mkdir(parents=True, exist_ok=True)
        export_stl(import_step(str(HANDUMI / "STEP" / "right" / "right_controller_support.step")), str(p),
                   tolerance=0.05)
    return trimesh.load(p).apply_transform(_matrix(t_support()))


QUEST_STL = Path(__file__).resolve().parent / "source" / "quest" / "MetaQuestTouchPro_Right.stl"


def controller_envelope(side="right"):
    """The Meta Quest Touch Pro controller standing in HandUMI's ring: handle upright through the ring, centred in it,
    head above, turned so its face (thumbstick, buttons) looks forward over the gripper. The left hand's is the mirror
    image (the left controller). Uses Meta's model in source/quest/ (not redistributed, see source/quest/README.md);
    without it, a capsule-and-block stand-in of the same size."""
    import numpy as np
    if QUEST_STL.exists():
        import trimesh
        m = trimesh.load(QUEST_STL)
        v = m.vertices
        h = v[v[:, 1] > 15.0] - v[v[:, 1] > 15.0].mean(0)      # the handle (the head disc sits at -y)
        a = np.linalg.eigh(h.T @ h)[1][:, -1]
        a = a if a[2] > 0 else -a                              # handle axis, towards the head
        fwd = np.array([0.0, 1.0, 0.0])                        # the face (buttons) looks towards +y: forward
        fwd = fwd - fwd.dot(a) * a
        fwd /= np.linalg.norm(fwd)
        R = np.eye(4)
        R[:3, :3] = np.column_stack([fwd, np.cross(a, fwd), a]).T
        m.apply_transform(R)
        # stand the handle's straight part (20-50 mm up from its end) exactly upright, so it passes straight through
        # the ring
        z0 = m.bounds[0, 2]
        c = []
        for dz in np.arange(20.0, 51.0, 5.0):
            loop = max(m.section([0, 0, 1], [0, 0, z0 + dz]).discrete, key=lambda l: np.ptp(l[:, 0]) * np.ptp(l[:, 1]))
            c.append([z0 + dz, (loop[:, 0].max() + loop[:, 0].min()) / 2, (loop[:, 1].max() + loop[:, 1].min()) / 2])
        c = np.array(c)
        d = np.array([np.polyfit(c[:, 0], c[:, 1], 1)[0], np.polyfit(c[:, 0], c[:, 2], 1)[0], 1.0])
        m.apply_transform(trimesh.geometry.align_vectors(d / np.linalg.norm(d), [0, 0, 1.0]))
    else:
        m = _controller_standin()
    cx, cy = (_matrix(t_support()) @ [*RING_C, 0, 1])[:2]
    z_lo, z_hi = _ring_mesh().bounds[:, 2]
    m.apply_translation([0, 0, z_lo - QUEST_UNDER - m.vertices[:, 2].min()])
    for _ in range(3):                                         # centre the handle's section inside the ring
        b = m.vertices[(m.vertices[:, 2] > z_lo) & (m.vertices[:, 2] < z_hi)]
        m.apply_translation([cx - (b[:, 0].min() + b[:, 0].max()) / 2, cy - (b[:, 1].min() + b[:, 1].max()) / 2, 0])
    if side == "left":
        m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
        m.invert()
    return m


def _controller_standin():
    """A stand-in for the controller when Meta's model isn't in source/quest/ (handle along z, head leaning to +x)."""
    import trimesh
    from shapely.geometry import box as rect
    handle = trimesh.creation.capsule(height=70.0, radius=17.0, count=[32, 32])
    head2d = rect(-22, -24, 22, 24).buffer(9.0)
    head = trimesh.creation.extrude_polygon(head2d, 24.0)
    head.apply_translation([0, 0, -12.0])
    head.apply_transform(trimesh.transformations.rotation_matrix(math.radians(-35), [0, 1, 0]))
    head.apply_translation([4.0, 0, 52.0])
    return trimesh.util.concatenate([handle, head])


def main():
    from restyle import restyle
    for name, shape in (("ebox", restyle("ebox", build_box())), ("ebox_lid", build_lid()),
                        ("ebox_wrist_saddle", restyle("ebox_wrist_saddle", build_wrist_saddle())),
                        ("ebox_wrist_saddle_robot", restyle("ebox_wrist_saddle", build_wrist_saddle(R_ROBOT, tab=False))),
                        ("quest_arm_right", restyle("quest_arm", build_quest_arm("right"))),
                        ("quest_arm_left", restyle("quest_arm", build_quest_arm("left"))),
                        ("quest_support_right", quest_support("right")),
                        ("quest_support_left", quest_support("left"))):
        save_part(name, shape)
        bb = shape.bounding_box()
        print(f"  {name:20s} {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm, {shape.volume / 1e3:5.1f} cm3, "
              f"{len(shape.solids())} solid(s)")


if __name__ == "__main__":
    main()
