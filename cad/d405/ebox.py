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
quest_cradle: a padded V-cradle for a Meta Quest Touch Pro controller's handle, bolted to the box's back wall
(4 x M3 x 8 into heat-set inserts). The handle stands upright in a 90° V-groove (fits handles about 30-42 mm across, e.g. the Touch Pro,
with 2 mm foam), held by two 20 mm hook-and-loop straps through the cradle's slots; the controller's head and tracking
face stay above the box, clear of the hand. The controller is used for tracking only.

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
CRADLE = dict(x=(-62.0, -38.0), y=24.0, z=(-24.0, 22.0))
CRADLE_INSERTS = [(12.0, 14.0), (-12.0, 14.0), (12.0, -16.0), (-12.0, -16.0)]
STRAP = (21.0, 3.0)                 # 20 mm strap slots
R_ARM, PITCH, SADDLE_L, WING, MIN_T = 38.0, 15.0, 60.0, 7.0, 3.0
R_ROBOT = 68.5                      # ebox_wrist_saddle_robot: round robot forearms up to Ø137 (Shadow Hand)


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


def build_wrist_saddle(r_arm=R_ARM):
    """The lid with a saddle under it: concave across the forearm (R 38), pitched 15° so the D405 looks forward-down
    over the hand, with slotted wings for a 25 mm strap and access holes for the lid screws."""
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
    return lid + body


def build_cradle():
    (x0, x1), y, (z0, z1) = CRADLE["x"], CRADLE["y"], CRADLE["z"]
    c = _box((x0, x1), (-y, y), (z0, z1))
    c = fillet(c.edges().filter_by(Axis.Z), 4.0)
    # 90° V-groove opening backwards (-X), along Z, for the controller handle
    depth = 14.0
    v = Pos(x0, 0, (z0 + z1) / 2) * Rot(0, 0, 45) * Box(2 * depth / math.sqrt(2), 2 * depth / math.sqrt(2), z1 - z0 + 2)
    c -= v
    # strap slots straight through the cradle across Y, just in front of the groove's apex: each strap goes through,
    # round the handle and back
    for z in (z0 + 12.0, z1 - 12.0):
        c -= Pos(x0 + depth + 3.0, 0, z) * Box(STRAP[1], 2 * y + 2, STRAP[0])
    # screws into the box's back wall
    for yy, zz in CRADLE_INSERTS:
        c -= Pos(x1 - 3.0, yy, zz) * _x(Cylinder(1.7, 8.0))
        c -= Pos(x0 + 3.0, yy, zz) * _x(Cylinder(3.0, 10.0))       # counterbores from the groove side
    return c


QUEST_STL = Path(__file__).resolve().parent / "source" / "quest" / "MetaQuestTouchPro_Right.stl"
QUEST_FOAM, QUEST_END_UP = 2.0, 0.0


def controller_envelope():
    """The Meta Quest Touch Pro controller (right) standing in the cradle, for renders and the assembly: the handle
    upright in the V on 2 mm of foam, the ring-less head above the cradle, overhanging backwards so its tracking
    cameras and face stay clear. Uses Meta's model in source/quest/ (not redistributed here, see source/quest/README.md);
    without it, a capsule-and-block stand-in of the same size."""
    import numpy as np
    import trimesh
    if not QUEST_STL.exists():
        return _controller_standin()
    m = trimesh.load(QUEST_STL)
    v = m.vertices
    h = v[v[:, 1] > 15.0] - v[v[:, 1] > 15.0].mean(0)          # the handle (the head disc sits at -y)
    a = np.linalg.eigh(h.T @ h)[1][:, -1]
    a = a if a[2] > 0 else -a                                  # handle axis, towards the head
    back = -np.array([0.0, 1.0, 0.0])                          # the head overhangs towards -y
    back = back - back.dot(a) * a
    back /= np.linalg.norm(back)
    R = np.eye(4)
    R[:3, :3] = np.column_stack([-back, np.cross(a, -back), a]).T   # rows: gripper x, y, z in controller axes
    m.apply_transform(R)
    x0 = CRADLE["x"][0]
    z0, z1 = CRADLE["z"]
    apex = x0 + 14.0                                          # the V's apex (depth 14)
    v = m.vertices
    # the handle's end level with the cradle's bottom, so the straps grip its full length
    m.apply_translation([0, 0, z0 + QUEST_END_UP - v[:, 2].min()])
    # centre the handle across y and push it into the V until it sits on the foam
    v = m.vertices
    band = v[(v[:, 2] > z0) & (v[:, 2] < z1)]
    m.apply_translation([0, -(band[:, 1].min() + band[:, 1].max()) / 2, 0])
    band = m.vertices[(m.vertices[:, 2] > z0) & (m.vertices[:, 2] < z1)]
    gap = ((apex - band[:, 0] - np.abs(band[:, 1])) / np.sqrt(2)).min()
    m.apply_translation([(gap - QUEST_FOAM) * np.sqrt(2), 0, 0])            # moving +x by d closes the gap by d/√2
    return m


def _controller_standin():
    """A stand-in for the controller when Meta's model isn't in source/quest/."""
    import trimesh
    from shapely.geometry import box as rect
    x0 = CRADLE["x"][0]
    r = 17.0
    xc = x0 + 14.0 - r * math.sqrt(2) - 2.0          # tangent to both V faces, on 2 mm of foam
    handle = trimesh.creation.capsule(height=70.0, radius=r, count=[32, 32])
    handle.apply_translation([xc, 0, 0.0])
    head2d = rect(-22, -24, 22, 24).buffer(9.0)
    head = trimesh.creation.extrude_polygon(head2d, 24.0)
    head.apply_translation([0, 0, -12.0])
    head.apply_transform(trimesh.transformations.rotation_matrix(math.radians(35), [0, 1, 0]))
    head.apply_translation([xc - 4.0, 0, 52.0])
    return trimesh.util.concatenate([handle, head])


def main():
    from restyle import restyle
    for name, shape in (("ebox", restyle("ebox", build_box())), ("ebox_lid", build_lid()),
                        ("ebox_wrist_saddle", restyle("ebox_wrist_saddle", build_wrist_saddle())),
                        ("ebox_wrist_saddle_robot", restyle("ebox_wrist_saddle", build_wrist_saddle(R_ROBOT))),
                        ("quest_cradle", restyle("quest_cradle", build_cradle()))):
        save_part(name, shape)
        bb = shape.bounding_box()
        print(f"  {name:20s} {bb.size.X:5.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm, {shape.volume / 1e3:5.1f} cm3, "
              f"{len(shape.solids())} solid(s)")


if __name__ == "__main__":
    main()
