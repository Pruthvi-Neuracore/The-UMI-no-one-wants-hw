"""Clean restyle of HandUMI parts. Every mating feature is kept (holes, bores, pockets, rod walls,
servo boss, hinge tab, arm end); only non-functional geometry changes.

  fisheye_camera_main_support -> main_support (T-plate)
      * plate rebuilt from one outline (blends and rounded corners built in) with uniform edge rounding;
        every pocket, hole and nut slot carried over from the original
      * large blended fillets where the arm meets the bar
      * racetrack lightening slots through the two open bar spans
      * rounded arm-end corners
      * smooth blends where the bar meets the round servo boss
      * 45° chamfers on the top corners of the rod end wall and on the outer corners of both bar ends
      * PG logo engraved (0.2 mm) on the outside of the end wall
      * the D405 mast (camera.py): the robot-matched tab on an arch whose legs straddle the encoder
      * two counterbored M3 holes in the arm for the electronics box
  main_support_cover_plate -> end_cover: matching 45° top and corner chamfers
  electronics_box / electronics_lid: Pico 2 + IMU + USB 3 hub enclosure (electronics_box.py)
  *_thumb_link / *_index_middle_finger_link: universal tip flange (finger_link.py)

    python restyle.py      # -> ../hardware/{STEP,STL}/{right,left}/*
"""
from pathlib import Path

from build123d import (Axis, Box, GeomType, Circle, Cylinder, Face, Location, Plane, Polygon, Pos, Rectangle, Rot,
                       SlotCenterToCenter, export_step, export_stl, extrude, fillet, import_step, loft)

import electronics_box as EB
import camera as CAM
from finger_link import finger_link
from logo import logo_face

ROOT = Path(__file__).resolve().parents[1]
SRC = Path(__file__).resolve().parent / "source"   # original parts these edits start from (same for both hands)
OUT = ROOT / "hardware"

PLATE_Z = (0.0, 8.0)          # T-plate thickness in its own frame
BLEND_R = 18.0                # arm-to-bar blend radius
ARM_END_R = 8.0
END_CORNER_R = 5.0           # rounded bar-end corners
KEEL_LEN = 60.0              # length of the camera-post keel along the bar
OUTLINE_R = (3.0, 2.5, 2.0)  # top outline rounds (first that works)
UNDERSIDE_R = (2.0, 1.5)     # underside outline rounds (leaves a flat landing for the camera post)
WALL_CORNER_R = 6.0
SLOTS = ((33.0, 24.0, 10.0), (110.0, 24.0, 14.0))   # (centre y, length, width)
EDGE_R = 1.0                  # edge rounding on the plate outline
LOGO_W, LOGO_DEPTH = 6.0, 0.2


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def plate_prism(face2d, z0=PLATE_Z[0], z1=PLATE_Z[1]):
    return Pos(0, 0, z0) * extrude(face2d, z1 - z0, dir=(0, 0, 1))


def camera_post():
    """The D405 mast (camera.post_m): the robot-matched tab on an arch whose two legs straddle the encoder."""
    return CAM.post_m()


def end_wall(z0=8.0, z1=34.0, depth=10.0):
    """Rod end wall: plan shape matches the plate's rounded bar end, rounded top corners in side view."""
    r = END_CORNER_R
    plan = Pos(-18, depth / 2) * Rectangle(36, depth)
    for x, sx in ((-36.0, -1), (0.0, 1)):
        sq = Polygon((x, 0), (x - sx * r, 0), (x - sx * r, r), (x, r), align=None)
        plan = plan - (sq - Pos(x - sx * r, r) * Circle(r))
    wall = Pos(0, 0, z0) * extrude(plan, z1 - z0, dir=(0, 0, 1))
    side = Pos(-18, 0, (z0 + z1) / 2) * Box(36, 40, z1 - z0)
    side = fillet(side.edges().filter_by(Axis.Y).group_by(Axis.Z)[-1], WALL_CORNER_R)
    wall = wall & side
    for x in (-28.0, -8.0):                                              # Ø4.1 rod sockets from the inner face
        wall -= Pos(x, 5.0 + 3.0, 29.0) * Rot(90, 0, 0) * Cylinder(2.05, 6.02)
    return wall


def outline_2d(shape, z=4.0):
    """Outer outline of the plate at height z, as a face without interior holes."""
    sl = shape & box(-200, 200, -200, 300, z - 0.05, z + 0.05)
    top = max((f for f in sl.faces() if f.geom_type == GeomType.PLANE and f.normal_at().Z > 0.99), key=lambda f: f.area)
    return Face(top.outer_wire()).moved(Location((0, 0, -top.center().Z)))


def blend_faces():
    """2D additions: arm-to-bar blends and bar-to-boss blends."""
    faces = []
    for cy, sy in ((54.5, -1), (90.5, 1)):
        sq = Polygon((0, cy), (BLEND_R, cy), (BLEND_R, cy + sy * BLEND_R), (0, cy + sy * BLEND_R), align=None)
        faces.append(sq - Pos(BLEND_R, cy + sy * BLEND_R) * Circle(BLEND_R))
    cx, cy, R, xe, r = -18.0, 72.5, 22.5, -36.0, 10.0
    for sy in (1, -1):
        d = ((R + r) ** 2 - (cx - (xe - r)) ** 2) ** 0.5
        fc = (xe - r, cy - sy * d)
        i = (xe, cy - sy * (R**2 - (cx - xe) ** 2) ** 0.5)
        k = R / (R + r)
        b = (cx + (fc[0] - cx) * k, cy + (fc[1] - cy) * k)
        faces.append(Polygon((xe, fc[1]), i, b, align=None) - Pos(*fc) * Circle(r))
    return faces


def cut_faces():
    """2D removals: rounded arm end and bar-end corners."""
    faces = []
    for cy, sy in ((54.5, -1), (90.5, 1)):
        sq = Polygon((74.0, cy), (74.0 - ARM_END_R, cy), (74.0 - ARM_END_R, cy - sy * ARM_END_R), (74.0, cy - sy * ARM_END_R), align=None)
        faces.append(sq - Pos(74.0 - ARM_END_R, cy - sy * ARM_END_R) * Circle(ARM_END_R))
    r = END_CORNER_R                                                       # rounded bar-end corners
    for x, sx in ((-36.0, -1), (0.0, 1)):
        for y, sy in ((0.0, -1), (140.0, 1)):
            sq = Polygon((x, y), (x - sx * r, y), (x - sx * r, y - sy * r), (x, y - sy * r), align=None)
            faces.append(sq - Pos(x - sx * r, y - sy * r) * Circle(r))
    return faces


def logo_cut(width, depth, at, normal_rot):
    """Engraving prism for the PG logo: `at` = position on the face, `normal_rot` turns +Z into the face normal."""
    return Pos(*at) * normal_rot * Pos(0, 0, -depth) * extrude(logo_face(width), depth + 0.5)


def soften(shape, edges, radii, label):
    """Fillet `edges` with the first radius in `radii` that works; returns (shape, radius used)."""
    edges = list(edges)
    if not edges:
        return shape, 0.0
    for r in radii:
        try:
            out = fillet(edges, r)
            if out.is_valid:
                return out, r
        except Exception:
            pass
    print(f"  ! {label}: could not round {len(edges)} edges")
    return shape, 0.0


def flat_edges(shape, z, pred=lambda e: True):
    return [e for e in shape.edges()
            if abs(e.bounding_box().min.Z - z) < 0.01 and abs(e.bounding_box().max.Z - z) < 0.01 and pred(e)]


def main_support(logo=True):
    orig = import_step(str(SRC / "fisheye_camera_main_support.step"))
    z0, z1 = PLATE_Z
    # exact outline of the original plate (bar, arm, servo boss); its features are carried over below as voids
    up = lambda f: extrude(f, z1 - z0, dir=(0, 0, 1))
    old = (up(Pos(-18, 70) * Rectangle(36, 140)) + up(Pos(37, 72.5) * Rectangle(74, 36))
           + up(Pos(-18, 72.5) * Circle(22.5))).clean()
    plate = old
    for f in blend_faces():
        plate = plate + up(f)
    for f in cut_faces():
        plate = plate - Pos(0, 0, -1) * extrude(f, z1 - z0 + 2, dir=(0, 0, 1))
    plate = (plate.clean() + end_wall()).clean()                      # wall first, so the outline rounds run into it
    joint = [e for e in plate.edges() if abs(e.center().Z - z1) < 0.05 and abs(e.center().Y - 10.0) < 0.05
             and e.bounding_box().size.Z < 0.05]
    plate, _ = soften(plate, joint, (4.0, 3.0, 2.0), "wall-to-plate blend")   # big concave blend, before the outline rounds

    # 1) soft outline: big rounds top and bottom; small ones on the bar-end face (the end cover seats there)
    end_face = lambda e: e.center().Y > 139.0
    plate, _ = soften(plate, flat_edges(plate, z1, lambda e: e.center().Y > 14.05 and not end_face(e)), OUTLINE_R, "top outline")
    plate, _ = soften(plate, flat_edges(plate, z0, lambda e: not end_face(e)), UNDERSIDE_R, "bottom outline")
    plate, _ = soften(plate, flat_edges(plate, z0, end_face) + flat_edges(plate, z1, end_face), (1.0, 0.6), "bar-end face")
    # racetrack slots (the one next to the rod end wall is smaller: FEA shows the rod loads peak there), rounded rims
    for cy, length, width in SLOTS:
        plate -= Pos(-18.0, cy, -1) * extrude(Rot(0, 0, 90) * SlotCenterToCenter(length - width, width), 10)
    in_slot = lambda e: -26 < e.center().X < -10 and any(abs(e.center().Y - cy) < l / 2 + 1 for cy, l, _ in SLOTS)
    plate, _ = soften(plate, flat_edges(plate, z0, in_slot) + flat_edges(plate, z1, in_slot), (2.0, 1.5, 1.0), "slot rims")

    # carry over every pocket, hole and nut slot inside the plate (thin edge-rounding slivers are dropped)
    voids = old - orig
    for v in voids.solids():
        bb = v.bounding_box().size
        if bb.Z > 1.4 and min(bb.X, bb.Y) > 2.5:                       # real features; edge-rounding slivers are thin
            plate -= v

    # 2) soften the top of the end wall
    s = plate
    top = [e for e in s.edges() if e.center().Z > 33.9 and e.bounding_box().size.Z < 0.05]
    s, _ = soften(s, top, (1.5, 1.0), "wall top edges")

    # 3) camera post, blended into the underside
    s = (s + camera_post()).clean()
    feet = CAM.leg_feet()
    r = CAM.LEG_ROOT
    for fx, fy in feet:                                                   # blend each foot into the plate
        root = [e for e in s.edges() if abs(e.center().Z - z0) < 0.05 and e.bounding_box().size.Z < 0.05
                and abs(e.center().X - fx) < r["len"] / 2 + 0.6 and abs(e.center().Y - fy) < r["width"] / 2 + 0.6]
        s, _ = soften(s, root, (2.5, 1.5, 1.0), "leg-to-plate blend")

    for lx, ly in EB.ARM_SCREWS:                                      # electronics box: M3 up through the arm
        x, y = EB.to_m(lx, ly)
        s -= Pos(x, y, 4.0) * Cylinder(1.7, 10)
        s -= Pos(x, y, 8.0 - 1.75) * Cylinder(3.25, 3.6)             # counterbore: head flush with the hand side
    if logo:
        s -= logo_cut(LOGO_W, LOGO_DEPTH, (-18.0, 0.0, 17.0), Rot(90, 0, 0))   # outer face of the end wall (-Y)
    return s


def end_cover():
    """Matches the main support: rounded top corners and rounded outer vertical corners."""
    s = import_step(str(SRC / "main_support_cover_plate.step"))
    side = Pos(-18, -5, 17) * Box(36, 40, 34)                           # side profile with the wall's rounded top corners
    side = fillet(side.edges().filter_by(Axis.Y).group_by(Axis.Z)[-1], WALL_CORNER_R)
    s = s & side
    s, _ = soften(s, [e for e in s.edges().filter_by(Axis.Z) if e.center().Y > -1 and (e.center().X < -35 or e.center().X > -1)],
                  (END_CORNER_R, 4.0, 3.0), "cover outer corners")
    return s


PARTS = {"main_support": main_support, "end_cover": end_cover,
         "electronics_box": EB.shell, "electronics_lid": EB.lid}


def main():
    for side in ("right", "left"):                                      # finger links differ per hand
        for n in (f"{side}_thumb_link", f"{side}_index_middle_finger_link"):
            p = finger_link(SRC / f"{n}.step")
            print(f"{n:32s} valid={p.is_valid} solids={len(p.solids())} vol={p.volume:.0f}")
            export_step(p, str(OUT / "STEP" / side / f"{n}.step"))
            export_stl(p, str(OUT / "STL" / side / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.15)
    from finger_link import button_cover
    cov = button_cover()
    for side in ("right", "left"):
        export_step(cov, str(OUT / "STEP" / side / "button_cover_tpu.step"))
        export_stl(cov, str(OUT / "STL" / side / "button_cover_tpu.stl"), tolerance=0.01, angular_tolerance=0.1)
    print(f"button_cover_tpu valid={cov.is_valid} vol={cov.volume:.0f}")
    for name, fn in PARTS.items():
        p = fn()
        print(f"{name:16s} valid={p.is_valid} solids={len(p.solids())} vol={p.volume:.0f}")
        for side in ("right", "left"):                                  # identical for both hands
            export_step(p, str(OUT / "STEP" / side / f"{name}.step"))
            export_stl(p, str(OUT / "STL" / side / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.15)


if __name__ == "__main__":
    main()
