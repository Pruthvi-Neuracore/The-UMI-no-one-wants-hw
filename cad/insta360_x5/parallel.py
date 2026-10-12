"""PG Parallel: a parallel-jaw gripper on the same Insta360 X5 frame, handheld and robot.

Gripper frame as everywhere else (origin at the mounting face, +X forward, +Y left, +Z up). The X5 frame, its clip and
the robot mounts are the chopstick ones; only the part in front of the frame changes.

  * base: the chopstick base's interface to the X5 frame (everything behind x = 54 mm, so the frame's tongue, the clip
    and the two M3 x 55 clamp screws fit exactly as before) with a housing in front of it: a back wall, two end walls
    that hold two Ø6 x RODS_L mm steel rods along Y (at x = X_ROD, z = ±Z_ROD), and top and bottom plates.
  * carriages: one per finger, each with two LM6UU linear bushings (Ø12 x 19) on the rods. They are synchronised by a
    24-tooth, module 1.25 pinion on a vertical axle at (X_PIN, 0) and two racks:
      - the left carriage's rack runs in front of the pinion, in the upper half of its face width,
      - the right carriage's rack runs behind it, in the lower half; an arm over the pinion and the left rack ties it to
        the right carriage,
    so turning the pinion moves the carriages by the same amount in opposite directions.
  * fingers: L-shaped, from the carriage's front face stepping in so their TPU pads (`parallel_pad_<side>`) meet on
    the centre line when closed. Each finger carries the finger sleeve on its outer face (pad + strap fins + elastic
    strap, see sleeve.py), with the record button on the index side, and four red tracking dots on top.
  * handheld: two compression springs on the rods between the carriages push the jaws open; you pinch them shut.
    robot: a Feetech SM40BL under the housing turns the pinion through its rotor plate (`parallel_pinion_servo`).

Each finger travels 0 .. STROKE mm, so the pads open 0 .. 2 * STROKE mm.

    python parallel.py
"""
import math

import numpy as np
from bd_warehouse.gear import RackGear, SpurGear
from build123d import Box, Cylinder, Location, Plane, Pos, Rot

import sleeve
from parts import load_raw, part_path, save_part, save_reference

STROKE = 30.0                      # per finger, mm
MODULE, TEETH, FACE = 1.25, 24, 12.0
R_PITCH = MODULE * TEETH / 2       # 15
X_PIN = 82.0
X_BACK = (54.0, 58.0)              # housing back wall
CARR = dict(x=(106.0, 130.0), w=22.0, z=18.5)     # carriage: x span, width along y, half-height
Y_IN = 18.0                        # carriage inner face at closed (clear of the pinion's Ø32.5 tips)
X_ROD, Z_ROD, ROD_D, BUSH_D, BUSH_L = 118.0, 12.0, 6.0, 12.0, 19.0
Y_END = Y_IN + CARR["w"] + STROKE + 4.0           # inner face of each end wall
WALL = 4.0
PLATE_Z = (CARR["z"] + 1.0, CARR["z"] + 4.0)       # top plate (bottom plate mirrored)
X_FRONT = CARR["x"][1] + 4.0                       # housing front (end walls run to here)
RACK_DEPTH = 6.0                                   # rack body behind the pitch line
PHASE_DEG = 1.5                                    # pinion tooth phase that meshes with both racks (searched)
FINGER = dict(x0=CARR["x"][1], x_step=144.0, x1=214.0, t=12.0, h=24.0)   # finger: thickness along y, height
PAD_T, PAD_X = 2.0, (168.0, 214.0)                 # TPU pad on the inner face
SLEEVE_X = 168.0                                   # sleeve centre along the finger
DOT_X = (154.0, 169.0, 184.0, 199.0)               # red dots on the finger top
RODS_L = 2 * (Y_END + WALL)
# robot: where the servo envelope (modelled on the chopstick driver jaw's pivot) moves to put its shaft on the pinion
SHAFT_CHOPSTICK = (67.33, 15.0, -13.7)            # servo shaft position and the servo's top face, in robot_gripper.step
SERVO_TOP = -(CARR["z"] + 4.0)                     # under the bottom plate
SERVO_SHIFT = (X_PIN - SHAFT_CHOPSTICK[0], -SHAFT_CHOPSTICK[1], SERVO_TOP - SHAFT_CHOPSTICK[2])


def _box(x, y, z):
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def housing(kind):
    """The base: chopstick interface (behind x = 54) + the housing. kind: 'dc' handheld or 'sa' robot."""
    raw = load_raw()
    iface = raw[f"chopstick_base_{kind}"] & _box((30.0, X_BACK[0] + 0.5), (-60, 60), (-60, 40))
    zt0, zt1 = PLATE_Z
    h = iface
    h += _box(X_BACK, (-Y_END - WALL, Y_END + WALL), (-zt1, zt1))                         # back wall
    for sy in (-1, 1):
        h += _box((X_BACK[0], X_FRONT), sorted((sy * Y_END, sy * (Y_END + WALL))), (-zt1, zt1))   # end walls
    h += _box((X_BACK[0], X_FRONT), (-Y_END - WALL, Y_END + WALL), (zt0, zt1))           # top plate
    h += _box((X_BACK[0], X_FRONT), (-Y_END - WALL, Y_END + WALL), (-zt1, -zt0))         # bottom plate
    # rods: blind Ø6 holes in the end walls (press fit), the rod runs wall to wall
    for z in (-Z_ROD, Z_ROD):
        h -= Pos(X_ROD, 0, z) * Rot(90, 0, 0) * Cylinder(ROD_D / 2 - 0.05, RODS_L - 2 * 1.0)
    # pinion axle: Ø4.2 through the top and bottom plates (M4 x 45 shoulder bolt / pin)
    h -= Pos(X_PIN, 0, 0) * Cylinder(2.1, 2 * zt1 + 2)
    # lighten and show the mechanism: windows in the top plate over each carriage's travel
    for sy in (-1, 1):
        h -= _box((CARR["x"][0] + 3, CARR["x"][1] - 3), sorted((sy * (Y_IN + 2), sy * (Y_END - 2))), (zt0 - 1, zt1 + 1))
    if kind == "sa":
        # servo cradle under the bottom plate: the SM40BL's output shaft comes up through the plate at the pinion axis
        h -= Pos(X_PIN, 0, 0) * Cylinder(6.5, 2 * zt1 + 2)
        sb = raw["servo_envelope"].bounding_box()
        dx, dy, dz = SERVO_SHIFT
        x0, x1, y0, y1 = sb.min.X + dx, sb.max.X + dx, sb.min.Y + dy, sb.max.Y + dy
        z0, z1 = sb.min.Z + dz, sb.max.Z + dz
        c = 0.4
        cradle = _box((x0 - c - 2.4, x1 + c + 2.4), (y0 - c - 2.4, y1 + c + 2.4), (z0 + 6.0, -zt1))
        pocket = _box((x0 - c, x1 + c), (y0 - c, y1 + c), (z0 - 1.0, z1 + c))
        h += cradle - pocket
    return h


def pinion(kind):
    g = SpurGear(module=MODULE, tooth_count=TEETH, pressure_angle=20, thickness=FACE, root_fillet=0.2)
    g = Pos(X_PIN, 0, 0) * Rot(0, 0, PHASE_DEG) * g          # teeth in step with both racks
    if kind == "dc":
        g -= Pos(X_PIN, 0, 0) * Cylinder(2.15, FACE + 2)                     # on the M4 axle
    else:
        g -= Pos(X_PIN, 0, 0) * Cylinder(3.1, FACE + 2)                      # servo spline clearance
        for a in range(0, 360, 90):                                          # rotor plate screws, M3
            g -= Pos(X_PIN + 8 * math.cos(math.radians(a)), 8 * math.sin(math.radians(a)), 0) * Cylinder(1.6, FACE + 2)
    return g


def rack(side):
    """Rack for one carriage, at the closed pose. Left: in front of the pinion, teeth facing -X, upper half.
    Right: behind it, teeth facing +X, lower half."""
    n = 14
    r = RackGear(module=MODULE, tooth_count=n, pressure_angle=20, thickness=FACE / 2, base_height=RACK_DEPTH,
                 root_fillet=0.2)
    L = n * math.pi * MODULE
    # RackGear: length along X, teeth up (+Z) with the pitch line at z = 0, thickness along Y. Place it with a plane
    # whose x runs along Y (the rack's length) and whose z faces the pinion (the teeth).
    if side == "left":                                       # in front of the pinion, teeth facing -X, upper half
        y_c = -(STROKE + 5.0) + L / 2                        # covers y = -(STROKE + 5) .. at closed
        pl = Plane(origin=(X_PIN + R_PITCH, y_c, FACE / 4), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))
    else:                                                    # behind it, teeth facing +X, lower half
        y_c = (STROKE + 5.0) - L / 2
        pl = Plane(origin=(X_PIN - R_PITCH, y_c, -FACE / 4), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    return Location(pl) * r


def carriage(side):
    sy = 1 if side == "left" else -1
    (x0, x1), w, hz = CARR["x"], CARR["w"], CARR["z"]
    y0, y1 = sorted((sy * Y_IN, sy * (Y_IN + w)))
    c = _box((x0, x1), (y0, y1), (-hz, hz))
    for z in (-Z_ROD, Z_ROD):                                               # LM6UU bores, press fit
        c -= Pos(X_ROD, (y0 + y1) / 2, z) * Rot(90, 0, 0) * Cylinder(BUSH_D / 2 - 0.05, w + 2)
    rk = rack(side)
    bb = rk.bounding_box()
    if side == "left":
        # rack bar continues from the toothed part to the carriage, then a block onto the carriage's back face
        bar = _box((bb.min.X, bb.max.X), (bb.max.Y - 1, y1), (0.0, FACE / 2))
        tie = _box((bb.min.X, x0 + 0.5), (Y_IN, y1), (0.0, FACE / 2))
        c += rk + bar + tie
    else:
        # web down from an arm over the pinion and the left rack, to the rack's lower half behind the pinion
        arm_z = (FACE / 2 + 2.5, FACE / 2 + 7.5)
        ya = (y0 + 2, y0 + 12)                                                # within the carriage's own y band
        arm = _box((bb.min.X, x0 + 0.5), ya, arm_z)
        web = _box((bb.min.X, bb.max.X), ya, (-FACE / 2, arm_z[1]))
        bar = _box((bb.min.X, bb.max.X), (ya[0], bb.min.Y + 1), (-FACE / 2, 0.0))
        c += rk + arm + web + bar
    return c


def finger(side, sleeve_kind):
    """sleeve_kind: None (robot finger), 'plain' (thumb) or 'button' (index)."""
    sy = 1 if side == "left" else -1
    f = FINGER
    t, hh = f["t"], f["h"] / 2
    root = _box((f["x0"], f["x_step"]), sorted((sy * Y_IN, sy * (Y_IN + CARR["w"]))), (-hh, hh))
    step = _box((f["x_step"] - t, f["x_step"]), sorted((sy * PAD_T, sy * (Y_IN + CARR["w"]))), (-hh, hh))
    blade = _box((f["x_step"] - t, f["x1"]), sorted((sy * PAD_T, sy * (PAD_T + t))), (-hh, hh))
    fg = root + step + blade
    # rounded tip
    fg -= _box((f["x1"] - 6, f["x1"] + 1), sorted((sy * PAD_T, sy * (PAD_T + t))), (hh - 4, hh + 1))
    # screws to the carriage: 2 x M3 through the root into heat-set inserts in the carriage's front face
    for z in (-8.0, 8.0):
        fg -= Pos(f["x0"] + 6, sy * (Y_IN + CARR["w"] / 2), z) * Rot(0, 90, 0) * Cylinder(1.7, 14)
    pl = None
    if sleeve_kind:
        n = (0, sy, 0)
        pl = Plane(origin=(SLEEVE_X, sy * (PAD_T + t), 0.0), x_dir=(1, 0, 0), z_dir=n)
        add, remove = sleeve.pad_cutters(pl, sleeve_kind == "button")
        fg = (fg + add) - remove
    return fg, pl


def pad(side):
    sy = 1 if side == "left" else -1
    hh = FINGER["h"] / 2 - 1.0
    return _box(PAD_X, sorted((sy * 0.0, sy * PAD_T)), (-hh, hh))


def dots(side):
    """Red tracking dots on the finger's top face (centres, gripper frame, at the closed pose)."""
    sy = 1 if side == "left" else -1
    return [(x, sy * (PAD_T + FINGER["t"] / 2), FINGER["h"] / 2) for x in DOT_X]


def tcp():
    return (PAD_X[1] - 8.0, 0.0, 0.0)


def save(name, shape, tol=0.02):
    save_part(name, shape, tol)
    bb = shape.bounding_box()
    print(f"  {name:28s} {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm, {len(shape.solids())} solid(s)")


def main():
    import trimesh
    from restyle import restyle
    for kind in ("dc", "sa"):
        save(f"parallel_base_{kind}", restyle(f"parallel_base_{kind}", housing(kind)))
    save("parallel_pinion", pinion("dc"))
    save("parallel_pinion_servo", pinion("sa"))
    for side in ("left", "right"):
        save(f"parallel_carriage_{side}", restyle(f"parallel_carriage_{side}", carriage(side)))
        save(f"parallel_pad_{side}", pad(side), 0.01)
        for kind in (None, "plain", "button"):
            fg, pl = finger(side, kind)
            name = f"parallel_finger_{side}" + ("" if kind is None else "_sleeve" if kind == "plain" else "_button")
            save(name, fg)
            if kind == "button":
                save(f"parallel_button_cover_{side}", sleeve.cover(pl), 0.01)
            if kind == "plain":
                save_reference(f"parallel_strap_{side}", sleeve.strap(pl))
        # dot inlays
        discs = []
        for c in dots(side):
            d = trimesh.creation.cylinder(radius=1.75, height=0.6)
            d.apply_translation(np.array(c) - [0, 0, 0.3])
            discs.append(d)
        trimesh.util.concatenate(discs).export(part_path(f"parallel_finger_{side}_dots", "stl"))
    # rods and bushings, for the assemblies
    for z in (-Z_ROD, Z_ROD):
        r = Pos(X_ROD, 0, z) * Rot(90, 0, 0) * Cylinder(ROD_D / 2, RODS_L - 2.0)
        save_reference(f"parallel_rod_{'top' if z > 0 else 'bottom'}", r)


if __name__ == "__main__":
    main()
