"""PG Chopstick restyle: the PG look on the printed parts, without touching any mating feature.

  * Handheld jaws: a finger-sleeve pad with two strap fins for an elastic hook-and-loop strap, and a variant with the
    record button (sleeve.py).
  * PG badge: the PG logo engraved 0.3 mm deep with three slanted "speed" grooves beside it (0.6 mm deep, at the camera's
    10° tilt), on the best-placed flat face of each visible part (base top, camera-frame side, outer jaw face, ...).

Every edit is a shallow boolean cut on an outside face, checked to remove no more than the cutter's own volume
(so nothing internal is opened). Run after extract_parts.py; it rewrites the part files in place,
always starting from the unmodified parts, so re-running doesn't stack marks. Files go to hardware/<module>/.

    python restyle.py
"""
import math

import numpy as np
from build123d import Box, GeomType, Plane, Pos, Rot, Vector, export_step, export_stl, extrude

import sleeve
from logo import logo_face
from parts import load_raw, save_part, save_reference

LOGO_W, LOGO_D = 7.0, 0.3
GROOVE_L, GROOVE_W, GROOVE_D, GROOVE_PITCH, GROOVE_TILT = 9.0, 1.2, 0.6, 2.6, 10.0
MARGIN = 1.2

# preferred face normal for the badge on each part (gripper frame); parts not listed get no badge
BADGE = {
    "chopstick_base_dc": (0, 0, 1), "chopstick_base_sa": (0, -1, 0),
    "x5_lite_frame": (0, -1, 0), "x5_frame": (0, -1, 0),
    "jaw_left_dc": (0, 0, -1), "jaw_left_sa": (0.19, 0.98, 0), "jaw_right": (0.19, -0.98, 0),
    "jaw_right_dc": (0, 0, -1), "jaw_left_dc_button": (0, 0, -1), "jaw_right_dc_button": (0, 0, -1),
    # new PG parts (badged by their own scripts through restyle())
    "pg_pod": (0, -1, 0), "openarm_adapter": (1, 0, 0), "iso_flange_plate": (1, 0, 0),
    "ebox": (0, -1, 0), "ebox_wrist_saddle": (0, -1, 0), "quest_cradle": (0, 1, 0), "parallel_base_dc": (0, 0, 1), "parallel_base_sa": (0, 0, 1),
    "parallel_carriage_left": (1, 0, 0), "parallel_carriage_right": (1, 0, 0),
}


def badge(plane, grooves=True):
    """Logo (+ three slanted grooves to its right), as a cutter solid laid out in `plane` (z out of the face)."""
    shift = -GROOVE_PITCH * 1.5 if grooves else 0.0
    cut = plane * Pos(shift, 0, -LOGO_D) * extrude(logo_face(LOGO_W), LOGO_D + 0.5)
    if grooves:
        x0 = LOGO_W / 2 + shift + 2.5
        for i in range(3):
            g = Pos(x0 + i * GROOVE_PITCH, 0, (0.5 - GROOVE_D) / 2) * Rot(0, 0, 90 - 20) * Box(
                GROOVE_L, GROOVE_W, GROOVE_D + 0.5)
            cut += plane * g
    return cut


def badge_size(grooves=True):
    if grooves:
        return LOGO_W + 2.5 + 3 * GROOVE_PITCH + 2.0, GROOVE_L * math.cos(math.radians(20))
    return LOGO_W, LOGO_W / 2


def fits(face, plane, w, h):
    for u in np.linspace(-w / 2 - MARGIN, w / 2 + MARGIN, 9):
        for v in np.linspace(-h / 2 - MARGIN, h / 2 + MARGIN, 5):
            if face.distance_to(Vector(plane.from_local_coords((u, v, 0)))) > 1e-3:
                return False
    return True


def find_spot(part, prefer, grooves=True):
    """Plane on a flat face facing `prefer` where the whole badge fits, badge running along the face's long side."""
    w, h = badge_size(grooves)
    pv = np.array(prefer, float) / np.linalg.norm(prefer)
    faces = [f for f in part.faces() if f.geom_type == GeomType.PLANE]
    faces = sorted((f for f in faces if np.dot(tuple(f.normal_at()), pv) > 0.9), key=lambda f: -f.area)
    for f in faces[:8]:
        n = np.array(tuple(f.normal_at()))
        fb = f.bounding_box()
        span = np.array([fb.size.X, fb.size.Y, fb.size.Z])
        order = np.argsort(span)[::-1]
        xdir = np.eye(3)[order[0]]
        if order[0] == 2 and abs(n[2]) < 0.5:
            xdir = np.eye(3)[order[1]]            # read along the long horizontal side when there is one
        xdir = xdir - np.dot(xdir, n) * n
        if np.linalg.norm(xdir) < 1e-6:
            continue
        xdir /= np.linalg.norm(xdir)
        if xdir[0] < 0:
            xdir = -xdir
        ydir = np.cross(n, xdir)
        c = np.array(tuple(f.center()))
        half = span.max() / 2
        cands = [(du, dv) for du in np.linspace(-half, half, 21) for dv in np.linspace(-half, half, 21)]
        cands.sort(key=lambda q: q[0] ** 2 + q[1] ** 2)      # prefer spots near the middle of the face
        for du, dv in cands:
            o = c + du * xdir + dv * ydir
            if f.distance_to(Vector(*o)) > 1e-3:
                continue
            pl = Plane(origin=tuple(o), x_dir=tuple(xdir), z_dir=tuple(n))
            if fits(f, pl, w, h):
                return pl
    return None


def safe_cut(part, cutter, name, what):
    """part - cutter, refusing cuts that remove more than the cutter itself could (i.e. open an internal void)."""
    hit = part & cutter
    inter = hit.volume if hit is not None else 0.0
    if inter < 0.5:
        print(f"  {name}: skipped {what} (nothing there)")
        return part
    removed = part.volume - (part - cutter).volume
    if abs(removed - inter) > 0.05 * max(inter, 1.0):
        print(f"  {name}: skipped {what} (inconsistent cut)")
        return part
    print(f"  {name}: {what}, removed {removed:.0f} mm3")
    return part - cutter


SLEEVED = {"jaw_left_dc": "left", "jaw_right_dc": "right"}


def save(name, shape, tol=0.02):
    save_part(name, shape, tol)


def sleeved(name, part):
    """The two variants of a handheld jaw (plain pad, and pad with the button), plus its strap and button cover."""
    side = SLEEVED[name]
    out = {}
    for button in (False, True):
        jaw, pl = sleeve.add_sleeve(part, side, button)
        out[name + ("_button" if button else "")] = jaw
    print(f"  {name}: finger-sleeve pad + strap fins, plain and with the button")
    save(f"sleeve_button_cover_{side}", sleeve.cover(pl), 0.01)
    save_reference(f"sleeve_strap_{side}", sleeve.strap(pl))              # bought: drawn for assemblies only
    return out


def restyle(name, part):
    if name in BADGE:
        for grooves in (True, False):
            pl = find_spot(part, BADGE[name], grooves)
            if pl is not None:
                part = safe_cut(part, badge(pl, grooves), name, "PG badge" if grooves else "PG logo")
                break
        else:
            print(f"  {name}: no room for the badge")
    return part


def main():
    for name, part in load_raw().items():
        if name.endswith("_envelope") or name.startswith("x5_"):     # no Insta360 frames in the D405 kit
            continue
        variants = sleeved(name, part) if name in SLEEVED else {name: part}
        for vname, v in variants.items():
            save(vname, restyle(vname, v))


if __name__ == "__main__":
    main()
