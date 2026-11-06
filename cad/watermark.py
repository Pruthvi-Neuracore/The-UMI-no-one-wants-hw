"""Engrave a small PG logo on every printed part (a design mark), except gripper tips and the TPU button cover.

For each part, the largest flat face where an 8 mm logo fits with a margin (clear of holes and edges) is chosen, and the
logo is cut 0.2 mm deep there. B-rep parts (STEP) are engraved exactly and re-exported to STEP + STL.
Parts that already carry the mark (main_support, on its end wall) are skipped.

    python watermark.py          # run after restyle.py and camera.py
"""
from pathlib import Path

import numpy as np
from build123d import (Axis, GeomType, Location, Plane, Pos, Vector, export_step, export_stl, extrude, import_step)

from logo import logo_face

HW = Path(__file__).resolve().parents[1] / "hardware"
WIDTH, DEPTH, MARGIN = 5.0, 0.2, 1.0
SKIP = {"main_support", "button_cover_tpu"}
# visible face for each part (part frame); left-hand finger links are mirrored, so their preference flips in x
PREFER = {"end_cover": (0, 1, 0), "electronics_lid": (0, 0, 1), "electronics_box": (1, 0, 0),
          "crank_mechanism_plate": (0, -1, 0), "connecting_link_1": (0, -1, 0), "connecting_link_2": (0, -1, 0),
          "hand_support_base": (0, 0, 1), "right_controller_support": (1, 0, 0), "left_controller_support": (-1, 0, 0),
          "right_thumb_link": (1, 0, 0), "left_thumb_link": (-1, 0, 0),
          "right_index_middle_finger_link": (-1, 0, 0), "left_index_middle_finger_link": (1, 0, 0),
          "d405_hood": (1, 0, 0)}


def fits(face, plane, w, h):
    """True if a w x h rectangle (plus margin), laid out in `plane`, lies entirely on `face`."""
    for u in np.linspace(-w / 2 - MARGIN, w / 2 + MARGIN, 7):
        for v in np.linspace(-h / 2 - MARGIN, h / 2 + MARGIN, 5):
            p = plane.from_local_coords((u, v, 0))
            if face.distance_to(Vector(p)) > 1e-3:
                return False
    return True


def find_spot(part, prefer=None):
    logo = logo_face(WIDTH)
    bb = logo.bounding_box()
    w, h = bb.size.X, bb.size.Y
    faces = sorted((f for f in part.faces() if f.geom_type == GeomType.PLANE), key=lambda f: -f.area)
    if prefer is not None:
        pv = np.array(prefer, float)
        good = [f for f in faces if np.dot([f.normal_at().X, f.normal_at().Y, f.normal_at().Z], pv) > 0.9]
        faces = good + [f for f in faces if f not in good]
    for f in faces[:40]:
        n = f.normal_at()
        fb = f.bounding_box()
        span = np.array([fb.size.X, fb.size.Y, fb.size.Z])
        xdir = np.eye(3)[int(np.argmax(span))]
        nv = np.array([n.X, n.Y, n.Z])
        xdir = xdir - np.dot(xdir, nv) * nv
        if np.linalg.norm(xdir) < 1e-6:
            continue
        c = f.center()
        for du in np.linspace(-0.35, 0.35, 5) * max(span):
            for dv in np.linspace(-0.35, 0.35, 5) * sorted(span)[-2]:
                pl = Plane(origin=c, x_dir=tuple(xdir), z_dir=n)
                o = pl.from_local_coords((du, dv, 0))
                pl = Plane(origin=o, x_dir=tuple(xdir), z_dir=n)
                if fits(f, pl, w, h):
                    return pl
    return None


SRC = Path(__file__).resolve().parent / "source"


def engrave(path):
    # parts used unchanged from the base design are engraved from their clean originals, so re-runs don't stack marks
    clean = SRC / path.parent.name / path.name
    part = import_step(str(clean if clean.exists() else path))
    pl = find_spot(part, PREFER.get(path.stem))
    if pl is None:
        return False
    cut = pl * Pos(0, 0, -DEPTH) * extrude(logo_face(WIDTH), DEPTH + 0.5)
    out = part - cut
    export_step(out, str(path))
    export_stl(out, str(path.parents[2] / "STL" / path.parent.name / (path.stem + ".stl")), tolerance=0.02, angular_tolerance=0.15)
    return True


def main():
    for side in ("right", "left"):
        for f in sorted((HW / "STEP" / side).glob("*.step")):
            if f.stem in SKIP:
                continue
            print(f"{side}/{f.stem:32s}", "engraved" if engrave(f) else "NO SPOT FOUND")


if __name__ == "__main__":
    main()
