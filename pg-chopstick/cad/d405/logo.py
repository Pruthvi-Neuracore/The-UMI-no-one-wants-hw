"""PG logo as CAD geometry, traced from assets/pg_logo.png (alpha mask) into assets/pg_logo.json.

    python logo.py            # re-trace the PNG into JSON (once)
    logo_face(width_mm)       # build123d Face(s) in the XY plane, centred on the origin, reading +X / +Y
"""
import json
from pathlib import Path

ASSETS = Path(__file__).resolve().parent / "assets"


def trace(png=ASSETS / "pg_logo.png", out=ASSETS / "pg_logo.json", tol=0.9):
    import contourpy
    import matplotlib.pyplot as plt
    import numpy as np
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    a = plt.imread(png)[..., 3]
    a = np.pad(a, 2)
    lines = contourpy.contour_generator(z=a).lines(0.5)
    polys = [Polygon(l) for l in lines if len(l) > 8]
    # even-odd fill: XOR of all rings gives letters with their holes
    shape = polys[0]
    for p in polys[1:]:
        shape = shape.symmetric_difference(p)
    shape = unary_union(shape).simplify(tol)
    h, w = a.shape
    geoms = [shape] if shape.geom_type == "Polygon" else list(shape.geoms)
    data = {"width": w, "height": h, "polygons": [
        {"exterior": [[x, h - y] for x, y in g.exterior.coords],
         "holes": [[[x, h - y] for x, y in r.coords] for r in g.interiors]} for g in geoms if g.area > 20]}
    out.write_text(json.dumps(data))
    print(f"{len(data['polygons'])} polygons -> {out}")


def logo_face(width):
    from build123d import Face, Pos, Wire, Polyline
    d = json.loads((ASSETS / "pg_logo.json").read_text())
    k = width / d["width"]
    cx, cy = d["width"] / 2, d["height"] / 2
    faces = []
    for p in d["polygons"]:
        ring = lambda pts: Wire(Polyline(*[((x - cx) * k, (y - cy) * k) for x, y in pts], close=True).edges())
        faces.append(Face(ring(p["exterior"]), [ring(h) for h in p["holes"]]))
    out = faces[0]
    for f in faces[1:]:
        out = out + f
    return out


if __name__ == "__main__":
    trace()
