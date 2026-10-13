"""Bending check of the hard part along the finger: section modulus Z = I / c of each cross-section, in the closing
direction (z) and across (y), against a crash load at the fingertip.

    python check_strength.py [STL] [--load 100] [--sf 1.5] [--stress 45]

Requirement at x: Z >= load * (tip - x) * sf / stress (PETG, along the layers, about 45 MPa).
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh
from shapely import contains_xy
from shapely.geometry import Polygon


def section_moduli(m, x, h=0.25):
    s = m.section([1, 0, 0], [x, 0, 0])
    if s is None:
        return None
    shape = None
    for p in sorted((Polygon(l[:, 1:]) for l in s.discrete), key=lambda p: -p.area):
        shape = p if shape is None else (shape.difference(p) if shape.contains(p) else shape.union(p))
    y0, z0, y1, z1 = shape.bounds
    ys, zs = np.meshgrid(np.arange(y0, y1, h), np.arange(z0, z1, h))
    inside = contains_xy(shape, ys, zs)
    Y, Z = ys[inside], zs[inside]
    yc, zc = Y.mean(), Z.mean()
    area = inside.sum() * h * h
    z_close = ((Z - zc) ** 2).sum() * h * h / np.abs(Z - zc).max()
    z_side = ((Y - yc) ** 2).sum() * h * h / np.abs(Y - yc).max()
    return area, z_close, z_side


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl", nargs="?", default=str(Path(__file__).resolve().parents[1] /
                                                  "hardware/STL/hardshell_finger_left.stl"))
    ap.add_argument("--load", type=float, default=100.0, help="N at the fingertip")
    ap.add_argument("--sf", type=float, default=1.5)
    ap.add_argument("--stress", type=float, default=45.0, help="MPa")
    a = ap.parse_args()
    m = trimesh.load(a.stl)
    tip = m.bounds[1][0]
    print("    x   area   Z_close  Z_side  Z_needed  (mm2, mm3)")
    worst = np.inf
    for x in np.arange(3.0, tip - 3.0, 5.0):
        r = section_moduli(m, x)
        if r is None:
            continue
        need = a.load * (tip - x) * a.sf / a.stress
        worst = min(worst, min(r[1], r[2]) / need)
        print(f"{x:5.0f} {r[0]:6.0f} {r[1]:8.0f} {r[2]:7.0f} {need:8.0f}")
    print(f"weakest: {worst:.1f} x what is needed ({a.load:.0f} N at the tip, safety factor {a.sf}, {a.stress:.0f} MPa)")


if __name__ == "__main__":
    main()
