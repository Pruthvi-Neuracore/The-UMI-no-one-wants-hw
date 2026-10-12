"""Rigid placement and interference helpers on B-rep solids (OCC), for checks that need exact volumes."""
import numpy as np
from build123d import Location
from OCP.gp import gp_Trsf


def place(shape, mat4):
    m = np.asarray(mat4, float)
    t = gp_Trsf()
    t.SetValues(*m[0, :4], *m[1, :4], *m[2, :4])
    return shape.moved(Location(t))


def overlap(a, b):
    r = a & b
    return 0.0 if r is None else float(r.volume)
