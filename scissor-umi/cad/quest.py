"""The Quest Touch Pro controller on the head: its pose in G and the cradle's bore.

The controller stands upright on the gripper's centre plane (y = 0) right behind the D405, its handle along G x (up),
held the way a hand holds it: the trigger and the head's ring point forward (+z, over the jaws), the thumbstick face
looks up and back at the operator. The cradle is a split collar around
the straight part of the handle (24 mm of it), bored to the handle's own section, clamped with one M3 screw.

The handle's section is measured from Meta's model (source/quest/, not redistributed) and cached in
source/quest/handle_section.json (an outline only), so the cradle builds without the model.
"""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
QUEST_STL = HERE / "source" / "quest" / "MetaQuestTouchPro_Right.stl"
SECTION_JSON = HERE / "source" / "quest" / "handle_section.json"

HANDLE_Z = -48.0                  # G z of the handle's centre: behind the camera, on the centre plane
HANDLE_X0 = 90.0                  # G x of the handle's bottom end (clear above the hand)
COLLAR_DZ = (6.0, 30.0)           # the collar grips the handle from 6 to 30 mm above its bottom end
FIT = 0.4                         # radial clearance in the bore (the clamp closes it)
# kit frame (X towards the thumbstick face, Y, Z up the handle) -> G: the face looks back (-z), the trigger forward
K = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])


def _kit_mesh():
    """Meta's model with the handle along +Z and the face along +X (handle bottom at z = 0, unplaced in x, y)."""
    import trimesh
    m = trimesh.load(QUEST_STL)
    v = m.vertices
    h = v[v[:, 1] > 15.0] - v[v[:, 1] > 15.0].mean(0)
    a = np.linalg.eigh(h.T @ h)[1][:, -1]
    a = a if a[2] > 0 else -a
    fwd = np.array([0.0, 1.0, 0.0])
    fwd = fwd - fwd.dot(a) * a
    fwd /= np.linalg.norm(fwd)
    R = np.eye(4)
    R[:3, :3] = np.column_stack([fwd, np.cross(a, fwd), a]).T
    m.apply_transform(R)
    # the handle's straight part exactly along z
    z0 = m.bounds[0, 2]
    c = []
    for dz in np.arange(15.0, 51.0, 5.0):
        loop = max(m.section([0, 0, 1], [0, 0, z0 + dz]).discrete, key=lambda l: np.ptp(l[:, 0]) * np.ptp(l[:, 1]))
        c.append([z0 + dz, (loop[:, 0].max() + loop[:, 0].min()) / 2, (loop[:, 1].max() + loop[:, 1].min()) / 2])
    c = np.array(c)
    d = np.array([np.polyfit(c[:, 0], c[:, 1], 1)[0], np.polyfit(c[:, 0], c[:, 2], 1)[0], 1.0])
    m.apply_transform(trimesh.geometry.align_vectors(d / np.linalg.norm(d), [0, 0, 1.0]))
    m.apply_translation([0, 0, -m.bounds[0, 2]])
    return m


def _measure():
    """The handle's outline over the collar's height (union of sections), in the kit frame, centred."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    m = _kit_mesh()
    polys = []
    for dz in np.arange(COLLAR_DZ[0], COLLAR_DZ[1] + 0.1, 2.0):
        s = m.section([0, 0, 1], [0, 0, dz])
        loop = max(s.discrete, key=lambda l: np.ptp(l[:, 0]) * np.ptp(l[:, 1]))
        polys.append(Polygon(loop[:, :2]))
    u = unary_union(polys).convex_hull.simplify(0.2)
    cx, cy = u.centroid.x, u.centroid.y
    pts = [(x - cx, y - cy) for x, y in u.exterior.coords[:-1]]
    return dict(outline_kit_xy=pts, centre_kit_xy=[cx, cy], height=float(m.extents[2]))


def section():
    if QUEST_STL.exists() and not SECTION_JSON.exists():
        SECTION_JSON.write_text(json.dumps(_measure(), indent=1))
    return json.loads(SECTION_JSON.read_text())


def outline_g(offset=0.0):
    """The handle's outline in the G (z, y) plane... returned as (y, z) points around the handle centre."""
    from shapely.geometry import Polygon
    p = Polygon(section()["outline_kit_xy"])
    if offset:
        p = p.buffer(offset, join_style=1)
    # kit x (towards the face) -> G -z; kit y -> G y (as K)
    return [(y, HANDLE_Z - x) for x, y in p.exterior.coords[:-1]]


def controller_mesh(side="right"):
    """The controller placed in G (for renders and checks)."""
    import trimesh
    if QUEST_STL.exists():
        m = _kit_mesh()
        cx, cy = section()["centre_kit_xy"]
        m.apply_translation([-cx, -cy, 0])
    else:
        m = trimesh.creation.capsule(height=70.0, radius=17.0)
        m.apply_translation([0, 0, 35])
    T = np.eye(4)
    T[:3, :3] = K
    T[:3, 3] = [HANDLE_X0, 0.0, HANDLE_Z]
    m.apply_transform(T)
    if side == "left":
        m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
        m.invert()
    return m


def collar_frame():
    """4 x 4 in G: the collar's centre (on the handle axis, mid-height), x up the handle, z forward."""
    T = np.eye(4)
    T[:3, 3] = [HANDLE_X0 + sum(COLLAR_DZ) / 2, 0.0, HANDLE_Z]
    return T
