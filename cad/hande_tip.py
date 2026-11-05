"""Robotiq Hand-E finger as a tip, with the Hand-E's own mount, and the rack shim for real Robotiq fingers.

Source: the Hand-E finger visual mesh (robotiq_hande_description, as used in Neuracore_Robots), copied to
source/hand_e/finger.dae. That mesh has the finger and, as separate bodies, a stretch of the rack and two screws; only
the finger is kept. Its frame: z along the finger (mating face on the rack at z = 1, lips down to z = 0 either side of
the 6 mm rack), x the closing axis (the other finger is at -x), the rack along x at y = -14.2.

Hand-E-Finger   the finger, remeshed watertight, with the Hand-E mount from Robotiq's drawing (manual 5.1.4):
                2x 3.4 mm holes 9 mm apart along the rack, a 2 mm dowel hole between them, and the 6 mm rack groove
                filled flush with the lips, so it sits flat on HandUMI's flange (z = 0). Both jaws use it, one
                turned 180 degrees, as on the Hand-E.
Hand-E-Rack-Shim  for a real Robotiq finger (aluminium, NBR, or a fingertip holder): its rack groove is 1 mm deep, so
                this 1 mm strip stands in for the rack top. Put it in the groove, then screw the finger on through it.

Every dimension comes from finger_interfaces.INTERFACES["hande"] and the constants below.

    python hande_tip.py      # -> ../hardware/{STEP,STL}/gripper_tips/Robotiq-Hand-E/ (the finger is mesh-derived: STL only)
"""
from pathlib import Path

import numpy as np
import trimesh
from build123d import Axis, Box, Cylinder, Pos, export_step, export_stl, fillet

import finger_interfaces as FI

SRC = Path(__file__).resolve().parent / "source/hand_e/finger.dae"
OUT = Path(__file__).resolve().parents[1] / "hardware"
CENTRE = (1.5, -14.2)          # hole-pattern centre (x, y) in the finger mesh frame (tips.py uses the same)
RACK_W, GROOVE = 6.0, 1.0      # Hand-E rack width (across), depth of the finger's groove over it
FILL_X = (-6.1, 8.0)           # the groove runs between the finger's lips over this x range
TAB_TOP = 5.0                  # the mounting tab's top face (screw heads sit here)
HEAD_D = 6.2                   # clearance for the M3 heads above the tab
SHIM_L = 16.0                  # shim length along the rack


def to_mesh(shape, tol=0.02):
    v, t = shape.tessellate(tol, 0.1)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(t))


def holes(depth_from, depth_to, d_hole, d_dowel=None):
    """Hand-E pattern cutters in the mesh frame (u = -x, so the u offsets map to -x)."""
    itf = FI.INTERFACES["hande"]
    cx, cy = CENTRE
    out = []
    h = depth_to - depth_from
    for u, v in itf.holes:
        out.append(Pos(cx - u, cy, (depth_from + depth_to) / 2) * Cylinder(d_hole / 2, h))
    if d_dowel:
        for u, v, d, depth in itf.dowels:
            out.append(Pos(cx - u, cy, depth / 2 - 0.01) * Cylinder((d + FI.DOWEL_FIT) / 2, depth))
    return out


def finger():
    m = trimesh.load(SRC, force="mesh")
    m.apply_scale(1000.0)
    m.merge_vertices()                                                    # the .dae has every face on its own
    m = max(m.split(only_watertight=False), key=lambda b: b.bounds[1][2])      # the finger (reaches z = 46.5)
    vox = m.voxelized(pitch=0.25).fill()                                  # watertight solid from the visual mesh
    f = vox.marching_cubes
    f.apply_transform(vox.transform)
    trimesh.smoothing.filter_taubin(f, iterations=10)
    cx, cy = CENTRE
    fill = Pos((FILL_X[0] + FILL_X[1]) / 2, cy, GROOVE / 2 + 0.25) * Box(FILL_X[1] - FILL_X[0], RACK_W + 0.6, GROOVE + 0.5)
    flat = trimesh.creation.box(extents=[200, 200, 200])                  # trim the voxel skin below the lips
    flat.apply_translation([0, 0, 100.0])
    f = trimesh.boolean.intersection([trimesh.boolean.union([f, to_mesh(fill)], engine="manifold"), flat],
                                     engine="manifold")
    tap, clear, head = FI.SCREW[FI.INTERFACES["hande"].screw]
    cut = holes(-1.0, TAB_TOP + 1.0, clear, d_dowel=True)
    for u, v in FI.INTERFACES["hande"].holes:                            # room for the heads above the tab
        cut.append(Pos(cx - u, cy, TAB_TOP + 10.0) * Cylinder(HEAD_D / 2, 20.0))
    for c in cut:
        f = trimesh.boolean.difference([f, to_mesh(c)], engine="manifold")
    import manifold3d                                                     # simplify but stay watertight
    mf = manifold3d.Manifold(manifold3d.Mesh(f.vertices.astype(np.float32), f.faces.astype(np.uint32))).simplify(0.05)
    out = mf.to_mesh()
    return trimesh.Trimesh(np.asarray(out.vert_properties)[:, :3], np.asarray(out.tri_verts))


def shim():
    """1 mm strip in the Robotiq finger's rack groove: the finger's own screws and dowel pass through it."""
    cx, cy = CENTRE
    s = Pos(cx, cy, GROOVE / 2) * Box(SHIM_L, RACK_W - 0.1, GROOVE)
    s = fillet(s.edges().filter_by(Axis.Z), 1.0)
    clear = FI.SCREW[FI.INTERFACES["hande"].screw][1]
    for c in holes(-1.0, GROOVE + 1.0, clear):
        s -= c
    for u, v, d, depth in FI.INTERFACES["hande"].dowels:
        s -= Pos(cx - u, cy, GROOVE / 2) * Cylinder((d + 0.2) / 2, GROOVE + 2)  # the dowel passes through freely
    return s


def main():
    for t in ("STL", "STEP"):
        (OUT / t / "gripper_tips/Robotiq-Hand-E").mkdir(parents=True, exist_ok=True)
    tip = finger()
    stl = OUT / "STL/gripper_tips/Robotiq-Hand-E/Hand-E-Finger.stl"
    tip.export(stl)
    print(stl.name, "watertight", tip.is_watertight, "bounds", tip.bounds.round(1).tolist())
    s = shim()
    export_step(s, str(OUT / "STEP/gripper_tips/Robotiq-Hand-E/Hand-E-Rack-Shim.step"))
    export_stl(s, str(OUT / "STL/gripper_tips/Robotiq-Hand-E/Hand-E-Rack-Shim.stl"), tolerance=0.01, angular_tolerance=0.1)
    print("Hand-E-Rack-Shim valid", s.is_valid, "volume", round(s.volume, 1))


if __name__ == "__main__":
    main()
