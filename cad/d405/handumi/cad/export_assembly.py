"""Export the full right-hand assembly (one per tip set) to assembly/ as STL (mm, one merged mesh) and GLB (metres,
one coloured node per part). The left hand is the mirror image.

    python export_assembly.py [--opening 0..1]
"""
import argparse

import numpy as np
import trimesh

from assembly import ROOT, assemble, best_orientation
from tips import TIP_SETS

OUT = ROOT / "assembly"
UP_Z = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])        # in use -Z is up -> +Z up (STL viewers)
UP_Y = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])    # -Z up -> +Y up (glTF convention)


def to_mesh(shape, colour):
    v, f = shape.tessellate(0.05, 0.2)
    m = trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)
    rgba = np.array([int(colour[i:i + 2], 16) for i in (1, 3, 5)] + [255], np.uint8)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
    return m


def export(ts, opening):
    parts = assemble(opening, tipset=(ts, best_orientation(ts)))
    meshes = {name: to_mesh(sh, col) for name, (sh, col) in parts.items()}
    stem = f"umi_right_{ts.folder.lower()}"
    merged = trimesh.util.concatenate(list(meshes.values()))
    merged.apply_transform(UP_Z)
    merged.export(OUT / f"{stem}.stl")
    scene = trimesh.Scene()
    S = np.diag([1e-3, 1e-3, 1e-3, 1.0]) @ UP_Y                        # mm -> m
    for name, m in meshes.items():
        scene.add_geometry(m, node_name=name, geom_name=name, transform=S)
    scene.export(OUT / f"{stem}.glb")
    print(f"{stem}: {len(meshes)} parts, {len(merged.faces)} faces")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--opening", type=float, default=0.5)
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    for ts in TIP_SETS:
        export(ts, a.opening)
