"""Assemble the chopstick data gripper and the OpenArm robot gripper at a jaw angle, check them, and export them.

    python assembly.py [--theta-deg 20] [--check] [--openarm /path/to/openarm_description]

--check: clash checks (jaws vs the fixed parts at the given angle, adapter vs the X5 frame and clip B) and, with
--openarm, the adapter against the link7 mesh and its holes against link7's PCD-50 holes.
Exports to chopstick/assembly/: chopstick_data.{glb,stl} and chopstick_openarm.{glb,stl} (STL: one mesh in mm, Z up;
GLB: one coloured node per part, metres, Y up).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import trimesh

import openarm_adapter
from kinematics import OUT as KIN_JSON, overlap_cm3, to_mesh
from kiwi import DATA_GRIPPER, ROBOT_GRIPPER, ROOT, jaw_matrix, load_parts, side_of

OUT = ROOT / "assembly"
COLOURS = {"printed": "#3c4046", "tpu": "#9aa0a8", "camera": "#1d1f22", "servo": "#2a4d7a",
           "adapter": "#5b6068", "dot": "#d62828"}
UP_Y = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])   # Z up -> Y up (glTF)


def colour_of(name):
    if name.startswith("tip_tpu"):
        return COLOURS["tpu"]
    if name == "insta360_x5_envelope":
        return COLOURS["camera"]
    if name == "servo_envelope":
        return COLOURS["servo"]
    if name == "openarm_adapter":
        return COLOURS["adapter"]
    return COLOURS["printed"]


def paint(m, colour):
    rgba = np.array([int(colour[i:i + 2], 16) for i in (1, 3, 5)] + [255], np.uint8)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
    return m


def dot_meshes(theta, jaw_left):
    """Small red discs at each jaw's red-dot centres (for renders and the GLB), at jaw angle theta."""
    kin = json.loads(KIN_JSON.read_text())
    out = {}
    for name, side in ((jaw_left, "left"), ("jaw_right", "right")):
        for i, d in enumerate(kin["red_dots"][name]):
            c = trimesh.creation.cylinder(radius=1.75, height=0.3)
            z = np.array([0, 0, 1.0])
            n = np.array(d["normal"])
            R = trimesh.geometry.align_vectors(z, n)
            R[:3, 3] = np.array(d["centre_mm"]) + n * 0.1
            c.apply_transform(jaw_matrix(side, theta) @ R)
            out[f"dot_{side}_{i}"] = paint(c, COLOURS["dot"])
    return out


def assemble(build, theta, parts=None, dots=True):
    """{name: coloured trimesh in the gripper frame (mm)} for 'data' or 'openarm' at jaw angle theta (rad)."""
    parts = parts or load_parts()
    names = DATA_GRIPPER if build == "data" else ROBOT_GRIPPER
    out = {}
    for n in names + ["insta360_x5_envelope"] + (["servo_envelope"] if build == "openarm" else []):
        shape = openarm_adapter.build() if n == "openarm_adapter" else parts[n]
        m = to_mesh(shape, 0.05)
        side = side_of(n)
        if side:
            m.apply_transform(jaw_matrix(side, theta))
        out[n] = paint(m, colour_of(n))
    if dots:
        out.update(dot_meshes(theta, "jaw_left_dc" if build == "data" else "jaw_left_sa"))
    return out


def link7_in_gripper(openarm_dir):
    """OpenArm right link7 visual mesh in the gripper frame (mm)."""
    m = trimesh.load(Path(openarm_dir) / "meshes/arm/v10/visual/link7.stl", force="mesh")
    m.apply_translation([0, 0, -558.5])                   # mesh is in the arm's zero pose; link7 origin at z = 558.5
    S = np.diag([1e3, 1e3, 1e3, 1.0])
    T = S @ np.linalg.inv(openarm_adapter.T_LINK7) @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
    return m.apply_transform(T)


def check(theta, openarm_dir):
    parts = load_parts()
    ok = True
    for build in ("data", "openarm"):
        a = assemble(build, theta, parts, dots=False)
        fixed = [n for n in a if side_of(n) is None and n not in ("servo_envelope",)]
        moving = [n for n in a if side_of(n)]
        for mv in moving:
            for fx in fixed:
                v = overlap_cm3(a[mv], a[fx])
                if v > 0.02:
                    ok = False
                    print(f"  CLASH {build}: {mv} x {fx}: {v:.3f} cm3")
        if build == "openarm":
            for fx in ("x5_frame", "x5_clip_b", "chopstick_base_sa", "insta360_x5_envelope"):
                v = overlap_cm3(a["openarm_adapter"], a[fx])
                print(f"  adapter x {fx}: {v:.3f} cm3")
                ok &= v < 0.02
    if openarm_dir:
        l7 = link7_in_gripper(openarm_dir)
        ad = to_mesh(openarm_adapter.build(), 0.05)
        v = overlap_cm3(ad, l7)
        print(f"  adapter x link7: {v:.3f} cm3")
        ok &= v < 0.05
        # every adapter flange hole should sit on a link7 hole: probe just behind the back face
        for y, z in openarm_adapter.FLANGE_HOLES:
            p = np.array([[-openarm_adapter.T - 2.0, y, z]])
            inside = l7.contains(p)[0] if l7.is_watertight else None
            hit = trimesh.proximity.closest_point(l7, p)[1][0]
            print(f"  hole ({y:6.2f}, {z:6.2f}): link7 surface {hit:.2f} mm away"
                  + ("" if inside is None else f", inside solid: {inside}"))
    print("check:", "OK" if ok else "FAILED")
    return ok


def export(theta):
    OUT.mkdir(exist_ok=True)
    parts = load_parts()
    for build in ("data", "openarm"):
        meshes = assemble(build, theta, parts)
        merged = trimesh.util.concatenate(list(meshes.values()))
        stem = f"chopstick_{build}"
        merged.export(OUT / f"{stem}.stl")
        scene = trimesh.Scene()
        S = np.diag([1e-3, 1e-3, 1e-3, 1.0])
        for name, m in meshes.items():
            scene.add_geometry(m, node_name=name, geom_name=name, transform=UP_Y @ S)
        scene.export(OUT / f"{stem}.glb")
        print(f"{stem}: {len(meshes)} nodes, {len(merged.faces)} faces")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--theta-deg", type=float, default=20.0)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--openarm", help="path to openarm_description (for the link7 checks)")
    a = ap.parse_args()
    th = math.radians(a.theta_deg)
    if a.check:
        check(th, a.openarm)
    export(th)
