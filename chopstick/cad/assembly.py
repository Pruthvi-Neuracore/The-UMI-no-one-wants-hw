"""Assemble the PG Chopstick builds at a jaw angle, check them, and export them.

    python assembly.py [--theta-deg 20] [--check] [--openarm /path/to/openarm_description]

Builds (parts.BUILDS): data (handheld, Insta360 X5 + PG pod), data_d405 (handheld, RealSense D405),
openarm / openarm_d405 (OpenArm adapter) and iso50 / iso50_d405 (ISO 9409-1-50-4-M6 flange plate).
--check: every moving part against every fixed part at the given angle, and every pair of fixed parts; with
--openarm, also the OpenArm builds against the link7 mesh and the adapter's holes against link7's PCD-50 holes.
Exports assembly/pg_chopstick_<build>.{glb,stl} (STL: one mesh in mm, Z up; GLB: one coloured node per part, metres,
Y up).
"""
import argparse
import itertools
import json
import math
from pathlib import Path

import numpy as np
import trimesh

import openarm_adapter
from kinematics import OUT as KIN_JSON, overlap_cm3, to_mesh
from parts import BUILDS, CAMERA, ROOT, jaw_matrix, load_parts, side_of

OUT = ROOT / "assembly"
# PG blue and white: structure blue, moving and hand-touched parts white, red dots for the vision
COLOURS = {"blue": "#1f5fae", "white": "#f4f6f8", "tpu": "#dfe5ec", "camera": "#1d1f22", "servo": "#2a2d33",
           "dot": "#d62828", "button": "#1f5fae"}
WHITE_PARTS = {"jaw_left_dc", "jaw_left_sa", "jaw_right", "pg_pod", "pg_pod_lid"}
UP_Y = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])   # Z up -> Y up (glTF)


def colour_of(name):
    if name.startswith("tip_tpu"):
        return COLOURS["tpu"]
    if name in ("insta360_x5_envelope", "d405_envelope"):
        return COLOURS["camera"]
    if name == "servo_envelope":
        return COLOURS["servo"]
    if name == "pg_pod_button":
        return COLOURS["button"]
    return COLOURS["white"] if name in WHITE_PARTS else COLOURS["blue"]


def paint(m, colour):
    rgba = np.array([int(colour[i:i + 2], 16) for i in (1, 3, 5)] + [255], np.uint8)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
    return m


def dot_meshes(theta, jaw_left):
    """Red discs at each jaw's red-dot centres (for renders and the GLB), at jaw angle theta."""
    kin = json.loads(KIN_JSON.read_text())
    out = {}
    for name, side in ((jaw_left, "left"), ("jaw_right", "right")):
        for i, d in enumerate(kin["red_dots"][name]):
            c = trimesh.creation.cylinder(radius=1.75, height=0.3)
            n = np.array(d["normal"])
            R = trimesh.geometry.align_vectors([0, 0, 1.0], n)
            R[:3, 3] = np.array(d["centre_mm"]) + n * 0.1
            c.apply_transform(jaw_matrix(side, theta) @ R)
            out[f"dot_{side}_{i}"] = paint(c, COLOURS["dot"])
    return out


def is_robot(build):
    return not build.startswith("data")


def assemble(build, theta, parts=None, dots=True):
    """{name: coloured trimesh in the gripper frame (mm)} for one build at jaw angle theta (rad)."""
    parts = parts or load_parts()
    names = BUILDS[build] + [CAMERA[build]] + (["servo_envelope"] if is_robot(build) else [])
    out = {}
    for n in names:
        m = to_mesh(parts[n], 0.05)
        side = side_of(n)
        if side:
            m.apply_transform(jaw_matrix(side, theta))
        out[n] = paint(m, colour_of(n))
    if dots:
        out.update(dot_meshes(theta, "jaw_left_sa" if is_robot(build) else "jaw_left_dc"))
    return out


def link7_in_gripper(openarm_dir):
    """OpenArm right link7 visual mesh in the gripper frame (mm)."""
    m = trimesh.load(Path(openarm_dir) / "meshes/arm/v10/visual/link7.stl", force="mesh")
    m.apply_translation([0, 0, -558.5])                   # mesh is in the arm's zero pose; link7 origin at z = 558.5
    T = np.diag([1e3, 1e3, 1e3, 1.0]) @ np.linalg.inv(openarm_adapter.T_LINK7) @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
    return m.apply_transform(T)


def check(theta, openarm_dir):
    parts = load_parts()
    ok = True
    for build in BUILDS:
        a = assemble(build, theta, parts, dots=False)
        fixed = [n for n in a if side_of(n) is None]
        moving = [n for n in a if side_of(n)]
        worst = 0.0
        pairs = list(itertools.product(moving, fixed)) + list(itertools.combinations(fixed, 2))
        for p, q in pairs:
            if {p, q} == {"chopstick_base_sa", "servo_envelope"}:
                continue                                   # the servo sits in the base's pocket by design
            v = overlap_cm3(a[p], a[q])
            worst = max(worst, v)
            if v > 0.02:
                ok = False
                print(f"  CLASH {build}: {p} x {q}: {v:.3f} cm3")
        print(f"  {build:13s} {len(a):2d} parts, largest overlap {worst:.3f} cm3")
    if openarm_dir:
        l7 = link7_in_gripper(openarm_dir)
        for build in ("openarm", "openarm_d405"):
            for n in BUILDS[build] + [CAMERA[build]]:
                if not side_of(n):
                    v = overlap_cm3(to_mesh(parts[n], 0.1), l7)
                    if v > 0.02:
                        ok = False
                        print(f"  CLASH {build}: {n} x link7: {v:.3f} cm3")
        for y, z in openarm_adapter.FLANGE_HOLES:
            p = np.array([[-openarm_adapter.T - 2.0, y, z]])
            hit = trimesh.proximity.closest_point(l7, p)[1][0]
            print(f"  adapter hole ({y:6.2f}, {z:6.2f}): link7 surface {hit:.2f} mm away (1.25 = on a tapped hole)")
    print("check:", "OK" if ok else "FAILED")
    return ok


def export(theta):
    OUT.mkdir(exist_ok=True)
    for old in list(OUT.glob("chopstick_*")) + list(OUT.glob("pg_chopstick_*")):
        old.unlink()
    parts = load_parts()
    for build in BUILDS:
        meshes = assemble(build, theta, parts)
        merged = trimesh.util.concatenate(list(meshes.values()))
        stem = f"pg_chopstick_{build}"
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
