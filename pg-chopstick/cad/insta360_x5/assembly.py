"""Assemble every PG Chopstick X5 build, check it, and export it.

    python assembly.py [--open 0.45] [--check] [--openarm /path/to/openarm_description]

Builds (parts.BUILDS): chopstick_{right,left,openarm,iso50} and hand (the parallel gripper: t_gripper.py).
--open: how far the jaws / fingers are open in the exported assemblies (fraction of a typical working opening).
--check: every moving part against every fixed part at that opening, and every pair of fixed parts; with --openarm,
also the OpenArm builds against the link7 mesh and the adapter's holes against link7's PCD-50 holes.
Exports assemblies/<build>.{glb,stl} (STL: one mesh in mm, Z up; GLB: one coloured node per part, metres, Y up).
"""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np
import trimesh

import openarm_adapter
from kinematics import OUT as KIN_JSON, overlap_cm3, to_mesh
from parts import BUILDS, CAMERA, ROBOT_BUILDS, ROOT, family, load_parts, motion, q_display, side_of

OUT = ROOT / "assemblies"
# PG blue and white: structure blue, moving and hand-touched parts white, blue tracking dots; the record button's
# cover is green and the cancel button's red
COLOURS = {"blue": "#1f5fae", "white": "#f4f6f8", "tpu": "#dfe5ec", "camera": "#1d1f22", "servo": "#2a2d33",
           "dot": "#1e88ff", "steel": "#b9bec5",
           "record": "#2e9e44", "cancel": "#d62828", "strap": "#2c5aa0"}
WHITE = ("jaw_", "pg_pod", "pg_wrist")
BUTTON_COVERS = ("sleeve_button_cover_",)
STRAPS = ("sleeve_strap_",)


def colour_of(name):
    if name.startswith("tip_tpu"):
        return COLOURS["tpu"]
    if name == "insta360_x5_envelope":
        return COLOURS["camera"]
    if name.startswith("servo"):
        return COLOURS["servo"]
    if name.startswith(BUTTON_COVERS):              # green: record (right hand), red: cancel (left hand)
        return COLOURS["record"] if name.endswith("_right") else COLOURS["cancel"]
    if name.startswith(STRAPS):
        return COLOURS["strap"]
    return COLOURS["white"] if name.startswith(WHITE) else COLOURS["blue"]


def paint(m, colour):
    rgba = np.array([int(colour[i:i + 2], 16) for i in (1, 3, 5)] + [255], np.uint8)
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(rgba, (len(m.faces), 1)))
    return m


def dot_meshes(build, q):
    """Blue discs at the jaws' / fingers' tracking dots (for renders and the GLB)."""
    out = {}
    if family(build) == "chopstick":
        kin = json.loads(KIN_JSON.read_text())
        jaw_left = "jaw_left_sa" if build in ROBOT_BUILDS else "jaw_left_dc"
        for name, side in ((jaw_left, "left"), ("jaw_right", "right")):
            for i, d in enumerate(kin["red_dots"][name]):
                c = trimesh.creation.cylinder(radius=1.75, height=0.3)
                n = np.array(d["normal"])
                R = trimesh.geometry.align_vectors([0, 0, 1.0], n)
                R[:3, 3] = np.array(d["centre_mm"]) + n * 0.1
                c.apply_transform(motion(build, f"tip_tpu_{side}", q) @ R)
                out[f"dot_{side}_{i}"] = paint(c, COLOURS["dot"])
    return out


def servo(build, parts):
    """The robot's servo envelope, on the chopstick driver jaw's pivot."""
    m = to_mesh(parts["servo_envelope"], 0.1)
    return m


def assemble(build, frac=0.45, parts=None, dots=True, q=None):
    """{name: coloured trimesh in the gripper frame (mm)} for one build, opened `frac` of a working opening
    (or exactly q, rad per jaw)."""
    parts = parts or load_parts()
    q = q_display(build, frac) if q is None else q
    out = {}
    for n in BUILDS[build] + [CAMERA[build]]:
        m = to_mesh(parts[n], 0.05).apply_transform(motion(build, n, q))
        out[n] = paint(m, colour_of(n))
    if build in ROBOT_BUILDS:
        out["servo_envelope"] = paint(servo(build, parts), COLOURS["servo"])
    if dots:
        out.update(dot_meshes(build, q))
    return out


def link7_in_gripper(openarm_dir):
    """OpenArm right link7 visual mesh in the gripper frame (mm)."""
    m = trimesh.load(Path(openarm_dir) / "meshes/arm/v10/visual/link7.stl", force="mesh")
    m.apply_translation([0, 0, -558.5])                   # mesh is in the arm's zero pose; link7 origin at z = 558.5
    T = np.diag([1e3, 1e3, 1e3, 1.0]) @ np.linalg.inv(openarm_adapter.T_LINK7) @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
    return m.apply_transform(T)


# pairs that touch by design: the servo in its pocket
EXPECTED = [({"chopstick_base_sa", "servo_envelope"})]


def expected(p, q):
    if {p, q} in EXPECTED:
        return True
    return False


def check(frac, openarm_dir):
    parts = load_parts()
    ok = True
    for build in BUILDS:
        a = assemble(build, frac, parts, dots=False)
        fixed = [n for n in a if side_of(n) is None]
        moving = [n for n in a if side_of(n)]
        worst = 0.0
        pairs = list(itertools.product(moving, fixed)) + list(itertools.combinations(fixed, 2)) + \
            list(itertools.combinations(moving, 2))
        for p, q in pairs:
            if expected(p, q) or (side_of(p) and side_of(p) == side_of(q)):
                continue
            v = overlap_cm3(a[p], a[q])
            worst = max(worst, v)
            if v > 0.02:
                ok = False
                print(f"  CLASH {build}: {p} x {q}: {v:.3f} cm3")
        print(f"  {build:18s} {len(a):2d} parts, largest overlap {worst:.3f} cm3")
    if openarm_dir:
        l7 = link7_in_gripper(openarm_dir)
        for build in [b for b in ROBOT_BUILDS if b.endswith("_openarm")]:
            for n, m in assemble(build, frac, parts, dots=False).items():
                if not side_of(n) and overlap_cm3(m, l7) > 0.02:
                    ok = False
                    print(f"  CLASH {build}: {n} x link7")
        for y, z in openarm_adapter.FLANGE_HOLES:
            p = np.array([[-openarm_adapter.T - 2.0, y, z]])
            hit = trimesh.proximity.closest_point(l7, p)[1][0]
            print(f"  adapter hole ({y:6.2f}, {z:6.2f}): link7 surface {hit:.2f} mm away (1.25 = on a tapped hole)")
    print("check:", "OK" if ok else "FAILED")
    return ok


def export(frac):
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*"):
        old.unlink()
    parts = load_parts()
    for build in BUILDS:
        meshes = assemble(build, frac, parts)
        merged = trimesh.util.concatenate(list(meshes.values()))
        merged.export(OUT / f"{build}.stl")
        scene = trimesh.Scene()
        S = np.diag([1e-3, 1e-3, 1e-3, 1.0])
        up_y = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
        for name, m in meshes.items():
            scene.add_geometry(m, node_name=name, geom_name=name, transform=up_y @ S)
        scene.export(OUT / f"{build}.glb")
        print(f"{build}: {len(meshes)} nodes, {len(merged.faces)} faces")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--open", type=float, default=0.45)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--openarm", help="path to openarm_description (for the link7 checks)")
    a = ap.parse_args()
    if a.check:
        check(a.open, a.openarm)
    export(a.open)
