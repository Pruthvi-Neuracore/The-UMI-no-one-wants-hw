"""Measure the PG Chopstick's kinematics from the CAD and write them for the URDF and the retargeting code.

Writes chopstick/retarget/pg_chopstick/pg_chopstick_kinematics.json with:
  - the pivot axes and gear coupling,
  - the tool-tip (TCP) point where the TPU tips meet when closed,
  - the D405 camera frame (front-face centre, at the default tilt, with the hinge axis) and camera -> TCP,
  - the red-dot centres on each jaw, in that jaw's closed pose (from the painted slicer project), also written as
    hardware/STL/<jaw>_dots.stl inlay bodies,
  - a table of jaw angle -> tip gap (distance between the TPU tip ends) and the angle limits,
  - the robot mounts: pose of the gripper frame in the robot's flange frame, for the OpenArm adapter and the ISO 50
    flange plate (handheld and robot grippers share the camera -> TCP extrinsics, so this is the only robot-specific
    transform).

    python kinematics.py [--check]   (--check also sweeps the jaws for clashes with the base)
"""
import argparse
import json
import math

import manifold3d as mf
import numpy as np
import trimesh
from build123d import GeomType
from OCP.BRepAdaptor import BRepAdaptor_Surface
from scipy.sparse.csgraph import connected_components

from extract_parts import THREEMF, threemf_meshes
from parts import (BUILDS, GEAR_MODULE, GEAR_TEETH, PIVOT_L, PIVOT_R, ROOT, SA_MODEL_OPEN_DEG, family, jaw_matrix,
                   motion, part_path, q_max,
                   load_parts, load_raw, side_of)
from iso_flange_plate import T_FLANGE
from openarm_adapter import T_LINK7

OUT = ROOT / "hardware" / "kinematics.json"
JAW_3MF = {"jaw_left_dc": "spur_bearing_test_7_datacollection parts - Left Finger DC Spur Gear 24 teeth 1%",
           "jaw_right": "spur_bearing_test_7_datacollection parts - Right Finger DC-SA Spur Gear 24 teeth 1%",
           "jaw_left_sa": "Chopsticks - Left Finger SA Spur Gear 24 teeth 1%"}


def to_mesh(shape, tol=0.05):
    if isinstance(shape, trimesh.Trimesh):          # mesh-only parts (the D405 cup from the HandUMI build)
        return shape.copy()
    v, f = shape.tessellate(tol, 0.2)
    return trimesh.Trimesh(np.array([(q.X, q.Y, q.Z) for q in v]), np.array(f), process=True)


def to_manifold(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                               tri_verts=np.asarray(m.faces, np.uint32)))


def overlap_cm3(a, b):
    return (to_manifold(a) ^ to_manifold(b)).volume() / 1e3


def lens_centres(x5):
    """Centres of the two spherical lens domes on the X5 envelope, front (+X) first."""
    cs = []
    for f in x5.faces():
        if f.geom_type == GeomType.SPHERE:
            s = BRepAdaptor_Surface(f.wrapped).Sphere()
            c = s.Location()
            cs.append(((c.X(), c.Y(), c.Z()), s.Radius()))
    cs.sort(key=lambda c: -c[0][0])
    return cs


def dot_centres(step_jaw, painted):
    """Register the painted 3MF jaw onto the STEP jaw (closed pose) and return each red dot's centre and normal."""
    target = step_jaw
    src = painted.copy()
    # Coarse alignment by principal axes (tried with every sign flip), then ICP on the surfaces.
    best = None
    for flips in [np.diag([a, b, a * b, 1]) for a in (1, -1) for b in (1, -1)]:
        init = target.principal_inertia_transform.copy()
        init = np.linalg.inv(init) @ flips @ src.principal_inertia_transform
        m, _, cost = trimesh.registration.icp(src.sample(4000), target, initial=init, max_iterations=60,
                                              scale=False)
        if best is None or cost < best[1]:
            best = (m, cost)
    m, cost = best
    src.apply_transform(m)
    paint = painted.metadata["paint"]
    idx = np.flatnonzero(paint != "")
    sub = src.submesh([idx], append=True)
    adj = trimesh.graph.face_adjacency(sub.faces)
    n = len(sub.faces)
    from scipy.sparse import coo_matrix
    g = coo_matrix((np.ones(len(adj)), (adj[:, 0], adj[:, 1])), shape=(n, n))
    k, lab = connected_components(g, directed=False)
    dots = []
    for i in range(k):
        sel = lab == i
        area = sub.area_faces[sel]
        if area.sum() < 0.5:
            continue
        c = (sub.triangles_center[sel] * area[:, None]).sum(0) / area.sum()
        nrm = (sub.face_normals[sel] * area[:, None]).sum(0)
        dots.append({"centre_mm": np.round(c, 3).tolist(), "normal": np.round(nrm / np.linalg.norm(nrm), 4).tolist(),
                     "area_mm2": round(float(area.sum()), 2)})
    dots.sort(key=lambda d: d["centre_mm"][0])
    rms = float(np.sqrt(np.mean(trimesh.proximity.closest_point(target, src.sample(3000))[1] ** 2)))
    return dots, rms


def write_dot_inlays(jaw, dots, depth=0.6):
    """The red dots as separate bodies (STL), 0.6 mm deep into the jaw's top face, for any multi-material slicer:
    load the jaw, add this file as a part/modifier and give it the red filament."""
    discs = []
    for d in dots:
        r = math.sqrt(d["area_mm2"] / math.pi)
        n = np.array(d["normal"])
        c = trimesh.creation.cylinder(radius=r, height=depth)
        T = trimesh.geometry.align_vectors([0, 0, 1], n)
        T[:3, 3] = np.array(d["centre_mm"]) - n * depth / 2
        discs.append(c.apply_transform(T))
    trimesh.util.concatenate(discs).export(part_path(f"{jaw}_dots", "stl"))


def tip_end(tip):
    """Distal end of a TPU tip: the most forward vertex, and the tip's z centre there."""
    v = tip.vertices
    end = v[v[:, 0].argmax()]
    near = v[v[:, 0] > end[0] - 1.0]
    return np.array([end[0], end[1], (near[:, 2].min() + near[:, 2].max()) / 2])


def gap_table(tl, tr, theta_max_deg=70.0, step_deg=0.5):
    """Jaw angle -> distance between the two TPU tip ends (negative while the tips overlap)."""
    el, er = tip_end(tl), tip_end(tr)
    rows = []
    for deg in np.arange(0.0, theta_max_deg + 1e-9, step_deg):
        th = math.radians(deg)
        pl = (jaw_matrix("left", th) @ [*el, 1])[:3]
        pr = (jaw_matrix("right", th) @ [*er, 1])[:3]
        rows.append([round(float(deg), 2), round(float(np.copysign(np.linalg.norm(pl - pr), pl[1] - pr[1])), 3)])
    return rows


def clash_sweep(parts, build, steps=36):
    """First opening (rad for chopstick, mm for parallel) at which a moving part overlaps a fixed part by more than
    0.05 cm3, sweeping from closed to q_max(build); also the overlap at the closed pose."""
    names = BUILDS[build]
    fixed = [n for n in names if side_of(n) is None]
    moving = [n for n in names if side_of(n) not in (None, "pinion")]
    base = to_manifold(to_mesh(parts[fixed[0]], 0.1))
    for n in fixed[1:]:
        base += to_manifold(to_mesh(parts[n], 0.1))
    base = base.to_mesh()
    base = trimesh.Trimesh(base.vert_properties[:, :3], base.tri_verts)
    meshes = {n: to_mesh(parts[n], 0.1) for n in moving}

    def overlap(q):
        return sum(overlap_cm3(base, m.copy().apply_transform(motion(build, n, q))) for n, m in meshes.items())

    rest = overlap(0.0)
    qs = np.linspace(0.0, q_max(build), steps + 1)[1:]
    prev = 0.0
    for q in qs:
        if overlap(q) - rest > 0.05:
            return float(prev), rest
        prev = q
    return float(q_max(build)), rest


def main(check):
    parts = load_parts()
    tl, tr = to_mesh(parts["tip_tpu_left"]), to_mesh(parts["tip_tpu_right"])
    el, er = tip_end(tl), tip_end(tr)
    tcp = (el + er) / 2
    tcp[1] = 0.0
    import d405_frame as D
    T_grip_cam = D.camera_frame()
    T_grip_tcp = np.eye(4)
    T_grip_tcp[:3, 3] = tcp

    painted = threemf_meshes(THREEMF, painted=True)
    raw = load_raw()                    # register against the unmodified jaws (no sleeve, no badge)
    dots = {}
    for name, src in JAW_3MF.items():
        d, rms = dot_centres(to_mesh(raw[name], 0.1), painted[src])
        dots[name] = d
        print(f"{name}: {len(d)} red dots, registration rms {rms:.3f} mm")
        write_dot_inlays(name, d)
    for v in ("jaw_right_dc", "jaw_right_dc_button"):        # the handheld variants: same jaw, same dots
        write_dot_inlays(v, dots["jaw_right"])
    write_dot_inlays("jaw_left_dc_button", dots["jaw_left_dc"])

    table = gap_table(tl, tr)
    data = {
        "units": "mm, rad unless noted; gripper frame: origin at the plate mounting face, +X along the jaws, "
                 "+Y to the left jaw, +Z up (camera side)",
        "source": "measured from the PG Chopstick CAD by cad/kinematics.py (see NOTICE.md)",
        "pivot_left_mm": [*PIVOT_L, 0.0],
        "pivot_right_mm": [*PIVOT_R, 0.0],
        "pivot_axis": [0, 0, 1],
        "gear": {"teeth": GEAR_TEETH, "module": GEAR_MODULE, "ratio": -1.0,
                 "note": "left jaw turns +theta about +Z, right jaw -theta"},
        "tip_end_left_mm": np.round(el, 3).tolist(),
        "tip_end_right_mm": np.round(er, 3).tolist(),
        "tcp_mm": np.round(tcp, 3).tolist(),
        "cameras": {
            "d405": {
                "model": "Intel RealSense D405 in the HandUMI cup on d405_frame's tilt hinge",
                "tilt_deg": D.TILT_DEG, "tilt_range_deg": list(D.TILT_RANGE),
                "hinge_axis_mm": [D.HINGE[0], 0.0, D.HINGE[1]], "hinge_dir": [0, 1, 0],
                "T_gripper_camera_mm": np.round(T_grip_cam, 6).tolist(),
                "T_camera_tcp_mm": np.round(np.linalg.inv(T_grip_cam) @ T_grip_tcp, 6).tolist(),
                "frame_note": "origin at the centre of the camera's front face, +X along the view axis, +Z up; "
                              "at another tilt, turn it about the hinge axis",
            },
        },
        "red_dots": dots,
        "gap_table_deg_mm": table,
        "robot_gripper_model_open_deg": SA_MODEL_OPEN_DEG,
        "mounts": {
            "openarm": {"T_flange_gripper_m": T_LINK7.tolist(), "flange_frame": "openarm_{left,right}_link7",
                        "plate": "openarm_adapter (6 x M3, PCD 50)"},
            "iso50": {"T_flange_gripper_m": T_FLANGE.tolist(), "flange_frame": "tool flange (UR tool0, ...)",
                      "plate": "iso_flange_plate (ISO 9409-1-50-4-M6)"},
        },
    }
    import parallel
    data["parallel"] = {
        "stroke_per_finger_mm": parallel.STROKE,
        "pinion": {"teeth": parallel.TEETH, "module": parallel.MODULE, "pitch_radius_mm": parallel.R_PITCH,
                   "axis_mm": [parallel.X_PIN, 0.0, 0.0], "axis": [0, 0, 1]},
        "pad_inner_closed_y_mm": 0.0,
        "tcp_mm": list(parallel.tcp()),
        "red_dots_mm": {s: [list(c) for c in parallel.dots(s)] for s in ("left", "right")},
        "note": "left finger moves +q along Y, right -q; pad gap = 2 q; pinion turns q / pitch_radius rad",
    }
    if check:
        lims = {}
        for build in BUILDS:
            if family(build) == "hand":
                continue
            lims[build], rest = clash_sweep(parts, build)
            unit = "mm" if family(build) == "parallel" else "deg"
            val = lims[build] if unit == "mm" else math.degrees(lims[build])
            print(f"{build:18s} closed-pose overlap {rest:.3f} cm3, clash-free opening per side {val:.1f} {unit}")
        data["opening_limits"] = {b: (round(math.degrees(v), 1) if family(b) == "chopstick" else round(v, 2))
                                  for b, v in lims.items()}
        data["opening_limits"]["note"] = "chopstick: deg per jaw; parallel: mm per finger (first clash, or full range)"
    elif OUT.exists() and "opening_limits" in json.loads(OUT.read_text()):
        data["opening_limits"] = json.loads(OUT.read_text())["opening_limits"]   # keep the last measured limits
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1))
    gaps = dict((r[0], r[1]) for r in table)
    for k, c in data["cameras"].items():
        T = np.array(c["T_gripper_camera_mm"])
        print(f"{k} camera origin ({T[0, 3]:.1f}, {T[1, 3]:.1f}, {T[2, 3]:.1f}) mm")
    print(f"TCP {data['tcp_mm']} mm")
    print(f"tip gap: 10 deg -> {gaps[10.0]:.1f} mm, 20 deg -> {gaps[20.0]:.1f} mm, 44 deg -> {gaps[44.0]:.1f} mm")
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    main(ap.parse_args().check)
