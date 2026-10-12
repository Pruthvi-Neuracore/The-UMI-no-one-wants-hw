"""Measure the chopstick gripper's kinematics from the CAD and write them for the URDF and the retargeting code.

Writes chopstick/retarget/chopstick_retarget/chopstick_kinematics.json with:
  - the pivot axes and gear coupling,
  - the tool-tip (TCP) point where the TPU tips meet when closed,
  - the Insta360 X5 lens centres (front and back dome) and the camera frame between them,
  - the red-dot centres on each jaw, in that jaw's closed pose (taken from the paint in KIWI's 3MF),
  - a table of jaw angle -> tip gap (distance between the TPU tip ends) and the angle limits,
  - the OpenArm mount: pose of the gripper frame in link7 (the data gripper and robot gripper share the
    camera -> TCP extrinsics, so this is the only robot-specific transform).

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
from kiwi import (GEAR_MODULE, GEAR_TEETH, PIVOT_L, PIVOT_R, ROOT, SA_MODEL_OPEN_DEG, jaw_matrix, load_parts)
from openarm_adapter import T_LINK7

OUT = ROOT / "retarget" / "chopstick_retarget" / "chopstick_kinematics.json"
JAW_3MF = {"jaw_left_dc": "spur_bearing_test_7_datacollection parts - Left Finger DC Spur Gear 24 teeth 1%",
           "jaw_right": "spur_bearing_test_7_datacollection parts - Right Finger DC-SA Spur Gear 24 teeth 1%",
           "jaw_left_sa": "Chopsticks - Left Finger SA Spur Gear 24 teeth 1%"}


def to_mesh(shape, tol=0.05):
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


def clash_sweep(parts, fixed, jaws, step_deg=2.0, max_deg=80.0):
    """First jaw angle (deg) at which either jaw (with its tip) overlaps the fixed parts by more than 0.05 cm3."""
    base = to_manifold(to_mesh(parts[fixed[0]], 0.1))
    for n in fixed[1:]:
        base += to_manifold(to_mesh(parts[n], 0.1))
    base = base.to_mesh()
    base = trimesh.Trimesh(base.vert_properties[:, :3], base.tri_verts)
    left = trimesh.util.concatenate([to_mesh(parts[n], 0.1) for n in jaws["left"]])
    right = trimesh.util.concatenate([to_mesh(parts[n], 0.1) for n in jaws["right"]])
    rest = overlap_cm3(base, left) + overlap_cm3(base, right)
    for deg in np.arange(step_deg, max_deg + 1e-9, step_deg):
        th = math.radians(deg)
        l = left.copy().apply_transform(jaw_matrix("left", th))
        r = right.copy().apply_transform(jaw_matrix("right", th))
        v = overlap_cm3(base, l) + overlap_cm3(base, r)
        if v - rest > 0.05:
            return float(deg - step_deg), rest
    return float(max_deg), rest


def main(check):
    parts = load_parts()
    tl, tr = to_mesh(parts["tip_tpu_left"]), to_mesh(parts["tip_tpu_right"])
    el, er = tip_end(tl), tip_end(tr)
    tcp = (el + er) / 2
    tcp[1] = 0.0
    lenses = lens_centres(parts["insta360_x5_envelope"])
    front, back = np.array(lenses[0][0]), np.array(lenses[-1][0])
    cam_origin = (front + back) / 2
    cam_x = (front - back) / np.linalg.norm(front - back)          # optical axis of the front lens
    cam_z = np.cross(cam_x, [0, 1, 0]); cam_z /= np.linalg.norm(cam_z)
    cam_z = cam_z if cam_z[2] > 0 else -cam_z
    cam_y = np.cross(cam_z, cam_x)
    T_grip_cam = np.eye(4)
    T_grip_cam[:3, :3] = np.column_stack([cam_x, cam_y, cam_z])
    T_grip_cam[:3, 3] = cam_origin
    T_grip_tcp = np.eye(4)
    T_grip_tcp[:3, 3] = tcp
    T_cam_tcp = np.linalg.inv(T_grip_cam) @ T_grip_tcp

    painted = threemf_meshes(THREEMF, painted=True)
    dots = {}
    for name, src in JAW_3MF.items():
        d, rms = dot_centres(to_mesh(parts[name], 0.1), painted[src])
        dots[name] = d
        print(f"{name}: {len(d)} red dots, registration rms {rms:.3f} mm")

    table = gap_table(tl, tr)
    data = {
        "units": "mm, rad unless noted; gripper frame: origin at the plate mounting face, +X along the jaws, "
                 "+Y to the left jaw, +Z up (camera side)",
        "source": "KIWI_hardware chop_DC.step / chop_SA.step / KIWI-3dprint.3mf (Apache-2.0), measured by cad/kinematics.py",
        "pivot_left_mm": [*PIVOT_L, 0.0],
        "pivot_right_mm": [*PIVOT_R, 0.0],
        "pivot_axis": [0, 0, 1],
        "gear": {"teeth": GEAR_TEETH, "module": GEAR_MODULE, "ratio": -1.0,
                 "note": "left jaw turns +theta about +Z, right jaw -theta"},
        "tip_end_left_mm": np.round(el, 3).tolist(),
        "tip_end_right_mm": np.round(er, 3).tolist(),
        "tcp_mm": np.round(tcp, 3).tolist(),
        "camera": {
            "model": "Insta360 X5",
            "front_lens_centre_mm": np.round(front, 3).tolist(),
            "back_lens_centre_mm": np.round(back, 3).tolist(),
            "lens_dome_radius_mm": round(lenses[0][1], 3),
            "T_gripper_camera_mm": np.round(T_grip_cam, 6).tolist(),
            "frame_note": "origin midway between the two lens domes, +X out of the front lens, +Z up",
        },
        "T_camera_tcp_mm": np.round(T_cam_tcp, 6).tolist(),
        "red_dots": dots,
        "gap_table_deg_mm": table,
        "robot_gripper_model_open_deg": SA_MODEL_OPEN_DEG,
        "openarm": {
            "T_link7_gripper_m": T_LINK7.tolist(),
            "note": "openarm_adapter on the link7 flange (6 x M3, PCD 50); translation in metres",
        },
    }
    if check:
        lim_dc, rest_dc = clash_sweep(parts, ["chopstick_base_dc", "chopstick_clip_dc", "x5_lite_frame"],
                                      {"left": ["jaw_left_dc", "tip_tpu_left"], "right": ["jaw_right", "tip_tpu_right"]})
        lim_sa, rest_sa = clash_sweep(parts, ["chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b"],
                                      {"left": ["jaw_left_sa", "tip_tpu_left"], "right": ["jaw_right", "tip_tpu_right"]})
        print(f"closed-pose overlap with base: data {rest_dc:.3f} cm3, robot {rest_sa:.3f} cm3")
        print(f"clash-free opening per jaw: data gripper {lim_dc:.0f} deg, robot gripper {lim_sa:.0f} deg")
        data["theta_max_deg"] = {"data_gripper": lim_dc, "robot_gripper": lim_sa,
                                 "note": "first clash between a jaw and the base, in 2 deg steps"}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data, indent=1))
    gaps = dict((r[0], r[1]) for r in table)
    print(f"TCP {data['tcp_mm']} mm; camera origin {data['camera']['T_gripper_camera_mm'][0][3]:.1f}, "
          f"{data['camera']['T_gripper_camera_mm'][2][3]:.1f} (x, z) mm")
    print(f"tip gap: 10 deg -> {gaps[10.0]:.1f} mm, 20 deg -> {gaps[20.0]:.1f} mm, 44 deg -> {gaps[44.0]:.1f} mm")
    print(f"wrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    main(ap.parse_args().check)
