"""Write the URDFs for every PG Chopstick D405 build (parts.BUILDS) from the CAD and hardware/kinematics.json.

  urdf/pg_<build>.urdf   e.g. pg_chopstick_right, pg_chopstick_iso50, pg_hand
  urdf/meshes/*.stl      simplified meshes in each link's frame (mm, scaled by 0.001 in the URDF)

Frames: pg_base (gripper frame), pg_camera (D405: front-face centre at the default tilt, +X along the view, +Z up),
pg_tcp, pg_tip_{left,right}. Chopstick: revolute pg_jaw (rad, one jaw) + pg_jaw_right mimic. Parallel: prismatic
pg_finger (m, one finger) + pg_finger_right mimic + pg_pinion following. Robot builds have a pg_flange root that is
the robot's flange frame (OpenArm link7, or the cobot's tool flange such as UR tool0).

    python urdf.py [--openarm-urdf in.urdf --out out.urdf]   (also splice the chopstick onto both OpenArm arms)
"""
import argparse
import json
import math
import re
import warnings
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from assembly import COLOURS, colour_of
from kinematics import OUT as KIN_JSON, to_mesh
from parts import BUILDS, CAMERA, PIVOT_L, PIVOT_R, ROBOT_BUILDS, ROOT, family, load_parts, side_of

URDF = ROOT / "urdf"
MESHES = URDF / "meshes"
FACES = 6000
DENSITY = {"pla": 1.24e-3 * 0.55, "tpu": 1.21e-3}       # g/mm3; PLA at ~55 % effective fill (walls + 15 % infill)
X5_MASS_G, D405_MASS_G, SERVO_MASS_G = 200.0, 60.0, 150.0  # X5 ~200 g, D405 ~60 g; servo is an estimate, weigh yours


def xyz_rpy(T):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")      # gimbal lock at pitch = ±90° still gives a valid rpy
        rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz")
    return " ".join(f"{v:.9g}" for v in T[:3, 3]), " ".join(f"{v:.9g}" for v in rpy)


def origin(T):
    x, r = xyz_rpy(T)
    return f'<origin xyz="{x}" rpy="{r}"/>'


def write_mesh(name, mesh):
    MESHES.mkdir(parents=True, exist_ok=True)
    m = mesh if len(mesh.faces) <= FACES else mesh.simplify_quadric_decimation(face_count=FACES)
    m.export(MESHES / f"{name}.stl")
    return f"meshes/{name}.stl"


def link_xml(name, visuals, mass_g, com_mm, inertia_gmm2):
    vis = "\n".join(
        f'    <visual name="{v}">{origin(np.eye(4))}<geometry><mesh filename="{p}" scale="0.001 0.001 0.001"/></geometry>'
        f'<material name="{mat}"/></visual>\n'
        f'    <collision name="{v}">{origin(np.eye(4))}<geometry><mesh filename="{p}" scale="0.001 0.001 0.001"/></geometry></collision>'
        for v, p, mat in visuals)
    I = np.asarray(inertia_gmm2) * 1e-9  # g mm2 -> kg m2
    c = " ".join(f"{v * 1e-3:.6g}" for v in com_mm)
    return (f'  <link name="{name}">\n{vis}\n'
            f'    <inertial><origin xyz="{c}" rpy="0 0 0"/><mass value="{mass_g * 1e-3:.5g}"/>'
            f'<inertia ixx="{I[0, 0]:.4g}" ixy="{I[0, 1]:.4g}" ixz="{I[0, 2]:.4g}" iyy="{I[1, 1]:.4g}" '
            f'iyz="{I[1, 2]:.4g}" izz="{I[2, 2]:.4g}"/></inertial>\n  </link>')


def body(meshes_g):
    """Mass (g), centre of mass (mm) and inertia about it (g mm2) of a list of (mesh, mass_g or density)."""
    total, mom, parts = 0.0, np.zeros(3), []
    for m, spec in meshes_g:
        mass = spec[1] if spec[0] == "mass" else m.volume * spec[1]
        mm = m.copy()
        mm.density = mass / max(m.volume, 1e-9)
        parts.append(mm)
        total += mass
        mom += mass * mm.center_mass
    com = mom / total
    I = np.zeros((3, 3))
    for mm in parts:
        I += mm.moment_inertia_frame(trimesh.transformations.translation_matrix(com))
    return total, com, I


def material_of(n):
    c = colour_of(n)
    if c == COLOURS["tpu"]:
        return "tpu"
    if c in (COLOURS["record"], COLOURS["cancel"], COLOURS["strap"]):
        return {COLOURS["record"]: "pg_green", COLOURS["cancel"]: "pg_red", COLOURS["strap"]: "pg_strap"}[c]
    return "pg_white" if c == COLOURS["white"] else "pg_blue"


def build(kind, parts, kin):
    """links, joints for one build. Frames: pg_base (gripper frame), pg_camera (X5, between its lens domes),
    pg_tcp, and per family: chopstick -> pg_jaw (revolute, rad, one jaw) with pg_jaw_right as its -1 mimic;
    hand -> no joints. Robot builds hang from pg_flange (the robot's flange frame)."""
    fam = family(kind)
    robot = kind in ROBOT_BUILDS
    names = list(BUILDS[kind]) + [CAMERA[kind]]
    meshes = {n: to_mesh(parts[n], 0.1) for n in names}
    if robot:
        from assembly import servo
        names.append("servo_envelope")
        meshes["servo_envelope"] = servo(kind, parts)
    links, joints = [], []

    def spec(n):
        if n == "d405_envelope":
            return ("mass", 60.0)                         # RealSense D405
        if n == "quest_controller_envelope":
            return ("mass", 164.0)                        # Quest Touch Pro controller
        if n.startswith("sleeve_strap"):
            return ("mass", 3.0)                         # elastic hook-and-loop strap
        if n == "servo_envelope":
            return ("mass", SERVO_MASS_G)
        return ("density", DENSITY["tpu" if n.startswith("tip_tpu") else "pla"])

    def link(name, part_names, T=np.eye(4), suffix=""):
        ms = {n: meshes[n].copy().apply_transform(T) for n in part_names}
        vis = [(n, write_mesh(n + suffix, ms[n]), "camera" if "envelope" in n else material_of(n)) for n in part_names]
        links.append(link_xml(name, vis, *body([(ms[n], spec(n)) for n in part_names])))

    def fixed_frame(child, parent, T):
        links.append(f'  <link name="{child}"/>')
        joints.append(f'  <joint name="{child}_joint" type="fixed"><parent link="{parent}"/>'
                      f'<child link="{child}"/>{origin(T)}</joint>')

    link("pg_base", [n for n in names if side_of(n) is None])
    if fam == "chopstick":
        q_max = math.radians(kin["opening_limits"][kind]) if kind in kin.get("opening_limits", {}) else math.radians(72)
        for side, pivot in (("left", PIVOT_L), ("right", PIVOT_R)):
            jn = [n for n in names if side_of(n) == side]
            link(f"pg_jaw_{side}", jn, trimesh.transformations.translation_matrix([-pivot[0], -pivot[1], 0]),
                 "_jawframe")
            T = trimesh.transformations.translation_matrix([pivot[0] * 1e-3, pivot[1] * 1e-3, 0])
            mimic = '\n    <mimic joint="pg_jaw" multiplier="-1" offset="0"/>' if side == "right" else ""
            jname = "pg_jaw" if side == "left" else "pg_jaw_right"
            lo, hi = (0.0, q_max) if side == "left" else (-q_max, 0.0)
            joints.append(f'  <joint name="{jname}" type="revolute">\n    <parent link="pg_base"/>'
                          f'<child link="pg_jaw_{side}"/>\n    {origin(T)}<axis xyz="0 0 1"/>\n'
                          f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="4" velocity="6"/>{mimic}\n  </joint>')
            tip = np.array(kin[f"tip_end_{side}_mm"]) - [pivot[0], pivot[1], 0]
            fixed_frame(f"pg_tip_{side}", f"pg_jaw_{side}", trimesh.transformations.translation_matrix(tip * 1e-3))
        tcp = np.array(kin["tcp_mm"])
    else:
        tcp = None
    Tcam = np.array(kin["cameras"]["d405"]["T_gripper_camera_mm"])
    Tcam[:3, 3] *= 1e-3
    fixed_frame("pg_camera", "pg_base", Tcam)
    if tcp is not None:
        fixed_frame("pg_tcp", "pg_base", trimesh.transformations.translation_matrix(tcp * 1e-3))
    if robot:
        T = np.array(kin["mounts"][kind.split("_")[-1]]["T_flange_gripper_m"])
        links.insert(0, '  <link name="pg_flange"/>')
        joints.insert(0, '  <joint name="pg_mount_joint" type="fixed"><parent link="pg_flange"/>'
                         f'<child link="pg_base"/>{origin(T)}</joint>')
    return links, joints


MATERIALS = ('  <material name="pg_blue"><color rgba="0.12 0.37 0.68 1"/></material>\n'
             '  <material name="pg_white"><color rgba="0.96 0.96 0.97 1"/></material>\n'
             '  <material name="tpu"><color rgba="0.87 0.9 0.93 1"/></material>\n'
             '  <material name="pg_green"><color rgba="0.18 0.62 0.27 1"/></material>\n'
             '  <material name="pg_red"><color rgba="0.84 0.16 0.16 1"/></material>\n'
             '  <material name="pg_strap"><color rgba="0.17 0.35 0.63 1"/></material>\n'
             '  <material name="camera"><color rgba="0.11 0.12 0.13 1"/></material>')


def write(kind, parts, kin):
    links, joints = build(kind, parts, kin)
    name = f"pg_{kind}"
    text = (f'<?xml version="1.0"?>\n<!-- PG Chopstick D405 hardware, generated by cad/urdf.py. Units: m, rad. -->\n'
            f'<robot name="{name}">\n{MATERIALS}\n' + "\n".join(links) + "\n" + "\n".join(joints) + "\n</robot>\n")
    (URDF / f"{name}.urdf").write_text(text)
    print(f"wrote urdf/{name}.urdf")
    return text


def splice_openarm(src, dst, gripper_urdf, mesh_dir):
    """Replace the stock hand and fingers on both OpenArm arms with the PG Chopstick robot gripper."""
    s = Path(src).read_text()
    for side in ("left", "right"):
        # drop the stock hand, its tcp, both fingers and their joints
        for pat in (rf'\s*<link name="openarm_{side}_(hand|hand_tcp|left_finger|right_finger)"\s*/>',
                    rf'\s*<link name="openarm_{side}_(hand|hand_tcp|left_finger|right_finger)">.*?</link>',
                    rf'\s*<joint name="(?:{side}_openarm_hand_joint|openarm_{side}_hand_tcp_joint|openarm_{side}_finger_joint[12])"'
                    rf'[^>]*>.*?</joint>'):
            s = re.sub(pat, "", s, flags=re.S)
        g = gripper_urdf.split("\n", 3)[3].rsplit("</robot>", 1)[0]
        g = re.sub(r'  <material name="[^"]+">.*?</material>\n', "", g)
        g = g.replace('name="pg_', f'name="openarm_{side}_pg_').replace(
            'link="pg_', f'link="openarm_{side}_pg_').replace(
            'joint="pg_', f'joint="openarm_{side}_pg_').replace('filename="meshes/', f'filename="{mesh_dir}/')
        g += (f'  <joint name="openarm_{side}_pg_attach" type="fixed"><parent link="openarm_{side}_link7"/>'
              f'<child link="openarm_{side}_pg_flange"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n')
        s = s.replace("</robot>", g + "</robot>")
    if '<material name="pg_blue">' not in s:
        s = s.replace(">", ">\n" + MATERIALS, 1) if s.lstrip().startswith("<robot") else re.sub(
            r"(<robot[^>]*>)", r"\1\n" + MATERIALS.replace("\\", "\\\\"), s, count=1)
    Path(dst).write_text(s)
    print(f"wrote {dst}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--openarm-urdf")
    ap.add_argument("--out")
    a = ap.parse_args()
    kin = json.loads(KIN_JSON.read_text())
    if "opening_limits" not in kin:
        raise SystemExit("run `python kinematics.py --check` first (it measures the opening limits)")
    parts = load_parts()
    for old in URDF.glob("*.urdf"):
        old.unlink()
    for old in MESHES.glob("*.stl"):
        old.unlink()
    texts = {kind: write(kind, parts, kin) for kind in BUILDS}
    robot = texts["chopstick_openarm"]
    if a.openarm_urdf:
        out = Path(a.out or "openarm_pg_chopstick.urdf").resolve()
        splice_openarm(a.openarm_urdf, out, robot, str(MESHES.resolve()))


if __name__ == "__main__":
    main()
