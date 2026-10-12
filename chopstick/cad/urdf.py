"""Write the PG Chopstick URDFs from the CAD and the measured kinematics, one per build (parts.BUILDS):

  urdf/pg_chopstick_data.urdf, ..._data_d405.urdf      handheld: base (+ camera, + pod) and two gear-coupled jaws
  urdf/pg_chopstick_openarm.urdf, ..._openarm_d405.urdf robot on the OpenArm adapter
  urdf/pg_chopstick_iso50.urdf, ..._iso50_d405.urdf     robot on the ISO 9409-1-50-4-M6 plate
  urdf/meshes/*.stl                                     simplified meshes in each link's frame (mm, scaled by 0.001)

Robot builds have a root link `chopstick_flange` that is the robot's flange frame (OpenArm link7, or the cobot's tool
flange such as UR tool0), so they attach with an identity fixed joint.
Frames: chopstick_base (gripper frame), chopstick_camera (X5: between the lens domes; D405: front-face centre; +X along
the view, +Z up), chopstick_tcp (where the TPU tips meet), chopstick_tip_{left,right}, and one revolute joint
`chopstick_jaw` (rad, one jaw's opening) with the right jaw as its -1 mimic. For a given camera, camera -> TCP is the
same in the handheld and robot files, which is what retargeting relies on.

    python urdf.py [--openarm-urdf in.urdf --out out.urdf]   (also splice the gripper onto both OpenArm arms)
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

from assembly import WHITE_PARTS
from kinematics import OUT as KIN_JSON, to_mesh
from parts import BUILDS, CAMERA, PIVOT_L, PIVOT_R, ROOT, load_parts, side_of

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
    if n.startswith("tip_tpu"):
        return "tpu"
    return "pg_white" if n in WHITE_PARTS else "pg_blue"


def build(kind, parts, kin):
    robot = not kind.startswith("data")
    names = BUILDS[kind]
    fixed = [n for n in names if side_of(n) is None] + [CAMERA[kind]] + (["servo_envelope"] if robot else [])
    meshes = {n: to_mesh(parts[n], 0.1) for n in set(fixed) | {n for n in names if side_of(n)}}
    theta_max = math.radians(kin["theta_max_deg"]["robot_gripper" if robot else "data_gripper"])
    links, joints = [], []

    def spec(n):
        if n == "insta360_x5_envelope":
            return ("mass", X5_MASS_G)
        if n == "d405_envelope":
            return ("mass", D405_MASS_G)
        if n.startswith("sleeve_strap"):
            return ("mass", 3.0)                         # elastic hook-and-loop strap
        if n == "servo_envelope":
            return ("mass", SERVO_MASS_G)
        return ("density", DENSITY["tpu" if n.startswith("tip_tpu") else "pla"])

    vis = [(n, write_mesh(n, meshes[n]), "camera" if "envelope" in n else material_of(n)) for n in fixed]
    links.append(link_xml("chopstick_base", vis, *body([(meshes[n], spec(n)) for n in fixed])))
    for side, pivot in (("left", PIVOT_L), ("right", PIVOT_R)):
        jn = [n for n in names if side_of(n) == side]
        to_jaw = trimesh.transformations.translation_matrix([-pivot[0], -pivot[1], 0])
        jm = {n: meshes[n].copy().apply_transform(to_jaw) for n in jn}
        vis = [(n, write_mesh(f"{n}_jawframe", jm[n]), material_of(n)) for n in jn]
        links.append(link_xml(f"chopstick_jaw_{side}", vis, *body([(jm[n], spec(n)) for n in jn])))
        T = trimesh.transformations.translation_matrix([pivot[0] * 1e-3, pivot[1] * 1e-3, 0])
        mimic = '\n    <mimic joint="chopstick_jaw" multiplier="-1" offset="0"/>' if side == "right" else ""
        jname = "chopstick_jaw" if side == "left" else "chopstick_jaw_right"
        lo, hi = (0.0, theta_max) if side == "left" else (-theta_max, 0.0)
        joints.append(f'  <joint name="{jname}" type="revolute">\n    <parent link="chopstick_base"/>'
                      f'<child link="chopstick_jaw_{side}"/>\n    {origin(T)}<axis xyz="0 0 1"/>\n'
                      f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="4" velocity="6"/>{mimic}\n  </joint>')
        tip = np.array(kin[f"tip_end_{side}_mm"]) - [pivot[0], pivot[1], 0]
        links.append(f'  <link name="chopstick_tip_{side}"/>')
        joints.append(f'  <joint name="chopstick_tip_{side}_joint" type="fixed"><parent link="chopstick_jaw_{side}"/>'
                      f'<child link="chopstick_tip_{side}"/>{origin(trimesh.transformations.translation_matrix(tip * 1e-3))}</joint>')
    Tcam = np.array(kin["cameras"]["d405" if kind.endswith("_d405") else "x5"]["T_gripper_camera_mm"])
    Tcam[:3, 3] *= 1e-3
    Ttcp = trimesh.transformations.translation_matrix(np.array(kin["tcp_mm"]) * 1e-3)
    for child, T in (("chopstick_camera", Tcam), ("chopstick_tcp", Ttcp)):
        links.append(f'  <link name="{child}"/>')
        joints.append(f'  <joint name="{child}_joint" type="fixed"><parent link="chopstick_base"/>'
                      f'<child link="{child}"/>{origin(T)}</joint>')
    if robot:
        T = np.array(kin["mounts"][kind.split("_")[0]]["T_flange_gripper_m"])
        links.insert(0, '  <link name="chopstick_flange"/>')
        joints.insert(0, '  <joint name="chopstick_mount_joint" type="fixed"><parent link="chopstick_flange"/>'
                         f'<child link="chopstick_base"/>{origin(T)}</joint>')
    return links, joints


MATERIALS = ('  <material name="pg_blue"><color rgba="0.12 0.37 0.68 1"/></material>\n'
             '  <material name="pg_white"><color rgba="0.96 0.96 0.97 1"/></material>\n'
             '  <material name="tpu"><color rgba="0.87 0.9 0.93 1"/></material>\n'
             '  <material name="camera"><color rgba="0.11 0.12 0.13 1"/></material>')


def write(kind, parts, kin):
    links, joints = build(kind, parts, kin)
    name = f"pg_chopstick_{kind}"
    text = (f'<?xml version="1.0"?>\n<!-- PG Chopstick, generated by chopstick/cad/urdf.py. Units: m, rad. -->\n'
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
        g = g.replace('name="chopstick_', f'name="openarm_{side}_chopstick_').replace(
            'link="chopstick_', f'link="openarm_{side}_chopstick_').replace(
            'joint="chopstick_', f'joint="openarm_{side}_chopstick_').replace('filename="meshes/', f'filename="{mesh_dir}/')
        g += (f'  <joint name="openarm_{side}_chopstick_attach" type="fixed"><parent link="openarm_{side}_link7"/>'
              f'<child link="openarm_{side}_chopstick_flange"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>\n')
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
    if "theta_max_deg" not in kin:
        raise SystemExit("run `python kinematics.py --check` first (it measures the jaw limits)")
    parts = load_parts()
    for old in URDF.glob("*.urdf"):
        old.unlink()
    for old in MESHES.glob("*.stl"):
        old.unlink()
    texts = {kind: write(kind, parts, kin) for kind in BUILDS}
    robot = texts["openarm"]
    if a.openarm_urdf:
        out = Path(a.out or "openarm_pg_chopstick.urdf").resolve()
        splice_openarm(a.openarm_urdf, out, robot, str(MESHES.resolve()))


if __name__ == "__main__":
    main()
