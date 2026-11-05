"""Scissor-UMI kinematics, shared by the CAD, the renders and the URDF.

Gripper frame G (mm), the same as the Robotiq URDF's `robotiq_arg2f_base_link`: z = approach (towards the fingertips),
y = the jaws' opening direction, x = up (the camera and Quest side). Its origin is the gripper body's mounting face,
8 mm in front of the coupling's (`robotiq_arg2f_coupling`) origin.

Each jaw is a parallelogram four-bar copied from the robot gripper: an outer link and an inner link of equal length
pivot on the base, and their far ends carry the finger (the coupler), so the pad translates on an arc and stays
parallel. q is the robot gripper's `finger_joint` (0 = open, Q_MAX = closed). The handheld's jaws use the same q, so a
recorded q drives the robot's gripper directly and the pads are in the same place.

Every gripper is written in the same form: the left jaw's offsets in the base plane, with the links' frame at
link_angle0 - q. The 2F-85's URDF turns its link frames 180° about z instead (rpy z = pi, axis +x), which in the (y, z)
plane is a reflection: its offsets are stored reflected (y -> -y) with link_angle0 = 0, and `mesh_mirror` tells the mesh
code that Robotiq's part meshes sit in the reflected frame. Pick the gripper with SCISSOR_GRIPPER (default below).
"""
import math
import os

import numpy as np

GRIPPERS = {
    # Robotiq 2F-140, from robotiq_arg2f_140_model (finger_joint and left_inner_knuckle_joint at rpy x = 2.2959,
    # outer_finger + inner_finger offsets, inner_finger_pad 27 x 65 x 7.5 mm). Open at q = 0 (~140 mm between the
    # robot's pads), closed at q = 0.7.
    "2f-140": dict(
        outer_pivot=(-30.601, 54.905),      # left jaw (y < 0), (y, z): finger_joint
        inner_pivot=(-12.7, 61.42),         # left_inner_knuckle_joint
        link_angle0=2.2959,                 # the links' joint frame angle about +x at q = 0
        outer_to_finger=((18.22, 26.0018), (81.7554, -28.2203)),   # outer knuckle -> outer finger -> inner finger joint
        finger_angle=-0.725,                # inner finger joint frame, relative to the outer finger, at q = 0
        pad_offset=(45.7554, -27.2203),     # inner finger joint -> pad centre, in the inner finger frame
        pad_size=(27.0, 65.0, 7.5),         # x (width), along the finger, thickness (y)
        q_max=0.7,
        coupling_to_base=8.0,               # coupling origin -> base frame origin, along z
        coupling_back=7.0,                  # coupling's back face (robot flange) behind its origin
        mesh="140", mesh_mirror=False,      # Robotiq's part meshes: robotiq_arg2f_140_*.stl, link frames as above
        pad_local=((0.0, 45.7554, -27.2203), (27.0, 65.0, 7.5)),   # pad box (centre, size) in the inner finger frame
        clevis=12.2,                        # the inner knuckle's base prongs start at |x| = 12.2
        knuckle_arm=(1.0, 0.0),             # the inner knuckle's arm direction in its own (y, z)
        key_at=(12.0, 20.0),                # M3 keys along that arm through a prong into the knuckle gear
        p1_hub=4.9,                         # the outer knuckle's hub half-width at the outer pivot
    ),
    # Robotiq 2F-85, from robotiq_arg2f_85_model (finger_joint and left_inner_knuckle_joint at rpy z = pi, axis +x;
    # outer_finger (0, 31.5, -4.1), inner_finger (0, 6.1, 47.1); the pad mesh 20 x 35 x 5 mm at (0, -21.4, 31.0) in the
    # inner finger frame). Same base pivots as the 2F-140; open at q = 0 (~89 mm between the pad meshes), closed at 0.8.
    "2f-85": dict(
        outer_pivot=(-30.6011, 54.904),
        inner_pivot=(-12.7, 61.42),
        link_angle0=0.0,
        outer_to_finger=((-31.5, -4.1), (-6.1, 47.1)),              # reflected (y -> -y)
        finger_angle=0.0,
        pad_offset=(21.4, 31.0),                                    # reflected
        pad_size=(20.0, 35.0, 5.0),                                 # width, along the finger, thickness
        q_max=0.8,
        coupling_to_base=8.0,
        coupling_back=7.0,
        mesh="85", mesh_mirror=True,
        pad_local=((0.0, -21.4, 31.0), (20.0, 5.0, 35.0)),
        clevis=12.0,
        knuckle_arm=(0.6580, 0.7530),       # towards its far joint E (37.6, 43.0)
        key_at=(10.0, 18.0),
        p1_hub=5.5,
    ),
}
GRIPPER = os.environ.get("SCISSOR_GRIPPER", "2f-85")


def g(name=GRIPPER):
    return GRIPPERS[name]


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def mesh_to_base(name=GRIPPER):
    """2 x 2: a Robotiq part mesh's own (y, z) -> the gripper frame's, for the left jaw's links at q = 0."""
    M = rot(g(name)["link_angle0"])
    return M @ np.diag([-1.0, 1.0]) if g(name)["mesh_mirror"] else M


def local(v, name=GRIPPER):
    """A stored (reflected) offset as the Robotiq mesh frame sees it."""
    return np.array(v) * ([-1, 1] if g(name)["mesh_mirror"] else [1, 1])


def link_angle(q, name=GRIPPER):
    """The links' frame angle (about +x, in the (y, z) plane) of the LEFT jaw at q. finger_joint's axis is -x."""
    return g(name)["link_angle0"] - q


def joints(q, side="left", name=GRIPPER):
    """(y, z) of the four joints of one jaw at q: outer pivot P1, inner pivot P2, finger joint B (outer link's end),
    E (inner link's end), plus the pad centre. The right jaw is the left one mirrored in y."""
    p = g(name)
    P1, P2 = np.array(p["outer_pivot"]), np.array(p["inner_pivot"])
    a = link_angle(q, name)
    (o1, o2) = p["outer_to_finger"]
    B = P1 + rot(a) @ np.array(o1) + rot(a) @ np.array(o2)
    E = B + (P2 - P1)
    # the finger translates: its frame angle stays the q = 0 one
    a0 = link_angle(0.0, name) + p["finger_angle"]
    pad = B + rot(a0) @ np.array(p["pad_offset"])
    out = dict(P1=P1, P2=P2, B=B, E=E, pad=pad)
    if side == "right":
        out = {k: v * [-1, 1] for k, v in out.items()}
    return out


def link_length(name=GRIPPER):
    j = joints(0.0, name=name)
    return float(np.linalg.norm(j["B"] - j["P1"]))


def pad_gap(q, name=GRIPPER):
    """Distance between the two pads' inner faces at q (mm)."""
    t = g(name)["pad_size"][2]
    return 2 * (-joints(q, name=name)["pad"][0] - t / 2)


def q_of_gap(w, name=GRIPPER):
    lo, hi = 0.0, g(name)["q_max"] + 0.2
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if pad_gap(m, name) > w else (lo, m)
    return (lo + hi) / 2


if __name__ == "__main__":
    print(f"link length {link_length():.2f} mm")
    for q in (0.0, g()["q_max"] / 2, g()["q_max"]):
        j = joints(q)
        print(f"q={q:.2f}: B={j['B'].round(2)} E={j['E'].round(2)} pad={j['pad'].round(2)} gap={pad_gap(q):.1f} mm")
    print(f"q at gap 0: {q_of_gap(0.0):.3f}")
