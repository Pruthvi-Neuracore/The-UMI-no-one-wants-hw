"""Fit the gripper-tip sets onto the finger-link flange.

Each tip file lives in its own frame. A tip set is described by its mounting holes: the axis the screws run along
(pointing from the mounting face into the tip body), the position of the mounting face on that axis, the hole-pattern
centre, and which in-plane axis carries the 8 mm and the 12 mm spacing. fit() tries every rigid orientation that maps the
pattern onto the link's (8 mm along link x, 12 mm along link z, tip growing along +y from the y = 0 face) and keeps the
one where the two jaws face each other: smallest gap at full close without overlapping.
"""
from dataclasses import dataclass, field

import numpy as np

AX = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0]), "-y": np.array([0, -1.0, 0])}


@dataclass
class TipSet:
    name: str
    folder: str
    right: list            # files on the thumb link (main jaw first)
    left: list             # files on the index/middle link
    axis: str              # screw axis, pointing from the mounting face into the tip body
    face: float            # coordinate of the mounting face along that axis
    centre_r: tuple        # hole-pattern centre (3D) for the right file
    centre_l: tuple
    u8: str                # tip axis carrying the 8 mm spacing (link x)
    v12: str               # tip axis carrying the 12 mm spacing (link z)
    u_offsets: tuple = (0.0,)   # for 2-hole sets: which link column (x = ±4) the pair uses
    robot: str = ""
    travel: str = ""        # native axis the jaws close along (defaults to u8)
    lr_sign: int = 1        # +1: the LEFT file sits at +travel from the RIGHT file in the source; 0: same place
    flip_index: bool = False  # identical left/right parts: turn the index jaw 180° about the screw axis to mirror it
    s_only: int = 0         # force the closing-axis sign (+1 / -1) when the automatic facing check picks the wrong way


TIP_SETS = [
    # AgileX Piper jaw + pad
    TipSet("AgileX Piper", "AgileX-Piper", ["Piper-RIGHT-Gripper-Jaw.step", "Piper-RIGHT-Gripper-Pad.step"],
           ["Piper-LEFT-Gripper-Jaw.step", "Piper-LEFT-Gripper-Pad.step"], "x", 10.8,
           (10.8, 172.7, -56.9), (10.8, 238.5, -56.9), "y", "z", robot="piper"),
    # Robotiq Hand-E finger on a flange plate (hande_tip.py); both jaws are the same part, one turned 180°
    TipSet("Robotiq Hand-E", "Robotiq-Hand-E", ["Hand-E-Finger.step"], ["Hand-E-Finger.step"], "z", 2.0,
           (5.5, 0.0, 2.0), (5.5, 0.0, 2.0), "x", "y", lr_sign=0, flip_index=True, s_only=-1),
    # Open-ENPIRE universal compliant finger, adapted to the flange by enpire_tip.py (mirror pair)
    TipSet("Open-ENPIRE (UCG)", "Open-ENPIRE", ["ENPIRE-RIGHT-Jaw.step", "ENPIRE-RIGHT-Soft-Insert.step"],
           ["ENPIRE-LEFT-Jaw.step", "ENPIRE-LEFT-Soft-Insert.step"], "-y", -3.0,
           (-8.9, -3.0, 12.0), (-8.9, -3.0, 12.0), "z", "x", lr_sign=0),
]


def orientations(ts):
    """Candidates (R, du, thumb_side). Both jaws share one rotation R (as in the source assembly): the screw axis maps
    to the link's +y and the closing axis to ±x. The side that ends up toward the thumb link gets the thumb."""
    a = AX[ts.axis]
    tr = AX[ts.travel or ts.u8]
    w = np.cross(a, tr)
    out = []
    for s in ((ts.s_only,) if ts.s_only else (1, -1)):
        M_tip = np.column_stack([a, tr, w])
        M_link = np.column_stack([AX["y"], s * AX["x"], np.cross(AX["y"], s * AX["x"])])
        R = M_link @ np.linalg.inv(M_tip)
        # thumb sits at -x (link) from the index; LEFT sits at lr_sign·travel from RIGHT in the source
        thumb_side = "left" if ts.lr_sign * s < 0 else "right"
        R_index, sign_du = R, 1.0
        if ts.flip_index:                                       # 180° about the screw axis (tip frame), then map
            R_index, sign_du = R @ (2 * np.outer(a, a) - np.eye(3)), -1.0
        for du in ts.u_offsets:
            out.append(((R, R_index), (du, sign_du * du), thumb_side))
    return out


def link_transform(R, centre, face_axis, face, du=0.0):
    """4x4 taking tip coordinates into link coordinates (mounting face on y = 0, pattern centred, shifted du in x)."""
    c = np.array(centre, float)
    c[np.abs(AX[face_axis]).argmax()] = face
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([du, 0.0, 0.0]) - R @ c
    return T
