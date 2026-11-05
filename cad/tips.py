"""Gripper-tip sets on the finger links: each is a robot's own finger, bolted on through that robot's interface
(finger_interfaces.py).

A tip set names its interface and, for each finger file, where the file's hole pattern is: its centre on the mating
face and two directions in the file's own frame, u (closing axis, toward the gripping face) and w (out of the mating
face into the finger). That fixes the file's pattern frame, and finger_interfaces.frame() puts that frame on each link,
so the tip lands exactly on the holes the flange was cut with. No fitting or searching.
"""
from dataclasses import dataclass

import numpy as np

import finger_interfaces as FI


@dataclass(frozen=True)
class FingerFile:
    files: tuple            # main part first (pads / soft inserts follow)
    centre: tuple           # hole-pattern centre on the mating face, in the file's frame (mm)
    u: tuple                # closing axis, toward the gripping face
    w: tuple                # out of the mating face, into the finger


@dataclass(frozen=True)
class TipSet:
    name: str
    folder: str             # under hardware/{STEP,STL}/gripper_tips/
    interface: str          # key in finger_interfaces.INTERFACES
    ref: FingerFile         # the interface's reference finger (right thumb)
    mirror: FingerFile      # its mirror image (same as ref for a "same" pair)
    robot: str = ""

    @property
    def itf(self):
        return FI.INTERFACES[self.interface]


_hande = FingerFile(("Hand-E-Finger.step",), (1.5, -14.2, 0.0), (-1, 0, 0), (0, 0, 1))
_enpire = FingerFile(("ENPIRE-RIGHT-Jaw.step", "ENPIRE-RIGHT-Soft-Insert.step"), (-10.2, -7.8, 14.3), (0, 0, -1), (0, -1, 0))

TIP_SETS = [
    TipSet("AgileX Piper", "AgileX-Piper", "piper",
           FingerFile(("Piper-RIGHT-Gripper-Jaw.step", "Piper-RIGHT-Gripper-Pad.step"), (10.77, 172.75, -56.89),
                      (0, 1, 0), (1, 0, 0)),
           FingerFile(("Piper-LEFT-Gripper-Jaw.step", "Piper-LEFT-Gripper-Pad.step"), (10.77, 238.45, -56.89),
                      (0, -1, 0), (1, 0, 0)), robot="piper"),
    # the Robotiq finger itself (hande_tip.py); both jaws are the same part, one turned 180 degrees, as on the Hand-E
    TipSet("Robotiq Hand-E", "Robotiq-Hand-E", "hande", _hande, _hande, robot="hande"),
    # Open-ENPIRE compliant finger for the YAM, unmodified (enpire_tip.py); LEFT is its mirror across z = 12
    TipSet("Open-ENPIRE (I2RT YAM mount)", "Open-ENPIRE", "yam", _enpire,
           FingerFile(("ENPIRE-LEFT-Jaw.step", "ENPIRE-LEFT-Soft-Insert.step"), (-10.2, -7.8, 9.7), (0, 0, 1), (0, -1, 0)),
           robot="yam"),
    # OpenArm v1 finger base under the hard-shell finger (openarm_tip.py); right is the mirror of left in y
    TipSet("OpenArm v1 (hard-shell finger)", "OpenArm", "openarm",
           FingerFile(("OpenArm-LEFT-Finger.step", "OpenArm-LEFT-Soft.step"), (1.4, 0.0, 45.2), (0, 0, -1), (1, 0, 0)),
           FingerFile(("OpenArm-RIGHT-Finger.step", "OpenArm-RIGHT-Soft.step"), (1.4, 0.0, 45.2), (0, 0, -1), (1, 0, 0)),
           robot="openarm"),
]


def finger_file(ts, hand, link):
    return ts.mirror if FI.uses_mirror(ts.itf, hand, link) else ts.ref


def link_transform(ts, hand="right", link="thumb"):
    """4x4 taking the finger file's coordinates into the finger link's frame (flange face y = 0)."""
    ff = finger_file(ts, hand, link)
    u, w = np.array(ff.u, float), np.array(ff.w, float)
    B = np.column_stack([u, np.cross(w, u), w])                 # file's (u, v, w) axes
    R, o = FI.frame(ts.itf, hand, link)
    if FI.uses_mirror(ts.itf, hand, link):                      # the mirror file's own frame is right-handed again,
        R = R @ np.diag([1.0, -1.0, 1.0])                       # so its features are at (u, -v) of the reference
    T = np.eye(4)
    T[:3, :3] = R @ B.T
    T[:3, 3] = o - T[:3, :3] @ np.array(ff.centre, float)
    return T
