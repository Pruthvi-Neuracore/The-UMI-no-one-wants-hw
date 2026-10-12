"""Shared geometry for the chopstick gripper: source files, part table, pivot axes and jaw poses.

Gripper frame (mm, same as the upstream assemblies in source/upstream): origin at the centre of the robot-plate
mounting face, +X forward along the jaws, +Y to the left jaw, +Z up (camera side). The two jaws turn about
vertical axes through PIVOT_L and PIVOT_R and are tied by a pair of 24-tooth spur gears (module 1.25 on a
30 mm centre distance), so they always open by the same angle in opposite directions.

Jaw angle `theta` (rad) is the opening of ONE jaw from fully closed: the left jaw turns +theta about +Z,
the right jaw -theta. theta = 0 is the pose the data gripper is modelled in (tips touching).
"""
import math
from pathlib import Path

import numpy as np
from build123d import Compound, Pos, Rot, import_step

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent  # chopstick/
SRC = HERE / "source" / "upstream"
HW = ROOT / "hardware"

PIVOT_L = (67.33, 15.0)
PIVOT_R = (67.33, -15.0)
GEAR_TEETH = 24
GEAR_MODULE = 1.25

# The robot gripper (robot_gripper.step) is modelled with each jaw turned this far open from the data gripper's closed pose.
SA_MODEL_OPEN_DEG = 44.21

# STEP labels -> part file names. Jaws and tips are stored at theta = 0 (closed) in the gripper frame.
DC_PARTS = {
    "ChopstickBase-DC": "chopstick_base_dc",
    "ChopstickClip-DC": "chopstick_clip_dc",
    "Insta360X5_Lite_Frame": "x5_lite_frame",
    "Left_Finger_DC_Spur_Gear_24_teeth_1%": "jaw_left_dc",
    "Right_Finger_DC/SA_Spur_Gear_24_teeth_1%": "jaw_right",
    "Chopstick_-_TPU_left_finger": "tip_tpu_left",
    "Chopstick_-_TPU_right_finger": "tip_tpu_right",
}
SA_PARTS = {
    "ChopstickBase-SA": "chopstick_base_sa",
    "ChopstickClip-SA": "chopstick_clip_sa",
    "Insta360X5_Frame": "x5_frame",
    "Insta360X5_ClipB": "x5_clip_b",
    "Left_Finger_SA_Spur_Gear_24_teeth_1%": "jaw_left_sa",
}
# Parts that move with each jaw, by side.
LEFT_MOVING = {"jaw_left_dc", "jaw_left_sa", "tip_tpu_left", "sleeve_button_cover_left"}
RIGHT_MOVING = {"jaw_right", "jaw_right_dc", "tip_tpu_right", "sleeve_button_cover_right"}

# Reference bodies that are not printed and are not redistributed as parts.
X5_LABEL = "Insta360_X5"
SERVO_LABEL = "Servo"


def jaw_loc(side, theta):
    """Location that turns a closed (theta = 0) jaw part open by theta radians about its pivot."""
    px, py = PIVOT_L if side == "left" else PIVOT_R
    deg = math.degrees(theta) * (1 if side == "left" else -1)
    return Pos(px, py, 0) * Rot(0, 0, deg) * Pos(-px, -py, 0)


def jaw_matrix(side, theta):
    px, py = PIVOT_L if side == "left" else PIVOT_R
    a = theta * (1 if side == "left" else -1)
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    m[:2, :2] = [[c, -s], [s, c]]
    m[:2, 3] = [px - c * px + s * py, py - s * px - c * py]
    return m


def side_of(name):
    if name in LEFT_MOVING:
        return "left"
    if name in RIGHT_MOVING:
        return "right"
    return None


def plain(shape):
    """A detached copy of a STEP child (its solids only), so it exports and moves like any other shape."""
    solids = shape.solids()
    return solids[0] if len(solids) == 1 else Compound(solids)


def load_children(step):
    """{label: shape placed in the gripper frame} for one upstream assembly STEP."""
    asm = import_step(str(SRC / step))
    return {ch.label: plain(ch) for ch in asm.children}


def load_raw():
    """Every upstream printed part at theta = 0 in the gripper frame, plus the X5 and servo reference envelopes.

    Returns {name: shape}. The robot gripper's driver jaw is turned back from its modelled open pose to closed.
    """
    dc = load_children("data_gripper.step")
    sa = load_children("robot_gripper.step")
    parts = {name: dc[label] for label, name in DC_PARTS.items()}
    for label, name in SA_PARTS.items():
        shape = sa[label]
        if name == "jaw_left_sa":
            shape = shape.moved(jaw_loc("left", -math.radians(SA_MODEL_OPEN_DEG)))
        parts[name] = shape
    parts["jaw_right_dc"] = parts["jaw_right"]          # the handheld right jaw gets a sleeve; the robot's doesn't
    parts["insta360_x5_envelope"] = dc[X5_LABEL]
    parts["servo_envelope"] = sa[SERVO_LABEL]
    return parts


def load_parts():
    """The finished parts from hardware/STEP (restyled, plus the new PG parts) and the reference envelopes."""
    raw = load_raw()
    parts = {p.stem: plain(import_step(str(p))) for p in sorted((HW / "STEP").glob("*.step"))}
    for k in ("insta360_x5_envelope", "servo_envelope"):
        parts[k] = raw[k]
    parts.update(envelopes())
    return parts


def envelopes():
    """Simple bodies for the bought parts the new PG parts carry (not printed)."""
    import d405_frame
    return {"d405_envelope": d405_frame.camera_envelope()}


# Which parts make each build. Moving parts are listed under the side they turn with.
JAWS_DC = ["jaw_left_dc", "jaw_right_dc", "tip_tpu_left", "tip_tpu_right",
           "sleeve_button_cover_left", "sleeve_button_cover_right"]
JAWS_SA = ["jaw_left_sa", "jaw_right", "tip_tpu_left", "tip_tpu_right"]
POD = ["pg_pod", "pg_pod_lid", "pg_pod_button"]
BUILDS = {
    # handheld, Insta360 X5, with the PG pod under the camera
    "data": ["chopstick_base_dc", "chopstick_clip_dc", "x5_lite_frame"] + JAWS_DC + POD,
    # handheld, RealSense D405 instead of the X5
    "data_d405": ["chopstick_base_dc", "chopstick_clip_dc", "d405_frame"] + JAWS_DC,
    # OpenArm, Insta360 X5
    "openarm": ["chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b", "openarm_adapter"] + JAWS_SA,
    # OpenArm, RealSense D405 (same d405_frame; its back takes the adapter directly)
    "openarm_d405": ["chopstick_base_sa", "chopstick_clip_sa", "d405_frame", "openarm_adapter"] + JAWS_SA,
    # any cobot with an ISO 9409-1-50-4-M6 tool flange (UR e-series, ABB GoFa, FANUC CRX, ...)
    "iso50": ["chopstick_base_sa", "chopstick_clip_sa", "x5_frame", "x5_clip_b", "iso_flange_plate"] + JAWS_SA,
    "iso50_d405": ["chopstick_base_sa", "chopstick_clip_sa", "d405_frame", "iso_flange_plate"] + JAWS_SA,
}
CAMERA = {b: ("d405_envelope" if b.endswith("_d405") else "insta360_x5_envelope") for b in BUILDS}
MOUNT = {"openarm": "openarm_adapter", "openarm_d405": "openarm_adapter",
         "iso50": "iso_flange_plate", "iso50_d405": "iso_flange_plate"}
DATA_GRIPPER, ROBOT_GRIPPER = BUILDS["data"], BUILDS["openarm"]
