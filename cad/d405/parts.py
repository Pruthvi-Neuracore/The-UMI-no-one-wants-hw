"""PG Chopstick D405: shared geometry, the part table, where each part file lives, and how the moving parts move.

Gripper frame (mm, every part file and URDF): origin at the centre of the robot-plate mounting face, +X forward
along the jaws, +Y to the left jaw, +Z up (camera side).

End effectors:
  * chopstick: two jaws turning about vertical axes through PIVOT_L / PIVOT_R, tied by 24-tooth spur gears, so they
    open by the same angle in opposite directions. q = one jaw's angle from closed (rad); left turns +q, right -q.
  * parallel: two fingers on carriages sliding along Y, tied by a rack and pinion. q = one finger's travel from
    closed (mm); left moves +q, right -q (parallel.py).
  * hand: no jaws; the D405 head (camera, electronics box, Quest controller) rides on a wrist saddle (ebox.py).

The head (hardware/head): d405_frame + the HandUMI D405 cup on its tilt hinge, the electronics box (Pico 2, IMU,
USB 3 hub) and the Meta Quest 3S controller cradle (tracking only).
"""
import math
from pathlib import Path

import numpy as np
from build123d import Compound, Pos, Rot, import_step

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1] / "build" / HERE.name   # generated files: build/insta360_x5 or build/d405
SRC = HERE / "source" / "upstream"
HW = ROOT / "hardware"
REF = HW / "reference"            # bought parts drawn for assemblies only (straps, rods, ...)

PIVOT_L = (67.33, 15.0)
PIVOT_R = (67.33, -15.0)
GEAR_TEETH = 24
GEAR_MODULE = 1.25

# The robot gripper (robot_gripper.step) is modelled with each jaw turned this far open from the closed pose.
SA_MODEL_OPEN_DEG = 44.21

# STEP labels -> part file names. Jaws and tips are stored closed in the gripper frame.
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
X5_LABEL = "Insta360_X5"
SERVO_LABEL = "Servo"

# Where each part's files go: hardware/<module>/{STEP,STL}/<name>.* (first matching prefix wins)
MODULES = [
    ("head", ("d405_", "ebox", "quest_", "chopstick_clip_")),   # camera, electronics, controller, end-effector clips
    ("mounts", ("openarm_adapter", "iso_flange_plate")),
    ("parallel", ("parallel_",)),
    ("chopstick", ("chopstick_base", "jaw_", "tip_tpu", "sleeve_")),
]


def module_of(name):
    for module, prefixes in MODULES:
        if name.startswith(prefixes):
            return module
    raise KeyError(f"no module for part {name!r}")


def part_path(name, ext):
    """hardware/<module>/<EXT>/<name>.<ext>, with the folder made."""
    d = HW / module_of(name) / ext.upper()
    d.mkdir(parents=True, exist_ok=True)
    return d / f"{name}.{ext}"


def save_part(name, shape, tol=0.02):
    from build123d import export_step, export_stl
    export_step(shape, str(part_path(name, "step")))
    export_stl(shape, str(part_path(name, "stl")), tolerance=tol, angular_tolerance=0.15)


def save_reference(name, shape):
    """A bought part, drawn for the assemblies (STEP in hardware/reference)."""
    from build123d import export_step
    REF.mkdir(parents=True, exist_ok=True)
    export_step(shape, str(REF / f"{name}.step"))


# --- motion ---------------------------------------------------------------------------------------------------------
CHOPSTICK_LEFT = {"jaw_left_dc", "jaw_left_dc_button", "jaw_left_sa", "tip_tpu_left", "sleeve_button_cover_left",
                  "sleeve_strap_left"}
CHOPSTICK_RIGHT = {"jaw_right", "jaw_right_dc", "jaw_right_dc_button", "tip_tpu_right", "sleeve_button_cover_right",
                   "sleeve_strap_right"}


def side_of(name):
    """'left' / 'right' for a part that moves with a jaw or finger, 'pinion' for the parallel pinion, else None."""
    if name in CHOPSTICK_LEFT:
        return "left"
    if name in CHOPSTICK_RIGHT:
        return "right"
    if name.startswith("parallel_pinion"):
        return "pinion"
    if name.startswith(("parallel_carriage_", "parallel_finger_", "parallel_pad_", "parallel_button_cover_",
                        "parallel_strap_")):
        return "left" if "_left" in name else "right"
    return None


def jaw_loc(side, theta):
    """Location that turns a closed chopstick jaw part open by theta radians about its pivot."""
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


def family(build):
    return build.split("_")[0]          # chopstick / parallel / hand


def q_max(build):
    """Full opening of one jaw (chopstick, rad) or one finger (parallel, mm)."""
    if family(build) == "parallel":
        import parallel
        return parallel.STROKE
    return math.radians(72.0)


def q_display(build, frac):
    """The jaw / finger value for a fraction of a typical working opening (renders, assemblies)."""
    if family(build) == "parallel":
        import parallel
        return frac * parallel.STROKE
    return frac * math.radians(45.0)


def motion(build, name, q):
    """4x4 that moves part `name` of `build` from closed to opening q (mm)."""
    side = side_of(name)
    if side is None:
        return np.eye(4)
    if family(build) == "parallel":
        import parallel
        if side == "pinion":                 # turns with the racks
            a = q / parallel.R_PITCH
            c, s = math.cos(a), math.sin(a)
            m = np.eye(4)
            m[:2, :2] = [[c, -s], [s, c]]
            x0 = parallel.X_PIN
            m[:2, 3] = [x0 - c * x0, -s * x0]
            return m
        m = np.eye(4)
        m[1, 3] = q if side == "left" else -q
        return m
    return jaw_matrix(side, q)


# --- loading --------------------------------------------------------------------------------------------------------
def plain(shape):
    """A detached copy of a STEP child (its solids only), so it exports and moves like any other shape."""
    solids = shape.solids()
    return solids[0] if len(solids) == 1 else Compound(solids)


def load_children(step):
    """{label: shape placed in the gripper frame} for one upstream assembly STEP."""
    asm = import_step(str(SRC / step))
    return {ch.label: plain(ch) for ch in asm.children}


def load_raw():
    """Every upstream printed part, closed, in the gripper frame, plus the X5 and servo reference envelopes."""
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
    """The finished parts (hardware/*/STEP), the bought parts drawn in hardware/reference, and the envelopes."""
    import d405_frame
    import ebox
    raw = load_raw()
    files = sorted(HW.glob("*/STEP/*.step")) + sorted(REF.glob("*.step"))
    parts = {p.stem: plain(import_step(str(p))) for p in files}
    parts["servo_envelope"] = raw["servo_envelope"]
    parts["d405_cup"] = d405_frame.cup()                    # the HandUMI cup (mesh), on the hinge at TILT_DEG
    parts["d405_envelope"] = d405_frame.camera_envelope()
    parts["quest_controller_envelope"] = ebox.controller_envelope()
    return parts


# --- builds ---------------------------------------------------------------------------------------------------------
TIPS = ["tip_tpu_left", "tip_tpu_right"]
STRAPS = ["sleeve_strap_left", "sleeve_strap_right"]
HEAD_HANDHELD = ["d405_frame", "d405_cup", "ebox", "ebox_lid", "quest_cradle", "quest_controller_envelope",
                 "chopstick_clip_dc"]
HEAD_ROBOT = ["d405_frame", "d405_cup", "chopstick_clip_sa"]
CHOP_SA = ["chopstick_base_sa", "jaw_left_sa", "jaw_right"] + TIPS
PAR_HAND = ["parallel_base_dc", "parallel_pinion", "parallel_carriage_left", "parallel_carriage_right",
            "parallel_pad_left", "parallel_pad_right", "parallel_strap_left", "parallel_strap_right"]
PAR_ROBOT = ["parallel_base_sa", "parallel_pinion_servo", "parallel_carriage_left", "parallel_carriage_right",
             "parallel_finger_left", "parallel_finger_right", "parallel_pad_left", "parallel_pad_right"]
BUILDS = {
    # chopstick, handheld: the record button is in the index-finger sleeve only, wired to the Pico
    # (right hand -> right jaw: start / stop; left hand -> left jaw: cancel)
    "chopstick_right": HEAD_HANDHELD + ["chopstick_base_dc"] + TIPS + STRAPS
                       + ["jaw_left_dc", "jaw_right_dc_button", "sleeve_button_cover_right"],
    "chopstick_left": HEAD_HANDHELD + ["chopstick_base_dc"] + TIPS + STRAPS
                      + ["jaw_left_dc_button", "jaw_right_dc", "sleeve_button_cover_left"],
    "chopstick_openarm": HEAD_ROBOT + CHOP_SA + ["openarm_adapter"],
    "chopstick_iso50": HEAD_ROBOT + CHOP_SA + ["iso_flange_plate"],
    "parallel_right": HEAD_HANDHELD + PAR_HAND
                      + ["parallel_finger_left_sleeve", "parallel_finger_right_button", "parallel_button_cover_right"],
    "parallel_left": HEAD_HANDHELD + PAR_HAND
                     + ["parallel_finger_left_button", "parallel_finger_right_sleeve", "parallel_button_cover_left"],
    "parallel_openarm": HEAD_ROBOT + PAR_ROBOT + ["openarm_adapter"],
    "parallel_iso50": HEAD_ROBOT + PAR_ROBOT + ["iso_flange_plate"],
    # bare hand: the head on a wrist saddle (in place of the box's lid)
    "hand": ["d405_frame", "d405_cup", "ebox", "ebox_wrist_saddle", "quest_cradle", "quest_controller_envelope"],
}
ROBOT_BUILDS = [b for b in BUILDS if b.endswith(("_openarm", "_iso50"))]
CAMERA = {b: "d405_envelope" for b in BUILDS}
