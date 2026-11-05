"""OpenArm v1 finger as a tip: the hard-shell finger on OpenArm's own printed finger base.

Source: the openarm-hardshell-fingers branch of this repository (hardware/STEP/hardshell_{finger,tpu}_{left,right}),
copied to source/openarm/. Its base is OpenArm v1's printed finger base (the 5.2 mm plate with 4x M2.5 on 10 x 15 mm,
counterbored on the finger side), so it bolts onto the flange's OpenArm interface
(finger_interfaces.INTERFACES["openarm"]) with OpenArm's own four M2.5 screws, and onto the robot's carriage the same
way. The rib behind the plate (beside OpenArm's carriage) is cut off flush with the plate's back, x = PLATE_BACK:
on HandUMI the flange backs the plate there (see the note in finger_interfaces.py).

Frame (the source's): x along the finger from the carriage (mating face x = 1.4), y across, z the closing axis (the
gripping face at low z). The right finger is the mirror of the left in y.

    python openarm_tip.py      # -> ../hardware/{STEP,STL}/gripper_tips/OpenArm/
"""
from pathlib import Path

from build123d import Box, Pos, export_step, export_stl, import_step

SRC = Path(__file__).resolve().parent / "source/openarm"
OUT = Path(__file__).resolve().parents[1] / "hardware"
PLATE_BACK = 1.4                 # the plate's back face: the mating face on the carriage (and on the flange)


def main():
    for t in ("STL", "STEP"):
        (OUT / t / "gripper_tips/OpenArm").mkdir(parents=True, exist_ok=True)
    keep = Pos(100.0 + PLATE_BACK, 0, 0) * Box(200.0, 400.0, 400.0)          # everything in front of the plate's back
    for side in ("left", "right"):
        for src, name in ((f"hardshell_finger_{side}.step", "Finger"), (f"hardshell_tpu_{side}.step", "Soft")):
            p = import_step(str(SRC / src))
            if name == "Finger":
                p = p & keep
            stem = f"OpenArm-{side.upper()}-{name}"
            export_step(p, str(OUT / "STEP/gripper_tips/OpenArm" / f"{stem}.step"))
            export_stl(p, str(OUT / "STL/gripper_tips/OpenArm" / f"{stem}.stl"), tolerance=0.02, angular_tolerance=0.15)
            bb = p.bounding_box()
            print(stem, "valid", p.is_valid, "solids", len(p.solids()), "x", round(bb.min.X, 2), round(bb.max.X, 2))


if __name__ == "__main__":
    main()
