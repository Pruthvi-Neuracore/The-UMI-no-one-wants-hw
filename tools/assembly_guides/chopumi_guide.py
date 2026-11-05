"""Chop-UMI (D405 + Quest kit, right hand): IKEA-style assembly pages from its assembly GLB.

    python chopumi_guide.py        # -> pg-chopstick/d405/chop-umi/assembly_guide/

The GLB is the kit's own assembly (metres, Y up, the jaws point along +X, the right jaw on +Z). Steps and hardware
follow the folder's README (Build) and bom.md.
"""
from pathlib import Path

import numpy as np

from ikea import HW, Step, build, load_glb

ROOT = Path(__file__).resolve().parents[2]
KIT = ROOT / "pg-chopstick" / "d405" / "chop-umi"
GLB = KIT / "assembly" / "chop-umi_right.glb"
OUT = KIT / "assembly_guide"
UP = (0, 1, 0)
VIEW = (0.8, 0.75, 0.9)            # in front (+X), above, on the right jaw's side (+Z)
BACK = (-0.9, 0.7, 0.8)
X, Y, Z = np.eye(3)


def steps(m):
    dl = [n for n in m if n.startswith("dot_left")]
    dr = [n for n in m if n.startswith("dot_right")]
    jl, jr = ["jaw_left_dc"] + dl, ["jaw_right_dc_button"] + dr
    head = ["d405_frame", "d405_cup", "d405_envelope", "ebox", "ebox_lid", "quest_arm_right", "quest_support_right",
            "quest_controller_envelope"]
    jaws = jl + jr + ["tip_tpu_left", "tip_tpu_right", "sleeve_button_cover_right"]
    return [
        Step("Base", ["chopstick_base_dc"], {},
             [HW("insert", "M2 × 4 insert", 2, dia=3, length=4), HW("insert", "M3 × 4 insert", 2, dia=4, length=4)],
             show=[], note="Fit the M2 heat-set inserts in the back of the base and the M3 inserts in its underside."),
        Step("Jaws", ["tip_tpu_left", "tip_tpu_right"], {"tip_tpu_left": X * 70, "tip_tpu_right": X * 70},
             [HW("glue", "CA glue", 1), HW("bearing", "624 bearing", 4, dia=13)], show=jl + jr,
             note="Glue the TPU tips onto the jaws and press two 624 bearings into each jaw."),
        Step("Record button", ["sleeve_button_cover_right"], {"sleeve_button_cover_right": Y * 45},
             [HW("switch", "6 × 6 switch + 30 cm wire", 1)], show=jr + ["tip_tpu_right"], zoom=1.2,
             note="Solder the wire to the switch and lay it in the index jaw's channel. Press the switch in, plunger "
                  "up, and press the green TPU cover over it (red on the left hand: cancel)."),
        Step("Join the jaws", jaws, {n: X * 70 for n in jaws},
             [HW("band", "rubber band", 1), HW("screw", "M3 shoulder screw, Ø4 × 18", 2, length=18)],
             show=["chopstick_base_dc"],
             note="Hook the rubber band through the base's centre cavity onto both jaws. Mesh the gears with the tips "
                  "touching, then fit the two shoulder screws."),
        Step("Camera", ["d405_cup", "d405_envelope"], {"d405_cup": Y * 45, "d405_envelope": Y * 45 + X * 50},
             [HW("screw", "M3 × 6", 2, length=6), HW("screw", "M4 × 30 + nyloc nut", 1, dia=4, length=30,
                                                     head="socket")],
             show=["d405_frame"],
             note="Screw the D405 into the cup (2 × M3 × 6). Hang the cup on the frame's knuckle with the M4 × 30 "
                  "bolt and nyloc nut, set the tilt and tighten. Route the cable out of the cup's top window."),
        Step("Electronics box", ["ebox", "ebox_lid"], {"ebox": -X * 60, "ebox_lid": -X * 60 - Y * 45},
             [HW("screw", "M3 × 10", 4, length=10), HW("insert", "M3 insert", 4, dia=4, length=5),
              HW("screw", "M2 × 6 self-tapping", 4, dia=2, length=6)],
             show=["d405_frame", "d405_cup", "d405_envelope"], view=BACK,
             note="Fit the electronics (electronics.md). Bolt the box to the frame's back plate, feed the D405 cable "
                  "into its top slot and the button wire through the side hole, then close the lid."),
        Step("Quest controller", ["quest_arm_right", "quest_support_right", "quest_controller_envelope"],
             {"quest_arm_right": -X * 45, "quest_support_right": -X * 45 + Y * 40,
              "quest_controller_envelope": -X * 45 + Y * 140},
             [HW("screw", "M3 × 10", 4, length=10), HW("screw", "M3 × 25 + nut", 2, length=25),
              HW("tape", "foam tape, 0.5 mm", 1)],
             show=["d405_frame", "d405_cup", "d405_envelope", "ebox", "ebox_lid"], view=BACK,
             note="Screw quest_arm to the box's back. Bolt HandUMI's controller support to its arm end. Line the ring "
                  "with foam tape and push the right controller's handle down through it, trigger forward."),
        Step("Head onto the base", head + ["chopstick_clip_dc"],
             {**{n: Y * 70 for n in head}, "chopstick_clip_dc": -Y * 50},
             [HW("screw", "M3 × 55 + nut + washer", 2, length=55)],
             show=["chopstick_base_dc"] + jaws,
             note="Join the frame to the base with the clip and the two M3 × 55 screws (nut and washer on each)."),
        Step("Finger straps", ["sleeve_strap_left", "sleeve_strap_right"],
             {"sleeve_strap_left": Y * 35, "sleeve_strap_right": Y * 35},
             [HW("strap", "16 mm elastic strap, 12 cm", 2)], zoom=1.1,
             note="Loop each strap through one fin, over your finger, through the other fin and fold it back."),
    ]


PARTS = [("chopstick_base_dc", "chopstick_base_dc", 1), ("chopstick_clip_dc", "chopstick_clip_dc", 1),
         ("jaw_left_dc", "jaw_left_dc", 1), ("jaw_right_dc_button", "jaw_right_dc_button", 1),
         ("tip_tpu_left", "tip_tpu_left", 1), ("tip_tpu_right", "tip_tpu_right", 1),
         ("sleeve_button_cover_right", "sleeve_button_cover_right", 1), ("d405_frame", "d405_frame", 1),
         ("d405_cup", "d405_cup", 1), ("ebox", "ebox", 1), ("ebox_lid", "ebox_lid", 1),
         ("quest_arm_right", "quest_arm_right", 1), ("quest_support_right", "quest_support_right", 1)]


if __name__ == "__main__":
    m = load_glb(GLB)
    build(m, steps(m), OUT, "Chop-UMI · D405 + Quest · right hand", UP, VIEW, parts=PARTS,
          title="Chop-UMI · D405 + Quest: assembly",
          parts_sub="Print these (STEP/ or STL/). Left hand: the _left parts and the jaw pair with the button on the "
                    "left jaw.",
          intro="IKEA-style pages drawn from the kit's own assembly (`assembly/chop-umi_right.glb`): new parts in "
                "blue, pulled out along the way they go in, parts already built in grey, the hardware for each step "
                "at the top. Right hand shown; the left hand mirrors it. Hardware as in [`../bom.md`](../bom.md).")
