"""HandUMI (right hand, Robotiq Hand-E tips): IKEA-style assembly pages from handumi/assembly/*.glb.

    python handumi_guide.py        # -> handumi/docs/assembly_guide/

The GLB is HandUMI's own assembly (metres, Y up; the tips point along -X, the index side is -Z). The order and the
hardware follow HandUMI's BOM (handumi/docs/bom.md); where the BOM lists screws for the mechanism as one set, the
step's box does too.
"""
from pathlib import Path

import numpy as np

from ikea import HW, Step, build, load_glb

ROOT = Path(__file__).resolve().parents[2]
GLB = ROOT / "handumi" / "assembly" / "umi_right_robotiq-hand-e.glb"
OUT = ROOT / "handumi" / "docs" / "assembly_guide"
UP = (0, 1, 0)
VIEW = (-0.9, 0.75, -0.8)          # in front (-X), above, on the index side (-Z)
BACK = (0.9, 0.7, -0.8)            # behind (+X), above, index side
X, Y, Z = np.eye(3)


def steps():
    lm_i = ["lm4uu_index_-8", "lm4uu_index_-28"]
    lm_t = ["lm4uu_thumb_-8", "lm4uu_thumb_-28"]
    return [
        Step("Linear bearings", lm_i + lm_t, {**{n: -Z * 35 for n in lm_i}, **{n: Z * 35 for n in lm_t}},
             [HW("bearing", "LM4UU 4 × 8 × 12", 4, dia=8)], show=["index_link", "thumb_link"],
             note="Press two LM4UU linear bearings into each finger link (index and thumb)."),
        Step("Rods and links", ["main_support", "index_link", "thumb_link", "rod_-8", "rod_-28",
                                "main_support_cover_plate"],
             {"rod_-8": Z * 170, "rod_-28": Z * 170, "index_link": -X * 45, "thumb_link": -X * 45,
              "main_support_cover_plate": Z * 90},
             [HW("pin", "Ø4 × 135 rod", 2, dia=4, length=135)], show=lm_i + lm_t,
             note="Hold both links in the main support and push the two Ø4 rods through the support and the links' "
                  "bearings. Close the rod ends with the cover plate."),
        Step("Encoder and linkage", ["servo_sts3215", "crank_mechanism_plate", "connecting_link_1",
                                     "connecting_link_2"],
             {"servo_sts3215": Y * 70, "crank_mechanism_plate": -X * 40, "connecting_link_1": -X * 70,
              "connecting_link_2": -X * 70},
             [HW("other", "STS3215 servo", 1), HW("bearing", "MR63ZZ 3 × 6 × 2.5", 4, dia=6),
              HW("screw", "M3 × 12", 4, length=12), HW("screw", "M3 × 15", 4, length=15),
              HW("screw", "M3 × 20", 2, length=20)],
             note="Seat the STS3215 (encoder, torque off) in the support. Fit the crank on its horn and join it to "
                  "both links with the connecting links on MR63ZZ bearings. Screws: HandUMI's mechanism set.",
             view=(-0.9, 0.9, -0.5)),
        Step("Hand support", ["hand_support_base"], {"hand_support_base": -Y * 60},
             [HW("strap", "hook-and-loop strap", 4)],
             note="Screw the hand support under the main support and thread the hook-and-loop straps through it.",
             view=(-0.8, -0.9, -0.7)),
        Step("Record button", ["record_button", "button_cover"], {"record_button": -Y * 30, "button_cover": -Y * 55},
             [HW("switch", "6 × 6 tactile switch + wire", 1)],
             note="Lay the switch's wire in the index link's channel, press the switch in and push the TPU cover "
                  "over it.", view=(-0.6, -0.9, -0.9), zoom=1.3),
        Step("Tips", ["tip_index", "tip_thumb"], {"tip_index": -X * 60, "tip_thumb": -X * 60},
             [HW("screw", "M2 screws + nuts", 4, dia=2, length=8)],
             note="Screw the tip set for your robot (here: Robotiq Hand-E) onto the two links."),
        Step("Electronics", ["pico2", "imu", "usb3_hub"], {"pico2": Y * 50, "imu": Y * 40, "usb3_hub": Y * 30},
             [HW("screw", "M2 screws (Pico)", 4, dia=2, length=6)], show=["electronics_box"], view=(0.5, 1.0, -0.6),
             note="Fit the Pico 2, the IMU and the USB 3 hub in the electronics box and wire them as in "
                  "docs/electronics.md."),
        Step("Electronics box", ["electronics_box", "pico2", "imu", "usb3_hub", "electronics_lid"],
             {"electronics_box": X * 70, "pico2": X * 70, "imu": X * 70, "usb3_hub": X * 70,
              "electronics_lid": X * 70 + Y * 45},
             [HW("screw", "M3 × 8", 4, length=8), HW("screw", "M3 × 6 (lid)", 4, length=6),
              HW("insert", "M3 heat-set insert", 8, dia=4, length=5)], view=BACK,
             note="Screw the box to the support's arm (4 × M3 × 8), then close the lid (4 × M3 × 6 into inserts)."),
        Step("Camera", ["camera_mount", "d405"], {"camera_mount": Y * 50, "d405": Y * 50 - X * 50},
             [HW("screw", "M3 × 5", 2, length=5), HW("screw", "M4 × 30 + nyloc nut", 1, dia=4, length=30,
                                                     head="socket")],
             note="Screw the D405 into its cup (2 × M3 × 5 into its rear holes). Hang the cup on the hinge with the M4 "
                  "× 30 bolt and nyloc nut, set the tilt and tighten."),
        Step("Controller support", ["controller_support"], {"controller_support": X * 70}, view=BACK,
             note="Bolt the controller support to the back of the main support and push the Quest controller "
                  "into its ring."),
    ]


PARTS = [("main_support", "main_support", 1), ("main_support_cover_plate", "main_support_cover_plate", 1),
         ("hand_support_base", "hand_support_base", 1), ("index_link", "index_link", 1),
         ("thumb_link", "thumb_link", 1), ("crank_mechanism_plate", "crank_mechanism_plate", 1),
         ("connecting_link_1", "connecting_link_1", 1), ("connecting_link_2", "connecting_link_2", 1),
         ("camera_mount", "camera_mount", 1), ("electronics_box", "electronics_box", 1),
         ("electronics_lid", "electronics_lid", 1), ("controller_support", "controller_support", 1),
         ("tip_index", "tip_index (your tip set)", 1), ("tip_thumb", "tip_thumb (your tip set)", 1),
         ("button_cover", "button_cover_tpu", 1)]


if __name__ == "__main__":
    m = load_glb(GLB)
    build(m, steps(), OUT, "HandUMI · right hand", UP, VIEW, parts=PARTS, title="HandUMI: assembly",
          parts_sub="Print these (hardware/STEP or STL, right/ for the right hand, left/ for the left).",
          intro="IKEA-style pages drawn from HandUMI's own assembly (`handumi/assembly/`): new parts in blue, pulled "
                "out along the way they go in, parts already built in grey, the hardware for each step at the top. "
                "Right hand shown; the left hand is the mirror image. Hardware as in [`../bom.md`](../bom.md).")
