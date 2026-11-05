"""Open-ENPIRE compliant finger for the I2RT YAM, as a tip, unmodified.

Source: pgeedh/Open-ENPIRE-Gripper, grippers-stl/enpire_i2rt_yam_stl (Wenli Xiao's ENPIRE finger: rigid jaw + soft
insert, Apache-2.0, copied to source/enpire/). It keeps its own YAM mount: the 18.7 x 3 x 5 mm key and the two 4.6 mm
holes 21 mm apart, which bolt straight onto the flange's YAM interface (finger_interfaces.INTERFACES["yam"]) with the
YAM's own 2x M4 screws, put in from the finger side. The second jaw is the mirror image across the closing axis.

Frame (the source's): y along the finger (the mount at the top, y = -7.8 the mating face), z the closing axis (the
soft gripping face at low z), x across.

    python enpire_tip.py      # -> ../hardware/STL/gripper_tips/Open-ENPIRE/ (mesh-derived, so STL only)
"""
from pathlib import Path

import numpy as np
import trimesh

SRC = Path(__file__).resolve().parent / "source/enpire"
OUT = Path(__file__).resolve().parents[1] / "hardware"
CZ = 12.0                                    # middle of the finger's base block along the closing axis


def main():
    hard = trimesh.load(SRC / "i2rt_UCG_Hard.stl")
    soft = trimesh.load(SRC / "I2RT_UCG_Soft.stl")
    mirror = np.eye(4)
    mirror[2, 2], mirror[2, 3] = -1.0, 2 * CZ                         # mirror across z = CZ
    (OUT / "STL/gripper_tips/Open-ENPIRE").mkdir(parents=True, exist_ok=True)      # mesh-derived: STL only
    for side, M in (("RIGHT", np.eye(4)), ("LEFT", mirror)):
        for name, mesh in (("Jaw", hard), ("Soft-Insert", soft)):
            m = mesh.copy()
            m.apply_transform(M)
            stl = OUT / "STL/gripper_tips/Open-ENPIRE" / f"ENPIRE-{side}-{name}.stl"
            m.export(stl)
            print(stl.name, "watertight", m.is_watertight, "volume", round(m.volume))


if __name__ == "__main__":
    main()
