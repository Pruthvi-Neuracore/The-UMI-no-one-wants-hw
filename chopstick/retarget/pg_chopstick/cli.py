"""Retarget a recorded hand trajectory onto OpenArm or an ISO-flange cobot, offline.

Input CSV, one row per frame, with a header:
    t, x, y, z, qx, qy, qz, qw, theta
where (x .. qw) is the pose of the hand's camera frame (X5 or D405, see --camera) in the demo world (m, quaternion xyzw) and theta is the jaw
angle in rad. Give `gap` (tip gap, m) instead of `theta` if that is what you have.

Output CSV: t, flange pose (OpenArm link7 / tool flange; columns named link7_*) in the arm base frame, tcp pose, and the gripper command
(theta and servo_steps for --ee chopstick, finger for --ee stock).

    pg-chopstick-retarget demo_right.csv -o right_openarm.csv [--ee chopstick|stock] [--base-world T.npy]
                       [--servo-zero 2048] [--servo-sign 1]
"""
import argparse
import csv

import numpy as np

from .gripper import ChopstickKinematics
from .retarget import Retargeter, ServoMap, matrix_to_pose, pose_to_matrix


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--ee", choices=["chopstick", "stock"], default="chopstick")
    ap.add_argument("--camera", choices=["x5", "d405"], default="x5", help="camera the demo was recorded with")
    ap.add_argument("--mount", choices=["openarm", "iso50"], default="openarm",
                    help="openarm adapter (link7) or ISO 9409-1-50-4-M6 plate (tool flange)")
    ap.add_argument("--base-world", help=".npy 4x4 pose of the demo world in the arm base frame (default identity)")
    ap.add_argument("--servo-zero", type=int, default=2048)
    ap.add_argument("--servo-sign", type=int, choices=[1, -1], default=1)
    a = ap.parse_args(argv)

    kin = ChopstickKinematics.load(a.camera, a.mount)
    T_bw = np.load(a.base_world) if a.base_world else np.eye(4)
    rt = Retargeter(T_bw, a.ee, kin, ServoMap(a.servo_zero, a.servo_sign, theta_max=kin.theta_max))
    with open(a.input) as f, open(a.output, "w", newline="") as g:
        rows = csv.DictReader(f)
        cmd = ["theta", "servo_steps"] if a.ee == "chopstick" else ["finger"]
        pose = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        w = csv.writer(g)
        w.writerow(["t"] + [f"link7_{k}" for k in pose] + [f"tcp_{k}" for k in pose] + cmd)
        n = 0
        for r in rows:
            T = pose_to_matrix([float(r[k]) for k in "xyz"], [float(r[k]) for k in ("qx", "qy", "qz", "qw")])
            theta = float(r["theta"]) if "theta" in r and r["theta"] != "" else kin.theta_from_gap(float(r["gap"]))
            out = rt(T, theta)
            l7, tcp = matrix_to_pose(out["T_base_link7"]), matrix_to_pose(out["T_base_tcp"])
            w.writerow([r["t"], *np.round(np.r_[l7[0], l7[1]], 6), *np.round(np.r_[tcp[0], tcp[1]], 6),
                        *[out[k] if isinstance(out[k], int) else round(out[k], 6) for k in cmd]])
            n += 1
    print(f"retargeted {n} frames -> {a.output}")


if __name__ == "__main__":
    main()
