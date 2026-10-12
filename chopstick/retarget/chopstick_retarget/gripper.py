"""Chopstick gripper kinematics: jaw angle <-> tip gap, jaw angle from the red dots, and the fixed frames.

All numbers come from chopstick_kinematics.json, which cad/kinematics.py measures from the CAD. Lengths are in metres
here (the JSON stores millimetres). Jaw angle `theta` is one jaw's opening from closed, in radians; the two jaws are
geared 1:-1, so the left jaw turns +theta about the gripper +Z axis and the right jaw -theta.
"""
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

KIN_JSON = Path(__file__).with_name("chopstick_kinematics.json")


def _mm_to_m(T):
    T = np.array(T, float)
    T[:3, 3] *= 1e-3
    return T


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


@dataclass
class ChopstickKinematics:
    pivot_left: np.ndarray
    pivot_right: np.ndarray
    tip_left: np.ndarray            # tip ends at theta = 0, gripper frame
    tip_right: np.ndarray
    T_gripper_camera: np.ndarray
    T_gripper_tcp: np.ndarray
    T_link7_gripper: np.ndarray     # OpenArm adapter
    theta_max: float                # clash-free opening of one jaw (robot gripper)
    dots: dict = field(default_factory=dict)   # {"left": (N, 3), "right": (N, 3)} red-dot centres at theta = 0

    @classmethod
    def load(cls, path=KIN_JSON):
        k = json.loads(Path(path).read_text())
        T_tcp = np.eye(4)
        T_tcp[:3, 3] = np.array(k["tcp_mm"]) * 1e-3
        lim = k.get("theta_max_deg", {}).get("robot_gripper", 60.0)
        return cls(
            pivot_left=np.array(k["pivot_left_mm"]) * 1e-3,
            pivot_right=np.array(k["pivot_right_mm"]) * 1e-3,
            tip_left=np.array(k["tip_end_left_mm"]) * 1e-3,
            tip_right=np.array(k["tip_end_right_mm"]) * 1e-3,
            T_gripper_camera=_mm_to_m(k["camera"]["T_gripper_camera_mm"]),
            T_gripper_tcp=T_tcp,
            T_link7_gripper=np.array(k["openarm"]["T_link7_gripper_m"], float),
            theta_max=np.radians(lim),
            dots={"left": np.array([d["centre_mm"] for d in k["red_dots"]["jaw_left_dc"]]) * 1e-3,
                  "right": np.array([d["centre_mm"] for d in k["red_dots"]["jaw_right"]]) * 1e-3},
        )

    # --- jaw angle and tip gap -------------------------------------------------------------------------------------
    def jaw_point(self, side, p0, theta):
        """Where a point p0 on a jaw (gripper frame, theta = 0) is when the jaws are open by theta."""
        piv = self.pivot_left if side == "left" else self.pivot_right
        a = theta if side == "left" else -theta
        p0 = np.asarray(p0, float)
        return (rot_z(a) @ (p0 - piv).T).T + piv

    def tip_gap(self, theta):
        """Distance between the two TPU tip ends (m) at jaw angle theta (rad). Vectorised over theta.

        Signed: negative while the tips still overlap. In the CAD the closed tips cross by about 0.17 mm (the TPU
        preload), so the gap is slightly negative at theta = 0 and passes through zero just after.
        """
        theta = np.asarray(theta, float)
        out = np.empty_like(theta, dtype=float)
        for i, t in np.ndenumerate(theta):
            d = self.jaw_point("left", self.tip_left, t) - self.jaw_point("right", self.tip_right, t)
            out[i] = np.copysign(np.linalg.norm(d), d[1])
        return out if out.shape else float(out)

    def theta_from_gap(self, gap):
        """Inverse of tip_gap on [0, theta_max]; gaps outside the range are clipped."""
        grid = np.linspace(0.0, self.theta_max, 2001)
        return np.interp(gap, self.tip_gap(grid), grid)

    # --- jaw angle from the red dots -------------------------------------------------------------------------------
    def theta_from_dots(self, left=None, right=None, frame="gripper"):
        """Least-squares jaw angle from observed red-dot positions (m).

        left / right: (N, 3) arrays of the dots on each jaw in the same order as `self.dots` (rear to front); rows of
        NaN mark dots that were not seen. frame: "gripper" or "camera" (the X5 frame). Each dot gives an angle about
        its jaw's pivot in the XY plane; the result is the weighted circular mean over both jaws (the right jaw's sign
        flipped, since the gears turn it the other way).
        """
        T = np.eye(4) if frame == "gripper" else self.T_gripper_camera
        angles, weights = [], []
        for side, obs in (("left", left), ("right", right)):
            if obs is None:
                continue
            obs = np.atleast_2d(np.asarray(obs, float))
            ok = ~np.isnan(obs).any(axis=1)
            if not ok.any():
                continue
            pts = (T[:3, :3] @ obs[ok].T).T + T[:3, 3]
            piv = self.pivot_left if side == "left" else self.pivot_right
            ref = self.dots[side][ok] - piv
            cur = pts - piv
            a = np.arctan2(ref[:, 0] * cur[:, 1] - ref[:, 1] * cur[:, 0], (ref[:, :2] * cur[:, :2]).sum(1))
            angles.append(a if side == "left" else -a)
            weights.append(np.linalg.norm(ref[:, :2], axis=1))   # dots further from the pivot resolve angle better
        if not angles:
            raise ValueError("no dots observed")
        a, w = np.concatenate(angles), np.concatenate(weights)
        return float(np.arctan2((w * np.sin(a)).sum(), (w * np.cos(a)).sum()))

    def dots_at(self, theta, frame="gripper"):
        """Red-dot positions at jaw angle theta, in the gripper or camera frame: {"left": (N, 3), "right": (N, 3)}."""
        Tinv = np.eye(4) if frame == "gripper" else np.linalg.inv(self.T_gripper_camera)
        out = {}
        for side in ("left", "right"):
            p = self.jaw_point(side, self.dots[side], theta)
            out[side] = (Tinv[:3, :3] @ p.T).T + Tinv[:3, 3]
        return out

    # --- fixed frames ----------------------------------------------------------------------------------------------
    @property
    def T_camera_tcp(self):
        return np.linalg.inv(self.T_gripper_camera) @ self.T_gripper_tcp

    @property
    def T_link7_camera(self):
        return self.T_link7_gripper @ self.T_gripper_camera

    @property
    def T_link7_tcp(self):
        return self.T_link7_gripper @ self.T_gripper_tcp
