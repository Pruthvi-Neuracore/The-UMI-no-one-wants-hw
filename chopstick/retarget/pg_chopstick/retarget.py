"""Retarget PG Chopstick demonstrations onto a robot arm: OpenArm, or any cobot with an ISO 9409-1-50-4-M6 flange.

A demonstration gives, per frame, the pose of each hand's camera (Insta360 X5 or RealSense D405, from any SLAM /
visual-inertial odometry that tracks it) and the jaw angle (from the red dots). Retargeting turns that into what the robot needs per frame:

  * the flange target pose (OpenArm link7, or the cobot's tool flange, e.g. UR tool0) for the arm's IK, and
  * the gripper command: the jaw angle and the servo position for the chopstick robot gripper, or the finger opening
    for the stock OpenArm parallel hand.

With the chopstick robot gripper the robot carries the same jaws and the same camera mount as the handheld gripper, so
the mapping is exact: the robot's camera should be where the demo camera was, which fixes link7, and the jaw angle is
copied unchanged. With the stock hand, the TCP is matched instead and the tip gap becomes the finger opening.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .gripper import ChopstickKinematics

# Stock OpenArm hand (openarm_description): hand at link7 z = 0.1001, tcp 0.08 further with rpy (0, -pi/2, pi);
# fingers are prismatic 0..0.044 m each, mirrored.
_T_HAND = np.eye(4)
_T_HAND[2, 3] = 0.1001
_T_HAND_TCP = np.eye(4)
_T_HAND_TCP[:3, :3] = Rotation.from_euler("xyz", [0, -np.pi / 2, np.pi]).as_matrix()
_T_HAND_TCP[2, 3] = 0.08
T_LINK7_STOCK_TCP = _T_HAND @ _T_HAND_TCP
STOCK_FINGER_MAX = 0.044


def pose_to_matrix(xyz, quat_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = xyz
    return T


def matrix_to_pose(T):
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_quat()


@dataclass
class ServoMap:
    """Jaw angle -> servo position for the robot gripper's servo (Feetech SM40BL, 4096 steps per turn).

    The servo's rotor plate is screwed straight onto the left (driver) jaw, so one servo step is one jaw step.
    Calibrate `zero` by reading the servo position with the jaws closed (tips touching), and flip `sign` if the jaws
    close when they should open.
    """
    zero: int = 2048
    sign: int = 1
    steps_per_rev: int = 4096
    theta_max: float = np.radians(60.0)

    def steps(self, theta):
        theta = np.clip(theta, 0.0, self.theta_max)
        return np.rint(self.zero + self.sign * theta * self.steps_per_rev / (2 * np.pi)).astype(int)

    def theta(self, steps):
        return (np.asarray(steps, float) - self.zero) * self.sign * 2 * np.pi / self.steps_per_rev


class Retargeter:
    """Per-frame retargeting of one hand onto one arm.

    T_base_world: pose of the demo's world frame in the robot arm's base frame (where the scene sits relative to the
    robot). Identity if the demo was already expressed in the robot base frame.
    end_effector: "chopstick" (the PG Chopstick robot gripper on openarm_adapter) or "stock" (OpenArm parallel hand).
    camera: "x5" or "d405" -- the camera the demo was recorded with (the robot gripper must carry the same one).
    mount: "openarm" (openarm_adapter, flange = link7) or "iso50" (iso_flange_plate, flange = the tool flange).
    The outputs keep the OpenArm-era key "T_base_link7"; it is the flange pose for either mount.
    """

    def __init__(self, T_base_world=np.eye(4), end_effector="chopstick", kin=None, servo=None,
                 theta_max=None, camera="x5", mount="openarm"):
        self.kin = kin or ChopstickKinematics.load(camera, mount)
        if end_effector == "stock" and self.kin.mount != "openarm":
            raise ValueError("the stock-hand mapping is for OpenArm's parallel hand (mount='openarm')")
        self.T_base_world = np.asarray(T_base_world, float)
        self.ee = end_effector
        self.theta_max = self.kin.theta_max if theta_max is None else theta_max
        self.servo = servo or ServoMap(theta_max=self.theta_max)
        if end_effector == "chopstick":
            self._T_cam_link7 = np.linalg.inv(self.kin.T_flange_camera)
        elif end_effector == "stock":
            # match the TCP: link7 = camera * (camera -> tcp) * (tcp -> link7) with the stock hand's tcp
            self._T_cam_link7 = self.kin.T_camera_tcp @ self._tcp_align() @ np.linalg.inv(T_LINK7_STOCK_TCP)
        else:
            raise ValueError(f"unknown end effector {end_effector!r}")

    def _tcp_align(self):
        """Rotation from the chopstick TCP axes to the stock hand TCP axes, chosen so both close along the same line.

        Chopstick TCP: +X along the jaws, +Y across them (the opening direction). Stock hand TCP (from its URDF): the
        rpy (0, -pi/2, pi) maps link7 +Z (approach) to TCP +X and link7 +Y (finger motion) to TCP -Y. So the two TCP
        frames already agree: approach along +X, closing along Y.
        """
        return np.eye(4)

    def link7_target(self, T_world_camera):
        """OpenArm link7 pose in the arm base frame that puts the robot's camera (or TCP) where the demo's was."""
        return self.T_base_world @ np.asarray(T_world_camera, float) @ self._T_cam_link7

    def tcp_target(self, T_world_camera):
        return self.T_base_world @ np.asarray(T_world_camera, float) @ self.kin.T_camera_tcp

    def gripper_command(self, theta):
        """{"theta": rad, "servo_steps": int} for the chopstick gripper, {"finger": m} for the stock hand."""
        theta = float(np.clip(theta, 0.0, self.theta_max))
        if self.ee == "chopstick":
            return {"theta": theta, "servo_steps": int(self.servo.steps(theta))}
        gap = self.kin.tip_gap(theta)
        return {"finger": float(np.clip(gap / 2, 0.0, STOCK_FINGER_MAX))}

    def __call__(self, T_world_camera, theta):
        out = {"T_base_link7": self.link7_target(T_world_camera), "T_base_tcp": self.tcp_target(T_world_camera)}
        out.update(self.gripper_command(theta))
        return out


OpenArmRetargeter = Retargeter
