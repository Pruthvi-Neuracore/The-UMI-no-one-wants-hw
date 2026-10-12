from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pg_chopstick import ChopstickKinematics, OpenArmRetargeter, Retargeter, ServoMap
from pg_chopstick.retarget import T_LINK7_STOCK_TCP

URDF = Path(__file__).resolve().parents[2] / "urdf"
kin = ChopstickKinematics.load()


def random_pose(rng):
    T = np.eye(4)
    T[:3, :3] = Rotation.random(random_state=rng).as_matrix()
    T[:3, 3] = rng.uniform(-0.5, 0.5, 3)
    return T


def test_closed_tips_touch_and_gap_grows():
    assert abs(kin.tip_gap(0.0)) < 1e-3
    g = kin.tip_gap(np.radians(np.arange(0, 60, 5)))
    assert np.all(np.diff(g) > 0)


@pytest.mark.parametrize("deg", [0.0, 3.0, 17.5, 44.2, 60.0])
def test_gap_roundtrip(deg):
    th = np.radians(deg)
    assert abs(kin.theta_from_gap(kin.tip_gap(th)) - th) < 1e-3


@pytest.mark.parametrize("frame", ["gripper", "camera"])
def test_theta_from_dots(frame):
    rng = np.random.default_rng(0)
    for deg in (0.0, 12.0, 35.0):
        th = np.radians(deg)
        d = kin.dots_at(th, frame)
        noisy = {s: p + rng.normal(0, 0.5e-3, p.shape) for s, p in d.items()}   # 0.5 mm noise
        noisy["right"][1] = np.nan                                             # one dot not seen
        est = kin.theta_from_dots(noisy["left"], noisy["right"], frame)
        assert abs(np.degrees(est) - deg) < 0.6
    assert abs(kin.theta_from_dots(left=kin.dots_at(0.3)["left"]) - 0.3) < 1e-9


def test_pg_chopstick_puts_robot_camera_on_demo_camera():
    rng = np.random.default_rng(1)
    T_bw = random_pose(rng)
    rt = OpenArmRetargeter(T_bw)
    for _ in range(5):
        T_wc = random_pose(rng)
        out = rt(T_wc, np.radians(20))
        np.testing.assert_allclose(out["T_base_link7"] @ kin.T_link7_camera, T_bw @ T_wc, atol=1e-12)
        np.testing.assert_allclose(out["T_base_link7"] @ kin.T_link7_tcp, out["T_base_tcp"], atol=1e-12)


def test_stock_retarget_matches_tcp():
    rng = np.random.default_rng(2)
    rt = OpenArmRetargeter(end_effector="stock")
    T_wc = random_pose(rng)
    out = rt(T_wc, np.radians(5))
    np.testing.assert_allclose(out["T_base_link7"] @ T_LINK7_STOCK_TCP, out["T_base_tcp"], atol=1e-12)
    assert 0 < out["finger"] <= 0.044


def test_servo_map():
    s = ServoMap(zero=1000, sign=-1, theta_max=np.radians(60))
    assert s.steps(0.0) == 1000
    assert s.steps(np.pi / 2) == 1000 - 683           # clipped to 60 deg = 682.7 steps
    assert abs(s.theta(s.steps(0.4)) - 0.4) < 2 * np.pi / 4096


BUILDS = {"data": ("x5", None), "data_d405": ("d405", None), "openarm": ("x5", "openarm"),
          "openarm_d405": ("d405", "openarm"), "iso50": ("x5", "iso50"), "iso50_d405": ("d405", "iso50")}


@pytest.mark.parametrize("build", BUILDS)
def test_urdf_matches_json(build):
    yourdfpy = pytest.importorskip("yourdfpy")
    camera, mount = BUILDS[build]
    k = ChopstickKinematics.load(camera, mount or "openarm")
    r = yourdfpy.URDF.load(str(URDF / f"pg_chopstick_{build}.urdf"), load_meshes=False)
    np.testing.assert_allclose(r.get_transform("chopstick_tcp", "chopstick_camera"), k.T_camera_tcp, atol=1e-6)
    r.update_cfg({"chopstick_jaw": 0.35})
    tl = r.get_transform("chopstick_tip_left", "chopstick_base")[:3, 3]
    tr = r.get_transform("chopstick_tip_right", "chopstick_base")[:3, 3]
    assert abs(np.linalg.norm(tl - tr) - k.tip_gap(0.35)) < 1e-6
    if mount:
        np.testing.assert_allclose(r.get_transform("chopstick_camera", "chopstick_flange"), k.T_flange_camera,
                                   atol=1e-6)


@pytest.mark.parametrize("camera", ["x5", "d405"])
@pytest.mark.parametrize("mount", ["openarm", "iso50"])
def test_retarget_any_camera_and_mount(camera, mount):
    rng = np.random.default_rng(3)
    k = ChopstickKinematics.load(camera, mount)
    rt = Retargeter(camera=camera, mount=mount)
    T_wc = random_pose(rng)
    out = rt(T_wc, 0.2)
    np.testing.assert_allclose(out["T_base_link7"] @ k.T_flange_camera, T_wc, atol=1e-12)


def test_cameras_share_the_tcp():
    """Both cameras see the same TCP: their camera -> TCP transforms differ, the TCP in the gripper frame does not."""
    a, b = ChopstickKinematics.load("x5"), ChopstickKinematics.load("d405")
    np.testing.assert_allclose(a.T_gripper_camera @ a.T_camera_tcp, b.T_gripper_camera @ b.T_camera_tcp, atol=1e-12)
    assert not np.allclose(a.T_camera_tcp, b.T_camera_tcp)


def test_stock_hand_needs_openarm():
    with pytest.raises(ValueError):
        Retargeter(end_effector="stock", mount="iso50")
