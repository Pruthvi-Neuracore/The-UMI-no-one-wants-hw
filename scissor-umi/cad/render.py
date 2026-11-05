"""README renders (pyvista, off screen) to ../2f-140/img/ (or --out).

    python render.py              # all
    python render.py hero robot   # some
"""
import argparse
import math
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

import assembly as A
import kin
import scissor as S

OUT = S.HERE.parent / kin.GRIPPER / "img"
Q_MAX = kin.g()["q_max"]


def add(p, m):
    c = m.visual.face_colors[0, :3] / 255.0
    p.add_mesh(pv.wrap(m), color=c, smooth_shading=True, split_sharp_edges=True, specular=0.3, specular_power=25)


def plotter(size=(1600, 1100)):
    p = pv.Plotter(off_screen=True, window_size=size, lighting="three lights")
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    p.enable_ssao(radius=6.0, bias=0.3, kernel_size=128, blur=True)
    return p


def tidy(path, aspect=1.6, margin=0.07):
    from PIL import Image, ImageChops
    im = Image.open(path).convert("RGB")
    w0, h0 = im.size
    im = im.crop((3, 3, w0 - 3, h0 - 3))
    bg = Image.new("RGB", im.size, (255, 255, 255))
    box = ImageChops.difference(im, bg).convert("L").point(lambda v: 255 if v > 8 else 0).getbbox()
    if not box:
        return
    im = im.crop(box)
    w, h = im.size
    W = max(w * (1 + 2 * margin), h * (1 + 2 * margin) * aspect)
    out = Image.new("RGB", (int(W), int(W / aspect)), (255, 255, 255))
    out.paste(im, ((int(W) - w) // 2, (int(W / aspect) - h) // 2))
    out.save(path)


def shot(p, name, pos, focal, up=(1, 0, 0), zoom=1.0, aspect=1.6):
    path = OUT / name
    p.camera_position = [pos, focal, up]
    p.reset_camera()
    p.camera.zoom(zoom)
    p.screenshot(str(path))
    p.close()
    tidy(path, aspect)
    print("wrote", path)


def place(p, meshes, T=np.eye(4)):
    for m in meshes.values():
        add(p, m.copy().apply_transform(T))


def tr(v):
    return trimesh.transformations.translation_matrix(v)


VIEW = np.array([150.0, -330.0, 260.0])     # from up, right and in front (G: x up, y left-right, z forward)


def hero():
    p = plotter()
    place(p, A.handheld(0.25))
    shot(p, "hero.png", tuple(np.array([60, 0, 60]) + VIEW * 1.4), (60, 0, 60), zoom=1.25)


def robot():
    p = plotter()
    place(p, A.robot(0.25))                     # the real gripper, the team's camera plate and the D405
    shot(p, "robot_2f140.png", tuple(np.array([60, 0, 60]) + VIEW * 1.4), (60, 0, 60), zoom=1.25)


def twins():
    """Handheld and robot gripper side by side at the same q: the camera, the controller and the pads line up."""
    p = plotter((2200, 1100))
    gap = 260.0
    place(p, A.handheld(0.3), tr([0, -gap / 2, 0]))
    place(p, A.robot(0.3), tr([0, gap / 2, 0]))
    shot(p, "twins.png", (330, 0, 480), (60, 0, 40), zoom=1.45, aspect=2.0)


def top():
    """From above: the scissor handles, the gears and the four-bars (head and electronics hidden)."""
    for q, name in ((0.0, "top_open.png"), (Q_MAX, "top_closed.png")):
        p = plotter((1400, 1600))
        m = A.handheld(q, quest=False)
        for k in ("tab_mast", "d405_hood", "d405", "quest_arm", "base_top"):
            m.pop(k, None)
        place(p, m)
        shot(p, name, (600, 0, 60), (0, 0, 60), up=(0, 0, 1), zoom=1.1, aspect=0.9)


def stroke():
    p = plotter((2400, 900))
    qs = (0.0, Q_MAX / 3, 2 * Q_MAX / 3, Q_MAX)
    for i, q in enumerate(qs):
        m = A.handheld(q, quest=False)
        for k in ("tab_mast", "d405_hood", "d405", "quest_arm", "base_top"):
            m.pop(k, None)
        place(p, m, tr([0, 0, -i * 330]) @ trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    shot(p, "stroke.png", (700, 0, -495), (0, 0, -495), up=(0, 1, 0), zoom=1.7, aspect=2.6)


def camera_views():
    """What the D405 sees on each build (pinhole, the D405's 87° x 58° colour field of view), same q."""
    from PIL import Image
    T = S.camera_pose()
    eye = T[:3, 3]
    view, down = T[:3, 2], T[:3, 1]
    ims = []
    for name, f in (("handheld", A.handheld), ("robot", A.robot)):
        p = plotter((1280, 720))
        p.disable_ssao() if hasattr(p, "disable_ssao") else None
        m = f(0.3, quest=False)
        for k in ("d405", "d405_hood"):
            m.pop(k, None)
        place(p, m)
        p.camera_position = [tuple(eye), tuple(eye + view * 100), tuple(-down)]
        p.camera.view_angle = 58.0
        p.camera.clipping_range = (5.0, 2000.0)
        path = OUT / f"_view_{name}.png"
        p.screenshot(str(path))
        p.close()
        ims.append(Image.open(path))
        path.unlink()
    w, h = ims[0].size
    out = Image.new("RGB", (2 * w + 20, h), "white")
    out.paste(ims[0], (0, 0))
    out.paste(ims[1], (w + 20, 0))
    out.save(OUT / "camera_views.png")
    print("wrote", OUT / "camera_views.png")


def electronics():
    """From below with the box's cover hidden: the STS3215 on the scissor axle, the Pico 2, the hub and the IMU."""
    p = plotter((1600, 1100))
    m = A.handheld(0.25, quest=False)
    m.pop("ebox_cover", None)
    place(p, m)
    shot(p, "electronics.png", (-420, -260, 120), (-40, 0, 40), up=(1, 0, 0), zoom=1.5)


def hand_view():
    """The grips from in front, below and to the side: the palm bar, the finger loop and the button pod, for the
    right-hand build (grip.png) and the left-hand build, its mirror (grip_left.png)."""
    for side, name, sy in (("right", "grip.png", -1), ("left", "grip_left.png", 1)):
        p = plotter((1600, 1100))
        place(p, A.handheld(0.35, side))
        shot(p, name, (-300, sy * 330, 260), (0, 0, -40), up=(1, 0, 0), zoom=1.2)


def hands():
    """The left-hand and the right-hand builds side by side, from in front and above."""
    p = plotter((1800, 1000))
    gap = 300.0
    place(p, A.handheld(0.2, "left"), tr([0, -gap / 2, 0]))
    place(p, A.handheld(0.2, "right"), tr([0, gap / 2, 0]))
    shot(p, "hands.png", (520, 0, 420), (30, 0, -10), up=(1, 0, 0), zoom=1.15, aspect=1.8)


ALL = dict(hero=hero, robot=robot, twins=twins, top=top, stroke=stroke, camera_views=camera_views,
           electronics=electronics, grip=hand_view, hands=hands)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("which", nargs="*")
    ap.add_argument("--out")
    a = ap.parse_args()
    if a.out:
        OUT = Path(a.out)
    OUT.mkdir(parents=True, exist_ok=True)
    for n in a.which or ALL:
        ALL[n]()
