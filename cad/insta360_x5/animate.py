"""Animated overview video segments for the README (dark background, 1280 x 720, 24 fps).

    python animate.py /path/to/Neuracore_Robots OUT_DIR [segment ...]

Writes OUT_DIR/<segment>.mp4 for each segment of this kit (see SEGMENTS); cad/make_video.py strings the segments of
both kits together. The parts move exactly as they do in the CAD (motion() in parts.py), and the robots' wrists are
drawn in their own colours from their descriptions (not redistributed).
"""
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh

from assembly import assemble
from parts import BUILDS, HERE, load_parts, motion, q_display, side_of

KIT = HERE.name
W, H, FPS = 1280, 720, 24
BG = "#1b2328"
SIDE = (0.0, 1.0, 0.0)               # look from +Y: the jaws point left, the robot docks from the right


def ease(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


class Scene:
    """One plotter with named actors; frames are written as numbered PNGs."""

    def __init__(self, frames_dir):
        self.p = pv.Plotter(off_screen=True, window_size=(W, H), lighting="three lights")
        self.p.set_background(BG)
        self.p.enable_anti_aliasing("ssaa")
        self.p.enable_ssao(radius=6.0, bias=0.3, kernel_size=64, blur=True)
        self.actors = {}
        self.dir = Path(frames_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.n = 0

    def add(self, name, mesh, colour=None, opacity=1.0):
        if colour is None:
            colour = mesh.visual.face_colors[0, :3] / 255.0
        a = self.p.add_mesh(pv.wrap(mesh), color=colour, smooth_shading=True, split_sharp_edges=True,
                            specular=0.3, specular_power=25, opacity=opacity)
        self.actors[name] = a
        return a

    def move(self, name, T):
        self.actors[name].user_matrix = np.asarray(T, float)

    def show(self, name, on):
        self.actors[name].SetVisibility(bool(on))

    def label(self, text, sub=""):
        self.p.add_text(text, position=(36, H - 64), font_size=20, color="white", name="label")
        self.p.add_text(sub, position=(36, H - 92), font_size=11, color="#9fb3c8", name="sub")

    def camera(self, focal, direction, dist, up=(0, 0, 1)):
        d = np.asarray(direction, float)
        d /= np.linalg.norm(d)
        self.p.camera_position = [tuple(np.asarray(focal) + d * dist), tuple(focal), up]

    def frame(self):
        self.p.render()
        self.p.screenshot(str(self.dir / f"{self.n:05d}.png"))
        self.n += 1

    def encode(self, out):
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(FPS), "-i", str(self.dir / "%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
        self.p.close()


def orbit_dir(az_deg, el_deg):
    """Camera direction from the scene centre: azimuth 0 = from +Y (side view), positive turns towards the front."""
    az, el = math.radians(az_deg), math.radians(el_deg)
    return (math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))


def add_build(sc, build, parts, prefix=""):
    meshes = assemble(build, q=0.0, parts=parts, dots=False)
    for n, m in meshes.items():
        sc.add(prefix + n, m)
    return meshes


def pose(sc, build, meshes, q, prefix=""):
    for n in meshes:
        if side_of(n):
            sc.move(prefix + n, motion(build, n, q))


def handheld(out, build, title, sub, parts, seconds=4.5):
    """Turn a handheld build from a three-quarter view into the side view while its jaws open and close."""
    with tempfile.TemporaryDirectory() as tmp:
        sc = Scene(tmp)
        meshes = add_build(sc, build, parts)
        lo = np.min([m.bounds[0] for m in meshes.values()], axis=0)
        hi = np.max([m.bounds[1] for m in meshes.values()], axis=0)
        c = (lo + hi) / 2
        dist = 1.45 * np.linalg.norm(hi - lo)
        sc.label(title, sub)
        nf = int(seconds * FPS)
        q1 = q_display(build, 0.85)
        for i in range(nf):
            t = i / (nf - 1)
            sc.camera(c, orbit_dir(65 - 65 * ease(t), 28 - 18 * ease(t)), dist)
            pose(sc, build, meshes, q1 * (0.5 - 0.5 * math.cos(2 * math.pi * min(t * 1.4, 1.0))))
            sc.frame()
        sc.encode(out)


def wrist_meshes(root, name):
    """The robot's last links (within 0.45 m of its flange) in the docking frame: origin on the flange face, +X along
    the approach (towards the gripper), +Z up; metres -> mm. Colours from the description."""
    import yourdfpy
    import robots as R
    spec = R.ROBOTS[name]
    path = Path(root) / spec["urdf"]
    r = yourdfpy.URDF.load(str(path), build_scene_graph=True, load_meshes=False)
    arm = R.Arm(r, "left_" if spec["dual"] else "")
    tcp = np.array(spec["tcp"]["left"]) - (0 if spec["dual"] else np.array(spec["bases"]["left"]))
    R.solve(arm, tcp, spec["approach"], spec["seed"], spec["elbow"])        # a natural reaching pose
    a_f = R.approach_in_flange(arm)
    T_dock = R.mount_frame(arm, a_f)
    F = T_dock[:3, 3]
    out = []
    S = np.diag([1e3, 1e3, 1e3, 1.0]) @ np.linalg.inv(T_dock)
    for m, rgb in R.link_meshes(r, path.parent):
        if spec["dual"] and m.bounds[0][1] > -0.05:            # keep the left arm only
            continue
        if np.linalg.norm(m.centroid - F) > 0.45:
            continue
        out.append((m.apply_transform(S), rgb))
    return out


def screws(kind):
    """Bolts that hold the plate to the robot (drawn as plain cylinders), in the gripper frame, before sliding."""
    import iso_flange_plate as ISO
    import openarm_adapter as OA
    out = []
    if kind == "iso50":
        for y, z in ISO.BOLTS:
            s = trimesh.creation.cylinder(radius=3.0, height=14.0, sections=24)
            s.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            s.apply_translation([ISO.CBORE_FLOOR_X + 7.0 - 0.0, y, z])
            head = trimesh.creation.cylinder(radius=5.0, height=6.0, sections=24)
            head.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            head.apply_translation([ISO.CBORE_FLOOR_X + 3.0, y, z])
            out.append(trimesh.util.concatenate([s, head]))
    else:
        for y, z in OA.FLANGE_HOLES:
            s = trimesh.creation.cylinder(radius=1.5, height=16.0, sections=20)
            s.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            s.apply_translation([OA.CBORE_FLOOR_X - 8.0, y, z])
            head = trimesh.creation.cylinder(radius=2.75, height=3.0, sections=20)
            head.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
            head.apply_translation([OA.CBORE_FLOOR_X + 1.5, y, z])
            out.append(trimesh.util.concatenate([s, head]))
    return out


def docking(out, root, family_name, parts, robots=(("ur5", "iso50", "UR5e"), ("abb_gofa", "iso50", "ABB GoFa"),
                                                       ("fanuc_crx", "iso50", "FANUC CRX"),
                                                       ("openarm", "openarm", "OpenArm")), per=4.0):
    """The robot twin, side on; for each robot the mount plate and its bolts slide in from the right, then the robot's
    wrist docks onto the plate, holds, and leaves."""
    import iso_flange_plate as ISO
    import openarm_adapter as OA
    from assembly import link7_in_gripper, paint, COLOURS
    with tempfile.TemporaryDirectory() as tmp:
        sc = Scene(tmp)
        base_build = f"{family_name}_iso50"
        meshes = add_build(sc, base_build, parts)
        plate_names = [n for n in meshes if n in ("iso_flange_plate", "openarm_adapter")]
        for n in plate_names:
            sc.show(n, False)
        oa_plate = paint(assemble(f"{family_name}_openarm", q=0.0, parts=parts, dots=False)["openarm_adapter"].copy(),
                         COLOURS["blue"])
        sc.add("oa_plate", oa_plate)
        sc.show("oa_plate", False)
        groups = {}
        for key, kind, label in robots:
            names = []
            plate = "iso_flange_plate" if kind == "iso50" else "oa_plate"
            names.append(plate)
            for i, s in enumerate(screws(kind)):
                nm = f"{key}_screw{i}"
                sc.add(nm, s, "#c9ced4")
                names.append(nm)
            if key == "openarm":
                l7 = link7_in_gripper(Path(root) / "example_openarm/openarm_description")
                sc.add(f"{key}_wrist0", l7, "#d9dde2")
                names.append(f"{key}_wrist0")
            else:
                for i, (m, rgb) in enumerate(wrist_meshes(root, key)):
                    m.apply_translation([-ISO.T, 0, 0])
                    sc.add(f"{key}_wrist{i}", m, rgb)
                    names.append(f"{key}_wrist{i}")
            groups[key] = (kind, label, names)
            for n in names:
                sc.show(n, False)
        lo = np.min([m.bounds[0] for m in meshes.values()], axis=0)
        hi = np.max([m.bounds[1] for m in meshes.values()], axis=0)
        c = (lo + hi) / 2 + np.array([-45.0, 0, 0])
        dist = 1.9 * np.linalg.norm(hi - lo)
        q = q_display(base_build, 0.35)
        pose(sc, base_build, meshes, q)
        sc.camera(c, orbit_dir(0, 8), dist)
        nf = int(per * FPS)
        for key, (kind, label, names) in groups.items():
            sc.label(label, "ISO 9409-1-50-4-M6 plate" if kind == "iso50" else "OpenArm link7 adapter")
            for n in names:
                sc.show(n, True)
            for i in range(nf):
                t = i / (nf - 1)
                plate_in = 1 - ease(t / 0.25)                     # 0 .. 1 s: plate and bolts slide in
                robot_in = 1 - ease((t - 0.2) / 0.35)             # 0.8 .. 2.2 s: the wrist docks
                leave = ease((t - 0.85) / 0.15)                   # last 0.6 s: everything slides out
                for n in names:
                    off = 260.0 * (robot_in if "wrist" in n else plate_in) + 400.0 * leave
                    sc.move(n, trimesh.transformations.translation_matrix([-off, 0, 0]))
                sc.frame()
            for n in names:
                sc.show(n, False)
        sc.encode(out)


def hand(out, parts, seconds=4.5):
    """The X5 on the wrist, with a stand-in forearm and hand, turning slowly."""
    import hand as Hd
    import pod as PD
    with tempfile.TemporaryDirectory() as tmp:
        sc = Scene(tmp)
        meshes = add_build(sc, "hand", parts)
        xc = (PD.X0 + PD.X1) / 2
        a = math.radians(Hd.PITCH)
        arm = trimesh.creation.capsule(height=230, radius=Hd.R_ARM - 1.0, count=[48, 48])
        arm.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2 - a, [0, 1, 0]))
        z_top = PD.Z_BOT - PD.LID - Hd.MIN_T
        x1 = xc + Hd.LENGTH / 2
        arm.apply_translation([x1 - 10 * math.cos(a), 0, z_top - Hd.R_ARM - 10 * math.sin(a) + 1.0])
        sc.add("arm", arm, "#d8b59a")
        lo = np.min([m.bounds[0] for m in meshes.values()], axis=0)
        hi = np.max([m.bounds[1] for m in meshes.values()], axis=0)
        c = (lo + hi) / 2
        dist = 1.9 * np.linalg.norm(hi - lo)
        sc.label("Hand", "the camera rides on your wrist")
        nf = int(seconds * FPS)
        for i in range(nf):
            t = i / (nf - 1)
            sc.camera(c, orbit_dir(70 - 80 * ease(t), 22), dist)
            sc.frame()
        sc.encode(out)


SEGMENTS = {
    "x5_chopstick": lambda root, out, P: handheld(out, "chopstick_right", "Insta360 X5 · Chopstick",
                                                  "pinch the geared jaws; the 360° camera rides on top", P),
    "x5_parallel": lambda root, out, P: handheld(out, "parallel_right", "Insta360 X5 · Parallel",
                                                 "rack-and-pinion fingers, 0 to 60 mm", P),
    "x5_docking": lambda root, out, P: docking(out, root, "chopstick", P),
    "x5_hand": lambda root, out, P: hand(out, P),
}


def main():
    root, out_dir = sys.argv[1], Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    names = sys.argv[3:] or list(SEGMENTS)
    parts = load_parts()
    for n in names:
        SEGMENTS[n](root, out_dir / f"{n}.mp4", parts)
        print("wrote", out_dir / f"{n}.mp4")


if __name__ == "__main__":
    main()
