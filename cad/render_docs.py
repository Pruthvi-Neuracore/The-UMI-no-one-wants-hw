"""Render the README images.  python render_docs.py"""
import numpy as np
import pyvista as pv
from build123d import import_step

from assembly import BLUE, DARK, HW, PCB, ROOT, WHITE, assemble, cables, d405_dummy
from occ import place

pv.OFF_SCREEN = True
IMG = ROOT / "docs/img"
BG = "#e6eaef"


def poly(shape, tol=0.06):
    v, f = shape.tessellate(tol, 0.2)
    return pv.PolyData(np.array([(q.X, q.Y, q.Z) for q in v]), np.hstack([np.full((len(f), 1), 3), np.array(f)]).ravel())


def plotter(shape, size):
    pl = pv.Plotter(off_screen=True, shape=shape, window_size=size, border=False)
    pl.enable_anti_aliasing("ssaa")
    return pl


def show(pl, items, view, up, title=None, zoom=1.2):
    pl.set_background(BG)
    for m, c in items:
        pl.add_mesh(m, color=c, specular=0.4, specular_power=20, smooth_shading=True, split_sharp_edges=True)
    if title:
        pl.add_text(title, font_size=11, color="black")
    pl.view_vector(view, viewup=up)
    pl.reset_camera()
    pl.camera.zoom(zoom)


def hero(opening=0.55, tipset=None, out="hero.png", views=((-0.9, -1.0, -0.8), (-0.25, 1.0, -0.65)), size=(2400, 1150)):
    meshes = [(poly(s), c) for s, c in assemble(opening, tipset=tipset).values()] + [(m, c) for _, m, c in cables(opening)]
    pl = plotter((1, len(views)), size)
    for i, v in enumerate(views):
        pl.subplot(0, i)
        show(pl, meshes, v, (0, 0, -1), zoom=1.3)
    pl.screenshot(str(IMG / out))


def button_closeup(opening=0.35):
    """Zoomed-in view of the record button on the index sleeve, with the press direction and what it does."""
    import numpy as np
    import finger_link as FL
    from assembly import link_positions
    p = assemble(opening)
    keys = ["index_link", "record_button", "button_cover"]
    items = [(poly(p[k][0]), p[k][1]) for k in keys if k in p]
    _, _, _, _, yi = link_positions(opening)
    RL = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
    to_m = lambda q: RL @ np.array(q, float) + np.array([-48.5, yi, 30.5])
    c = np.array(FL.button_centre(1))
    tip = to_m(c + np.array([1.5, 0.0, 0.0]))                         # the finger pad presses the wall (-x)
    tail = to_m(c + np.array([16.0, 0.0, 0.0]))
    arrow = pv.Arrow(start=tail, direction=tip - tail, scale=float(np.linalg.norm(tip - tail)), tip_length=0.3, shaft_radius=0.04)
    pl = plotter((1, 1), (1400, 1000))
    pl.set_background(BG)
    for (m, col), k in zip(items, keys):
        pl.add_mesh(m, color=col, specular=0.4, smooth_shading=True, split_sharp_edges=True,
                    opacity=0.6 if k == "index_link" else 1.0)
    pl.add_mesh(arrow, color="#d23b2a")
    pl.view_vector((0.9, -0.75, -0.55), viewup=(0, 0, -1))
    pl.reset_camera()
    pl.camera.focal_point = tuple(to_m(c))
    pl.camera.zoom(1.6)
    pl.add_point_labels([to_m(c + np.array([0, 0, 14.0]))], ["record button under a soft TPU cover: press to start, press again to stop"],
                        font_size=22, text_color="black", point_size=1, shape_opacity=0.85, always_visible=True)
    pl.screenshot(str(IMG / "record_button.png"))


def tip_gallery():
    """One render per tip set (closed and open)."""
    from tips import TIP_SETS
    for ts in TIP_SETS:
        slug = ts.folder.lower()
        hero(0.15, ts, f"tips_{slug}.png", views=((-0.9, -1.0, -0.8),), size=(1200, 1000))


def parts():
    rd = HW / "STEP/right"
    ms, ec, eb, el = (import_step(str(rd / f)) for f in
                      ("main_support.step", "end_cover.step", "electronics_box.step", "electronics_lid.step"))
    hood = import_step(str(rd / "d405_hood.step"))
    import d405_mount as DM
    import electronics_box as EB
    from build123d import Box, Pos
    bz = EB.BOX[2]
    boards = [(poly(Pos(*EB.HUB_AT, EB.FLOOR + 5.0) * Box(EB.HUB_BAY[0] - 1, EB.HUB_BAY[1] - 1, 10.0)), PCB),
              (poly(Pos(*EB.IMU_AT, EB.FLOOR - EB.IMU_POCKET[2] + 0.8) * Box(25.0, 22.0, 1.6)), PCB)]
    pico = [(poly(Pos(0, 0, bz - EB.PICO_STANDOFF_H - 0.5) * Box(21.0, 51.0, 1.0)), PCB)]
    up = (0, 0, -1)
    panels = [([(poly(ms, 0.04), BLUE)], "main support", (0.8, -0.6, -0.9), up),
              ([(poly(ms, 0.04), BLUE)], "main support: braced post, fixed D405 tab", (-0.9, -0.5, 0.6), up),
              ([(poly(DM.tab_cam(), 0.04), BLUE), (poly(hood, 0.04), WHITE), (poly(d405_dummy(DM.TAB_FACE), 0.04), DARK)],
               "D405 on the tab, with its hood", (1, -1.3, 0.9), (0, 0, 1)),
              ([(poly(eb, 0.04), WHITE)] + boards, "electronics box: USB 3 hub + IMU", (0.6, -0.9, 1.2), (0, 0, 1)),
              ([(poly(el, 0.04), BLUE)] + pico, "lid with Pico 2 underneath", (0.5, -0.8, -1.0), (0, 0, -1)),
              ([(poly(ec, 0.04), BLUE)], "end cover", (0.6, -1.0, -0.5), up)]
    pl = plotter((2, 3), (2100, 1300))
    for i, (items, title, v, u) in enumerate(panels):
        pl.subplot(i // 3, i % 3)
        show(pl, items, v, u, title, zoom=1.15)
    pl.screenshot(str(IMG / "parts.png"))


def camera_closeup(opening=0.45):
    """The D405 on its tab with the hood: from the side and from behind."""
    import numpy as np
    import d405_mount as DM
    p = assemble(opening)
    keys = ["main_support", "camera_hood", "d405", "thumb_link", "index_link", "tip_thumb", "tip_index"]
    items = [(poly(p[k][0]), p[k][1]) for k in keys if k in p]
    R, t = DM.camera_pose()
    focal = tuple(t + R @ np.array([0.0, -8.0, 8.0]))
    pl = plotter((1, 2), (1900, 950))
    for i, v in enumerate(((0.15, -1.0, -0.35), (0.9, 0.35, -0.9))):
        pl.subplot(0, i)
        show(pl, items, v, (0, 0, -1), zoom=1.0)
        pl.camera.focal_point = focal
        pl.camera.zoom(1.9)
    pl.screenshot(str(IMG / "camera_mount.png"))


def flange_diagram():
    """Hole map of both flanges (right hand), drawn from finger_interfaces.py: one colour per robot."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch, Rectangle
    import finger_interfaces as FI
    cols = {"piper": "#2a9d8f", "hande": "#d23b2a", "yam": "#e07020", "openarm": "#1d4f9c"}
    fig, axes = plt.subplots(1, 2, figsize=(11, 6.2))
    for ax, link in zip(axes, FI.LINKS):
        x0, x1 = FI.flange_x("right", link)
        z0, z1 = FI.flange_z("right", link)
        ax.add_patch(FancyBboxPatch((x0, z0), x1 - x0, z1 - z0, boxstyle="round,pad=0,rounding_size=3",
                                    fc="#f4f5f7", ec="#555", lw=1.5))
        for f in FI.features("right", link):
            c = cols[f[1]]
            if f[0] in ("hole", "dowel"):
                ax.add_patch(Circle((f[2], f[3]), f[4] / 2, fc=c if f[0] == "hole" else "white", ec=c, lw=1.5))
            elif f[0] == "pocket":
                xa, xb = max(f[2], x0), min(f[3], x1)
                ax.add_patch(Rectangle((xa, f[4]), xb - xa, f[5] - f[4], fc=c, alpha=0.25, ec=c, lw=1.2, ls="--"))
        other = "index" if link == "thumb" else "thumb"
        side = 1 if FI.u_sign("right", link) > 0 else -1
        ax.annotate(f"to the {other} jaw", xy=(side * 13.5, 0), ha="center", va="center", rotation=90, fontsize=9,
                    color="#555")
        ax.set_xlim(16, -16)                                               # seen from the tip side (+y)
        ax.set_ylim(-21, 21)
        ax.set_aspect("equal")
        ax.set_title(f"right {link} link (flange face, from the tip side)", fontsize=11)
        ax.set_xlabel("x: closing axis (mm)")
        ax.set_ylabel("z: across (mm)")
        ax.grid(alpha=0.25)
    handles = [plt.Line2D([], [], marker="o", ls="", ms=9, mfc=cols[k], mec=cols[k],
                          label=f"{FI.INTERFACES[k].name}: {len(FI.INTERFACES[k].holes)}x {FI.INTERFACES[k].screw}")
               for k in FI.INTERFACES]
    handles += [plt.Line2D([], [], marker="o", ls="", ms=7, mfc="white", mec=cols["hande"], label="Hand-E Ø2 dowel"),
                Rectangle((0, 0), 1, 1, fc=cols["yam"], alpha=0.25, ec=cols["yam"], ls="--", label="YAM key slot")]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, 0.1, 1, 1))
    fig.savefig(IMG / "flange_interfaces.png", dpi=150)


if __name__ == "__main__":
    hero()
    parts()
    button_closeup()
    tip_gallery()
    camera_closeup()
    flange_diagram()
    print("ok")
