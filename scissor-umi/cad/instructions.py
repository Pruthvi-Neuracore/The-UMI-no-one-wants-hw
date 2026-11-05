"""IKEA-style assembly instructions for the Scissor-UMI handheld (and the robot side), drawn from the CAD.

Every step is rendered from the real parts at their assembled poses: the parts added in that step in colour and pulled
out along the way they go in, what is already built in light grey, and each screw, dowel and pin at its real hole with a
dashed line into it. A box at the top lists the hardware for the step (to scale, with the count), so the pages need
almost no words. The parts list and the heat-set-insert page come first.

    python instructions.py            # ../<gripper>/assembly/step_*.png, ASSEMBLY.md and assembly.pdf
"""
import math
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh
from PIL import Image, ImageDraw, ImageFont

import assembly as A
import head
import kin
import scissor as S

OUT = S.HERE.parent / kin.GRIPPER / "assembly"
W, H_PAGE = 1600, 1200
import matplotlib                                     # noqa: E402  (only for its bundled DejaVu fonts)
FONT_DIR = Path(matplotlib.__file__).parent / "mpl-data" / "fonts" / "ttf"

GREY = "#dcdfe3"
NEW = "#9cc3ef"
METAL = "#7d838b"
INK = (25, 28, 32)
ACCENT = (31, 95, 174)


def font(size, bold=False):
    try:
        return ImageFont.truetype(str(FONT_DIR / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")), size)
    except OSError:
        return ImageFont.load_default()


# ---- hardware ------------------------------------------------------------------------------------------------------
def _cyl(p0, p1, r, sections=24):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    return trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=sections)


def screw_mesh(seat, d, length, dia, head="button"):
    """A screw with its head on `seat`, shank along unit d (the way it goes in)."""
    seat, d = np.asarray(seat, float), np.asarray(d, float) / np.linalg.norm(d)
    hr, hh = (dia * 0.95, dia * 0.55) if head == "button" else (dia * 0.85, dia * 0.9)
    parts = [_cyl(seat, seat + d * length, dia / 2), _cyl(seat - d * hh, seat, hr)]
    return trimesh.util.concatenate(parts)


def pin_mesh(p0, p1, dia):
    return _cyl(p0, p1, dia / 2)


def seat_on(mesh, p, d, r_off=2.8):
    """Where a fastener along the axis (p, d) meets `mesh` first coming in along d: cast rays parallel to the axis
    r_off off it (the hole itself is empty) and take the nearest hit."""
    p, d = np.asarray(p, float), np.asarray(d, float) / np.linalg.norm(d)
    a = np.cross(d, [1, 0, 0] if abs(d[0]) < 0.9 else [0, 1, 0])
    a /= np.linalg.norm(a)
    b = np.cross(d, a)
    best = None
    for t in np.linspace(0, 2 * math.pi, 12, endpoint=False):
        o = p - d * 400 + r_off * (math.cos(t) * a + math.sin(t) * b)
        loc, _, _ = mesh.ray.intersects_location([o], [d])
        if len(loc):
            s = min(float(np.dot(l - o, d)) for l in loc) - 400
            best = s if best is None else min(best, s)
    return p + d * (best if best is not None else 0.0)


class HW:
    """One kind of hardware in a step: drawn at each place, and listed (to scale) in the step's box."""

    def __init__(self, kind, label, count, places=(), dia=3.0, length=10.0, pull=35.0, head_="button"):
        self.kind, self.label, self.count, self.places = kind, label, count, list(places)
        self.dia, self.length, self.pull, self.head = dia, length, pull, head_

    def meshes(self):
        out = []
        for seat, d in self.places:
            d = np.asarray(d, float) / np.linalg.norm(d)
            back = np.asarray(seat, float) - d * self.pull
            if self.kind == "screw":
                m = screw_mesh(back, d, self.length, self.dia, self.head)
            else:
                m = pin_mesh(back, back + d * self.length, self.dia)
            out.append((m, back + d * self.length, np.asarray(seat, float)))
        return out


# ---- steps ---------------------------------------------------------------------------------------------------------
X, Y, Z = np.eye(3)
ISO = np.array([170.0, -330.0, 250.0])         # above, left, in front
ISO_R = np.array([170.0, 330.0, 250.0])        # above, right, in front
BELOW = np.array([-330.0, -260.0, 200.0])      # under, left, in front
BACK = np.array([120.0, -260.0, -330.0])       # above, left, behind


def build_steps(hand="right"):
    m0 = A.handheld(0.0, hand)
    T = head.tab_frame()
    n_tab = T[:3, 2]
    P = {k: np.array([0.0, *v]) for k, v in dict(P1L=S.P1L, P1R=S.P1R, P2L=S.P2L, P2R=S.P2R, H=S.H).items()}
    j_l, j_r = kin.joints(0.0, "left"), kin.joints(0.0, "right")
    import replica
    arm = kin.mesh_to_base() @ np.array(kin.g()["knuckle_arm"])
    key_l = [np.array([0.0, *(S.P2L + arm * k)]) for k in kin.g()["key_at"]]
    key_r = [k * [1, -1, 1] for k in key_l]
    Tf = replica.poses()["inner_finger"]
    if kin.g()["mesh"] == "85":                      # M2 self-tapping from the finger plate's back (local -y)
        pad_l = [(Tf @ [0, -11.9 + 1.5, z, 1])[:3] for z in replica.PAD85_SCREWS]
        pad_dir = Tf[:3, :3] @ [0, -1.0, 0]
        pad_hw = ("M2 × 8 self-tapping", 2.0, 8.0)
    else:
        pad_l = [np.array([0.0, -65.2, 151.2]), np.array([0.0, -65.2, 169.2])]
        pad_dir = Y
        pad_hw = ("M3 × 10", 3.0, 10.0)
    pad_r = [k * [1, -1, 1] for k in pad_l]
    opened, _ = S.handle_angles()
    sp = S.H + S.STOP_R * np.array([math.cos(math.radians(opened + 8)), math.sin(math.radians(opened + 8))])

    def seats(host, pts, d, r_off=2.8):
        mesh = trimesh.util.concatenate([m0[h] for h in host]) if isinstance(host, (list, tuple)) else m0[host]
        return [(seat_on(mesh, p, d, r_off), d) for p in pts]

    pads = ["pad_core_left", "pad_skin_left", "pad_core_right", "pad_skin_right"]
    jaw_l = ["outer_link_left", "inner_knuckle_left", "knuckle_gear_left"]
    jaw_r = ["outer_link_right", "inner_knuckle_right", "knuckle_gear_right"]
    fin = ["inner_finger_left", "inner_finger_right"]
    elec = ["servo", "pico", "usb_hub", "imu"]
    loop_w = "a" if hand == "right" else "b"

    # the dowels stay in once fitted (drawn grey in the later steps)
    m0["dowels_base"] = trimesh.util.concatenate(
        [pin_mesh(P[k] + X * S.BOT[0], P[k] + X * (S.BOT[0] + 50), 4) for k in ("P1L", "P1R", "P2L", "P2R")])
    m0["dowels_finger"] = trimesh.util.concatenate(
        [pin_mesh(np.array([-12.5, *j[k]]), np.array([12.5, *j[k]]), 4) for j in (j_l, j_r) for k in ("B", "E")])
    steps = []

    def step(title, new, explode, hw=(), show=None, view=ISO, focal=None, note="", times=None, zoom=1.0):
        steps.append(dict(title=title, new=new, explode=explode, hw=list(hw), show=show, view=view, focal=focal,
                          note=note, times=times, zoom=zoom))

    # 1. pads (sub-assembly, x2)
    step("Pads", ["pad_skin_left", "pad_core_left", "inner_finger_left"],
         {"pad_skin_left": Y * 70, "pad_core_left": Y * 35},
         [HW("glue", "TPU skin: CA glue", 1),
          HW("screw", pad_hw[0], 2, seats("inner_finger_left", pad_l, pad_dir) if kin.g()["mesh"] != "85" else
             [(p_, pad_dir) for p_ in pad_l], pad_hw[1], pad_hw[2])],
         show=[], view=np.array([120.0, -300.0, 200.0]), times=2,
         note="Glue the TPU skin to the core. Screw the core to the inner finger. Make two.")
    # 2. gear into the inner knuckle (left and right are different)
    step("Knuckle gears", ["knuckle_gear_left", "knuckle_gear_right"],
         {"knuckle_gear_left": Z * -45, "knuckle_gear_right": Z * -45},
         [HW("screw", "M3 × 10", 4, seats("inner_knuckle_left", key_l, X) + seats("inner_knuckle_right", key_r, -X),
             3, 10, pull=30)],
         show=["inner_knuckle_left", "inner_knuckle_right"], view=np.array([260.0, -260.0, 220.0]),
         note="Slide each gear into its inner knuckle's fork. Left gear: screws from below. Right gear: from above.")
    # 3. bottom plate: dowels and the record button
    step("Bottom plate", ["base_bottom"], {},
         [HW("pin", "Ø4 × 50 dowel", 4, [(P[k] + X * S.BOT[0], X) for k in ("P1L", "P1R", "P2L", "P2R")],
             4, 50, pull=70),
          HW("switch", "12 × 12 tactile switch", 1)],
         show=[], note="Press the four long dowels into the plate (a vice helps). Push the switch into the pod "
                       "underneath, button down.")
    # 4. handle A on its hub + the stop screw
    step("Handle A", [f"handle_a"], {"handle_a": X * 60},
         [HW("screw", "M3 × 20", 1, [(np.array([S.BOT[0] - 2.0, *sp]), X)], 3, 20, pull=40)],
         show=["base_bottom", "dowels_base"], view=BELOW, focal=np.array([-10.0, 0, 30]),
         note="Drop handle A's hub into the bottom plate's bearing. From below, screw the stop screw through the arc "
              "slot into handle A.")
    # 5. jaws onto the dowels
    step("Jaws", jaw_l + jaw_r, {n: X * 55 for n in jaw_l + jaw_r},
         show=["base_bottom", "handle_a", "dowels_base"],
         note="Fully open: hold handle A against its open stop. Slide both outer links and both knuckle + gear "
              "assemblies onto the dowels so the gears mesh with each other and with handle A.")
    # 6. inner fingers + pads
    step("Fingers", fin + pads, {n: X * 45 for n in fin + pads},
         [HW("pin", "Ø4 × 25 dowel", 4,
             [(np.array([12.5, *j[k]]), -X) for j in (j_l, j_r) for k in ("B", "E")], 4, 25, pull=40)],
         show=["base_bottom", "handle_a", "dowels_base"] + jaw_l + jaw_r,
         note="Fit each finger (with its pad) to its outer link and inner knuckle and pin both joints.")
    # 7. handle B
    step("Handle B", ["handle_b"], {"handle_b": X * 60},
         show=["base_bottom", "handle_a", "dowels_base", "dowels_finger"] + jaw_l + jaw_r + fin + pads,
         note="With the jaws still open, lay handle B on the axle so its teeth mesh with the right gear.")
    # 8. top plate + axle bolt
    top_hw = [HW("screw", "M3 × 10", 5, [(np.array([S.TOP[1], *s]), -X) for s in S.STANDOFFS], 3, 10, pull=85),
              HW("screw", "M4 × 35", 1, [(np.array([S.TOP[1], *S.H]), -X)], 4, 35, pull=95, head_="socket"),
              HW("glue", "threadlocker on the M4", 1)]
    step("Top plate", ["base_top"], {"base_top": X * 50}, top_hw,
         show=["base_bottom", "handle_a", "handle_b", "dowels_base", "dowels_finger"] + jaw_l + jaw_r + fin + pads,
         note="Fit the top plate over the dowels. Screw it to the five standoffs. Axle bolt through the top plate and "
              "handle B into handle A, with threadlocker; snug, so the handles still turn freely.")
    # 9. electronics in the box
    step("Electronics", elec + ["servo_coupler"], {n: X * 60 for n in elec + ["servo_coupler"]},
         [HW("screw", "M2 × 8", 4, [(np.array([-72.0 - 1.0, y, z]), X) for y in (-10.2, 10.2) for z in (33.8, 54.5)],
             2, 8, pull=30),
          HW("screw", "M2 × 6", 4, [(np.array([-27.4, y, z]), -X) for y in (-4.9, 4.9) for z in (20.5, 30.4)],
             2, 6, pull=25),
          HW("screw", "M2 × 5 self-tapping", 4, [], 2, 5)],
         show=["ebox_cover"], view=np.array([300.0, -260.0, 230.0]), focal=np.array([-45.0, 0, 38]),
         note="Servo on the box floor (4 × M2 × 8 from below). Coupler on the servo horn (4 × M2 × 6). Pico 2 on "
              "its standoffs, hub in its bay, IMU on the back wall. Wire them as in electronics.md.")
    # 10. box to the bottom plate
    step("Electronics box", ["ebox_cover"] + elec + ["servo_coupler"],
         {n: X * -60 for n in ["ebox_cover"] + elec + ["servo_coupler"]},
         [HW("screw", "M3 × 50", 4, [(np.array([-72.0, y, z]), X) for y in (-30.5, 30.5) for z in (8.0, 68.0)],
             3, 50, pull=100)],
         show=["base_bottom", "base_top", "handle_a", "handle_b", "dowels_base", "dowels_finger"] + jaw_l + jaw_r +
         fin + pads, view=BELOW,
         focal=np.array([-40.0, 0, 40]),
         note="Jaws open, servo at its open position: push the box up so handle A's hex enters the coupler. Screw "
              "the box to the bottom plate.")
    built = ["base_bottom", "base_top", "handle_a", "handle_b", "ebox_cover", "servo_coupler", "dowels_base",
             "dowels_finger"] + elec + jaw_l + jaw_r + fin + pads
    # 11. the mast
    step("Camera mast", ["tab_mast"], {"tab_mast": X * 60},
         [HW("screw", "M3 × 12", 4, [(np.array([31.5, y, z]), -X) for y, z in head.mast_foot_holes()], 3, 12,
             pull=95)],
         show=built, note="Screw the mast's foot to the top plate.")
    # 12. the D405
    hood_pts = [np.array([80.9, v, -21.3]) for v in (-22.5, 22.5)]
    step("D405", ["d405", "d405_hood"], {"d405": n_tab * 60, "d405_hood": n_tab * 110},
         [HW("screw", "M3 × 12 self-tapping", 2, seats("tab_mast", hood_pts, n_tab), 3, 12, pull=40)],
         show=built + ["tab_mast"], view=np.array([150.0, -300.0, 330.0]), focal=np.array([70.0, 0, 0]),
         note="Fix the D405 to the tab exactly as on the robot's camera plate. Put the hood over it and screw it on "
              "from behind the mast.")
    # 13. Quest arm + controller
    qa_pts = [np.array([109.0, v, -23.4]) for v in (-11, 11)] + [np.array([116.4, v, -22.3]) for v in (-11, 11)]
    step("Quest controller", ["quest_arm", "quest_controller"],
         {"quest_arm": -n_tab * 50, "quest_controller": X * 75},
         [HW("screw", "M3 × 16", 4, seats("tab_mast", qa_pts, -n_tab), 3, 16, pull=40),
          HW("screw", "M3 × 20 + nut", 1, [(np.array([108.0, 14.0, -78.8]), -Y)], 3, 20, pull=30, head_="socket")],
         show=["base_top", "tab_mast", "d405", "d405_hood"], view=BACK, focal=np.array([120.0, 0, -40]),
         note=f"Screw the {hand} Quest arm to the mast's back from the front. Push the {hand} Touch Pro's handle into "
              "the collar, trigger forward, and clamp it.")
    return steps, m0


# ---- rendering -----------------------------------------------------------------------------------------------------
def _dashes(p0, p1, dash=4.0, gap=3.0):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    if L < 1e-6:
        return None
    d = (p1 - p0) / L
    pts, lines, t = [], [], 0.0
    while t < L:
        a, b = p0 + d * t, p0 + d * min(t + dash, L)
        lines += [2, len(pts), len(pts) + 1]
        pts += [a, b]
        t += dash + gap
    return pv.PolyData(np.array(pts), lines=np.array(lines))


def _add(p, m, color, edge=2.0):
    pm = pv.wrap(m)
    p.add_mesh(pm, color=color, smooth_shading=False, ambient=0.45, diffuse=0.6, specular=0.0)
    try:
        p.add_mesh(pm.extract_feature_edges(38, boundary_edges=False, non_manifold_edges=False,
                                            manifold_edges=False), color="black", line_width=1.0)
    except Exception:                                   # noqa: BLE001
        pass
    p.add_silhouette(pm, color="black", line_width=edge)


def render_step(s, meshes, path, size=(1500, 950)):
    p = pv.Plotter(off_screen=True, window_size=size, lighting="light kit")
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    show = list(meshes) if s["show"] is None else s["show"]
    pts = []
    for n in show:
        if n in meshes and n not in s["new"]:
            _add(p, meshes[n], GREY, 1.5)
            pts.append(meshes[n].bounds)
    for n in s["new"]:
        if n not in meshes:
            continue
        v = np.asarray(s["explode"].get(n, np.zeros(3)), float)
        m = meshes[n].copy().apply_translation(v)
        _add(p, m, NEW, 2.5)
        pts.append(m.bounds)
        if np.linalg.norm(v) > 1:
            c = meshes[n].bounds.mean(0)
            ln = _dashes(c + v, c)
            if ln is not None:
                p.add_mesh(ln, color="#333333", line_width=2)
    for hw in s["hw"]:
        for m, tip, seat in hw.meshes():
            _add(p, m, METAL, 1.5)
            ln = _dashes(tip, seat + (seat - tip) / max(np.linalg.norm(seat - tip), 1e-6) * 3)
            if ln is not None:
                p.add_mesh(ln, color="#333333", line_width=2)
            pts.append(m.bounds)
    b = np.vstack(pts)
    lo, hi = b.min(0), b.max(0)
    focal = s["focal"] if s["focal"] is not None else (lo + hi) / 2
    p.camera_position = [tuple(focal + s["view"] * 2.0), tuple(focal), (1, 0, 0)]
    p.reset_camera()
    p.camera.zoom(1.05 * s["zoom"])
    img = p.screenshot(return_img=True)
    p.close()
    im = Image.fromarray(img)
    # crop to content
    bg = Image.new("RGB", im.size, (255, 255, 255))
    from PIL import ImageChops
    box = ImageChops.difference(im.convert("RGB"), bg).convert("L").point(lambda q: 255 if q > 8 else 0).getbbox()
    if box:
        im = im.crop(box)
    im.save(path)
    return im


def _hw_icon(d, x, y, hw, scale=3.2):
    """Draw one hardware item to scale (side view) at (x, y) (top-left); returns its width."""
    if hw.kind == "screw":
        L, r = hw.length * scale, hw.dia * scale / 2
        hr, hh = (hw.dia * 0.95 * scale, hw.dia * 0.55 * scale) if hw.head == "button" else \
            (hw.dia * 0.85 * scale, hw.dia * 0.9 * scale)
        cx = x + hr
        if hw.head == "button":
            d.pieslice([cx - hr, y, cx + hr, y + 2 * hh], 180, 360, fill=(200, 203, 208), outline=INK, width=2)
            top = y + hh
        else:
            d.rectangle([cx - hr, y, cx + hr, y + hh], fill=(200, 203, 208), outline=INK, width=2)
            top = y + hh
        d.rectangle([cx - r, top, cx + r, top + L], fill=(225, 228, 232), outline=INK, width=2)
        for t in np.arange(top + 4, top + L - 2, 4):
            d.line([cx - r, t, cx + r, t + 2], fill=INK, width=1)
        return 2 * hr, hh + L
    if hw.kind == "pin":
        L, r = min(hw.length * scale, 170), hw.dia * scale / 2
        d.rounded_rectangle([x, y, x + 2 * r, y + L], radius=r, fill=(225, 228, 232), outline=INK, width=2)
        return 2 * r, L
    if hw.kind == "switch":
        d.rectangle([x, y + 10, x + 38, y + 48], fill=(225, 228, 232), outline=INK, width=2)
        d.ellipse([x + 9, y, x + 29, y + 20], fill=(46, 158, 68), outline=INK, width=2)
        return 38, 48
    if hw.kind == "glue":
        d.polygon([(x + 16, y), (x + 30, y + 26), (x + 2, y + 26)], fill=(255, 255, 255), outline=INK)
        d.ellipse([x + 2, y + 14, x + 30, y + 42], fill=(255, 255, 255), outline=INK, width=2)
        return 32, 42
    return 30, 30


def compose(s, num, im, path, hand):
    page = Image.new("RGB", (W, H_PAGE), "white")
    d = ImageDraw.Draw(page)
    # step number
    d.text((50, 30), str(num), fill=INK, font=font(110, True))
    d.text((52, 160), s["title"], fill=ACCENT, font=font(30, True))
    if s["times"]:
        d.ellipse([200, 55, 300, 155], outline=INK, width=4)
        d.text((250, 105), f"{s['times']}×", fill=INK, font=font(46, True), anchor="mm")
    # hardware box
    items = s["hw"]
    if items:
        x0, y0 = 360, 30
        fl = font(19)
        icon_w = {"screw": lambda h: 2 * h.dia * 0.95 * 3.2, "pin": lambda h: h.dia * 3.2}
        widths = [int(icon_w.get(hw.kind, lambda h: 40)(hw)) + 12 + int(max(d.textlength(hw.label, font=fl), 70))
                  + 40 for hw in items]
        bw = sum(widths) + 30
        bx1 = min(W - 40, x0 + bw)
        d.rounded_rectangle([x0, y0, bx1, y0 + 200], radius=14, outline=INK, width=3)
        x = x0 + 25
        for hw, w_ in zip(items, widths):
            iw, ih = _hw_icon(d, x, y0 + 18, hw)
            d.text((x + iw + 12, y0 + 30), f"{hw.count}×", fill=INK, font=font(34, True))
            label = hw.label
            d.text((x + iw + 12, y0 + 76), label, fill=INK, font=font(19))
            x += w_
        # if the labels overflow, nothing breaks: the box simply ends at the page edge
    # the drawing
    area = (40, 250, W - 40, H_PAGE - 120)
    aw, ah = area[2] - area[0], area[3] - area[1]
    k = min(aw / im.width, ah / im.height, 1.6)
    im2 = im.resize((int(im.width * k), int(im.height * k)), Image.LANCZOS)
    page.paste(im2, (area[0] + (aw - im2.width) // 2, area[1] + (ah - im2.height) // 2))
    # note
    note = s["note"]
    words, lines, cur = note.split(), [], ""
    f = font(24)
    for wd in words:
        t = (cur + " " + wd).strip()
        if d.textlength(t, font=f) > W - 120:
            lines.append(cur)
            cur = wd
        else:
            cur = t
    if cur:
        lines.append(cur)
    for i, ln in enumerate(lines[:3]):
        d.text((60, H_PAGE - 105 + i * 32), ln, fill=(70, 74, 80), font=f)
    d.text((W - 40, H_PAGE - 30), f"Scissor-UMI · {kin.GRIPPER} · {hand} hand", fill=(150, 150, 150),
           font=font(18), anchor="rs")
    page.save(path)
    return page


def parts_page(meshes, path, hand):
    """Page 0: every printed part (thumbnail, name, count) and the bought hardware."""
    items = [("base_bottom", 1), ("base_top", 1), ("tab_mast", 1), ("d405_hood", 1), ("quest_arm", 1),
             ("ebox_cover", 1), ("servo_coupler", 1), ("handle_a", 1), ("handle_b", 1), ("knuckle_gear_left", 1),
             ("knuckle_gear_right", 1), ("outer_link_left", 2), ("inner_knuckle_left", 2), ("inner_finger_left", 2),
             ("pad_core_left", 2), ("pad_skin_left", 2), ("grip_tpu_a", 1), ("grip_tpu_b", 1)]
    h = S.HANDS[hand]
    step_name = {"handle_a": f"handle_a_{h}", "handle_b": f"handle_b_{h}", "grip_tpu_a": f"grip_tpu_a_{h}",
                 "grip_tpu_b": f"grip_tpu_b_{h}", "quest_arm": f"quest_arm_{hand}"}
    page = Image.new("RGB", (W, H_PAGE), "white")
    d = ImageDraw.Draw(page)
    d.text((50, 30), "Parts", fill=INK, font=font(70, True))
    d.text((52, 120), f"Print these (STEP in ../STEP). {hand.capitalize()}-hand build.", fill=(90, 90, 90),
           font=font(24))
    cols, cw, ch = 6, (W - 80) // 6, 300
    for i, (n, q) in enumerate(items):
        p = pv.Plotter(off_screen=True, window_size=(500, 420), lighting="light kit")
        p.set_background("white")
        m = meshes[n].copy()
        m.apply_translation(-m.bounds.mean(0))
        _add(p, m, NEW if "tpu" not in n and "skin" not in n else "#4a4f57", 2.0)
        p.camera_position = [tuple(ISO * 2), (0, 0, 0), (1, 0, 0)]
        p.reset_camera()
        img = Image.fromarray(p.screenshot(return_img=True))
        p.close()
        img.thumbnail((cw - 20, ch - 80))
        cx, cy = 40 + (i % cols) * cw, 180 + (i // cols) * ch
        page.paste(img, (cx + (cw - img.width) // 2, cy))
        t = kin.GRIPPER.replace("-", "")
        label = step_name.get(n, n.replace("_left", "").replace("outer_link", f"outer_link_{t}")
                              .replace("inner_knuckle", f"inner_knuckle_{t}").replace("inner_finger", f"inner_finger_{t}")
                              .replace("pad_core", f"pad_core_{t}").replace("pad_skin", f"pad_skin_{t}"))
        if n in ("knuckle_gear_left", "knuckle_gear_right"):
            label = n
        d.text((cx + cw // 2, cy + ch - 70), label, fill=INK, font=font(17), anchor="mm")
        d.text((cx + cw // 2, cy + ch - 42), f"{q}×", fill=ACCENT, font=font(28, True), anchor="mm")
    page.save(path)


def inserts_page(path):
    """Page 0b: heat-set inserts, where and how many."""
    rows = [("base_bottom", "9 × M3", "5 standoff tops, 4 underneath for the box"),
            ("base_top", "4 × M3", "the mast's foot"),
            ("handle_a_*", "1 × M4 + 1 × M3", "axle bolt (top of the hub), stop screw (underneath)"),
            ("knuckle_gear_left / _right", "2 × M3 each", "the key screws"),
            *([] if kin.g()["mesh"] == "85" else [("pad_core_2f140", "2 × M3 each", "the pad screws")]),
            ("quest_arm_*", "4 × M3", "the pad on the mast's back")]
    page = Image.new("RGB", (W, H_PAGE), "white")
    d = ImageDraw.Draw(page)
    d.text((50, 30), "0", fill=INK, font=font(110, True))
    d.text((52, 160), "Heat-set inserts", fill=ACCENT, font=font(30, True))
    # an insert and an iron, drawn
    x, y = 420, 50
    d.rounded_rectangle([x, y, x + 46, y + 70], radius=6, fill=(205, 170, 90), outline=INK, width=3)
    for t in range(y + 8, y + 66, 9):
        d.line([x, t, x + 46, t + 5], fill=INK, width=2)
    d.polygon([(x + 120, y - 20), (x + 150, y - 20), (x + 142, y + 50), (x + 128, y + 50)], fill=(160, 160, 160),
              outline=INK)
    d.rectangle([x + 115, y - 140, x + 155, y - 20], fill=(60, 60, 60), outline=INK) if y > 140 else None
    d.text((x + 190, y + 10), "Press each insert in with a soldering iron at ~220 °C,", fill=INK, font=font(24))
    d.text((x + 190, y + 44), "flush with the surface, before assembling.", fill=INK, font=font(24))
    yy = 290
    d.text((80, yy), "Part", fill=ACCENT, font=font(26, True))
    d.text((620, yy), "Inserts", fill=ACCENT, font=font(26, True))
    d.text((900, yy), "Where", fill=ACCENT, font=font(26, True))
    for i, (a, b, c) in enumerate(rows):
        y_ = yy + 60 + i * 64
        d.line([70, y_ - 14, W - 70, y_ - 14], fill=(220, 220, 220), width=2)
        d.text((80, y_), a, fill=INK, font=font(24))
        d.text((620, y_), b, fill=INK, font=font(24, True))
        d.text((900, y_), c, fill=(80, 80, 80), font=font(22))
    d.text((80, yy + 60 + len(rows) * 64 + 30), f"Total: {22 if kin.g()['mesh'] == '85' else 26} × M3 (M3 × 5.7) and 1 × M4. Glue the TPU grip inserts "
           "into their handles (CA glue).", fill=INK, font=font(24))
    page.save(path)


def robot_page(path):
    """The robot side: the camera plate between the coupling and the flange, the D405 on its tab."""
    r = A.robot(0.0)
    p_new = ["ur_camera_plate", "d405"]
    s = dict(title="Robot", new=p_new, explode={"ur_camera_plate": Z * -60, "d405": head.tab_frame()[:3, 2] * 60},
             hw=[HW("screw", "M6, 7 mm longer", 4, [], 6, 20)], show=None, view=ISO, focal=None, zoom=1.0,
             note=f"On the robot: the Robotiq {kin.GRIPPER.upper()} as it is. Put our camera plate between its coupling and the UR "
                  "flange (M6 screws 7 mm longer than the coupling's own) and fix the D405 to the plate's tab.",
             times=None)
    im = render_step(s, r, OUT / "_robot_drawing.png")
    compose(s, len(STEPS) + 1, im, path, "robot")


STEPS = []


def main(hand="right"):
    global STEPS
    OUT.mkdir(parents=True, exist_ok=True)
    STEPS, m0 = build_steps(hand)
    pages = []
    parts_page(m0, OUT / "page_00_parts.png", hand)
    pages.append(OUT / "page_00_parts.png")
    inserts_page(OUT / "page_01_inserts.png")
    pages.append(OUT / "page_01_inserts.png")
    for i, s in enumerate(STEPS, 1):
        im = render_step(s, m0, OUT / f"_drawing_{i:02d}.png")
        path = OUT / f"step_{i:02d}.png"
        compose(s, i, im, path, hand)
        pages.append(path)
        print("wrote", path)
    robot_page(OUT / f"step_{len(STEPS) + 1:02d}_robot.png")
    pages.append(OUT / f"step_{len(STEPS) + 1:02d}_robot.png")
    for f in OUT.glob("_*.png"):
        f.unlink()
    ims = [Image.open(p).convert("RGB").quantize(256) for p in pages]     # palette PDF (no JPEG codec needed)
    raw = OUT / "_raw.pdf"
    ims[0].save(raw, save_all=True, append_images=ims[1:], resolution=150)
    import shutil
    import subprocess
    if shutil.which("gs"):                              # the raw PDF is uncompressed: Flate-compress it (lossless)
        subprocess.run(["gs", "-q", "-dBATCH", "-dNOPAUSE", "-sDEVICE=pdfwrite", "-dAutoFilterColorImages=false",
                        "-dColorImageFilter=/FlateEncode", "-dDownsampleColorImages=false",
                        f"-sOutputFile={OUT / 'assembly.pdf'}", str(raw)], check=True)
        raw.unlink()
    else:
        raw.rename(OUT / "assembly.pdf")
    md = [f"# Scissor-UMI · {kin.GRIPPER}: assembly", "",
          "IKEA-style, drawn from the CAD: new parts in blue, pulled out along the way they go in; parts already "
          "built in grey; screws and dowels at their real holes, with the hardware for each step (to scale) at the "
          "top. All pages as one PDF: [`assembly.pdf`](assembly.pdf).", "",
          f"Drawn for the right-hand build. The left-hand build is the same with `handle_a_lh` / `handle_b_lh`, "
          "`grip_tpu_*_lh` and `quest_arm_left` (palm bar and finger loop swap sides).", "",
          "Tools: soldering iron (heat-set inserts), 2.5 / 2 / 1.5 mm hex keys, small Phillips, CA glue, "
          "threadlocker, a vice or arbor press for the dowels.", ""]
    for p_ in pages:
        md += [f"![{p_.stem}]({p_.name})", ""]
    (OUT / "ASSEMBLY.md").write_text("\n".join(md))
    print("wrote", OUT / "ASSEMBLY.md")


if __name__ == "__main__":
    main()
