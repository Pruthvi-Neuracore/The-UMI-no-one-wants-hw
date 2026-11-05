"""IKEA-style assembly pages drawn from a CAD assembly (a GLB with one node per part, or a dict of meshes).

Each step shows the parts added in it in blue, pulled out along the way they go in (dashed lines), the parts already
built in light grey, and a box at the top with that step's hardware drawn to scale with its count. A parts page comes
first. Output: step_NN.png pages, one PDF of all of them (Flate-compressed with Ghostscript when it is installed) and
an ASSEMBLY.md that shows them in order.
"""
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pyvista as pv
import trimesh
from PIL import Image, ImageChops, ImageDraw, ImageFont

W, H_PAGE = 1600, 1200
GREY, NEW, METAL = "#dcdfe3", "#9cc3ef", "#7d838b"
INK, ACCENT = (25, 28, 32), (31, 95, 174)
try:
    import matplotlib
    FONT_DIR = Path(matplotlib.__file__).parent / "mpl-data" / "fonts" / "ttf"
except ImportError:                                   # noqa: BLE001
    FONT_DIR = Path(".")


def font(size, bold=False):
    try:
        return ImageFont.truetype(str(FONT_DIR / ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf")), size)
    except OSError:
        return ImageFont.load_default()


def load_glb(path, scale=1000.0, skip=()):
    """{node name: mesh in mm} from a GLB with one node per part."""
    s = trimesh.load(path)
    out = {}
    for node in s.graph.nodes_geometry:
        T, g = s.graph[node]
        if any(g.startswith(k) for k in skip):
            continue
        m = s.geometry[g].copy()
        m.apply_transform(T)
        m.apply_scale(scale)
        out[g] = m
    return out


class HW:
    """Hardware for one step: drawn in the box at the top (to scale) and, if places are given, at its holes.
    kind: screw, pin, insert, bearing, nut, switch, glue, strap, tape, band, other."""

    def __init__(self, kind, label, count, dia=3.0, length=10.0, places=(), pull=35.0, head="button"):
        self.kind, self.label, self.count = kind, label, count
        self.dia, self.length, self.places, self.pull, self.head = dia, length, list(places), pull, head

    def meshes(self):
        out = []
        for seat, d in self.places:
            d = np.asarray(d, float) / np.linalg.norm(d)
            back = np.asarray(seat, float) - d * self.pull
            parts = [trimesh.creation.cylinder(radius=self.dia / 2, segment=[back, back + d * self.length])]
            if self.kind == "screw":
                hr, hh = self.dia * 0.95, self.dia * 0.6
                parts.append(trimesh.creation.cylinder(radius=hr, segment=[back - d * hh, back]))
            out.append((trimesh.util.concatenate(parts), back + d * self.length, np.asarray(seat, float)))
        return out


class Step:
    def __init__(self, title, new, explode=None, hw=(), note="", view=None, show=None, times=None, zoom=1.0,
                 focal=None):
        self.title, self.new, self.explode, self.hw, self.note = title, list(new), dict(explode or {}), list(hw), note
        self.view, self.show, self.times, self.zoom, self.focal = view, show, times, zoom, focal


def _dashes(p0, p1, dash=4.0, gap=3.0):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    if L < 1e-6:
        return None
    d = (p1 - p0) / L
    pts, lines, t = [], [], 0.0
    while t < L:
        lines += [2, len(pts), len(pts) + 1]
        pts += [p0 + d * t, p0 + d * min(t + dash, L)]
        t += dash + gap
    return pv.PolyData(np.array(pts), lines=np.array(lines))


def _add(p, m, color, edge=2.0):
    pm = pv.wrap(m)
    p.add_mesh(pm, color=color, smooth_shading=False, ambient=0.45, diffuse=0.6, specular=0.0)
    try:
        p.add_mesh(pm.extract_feature_edges(38, boundary_edges=False, non_manifold_edges=False, manifold_edges=False),
                   color="black", line_width=1.0)
    except Exception:                                   # noqa: BLE001
        pass
    p.add_silhouette(pm, color="black", line_width=edge)


def _crop(im):
    bg = Image.new("RGB", im.size, (255, 255, 255))
    box = ImageChops.difference(im.convert("RGB"), bg).convert("L").point(lambda q: 255 if q > 8 else 0).getbbox()
    return im.crop(box) if box else im


def render(step, meshes, shown, up, view, size=(1500, 950)):
    p = pv.Plotter(off_screen=True, window_size=size, lighting="light kit")
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    boxes = []
    for n in shown:
        if n in meshes and n not in step.new:
            _add(p, meshes[n], GREY, 1.5)
            boxes.append(meshes[n].bounds)
    for n in step.new:
        if n not in meshes:
            continue
        v = np.asarray(step.explode.get(n, np.zeros(3)), float)
        m = meshes[n].copy().apply_translation(v)
        _add(p, m, NEW, 2.5)
        boxes.append(m.bounds)
        if np.linalg.norm(v) > 1:
            c = meshes[n].bounds.mean(0)
            ln = _dashes(c + v, c)
            if ln is not None:
                p.add_mesh(ln, color="#333333", line_width=2)
    for hw in step.hw:
        for m, tip, seat in hw.meshes():
            _add(p, m, METAL, 1.5)
            ln = _dashes(tip, seat)
            if ln is not None:
                p.add_mesh(ln, color="#333333", line_width=2)
            boxes.append(m.bounds)
    b = np.vstack(boxes)
    focal = np.asarray(step.focal, float) if step.focal is not None else (b.min(0) + b.max(0)) / 2
    v = np.asarray(step.view if step.view is not None else view, float)
    p.camera_position = [tuple(focal + v / np.linalg.norm(v) * 800), tuple(focal), tuple(up)]
    p.reset_camera()
    p.camera.zoom(1.05 * step.zoom)
    im = Image.fromarray(p.screenshot(return_img=True))
    p.close()
    return _crop(im)


def _icon(d, x, y, hw, scale=3.2):
    """Draw one item to scale (side view) at (x, y); returns (w, h)."""
    k = hw.kind
    fill, pale = (200, 203, 208), (225, 228, 232)
    if k == "screw":
        L, r = min(hw.length * scale, 175), hw.dia * scale / 2
        hr, hh = hw.dia * 0.95 * scale, hw.dia * 0.55 * scale
        cx = x + hr
        if hw.head == "button":
            d.pieslice([cx - hr, y, cx + hr, y + 2 * hh], 180, 360, fill=fill, outline=INK, width=2)
        else:
            d.rectangle([cx - hr, y, cx + hr, y + hh], fill=fill, outline=INK, width=2)
        d.rectangle([cx - r, y + hh, cx + r, y + hh + L], fill=pale, outline=INK, width=2)
        for t in np.arange(y + hh + 4, y + hh + L - 2, 4):
            d.line([cx - r, t, cx + r, t + 2], fill=INK, width=1)
        return 2 * hr, hh + L
    if k == "pin":
        L, r = min(hw.length * scale, 175), hw.dia * scale / 2
        d.rounded_rectangle([x, y, x + 2 * r, y + L], radius=r, fill=pale, outline=INK, width=2)
        return 2 * r, L
    if k == "insert":
        w_, h_ = hw.dia * scale * 1.3, hw.length * scale
        d.rounded_rectangle([x, y, x + w_, y + h_], radius=4, fill=(205, 170, 90), outline=INK, width=2)
        for t in np.arange(y + 5, y + h_ - 3, 6):
            d.line([x, t, x + w_, t + 3], fill=INK, width=1)
        return w_, h_
    if k == "bearing":
        R = hw.dia * scale / 2
        d.ellipse([x, y, x + 2 * R, y + 2 * R], fill=pale, outline=INK, width=2)
        d.ellipse([x + R * 0.45, y + R * 0.45, x + R * 1.55, y + R * 1.55], fill="white", outline=INK, width=2)
        return 2 * R, 2 * R
    if k == "nut":
        import math
        R = hw.dia * scale
        pts = [(x + R + R * math.cos(math.radians(a)), y + R + R * math.sin(math.radians(a))) for a in range(0, 360, 60)]
        d.polygon(pts, fill=pale, outline=INK)
        d.ellipse([x + R * 0.5, y + R * 0.5, x + R * 1.5, y + R * 1.5], fill="white", outline=INK, width=2)
        return 2 * R, 2 * R
    if k == "switch":
        d.rectangle([x, y + 10, x + 38, y + 48], fill=pale, outline=INK, width=2)
        d.ellipse([x + 9, y, x + 29, y + 20], fill=(46, 158, 68), outline=INK, width=2)
        return 38, 48
    if k == "glue":
        d.polygon([(x + 16, y), (x + 30, y + 26), (x + 2, y + 26)], fill="white", outline=INK)
        d.ellipse([x + 2, y + 14, x + 30, y + 42], fill="white", outline=INK, width=2)
        return 32, 42
    if k in ("strap", "tape", "band"):
        d.rounded_rectangle([x, y + 10, x + 70, y + 30], radius=10, fill=(44, 90, 160) if k == "strap" else pale,
                            outline=INK, width=2)
        return 70, 40
    d.rounded_rectangle([x, y, x + 46, y + 46], radius=8, fill=pale, outline=INK, width=2)
    return 46, 46


def compose(step, num, im, path, footer):
    page = Image.new("RGB", (W, H_PAGE), "white")
    d = ImageDraw.Draw(page)
    d.text((50, 30), str(num), fill=INK, font=font(110, True))
    d.text((52, 160), step.title, fill=ACCENT, font=font(30, True))
    if step.times:
        d.ellipse([210, 55, 310, 155], outline=INK, width=4)
        d.text((260, 105), f"{step.times}×", fill=INK, font=font(46, True), anchor="mm")
    if step.hw:
        fl = font(19)
        widths = []
        for hw in step.hw:
            iw = {"screw": 2 * hw.dia * 0.95 * 3.2, "pin": hw.dia * 3.2, "insert": hw.dia * 3.2 * 1.3,
                  "bearing": hw.dia * 3.2, "nut": 2 * hw.dia * 3.2}.get(hw.kind, 70)
            widths.append(int(iw) + 12 + int(max(d.textlength(hw.label, font=fl), 70)) + 40)
        x0, y0 = int(max(360, 52 + d.textlength(step.title, font=font(30, True)) + 40)), 30
        d.rounded_rectangle([x0, y0, min(W - 40, x0 + sum(widths) + 30), y0 + 200], radius=14, outline=INK, width=3)
        x = x0 + 25
        for hw, w_ in zip(step.hw, widths):
            iw, _ = _icon(d, x, y0 + 18, hw)
            d.text((x + iw + 12, y0 + 30), f"{hw.count}×", fill=INK, font=font(34, True))
            d.text((x + iw + 12, y0 + 76), hw.label, fill=INK, font=fl)
            x += w_
    area = (40, 250, W - 40, H_PAGE - 120)
    aw, ah = area[2] - area[0], area[3] - area[1]
    k = min(aw / im.width, ah / im.height, 1.6)
    im2 = im.resize((int(im.width * k), int(im.height * k)), Image.LANCZOS)
    page.paste(im2, (area[0] + (aw - im2.width) // 2, area[1] + (ah - im2.height) // 2))
    f, lines, cur = font(24), [], ""
    for wd in step.note.split():
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
    d.text((W - 40, H_PAGE - 30), footer, fill=(150, 150, 150), font=font(18), anchor="rs")
    page.save(path)


def parts_page(meshes, items, path, title, sub, up, view):
    """items: [(mesh name, label, qty)]."""
    page = Image.new("RGB", (W, H_PAGE), "white")
    d = ImageDraw.Draw(page)
    d.text((50, 30), title, fill=INK, font=font(70, True))
    d.text((52, 120), sub, fill=(90, 90, 90), font=font(24))
    cols = 6
    cw = (W - 80) // cols
    rows = (len(items) + cols - 1) // cols
    ch = min(300, (H_PAGE - 200) // max(rows, 1))
    for i, (n, label, q) in enumerate(items):
        p = pv.Plotter(off_screen=True, window_size=(500, 420), lighting="light kit")
        p.set_background("white")
        m = meshes[n].copy()
        m.apply_translation(-m.bounds.mean(0))
        _add(p, m, NEW, 2.0)
        v = np.asarray(view, float)
        p.camera_position = [tuple(v / np.linalg.norm(v) * 600), (0, 0, 0), tuple(up)]
        p.reset_camera()
        img = _crop(Image.fromarray(p.screenshot(return_img=True)))
        p.close()
        img.thumbnail((cw - 20, ch - 80))
        cx, cy = 40 + (i % cols) * cw, 180 + (i // cols) * ch
        page.paste(img, (cx + (cw - img.width) // 2, cy))
        d.text((cx + cw // 2, cy + ch - 62), label, fill=INK, font=font(16), anchor="mm")
        d.text((cx + cw // 2, cy + ch - 34), f"{q}×", fill=ACCENT, font=font(26, True), anchor="mm")
    page.save(path)


def build(meshes, steps, out, footer, up, view, parts=None, title="", intro="", parts_sub=""):
    """Render every step into out/ and write ASSEMBLY.md and assembly.pdf."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for f in out.glob("step_*.png"):
        f.unlink()
    pages = []
    if parts:
        parts_page(meshes, parts, out / "page_00_parts.png", "Parts", parts_sub, up, view)
        pages.append(out / "page_00_parts.png")
    built = []
    for i, s in enumerate(steps, 1):
        shown = s.show if s.show is not None else built
        im = render(s, meshes, list(shown) + s.new, up, view)
        path = out / f"step_{i:02d}.png"
        compose(s, i, im, path, footer)
        pages.append(path)
        built = list(dict.fromkeys(built + s.new))
        print("wrote", path)
    raw = out / "_raw.pdf"
    ims = [Image.open(p).convert("RGB").quantize(256) for p in pages]
    ims[0].save(raw, save_all=True, append_images=ims[1:], resolution=150)
    if shutil.which("gs"):
        subprocess.run(["gs", "-q", "-dBATCH", "-dNOPAUSE", "-sDEVICE=pdfwrite", "-dAutoFilterColorImages=false",
                        "-dColorImageFilter=/FlateEncode", "-dDownsampleColorImages=false",
                        f"-sOutputFile={out / 'assembly.pdf'}", str(raw)], check=True)
        raw.unlink()
    else:
        raw.rename(out / "assembly.pdf")
    md = [f"# {title}", "", intro, "", "All pages as one PDF: [`assembly.pdf`](assembly.pdf).", ""]
    for p_ in pages:
        md += [f"![{p_.stem}]({p_.name})", ""]
    (out / "ASSEMBLY.md").write_text("\n".join(md))
    print("wrote", out / "ASSEMBLY.md")
