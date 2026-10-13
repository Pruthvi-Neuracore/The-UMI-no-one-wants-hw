"""Print-ready ArUco tag sheet for the PG Chopstick grippers (one tag per gripper, stuck on the base).

DICT_4X4_50, 14 mm markers inside 20 mm dotted cut squares, ids 0-49 on one A4 page at true scale, with a 100 mm scale
bar to check the print (if it doesn't measure 100 mm, the print dialog rescaled the page).
Needs OpenCV with the aruco module and Pillow.

    python make_tags.py [--size-mm 14] [--ids 0-49] [--out ../../tags/pg_tags_4x4_50_14mm.pdf]
"""
import argparse
from pathlib import Path

import cv2
from PIL import Image, ImageDraw, ImageFont

DPI = 600
A4_MM = (210, 297)


def mm(v):
    return int(round(v / 25.4 * DPI))


def sheet(ids, size_mm, cut_mm, out):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    page = Image.new("L", (mm(A4_MM[0]), mm(A4_MM[1])), 255)
    draw = ImageDraw.Draw(page)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", mm(2.6))
    except OSError:
        font = ImageFont.load_default()
    margin, pitch = 15.0, cut_mm + 4.0
    cols = int((A4_MM[0] - 2 * margin) // pitch)
    for k, i in enumerate(ids):
        r, c = divmod(k, cols)
        x0, y0 = margin + c * pitch, margin + 12 + r * pitch
        # dotted cut square
        for t in range(0, mm(cut_mm), mm(1.0)):
            for a, b in (((x0, y0), (1, 0)), ((x0, y0 + cut_mm), (1, 0)), ((x0, y0), (0, 1)), ((x0 + cut_mm, y0), (0, 1))):
                px, py = mm(a[0]) + b[0] * t, mm(a[1]) + b[1] * t
                draw.rectangle([px, py, px + 2, py + 2], fill=120)
        img = cv2.aruco.generateImageMarker(d, i, mm(size_mm))
        off = (cut_mm - size_mm) / 2
        page.paste(Image.fromarray(img), (mm(x0 + off), mm(y0 + off)))
        draw.text((mm(x0 + 0.6), mm(y0 + cut_mm - 2.9)), str(i), fill=90, font=font)
    draw.text((mm(margin), mm(5)), f"PG Chopstick gripper tags - DICT_4X4_50, {size_mm:g} mm, cut on the dotted squares",
              fill=0, font=font)
    y = A4_MM[1] - 15
    draw.rectangle([mm(margin), mm(y), mm(margin + 100), mm(y) + mm(1.0)], fill=0)
    for t in range(0, 101, 10):
        draw.rectangle([mm(margin + t), mm(y - 2), mm(margin + t) + 3, mm(y)], fill=0)
    draw.text((mm(margin + 103), mm(y - 2)), "100 mm", fill=0, font=font)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    page.save(out, resolution=DPI)
    print(f"{len(ids)} tags -> {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size-mm", type=float, default=14.0)
    ap.add_argument("--cut-mm", type=float, default=20.0)
    ap.add_argument("--ids", default="0-49")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[2] / "tags/pg_tags_4x4_50_14mm.pdf"))
    a = ap.parse_args()
    lo, hi = (int(v) for v in a.ids.split("-"))
    sheet(list(range(lo, hi + 1)), a.size_mm, a.cut_mm, a.out)


if __name__ == "__main__":
    main()
