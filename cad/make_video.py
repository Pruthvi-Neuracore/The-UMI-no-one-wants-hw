"""Build the README overview video from both kits' animation segments.

    python make_video.py /path/to/Neuracore_Robots

Renders every segment (cad/<kit>/animate.py), adds a title card, joins them with short crossfades into
docs/media/pg_chopstick.mp4, and makes a lighter animated GIF of it (docs/media/pg_chopstick.gif) for the README
(GitHub plays GIFs inline but not video files from the repository).
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
MEDIA = HERE.parent / "docs" / "media"
W, H, FPS = 1280, 720, 24
BG = (27, 35, 40)
ORDER = [("insta360_x5", "x5_chopstick"), ("insta360_x5", "x5_parallel"), ("insta360_x5", "x5_docking"),
         ("insta360_x5", "x5_hand"), ("d405", "d405_chopstick"), ("d405", "d405_parallel"), ("d405", "d405_hand")]
FADE = 0.5


def font(size, bold=False):
    for name in ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf",):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def card(path, title, lines, seconds):
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    d.text((W / 2, H / 2 - 60), title, fill="white", font=font(64, True), anchor="mm")
    for i, line in enumerate(lines):
        d.text((W / 2, H / 2 + 20 + i * 40), line, fill=(159, 179, 200), font=font(26), anchor="mm")
    png = path.with_suffix(".png")
    im.save(png)
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-loop", "1", "-i", str(png), "-t", str(seconds), "-r", str(FPS),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", str(path)], check=True)


def duration(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(path)],
                         capture_output=True, text=True, check=True).stdout
    return float(json.loads(out)["format"]["duration"])


def main():
    global ORDER
    root = sys.argv[1]
    ORDER = [(k, s) for k, s in ORDER if (HERE / k).exists()]      # only the kits on this branch
    MEDIA.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for kit in ("insta360_x5", "d405"):
            names = [s for k, s in ORDER if k == kit]
            subprocess.run([sys.executable, "animate.py", root, str(tmp), *names], cwd=HERE / kit, check=True)
        clips = [tmp / "title.mp4"]
        kits = {"insta360_x5": "Insta360 X5", "d405": "RealSense D405 + Quest 3S"}
        card(clips[0], "PG Chopstick", ["handheld grippers and wrist cameras for robot data collection",
                                        "  ·  ".join(v for k, v in kits.items() if (HERE / k).exists())], 3.0)
        clips += [tmp / f"{s}.mp4" for _, s in ORDER]
        end = tmp / "end.mp4"
        card(end, "Chopstick · Parallel · Hand", ["one interface for every gripper and every robot",
                                                 "OpenArm  ·  UR  ·  ABB GoFa  ·  FANUC CRX"], 3.0)
        clips.append(end)
        # chain crossfades
        inputs, filt, last, t = [], [], "0:v", 0.0
        for c in clips:
            inputs += ["-i", str(c)]
        for i in range(1, len(clips)):
            t += duration(clips[i - 1]) - FADE
            out = f"v{i}"
            filt.append(f"[{last}][{i}:v]xfade=transition=fade:duration={FADE}:offset={t:.3f}[{out}]")
            last = out
        mp4 = MEDIA / "pg_chopstick.mp4"
        subprocess.run(["ffmpeg", "-v", "error", "-y", *inputs, "-filter_complex", ";".join(filt), "-map", f"[{last}]",
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", "-movflags", "+faststart", str(mp4)],
                       check=True)
        gif = MEDIA / "pg_chopstick.gif"
        pal = tmp / "palette.png"
        vf = "fps=12,scale=720:-1:flags=lanczos"
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(mp4), "-vf", f"{vf},palettegen=max_colors=128",
                        str(pal)], check=True)
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(mp4), "-i", str(pal), "-lavfi",
                        f"{vf}[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4", str(gif)], check=True)
    for f in (mp4, gif):
        print(f"wrote {f.relative_to(HERE.parent)} ({f.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
