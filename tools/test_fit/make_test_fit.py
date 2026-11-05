"""Printable stand-ins for HandUMI's bought motion parts, to test fits before the real ones arrive.

    python make_test_fit.py        # -> handumi/hardware/STEP/test_fit/*.step

  lm4uu_standin_od7p9 / od8p0 / od8p1   LM4UU (8 x 4 x 12): plain bushing, three outer diameters for the press fit,
                                        4.35 mm bore so it slides on a Ø4 rod
  mr63zz_standin_od5p9 / od6p0 / od6p1  MR63ZZ (6 x 3 x 2.5): plain ring, 3.2 mm bore for the M3 pivot screw
  rod_gauge_d4p1_l135                   Ø4 x 135 rod stand-in, 4.1 mm with a 0.4 mm flat to print lying on (sand to fit);
                                        a gauge for the holes only (M4 threaded rod is the better stand-in)
  pin_gauges_d3p9_to_4p2                four 30 mm pins, Ø3.9 / 4.0 / 4.1 / 4.2, on a tag: which one fits each hole
"""
from pathlib import Path

from build123d import Box, Cylinder, Pos, Rot, Text, chamfer, export_step, extrude, Axis

OUT = Path(__file__).resolve().parents[2] / "handumi" / "hardware" / "STEP" / "test_fit"


def bushing(od, idia, length, ch=0.3):
    b = Cylinder(od / 2, length) - Cylinder(idia / 2, length + 1)
    try:
        b = chamfer(b.edges().filter_by(lambda e: True), ch)
    except Exception:                                   # noqa: BLE001
        pass
    return b


def rod(d, length, flat=0.4):
    r = Rot(0, 90, 0) * Cylinder(d / 2, length)          # along x, lying down
    return r - Pos(0, 0, -d / 2 - 5 + flat) * Box(length + 2, d + 2, 10)


def pin_gauges():
    out = Pos(0, 0, -0.75) * Box(52, 14, 1.5)            # the tag the pins stand on
    for i, d in enumerate((3.9, 4.0, 4.1, 4.2)):
        x = -19.5 + 13 * i
        out += Pos(x, 0, 15) * Cylinder(d / 2, 30)
    return out


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    parts = {}
    for od in (7.9, 8.0, 8.1):
        parts[f"lm4uu_standin_od{str(od).replace('.', 'p')}"] = bushing(od, 4.35, 12.0)
    for od in (5.9, 6.0, 6.1):
        parts[f"mr63zz_standin_od{str(od).replace('.', 'p')}"] = bushing(od, 3.2, 2.5, ch=0.2)
    parts["rod_gauge_d4p1_l135"] = rod(4.1, 135.0)
    parts["pin_gauges_d3p9_to_4p2"] = pin_gauges()
    for n, s in parts.items():
        export_step(s, str(OUT / f"{n}.step"))
        bb = s.bounding_box()
        print(f"{n:28s} {bb.size.X:6.1f} x {bb.size.Y:5.1f} x {bb.size.Z:5.1f} mm  valid {s.is_valid}")


if __name__ == "__main__":
    main()
