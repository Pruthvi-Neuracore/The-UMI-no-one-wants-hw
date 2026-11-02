"""Checks that the handheld works and matches the robot gripper:

  1. gears: every mesh (sync, handle A - left knuckle, handle B - right knuckle) stays engaged over the whole stroke,
     with no tooth interference and only the designed backlash (sections of the real parts at each layer's mid-plane);
  2. jaws: the handheld's 2F-140 replica parts against Robotiq's own meshes on the robot at the same q (surface
     distance), and the pads' boxes;
  3. clearance: assembly.check() (interference between all parts over the stroke, head / Quest vs both builds).

    python verify.py
"""
import math

import numpy as np
import trimesh
from shapely import affinity
from shapely.ops import unary_union

import assembly as A
import kin
import scissor as S


def _section(name, q, x):
    step, col = A.HANDHELD[name]
    m = A._place(name, step, col, A._T(S.motion(name, q)))
    p, _ = m.section([1, 0, 0], [x, 0, 0]).to_2D(to_2D=np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, -x],
                                                                 [0, 0, 0, 1.0]]))
    return unary_union(list(p.polygons_full))


def gears(n=29):
    pairs = [("sync (left - right knuckle gear)", "knuckle_gear_left", "knuckle_gear_right", sum(S.L2) / 2, S.P2R),
             ("handle A - left knuckle gear", "knuckle_gear_left", "handle_a", sum(S.L1) / 2, S.H),
             ("handle B - right knuckle gear", "knuckle_gear_right", "handle_b", sum(S.L3) / 2, S.H)]
    for lab, a, b, x, cb in pairs:
        ov, lo, hi = 0.0, 99.0, 0.0
        for q in np.linspace(0, kin.g()["q_max"], n):
            pa, pb = _section(a, q, x), _section(b, q, x)
            ov = max(ov, pa.intersection(pb).area)
            play = 0.0
            for sgn in (1, -1):                          # turn the driven part until its teeth touch
                for d in np.arange(0.05, 6.0, 0.05):
                    if pa.intersection(affinity.rotate(pb, sgn * d, origin=tuple(cb))).area > 0.02:
                        play += d
                        break
                else:
                    play += 99
            lo, hi = min(lo, play), max(hi, play)
        ok = ov < 0.05 and hi < 5
        print(f"  {lab}: tooth interference {ov:.3f} mm2, backlash {lo:.2f} .. {hi:.2f} deg  "
              f"{'OK' if ok else 'FAIL'}")


def jaws(qs=(0.0, 0.35, 0.7)):
    for q in qs:
        r, h = A.robotiq(q), A.handheld(q, quest=False)
        rows = []
        for s in ("left", "right"):
            for hp, rp in (("outer_link", "outer_finger"), ("inner_knuckle", "inner_knuckle"),
                           ("inner_finger", "inner_finger")):
                pts, _ = trimesh.sample.sample_surface(r[f"robotiq_{rp}_{s}"], 4000, seed=1)
                d = np.abs(trimesh.proximity.signed_distance(h[f"{hp}_{s}"], pts))
                rows.append(f"{hp}_{s} median {np.median(d):.2f} mm")
            pad = trimesh.util.concatenate([h[f"pad_core_{s}"], h[f"pad_skin_{s}"]])
            rows.append(f"pad_{s} box within {np.abs(pad.bounds - r[f'robotiq_pad_{s}'].bounds).max():.2f} mm")
        print(f"  q = {q:.2f}: " + "; ".join(rows))


if __name__ == "__main__":
    print("1. gears over the stroke")
    gears()
    print("2. jaws vs the robot's 2F-140 (Robotiq meshes)")
    jaws()
    print("3. clearance")
    A.check()
