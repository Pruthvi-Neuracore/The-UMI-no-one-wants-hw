"""Checks that the handheld works and matches the robot gripper:

  1. gears: every mesh (sync, handle A - left knuckle, handle B - right knuckle) stays engaged over the whole stroke,
     with no tooth interference and only the designed backlash (sections of the real parts at each layer's mid-plane);
  2. jaws: the handheld's 2F-140 replica parts against Robotiq's own meshes on the robot at the same q (surface
     distance), and the pads' boxes;
  3. camera: the handheld's camera tab (tab_mast STEP) against the robot's camera plate tab (ur_camera_plate STEP,
     placed on the gripper), measured from the two STEP files: the tab faces' normals and the six hole centres, and
     so the D405's pose on each;
  4. clearance: assembly.check() (interference between every pair of parts over the stroke).

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


def _tab_measure(shape):
    """The tab face (plane with the tab's normal that carries the M3 pattern) and the hole centres on it, in G."""
    from build123d import GeomType
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    import head
    n0 = head.tab_frame()[:3, 2]
    holes = []
    for f in shape.faces():
        if f.geom_type == GeomType.CYLINDER:
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            if abs(c.Radius() - 1.7) < 0.01:
                d = c.Axis().Direction()
                if abs(abs(np.dot([d.X(), d.Y(), d.Z()], n0)) - 1) < 1e-3:
                    l = c.Location()
                    holes.append((np.array([l.X(), l.Y(), l.Z()]), np.array([d.X(), d.Y(), d.Z()])))
    faces = [f for f in shape.faces() if f.geom_type == GeomType.PLANE
             and np.dot([f.normal_at().X, f.normal_at().Y, f.normal_at().Z], n0) > 1 - 1e-4]
    # the tab face is the forward-facing plane the hole axes pierce at the largest w
    best = max(faces, key=lambda f: np.dot([f.center().X, f.center().Y, f.center().Z], n0))
    p0 = np.array([best.center().X, best.center().Y, best.center().Z])
    nrm = np.array([best.normal_at().X, best.normal_at().Y, best.normal_at().Z])
    pts = []
    for L, D in holes:
        t = np.dot(p0 - L, nrm) / np.dot(D, nrm)
        pts.append(L + t * D)
    return nrm, np.array(pts)


def camera():
    from build123d import import_step
    import head
    rn, rp = _tab_measure(import_step(str(S.STEP_DIR / "_ur_camera_plate.step")))
    hn, hp = _tab_measure(import_step(str(S.STEP_DIR / "tab_mast.step")))
    tol_hole = 0.05
    d = [np.min(np.linalg.norm(hp - p, axis=1)) for p in rp]
    hp_on_tab = [p for p, dd in zip(rp, d) if dd < tol_hole]
    ang = math.degrees(math.acos(min(1.0, abs(float(np.dot(rn, hn))))))
    off = abs(float(np.dot(hp.mean(0) - rp.mean(0), rn))) if len(hp) else 99
    ok = len(rp) == 6 and len(hp_on_tab) == 6 and ang < 0.01 and off < 0.01
    print(f"  robot plate tab: {len(rp)} holes; handheld tab: {len(hp)} holes ({len(hp_on_tab)} on the robot's "
          f"pattern, worst {max(d):.3f} mm); tab faces {ang:.4f} deg apart, {off:.3f} mm apart  {'OK' if ok else 'FAIL'}")
    T = head.camera_pose()
    view = T[:3, 2]
    print(f"  D405 (both builds): optical centre {np.round(T[:3, 3], 1)} mm in G, view {np.round(view, 3)} "
          f"({math.degrees(math.atan2(-view[0], view[2])):.1f} deg in towards the axis)")
    return ok


if __name__ == "__main__":
    print("1. gears over the stroke")
    gears()
    print("2. jaws vs the robot's 2F-140 (Robotiq meshes)")
    jaws()
    print("3. camera on the handheld vs on the robot")
    camera()
    print("4. clearance")
    A.check()
