"""Fixed D405 mount: the camera screws straight onto a tab at the end of the main support's camera post, as on the
Scissor-UMI (and as a D405 sits on a robot's camera plate). Nothing turns: the view is set by TILT_DEG here.

  tab (part of main_support, restyle.camera_post): a TAB_T plate under the camera, carried by the braced post. The
      D405's back sits flat on it; 2x M3 x SCREW_L through counterbored holes into the D405's rear threads
      (2x M3, 20 mm apart, 4 mm deep). Two tapped holes take the hood.
  d405_hood: an open frame round the camera (no floor, so the camera sits on the tab itself), screwed to the tab
      with 2x M3 through two ears at its top corners (behind the camera is where the post comes in). It guards the
      D405's sides; its top wall has the cable window.

Frames. Main-support frame M: the post's anchor at (x, z) = ANCHOR_M, y = Y_CENTRE. Camera frame (cam): D405 centred
on (0, 0), its back on the tab face z = TAB_FACE, looking +z; the post meets the tab at (y, z) = ANCHOR_CAM, on the
camera's -y side; the cable leaves at +y. The camera sits where HandUMI's hinge held it at 65 degrees.

    python d405_mount.py      # -> ../hardware/{STEP,STL}/{right,left}/d405_hood.*
"""
from pathlib import Path

import numpy as np
from build123d import Axis, Box, Cylinder, Pos, Rot, export_step, export_stl, fillet

OUT = Path(__file__).resolve().parents[1] / "hardware"

# ---- pose (unchanged from HandUMI's hinge at 65 degrees)
ANCHOR_M = (-29.7, -45.0)      # where the post ends, in M (x, z)
ANCHOR_CAM = (-29.5, 1.5)      # the same point in the camera frame (y, z)
Y_CENTRE = 72.5                # the post's centre along the bar (M y)
TILT_DEG = 65.0                # view, degrees below horizontal (forward-down at the fingers)

# ---- the D405 and its screws
CAM = (42.0, 42.0, 23.0)       # D405 body x, y, depth
CAM_HOLES = ((-10.0, 0.0), (10.0, 0.0))   # rear M3 threads, 20 mm apart, 4 mm deep
CAM_THREAD = 4.0
M3_CLEAR, M3_HEAD, M3_TAP = 3.4, 6.2, 2.6

# ---- tab (on the main support)
TAB_FACE = 2.0                 # camera back (tab face) on cam z
TAB_T = 4.0                    # tab thickness
TAB_CBORE = 1.5                # counterbore depth for the camera screws (from the tab's back)
CLR, WALL = 0.3, 2.4           # hood: clearance round the camera, wall thickness
HALF = CAM[0] / 2 + CLR + WALL  # tab and hood half-width (x) and half-height on +y
TAB_Y0 = ANCHOR_CAM[0] - 3.0   # tab runs from here (over the post) to +HALF
TAB_R = 4.0                    # tab corner radius
EAR = 7.0                      # hood ears (and the tab under them) reach this far past the hood's top wall
HOOD_SCREWS = ((-18.5, HALF + EAR / 2), (18.5, HALF + EAR / 2))   # hood to tab, through the ears (cam x, y)
TAB_Y1 = HALF + EAR            # tab's top edge
RIB_W, RIB_H = 10.0, 7.0       # spine under the tab, from the post to the camera's centre line
SCREW_L = TAB_T - TAB_CBORE + CAM_THREAD - 0.5  # longest camera screw that stays in the D405's threads

# ---- hood
HOOD_H = CAM[2] - 4.0          # walls stop 4 mm short of the camera's face, so the glass stands proud
HOOD_FLANGE_T = 3.0            # the hood's ears on the tab
WINDOW = dict(x=(-13.0, 13.0), z=(0.6, 14.0), r=3.0)   # cable window in the +y wall (z above the tab face)


def camera_pose():
    """Rotation and translation taking camera-frame points into the main-support frame M."""
    a = np.radians(TILT_DEG)
    base = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], float)   # cam x -> +Y, cam y -> -Z, cam z (view) -> -X
    tilt = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    R = tilt @ base
    t = np.array([ANCHOR_M[0], Y_CENTRE, ANCHOR_M[1]]) - R @ np.array([0.0, *ANCHOR_CAM])
    return R, t


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def tab_cam():
    """The tab and its spine, in the camera frame (main_support adds it in M with camera_pose())."""
    z0, z1 = TAB_FACE - TAB_T, TAB_FACE
    tab = _box(-HALF, HALF, TAB_Y0, TAB_Y1, z0, z1)
    tab = fillet(tab.edges().filter_by(Axis.Z), TAB_R)
    ay, az = ANCHOR_CAM
    spine = _box(-RIB_W / 2, RIB_W / 2, TAB_Y0, 0.0, z0 - RIB_H, z0 + 0.5)   # under the tab, post to camera centre
    body = (tab + spine).clean()
    try:
        body = fillet(body.edges().filter_by(Axis.X).filter_by_position(Axis.Z, z0 - RIB_H - 0.1, z0 - RIB_H + 0.1), 2.0)
    except Exception:
        pass
    for x, y in CAM_HOLES:                                                    # camera screws, heads from behind
        body -= Pos(x, y, (z0 + z1) / 2) * Cylinder(M3_CLEAR / 2, TAB_T + 2)
        body -= Pos(x, y, z0 + TAB_CBORE / 2 - 0.01) * Cylinder(M3_HEAD / 2, TAB_CBORE)
    for x, y in HOOD_SCREWS:                                                  # hood screws: thread cut in the tab
        body -= Pos(x, y, (z0 + z1) / 2) * Cylinder(M3_TAP / 2, TAB_T + 2)
    return body


def to_m(shape):
    """Move a camera-frame shape into the main-support frame M."""
    from build123d import Location
    from OCP.gp import gp_Trsf
    R, t = camera_pose()
    T = gp_Trsf()
    T.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
    return shape.moved(Location(T))


def tab_m():
    return to_m(tab_cam())


def hood():
    """Open frame round the D405 (camera frame): walls to HOOD_H above the tab face, two ears at the top corners with
    the screw holes, a cable window in the +y wall."""
    hi = CAM[0] / 2 + CLR
    z0 = TAB_FACE
    walls = _box(-HALF, HALF, -HALF, HALF, z0, z0 + HOOD_H) - _box(-hi, hi, -hi, hi, z0 - 1, z0 + HOOD_H + 1)
    ears = sum((_box(x - 4.5, x + 4.5, hi, TAB_Y1, z0, z0 + HOOD_FLANGE_T) for x, _ in HOOD_SCREWS[1:]),
               _box(HOOD_SCREWS[0][0] - 4.5, HOOD_SCREWS[0][0] + 4.5, hi, TAB_Y1, z0, z0 + HOOD_FLANGE_T))
    h = walls + ears
    corners = [e for e in h.edges().filter_by(Axis.Z) if abs(e.center().X) > HALF - 0.1]   # the outer corners
    try:
        h = fillet(corners, 2.0)
    except ValueError:
        pass
    (wx0, wx1), (wz0, wz1) = WINDOW["x"], WINDOW["z"]
    win = _box(wx0, wx1, hi - 1, HALF + 1, z0 + wz0, z0 + wz1)
    win = fillet(win.edges().filter_by(Axis.Y), WINDOW["r"])
    h -= win
    for x, y in HOOD_SCREWS:
        h -= Pos(x, y, z0 + HOOD_FLANGE_T / 2) * Cylinder(M3_CLEAR / 2, HOOD_FLANGE_T + 2)
        h -= Pos(x, y, z0 + HOOD_FLANGE_T - 0.8) * Cylinder(M3_HEAD / 2, 1.62)       # heads sit 0.8 mm in
    return h


def main():
    h = hood()
    print(f"d405_hood valid={h.is_valid} solids={len(h.solids())} vol={h.volume:.0f}  camera screws: 2x M3 x {SCREW_L:.0f}")
    for side in ("right", "left"):                                            # same part for both hands
        export_step(h, str(OUT / "STEP" / side / "d405_hood.step"))
        export_stl(h, str(OUT / "STL" / side / "d405_hood.stl"), tolerance=0.02, angular_tolerance=0.15)


if __name__ == "__main__":
    main()
