"""The handheld's jaws: printed copies of the robot gripper's own jaw parts (Robotiq 2F-140 or 2F-85, kin.GRIPPER).

The text below describes the 2F-140; the 2F-85's parts get the same treatment (see the *_85 functions).

Each jaw uses the robot gripper's parts at the robot's joint positions:

  outer link    = robotiq_arg2f_140_outer_knuckle + robotiq_arg2f_140_outer_finger, one piece (on the gripper they
                  are bolted together and never move relative to each other)
  inner knuckle = robotiq_arg2f_140_inner_knuckle
  inner finger  = robotiq_arg2f_140_inner_finger
  pad           = the URDF's inner_finger_pad (27 x 65 x 7.5 mm), split into a PETG core and a 2 mm TPU face

The geometry is Robotiq's (the visual meshes of their ROS description, in source/robotiq_2f_140/), placed with the URDF's
joint frames. Only features that are hidden on the gripper change:
  * Ø4.2 bores on every joint axis (Ø4 steel dowels),
  * the inner knuckle's base end: the pin between its prongs is removed (the knuckle gear sits there) and two M3 holes
    through the prongs key the gear to the knuckle,
  * the outer knuckle's drive lever, which sits inside the gripper's body behind its pivot, is cut off,
  * two M3 counterbored holes through the inner finger hold the pad core (heat-set inserts in the core).

Each part is built in its own Robotiq link frame and then placed in G for the LEFT jaw at q = 0. The right jaw uses the
same parts turned 180° about z, as on the gripper (rpy z = pi in the URDF).
"""
import math

import numpy as np
import trimesh
from build123d import Box, Cylinder, Location, Pos, Rot, Solid

import kin

SRC = kin.__dict__.get("SRC")
BORE = 4.2
KEY_AT = kin.g()["key_at"]       # M3 key holes along the inner knuckle's arm, through its prongs
PAD_SKIN = 2.0                   # TPU face of the pad
PAD_SCREWS = (20.0, 38.0)        # M3 through the inner finger into the pad core (local y)
M = kin.g()["mesh"]              # "140" / "85": Robotiq's mesh names
F = np.diag([-1.0, -1.0, 1.0, 1.0]) if kin.g()["mesh_mirror"] else np.eye(4)   # the mesh frame (rpy z = pi)


def _src():
    from pathlib import Path
    return Path(__file__).resolve().parent / "source" / f"robotiq_2f_{M}"


def mesh(name):
    m = trimesh.load(_src() / f"robotiq_arg2f_{name}.stl")
    if m.extents.max() < 1:
        m.apply_scale(1000.0)
    return m


def closed(m):
    """A mesh as a closed volume for manifold booleans (Robotiq's meshes can hold several shells or split seams)."""
    if m.is_volume:
        return m
    import manifold3d as mf
    m = m.copy()
    m.merge_vertices()
    mm = mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32))
    mm.merge()
    out = mf.Manifold(mm).to_mesh()
    c = trimesh.Trimesh(out.vert_properties[:, :3], out.tri_verts)
    assert c.is_volume, "mesh could not be closed"
    return c


def mesh_solid(m):
    """A watertight triangle mesh as a build123d Solid (planar faces sewn into a closed shell)."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Sewing
    from OCP.gp import gp_Pnt
    from OCP.ShapeFix import ShapeFix_Solid
    from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
    from OCP.TopoDS import TopoDS
    sew = BRepBuilderAPI_Sewing(1e-3)
    for f in m.faces:
        a, b, c = (m.vertices[i] for i in f)
        poly = BRepBuilderAPI_MakePolygon(gp_Pnt(*map(float, a)), gp_Pnt(*map(float, b)), gp_Pnt(*map(float, c)), True)
        face = BRepBuilderAPI_MakeFace(poly.Wire(), True)
        if face.IsDone():
            sew.Add(face.Face())
    sew.Perform()
    from OCP.TopAbs import TopAbs_SHELL
    from OCP.TopExp import TopExp_Explorer
    solids = []
    ex = TopExp_Explorer(sew.SewedShape(), TopAbs_SHELL)
    while ex.More():                                   # one solid per closed shell (some meshes hold several bodies)
        solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell(ex.Current())).Solid()
        fix = ShapeFix_Solid(solid)
        fix.Perform()
        u = ShapeUpgrade_UnifySameDomain(fix.Solid(), True, True, True)
        u.Build()
        s = Solid(u.Shape())
        if s.volume < 0:
            s = Solid(s.wrapped.Reversed())
        solids.append(s)
        ex.Next()
    out = solids[0]
    for s in solids[1:]:
        out = out + s
    return out


def _gap(y, z, half, r=6.6):
    """The pin body Robotiq's meshes model inside a clevis gap (|x| < half) around a joint: removed (steel dowel)."""
    return Pos(0, y, z) * Rot(0, 90, 0) * Cylinder(r, 2 * half)


def _faces(y, z, inner, r=7.2):
    """Shave a clevis tongue's side faces to |x| <= inner around a joint (running clearance to the prongs)."""
    return (Pos(0, y, z) * Rot(0, 90, 0) * Cylinder(r, 60)) - Pos(0, y, z) * Box(2 * inner, 40, 40)


def _xbore(y, z, r=BORE / 2, length=60.0):
    """A cylinder along local x through (y, z)."""
    return Pos(0, y, z) * Rot(0, 90, 0) * Cylinder(r, length)


def _rx(a):
    return trimesh.transformations.rotation_matrix(a, [1, 0, 0])


def _tr(y, z):
    return trimesh.transformations.translation_matrix([0, y, z])


def _loc4(T):
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*T[0, :4], *T[1, :4], *T[2, :4])
    return Location(t)


def poses():
    """Each part's link frame in G for the left jaw at q = 0 (4 x 4)."""
    g = kin.g()
    j = kin.joints(0.0)
    a0 = kin.link_angle(0.0)
    af = a0 + g["finger_angle"]
    return {
        "outer_link": _tr(*g["outer_pivot"]) @ _rx(a0) @ F,
        "inner_knuckle": _tr(*g["inner_pivot"]) @ _rx(a0) @ F,
        "inner_finger": _tr(*j["B"]) @ _rx(af) @ F,
        "pad": _tr(*j["B"]) @ _rx(af) @ F,
    }


def _local_joints():
    g = kin.g()
    j = kin.joints(0.0)
    a0 = kin.link_angle(0.0)
    af = a0 + g["finger_angle"]
    D = np.diag([-1.0, 1.0]) if g["mesh_mirror"] else np.eye(2)
    R = lambda a: np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]) @ D  # noqa: E731  (mesh -> base)
    o1, o2 = (kin.local(v) for v in g["outer_to_finger"])
    inv = np.linalg.inv
    return dict(B_outer=o1 + o2, OF=o1, E_knuckle=inv(R(a0)) @ (j["E"] - j["P2"]),
                E_finger=inv(R(af)) @ (j["E"] - j["B"]))


def outer_link():
    if M == "85":
        return outer_link_85()
    lj = _local_joints()
    # Done on the meshes (manifold3d) and only then made a solid: OCC's booleans on the sewn mesh solids can fail
    # silently (the pin cut and the fuse came back unchanged; found by the clearance check).
    X = trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])          # cylinder axis z -> x

    def xcyl(y, z, r, h):
        return trimesh.creation.cylinder(radius=r, height=h, sections=64, transform=_tr(y, z) @ X)

    def box(T, ext):
        return trimesh.creation.box(extents=ext, transform=T)
    U = lambda *ms: trimesh.boolean.union([m for m in ms if len(m.faces)], engine="manifold")  # noqa: E731
    I = lambda *ms: trimesh.boolean.intersection(list(ms), engine="manifold")   # noqa: E731
    Df = lambda a, b: trimesh.boolean.difference([a, b], engine="manifold")    # noqa: E731
    okm = closed(mesh("140_outer_knuckle"))
    # the drive lever behind the pivot (inside the gripper body): keep only what is in front of the line through the
    # pivot boss, perpendicular to the pivot -> finger-joint direction
    d = lj["OF"] / np.linalg.norm(lj["OF"])
    ang = math.atan2(d[1], d[0])
    keep = box(_rx(ang) @ _tr(100 - 7.5, 0), (60, 200, 200))                   # local y' >= -7.5 along d
    lever = box(_rx(ang) @ _tr(-100 - 7.5, 0), (60, 200, 200))
    boss = xcyl(0, 0, 6.5, 30)                                                  # keep the pivot boss whole
    okm = U(I(okm, keep), I(okm, lever, boss))
    stubs = Df(xcyl(0, 0, 7.2, 60), box(np.eye(4), (2 * 4.9, 40, 40)))         # the modelled pin stubs past the hub
    okm = Df(okm, stubs)
    ofm = closed(mesh("140_outer_finger")).apply_translation([0, *lj["OF"]])
    ofm = Df(ofm, xcyl(*lj["B_outer"], 6.6, 2 * 7.75))                         # the pin between the finger-joint prongs
    p = mesh_solid(U(okm, ofm))
    # the mesh's pin sits in a slightly larger hole in the prongs: fill the prongs' hole before boring it to Ø4.2
    for sx in (-1, 1):
        p += Pos(sx * (7.75 + 12.2) / 2, *lj["B_outer"]) * Rot(0, 90, 0) * Cylinder(3.2, 12.2 - 7.75)
    for y, z in ((0.0, 0.0), tuple(lj["B_outer"])):
        p -= _xbore(y, z)
    return p


def inner_knuckle():
    if M == "85":
        return inner_knuckle_85()
    lj = _local_joints()
    k = mesh_solid(mesh("140_inner_knuckle"))
    k -= Pos(0, 11.0, 0) * Box(24.4, 34.0, 20.0)          # between the prongs at the base end (|x| < 12.2, y < 28)
    k -= _gap(*lj["E_knuckle"], 7.65)                     # the modelled pin between the prongs at E
    for y, z in ((0.0, 0.0), tuple(lj["E_knuckle"])):
        k -= _xbore(y, z)
    for y in KEY_AT:
        k -= _xbore(y, 0.0, 1.7)
    return k


def inner_finger():
    if M == "85":
        return inner_finger_85()
    lj = _local_joints()
    f = mesh_solid(mesh("140_inner_finger"))
    f -= _faces(0.0, 0.0, 7.2) + _faces(*lj["E_finger"], 7.2)   # 0.3-0.5 mm running clearance to the prongs
    for y, z in ((0.0, 0.0), tuple(lj["E_finger"])):
        f -= _xbore(y, z)
    for y in PAD_SCREWS:                                  # M3 along local z, head on the back (z = +6) face
        f -= Pos(0, y, 0) * Cylinder(1.7, 60)
        f -= Pos(0, y, 6.0) * Cylinder(3.1, 6.0)
    return f


# ---- 2F-85 --------------------------------------------------------------------------------------------------------
# Measured on Robotiq's meshes: the outer finger's prongs straddle the outer knuckle (|x| 5..12) at its foot and the
# inner finger (|x| 7.5..12) at its head; the inner knuckle's base prongs are |x| 12..19.5 (the knuckle gear sits
# between them) and its head prongs |x| 7.5..12; the pins Robotiq models through the heads reach |x| = 13.5.
PAD85_SKIN = 1.5                 # the 2F-85 pad is 5 mm thick: 3.5 mm PETG core + 1.5 mm TPU face
PAD85_SCREWS = (19.0, 37.0)      # M2 self-tapping along local -y, from the finger plate's back (y = -11.9)


# All 2F-85 cuts are done on the meshes (manifold3d) and the result made a solid once: OCC's booleans on solids sewn
# from triangle meshes can fail silently (a cut that changes nothing).
_X = trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])          # cylinder axis z -> x
_Y = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])          # cylinder axis z -> y


def _mcyl(y, z, r, h, x=0.0, axis="x"):
    T = trimesh.transformations.translation_matrix([x, y, z]) @ (_X if axis == "x" else _Y)
    return trimesh.creation.cylinder(radius=r, height=h, sections=64, transform=T)


def _mbox(c, ext):
    return trimesh.creation.box(extents=ext, transform=trimesh.transformations.translation_matrix(c))


def _U(*ms):
    return trimesh.boolean.union([m for m in ms if len(m.faces)], engine="manifold")


def _D(a, *bs):
    return trimesh.boolean.difference([a, *bs], engine="manifold")


def _mfaces(y, z, inner, r):
    """Mesh version of _faces: everything within r of the joint axis past |x| = inner."""
    return _D(_mcyl(y, z, r, 60), _mbox((0, y, z), (2 * inner, 4 * r, 4 * r)))


def _munpin(m, y, z, gap, prong, r_gap=5.5):
    """At a fork head: cut the pin Robotiq models between the prongs (|x| < gap) and its stubs past them, fill the
    prongs' pin holes, ready for the Ø4.2 bore."""
    m = _D(m, _mcyl(y, z, r_gap, 2 * gap), _mfaces(y, z, prong, 3.2))
    return _U(m, *[_mcyl(y, z, 3.2, prong - gap, x=sx * (gap + prong) / 2) for sx in (-1, 1)])


def outer_link_85():
    lj = _local_joints()
    m = _U(closed(mesh("85_outer_knuckle")), closed(mesh("85_outer_finger")).apply_translation([0, *lj["OF"]]))
    m = _D(m, _mfaces(*lj["OF"], 12.0, 3.2))        # the foot pin's stubs past the prongs (the foot is fused)
    m = _munpin(m, *lj["B_outer"], 7.5, 12.0)      # the head pin: the inner finger turns on a dowel here
    m = _D(m, *[_mcyl(y, z, BORE / 2, 60) for y, z in ((0.0, 0.0), tuple(lj["B_outer"]))])
    return mesh_solid(m)


def inner_knuckle_85():
    lj = _local_joints()
    m = _munpin(closed(mesh("85_inner_knuckle")), *lj["E_knuckle"], 7.5, 12.0)
    u = np.array(kin.g()["knuckle_arm"])
    cuts = [_mcyl(y, z, BORE / 2, 60) for y, z in ((0.0, 0.0), tuple(lj["E_knuckle"]))]
    cuts += [_mcyl(*(u * d), 1.7, 60) for d in KEY_AT]
    return mesh_solid(_D(m, *cuts))


def inner_finger_85():
    return mesh_solid(_inner_finger_85_mesh())


def _inner_finger_85_mesh():
    lj = _local_joints()
    m = closed(mesh("85_inner_finger"))
    m = _D(m, _mfaces(0.0, 0.0, 7.2, 7.2), _mfaces(*lj["E_finger"], 7.2, 7.2))   # 0.3 mm running clearance
    cuts = [_mcyl(y, z, BORE / 2, 60) for y, z in ((0.0, 0.0), tuple(lj["E_finger"]))]
    for z in PAD85_SCREWS:                               # M2 clearance + a counterbore on the plate's back
        cuts += [_mcyl(-15.0, z, 1.2, 12.0, axis="y"), _mcyl(-11.9, z, 2.1, 3.0, axis="y")]
    return _D(m, *cuts)


def _pad85():
    (cx, cy, cz), (sx, sy, sz) = kin.g()["pad_local"]
    return cy + sy / 2, cy - sy / 2, cz, sx, sz           # back face (on the finger), front face, centre z, w, l


def pad_core_85():
    back, front, cz, w, l = _pad85()
    c = _mbox((0, (back + front + PAD85_SKIN) / 2, cz), (w, back - front - PAD85_SKIN, l))
    pilots = [_mcyl(back - 1.5, z, 0.8, 3.01, axis="y") for z in PAD85_SCREWS]   # Ø1.6 pilots for the M2 screws
    return mesh_solid(_D(c, closed(mesh("85_inner_finger")), *pilots))           # clear of the finger's foot


def pad_skin_85():
    back, front, cz, w, l = _pad85()
    return Pos(0, front + PAD85_SKIN / 2, cz) * Box(w, PAD85_SKIN, l)


def _pad_box():
    g = kin.g()
    pw, pl, pt = g["pad_size"]
    py, pz = g["pad_offset"]
    return pw, pl, pt, py, pz


def pad_core():
    """PETG back of the pad (the URDF box minus the TPU face), screwed to the inner finger."""
    if M == "85":
        return pad_core_85()
    pw, pl, pt, py, pz = _pad_box()
    top = pz + pt / 2
    top = min(top, mesh(f"{M}_inner_finger").bounds[0, 2])         # flush with the finger's face (0.43 mm under)
    bot = pz - pt / 2 + PAD_SKIN
    c = Pos(0, py, (top + bot) / 2) * Box(pw, pl, top - bot)
    for y in PAD_SCREWS:
        c -= Pos(0, y, top - 3.0) * Cylinder(2.0, 6.01)             # M3 heat-set inserts
    return c


def pad_skin():
    if M == "85":
        return pad_skin_85()
    pw, pl, pt, py, pz = _pad_box()
    z0 = pz - pt / 2
    return Pos(0, py, z0 + PAD_SKIN / 2) * Box(pw, pl, PAD_SKIN)


LOCAL = {"outer_link": outer_link, "inner_knuckle": inner_knuckle, "inner_finger": inner_finger,
         "pad_core": pad_core, "pad_skin": pad_skin}
POSE_OF = {"outer_link": "outer_link", "inner_knuckle": "inner_knuckle", "inner_finger": "inner_finger",
           "pad_core": "pad", "pad_skin": "pad"}


def part(name):
    """The part placed for the left jaw at q = 0, in G."""
    return LOCAL[name]().moved(_loc4(poses()[POSE_OF[name]]))
