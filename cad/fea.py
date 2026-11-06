"""Linear-elastic FEA of the printed parts (PLA), one or more load cases per part.

Mesh: gmsh from the part STEP (linear tets); field: quadratic (P2) displacement; solver: AMG-preconditioned CG.
Units: mm, N, MPa. Each case fixes a set of surfaces and applies a total force on another set.
Results: peak von Mises (99.5th percentile over elements, which ignores single-element spikes at load/support
edges) and the safety factor against printed PLA. Stress maps go to docs/img/fea/.

    python fea.py [part ...]
"""
import sys
from dataclasses import dataclass, field
from pathlib import Path

import gmsh
import numpy as np
import pyamg
from skfem import (Basis, ElementTetP2, ElementVector, FacetBasis, LinearForm, MeshTet, asm, condense)
from skfem.helpers import dot
from skfem.models.elasticity import lame_parameters, linear_elasticity

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "hardware/STEP/right"
SRC = ROOT / "cad/source"
OUT = ROOT / "docs/img/fea"

E, NU = 3000.0, 0.35            # printed PLA
UTS = 30.0                      # MPa, printed PLA along layers (conservative)
UTS_Z = 15.0                    # MPa, across layers (worst case orientation)


# ---------------------------------------------------------------- selectors (on facet centroids, part frame)
def cyl(axis, c, r, lo=-1e9, hi=1e9, tol=0.45):
    ia = "xyz".index(axis)
    io = [i for i in range(3) if i != ia]

    def f(p):
        d = np.hypot(p[io[0]] - c[0], p[io[1]] - c[1])
        return (np.abs(d - r) < tol) & (p[ia] > lo - 1e-6) & (p[ia] < hi + 1e-6)
    return f


def plane(axis, v, tol=0.15, **box):
    ia = "xyz".index(axis)

    def f(p):
        m = np.abs(p[ia] - v) < tol
        for k, (a, b) in box.items():
            i = "xyz".index(k)
            m &= (p[i] > a) & (p[i] < b)
        return m
    return f


def in_cam(f):
    """Apply a selector written in the D405 body frame (camera.camera_pose) to main-support points."""
    import camera as CAM
    R, t = CAM.camera_pose()
    return lambda p: f(R.T @ (p - t[:, None]))


def any_of(*fs):
    return lambda p: np.any([f(p) for f in fs], axis=0)


@dataclass
class Case:
    name: str
    fixed: object
    load: object
    force: tuple                     # total force vector (N)
    moment: tuple = None             # optional (arm_axis, traction_axis, M [N·mm], centre): adds t[traction] = k·(x[arm] - c)
                                     # with ∫ (x[arm] - c) · t[traction] dA = M (a couple on the loaded face)


@dataclass
class Part:
    name: str
    step: Path
    size: float
    cases: list = field(default_factory=list)


# ---------------------------------------------------------------- mesh + solve
def mesh_step(path, size):
    """Tet mesh of a STEP part (or of a build123d shape returned by a callable: the parts are meshed without their
    0.2 mm engraved logo, whose letter faces gmsh can't mesh and which carry no load). If the surface mesh
    self-intersects (small fillet slivers), retry with another surface algorithm (MeshAdapt), then a finer size."""
    if callable(path):
        import tempfile
        from build123d import export_brep
        tmp = Path(tempfile.mkdtemp()) / "part.brep"
        export_brep(path(), str(tmp))
        path = tmp
    tries = [(size, 6, 12), (size, 1, 12), (size * 0.83, 6, 20)]      # (size, 2D algorithm, curvature points)
    for k, (sz, algo, curv) in enumerate(tries):
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(str(path))
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMax", sz)
        gmsh.option.setNumber("Mesh.MeshSizeMin", sz * 0.35)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", curv)
        gmsh.option.setNumber("Mesh.Algorithm", algo)
        try:
            gmsh.model.mesh.generate(3)
            if len(gmsh.model.mesh.getElementsByType(4)[0]) == 0:
                raise RuntimeError("no tetrahedra")
            break
        except Exception as ex:
            gmsh.finalize()
            if k == len(tries) - 1:
                raise
            print(f"  mesh retry ({str(ex).splitlines()[0]}): size {tries[k + 1][0]:.2f}, 2D algorithm {tries[k + 1][1]}")
    _, coords, _ = gmsh.model.mesh.getNodes()
    tags, conn = gmsh.model.mesh.getElementsByType(4)
    ntags, _, _ = gmsh.model.mesh.getNodes()
    gmsh.finalize()
    p = coords.reshape(-1, 3)
    idx = {t: i for i, t in enumerate(ntags)}
    t = np.vectorize(idx.get)(conn.reshape(-1, 4))
    used = np.unique(t)
    remap = -np.ones(len(p), int)
    remap[used] = np.arange(len(used))
    return MeshTet(p[used].T, remap[t].T)


def solve(mesh, case):
    e = ElementVector(ElementTetP2())
    ib = Basis(mesh, e)
    lam, mu = lame_parameters(E, NU)
    K = asm(linear_elasticity(lam, mu), ib)

    bf = mesh.boundary_facets()
    mid = mesh.p[:, mesh.facets[:, bf]].mean(axis=1)
    fixed = bf[case.fixed(mid)]
    loaded = bf[case.load(mid)]
    if len(fixed) == 0 or len(loaded) == 0:
        raise ValueError(f"{case.name}: empty selection (fixed {len(fixed)}, loaded {len(loaded)})")

    fb = FacetBasis(mesh, e, facets=loaded)
    area = asm(LinearForm(lambda v, w: v), FacetBasis(mesh, ElementTetP2(), facets=loaded)).sum()
    trac = np.array(case.force, float) / area

    if case.moment:
        arm, tax, M, c = case.moment                             # couple as a linear traction across the face
        ia, it = "xyz".index(arm), "xyz".index(tax)
        q = FacetBasis(mesh, ElementTetP2(), facets=loaded)
        J = asm(LinearForm(lambda v, w: (w.x[ia] - c) ** 2 * v), q).sum()
        k = M / J

        @LinearForm
        def load(v, w):
            t = [trac[0] + 0 * w.x[0], trac[1] + 0 * w.x[0], trac[2] + 0 * w.x[0]]
            t[it] = t[it] + k * (w.x[ia] - c)
            return t[0] * v[0] + t[1] * v[1] + t[2] * v[2]
    else:
        @LinearForm
        def load(v, w):
            return trac[0] * v[0] + trac[1] * v[1] + trac[2] * v[2]

    f = asm(load, fb)
    D = ib.get_dofs(facets=fixed).all()
    A, b, x0, I = condense(K, f, D=D)
    ml = pyamg.smoothed_aggregation_solver(A.tocsr(), B=None)
    xi = ml.solve(b, tol=1e-8, accel="cg", maxiter=400)
    u = np.zeros(K.shape[0])
    u[I] = xi

    # von Mises per element (at quadrature points, max per element)
    du = ib.interpolate(u).grad
    eps = 0.5 * (du + np.transpose(du, (1, 0, 2, 3)))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    s = 2 * mu * eps
    for i in range(3):
        s[i, i] += lam * tr
    vm = np.sqrt(0.5 * ((s[0, 0] - s[1, 1]) ** 2 + (s[1, 1] - s[2, 2]) ** 2 + (s[2, 2] - s[0, 0]) ** 2)
                 + 3 * (s[0, 1] ** 2 + s[1, 2] ** 2 + s[0, 2] ** 2))
    vm_el = vm.max(axis=1)
    disp = np.linalg.norm(u[ib.nodal_dofs].T, axis=1)
    return vm_el, disp, len(fixed), len(loaded)


def render(mesh, vm_el, title, out):
    import pyvista as pv
    pv.OFF_SCREEN = True
    cells = np.hstack([np.full((mesh.t.shape[1], 1), 4), mesh.t.T]).ravel()
    grid = pv.UnstructuredGrid(cells, np.full(mesh.t.shape[1], 10, np.uint8), mesh.p.T)
    grid.cell_data["von Mises (MPa)"] = vm_el
    pl = pv.Plotter(off_screen=True, window_size=(1000, 750))
    pl.set_background("white")
    pl.add_mesh(grid.extract_surface(), scalars="von Mises (MPa)", cmap="turbo", clim=(0, np.percentile(vm_el, 99.5)),
                show_edges=False, scalar_bar_args={"color": "black", "title": "von Mises (MPa)"})
    pl.add_text(title, font_size=11, color="black")
    pl.view_isometric()
    pl.screenshot(str(out))


# ---------------------------------------------------------------- parts and load cases
G = 9.81
ARM_HOLES = any_of(cyl("z", (69.0, 64.5), 1.6, -1, 9), cyl("z", (69.0, 80.5), 1.6, -1, 9))
CAM_SCREWS = in_cam(any_of(cyl("z", (-10.0, 0.0), 1.7, -9.0, 0.5), cyl("z", (10.0, 0.0), 1.7, -9.0, 0.5)))   # D405 on the tab
CAM_SHOCK = 0.07 * 15 * G      # D405 + hood at 15 g


def parts():
    return [
        Part("main_support", lambda: __import__("restyle").main_support(logo=False), 1.8, [
            Case("camera knock, vertical (15 g)", ARM_HOLES, CAM_SCREWS, (0, 0, -CAM_SHOCK)),
            Case("camera knock, sideways (15 g)", ARM_HOLES, CAM_SCREWS, (0, CAM_SHOCK, 0)),
            Case("camera knock, fore-aft (15 g)", ARM_HOLES, CAM_SCREWS, (CAM_SHOCK, 0, 0)),
            Case("tip push on rods, 2 x 30 N at the end wall", ARM_HOLES,
                 any_of(cyl("y", (-28.0, 29.0), 2.05, 4.5, 10.5), cyl("y", (-8.0, 29.0), 2.05, 4.5, 10.5)), (0, 0, 60.0)),
        ]),
        Part("finger_link (right thumb)", lambda: __import__("finger_link").finger_link(SRC / "right_thumb_link.step"), 1.3, [
            Case("30 N grip on the tip, 70 mm lever", any_of(cyl("x", (-40.5, 1.5), 4.1), cyl("x", (-20.5, 1.5), 4.1)),
                 plane("y", 0.0, x=(-12.5, 10.5), z=(-18.5, 18.5)), (30.0, 0, 0), moment=("x", "y", -30.0 * 70.0, 0.0)),
            Case("finger presses the paddle, 40 N", any_of(cyl("x", (-40.5, 1.5), 4.1), cyl("x", (-20.5, 1.5), 4.1)),
                 plane("x", 6.0, tol=0.3, y=(-49, -14), z=(-42, -14)), (-40.0, 0, 0)),
        ]),
        Part("end_cover", R / "end_cover.step", 1.3, [
            Case("tip push on rods, 2 x 30 N", any_of(cyl("y", (-28.0, 4.0), 1.6), cyl("y", (-8.0, 4.0), 1.6)),
                 any_of(cyl("y", (-28.0, 29.0), 2.05), cyl("y", (-8.0, 29.0), 2.05)), (0, 0, 60.0)),
        ]),
        Part("electronics_box", R / "electronics_box.step", 1.6, [
            Case("contents at 15 g + 20 N press on the walls", any_of(cyl("z", (-10.0, -11.0), 2.0, 0, 8), cyl("z", (10.0, -11.0), 2.0, 0, 8)),
                 plane("z", 30.0, tol=0.2), (0, 0, -20.0)),
        ]),
        Part("electronics_lid", R / "electronics_lid.step", 1.2, [
            Case("20 N press in the middle", any_of(*[cyl("z", (sx * 17.0, sy * 41.0), 1.7) for sx in (-1, 1) for sy in (-1, 1)]),
                 plane("z", 33.0, tol=0.2, x=(-8, 8), y=(-8, 8)), (0, 0, -20.0)),
        ]),
        Part("crank_mechanism_plate", R / "crank_mechanism_plate.step", 1.0, [
            Case("30 N on each crank pin", any_of(*[cyl("y", (sx * 4.9, sz * 4.9), 1.5) for sx in (-1, 1) for sz in (-1, 1)]),
                 cyl("y", (21.5, 0.0), 1.5), (0, 0, 30.0)),
        ]),
        Part("connecting_link", R / "connecting_link_1.step", 0.8, [
            Case("40 N pull between pins", cyl("y", (-18.0, 0.0), 3.01), cyl("y", (18.0, 0.0), 3.01), (40.0, 0, 0)),
        ]),
        Part("hand_support_base", R / "hand_support_base.step", 1.5, [
            Case("hand pushes 60 N on the saddle", any_of(cyl("z", (38.0, 10.0), 1.75), cyl("z", (38.0, 26.0), 1.75)),
                 plane("z", 18.0, tol=0.2), (0, 0, -60.0)),
        ]),
        Part("controller_support", R / "right_controller_support.step", 1.5, [
            Case("VR controller at 15 g (150 g)", any_of(cyl("z", (14.9, 16.0), 1.6), cyl("z", (30.9, 16.0), 1.6)),
                 cyl("z", (22.6, 47.8), 18.0, -18, 4, tol=0.6), (0.15 * 15 * G, 0, 0)),
        ]),
    ]


def main(only=(), case_filter=None):
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for part in parts():
        if only and not any(o in part.name for o in only):
            continue
        mesh = mesh_step(part.step, part.size)
        for i, case in enumerate(part.cases):
            if case_filter and case_filter not in case.name:
                continue
            try:
                vm, disp, nf, nl = solve(mesh, case)
            except Exception as ex:
                rows.append((part.name, case.name, None, None, None, str(ex)[:60]))
                print(part.name, case.name, "FAILED", ex)
                continue
            p995 = float(np.percentile(vm, 99.5))
            sf, sf_z = UTS / p995, UTS_Z / p995
            rows.append((part.name, case.name, p995, float(disp.max()), sf, sf_z))
            with open(ROOT / "docs/fea_results.csv", "a") as fh:                 # saved per case
                fh.write(f"{part.name},{case.name},{p995:.2f},{disp.max():.3f},{sf:.1f},{sf_z:.1f}\n")
            print(flush=True)
            print(f"{part.name:28s} | {case.name:45s} | vm {p995:6.2f} MPa | disp {disp.max():.3f} mm | "
                  f"SF {sf:5.1f} (layers {sf_z:4.1f}) | elems {mesh.t.shape[1]}")
            render(mesh, vm, f"{part.name}: {case.name}  (99.5% vm {p995:.1f} MPa, SF {sf:.1f})",
                   OUT / f"{part.name.split(' ')[0]}_{i}.png")
    return rows


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--case=")]
    cf = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--case=")), None)
    main(args, cf)
