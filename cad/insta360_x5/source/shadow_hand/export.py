"""Pose MuJoCo Menagerie's Shadow Hand (right) and write each visual geom, in the forearm frame, to one PLY per material."""
import sys, numpy as np, mujoco
xml, out = sys.argv[1], sys.argv[2]
m = mujoco.MjModel.from_xml_path(xml)
d = mujoco.MjData(m)
import re, pathlib
kx = (pathlib.Path(xml).parent / "keyframes.xml").read_text()
keys = {n: np.array(v.split(), float) for n, v in re.findall(r'name="([^"]+)"\s+qpos="([^"]+)"', kx)}
q = 0.3 * keys["close hand"] + 0.7 * keys["pre grasp"]
q[:2] = 0
d.qpos[:] = q
mujoco.mj_forward(m, d)
fa = m.body("rh_forearm").id
Rf, pf = d.xmat[fa].reshape(3, 3), d.xpos[fa]
groups = {}
for g in range(m.ngeom):
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
        continue
    mid = m.geom_dataid[g]
    va, nv = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
    fa_, nf = m.mesh_faceadr[mid], m.mesh_facenum[mid]
    v = m.mesh_vert[va:va + nv]
    f = m.mesh_face[fa_:fa_ + nf]
    R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    w = v @ R.T + p
    loc = (w - pf) @ Rf                      # into the forearm frame
    rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
    key = "%02x%02x%02x" % tuple(int(c * 255) for c in rgba[:3])
    groups.setdefault(key, []).append((loc * 1000.0, f))
arrs = {}
for k, ms in groups.items():
    off, V, F = 0, [], []
    for v, f in ms:
        V.append(v); F.append(f + off); off += len(v)
    arrs[k + "_v"], arrs[k + "_f"] = np.vstack(V), np.vstack(F)
    print(k, len(ms), arrs[k + "_v"].min(0).round(1), arrs[k + "_v"].max(0).round(1))
np.savez_compressed(out, **arrs)
