"""A posed human hand for the wrist renders: Meta's rigged hand model, curled into a relaxed pose by linear-blend skinning.

The model (source/hand/HandR.glb, the hand from Meta's Quest hand-tracking samples) is not redistributed here: see
source/hand/README.md. Without it the renders fall back to a plain forearm.
"""
import json
import struct
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

GLB = Path(__file__).resolve().parent / "source" / "hand" / "HandR.glb"
DTYPE = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
SIZE = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
# relaxed curl per joint (degrees of flexion, towards the palm)
CURL = {"thumb1": 8, "thumb2": 10, "thumb3": 12,
        "index1": 14, "index2": 22, "index3": 14,
        "middle1": 18, "middle2": 26, "middle3": 16,
        "ring1": 22, "ring2": 30, "ring3": 18,
        "pinky1": 26, "pinky2": 34, "pinky3": 20}


def _read(path):
    data = path.read_bytes()
    n = struct.unpack("<I", data[12:16])[0]
    g = json.loads(data[20:20 + n])
    binary = data[20 + n + 8:]

    def acc(i):
        a = g["accessors"][i]
        bv = g["bufferViews"][a["bufferView"]]
        k, dt = SIZE[a["type"]], DTYPE[a["componentType"]]
        start = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        stride = bv.get("byteStride", 0) or k * np.dtype(dt).itemsize
        raw = np.frombuffer(binary, np.uint8, count=stride * (a["count"] - 1) + k * np.dtype(dt).itemsize, offset=start)
        rows = np.lib.stride_tricks.as_strided(raw, (a["count"], k * np.dtype(dt).itemsize), (stride, 1))
        return np.ascontiguousarray(rows).view(dt).reshape(a["count"], k)
    return g, acc


def _local(node):
    if "matrix" in node:
        return np.array(node["matrix"], float).reshape(4, 4).T
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(node.get("rotation", [0, 0, 0, 1])).as_matrix() @ np.diag(node.get("scale", [1, 1, 1]))
    T[:3, 3] = node.get("translation", [0, 0, 0])
    return T


def _globals(g, local):
    parent = {c: i for i, nd in enumerate(g["nodes"]) for c in nd.get("children", [])}
    out = {}

    def G(i):
        if i not in out:
            out[i] = (G(parent[i]) if i in parent else np.eye(4)) @ local[i]
        return out[i]
    for i in range(len(g["nodes"])):
        G(i)
    return out


def hand(curl=1.0):
    """The right hand in its own frame (mm): wrist at the origin, fingers along +x, back of the hand +z, thumb +y."""
    g, acc = _read(GLB)
    nodes = g["nodes"]
    mesh_node = next(i for i, nd in enumerate(nodes) if "mesh" in nd)
    prim = g["meshes"][nodes[mesh_node]["mesh"]]["primitives"][0]
    V = acc(prim["attributes"]["POSITION"]).astype(float)
    F = acc(prim["indices"]).reshape(-1, 3).astype(np.int64)
    J = acc(prim["attributes"]["JOINTS_0"]).astype(int)
    W = acc(prim["attributes"]["WEIGHTS_0"]).astype(float)
    skin = g["skins"][0]
    joints = skin["joints"]
    IBM = acc(skin["inverseBindMatrices"]).reshape(-1, 4, 4).transpose(0, 2, 1)
    local = {i: _local(nd) for i, nd in enumerate(nodes)}
    G0 = _globals(g, local)
    # palm normal and finger direction from the bind pose
    pos = {nodes[i]["name"]: G0[i][:3, 3] for i in range(len(nodes))}
    wrist = pos["b_r_wrist"] if "b_r_wrist" in pos else np.mean([pos["b_r_index1"], pos["b_r_pinky1"]], 0)
    fwd = pos["b_r_middle1"] - wrist
    fwd /= np.linalg.norm(fwd)
    across = pos["b_r_index1"] - pos["b_r_pinky1"]
    palm = np.cross(across, fwd)                     # right hand: index x pinky ... checked below against the thumb
    palm /= np.linalg.norm(palm)
    if np.dot(pos["b_r_thumb3"] - wrist, palm) < 0:   # the thumb tip sits a little on the palm side
        palm = -palm
    for i, nd in enumerate(nodes):
        name = nd.get("name", "")
        key = name.replace("b_r_", "")
        if key not in CURL or not any(c.get("name") for c in [nodes[k] for k in nd.get("children", [])]):
            continue
        child = next(k for k in nd["children"] if nodes[k].get("name", "").startswith("b_r_"))
        d = G0[child][:3, 3] - G0[i][:3, 3]
        d /= np.linalg.norm(d)
        axis_w = np.cross(d, palm)
        axis_w /= np.linalg.norm(axis_w)
        Rw = Rotation.from_rotvec(axis_w * np.radians(CURL[key] * curl)).as_matrix()
        Gi = G0[i][:3, :3]
        Rl = np.linalg.inv(Gi) @ Rw @ Gi                # same rotation expressed in the joint's local frame
        local[i] = local[i].copy()
        local[i][:3, :3] = local[i][:3, :3] @ Rl
        G0 = G0  # bind pose kept for the axes
    G1 = _globals(g, local)
    Mj = np.array([G1[j] @ IBM[k] for k, j in enumerate(joints)])
    Vh = np.c_[V, np.ones(len(V))]
    out = np.zeros((len(V), 4))
    for c in range(4):
        out += W[:, c:c + 1] * np.einsum("nij,nj->ni", Mj[J[:, c]], Vh)
    Vw = out[:, :3]
    # into the hand frame (mm): wrist at the origin, +x fingers, +z back of the hand
    z = -palm
    x = fwd - z * fwd.dot(z)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(z, x), z])
    wrist_cap = Vw[(Vw - wrist) @ fwd < (Vw - wrist).dot(fwd).min() + 0.004]   # the open cut at the wrist
    o = wrist_cap.mean(0)
    m = trimesh.Trimesh((Vw - o) @ R * 1000.0, F, process=True)
    return m


if __name__ == "__main__":
    m = hand()
    print(m.bounds.round(1), m.is_watertight)
