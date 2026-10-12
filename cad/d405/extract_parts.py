"""Read the upstream slicer project (source/upstream/print.3mf): the two camera-button pieces, which exist only there,
and the painted red dots on the jaws (used by kinematics.py).

    python extract_parts.py      # writes hardware/frame/STL/x5_button_cap.stl and x5_button_shaft.stl
"""
import re
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from parts import SRC, part_path

THREEMF = SRC / "print.3mf"
NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
# 3MF object names -> part file names, for parts that are not in the STEP assemblies.
THREEMF_ONLY = {
    "Insta360 X5 Camera Frame - Insta360X5 Button - Cap": "x5_button_cap",
    "Insta360 X5 Camera Frame - Insta360X5 Button - Shaft": "x5_button_shaft",
}


def threemf_meshes(path, painted=False):
    """{object name: trimesh} for every named object in a Bambu 3MF, in its print orientation.

    With painted=True each mesh also carries `metadata["paint"]`: the Bambu paint code of every triangle ("" if
    unpainted), which is how the red dots on the jaws are stored.
    """
    z = zipfile.ZipFile(path)
    cfg = z.read("Metadata/model_settings.config").decode()
    names = dict(re.findall(r'<object id="(\d+)">\s*<metadata key="name" value="([^"]*)"', cfg))
    root = ET.fromstring(z.read("3D/3dmodel.model"))
    out = {}
    for obj in root.iter(f"{{{NS['m']}}}object"):
        oid = obj.get("id")
        comp = obj.find("m:components/m:component", NS)
        if oid not in names or comp is None:
            continue
        sub = comp.get("{http://schemas.microsoft.com/3dmanufacturing/production/2015/06}path")
        sub_root = ET.fromstring(z.read(sub.lstrip("/")))
        mesh = next(o for o in sub_root.iter(f"{{{NS['m']}}}object")
                    if o.get("id") == comp.get("objectid")).find("m:mesh", NS)
        v = np.array([[float(e.get(k)) for k in "xyz"] for e in mesh.find("m:vertices", NS)])
        tris = list(mesh.find("m:triangles", NS))
        f = np.array([[int(e.get(k)) for k in ("v1", "v2", "v3")] for e in tris])
        name = re.sub(r"\s*\(\d+\)\.stl$|\.stl$", "", names[oid])
        if name in out:
            continue
        m = trimesh.Trimesh(v, f, process=False)
        if painted:
            m.metadata["paint"] = np.array([e.get("paint_color", "") for e in tris])
        out[name] = m
    return out


def main():
    return                                   # the camera-button pieces belong to the Insta360 frame (not in this kit)
    meshes = threemf_meshes(THREEMF)
    for src, name in THREEMF_ONLY.items():
        m = meshes[src]
        m.export(part_path(name, "stl"))
        print(f"{name:22s} {' x '.join(f'{e:6.1f}' for e in m.extents)} mm")


if __name__ == "__main__":
    main()
