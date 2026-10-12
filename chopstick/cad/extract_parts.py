"""Split KIWI's chopstick assemblies into one STEP and one STL per printed part, in the gripper frame.

Jaws and tips are written closed (theta = 0), so every file lines up with the others and with the URDF.
The Insta360 button pieces exist only in KIWI's slicer project, so they are taken from the 3MF as STL.

    python extract_parts.py
"""
import re
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from build123d import export_step, export_stl

from kiwi import HW, load_parts

THREEMF = HW / "print" / "KIWI-3dprint.3mf"
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
    (HW / "STEP").mkdir(parents=True, exist_ok=True)
    (HW / "STL").mkdir(parents=True, exist_ok=True)
    for name, shape in load_parts().items():
        if name.endswith("_envelope"):
            continue
        export_step(shape, str(HW / "STEP" / f"{name}.step"))
        export_stl(shape, str(HW / "STL" / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.15)
        bb = shape.bounding_box()
        print(f"{name:22s} {bb.size.X:6.1f} x {bb.size.Y:6.1f} x {bb.size.Z:6.1f} mm  {shape.volume / 1e3:6.1f} cm3")
    meshes = threemf_meshes(THREEMF)
    for src, name in THREEMF_ONLY.items():
        m = meshes[src]
        m.export(HW / "STL" / f"{name}.stl")
        print(f"{name:22s} {' x '.join(f'{e:6.1f}' for e in m.extents)} mm  (from 3MF)")


if __name__ == "__main__":
    main()
