"""Copy this kit's generated files into the repository's assembly tree, one folder per end effector.

    python package.py

For each family (chopstick, parallel, hand) of this kit, assembly/<kit>/<family>/ gets:
  STEP/, STL/   every printed part any of the family's builds uses (handheld and robot), plus dot inlays
  assembly/     <build>.glb and <build>.stl for each build
  urdf/         pg_<build>.urdf for each build, with only the meshes they use
  img/          this family's renders
The README.md, bom.md and electronics.md in each folder are written by hand and are not touched.
"""
import re
import shutil

from parts import BUILDS, HERE, HW, REF, ROOT, family

KIT = HERE.name
DEST = HERE.parents[1] / "assembly" / KIT
IMAGES = {
    "chopstick": ["hero", "opening", "pair", "sleeve", "kit", "openarm_bimanual", "robot_ur5", "robot_fanuc_crx",
                  "parts", "plates", "pod"],
    "parallel": ["parallel", "parallel_opening", "robot_abb_gofa", "parts", "plates", "pod"],
    "hand": ["hand", "pod", "parts"],
}


# parts every variant of this kit needs that aren't placed in the assemblies (print-orientation files)
EXTRA = {"insta360_x5": [], "d405": []}           # the X5 starts by voice: no camera-button pieces


def part_files(name):
    """The STEP and STL files of a part (and its dot inlay), wherever its module put them."""
    out = []
    for ext in ("STEP", "STL"):
        out += list(HW.glob(f"*/{ext}/{name}.{ext.lower()}"))
    out += list(HW.glob(f"*/STL/{name}_dots.stl"))
    return out


def package(fam):
    dest = DEST / fam
    builds = [b for b in BUILDS if family(b) == fam]
    for sub in ("STEP", "STL", "assembly", "urdf", "img"):
        if (dest / sub).exists():
            shutil.rmtree(dest / sub)
        (dest / sub).mkdir(parents=True)
    names = sorted({n for b in builds for n in BUILDS[b]} | set(EXTRA[KIT]))
    copied = 0
    for n in names:
        for f in part_files(n):
            shutil.copy2(f, dest / f.parent.name / f.name)
            copied += 1
    for b in builds:
        for ext in ("glb", "stl"):
            src = ROOT / "assemblies" / f"{b}.{ext}"
            if src.exists():
                shutil.copy2(src, dest / "assembly" / src.name)
        urdf = ROOT / "urdf" / f"pg_{b}.urdf"
        if urdf.exists():
            shutil.copy2(urdf, dest / "urdf" / urdf.name)
            (dest / "urdf" / "meshes").mkdir(exist_ok=True)
            for m in re.findall(r'filename="meshes/([^"]+)"', urdf.read_text()):
                shutil.copy2(ROOT / "urdf" / "meshes" / m, dest / "urdf" / "meshes" / m)
    for img in IMAGES[fam]:
        src = ROOT / "docs" / "img" / f"{img}.png"
        if src.exists():
            shutil.copy2(src, dest / "img" / src.name)
    print(f"{KIT}/{fam}: {len(builds)} builds, {len(names)} parts ({copied} files)")


def main():
    for fam in IMAGES:
        package(fam)


if __name__ == "__main__":
    main()
