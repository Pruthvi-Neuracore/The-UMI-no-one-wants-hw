"""Copy this kit's generated files into the repository's assembly tree, one folder per end effector.

    python package.py

Folders and file names: the chopstick family is Chop-UMI (chop-umi/, chop-umi_<build>.*), the parallel family is
Para-UMI (para-umi/, para-umi_right_<tips>.*), the hand family is hand/.

For the chopstick and hand families of this kit, assembly/<kit>/<folder>/ gets:
  STEP/, STL/   every printed part any of the family's builds uses (handheld and robot), plus dot inlays
  assembly/     <build>.glb and <build>.stl for each build
  urdf/         pg_<build>.urdf for each build, with only the meshes they use
  img/          this family's renders
The parallel family (t_gripper.py) gets HandUMI's parts per hand (STEP|STL/{right,left}), the tip sets
(STEP|STL/gripper_tips), one assembly per tip set, and its renders.
The README.md, bom.md and electronics.md in each folder are written by hand and are not touched.
"""
import re
import shutil

from parts import BUILDS, HERE, HW, REF, ROOT, family

KIT = HERE.name
DEST = HERE.parents[1] / KIT  # pg-chopstick/<kit>, as on main
IMAGES = {
    "chopstick": ["hero", "opening", "pair", "sleeve", "kit", "openarm_bimanual", "robot_ur5", "robot_fanuc_crx",
                  "robot_abb_gofa", "parts", "plates", "pod"],
    "parallel": ["parallel", "parallel_back", "parallel_opening", "parallel_parts", "camera_mount",
                 "tips_agilex-piper", "tips_robotiq-hand-e", "tips_open-enpire",
                 "robot_camera_hande", "robot_camera_piper", "robot_camera_openarm"],
    "hand": ["hand", "hand_palm", "hand_robot", "pod", "parts"],
}
FOLDER = {"chopstick": "chop-umi", "parallel": "para-umi", "hand": "hand"}


def public(name):
    """A build's file name in the repository: chopstick_* -> chop-umi_*, *parallel* -> para-umi."""
    name = re.sub(r"^(pg_)?chopstick", lambda m: (m.group(1) or "") + "chop-umi", name)
    return re.sub(r"^(x5|d405)_parallel", "para-umi", name)


# parts a family needs that aren't in its builds: the wrist saddle for a robot hand's forearm (hand_robot.png), the
# palm button (on the thumb strap, not rigid to the head) and the left hand's Quest mount
EXTRA = {"insta360_x5": {"hand": ["pg_wrist_saddle_robot", "palm_button"]},
         "d405": {"hand": ["ebox_wrist_saddle_robot", "palm_button", "quest_arm_left", "quest_support_left"]}}


def part_files(name):
    """The STEP and STL files of a part (and its dot inlay), wherever its module put them."""
    out = []
    for ext in ("STEP", "STL"):
        out += list(HW.glob(f"*/{ext}/{name}.{ext.lower()}"))
    out += list(HW.glob(f"*/STL/{name}_dots.stl"))
    return out


def package_t_gripper(dest):
    """The parallel family is the HandUMI T-plate gripper (t_gripper.py): its parts per hand, the tip sets, one
    assembly per tip set, its renders and HandUMI's robot renders (the robots carry their own grippers)."""
    src = ROOT / "t_gripper"
    for sub in ("STEP", "STL", "assembly", "img", "urdf"):
        if (dest / sub).exists():
            shutil.rmtree(dest / sub)
    for sub in ("STEP", "STL"):
        shutil.copytree(src / "hardware" / sub, dest / sub)
    (dest / "assembly").mkdir()
    for f in sorted((src / "assembly").iterdir()):
        shutil.copy2(f, dest / "assembly" / public(f.name))
    (dest / "img").mkdir()
    for img in IMAGES["parallel"]:
        f = ROOT / "docs" / "img" / f"{img}.png"
        if f.exists():
            shutil.copy2(f, dest / "img" / f.name)
    for f in sorted((HERE / "handumi" / "docs" / "img").glob("*.png")):
        shutil.copy2(f, dest / "img" / f.name)
    n = sum(1 for _ in dest.rglob("*") if _.is_file())
    print(f"{KIT}/{FOLDER['parallel']}: HandUMI T-plate gripper, {n} files")


def package(fam):
    dest = DEST / FOLDER[fam]
    if fam == "parallel":
        return package_t_gripper(dest)
    builds = [b for b in BUILDS if family(b) == fam]
    for sub in ("STEP", "STL", "assembly", "urdf", "img"):
        if (dest / sub).exists():
            shutil.rmtree(dest / sub)
        (dest / sub).mkdir(parents=True)
    names = sorted({n for b in builds for n in BUILDS[b]} | set(EXTRA[KIT].get(fam, [])))
    copied = 0
    for n in names:
        for f in part_files(n):
            shutil.copy2(f, dest / f.parent.name / f.name)
            copied += 1
    for b in builds:
        for ext in ("glb", "stl"):
            src = ROOT / "assemblies" / f"{b}.{ext}"
            if src.exists():
                shutil.copy2(src, dest / "assembly" / public(src.name))
        urdf = ROOT / "urdf" / f"pg_{b}.urdf"
        if urdf.exists():
            shutil.copy2(urdf, dest / "urdf" / public(urdf.name))
            (dest / "urdf" / "meshes").mkdir(exist_ok=True)
            for m in re.findall(r'filename="meshes/([^"]+)"', urdf.read_text()):
                shutil.copy2(ROOT / "urdf" / "meshes" / m, dest / "urdf" / "meshes" / m)
    for img in IMAGES[fam]:
        src = ROOT / "docs" / "img" / f"{img}.png"
        if src.exists():
            shutil.copy2(src, dest / "img" / src.name)
    print(f"{KIT}/{FOLDER[fam]}: {len(builds)} builds, {len(names)} parts ({copied} files)")


def main():
    for fam in IMAGES:
        package(fam)


if __name__ == "__main__":
    main()
