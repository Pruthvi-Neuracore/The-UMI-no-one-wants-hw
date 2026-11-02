"""Build everything for the gripper in kin.GRIPPER into ../<gripper>/: STEP (printed parts), URDFs and renders.

    python package.py            # parts + check + urdf + renders
    python package.py --no-render
"""
import shutil
import sys

import assembly
import kin
import render
import scissor as S
import urdf

DST = S.HERE.parent / kin.GRIPPER


def main():
    S.build_all()
    assembly.export_refs()
    (DST / "STEP").mkdir(parents=True, exist_ok=True)
    for f in sorted(S.STEP_DIR.glob("*.step")):
        if not f.name.startswith("_"):                  # placed HandUMI references stay in build/
            shutil.copy(f, DST / "STEP" / f.name)
    assembly.check()
    urdf.main()
    if "--no-render" not in sys.argv:
        render.OUT = DST / "img"
        render.OUT.mkdir(parents=True, exist_ok=True)
        for f in render.ALL.values():
            f()


if __name__ == "__main__":
    main()
