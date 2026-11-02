# Scissor-UMI

FOMI-style scissor grippers ([robbyant-research.github.io/FOMI](https://robbyant-research.github.io/FOMI/)) whose
jaws copy a real robot gripper's linkage, with a RealSense D405 and a Meta Quest Touch Pro controller on top. The
robot twin is the real gripper with the same head on a flange collar.

| Robot gripper | Folder | Status |
|---|---|---|
| Robotiq 2F-140 | [`2f-140/`](2f-140/README.md) | first version, for review |
| Robotiq 2F-85 | (next) | add its entry to `GRIPPERS` in `cad/kin.py` |

## Build

The code is in `cad/` (build123d, bd_warehouse, trimesh, pyvista; the same environment as PG Chopstick):

```
cd scissor-umi/cad
python package.py          # parts -> ../<gripper>/STEP, clearance check, URDFs, renders -> ../<gripper>/img
python assembly.py --check # interference over the stroke only
```

The renders need two models that aren't in the repository. Put Meta's Touch Pro STL in `cad/source/quest/` and
Robotiq's 2F-140 visual meshes in `cad/source/robotiq_2f_140/`; each folder's README says where to get them.
HandUMI's D405 cup and Quest ring are read from `handumi/hardware/STEP/right/`.
