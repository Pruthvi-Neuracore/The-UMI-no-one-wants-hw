# Scissor-UMI

FOMI-style scissor grippers ([robbyant-research.github.io/FOMI](https://robbyant-research.github.io/FOMI/)) whose
jaws are printed copies of a real robot gripper's jaws, with a RealSense D405 and a Meta Quest Touch Pro controller on top. The
robot twin is the real gripper with the same head on a flange collar.

<p align="center"><img src="2f-140/img/hero.png" alt="Scissor-UMI for the Robotiq 2F-140: scissor handles, geared four-bar jaws, a D405 on a tilting hinge and a Quest Touch Pro on top" width="760"></p>

## Handheld and robot twin

<p align="center"><img src="2f-140/img/twins.png" alt="The handheld and the Robotiq 2F-140 with the same D405 + Quest head, seen from behind at the same jaw angle" width="880"></p>

| Handheld | Robot twin (real Robotiq 2F-140) |
|---|---|
| <img src="2f-140/img/hero.png" alt="Scissor-UMI handheld" width="420"> | <img src="2f-140/img/robot_2f140.png" alt="Robotiq 2F-140 with the collar, D405 and Quest" width="420"> |

## Same view on both

<p align="center"><img src="2f-140/img/camera_views.png" alt="What the D405 sees on the handheld (left) and on the 2F-140 (right) at the same q" width="880"></p>

What the D405 sees on the handheld (left) and on the robot (right) at the same q: the jaws and pads are identical.

## Mechanism

| Open (q = 0) | Closed (q = 0.7) |
|---|---|
| <img src="2f-140/img/top_open.png" alt="Top view, jaws open" width="380"> | <img src="2f-140/img/top_closed.png" alt="Top view, jaws closed" width="380"> |

<p align="center"><img src="2f-140/img/stroke.png" alt="The stroke from open to closed" width="880"></p>

Scissor handles drive two meshed 20-tooth knuckle gears through 40-tooth sectors (2:1). The jaws are printed copies
of the 2F-140's jaw parts on its own joint positions, so they look and move exactly like the robot's. Details, BOM and wiring are in
[`2f-140/README.md`](2f-140/README.md).

## Grippers

| Robot gripper | Folder | Status |
|---|---|---|
| Robotiq 2F-140 | [`2f-140/`](2f-140/README.md) | first version, for review |
| Robotiq 2F-85 | (next) | add its entry to `GRIPPERS` in `cad/kin.py` |

## Build

The code is in `cad/` (build123d, bd_warehouse, trimesh, pyvista; the same environment as PG Chopstick):

```
cd scissor-umi/cad
python package.py          # parts -> ../<gripper>/STEP, clearance check, URDFs, renders -> ../<gripper>/img
python verify.py           # gears over the stroke, jaws vs the robot's 2F-140, interference
```

The renders need two models that aren't in the repository. Put Meta's Touch Pro STL in `cad/source/quest/` and
Robotiq's 2F-140 visual meshes in `cad/source/robotiq_2f_140/`; each folder's README says where to get them.
HandUMI's D405 cup and Quest ring are read from `handumi/hardware/STEP/right/`.
