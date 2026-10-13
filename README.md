# The UMI no one wants

Open hardware for collecting robot-manipulation data with your own hands. Every device here is printable: STEP and STL
for every part, full assemblies, bills of materials, wiring and renders. Each robot twin carries the same jaws and
camera as the device, so a policy trained on your demonstrations sees the same thing on the robot.

<table>
<tr>
<td align="center" width="50%"><a href="handumi/README.md"><img src="handumi/docs/img/hero.png" alt="HandUMI: hand-worn parallel gripper with a RealSense D405 wrist camera" width="430"></a></td>
<td align="center" width="50%"><a href="pg-chopstick/README.md"><img src="pg-chopstick/insta360_x5/chopstick/img/hero.png" alt="PG Chopstick: handheld chopstick gripper with an Insta360 X5" width="430"></a></td>
</tr>
<tr>
<td><b><a href="handumi/README.md">HandUMI · D405</a></b>: a hand-worn parallel-jaw gripper with a RealSense D405
depth camera on a tilting wrist hinge, a record button under your index finger, an IMU + Pico 2 box on one USB cable,
and swappable tips for Piper, Robotiq Hand-E and Open-ENPIRE.</td>
<td><b><a href="pg-chopstick/README.md">PG Chopstick</a></b>: two camera kits (Insta360 X5, or D405 + Quest Touch
Pro) × three ways to grasp: geared chopsticks, the HandUMI parallel gripper, or the camera on your bare wrist. Robot
twins for UR5e, ABB GoFa, FANUC CRX and OpenArm.</td>
</tr>
</table>

## Pick a device

| Device | Camera | Gripper | Folder |
|---|---|---|---|
| HandUMI | RealSense D405 | Parallel, T-plate | [`handumi/`](handumi/) |
| PG Chopstick · X5 · chopstick | Insta360 X5 | Geared chopsticks | [`pg-chopstick/insta360_x5/chopstick/`](pg-chopstick/insta360_x5/chopstick/) |
| PG Chopstick · X5 · parallel | Insta360 X5 | HandUMI, X5 cradle on its hinge | [`pg-chopstick/insta360_x5/parallel/`](pg-chopstick/insta360_x5/parallel/) |
| PG Chopstick · X5 · hand | Insta360 X5 | Your hand (wrist mount) | [`pg-chopstick/insta360_x5/hand/`](pg-chopstick/insta360_x5/hand/) |
| PG Chopstick · D405 · chopstick | D405 + Quest Touch Pro | Geared chopsticks | [`pg-chopstick/d405/chopstick/`](pg-chopstick/d405/chopstick/) |
| PG Chopstick · D405 · parallel | D405 + Quest Touch Pro | HandUMI as-is | [`pg-chopstick/d405/parallel/`](pg-chopstick/d405/parallel/) |
| PG Chopstick · D405 · hand | D405 + Quest Touch Pro | Your hand (wrist mount) | [`pg-chopstick/d405/hand/`](pg-chopstick/d405/hand/) |

## Repository layout

```
handumi/              HandUMI: hardware/ (STEP, STL, tips, NOTICE), assembly/, docs/ (BOM, wiring, renders)
pg-chopstick/         PG Chopstick: insta360_x5/, d405/, tags/, docs/media/ (overview video), NOTICE, LICENSE
```

## Branches

| Branch | What's on it |
|---|---|
| `main` | Everything above: the files you print and build from |
| [`cad`](../../tree/cad) | The build code (Python, build123d) for both projects |
| [`pg-chopstick-x5`](../../tree/pg-chopstick-x5), [`pg-chopstick-d405`](../../tree/pg-chopstick-d405) | One PG Chopstick kit each, with its code |

## Licences

Apache-2.0. Third-party sources and modifications: [`handumi/hardware/NOTICE.md`](handumi/hardware/NOTICE.md) and
[`pg-chopstick/NOTICE.md`](pg-chopstick/NOTICE.md).
