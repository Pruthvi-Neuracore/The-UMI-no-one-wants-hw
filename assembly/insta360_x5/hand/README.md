# Insta360 X5 · Hand

<p align="center"><img src="img/hand.png" alt="The Insta360 X5 on the wrist saddle, strapped to a forearm, looking forward over the hand" width="700"></p>

For bare-hand demonstrations, the X5 rides on the back of your forearm, just behind the wrist, and looks forward over
your hand at what it grasps. There are no jaws: your own hand is the end effector. It uses the same camera frame and
PG pod as the handheld grippers; only the pod's lid changes.

| Build | Use | Files |
|---|---|---|
| `hand` | the X5 on the wrist, either arm | `assembly/hand.*`, `urdf/pg_hand.urdf` |

## How it works

- **`pg_wrist_saddle`** replaces the pod's lid. Its top is the lid: the same footprint, locating rim and four M2 screws.
  Underneath is a saddle shaped to the back of the forearm (concave across the arm, R 38 mm), 60 mm long, with two
  slotted wings for a 25 mm hook-and-loop strap.
- **Pitch:** the saddle tilts the device 15° forward. With the X5's own 10° in its frame, the camera looks about 25°
  down, over the back of your hand.
- **Same pod:** the IMU stays rigid to the camera, so syncing with the X5's IMU works as for the grippers. With no
  finger sleeve there is no record button; start and stop the X5 by voice.
- **Either arm:** the saddle is symmetric, so one build serves both.

<p align="center"><img src="img/pod.png" alt="The PG pod: the saddle replaces its lid" width="460"></p>

## Files

| Folder | Contents |
|---|---|
| `STEP/`, `STL/` | `x5_lite_frame`, `pg_pod` and `pg_wrist_saddle` |
| `assembly/` | `hand.glb` / `hand.stl` |
| `urdf/` | `pg_hand.urdf` (no joints; `pg_camera` on `pg_base`) |

- **Bill of materials:** [bom.md](bom.md).
- **Wiring:** [electronics.md](electronics.md).

## Build

1. **Print:** `x5_lite_frame` in blue PLA; `pg_pod` and `pg_wrist_saddle` in white PLA.
2. **Pod:** build it as in [electronics.md](electronics.md). Set the X5 in the frame, seat the pod under it, and screw
   the 1/4"-20 × 5/8" up through the pod into the camera.
3. **Saddle:** screw it on in place of the lid (4 × M2 × 6, through the access holes from below). Stick a thin foam pad
   under it.
4. **Wear it:** thread a 25 mm strap through both wings and strap it on just behind the wrist, with the camera facing
   your fingers.
