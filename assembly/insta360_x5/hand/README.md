# Insta360 X5 · Hand

<p align="center"><img src="img/hand.png" alt="The Insta360 X5 on the wrist saddle, strapped behind the wrist, looking forward over a relaxed hand; the thumb strap runs from the saddle's front tab along the back of the hand" width="760"></p>

For bare-hand demonstrations, the X5 rides on the back of your forearm, just behind the wrist, and looks forward over
your hand at what it grasps. There are no jaws: your own hand is the end effector. It uses the same camera frame and
PG pod as the handheld grippers; only the pod's lid changes. A thumb strap from the saddle round your palm carries a
soft **palm button**: curl your middle and ring fingers into your palm to start and stop recording.

| Build | Use | Files |
|---|---|---|
| `hand` | the X5 on the wrist, either arm | `assembly/hand.*`, `urdf/pg_hand.urdf` |

## How it works

- **`pg_wrist_saddle`** replaces the pod's lid. Its top is the lid: the same footprint, locating rim and four M2 screws.
  Underneath is a saddle shaped to the back of the forearm (concave across the arm, R 38 mm), 60 mm long, with two
  slotted wings for a 25 mm hook-and-loop strap round the forearm, and a **front tab** for the thumb strap.
- **Pitch:** the saddle tilts the device 15° forward. With the X5's own 10° in its frame, the camera looks about 25°
  down, over the back of your hand.
- **Same pod:** the IMU stays rigid to the camera, so syncing with the X5's IMU works as for the grippers. The palm
  button takes the record button's place (D2).
- **Either arm:** the saddle is symmetric, so one build serves both.

<p align="center"><img src="img/pod.png" alt="The PG pod: the saddle replaces its lid" width="460"></p>

## Thumb strap and palm button

<p align="center"><img src="img/hand_palm.png" alt="From the palm side: the thumb strap across the palm, between thumb and index finger, with the white palm button on it under the middle and ring fingers, and its wire running round the outer edge of the hand" width="760"></p>

The forearm strap holds the saddle where it sits; the thumb strap stops it sliding up the arm or turning round it, so
the camera stays put relative to your hand.

- **Thumb strap:** a 20 mm hook-and-loop strap. It loops round the front tab's bar, runs along the back of your hand,
  goes round the palm through the web between thumb and index finger, and closes on itself over the back of the hand.
  It doesn't move the saddle: the saddle stays just behind the wrist.
- **`palm_button`:** one soft TPU 95A part, 32 × 26 × 10 mm, threaded on the strap in your palm under the middle and
  ring fingers. **Curl those two fingers into your palm, like Spider-Man firing a web, to press it: once to start
  recording, again to stop.** Inside is a soft 6 × 6 mm tactile switch (about 160 gf) under a 1.2 mm membrane with a
  nub; three shallow rings on the dome let you find it by feel. It slides along the strap to fit your palm.
- **Wire:** a thin 2-core wire from the button, round the outer (little-finger) edge of the hand on the strap, back
  along it to the pod, and in through the pod's front-wall wire hole. Wiring: [electronics.md](electronics.md).

## On a robot hand

<p align="center"><img src="img/hand_robot.png" alt="The same X5 strapped to the forearm of a Shadow Hand on a UR5e, looking over the robot's fingers" width="760"></p>

For a dexterous robot hand, the same X5 goes on the robot's forearm, in the same place relative to the hand as on
your wrist. The robot then sees what you saw.

- **`pg_wrist_saddle_robot`** is `pg_wrist_saddle` with a wider saddle (R 68.5 mm) for round robot forearms up to
  Ø137 mm, and without the thumb tab (the robot doesn't need the thumb strap or the button).
- **Fixing:** the same 25 mm strap goes round the forearm through the wings.
- **Shown:** a Shadow Hand (MuJoCo Menagerie model) on a UR5e, with the saddle's front edge about 30 mm behind the
  wrist.

## Files

| Folder | Contents |
|---|---|
| `STEP/`, `STL/` | `x5_lite_frame`, `pg_pod`, `pg_wrist_saddle`, `pg_wrist_saddle_robot` and `palm_button` |
| `assembly/` | `hand.glb` / `hand.stl` |
| `urdf/` | `pg_hand.urdf` (no joints; `pg_camera` on `pg_base`) |

- **Bill of materials:** [bom.md](bom.md).
- **Wiring:** [electronics.md](electronics.md).

## Build

1. **Print:** `x5_lite_frame` in blue PLA; `pg_pod` and `pg_wrist_saddle` in white PLA; `palm_button` in TPU 95A.
2. **Pod:** build it as in [electronics.md](electronics.md). Set the X5 in the frame, seat the pod under it, and screw
   the 1/4"-20 × 5/8" up through the pod into the camera.
3. **Saddle:** screw it on in place of the lid (4 × M2 × 6, through the access holes from below). Stick a thin foam pad
   under it.
4. **Palm button:** push the switch into the button's pocket through the slit at its wrist end, solder the wire, and
   lead it out of the wire channel. Thread the thumb strap through the button's tunnel.
5. **Wear it:** thread a 25 mm strap through both wings and strap it on just behind the wrist, with the camera facing
   your fingers. Loop the thumb strap round the front tab's bar, lay it along the back of your hand, take it round your
   palm between thumb and index finger with the button under your middle and ring fingers, and close it on itself. Run
   the wire along the strap into the pod's front-wall hole on your little-finger side.
