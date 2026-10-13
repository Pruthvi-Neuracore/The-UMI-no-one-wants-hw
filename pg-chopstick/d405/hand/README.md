# D405 + Quest · Hand

<p align="center"><img src="img/hand.png" alt="The D405 head strapped behind the wrist, over a relaxed hand: D405 on its tilt hinge looking over the hand, the electronics box, and the Quest Touch Pro controller upright behind" width="700"></p>

For bare-hand demonstrations, the whole D405 head rides on the back of your forearm, just behind the wrist:

- **D405** on its tilt hinge, looking forward and down over your hand at what it grasps;
- **electronics box** (Pico 2, IMU, USB 3 hub);
- **Quest Touch Pro controller**, upright in its cradle, so it tracks the wrist.

There are no jaws: your own hand is the end effector. The only change from the handheld head is the box's lid, which
becomes a strap-on saddle.

| Build | Use | Files |
|---|---|---|
| `hand` | the D405 head on the wrist, either arm | `assembly/hand.*`, `urdf/pg_hand.urdf` |

## How it works

- **`ebox_wrist_saddle`** replaces `ebox_lid`. Its top is the lid: the same footprint and four M2 screws. Underneath is
  a saddle shaped to the back of the forearm (concave, R 38 mm), 60 mm long, with two slotted wings for a 25 mm
  hook-and-loop strap.
- **Pitch:** the saddle tilts the head 15° forward. Set the D405's own tilt on its hinge (15° to 65°) so it frames your
  fingers. The camera frame turns about the hinge axis at (2.4, 0, 37.2) mm.
- **Tracking:** the Quest Touch Pro controller stands upright behind the box above your arm, where its
  tracking cameras see the room. Its buttons are not used.
- **Unused interface:** the frame's front interface has no end effector here.

## On a robot hand

<p align="center"><img src="img/hand_robot.png" alt="The same D405 head strapped to the forearm of a Shadow Hand on a UR5e, looking over the robot's fingers" width="760"></p>

For a dexterous robot hand, the same D405 head goes on the robot's forearm, in the same place relative to the hand as on
your wrist. The robot then sees what you saw.

- **`ebox_wrist_saddle_robot`** is `ebox_wrist_saddle` with a wider saddle (R 68.5 mm) for round robot forearms up to Ø137 mm.
- **Fixing:** the same 25 mm strap goes round the forearm through the wings.
- **Shown:** a Shadow Hand (MuJoCo Menagerie model) on a UR5e, with the saddle's front edge about 30 mm behind the
  wrist.

## Files

| Folder | Contents |
|---|---|
| `STEP/`, `STL/` | `d405_frame`, `d405_cup` (HandUMI cup at 35°), `ebox`, `ebox_wrist_saddle`, `ebox_wrist_saddle_robot`, `quest_cradle` |
| `assembly/` | `hand.glb` / `hand.stl` |
| `urdf/` | `pg_hand.urdf` (no joints; `pg_camera` on `pg_base`) |

- **Bill of materials:** [bom.md](bom.md).
- **Wiring:** [electronics.md](electronics.md).

## Build

1. **Print:** `d405_frame`, `d405_cup` and `quest_cradle` in blue PLA; `ebox` and `ebox_wrist_saddle` in white PLA.
2. **Camera:** screw the D405 into the cup and hang the cup on the knuckle (M4 × 30 + nyloc). Route its cable out of
   the top window into the box.
3. **Box:** fit the electronics, bolt the box to the frame's back plate, and screw the saddle on in place of the lid.
   Add a thin foam pad under it.
4. **Controller:** screw the cradle to the box's back, line it with foam, and strap the controller's handle in.
5. **Wear it:** thread a 25 mm strap through the saddle's wings and strap it on just behind the wrist, with the camera
   facing your fingers.
