# Build

## Print

Print every part from `hardware/STL/` at 0.2 mm layers with a 0.4 mm nozzle, in the colours from [bom.md](bom.md).

- **Jaws:** print them flat (top face up) with the red dots. Load `jaw_*.stl`, add the matching `jaw_*_dots.stl` as a
  part (Bambu/Orca) or modifier (Prusa), and set it to red. On a single-extruder printer, mark the dots with a red
  paint pen instead; the dot STL shows where they go.
- **Robot plates** (`openarm_adapter`, `iso_flange_plate`): back face down, 4 walls, PETG, 100 % infill around the
  holes.
- **Pod:** `pg_pod` top-down, so the pocket is on the bed with light supports in the pocket. `pg_pod_lid` flat.
  `pg_pod_button` in TPU, flange down.
- **D405 frame:** on its back plate, with tree supports under the cup.

## Handheld gripper

1. **Inserts:** two M2 × 4 heat-set inserts in the back of the base, with M2 × 4 screws in them for positioning, and two
   M3 × 4 inserts from the underside.
2. **Jaws:** glue on the TPU tips and press two 624 bearings into each jaw.
3. **Sleeve button:** the button goes in the index-finger sleeve. That is the **right jaw (`jaw_right_dc`) on the
   right-hand gripper (start/stop)** and the **left jaw (`jaw_left_dc`) on the left-hand gripper (cancel)**. Solder the
   2-core wire to a 6 × 6 switch. Lay the wire in the channel
   at the bottom of the pad's pocket and out of the sleeve's back end, then press the switch into the pocket with its
   plunger up. Press the TPU cover into the oval recess so its nub sits on the plunger; a drop of flexible glue on the
   rim holds it. Put a cover without a switch on the thumb jaw's pocket. Run the wire along the jaw's
   underside to the pivot, leaving a loop of slack there for the jaw's swing.
4. **Join:** hook the rubber band through the centre cavity onto both jaws. Mesh the gears **with the tips touching**,
   then fit the two M3 shoulder screws. The jaws must swing freely; if they bind, check your printer's tolerances.
5. **Strap and foam:** pin the wrist strap under the base, then add foam and grip tape to taste.
6. **Camera frame:**
   - **X5:** build the button into `x5_lite_frame` (M2 × 4) and set the X5 in it. Then join the frame to the base with
     the clip, the M3 × 55 screws and nuts.
   - **D405:** screw the camera into `d405_frame`'s cup (2 × M3 × 6), feed the cable out of a side window, then join the
     frame to the base the same way.
7. **PG pod (X5 build):** follow [pod.md](pod.md) to wire it and close the lid. Seat the pod's pocket on the bottom
   of the X5 frame, then screw the 1/4"-20 × 5/8" button-head up from inside the pod, through the frame, into the
   camera. Feed the sleeve-button wire in through the front-wall hole on its side, and wire it to the XIAO. Do all of
   this before fitting the lid.
8. **Tag:** stick one ArUco tag on the base.

## Robot gripper

1. **Base and tips:** fit four M2 × 4 inserts in the back of the base and one M3 × 4 in its underside, then glue on the
   TPU tips.
2. **Servo:** take the rotor plate off the SM40BL and screw it to `jaw_left_sa` (4 × M3 × 12). Push the driver jaw into
   the base and slide the servo in from below. Fit the shoulder screw through into the servo's coupling plate, then fix
   the servo with its own screws.
3. **Driven jaw:** press the bearings into `jaw_right`. Mesh the gears **with both jaws closed**, then fit the second
   shoulder screw.
4. **Mount plate:** melt four M3 heat-set inserts into the plate's front, then bolt it to the robot:
   - **OpenArm:** `openarm_adapter` on link7, with 6 × M3 × 16 through the counterbores and the boss in the back
     pocket. The tongue's long edges run along link7 X, so the jaws open along link7 Y.
   - **ISO 50:** `iso_flange_plate` on the tool flange, with the spigot in the flange's centring bore, a Ø6 dowel in the
     pin hole, and 4 × M6 × 14 through the counterbores.
5. **Camera frame:**
   - **X5 frame:** drill its four rear-wall Ø2.8 holes out to Ø3.2. With the camera out, fit the tongue between the
     frame's rear legs and screw on from inside with 4 × M3 × 10. Fit clip B, then join the frame to the base.
   - **D405 frame:** its back plate already has Ø3.4 holes. Put it flat on the tongue and screw on with 4 × M3 × 10;
     reach the heads from the side with a ball-end hex key. Then join it to the base.
6. **Calibrate the servo:** with the tips just touching, read the servo position and set it as `ServoMap.zero`. Open the
   jaws a little: if the reading drops, set `ServoMap.sign = -1`.
