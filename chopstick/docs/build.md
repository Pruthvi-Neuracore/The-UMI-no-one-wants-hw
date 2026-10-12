# Build

The step-by-step photos are in KIWI's
[assembly guide](https://github.com/TToTMooN/KIWI_hardware/blob/main/chopsticks/chopsticks_assembly.pdf). This page is
the short version, plus the OpenArm adapter steps.

## Print

Open `hardware/print/KIWI-3dprint.3mf` in Bambu Studio. It holds four plates (H2D, 0.4 mm nozzle, 0.20 mm Standard):

- **DC** and **SA** plates: the bases, clips, X5 frames and button pieces, in black PLA Matte.
- **ARM** plate: the jaws, in black PLA Matte with the red dots painted in a second filament. It holds one data-gripper
  pair and one robot-gripper pair, so print it once per gripper.
- **Finger** plate: the four TPU tips, in TPU 95A.

On a single-extruder printer the jaws print in one colour. Mark the dots by hand with a red paint pen instead; their
positions are in `chopstick_kinematics.json` and in `docs/img/hero.png`.

Print `hardware/STL/openarm_adapter.stl` on its own: back face down, 4 walls, PETG or PLA.

## Data gripper

1. **Inserts:** two M2 × 4 heat-set inserts in the back of the base, with M2 × 4 screws in them for positioning, and two
   M3 × 4 inserts from the underside.
2. **Jaws:** thread a finger strap through each jaw to make a loop. Glue the TPU tip onto the jaw and press two 624
   bearings into each jaw.
3. **Strap:** pin the wrist strap under the base.
4. **Join:** hook the rubber band through the centre cavity onto both jaws. Mesh the gears with the jaws closed (tips
   touching) and fit the two M3 shoulder screws. The jaws must swing freely. If they bind, the print's preload needs
   adjusting for your printer.
5. **Foam:** stick foam under the base and jaws.
6. **Camera:** assemble the button in the frame (M2 × 4), fix the X5 with the 1/4"-20 screw, then join the frame to the
   base with the clip, the M3 × 55 screw and a nut.
7. **Tag and grip:** stick the ArUco tag on, then add grip tape where you hold it.

## Robot gripper on OpenArm

1. **Inserts:** four M2 × 4 in the back of the base and one M3 × 4 in the underside.
2. **Tips:** glue the TPU tips onto both jaws.
3. **Servo horn:** take the rotor plate off the SM40BL and screw it to `jaw_left_sa` with 4 × M3 × 12.
4. **Drive side:** push the driver jaw into the base and line up the joint axis. Slide the servo in from below. The M3
   shoulder screw goes through the body into the servo's coupling plate. Fix the servo with its own screws.
5. **Driven jaw:** press two 624 bearings into `jaw_right`. Mesh the gears **with both jaws closed**, then fit the second
   shoulder screw.
6. **Camera frame:** fit the button and the X5 in `x5_frame`. Join the frame to the base with clip A, clip B, the
   M3 × 55 screws and nuts.
7. **Adapter:** melt four M3 heat-set inserts into the adapter's front holes. Bolt the adapter to link7 with 6 × M3 × 16
   through the counterbores. The boss goes in the back pocket. The tongue's long edges run along link7 X, so the jaws
   will open along link7 Y.
8. **Frame onto the adapter:** drill the frame's four rear-wall holes out to Ø3.2. With the camera out, fit the tongue
   between the frame's two rear legs and screw the frame on from inside with 4 × M3 × 10. Put the camera back.
9. **Calibrate the servo:** close the jaws until the tips just touch and read the servo position. That reading is
   `ServoMap.zero`. Open the jaws a little: if the reading went down, set `ServoMap.sign = -1`.
