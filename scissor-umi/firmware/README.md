# Firmware: the jaw opening from the STS3215

The Pico 2 in the electronics box reads the jaw opening from the Feetech STS3215 on the scissor axle and streams it
to the PC over the same USB cable as the D405 (through the hub).

1. Flash MicroPython for the Pico 2 (RP2350) and copy [`pico/main.py`](pico/main.py) to it as `main.py`.
2. Calibrate once: `python tools/read_gripper.py --calibrate`. It asks for the open stop, then the pads touching.
   The values are saved on the Pico (`calib.json`).
3. Read it: `python tools/read_gripper.py`, or open the serial port in your logger. You get one JSON line per sample
   (100 Hz by default):

```
{"t": 123456, "raw": 2731, "q": 0.412, "w": 61.3, "rec": 1}
```

* **q** is the robot gripper's `finger_joint` (0 open … 0.7 closed). Send it straight to the 2F-140, or convert it
  with its driver's position scale.
* **w** is the distance between the pad faces in mm, from Robotiq's URDF.
* **rec** is the record flag, toggled by the button under the base.

The servo stays at torque off (the firmware turns it off at start-up). It is used only for its encoder. Handle A
turns half as far as the jaws, so the reading spans about 228 of the servo's 4096 counts per turn over the stroke.
The two-point calibration takes care of the ratio and the direction.
