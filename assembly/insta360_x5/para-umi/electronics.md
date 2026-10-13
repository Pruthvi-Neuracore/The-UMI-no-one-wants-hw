# Electronics: Insta360 X5 · Para-UMI

This is HandUMI's electronics box, unchanged (Pico 2, IMU and USB hub, one USB-C cable to the laptop). The only
difference is the camera: the X5 records 360° video and its own IMU onto its own card, so it isn't wired to the box.

```
 Insta360 X5 ── records video + IMU on its card (start by voice or its own button)

 Raspberry Pi Pico 2 ──USB── hub (in the box) ══ one USB-C cable ══ laptop
   ├── I2C ── IMU (BNO085-class, in the box floor, rigid to the frame)
   ├── GPIO ── index-sleeve button: start / stop (right hand) or cancel (left hand)
   └── UART (half-duplex) ── STS3215 servo bus (gripper width encoder)
```

## Wiring

| From | To |
|---|---|
| IMU SDA / SCL | Pico GP4 / GP5 (I2C0) |
| IMU VIN / GND | Pico 3V3(OUT) / GND |
| Servo bus data | Pico GP0 (TX) through the buffer or 1 kΩ, and GP1 (RX) direct |
| Servo V+ / GND | Hub 5 V / GND (read as an encoder, torque off) |
| Button | Pico GP15 and GND (internal pull-up; pressed = low) |
| Pico USB | Hub downstream port |

**Button roles.** The two hands use the same parts and wiring. The firmware gives the right hand's button
start / stop, and the left hand's button cancel. The button wall is the one you push to grasp, so count only short
presses (under about 0.6 s).

**Sync.** The box's IMU is rigid to the main support, and so is the X5 on its hinge (once the bolt is tight).
Cross-correlate the box's gyro with the X5's gyro to put button presses and the width encoder on the video's clock. A
sharp tap at the start of each take makes the peak easy to find.

**Box openings.** HandUMI's D405 cable slot is unused here. The Pico and button cables share the other slot, and the
hub's USB-C port is in the side wall.
