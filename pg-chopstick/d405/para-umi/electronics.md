# Electronics: D405 + Quest · Para-UMI

This is HandUMI's electronics, unchanged. Everything on the device goes through a small USB 3 hub in the electronics
box, so one USB-C cable runs to the laptop. The Quest controller in the controller support is self-powered and is used
for tracking only: it has no wires, and its buttons are not used.

```
 D405 depth camera ──USB 3──┐
                            ├── USB 3 hub (in the box) ══ one USB-C cable ══ laptop
 Raspberry Pi Pico 2 ──USB──┘
   ├── I2C ── IMU (BNO085-class, in the box floor, rigid to the frame)
   ├── GPIO ── index-sleeve button: start / stop (right hand) or cancel (left hand)
   └── UART (half-duplex) ── STS3215 servo bus (gripper width encoder)
```

The Pico can't relay the camera: its USB port is full speed (12 Mbit/s), and the D405 needs USB 3 for depth plus colour.
So the camera goes straight to the hub, and the Pico streams the IMU, the button and the gripper width.

## Wiring

| From | To |
|---|---|
| IMU SDA / SCL | Pico GP4 / GP5 (I2C0) |
| IMU VIN / GND | Pico 3V3(OUT) / GND |
| Servo bus data | Pico GP0 (TX) through a 74LVC1G125 buffer or 1 kΩ, and GP1 (RX) direct |
| Servo V+ / GND | Hub 5 V / GND (read as an encoder, torque off) |
| Button | Pico GP15 and GND (internal pull-up; pressed = low) |
| Pico USB, D405 USB | Hub downstream ports (the D405 on USB 3) |

**Power.** Everything runs from the laptop port. A USB 3 port gives 900 mA, which covers the D405, the Pico, the IMU and
an idle servo. If the camera drops out, use a port that supplies more current, or a powered hub.

**Button.** The button wall is the one you push to grasp, so count only short presses (under about 0.6 s).
