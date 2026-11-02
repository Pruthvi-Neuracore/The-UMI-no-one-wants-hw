# Scissor-UMI · 2F-140: electronics (handheld)

This is the same as HandUMI ([`handumi/docs/electronics.md`](../../handumi/docs/electronics.md)): one USB-C cable to
the PC.

```
 D405 ──USB 3──┐
               ├── USB 3 hub (in the box) ══ one USB-C cable ══ PC
 Pico 2 ──USB──┘
   ├── UART (half-duplex) ── STS3215 on the scissor axle (jaw encoder, torque off)
   ├── I2C ── IMU (BNO085-class)
   └── GP15 ── record button (12 × 12 tactile, green) ── GND, internal pull-up
 Quest Touch Pro: its own battery and tracking, paired with the headset (buttons unused)
```

| From | To |
|---|---|
| Servo bus data | Pico GP0 (TX) through the buffer or 1 kΩ, and GP1 (RX) direct |
| Servo V+ / GND | Hub 5 V / GND. It is only read, so 5 V is enough |
| IMU SDA / SCL, VIN / GND | Pico GP4 / GP5, 3V3(OUT) / GND |
| Button | GP15 and GND |

* **Reading q.** The servo turns with handle A, which turns half as far as the jaws: q = 2 × (servo angle − its
  angle at the open stop). Calibrate at the open stop (q = 0) and with the pads touching (q = 0.7).
* **The robot needs no electronics here.** The 2F-140 reports its own position and the D405 plugs into the robot PC.
