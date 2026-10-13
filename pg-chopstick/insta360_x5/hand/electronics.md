# Electronics: Insta360 X5 · Hand

The X5 records video and its own IMU and is started by voice. The PG pod logs a camera-rigid IMU for syncing.

```
 1S LiPo 300 mAh ── slide switch ── XIAO ESP32-S3 BAT+ / BAT−   (on-board charger, USB-C in the back wall)
                                     ├── I²C  D4 SDA / D5 SCL ── IMU breakout (3V3, GND), on the pod's top wall
                                     ├── D2 ── palm button (on the thumb strap) ── GND   (pull-up, pressed = low):
                                     │         START / STOP
                                     └── Wi-Fi / BLE ── laptop or phone, or log to flash
```

| Part | Notes |
|---|---|
| Seeed XIAO ESP32-S3 | USB-C faces the slot in the back wall |
| IMU breakout (ICM-42688-P, LSM6DSOX or BNO085 class) | Flat under the pod's top wall, rigid to the camera |
| 1S LiPo 402535, 300 mAh, protected | On foam tape, above the saddle |
| Slide switch SS12D00 | In the battery lead |
| Tactile switch 6 × 6 mm, soft (about 160 gf) + 2-core wire, about 50 cm | In `palm_button`. The wire runs round the outer edge of the hand on the thumb strap, back along it, and into the pod through the front-wall hole on the little-finger side |

**Palm button.** It's on D2, like the finger-sleeve button on the grippers, so the same firmware works: start and stop
the X5 by voice (or its own button) and mark the take with the palm button, or have the pod's firmware trigger the X5
over Wi-Fi. You only press it on purpose, with a curl of the middle and ring fingers, so a soft switch is fine; still
count only short presses (under about 0.6 s), in case a grasp squeezes it.
