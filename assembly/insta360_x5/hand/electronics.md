# Electronics: Insta360 X5 · Hand

The X5 records video and its own IMU and is started by voice. The PG pod logs a camera-rigid IMU for syncing.

```
 1S LiPo 300 mAh ── slide switch ── XIAO ESP32-S3 BAT+ / BAT−   (on-board charger, USB-C in the back wall)
                                     ├── I²C  D4 SDA / D5 SCL ── IMU breakout (3V3, GND), on the pod's top wall
                                     └── Wi-Fi / BLE ── laptop or phone, or log to flash
```

| Part | Notes |
|---|---|
| Seeed XIAO ESP32-S3 | USB-C faces the slot in the back wall |
| IMU breakout (ICM-42688-P, LSM6DSOX or BNO085 class) | Flat under the pod's top wall, rigid to the camera |
| 1S LiPo 402535, 300 mAh, protected | On foam tape, above the saddle |
| Slide switch SS12D00 | In the battery lead |

There is no finger sleeve, so there is no record button. The pod's two front-wall wire holes are free if you want to
add a button on a fingertip strap; wire it to D2 (start / stop) like the grippers.
