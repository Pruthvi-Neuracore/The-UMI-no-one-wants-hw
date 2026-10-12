# Electronics: Insta360 X5 · Parallel

The camera is the X5 itself: it records video and its own IMU, and starts and stops by voice (it has no button on the gripper) ("start recording" /
"stop recording"). The rest is the PG pod under the camera, plus a servo on the robot twin.

## PG pod (handheld)

```
 1S LiPo 300 mAh ── slide switch ── XIAO ESP32-S3 BAT+ / BAT−   (on-board charger, USB-C in the back wall)
                                     ├── I²C  D4 SDA / D5 SCL ── IMU breakout (3V3, GND), on the pod's top wall
                                     ├── D2 ── right-finger sleeve button ── GND : START / STOP  (right-hand gripper)
                                     ├── D1 ── left-finger sleeve button  ── GND : CANCEL        (left-hand gripper)
                                     │         internal pull-ups, pressed = low; wire the one your gripper has
                                     └── Wi-Fi / BLE ── laptop or phone, or log to flash
```

| Part | Notes |
|---|---|
| Seeed XIAO ESP32-S3 | 21 × 17.5 mm; USB-C faces the 10 × 4.6 mm slot in the back wall; its charge LED shows through the Ø2 window |
| IMU breakout (ICM-42688-P, LSM6DSOX or BNO085 class) | I²C, ≤ 26 × 18 mm; tape or epoxy it flat under the top wall, so it is rigid to the camera |
| 1S LiPo 402535, 300 mAh, protected | On foam tape on the lid |
| Slide switch SS12D00 | In the battery lead, in the slot on the −Y wall |
| Tactile switch 6 × 6 mm, 250–320 gf | In the index finger's pad, under the TPU cover |
| 2-core silicone wire, about 25 cm | From the pad's wire channel, along the jaw's underside (with a slack loop at the pivot), into the pod through the front-wall hole on that side |

**Buttons.** The right hand's index finger lands on the right finger and the left hand's on the left finger, so the role
follows the finger: D2 (right finger) toggles recording and D1 (left finger) cancels. Both pods run the same firmware and report
to one host, so a cancel from the left hand discards the take the right hand started.

The wall that holds the button is the one you push to grasp. Count only short presses (under about 0.6 s); a grasp holds
the wall much longer.

**Sync.** The pod's IMU is clamped to the camera, so its gyro and the X5's gyro can be cross-correlated to put the
pod's timestamps (button presses) on the video's clock. A sharp tap at the start of each take makes the peak easy to
find.

## Robot twin

| Part | Wiring |
|---|---|
| Feetech SM40BL | Feetech bus to the robot PC through a Feetech USB adapter (URT-1 or similar), 12 V supply. Under the housing, its rotor plate is screwed to `parallel_pinion_servo`: 4096 steps per turn, and 1 mm of finger travel is 1/15 rad of pinion. Read its position with the pads touching as the zero |
| Insta360 X5 | Optional on the robot. Keep it in place anyway: it fixes the camera geometry, so the robot sees what the handheld saw |
