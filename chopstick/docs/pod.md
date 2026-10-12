# PG pod: electronics

A wireless sensor pod under the handheld gripper's X5. It logs a camera-rigid IMU and the record button, and it
charges over USB-C.

```
 1S LiPo 300 mAh ── slide switch ── XIAO ESP32-S3 BAT+/BAT− (on-board charger, USB-C on the back wall)
                                     ├── I²C (D4 SDA / D5 SCL) ── IMU breakout (3V3, GND)
                                     ├── D2 ── right-jaw sleeve button ── GND : START / STOP  (right hand's index finger)
                                     ├── D1 ── left-jaw sleeve button  ── GND : CANCEL        (left hand's index finger)
                                     │         (internal pull-ups, pressed = low; wire whichever is fitted)
                                     ├── D0 ── spare button on the pod's front ── GND
                                     └── Wi-Fi / BLE ── laptop or phone (stream), or log to flash
```

## Layout inside the pod

The pod's inside is about 30 × 49 × 22 mm.

| Where | What |
|---|---|
| Top wall, underside, centre-rear | **IMU**, taped or epoxied flat. The top wall is clamped to the camera by the 1/4"-20 screw, so the IMU moves with the X5 |
| Front wall | **Spare button**: the TPU cap fits through the Ø11 hole, and the 6 × 6 switch sits in the seat on the post behind it. **Two Ø3.5 holes** near the top corners take the sleeve-button wires from the jaws |
| Back wall, low | **XIAO ESP32-S3**, USB-C facing out through the 10 × 4.6 mm slot. The Ø2 hole beside it is a light pipe for the charge LED |
| −Y wall, low | **Slide switch** in the 9 × 4 mm slot |
| Floor (on the lid) | **LiPo** on foam tape |

Fit the 1/4"-20 × 5/8" camera screw from inside the pod before the electronics go in.

## Syncing with the camera

The X5 records its own IMU. Because the pod's IMU is rigid to the camera, cross-correlating the two gyro streams
gives the clock offset, so no wire or flash is needed. A sharp tap on the gripper at the start of each take makes the
peak easy to find. Button presses are logged with the pod's timestamps and land on the video timeline through the
same offset.

## Firmware

Not included yet. A minimal version:

- reads the IMU at 200–400 Hz,
- debounces the buttons,
- treats a sleeve press shorter than about 0.6 s as a deliberate press (a grasp holds the wall far longer),
- maps the right-jaw button (D2) to start/stop and the left-jaw button (D1) to cancel, so the same firmware runs on
  both hands' pods; the right hand's index finger lands on the right jaw and the left hand's on the left jaw,
- sends button events to one host, which owns the session: start/stop from the right pod opens and closes a take,
  and cancel from the left pod discards the open take on both hands,
- streams everything over UDP or BLE, or logs it to flash with timestamps.
