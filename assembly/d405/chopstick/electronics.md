# Electronics: D405 + Quest · Chopstick

One USB-C cable from the electronics box to the laptop carries the D405 and the Pico. The Quest 3S controller is
self-powered (AA cell) and is tracked by the headset; it has no wires and its buttons are not used.

```
 D405 depth camera ──USB 3 (out of the cup's top window, into the box's top slot)──┐
                                                                                   ├── USB 3 hub ══ USB-C ══ laptop
 Raspberry Pi Pico 2 ──USB─────────────────────────────────────────────────────────┘
   ├── I²C GP4 SDA / GP5 SCL ── IMU breakout (BNO085 class), flat under the box's top wall
   ├── GP15 ── index-finger sleeve button ── GND   (internal pull-up, pressed = low)
   │           right-hand gripper: START / STOP;  left-hand gripper: CANCEL
   └── (GP14 free: a second button if you want one)

 Meta Quest 3S controller in the cradle ── tracked by the headset (pose only; buttons unused)
```

| Part | Notes |
|---|---|
| USB 3 hub board, USB-C upstream (VL817-type, 4 port) | Stands against the box's front wall (bay 42 × 38 mm, ≤ 14 mm thick). The upstream USB-C leaves through the slot low in the back wall. **Check your board's size** |
| Raspberry Pi Pico 2 | On the box's back wall (M2 standoffs or foam tape), micro-USB to the hub with a short right-angle cable |
| IMU breakout, BNO085 class | Flat under the top wall, rigid to the head |
| D405 cable, USB-C to USB-A, USB 3, about 15 cm | From the cup's top window down into the box's top slot |
| Tactile switch 6 × 6 mm, 250–320 gf + 2-core wire | From the index jaw's pad channel, along the jaw (with a slack loop at the pivot), into the box through the side hole on that side |

**Buttons.** A gripper has one button, under its index finger. Tell the host which hand each box is on: the right
hand's starts and stops a take, and the left hand's discards it. The wall that holds the button is the one you push to
grasp, so count only short presses (under about 0.6 s).

**Power.** Everything runs from the laptop's USB 3 port (900 mA covers the D405, the Pico and the IMU). If the camera
drops out, use a port that gives more current or a powered hub.

**Robot twin.** The D405 plugs straight into the robot PC (USB 3). The Feetech SM40BL runs on its bus through a USB
adapter at 12 V. Its rotor plate is on `jaw_left_sa`, and its zero is read with the tips touching.
