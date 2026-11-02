# Scissor-UMI · 2F-140: electronics (handheld)

```
 D405 ──USB-C──┐
               ├── USB 3 hub ──USB-C── PC
 Pico 2 ──USB──┘
   ├─ SPI ── AS5047P (under the handle axle, magnet in handle A's hub)   CSn GP17, CLK GP18, MOSI GP19, MISO GP16
   └─ GP15 ── record button (12 × 12 tactile, green) ── GND, internal pull-up
 Quest Touch Pro: its own battery and tracking; paired with the headset (buttons unused)
```

* q, the robot's `finger_joint`, is 2 × the handle angle: the knuckle gears have 20 teeth and the handle sectors 40.
  Calibrate it with the jaws closed (pads touching, q = 0.7) and against the open stop (q = 0).
* The robot twin needs no electronics: the 2F-140 reports its own position, the D405 plugs into the robot PC, and the
  Quest controller tracks itself.
