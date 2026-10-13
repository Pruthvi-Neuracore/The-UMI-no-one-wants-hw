# Electronics: one USB cable

Everything on the device goes through a small USB 3 hub inside the electronics box, so one USB-C cable runs to the laptop.

```
 D405 depth camera ──USB 3──┐
                            ├── USB 3 hub (in the box) ══ one USB-C cable ══ laptop
 Raspberry Pi Pico 2 ──USB──┘
   ├── I2C ── IMU (BNO085-class, in the box floor, rigid to the frame)
   ├── GPIO ── record button (index sleeve: press to start, press again to stop)
   └── UART (half-duplex) ── STS3215 servo bus (gripper width encoder)
```

The Pico can't relay the camera: its USB port is full-speed (12 Mbit/s), and the D405 needs USB 3 for depth plus colour
(USB 2 at 480 Mbit/s only with reduced modes). So the camera goes straight to the hub, and the Pico streams the
IMU and gripper-width data. Both arrive on the laptop over the same cable and are synchronised there by timestamp.

## Parts

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 | On four M2 standoffs under the box lid (holes 47 × 11.4 mm) |
| BNO085 IMU breakout | In the box floor pocket (28 × 24 mm). **Check your board's outline**; secure it with double-sided tape or epoxy so it can't move |
| USB 3 hub board, USB-C upstream | Bay is 38 × 42 mm, up to ~14 mm tall (VL817-type 4-port boards). **Check your board's size** before printing |
| Short cables | D405 USB-C to USB-A (~15 cm, USB 3), Pico micro-USB to USB-A (~10 cm, right-angle helps) |
| 6 × 6 mm tactile switch, stiff (~250–320 gf) | Record start / stop. In the inner wall of the index sleeve under the domed TPU cover; legs and solder joints sit in the cavity behind it |
| 2-core wire, ~30 cm, flexible | Button to the Pico, with slack for the index link's travel |
| 74LVC1G125 or 1 kΩ resistor | Joins Pico TX/RX onto the servo's single-wire bus |
| 2× M3×8 + 2 M3 heat-set inserts | Box to arm, from the hand side (counterbored, heads sit flush) |
| 4× M3×6 + 4 M3 heat-set inserts | Lid to box |
| 4× M2×4 self-tapping screws | Pico to the lid standoffs |

## Wiring

| From | To |
|---|---|
| IMU SDA / SCL | Pico GP4 / GP5 (I2C0) |
| IMU VIN / GND | Pico 3V3(OUT) / GND |
| Servo bus data | Pico GP0 (TX) through the buffer or 1 kΩ, and GP1 (RX) direct |
| Servo V+ / GND | Hub 5 V / GND. The servo is only read as an encoder (torque off), so 5 V is enough |
| Record button | Pico GP15 and GND (use the internal pull-up; pressed = low) |
| Pico USB | Hub downstream port |
| D405 USB | Hub downstream port (USB 3) |

## Power

Everything runs from the laptop port. A USB 3 port gives 900 mA, which covers the D405, the Pico, the IMU and an idle
servo. If the camera drops out, use a USB-C port that supplies more current, or a powered hub.

## Box openings

- **Camera side:** a slot for the D405 cable (the plug passes through) and a slot shared by the servo and button cables.
- **Side wall:** the hub's USB-C upstream port.
- **Lid:** vent slots.
