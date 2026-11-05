"""Read the Scissor-UMI's jaw opening from its Pico over USB serial.

    python read_gripper.py                     # print q (rad), width (mm) and the record flag
    python read_gripper.py --calibrate         # guided: open stop, then pads touching
    python read_gripper.py --port /dev/ttyACM0

Needs pyserial. The Pico runs firmware/pico/main.py.
"""
import argparse
import json
import sys
import time

import serial
from serial.tools import list_ports


def find_port():
    for p in list_ports.comports():
        if p.vid == 0x2E8A:                    # Raspberry Pi (Pico / Pico 2)
            return p.device
    sys.exit("no Pico found; pass --port")


def lines(ser):
    while True:
        s = ser.readline().decode(errors="ignore").strip()
        if s.startswith("{"):
            try:
                yield json.loads(s)
            except ValueError:
                pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--calibrate", action="store_true")
    a = ap.parse_args()
    ser = serial.Serial(a.port or find_port(), 115200, timeout=1)
    if a.calibrate:
        for cmd, prompt in (("open", "Open the jaws fully (handles on the open stop), then press Enter"),
                            ("closed", "Close the jaws (pads touching), then press Enter")):
            input(prompt)
            ser.write((cmd + "\n").encode())
            time.sleep(0.2)
        for msg in lines(ser):
            if "calib" in msg:
                print("calibrated:", msg["calib"])
                break
        return
    for msg in lines(ser):
        if "q" in msg:
            print(f"q {msg['q']:.3f} rad   width {msg['w']:6.1f} mm   rec {msg['rec']}   raw {msg['raw']}", end="\r")
        else:
            print(f"raw {msg.get('raw')}  (not calibrated: run with --calibrate)", end="\r")


if __name__ == "__main__":
    main()
