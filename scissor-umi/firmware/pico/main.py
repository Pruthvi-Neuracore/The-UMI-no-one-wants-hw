"""Scissor-UMI · 2F-140 — Raspberry Pi Pico 2 firmware (MicroPython).

Reads the jaw opening from the Feetech STS3215 on the scissor axle (its built-in 12-bit magnetic encoder, torque off)
and the record button, and streams one JSON line per sample over USB serial:

    {"t": 123456, "raw": 2731, "q": 0.412, "w": 61.3, "rec": 1}

  t    Pico time, ms
  raw  STS3215 present position (0..4095, 4096 per turn)
  q    the 2F-140's finger_joint (rad): 0 open .. 0.7 closed, from the calibration below
  w    the distance between the pad faces (mm), from Robotiq's URDF (fit of kin.pad_gap)
  rec  1 while recording (the button toggles it)

Commands (one per line, from the PC):
  open     store the current reading as q = 0     (handles against the open stop)
  closed   store the current reading as q = 0.7   (pads touching)
  rate N   samples per second (default 100)

Wiring: GP0 (UART0 TX) through a 1 kΩ resistor (or a 74LVC1G125) and GP1 (RX) direct, both to the servo's data line;
servo V+ from the hub's 5 V; record button between GP15 and GND. Copy this file to the Pico as main.py.
"""
import json
import sys
import time

import select
from machine import UART, Pin

SERVO_ID = 1
BAUD = 1_000_000
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
Q_MAX = 0.7
W_POLY = (31.0586, -66.8619, -152.2784, 128.5506)          # pad gap (mm) vs q, cubic, < 0.01 mm from kin.pad_gap
CALIB_FILE = "calib.json"

uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), timeout=4, timeout_char=2)
button = Pin(15, Pin.IN, Pin.PULL_UP)


def _packet(instr, params):
    body = [SERVO_ID, len(params) + 2, instr] + list(params)
    return bytes([0xFF, 0xFF] + body + [(~sum(body)) & 0xFF])


def _transact(pkt, n_reply):
    """Half-duplex bus: our own bytes echo back first; drop them, then read the reply."""
    while uart.any():
        uart.read()
    uart.write(pkt)
    echo = uart.read(len(pkt))
    if echo is None or len(echo) != len(pkt):
        return None
    if not n_reply:
        return b""
    r = uart.read(n_reply)
    if r is None or len(r) != n_reply or r[0] != 0xFF or r[1] != 0xFF:
        return None
    if ((~sum(r[2:-1])) & 0xFF) != r[-1]:
        return None
    return r


def torque_off():
    _transact(_packet(0x03, [ADDR_TORQUE_ENABLE, 0]), 6)


def read_position():
    r = _transact(_packet(0x02, [ADDR_PRESENT_POSITION, 2]), 8)    # FF FF ID LEN ERR lo hi CHK
    if r is None:
        return None
    return r[5] | (r[6] << 8)


def load_calib():
    try:
        with open(CALIB_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"open": None, "closed": None}


def save_calib(c):
    with open(CALIB_FILE, "w") as f:
        json.dump(c, f)


def _delta(a, b):
    """b - a on a 4096-count circle, in -2048 .. 2047."""
    return (b - a + 2048) % 4096 - 2048


def to_q(raw, c):
    if c["open"] is None or c["closed"] is None or c["open"] == c["closed"]:
        return None
    return Q_MAX * _delta(c["open"], raw) / _delta(c["open"], c["closed"])


def to_w(q):
    a, b, cc, d = W_POLY
    return ((a * q + b) * q + cc) * q + d


def main():
    torque_off()
    calib = load_calib()
    rate = 100
    recording = 0
    last_btn = 1
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    line = ""
    next_t = time.ticks_ms()
    while True:
        # commands
        while poll.poll(0):
            ch = sys.stdin.read(1)
            if ch in "\r\n":
                cmd, line = line.strip().split(), ""
                raw = read_position()
                if cmd and cmd[0] in ("open", "closed") and raw is not None:
                    calib[cmd[0]] = raw
                    save_calib(calib)
                    print(json.dumps({"calib": calib}))
                elif cmd and cmd[0] == "rate" and len(cmd) > 1:
                    rate = max(1, min(500, int(cmd[1])))
            else:
                line += ch
        # button: toggle recording on each press
        b = button.value()
        if last_btn == 1 and b == 0:
            recording ^= 1
        last_btn = b
        # sample
        now = time.ticks_ms()
        if time.ticks_diff(now, next_t) >= 0:
            next_t = time.ticks_add(next_t, 1000 // rate)
            raw = read_position()
            q = to_q(raw, calib) if raw is not None else None
            out = {"t": now, "raw": raw, "rec": recording}
            if q is not None:
                q = min(max(q, 0.0), Q_MAX)
                out["q"] = round(q, 4)
                out["w"] = round(to_w(q), 2)
            print(json.dumps(out))


main()
