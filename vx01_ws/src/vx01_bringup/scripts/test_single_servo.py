#!/usr/bin/env python3
"""
test_single_servo.py
────────────────────
Run this BEFORE launching the full hardware stack.
It directly talks to the Maestro over serial and moves one servo at a time
so you can verify every channel number and direction without ROS running.

Usage:
  python3 test_single_servo.py --port /dev/ttyACM0 --channel 0
  python3 test_single_servo.py --port /dev/ttyACM0 --channel 0 --angle 30
  python3 test_single_servo.py --port /dev/ttyACM0 --sweep   (sweeps all 18 channels)

Requirements:
  pip3 install pyserial
"""

import argparse
import serial
import time
import struct


def microseconds_to_target(us: float) -> int:
    return int(us * 4)


def angle_to_target(angle_deg: float) -> int:
    # Linear map: -90° → 500µs, 0° → 1500µs, +90° → 2500µs
    us = 1500.0 + (angle_deg / 90.0) * 1000.0
    us = max(500.0, min(2500.0, us))
    return microseconds_to_target(us)


def set_target(ser: serial.Serial, channel: int, target: int):
    cmd = bytes([
        0x84,
        channel,
        target & 0x7F,
        (target >> 7) & 0x7F,
    ])
    ser.write(cmd)


def get_position(ser: serial.Serial, channel: int) -> int:
    ser.write(bytes([0x90, channel]))
    resp = ser.read(2)
    if len(resp) == 2:
        return resp[0] + (resp[1] << 8)
    return -1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',    default='/dev/ttyACM0')
    parser.add_argument('--channel', type=int, default=0,
                        help='Maestro channel to test (0–17)')
    parser.add_argument('--angle',   type=float, default=0.0,
                        help='Target angle in degrees (default 0 = centre)')
    parser.add_argument('--sweep',   action='store_true',
                        help='Sweep all 18 channels to centre one by one')
    args = parser.parse_args()

    print(f"Opening {args.port} at 115200 baud...")
    ser = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(0.1)
    print("Connected.\n")

    if args.sweep:
        print("Sweeping all channels to CENTRE (1500µs = 0°)")
        print("Watch which servo moves for each channel number.\n")
        for ch in range(18):
            t = microseconds_to_target(1500.0)
            set_target(ser, ch, t)
            pos = get_position(ser, ch)
            print(f"  Channel {ch:2d}: sent centre — position readback = {pos} ({pos/4.0:.0f}µs)")
            time.sleep(0.6)

    else:
        ch = args.channel
        angle = args.angle
        t = angle_to_target(angle)
        print(f"Moving channel {ch} to {angle:.1f}° (target={t}, {t/4.0:.0f}µs)")
        set_target(ser, ch, t)
        time.sleep(0.5)
        pos = get_position(ser, ch)
        print(f"Readback position: {pos} ({pos/4.0:.0f}µs)")

        print("\nPress Enter to return to centre, Ctrl+C to exit")
        input()
        set_target(ser, ch, microseconds_to_target(1500.0))
        print("Back to centre.")

    ser.close()


if __name__ == '__main__':
    main()