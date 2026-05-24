#!/usr/bin/env python3
"""soarm_leader.py — read SO-101 joint positions, emit JSON lines for follower_play.

The SO-101 driver board ships a custom MCU firmware that emits ASCII lines:
    [POS] <p1> <p2> <p3> <p4> <p5> <p6>\\n
where each p is a raw STS3215 servo position (0..4095, ≈ 360° / 4096 per step).
Serial port: 1 Mbaud, DTR=True, RTS=False is what triggers the stream.

This script:
  1. Opens the SO-101 serial port
  2. Captures the first valid [POS] line as the zero pose
  3. Streams deltas (in degrees, optionally scaled and sign-flipped) as JSON
     lines matching the wire format that `leader_stream` / `follower_play` use

Pipe into follower_play to drive Bruce (or any PiPER-X follower).

Usage:
    .venv/bin/python soarm_leader.py
    .venv/bin/python soarm_leader.py --port /dev/cu.usbserial-110 --scale 0.5
    .venv/bin/python soarm_leader.py --human   # debug: print degrees instead of JSON
"""

import argparse
import json
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial not found. Activate the venv: source .venv/bin/activate\n")
    sys.exit(1)

RAW_TO_DEG = 360.0 / 4096.0


def parse_pos_line(line):
    """Parse '[POS] 974 684 960 1585 1018 2083' → [974, 684, 960, 1585, 1018, 2083]."""
    s = line.decode('ascii', errors='replace').strip()
    if not s.startswith('[POS]'):
        return None
    parts = s[5:].split()
    try:
        vals = [int(p) for p in parts]
    except ValueError:
        return None
    if len(vals) != 6:
        return None
    return vals


def parse_signs(s):
    items = [int(x.strip()) for x in s.split(',')]
    if len(items) != 6:
        raise argparse.ArgumentTypeError(f'--signs needs 6 entries, got {len(items)}')
    for it in items:
        if it not in (-1, 1):
            raise argparse.ArgumentTypeError('--signs entries must be -1 or +1')
    return items


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/cu.usbserial-110')
    ap.add_argument('--baud', type=int, default=1_000_000)
    ap.add_argument('--scale', type=float, default=0.5,
                    help='Multiplier on every delta before emit (0.5 = follower moves half as much)')
    ap.add_argument('--signs', default='1,1,1,1,1,1',
                    help='Per-joint sign (+1 or -1) to flip mismatched axes')
    ap.add_argument('--human', action='store_true',
                    help='Print human-readable angles to stdout instead of JSON')
    ap.add_argument('--max-emit-hz', type=float, default=50.0,
                    help='Cap output rate. If the board emits slower, we emit slower.')
    args = ap.parse_args()

    signs = parse_signs(args.signs)
    period = 1.0 / args.max_emit_hz

    sys.stderr.write(
        f'soarm_leader — port={args.port} baud={args.baud} scale={args.scale} signs={signs}\n'
    )
    ser = serial.Serial(args.port, args.baud, timeout=0.5,
                        bytesize=8, parity='N', stopbits=1)
    ser.dtr = True
    ser.rts = False
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Capture zero pose
    sys.stderr.write('capturing zero pose…\n')
    zero = None
    for _ in range(20):
        line = ser.readline()
        if not line:
            continue
        vals = parse_pos_line(line)
        if vals is not None:
            zero = vals
            break
    if zero is None:
        sys.stderr.write('Could not read a [POS] line within 10 s. Aborting.\n')
        sys.exit(3)
    sys.stderr.write(f'  zero (raw): {zero}\n')
    sys.stderr.write('streaming. Ctrl+C to stop.\n\n')

    last_emit = 0.0
    last_good_deg = [0.0] * 6

    try:
        while True:
            line = ser.readline()
            if not line:
                # No new sample. Keep emitting last good at the cap rate so the
                # follower doesn't hit its stdin watchdog.
                now = time.monotonic()
                if now - last_emit >= period:
                    emit(args.human, last_good_deg)
                    last_emit = now
                continue

            vals = parse_pos_line(line)
            if vals is None:
                continue

            joints_deg = [(v - z) * RAW_TO_DEG * args.scale * sign
                          for v, z, sign in zip(vals, zero, signs)]
            last_good_deg = joints_deg

            now = time.monotonic()
            if now - last_emit >= period:
                emit(args.human, joints_deg)
                last_emit = now
    except KeyboardInterrupt:
        sys.stderr.write('\nstopped.\n')


def emit(human, joints_deg):
    t_us = int(time.time() * 1_000_000)
    if human:
        sys.stdout.write(f"[{t_us}] " +
                         ' '.join(f'J{i+1}={d:+7.2f}°' for i, d in enumerate(joints_deg))
                         + '\n')
    else:
        sys.stdout.write(json.dumps({'t_us': t_us, 'joints_deg': joints_deg}) + '\n')
    sys.stdout.flush()


if __name__ == '__main__':
    main()
