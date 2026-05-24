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
                    help='Per-joint sign (+1 or -1) to flip mismatched axes (6 entries for J1..J5 + gripper)')
    ap.add_argument('--human', action='store_true',
                    help='Print human-readable angles to stdout instead of JSON')
    ap.add_argument('--max-emit-hz', type=float, default=50.0,
                    help='Cap output rate. If the board emits slower, we emit slower.')
    # Gripper-specific
    ap.add_argument('--gripper-closed', type=int, default=1500,
                    help='SO-101 gripper raw value when fully closed (probably 1500-2000). Tune by inspecting --human output.')
    ap.add_argument('--gripper-open', type=int, default=2800,
                    help='SO-101 gripper raw value when fully open (probably 2500-3000)')
    ap.add_argument('--no-gripper', action='store_true',
                    help='Omit the gripper field from JSON output (treat all 6 as joints, original behavior).')
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
    # joints_deg has 6 entries (J1..J5 + a zero-padded J6 so PiPER J6 stays at seed).
    last_good_deg = [0.0] * 6
    last_good_gripper = 0.5  # half-open default

    grip_span = max(1, args.gripper_open - args.gripper_closed)

    try:
        while True:
            line = ser.readline()
            if not line:
                # No new sample. Keep emitting last good at the cap rate so
                # follower doesn't hit its stdin watchdog.
                now = time.monotonic()
                if now - last_emit >= period:
                    emit(args.human, last_good_deg, last_good_gripper, args.no_gripper)
                    last_emit = now
                continue

            vals = parse_pos_line(line)
            if vals is None:
                continue

            if args.no_gripper:
                # 6 joint deltas, no gripper (legacy behavior)
                joints_deg = [(v - z) * RAW_TO_DEG * args.scale * sign
                              for v, z, sign in zip(vals, zero, signs)]
                gripper_norm = 0.5
            else:
                # First 5 SO-101 servos = J1..J5 → PiPER J1..J5 deltas.
                # PiPER J6 padded with 0 (no delta → seed pose holds).
                joints_deg = [
                    (vals[i] - zero[i]) * RAW_TO_DEG * args.scale * signs[i]
                    for i in range(5)
                ] + [0.0]
                # 6th SO-101 servo = gripper. Map raw → normalized [0..1].
                gripper_norm = (vals[5] - args.gripper_closed) / grip_span
                gripper_norm = max(0.0, min(1.0, gripper_norm))

            last_good_deg = joints_deg
            last_good_gripper = gripper_norm

            now = time.monotonic()
            if now - last_emit >= period:
                emit(args.human, joints_deg, gripper_norm, args.no_gripper)
                last_emit = now
    except KeyboardInterrupt:
        sys.stderr.write('\nstopped.\n')


def emit(human, joints_deg, gripper, no_gripper):
    t_us = int(time.time() * 1_000_000)
    if human:
        line = f"[{t_us}] " + ' '.join(f'J{i+1}={d:+7.2f}°' for i, d in enumerate(joints_deg))
        if not no_gripper:
            line += f"   grip={gripper:.2f}"
        sys.stdout.write(line + '\n')
    else:
        obj = {'t_us': t_us, 'joints_deg': joints_deg}
        if not no_gripper:
            obj['gripper'] = gripper
        sys.stdout.write(json.dumps(obj) + '\n')
    sys.stdout.flush()


if __name__ == '__main__':
    main()
