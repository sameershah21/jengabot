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
    ap.add_argument('--scale', type=float, default=0.6,
                    help='Multiplier on every joint delta before emit. 0.6 worked well in '
                         'live SO-101 → Bruce testing — bigger SO-101 motion = visibly bigger '
                         'Bruce motion without overwhelming the follower step clamp.')
    ap.add_argument('--signs', default='1,1,-1,1,1,1',
                    help='Per-joint sign (+1 or -1). 6 entries for J1..J5 + gripper. '
                         'Default flips J3 because PiPER J3 range is [-175°, 0°] (negative '
                         'only) — without the flip, half of SO-101 J3 motion lands above 0 '
                         'and gets silently clamped to zero on Bruce.')
    ap.add_argument('--human', action='store_true',
                    help='Print human-readable angles to stdout instead of JSON')
    ap.add_argument('--max-emit-hz', type=float, default=50.0,
                    help='Cap output rate. If the board emits slower, we emit slower.')
    # Gripper-specific
    ap.add_argument('--gripper-scale', type=float, default=5.0,
                    help='Multiplier on the gripper raw-delta before normalizing to a 0..1 step. '
                         'Bigger = Bruce moves more per SO-101 squeeze. 5.0 came from live '
                         'testing where 3.0 was too gentle. In follower_play --incremental '
                         'mode this becomes an additive delta on Bruce\'s seed gripper position.')
    ap.add_argument('--gripper-invert', action='store_true',
                    help='Flip sign of the gripper delta (if Bruce opens when SO-101 closes).')
    ap.add_argument('--gripper-deadband', type=float, default=0.05,
                    help='Suppress the gripper field when |delta| < this. Prevents Bruce '
                         'from being commanded at startup (when SO-101 is at rest, delta≈0). '
                         'Bruce will hold whatever physical position it was in until you '
                         'actually squeeze SO-101 past the deadband.')
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
    # Gripper output is now a DELTA in [-∞..∞], typically ≈ [-1..1] for normal
    # squeeze range. follower_play in --incremental mode adds this delta to
    # Bruce's seed gripper position, so Bruce starts wherever it already is
    # rather than snapping to "full open" or any fixed value.
    last_good_gripper = 0.0

    grip_sign = -1.0 if args.gripper_invert else 1.0

    try:
        while True:
            line = ser.readline()
            if not line:
                # No new sample. Keep emitting last good at the cap rate so
                # follower doesn't hit its stdin watchdog.
                now = time.monotonic()
                if now - last_emit >= period:
                    grip_for_emit = (
                        last_good_gripper
                        if (not args.no_gripper)
                           and abs(last_good_gripper) >= args.gripper_deadband
                        else None
                    )
                    emit(args.human, last_good_deg, grip_for_emit, args.no_gripper)
                    last_emit = now
                continue

            vals = parse_pos_line(line)
            if vals is None:
                continue

            if args.no_gripper:
                # 6 joint deltas, no gripper (legacy behavior)
                joints_deg = [(v - z) * RAW_TO_DEG * args.scale * sign
                              for v, z, sign in zip(vals, zero, signs)]
                gripper_delta = 0.0
            else:
                # Joint remapping (observed live + operator-requested):
                #
                # SO-101 servo index  →  Bruce joint index
                #   0 (J1)            →  0 (J1)
                #   1 (J2)            →  1 (J2)
                #   2 (J3)            →  2 (J3)
                #   3 (J4)            →  4 (J5)
                #   4 (J5)            →  5 (J6)
                # Bruce J4 (idx 3) holds at seed — no SO-101 source.
                def delta(i):
                    return (vals[i] - zero[i]) * RAW_TO_DEG * args.scale * signs[i]
                joints_deg = [
                    delta(0),   # Bruce J1
                    delta(1),   # Bruce J2
                    delta(2),   # Bruce J3
                    0.0,        # Bruce J4 — hold (no leader source)
                    delta(3),   # Bruce J5 ← SO-101 J4
                    delta(4),   # Bruce J6 ← SO-101 J5
                ]
                # 6th SO-101 servo = gripper. Emit RAW DELTA from zero,
                # normalized so a ~1000-raw squeeze ≈ 1.0 with default
                # scale=3.0. follower_play in --incremental will add this
                # to Bruce's seed gripper position.
                raw_delta = (vals[5] - zero[5]) / 1000.0
                gripper_delta = raw_delta * args.gripper_scale * grip_sign

            last_good_deg = joints_deg
            last_good_gripper = gripper_delta

            now = time.monotonic()
            if now - last_emit >= period:
                # Suppress the gripper field when |delta| is below the
                # deadband — keeps Bruce's gripper untouched at startup
                # (and any time SO-101 isn't being squeezed). Bruce will
                # hold whatever physical position it's in.
                grip_for_emit = (
                    gripper_delta
                    if (not args.no_gripper)
                       and abs(gripper_delta) >= args.gripper_deadband
                    else None
                )
                emit(args.human, joints_deg, grip_for_emit, args.no_gripper)
                last_emit = now
    except KeyboardInterrupt:
        sys.stderr.write('\nstopped.\n')


def emit(human, joints_deg, gripper, no_gripper):
    """gripper: None → omit the field; float → include."""
    t_us = int(time.time() * 1_000_000)
    if human:
        line = f"[{t_us}] " + ' '.join(f'J{i+1}={d:+7.2f}°' for i, d in enumerate(joints_deg))
        if gripper is not None:
            line += f"   grip={gripper:+.2f}"
        elif not no_gripper:
            line += "   grip= idle"
        sys.stdout.write(line + '\n')
    else:
        obj = {'t_us': t_us, 'joints_deg': joints_deg}
        if gripper is not None:
            obj['gripper'] = gripper
        sys.stdout.write(json.dumps(obj) + '\n')
    sys.stdout.flush()


if __name__ == '__main__':
    main()
