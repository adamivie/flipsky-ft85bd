"""
spin_test.py
============
Interactive spin test — ramp both motors to a target eRPM, hold, then stop.

⚠  Lift the robot off the ground before running.
⚠  ESC Tool → Input Signal Type must be OFF on both channels.

Usage
-----
    python spin_test.py --port COM16 --erpm 5000 --duration 5
    python spin_test.py --port COM16 --erpm 50000 --duration 10 --slave 184
"""

import argparse
import sys
import time

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',     required=True)
    ap.add_argument('--baud',     default=DEFAULT_BAUD, type=int)
    ap.add_argument('--slave',    default=184, type=int,
                    help='CAN id of slave channel (default 184)')
    ap.add_argument('--erpm',     default=2000, type=int,
                    help='Target eRPM (default 2000)')
    ap.add_argument('--duration', default=3.0,  type=float,
                    help='Run duration in seconds (default 3)')
    ap.add_argument('--master-only', action='store_true',
                    help='Only spin master, skip slave')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    print(f'\nConnected to {args.port} @ {args.baud} baud')
    print(f'Target: {args.erpm} eRPM  Duration: {args.duration}s')
    print('⚠  Lift robot off the ground first!')
    input('\nPress ENTER to start, Ctrl+C to abort...\n')

    t_end = time.monotonic() + args.duration
    try:
        while time.monotonic() < t_end:
            esc.set_speed( args.erpm)                               # master forward
            if not args.master_only:
                esc.set_speed(-args.erpm, can_id=args.slave)        # slave reverse
            esc.keep_alive()

            master = esc.get_telemetry()
            slave  = None if args.master_only else esc.get_telemetry(can_id=args.slave)

            if master: print(f'[MASTER] {master}')
            if slave:  print(f'[SLAVE ] {slave}')
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\nInterrupted')
    finally:
        print('Stopping...')
        esc.stop()
        if not args.master_only:
            esc.stop(can_id=args.slave)
        time.sleep(0.1)
        ser.close()
        print('Done.')


if __name__ == '__main__':
    main()
