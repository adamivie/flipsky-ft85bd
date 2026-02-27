"""
telemetry.py
============
Print live telemetry from both ESC channels at 2 Hz.

Usage
-----
    python telemetry.py --port COM16              # Windows
    python telemetry.py --port /dev/ttyACM0       # Linux
    python telemetry.py --port COM16 --slave 184  # custom slave CAN id
"""

import argparse
import sys
import time

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',  required=True)
    ap.add_argument('--baud',  default=DEFAULT_BAUD, type=int)
    ap.add_argument('--slave', default=184, type=int,
                    help='CAN id of the slave channel (default 184)')
    ap.add_argument('--n',     default=0, type=int,
                    help='Number of samples, 0 = run forever (default)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)
    print(f'Connected to {args.port} @ {args.baud} baud\n')

    count = 0
    try:
        while args.n == 0 or count < args.n:
            master = esc.get_telemetry()
            slave  = esc.get_telemetry(can_id=args.slave)

            if master:
                print(f'[MASTER] {master}')
            else:
                print('[MASTER] no response')

            if slave:
                print(f'[SLAVE ] {slave}')
            else:
                print(f'[SLAVE  CAN={args.slave}] no response')

            print()
            count += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
