"""
identify_motors.py
==================
Spin each motor one at a time so you can identify which wheel
corresponds to which ESC CAN ID.

⚠  Lift the robot off the ground before running!

Usage
-----
    python identify_motors.py --port COM18
"""

import argparse
import sys
import time

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD


MOTOR_IDS = [
    (-1,  'MASTER (164)'),
    (115, 'SLAVE  (115)'),
    (184, 'SLAVE  (184)'),
    (251, 'SLAVE  (251)'),
]

SPIN_ERPM    = 2000   # gentle speed
SPIN_TIME    = 3.0    # seconds per motor
KEEPALIVE_HZ = 10


def main():
    ap = argparse.ArgumentParser(description='Identify each motor by spinning one at a time')
    ap.add_argument('--port',  required=True)
    ap.add_argument('--baud',  default=DEFAULT_BAUD, type=int)
    ap.add_argument('--erpm',  default=SPIN_ERPM, type=int,
                    help=f'Spin speed (default {SPIN_ERPM})')
    ap.add_argument('--time',  default=SPIN_TIME, type=float,
                    help=f'Seconds to spin each motor (default {SPIN_TIME})')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    all_ids = [m[0] for m in MOTOR_IDS]

    print(f'\nConnected to {args.port} @ {args.baud}')
    print(f'Will spin each motor at {args.erpm} eRPM for {args.time}s')
    print('⚠  Make sure wheels are OFF the ground!\n')

    try:
        for can_id, name in MOTOR_IDS:
            input(f'Press ENTER to spin  >>>  {name}  <<<   (Ctrl+C to abort) ')

            print(f'  Spinning {name} ...')
            t_end = time.monotonic() + args.time
            interval = 1.0 / KEEPALIVE_HZ

            while time.monotonic() < t_end:
                esc.set_speed(args.erpm, can_id=can_id)
                esc.keep_alive()
                time.sleep(interval)

            # Stop this motor
            esc.stop(can_id=can_id)
            esc.keep_alive()
            time.sleep(0.2)
            esc.stop(can_id=can_id)

            print(f'  Stopped. Which wheel was that?\n')

    except KeyboardInterrupt:
        print('\n\nAborted!')

    finally:
        print('Stopping all motors...')
        for can_id, _ in MOTOR_IDS:
            esc.stop(can_id=can_id)
        esc.keep_alive()
        time.sleep(0.2)
        ser.close()
        print('Done.')


if __name__ == '__main__':
    main()
