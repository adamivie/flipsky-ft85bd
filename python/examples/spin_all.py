"""
spin_all.py
===========
Spin all 4 motors simultaneously at a given eRPM for a set duration.

⚠  Lift the robot off the ground before running!

Usage
-----
    python spin_all.py --port COM18
    python spin_all.py --port COM18 --erpm 3000 --duration 5
"""

import argparse
import sys
import time

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD


MOTOR_IDS = [-1, 115, 184, 251]   # master + 3 slaves
LABELS    = {-1: 'MASTER(164)', 115: 'SLAVE(115)', 184: 'SLAVE(184)', 251: 'SLAVE(251)'}


def main():
    ap = argparse.ArgumentParser(description='Spin all 4 motors at once')
    ap.add_argument('--port',     required=True)
    ap.add_argument('--baud',     default=DEFAULT_BAUD, type=int)
    ap.add_argument('--erpm',     default=2000, type=int, help='eRPM (default 2000)')
    ap.add_argument('--duration', default=5.0, type=float, help='Seconds (default 5)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    print(f'\nConnected to {args.port} @ {args.baud}')
    print(f'Spinning all 4 motors at {args.erpm} eRPM for {args.duration}s')
    print('⚠  Make sure wheels are OFF the ground!')
    input('\nPress ENTER to start, Ctrl+C to abort...\n')

    t_end = time.monotonic() + args.duration
    try:
        while time.monotonic() < t_end:
            for cid in MOTOR_IDS:
                esc.set_speed(args.erpm, can_id=cid)
            esc.keep_alive()

            # Read telemetry
            for cid in MOTOR_IDS:
                t = esc.get_telemetry(can_id=cid)
                if t:
                    print(f'  {LABELS[cid]:16s}  eRPM={t.erpm:+6d}  '
                          f'duty={t.duty*100:5.1f}%  V={t.voltage_V:.1f}V  '
                          f'Tfet={t.temp_fet_C:.1f}°C')
            print()
            time.sleep(0.3)

    except KeyboardInterrupt:
        print('\nInterrupted!')

    finally:
        print('Stopping all motors...')
        for cid in MOTOR_IDS:
            esc.stop(can_id=cid)
        esc.keep_alive()
        time.sleep(0.2)
        for cid in MOTOR_IDS:
            esc.stop(can_id=cid)
        ser.close()
        print('Done.')


if __name__ == '__main__':
    main()
