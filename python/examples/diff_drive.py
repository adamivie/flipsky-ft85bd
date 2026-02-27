"""
diff_drive.py
=============
Simple keyboard-controlled differential drive demo.

Keys:  W / S = forward / back   A / D = turn left / right
       Space = stop             Q = quit

⚠  ESC Tool → Input Signal Type must be OFF on both channels.

Usage
-----
    python diff_drive.py --port COM16
    python diff_drive.py --port /dev/ttyACM0 --slave 184 --max-erpm 8000
"""

import argparse
import sys
import time
import threading

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD

try:
    import msvcrt   # Windows
    def getch():
        ch = msvcrt.getch()
        return ch.decode('utf-8', errors='ignore').lower()
except ImportError:
    import tty, termios
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port',      required=True)
    ap.add_argument('--baud',      default=DEFAULT_BAUD, type=int)
    ap.add_argument('--slave',     default=184, type=int)
    ap.add_argument('--max-erpm',  default=6000, type=int,
                    help='Maximum eRPM for full speed (default 6000)')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    left_erpm  = 0
    right_erpm = 0
    running    = True

    def control_loop():
        """Send speed commands and heartbeat at 20 Hz."""
        while running:
            esc.set_speed( left_erpm)
            esc.set_speed(-right_erpm, can_id=args.slave)  # right is inverted
            esc.keep_alive()
            time.sleep(0.05)

    t = threading.Thread(target=control_loop, daemon=True)
    t.start()

    print('\nDiff-drive demo  (W/S=fwd/back  A/D=turn  Space=stop  Q=quit)\n')
    try:
        while True:
            ch = getch()
            step = args.max_erpm
            if ch == 'w':
                left_erpm  =  step;  right_erpm =  step
            elif ch == 's':
                left_erpm  = -step;  right_erpm = -step
            elif ch == 'a':
                left_erpm  = -step // 2;  right_erpm = step // 2
            elif ch == 'd':
                left_erpm  =  step // 2;  right_erpm = -step // 2
            elif ch == ' ':
                left_erpm  = 0;  right_erpm = 0
            elif ch == 'q':
                break
            print(f'  left={left_erpm:+6d}  right={right_erpm:+6d} eRPM')
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        esc.stop()
        esc.stop(can_id=args.slave)
        time.sleep(0.1)
        ser.close()
        print('\nStopped.')


if __name__ == '__main__':
    main()
