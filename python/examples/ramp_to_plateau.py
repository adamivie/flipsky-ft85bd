"""
ramp_to_plateau.py
==================
Ramp all 4 motors up in eRPM steps, reading telemetry each step.
When any motor's actual eRPM stops increasing (plateau detected),
stop everything and report the results.

⚠  Lift the robot off the ground before running!
⚠  ESC Tool → Input Signal Type must be OFF on all channels.

Usage
-----
    python ramp_to_plateau.py --port COM18

ESC IDs (FT85BD x2):
    Master: 164   Slaves: 115, 184, 251
"""

import argparse
import sys
import time

import serial

sys.path.insert(0, '..')
from flipsky_ft85bd import FlipskyFT85BD, DEFAULT_BAUD


# ── Configuration ─────────────────────────────────────────────────────────

SLAVE_IDS      = [115, 184, 251]     # all 3 slave CAN IDs
ERPM_STEP      = 1000               # eRPM increment per step
ERPM_START     = 1000               # starting eRPM
ERPM_LIMIT     = 80000              # safety cap — never command more than this
HOLD_TIME      = 1.5                # seconds to hold at each step
KEEPALIVE_HZ   = 10                 # heartbeat rate while holding
PLATEAU_THRESH = 0.15               # if actual < (1 - thresh) * commanded → plateau
MIN_STEPS_PLATEAU = 2               # need N consecutive plateau readings to confirm


def label(can_id):
    return f'MASTER({can_id})' if can_id == -1 else f'SLAVE({can_id})'


def main():
    ap = argparse.ArgumentParser(description='Ramp all motors until plateau')
    ap.add_argument('--port',       required=True)
    ap.add_argument('--baud',       default=DEFAULT_BAUD, type=int)
    ap.add_argument('--step',       default=ERPM_STEP, type=int,
                    help=f'eRPM step size (default {ERPM_STEP})')
    ap.add_argument('--hold',       default=HOLD_TIME, type=float,
                    help=f'Seconds to hold each step (default {HOLD_TIME})')
    ap.add_argument('--limit',      default=ERPM_LIMIT, type=int,
                    help=f'Max eRPM safety cap (default {ERPM_LIMIT})')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    # All CAN IDs:  -1 = master (UART direct), then the three slaves
    all_ids = [-1] + SLAVE_IDS

    print(f'\nConnected to {args.port} @ {args.baud}')
    print(f'Motors: Master(164), Slaves {SLAVE_IDS}')
    print(f'Step: {args.step} eRPM   Hold: {args.hold}s   Limit: {args.limit} eRPM')
    print('⚠  Make sure wheels are OFF the ground!')
    input('\nPress ENTER to start ramp, Ctrl+C to abort...\n')

    # Track consecutive plateau counts per motor
    plateau_count = {cid: 0 for cid in all_ids}
    peak_erpm     = {cid: 0 for cid in all_ids}  # best actual eRPM seen
    plateau_motor = None
    commanded     = ERPM_START

    try:
        while commanded <= args.limit:
            print(f'\n{"="*60}')
            print(f'  Commanding {commanded} eRPM ...')
            print(f'{"="*60}')

            # ── Send speed to all motors ──────────────────────────────
            for cid in all_ids:
                esc.set_speed(commanded, can_id=cid)

            # ── Hold at this speed, sending keepalive ─────────────────
            t_end = time.monotonic() + args.hold
            interval = 1.0 / KEEPALIVE_HZ
            last_telem = {}

            while time.monotonic() < t_end:
                # re-send speed + keepalive
                for cid in all_ids:
                    esc.set_speed(commanded, can_id=cid)
                esc.keep_alive()

                # read telemetry from each motor
                for cid in all_ids:
                    t = esc.get_telemetry(can_id=cid)
                    if t:
                        last_telem[cid] = t

                time.sleep(interval)

            # ── Check telemetry after hold period ─────────────────────
            for cid in all_ids:
                t = last_telem.get(cid)
                if t is None:
                    print(f'  {label(cid):16s}  ⚠ no telemetry')
                    continue

                actual = abs(t.erpm)
                peak_erpm[cid] = max(peak_erpm[cid], actual)
                ratio = actual / commanded if commanded else 0

                status = ''
                if actual < commanded * (1 - PLATEAU_THRESH):
                    plateau_count[cid] += 1
                    status = f'  ← PLATEAU ({plateau_count[cid]}/{MIN_STEPS_PLATEAU})'
                else:
                    plateau_count[cid] = 0  # reset

                print(f'  {label(cid):16s}  cmd={commanded:6d}  actual={actual:6d}  '
                      f'({ratio*100:5.1f}%)  err={t.error_name:20s}  '
                      f'Tfet={t.temp_fet_C:.1f}°C  Tmot={t.temp_motor_C:.1f}°C'
                      f'{status}')

                # error? stop immediately
                if not t.ok:
                    print(f'\n⛔ ERROR on {label(cid)}: {t.error_name} — stopping!')
                    plateau_motor = cid
                    break

                # confirmed plateau?
                if plateau_count[cid] >= MIN_STEPS_PLATEAU:
                    plateau_motor = cid
                    break

            if plateau_motor is not None:
                break

            commanded += args.step

        # ── Report ────────────────────────────────────────────────────────
        print(f'\n{"="*60}')
        if plateau_motor is not None:
            print(f'  Plateau detected on {label(plateau_motor)}!')
        else:
            print(f'  Reached safety limit ({args.limit} eRPM)')
        print(f'  Commanded at stop: {commanded} eRPM')
        print(f'\n  Peak actual eRPM per motor:')
        for cid in all_ids:
            print(f'    {label(cid):16s}  {peak_erpm[cid]:6d} eRPM')
        print(f'{"="*60}')

    except KeyboardInterrupt:
        print('\n\nInterrupted by user!')

    finally:
        print('\nStopping all motors...')
        for cid in all_ids:
            esc.stop(can_id=cid)
        esc.keep_alive()
        time.sleep(0.2)
        for cid in all_ids:
            esc.stop(can_id=cid)
        ser.close()
        print('Done. All motors stopped.')


if __name__ == '__main__':
    main()
