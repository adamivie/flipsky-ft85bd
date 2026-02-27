"""
flipsky_ft85bd.py
=================
Python library for the Flipsky FT85BD dual ESC (and any Flipsky ESC using
the FTESC UART V1.4 protocol).

Protocol summary
----------------
Frame:  0xAA | DLEN | CMD | DATA... | CRC_hi | CRC_lo | 0xDD
CRC:    Modbus CRC-16  (poly 0xA001, init 0xFFFF)

Key commands
    0x00  OBTAIN_DATA_ONCE  – request 28-byte telemetry
    0x10  CAN_OVER          – forward cmd to slave via CAN bus
    0x19  KEEP_LIVE         – heartbeat (send every ≤500 ms while running)
    0x27  SET_SPEED         – int32 eRPM, big-endian

Hardware prerequisite
---------------------
In Flipsky ESC Tool → App Config → Input Signal Type must be set to **OFF**
on every channel you want to control via UART SET_SPEED.

Typical connection (FT85BD)
    Baud rate : 460800
    Master CAN ID : reported in telemetry mcu_id field (e.g. 251)
    Slave  CAN ID : reported in telemetry mcu_id field (e.g. 184)

Usage
-----
    import serial
    from flipsky_ft85bd import FlipskyFT85BD

    ser = serial.Serial('COM16', 460800, timeout=0.1)
    esc = FlipskyFT85BD(ser)

    t = esc.get_telemetry()          # master
    t = esc.get_telemetry(can_id=184) # slave via CAN

    esc.set_speed(5000)              # master forward
    esc.set_speed(-5000, can_id=184) # slave reverse
    esc.keep_alive()
"""

import struct
import time
from dataclasses import dataclass, field
from typing import Optional

try:
    import serial  # pyserial
except ImportError:
    serial = None  # allow import without pyserial for type-checking / docs

__all__ = ['FlipskyFT85BD', 'Telemetry', 'ERROR_NAMES']
__version__ = '1.0.0'

# ─── Protocol constants ────────────────────────────────────────────────────

STX = 0xAA
ETX = 0xDD

CMD_OBTAIN_DATA = 0x00
CMD_CAN_OVER    = 0x10
CMD_KEEP_LIVE   = 0x19
CMD_SET_SPEED   = 0x27

DEFAULT_BAUD    = 460800

ERROR_NAMES = {
    0:  'NONE',
    1:  'PHASE_A_OVER_CURRENT', 2:  'PHASE_B_OVER_CURRENT',
    3:  'PHASE_C_OVER_CURRENT', 4:  'PHASE_A_SENSOR',
    5:  'PHASE_B_SENSOR',       6:  'PHASE_C_SENSOR',
    7:  'PHASE_SUM_NOT_ZERO',   8:  'BUS_UNDER_VOLTAGE',
    9:  'BUS_OVER_VOLTAGE',     10: 'MOSFET_OVERHEAT',
    11: 'MOTOR_OVERHEAT',       12: 'MOSFET_TEMP_SENSOR',
    13: 'WATCHDOG_RESET',       14: 'FLASH_CORRUPT',
    15: 'MCU_UNDER_VOLTAGE',    16: 'MOTOR_TEMP_SENSOR',
    17: 'MOTOR_BLOCKED',        18: 'DRIVER_FAULT',
    19: 'PHASE_LOSS',           20: 'BUS_OVER_CURRENT',
}


# ─── Telemetry dataclass ───────────────────────────────────────────────────

@dataclass
class Telemetry:
    """Decoded telemetry from OBTAIN_DATA_ONCE (CMD 0x00)."""
    mcu_id:          int   = 0      # ESC's own CAN ID
    error_code:      int   = 0
    voltage_V:       float = 0.0
    batt_current_A:  float = 0.0
    motor_current_A: float = 0.0
    erpm:            int   = 0
    duty:            float = 0.0    # 0.0 – 1.0
    temp_fet_C:      float = 0.0
    temp_motor_C:    float = 0.0
    cpu_load_pct:    float = 0.0
    encoder_deg:     float = 0.0

    @property
    def error_name(self) -> str:
        return ERROR_NAMES.get(self.error_code, f'UNKNOWN({self.error_code})')

    @property
    def ok(self) -> bool:
        return self.error_code == 0

    def __str__(self) -> str:
        return (
            f'MCU={self.mcu_id}  err={self.error_name}  '
            f'V={self.voltage_V:.2f}V  '
            f'Ibatt={self.batt_current_A:.2f}A  Imot={self.motor_current_A:.2f}A  '
            f'eRPM={self.erpm:+d}  duty={self.duty*100:.1f}%  '
            f'Tfet={self.temp_fet_C:.1f}°C  Tmot={self.temp_motor_C:.1f}°C'
        )


# ─── Low-level frame helpers (usable without a class) ─────────────────────

def crc16(data: bytes) -> int:
    """Modbus CRC-16 (poly=0xA001, init=0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_frame(info: bytes) -> bytes:
    """Wrap an info payload in the FTESC UART frame."""
    c = crc16(info)
    return bytes([STX, len(info)]) + info + bytes([c >> 8, c & 0xFF, ETX])


def parse_frame(raw: bytes) -> Optional[bytes]:
    """
    Find and validate the first complete FTESC UART frame in *raw*.
    Returns the info payload bytes on success, or None.
    """
    idx = raw.find(STX)
    if idx == -1:
        return None
    try:
        dlen   = raw[idx + 1]
        info   = raw[idx + 2 : idx + 2 + dlen]
        crc_rx = (raw[idx + 2 + dlen] << 8) | raw[idx + 3 + dlen]
        etx    = raw[idx + 4 + dlen]
    except IndexError:
        return None
    if etx != ETX:
        return None
    if crc16(info) != crc_rx:
        return None
    return info


def parse_telemetry(info: bytes) -> Optional[Telemetry]:
    """Parse an info payload from CMD_OBTAIN_DATA response into a Telemetry."""
    if not info or info[0] != CMD_OBTAIN_DATA:
        return None
    d = info[1:]
    if len(d) < 28:
        return None

    i = 0
    def u8():
        nonlocal i; v = d[i]; i += 1; return v
    def i16():
        nonlocal i; v = struct.unpack_from('>h', d, i)[0]; i += 2; return v
    def i32():
        nonlocal i; v = struct.unpack_from('>i', d, i)[0]; i += 4; return v

    return Telemetry(
        mcu_id          = u8(),
        error_code      = u8(),
        voltage_V       = i16() / 100.0,
        batt_current_A  = i32() / 1_000_000.0,
        motor_current_A = i32() / 1_000_000.0,
        erpm            = i32(),
        duty            = i16() / 10_000.0,
        temp_fet_C      = i16() / 100.0,
        temp_motor_C    = i16() / 100.0,
        cpu_load_pct    = i16() / 100.0,
        encoder_deg     = i32() / 1_000_000.0,
    )


# ─── High-level class ──────────────────────────────────────────────────────

class FlipskyFT85BD:
    """
    Driver for the Flipsky FT85BD dual ESC (FTESC UART V1.4 protocol).

    Parameters
    ----------
    serial_port : serial.Serial
        An already-opened pyserial port (or any object with .write()/.read()
        and .in_waiting).
    read_timeout : float
        Seconds to wait for a telemetry reply (default 0.15 s).

    Example
    -------
    >>> import serial
    >>> from flipsky_ft85bd import FlipskyFT85BD
    >>> ser = serial.Serial('COM16', 460800, timeout=0.1)
    >>> esc = FlipskyFT85BD(ser)
    >>> t = esc.get_telemetry()
    >>> print(t)
    """

    def __init__(self, serial_port, read_timeout: float = 0.15):
        self._ser = serial_port
        self._read_timeout = read_timeout

    # ── Frame factories ──────────────────────────────────────────────────

    def _obtain_data_frame(self, can_id: int = -1) -> bytes:
        if can_id < 0:
            return build_frame(bytes([CMD_OBTAIN_DATA]))
        return build_frame(bytes([CMD_CAN_OVER, can_id, CMD_OBTAIN_DATA]))

    def _set_speed_frame(self, erpm: int, can_id: int = -1) -> bytes:
        payload = struct.pack('>i', erpm)
        if can_id < 0:
            return build_frame(bytes([CMD_SET_SPEED]) + payload)
        return build_frame(bytes([CMD_CAN_OVER, can_id, CMD_SET_SPEED]) + payload)

    def _keep_live_frame(self) -> bytes:
        return build_frame(bytes([CMD_KEEP_LIVE]))

    # ── Serial helpers ───────────────────────────────────────────────────

    def _query(self, request: bytes) -> Optional[Telemetry]:
        """Send request, wait for a valid telemetry reply."""
        self._ser.reset_input_buffer()
        self._ser.write(request)
        deadline = time.monotonic() + self._read_timeout
        buf = b''
        while time.monotonic() < deadline:
            chunk = self._ser.read(self._ser.in_waiting or 1)
            if chunk:
                buf += chunk
                info = parse_frame(buf)
                if info is not None:
                    return parse_telemetry(info)
        return None

    # ── Public API ───────────────────────────────────────────────────────

    def get_telemetry(self, can_id: int = -1) -> Optional[Telemetry]:
        """
        Request telemetry from master (can_id=-1) or a CAN slave.

        Returns a Telemetry dataclass, or None if no response within timeout.
        """
        return self._query(self._obtain_data_frame(can_id))

    def set_speed(self, erpm: int, can_id: int = -1) -> None:
        """
        Command a speed in electrical RPM.

        Positive = forward, negative = reverse.
        ESC Tool → Input Signal Type must be OFF for this to work.
        """
        self._ser.write(self._set_speed_frame(erpm, can_id))

    def stop(self, can_id: int = -1) -> None:
        """Immediately command zero speed."""
        self.set_speed(0, can_id)

    def keep_alive(self) -> None:
        """
        Send heartbeat.  Must be called at least every 500 ms while motors
        are running, otherwise the ESC will stop the motor.
        """
        self._ser.write(self._keep_live_frame())
