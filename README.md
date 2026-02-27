# flipsky-ft85bd

Arduino (ESP32 / Mega) and Python library for the **Flipsky FT85BD** dual ESC
and any ESC using the **Flipsky FTESC UART V1.4** protocol.

> **Note — this ESC is NON-VESC.** The FT85BD uses Flipsky's own proprietary
> FTESC UART protocol, not the VESC protocol.  This library implements the
> correct framing, CRC and commands from the official FTESC UART V1.4 spec.

---

## Features

- Read full telemetry: voltage, battery & motor current, eRPM, duty cycle, FET & motor temps, error codes, CPU load
- Command motor speed (eRPM) on master and/or CAN slave channels
- Heartbeat / keep-alive packet
- Modbus CRC-16 verified on every frame
- Arduino library — works with **ESP32** (native 460800 baud) and **Arduino Mega**
- Python library — works on any OS with `pyserial`
- Tested against a real FT85BD at 460800 baud (master CAN ID 251, slave CAN ID 184)

---

## Protocol summary

```
Frame:  0xAA | DLEN | CMD | DATA... | CRC_hi | CRC_lo | 0xDD
CRC:    Modbus CRC-16  (poly = 0xA001, init = 0xFFFF)
```

| CMD  | Name             | Payload                       |
|------|------------------|-------------------------------|
| 0x00 | OBTAIN_DATA_ONCE | _(none)_ — returns 28-byte telemetry |
| 0x10 | CAN_OVER         | `can_id, slave_cmd, [slave_data]` — forward to slave |
| 0x19 | KEEP_LIVE        | _(none)_ — heartbeat, send every ≤500 ms |
| 0x27 | SET_SPEED        | `int32` eRPM, big-endian |

**Hardware prerequisite:** In Flipsky ESC Tool → App Config → **Input Signal Type
must be set to OFF** on every channel you want to control via UART SET_SPEED.
With any other input type the speed command is silently ignored.

---

## Wiring

| FT85BD pin | MCU pin |
|-----------|---------|
| GND       | GND     |
| TX        | RX      |
| RX        | TX      |

Baud rate: **460800**

---

## Arduino / ESP32

### Installation

**Option A — Arduino Library Manager**
Search for `FlipskyFT85BD` and install.

**Option B — manual**
Copy the `arduino/FlipskyFT85BD/` folder into your Arduino `libraries/` directory.

### Quick start

```cpp
#include <FlipskyFT85BD.h>

// ESP32 — Serial2 on GPIO16 (RX) / GPIO17 (TX)
FlipskyFT85BD esc(Serial2, 460800, /*rx=*/16, /*tx=*/17);

// Arduino Mega — Serial1 (no pin args needed)
// FlipskyFT85BD esc(Serial1, 460800);

void setup() {
    Serial.begin(115200);
    esc.begin();
}

void loop() {
    // Read telemetry from master
    FlipskyFT85BD::Telemetry t;
    if (esc.getTelemetry(t)) {
        Serial.printf("V=%.2f V  eRPM=%d  err=%s\n",
                      t.voltage_V, t.erpm, t.errorName());
    }

    // Read telemetry from slave (CAN id 184)
    FlipskyFT85BD::Telemetry slave;
    if (esc.getTelemetry(slave, 184)) {
        Serial.printf("[slave] V=%.2f V  eRPM=%d\n",
                      slave.voltage_V, slave.erpm);
    }

    // Drive both motors (differential drive — right motor is inverted)
    esc.setSpeed( 3000);        // left  forward 3000 eRPM
    esc.setSpeed(-3000, 184);   // right reverse 3000 eRPM
    esc.keepAlive();            // must call every ≤500 ms
    delay(50);
}
```

### API reference

```cpp
// Constructor
FlipskyFT85BD(HardwareSerial& serial,
              uint32_t baud          = 460800,
              int8_t   rxPin         = -1,    // ESP32 only
              int8_t   txPin         = -1,    // ESP32 only
              uint16_t readTimeoutMs = 150);

void begin();                              // call in setup()

bool getTelemetry(Telemetry& t,
                  int16_t can_id = -1);   // -1 = master, 0-254 = slave

void setSpeed(int32_t erpm,
              int16_t can_id = -1);       // positive = forward

void stop(int16_t can_id = -1);           // setSpeed(0, can_id)

void keepAlive();                         // send heartbeat
```

#### `Telemetry` struct

```cpp
struct Telemetry {
    uint8_t  mcu_id;           // ESC's own CAN ID
    uint8_t  error_code;       // 0 = no error
    float    voltage_V;
    float    batt_current_A;
    float    motor_current_A;
    int32_t  erpm;             // positive = forward
    float    duty;             // 0.0 – 1.0
    float    temp_fet_C;
    float    temp_motor_C;
    float    cpu_load_pct;

    bool        ok() const;          // true when error_code == 0
    const char* errorName() const;   // e.g. "NONE", "BUS_OVER_VOLTAGE"
};
```

### Examples

| Sketch | Description |
|--------|-------------|
| `Telemetry` | Print live telemetry from both channels at 2 Hz |
| `SpinTest`  | Ramp to target eRPM, hold, stop |
| `DiffDrive` | Serial-controlled differential drive |

---

## Python

### Installation

```bash
pip install pyserial
```

No extra packages required.

### Quick start

```python
import serial
from flipsky_ft85bd import FlipskyFT85BD

ser = serial.Serial('COM16', 460800, timeout=0.1)   # Windows
# ser = serial.Serial('/dev/ttyACM0', 460800, timeout=0.1)  # Linux

esc = FlipskyFT85BD(ser)

# Telemetry
t = esc.get_telemetry()       # master
print(t)                       # MCU=251  err=NONE  V=24.93V  ...

t = esc.get_telemetry(184)     # slave via CAN

# Speed control
esc.set_speed(5000)            # master forward
esc.set_speed(-5000, 184)      # slave reverse
esc.keep_alive()               # heartbeat
```

### Examples

| Script | Description |
|--------|-------------|
| `examples/telemetry.py`  | Print live telemetry at 2 Hz |
| `examples/spin_test.py`  | Interactive spin test with safety prompt |
| `examples/diff_drive.py` | Keyboard-controlled differential drive (W/A/S/D) |

```bash
python examples/telemetry.py  --port COM16
python examples/spin_test.py  --port COM16 --erpm 5000 --duration 5
python examples/diff_drive.py --port COM16 --slave 184
```

---

## Tested hardware

| Item | Value |
|------|-------|
| ESC  | Flipsky FT85BD (dual channel, AT32F403, 84V/200A) |
| Master CAN ID | 251 (0xFB) |
| Slave CAN ID  | 184 (0xB8) |
| Baud rate     | 460800 |
| Motors        | Hoverboard hub motors 6.5" |
| Supply voltage | 24 V (6S LiPo) |

---

## Compatible ESCs

Any Flipsky ESC using the FTESC UART V1.4 protocol should work, including:

- FT85BD (dual)
- FSESC 4.20 / 6.80 / 75200 (single, via UART header)
- Other FTESC-series controllers

---

## Contributing

Issues and pull requests welcome. If you have a different Flipsky ESC and can
confirm compatibility (or not), please open an issue with your MCU ID and baud
rate — it helps build a compatibility list.

---

## License

MIT — see [LICENSE](LICENSE).
