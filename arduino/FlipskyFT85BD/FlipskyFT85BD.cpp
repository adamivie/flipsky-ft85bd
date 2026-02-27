/**
 * FlipskyFT85BD.cpp
 * =================
 * Implementation of the FlipskyFT85BD Arduino library.
 * See FlipskyFT85BD.h for full documentation.
 *
 * License: MIT
 */

#include "FlipskyFT85BD.h"

// ── Protocol constants ──────────────────────────────────────────────────────

static constexpr uint8_t STX = 0xAA;
static constexpr uint8_t ETX = 0xDD;

static constexpr uint8_t CMD_OBTAIN_DATA = 0x00;
static constexpr uint8_t CMD_CAN_OVER    = 0x10;
static constexpr uint8_t CMD_KEEP_LIVE   = 0x19;
static constexpr uint8_t CMD_SET_SPEED   = 0x27;

// ── Error name table ────────────────────────────────────────────────────────

static const char* const ERROR_NAMES[] = {
    "NONE",
    "PHASE_A_OVER_CURRENT", "PHASE_B_OVER_CURRENT", "PHASE_C_OVER_CURRENT",
    "PHASE_A_SENSOR",       "PHASE_B_SENSOR",        "PHASE_C_SENSOR",
    "PHASE_SUM_NOT_ZERO",
    "BUS_UNDER_VOLTAGE",    "BUS_OVER_VOLTAGE",
    "MOSFET_OVERHEAT",      "MOTOR_OVERHEAT",        "MOSFET_TEMP_SENSOR",
    "WATCHDOG_RESET",       "FLASH_CORRUPT",          "MCU_UNDER_VOLTAGE",
    "MOTOR_TEMP_SENSOR",    "MOTOR_BLOCKED",          "DRIVER_FAULT",
    "PHASE_LOSS",           "BUS_OVER_CURRENT"
};
static constexpr uint8_t ERROR_NAMES_COUNT =
    sizeof(ERROR_NAMES) / sizeof(ERROR_NAMES[0]);

const char* FlipskyFT85BD::Telemetry::errorName() const {
    if (error_code < ERROR_NAMES_COUNT) return ERROR_NAMES[error_code];
    return "UNKNOWN";
}

// ── Constructor / begin ─────────────────────────────────────────────────────

FlipskyFT85BD::FlipskyFT85BD(HardwareSerial& serial,
                               uint32_t baud,
                               int8_t   rxPin,
                               int8_t   txPin,
                               uint16_t readTimeoutMs)
    : _serial(serial)
    , _baud(baud)
    , _rxPin(rxPin)
    , _txPin(txPin)
    , _readTimeoutMs(readTimeoutMs)
{}

void FlipskyFT85BD::begin() {
#if defined(ESP32)
    if (_rxPin >= 0 && _txPin >= 0) {
        _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    } else {
        _serial.begin(_baud, SERIAL_8N1);
    }
#else
    // Arduino Mega / others: pin remapping not supported here
    _serial.begin(_baud);
#endif
}

// ── CRC-16 Modbus ───────────────────────────────────────────────────────────

uint16_t FlipskyFT85BD::_crc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
        }
    }
    return crc;
}

// ── Frame builder ───────────────────────────────────────────────────────────

uint8_t FlipskyFT85BD::_buildFrame(const uint8_t* info, uint8_t info_len) {
    // frame = STX | DLEN | INFO... | CRC_hi | CRC_lo | ETX
    uint16_t crc = _crc16(info, info_len);
    uint8_t  idx = 0;
    _buf[idx++] = STX;
    _buf[idx++] = info_len;
    for (uint8_t i = 0; i < info_len; i++) _buf[idx++] = info[i];
    _buf[idx++] = (uint8_t)(crc >> 8);
    _buf[idx++] = (uint8_t)(crc & 0xFF);
    _buf[idx++] = ETX;
    return idx;  // total frame length
}

// ── Low-level query ─────────────────────────────────────────────────────────

bool FlipskyFT85BD::_query(const uint8_t* req, uint8_t req_len, Telemetry& t) {
    // Flush stale data
    while (_serial.available()) _serial.read();

    _serial.write(req, req_len);

    // Accumulate bytes into rx_buf until we find a valid frame or time out
    uint8_t  rx_buf[64];
    uint8_t  rx_len  = 0;
    uint32_t deadline = millis() + _readTimeoutMs;

    while (millis() < deadline) {
        while (_serial.available() && rx_len < sizeof(rx_buf)) {
            rx_buf[rx_len++] = (uint8_t)_serial.read();
        }

        // Scan for STX
        for (uint8_t i = 0; i < rx_len; i++) {
            if (rx_buf[i] != STX) continue;

            // Need at least: STX DLEN ... CRC_hi CRC_lo ETX  = dlen + 5 bytes
            if (i + 1 >= rx_len) break;  // need DLEN byte
            uint8_t dlen = rx_buf[i + 1];
            uint8_t needed = dlen + 5;    // STX + DLEN + INFO + CRC_hi + CRC_lo + ETX

            if (i + needed > rx_len) break;  // incomplete frame

            const uint8_t* info = &rx_buf[i + 2];
            uint16_t crc_calc   = _crc16(info, dlen);
            uint16_t crc_rx     = ((uint16_t)rx_buf[i + 2 + dlen] << 8) |
                                              rx_buf[i + 3 + dlen];
            uint8_t etx         = rx_buf[i + 4 + dlen];

            if (etx != ETX || crc_calc != crc_rx) continue;

            return _parseTelemetry(info, dlen, t);
        }
        delay(1);
    }
    return false;  // timeout
}

// ── Telemetry parser ────────────────────────────────────────────────────────

bool FlipskyFT85BD::_parseTelemetry(const uint8_t* info, uint8_t info_len,
                                      Telemetry& t) {
    if (info_len < 1 || info[0] != CMD_OBTAIN_DATA) return false;

    const uint8_t* d = info + 1;
    uint8_t dlen     = info_len - 1;
    if (dlen < 28) return false;

    uint8_t i = 0;

    auto u8  = [&]() -> uint8_t  { return d[i++]; };
    auto i16 = [&]() -> int16_t  {
        int16_t v = (int16_t)((uint16_t)d[i] << 8 | d[i+1]);
        i += 2; return v;
    };
    auto i32 = [&]() -> int32_t  {
        int32_t v = (int32_t)((uint32_t)d[i]   << 24 | (uint32_t)d[i+1] << 16 |
                               (uint32_t)d[i+2] << 8  | (uint32_t)d[i+3]);
        i += 4; return v;
    };

    t.mcu_id          = u8();
    t.error_code      = u8();
    t.voltage_V       = i16() / 100.0f;
    t.batt_current_A  = i32() / 1000000.0f;
    t.motor_current_A = i32() / 1000000.0f;
    t.erpm            = i32();
    t.duty            = i16() / 10000.0f;
    t.temp_fet_C      = i16() / 100.0f;
    t.temp_motor_C    = i16() / 100.0f;
    t.cpu_load_pct    = i16() / 100.0f;
    // encoder_deg (4 bytes) intentionally skipped

    return true;
}

// ── Public API ──────────────────────────────────────────────────────────────

bool FlipskyFT85BD::getTelemetry(Telemetry& t, int16_t can_id) {
    uint8_t info[3];
    uint8_t info_len;

    if (can_id < 0) {
        info[0]  = CMD_OBTAIN_DATA;
        info_len = 1;
    } else {
        info[0]  = CMD_CAN_OVER;
        info[1]  = (uint8_t)can_id;
        info[2]  = CMD_OBTAIN_DATA;
        info_len = 3;
    }

    uint8_t frame_len = _buildFrame(info, info_len);
    return _query(_buf, frame_len, t);
}

void FlipskyFT85BD::setSpeed(int32_t erpm, int16_t can_id) {
    uint8_t info[7];
    uint8_t info_len;

    if (can_id < 0) {
        info[0]  = CMD_SET_SPEED;
        info[1]  = (uint8_t)(erpm >> 24);
        info[2]  = (uint8_t)(erpm >> 16);
        info[3]  = (uint8_t)(erpm >>  8);
        info[4]  = (uint8_t)(erpm);
        info_len = 5;
    } else {
        info[0]  = CMD_CAN_OVER;
        info[1]  = (uint8_t)can_id;
        info[2]  = CMD_SET_SPEED;
        info[3]  = (uint8_t)(erpm >> 24);
        info[4]  = (uint8_t)(erpm >> 16);
        info[5]  = (uint8_t)(erpm >>  8);
        info[6]  = (uint8_t)(erpm);
        info_len = 7;
    }

    uint8_t frame_len = _buildFrame(info, info_len);
    _serial.write(_buf, frame_len);
}

void FlipskyFT85BD::keepAlive() {
    uint8_t info[1] = { CMD_KEEP_LIVE };
    uint8_t frame_len = _buildFrame(info, 1);
    _serial.write(_buf, frame_len);
}
