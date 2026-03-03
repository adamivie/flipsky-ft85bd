/**
 * IBusRC.cpp
 * ==========
 * Implementation of the IBusRC library.
 * See IBusRC.h for full documentation.
 *
 * License: MIT
 */

#include "IBusRC.h"

// ── iBus protocol constants ──────────────────────────────────────────────────

static constexpr uint8_t  IBUS_FRAME_LEN  = 32;      // channel frame length
static constexpr uint8_t  IBUS_CMD_CHAN   = 0x40;     // channel data command
static constexpr uint8_t  IBUS_SENSOR_LEN = 4;        // sensor request length
static constexpr uint16_t IBUS_CHECKSUM_BASE = 0xFFFF;

// Sensor protocol command bases
static constexpr uint8_t  SENS_DISCOVER_BASE = 0x81;  // discovery: 0x81 + addr
static constexpr uint8_t  SENS_GETDATA_BASE  = 0xA1;  // data req:  0xA1 + addr
static constexpr uint8_t  SENS_REPLY_DISC    = 0x01;  // reply cmd: 0x01 + addr
static constexpr uint8_t  SENS_REPLY_DATA    = 0x21;  // reply cmd: 0x21 + addr

// iBus serial config
static constexpr uint32_t IBUS_BAUD = 115200;

// ── Checksum ─────────────────────────────────────────────────────────────────

uint16_t IBusRC::_checksum(const uint8_t* data, uint8_t len) {
    uint16_t sum = IBUS_CHECKSUM_BASE;
    for (uint8_t i = 0; i < len; i++) {
        sum -= data[i];
    }
    return sum;
}

// ── begin methods ────────────────────────────────────────────────────────────

void IBusRC::beginChannels(HardwareSerial& serial, int8_t rxPin, int8_t txPin) {
    _chSerial = &serial;
    #if defined(ARDUINO_ARCH_ESP32)
        _chSerial->begin(IBUS_BAUD, SERIAL_8N1, rxPin, txPin);
    #else
        _chSerial->begin(IBUS_BAUD);
        (void)rxPin; (void)txPin;
    #endif
    _chIdx = 0;
}

void IBusRC::beginSensor(HardwareSerial& serial, int8_t rxPin, int8_t txPin) {
    _snSerial = &serial;
    _sensorSamePort = (_snSerial == _chSerial);

    if (!_sensorSamePort) {
        #if defined(ARDUINO_ARCH_ESP32)
            _snSerial->begin(IBUS_BAUD, SERIAL_8N1, rxPin, txPin);
        #else
            _snSerial->begin(IBUS_BAUD);
            (void)rxPin; (void)txPin;
        #endif
    }
    _snIdx = 0;
}

// ── Sensor registration ─────────────────────────────────────────────────────

int8_t IBusRC::addSensor(uint16_t sensorType, SensorCallback callback) {
    if (_sensorCount >= MAX_SENSORS) return -1;
    uint8_t idx = _sensorCount++;
    _sensors[idx].type   = sensorType;
    _sensors[idx].cb     = callback;
    _sensors[idx].value  = 0;
    _sensors[idx].active = true;
    return (int8_t)idx;
}

void IBusRC::setSensorValue(uint8_t sensorIndex, uint16_t value) {
    if (sensorIndex < MAX_SENSORS && _sensors[sensorIndex].active) {
        _sensors[sensorIndex].value = value;
    }
}

// ── Main loop ────────────────────────────────────────────────────────────────

void IBusRC::loop() {
    // Process channel serial data
    if (_chSerial) {
        while (_chSerial->available()) {
            _processChannelByte((uint8_t)_chSerial->read());
        }
    }

    // Process sensor serial data (if separate port)
    if (_snSerial && !_sensorSamePort) {
        while (_snSerial->available()) {
            _processSensorByte((uint8_t)_snSerial->read());
        }
    }
}

// ── Channel frame parser ─────────────────────────────────────────────────────

void IBusRC::_processChannelByte(uint8_t b) {
    // State machine: accumulate bytes into _chBuf[32]
    if (_chIdx == 0) {
        // Looking for frame start: length byte = 0x20
        if (b == IBUS_FRAME_LEN) {
            _chBuf[0] = b;
            _chIdx = 1;
        }
        return;
    }

    _chBuf[_chIdx++] = b;

    // Also handle sensor requests interleaved on the same wire (SENSOR port)
    // Sensor requests are 4 bytes starting with 0x04
    // We only check after we know this isn't a channel frame

    if (_chIdx == 2) {
        // Second byte should be command
        if (b != IBUS_CMD_CHAN) {
            // Not a channel frame — could be a sensor request if using SENSOR port
            if (_sensorSamePort && _chBuf[0] == IBUS_SENSOR_LEN) {
                // Redirect to sensor parser
                _snBuf[0] = _chBuf[0];
                _snBuf[1] = _chBuf[1];
                _snIdx = 2;
            }
            _chIdx = 0;  // reset channel parser
            return;
        }
    }

    if (_chIdx >= IBUS_FRAME_LEN) {
        // Full frame received — validate and extract
        if (_validateChannelFrame()) {
            for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
                uint8_t offset = 2 + ch * 2;
                _channels[ch] = (uint16_t)_chBuf[offset] | ((uint16_t)_chBuf[offset + 1] << 8);
            }
            _lastFrameMs = millis();
            _channelFrameCount++;
        }
        _chIdx = 0;
    }
}

bool IBusRC::_validateChannelFrame() {
    // Checksum is the last 2 bytes (little-endian)
    uint16_t rxChk = (uint16_t)_chBuf[30] | ((uint16_t)_chBuf[31] << 8);
    uint16_t calcChk = _checksum(_chBuf, 30);
    return rxChk == calcChk;
}

// ── Sensor request parser ────────────────────────────────────────────────────

void IBusRC::_processSensorByte(uint8_t b) {
    if (!_snSerial) return;

    if (_snIdx == 0) {
        if (b == IBUS_SENSOR_LEN) {
            _snBuf[0] = b;
            _snIdx = 1;
        }
        return;
    }

    _snBuf[_snIdx++] = b;

    if (_snIdx >= IBUS_SENSOR_LEN) {
        _handleSensorRequest();
        _snIdx = 0;
    }
}

void IBusRC::_handleSensorRequest() {
    // Validate checksum
    uint16_t rxChk = (uint16_t)_snBuf[2] | ((uint16_t)_snBuf[3] << 8);
    uint16_t calcChk = _checksum(_snBuf, 2);
    if (rxChk != calcChk) return;

    uint8_t cmd = _snBuf[1];

    // ── Discovery: 0x81 + addr ───────────────────────────────────────────
    if (cmd >= SENS_DISCOVER_BASE && cmd < SENS_DISCOVER_BASE + MAX_SENSORS) {
        uint8_t addr = cmd - SENS_DISCOVER_BASE;
        if (addr < _sensorCount && _sensors[addr].active) {
            uint8_t reply[6];
            reply[0] = 0x06;                            // length
            reply[1] = SENS_REPLY_DISC + addr;          // command
            reply[2] = _sensors[addr].type & 0xFF;      // type low
            reply[3] = (_sensors[addr].type >> 8) & 0xFF; // type high
            uint16_t chk = _checksum(reply, 4);
            reply[4] = chk & 0xFF;
            reply[5] = (chk >> 8) & 0xFF;
            _sendSensorReply(reply, 6);
        }
        return;
    }

    // ── Data request: 0xA1 + addr ────────────────────────────────────────
    if (cmd >= SENS_GETDATA_BASE && cmd < SENS_GETDATA_BASE + MAX_SENSORS) {
        uint8_t addr = cmd - SENS_GETDATA_BASE;
        if (addr < _sensorCount && _sensors[addr].active) {
            // Get value — prefer callback, fallback to stored value
            uint16_t val = _sensors[addr].value;
            if (_sensors[addr].cb) {
                val = _sensors[addr].cb(addr);
            }

            uint8_t reply[6];
            reply[0] = 0x06;                        // length
            reply[1] = SENS_REPLY_DATA + addr;      // command
            reply[2] = val & 0xFF;                   // value low
            reply[3] = (val >> 8) & 0xFF;            // value high
            uint16_t chk = _checksum(reply, 4);
            reply[4] = chk & 0xFF;
            reply[5] = (chk >> 8) & 0xFF;
            _sendSensorReply(reply, 6);
        }
        return;
    }
}

void IBusRC::_sendSensorReply(const uint8_t* data, uint8_t len) {
    if (!_snSerial) return;

    // Small delay for bus turnaround (receiver needs time to switch to RX)
    delayMicroseconds(500);
    _snSerial->write(data, len);
    _snSerial->flush();  // wait for TX to complete
}
