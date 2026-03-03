/**
 * IBusRC.h
 * ========
 * Lightweight FlySky iBus library — channel reading + sensor telemetry.
 *
 * No timers, no interrupts, no allocations.  Just call loop() frequently.
 *
 * ────────────────────────────────────────────────────────────────────
 *  FS-iA6B Receiver Ports
 * ────────────────────────────────────────────────────────────────────
 *
 *   iBus SERVO  — one-way, sends 14-channel data every ~7 ms
 *                 Connect RX to any GPIO.  TX not needed.
 *
 *   iBus SENSOR — bidirectional, also sends channel data + polls sensors
 *                 Connect BOTH TX and RX.  This is where telemetry
 *                 responses go back to the transmitter.
 *
 * You can use one HardwareSerial for channels only (SERVO port),
 * or one for channels + telemetry (SENSOR port), or two separate ones.
 *
 * ────────────────────────────────────────────────────────────────────
 *  Protocol Summary
 * ────────────────────────────────────────────────────────────────────
 *
 *  Channel frame (32 bytes, every ~7 ms):
 *    Byte 0:   0x20 (length = 32)
 *    Byte 1:   0x40 (command = channels)
 *    Bytes 2–29: 14 channels, little-endian uint16 each (1000–2000)
 *    Bytes 30–31: checksum (little-endian), 0xFFFF minus sum of bytes 0–29
 *
 *  Sensor discovery (4 bytes from receiver):
 *    0x04 | 0x81+addr | chk_lo | chk_hi
 *    → Respond with: 0x06 | 0x01+addr | type_lo | type_hi | chk_lo | chk_hi
 *
 *  Sensor measurement request (4 bytes from receiver):
 *    0x04 | 0xA1+addr | chk_lo | chk_hi
 *    → Respond with: 0x06 | 0x21+addr | val_lo | val_hi | chk_lo | chk_hi
 *
 * ────────────────────────────────────────────────────────────────────
 *  Sensor Types (for transmitter display)
 * ────────────────────────────────────────────────────────────────────
 *    0x00  Internal Voltage    (unit: 0.01 V)
 *    0x01  Temperature         (unit: 0.1 °C, value = (°C + 40) × 10)
 *    0x02  RPM                 (direct value)
 *    0x03  External Voltage    (unit: 0.01 V)
 *    0x04  Temperature 2       (same encoding as 0x01)
 *    0x41  Pressure            (unit: hPa)
 *    0x42  GPS status          (sat count)
 *
 * License: MIT
 */

#pragma once
#include <Arduino.h>

class IBusRC {
public:

    // ── Sensor type constants ────────────────────────────────────────────

    static constexpr uint16_t SENSOR_INTV   = 0x00;  ///< Internal voltage (0.01 V)
    static constexpr uint16_t SENSOR_TEMP   = 0x01;  ///< Temperature (0.1°C, +400 offset)
    static constexpr uint16_t SENSOR_RPM    = 0x02;  ///< RPM
    static constexpr uint16_t SENSOR_EXTV   = 0x03;  ///< External voltage (0.01 V)
    static constexpr uint16_t SENSOR_TEMP2  = 0x04;  ///< Temperature 2
    static constexpr uint16_t SENSOR_PRESS  = 0x41;  ///< Pressure (hPa)
    static constexpr uint16_t SENSOR_GPS    = 0x42;  ///< GPS sat count

    // ── Limits ───────────────────────────────────────────────────────────

    static constexpr uint8_t MAX_CHANNELS = 14;
    static constexpr uint8_t MAX_SENSORS  = 10;

    // ── Callback for dynamic sensor values ───────────────────────────────

    /**
     * User callback to provide sensor value on demand.
     * @param sensorIndex  Index 0..MAX_SENSORS-1 as registered
     * @return             Raw uint16 value (encoding depends on sensor type)
     */
    using SensorCallback = uint16_t (*)(uint8_t sensorIndex);

    // ── Construction ─────────────────────────────────────────────────────

    /**
     * @param channelSerial  Serial port connected to iBus SERVO or SENSOR port
     * @param sensorSerial   Serial port for sensor telemetry (can be same as
     *                       channelSerial if using the SENSOR port for both).
     *                       Pass nullptr to disable sensor telemetry.
     */
    IBusRC() = default;

    /**
     * Attach the channel (SERVO) serial port.
     * @param serial  HardwareSerial at 115200 baud
     * @param rxPin   RX GPIO (-1 = default)
     * @param txPin   TX GPIO (-1 = default, not needed for channel-only)
     */
    void beginChannels(HardwareSerial& serial, int8_t rxPin = -1, int8_t txPin = -1);

    /**
     * Attach the sensor (telemetry) serial port.
     * Can be the same HardwareSerial as channels if using the SENSOR port for both.
     * @param serial  HardwareSerial at 115200 baud (needs TX + RX)
     * @param rxPin   RX GPIO (-1 = default)
     * @param txPin   TX GPIO (-1 = default)
     */
    void beginSensor(HardwareSerial& serial, int8_t rxPin = -1, int8_t txPin = -1);

    // ── Sensor registration ──────────────────────────────────────────────

    /**
     * Register a telemetry sensor.
     * @param sensorType  SENSOR_INTV, SENSOR_TEMP, etc.
     * @param callback    Function called when receiver requests this sensor's value.
     *                    Receives the sensor index (0-based registration order).
     * @return            Sensor index (0-based), or -1 if full.
     */
    int8_t addSensor(uint16_t sensorType, SensorCallback callback);

    /**
     * Set a sensor's value directly (alternative to callbacks).
     * @param sensorIndex  Index returned by addSensor()
     * @param value        Raw uint16 value
     */
    void setSensorValue(uint8_t sensorIndex, uint16_t value);

    // ── Helpers for sensor value encoding ────────────────────────────────

    /** Encode voltage in volts to raw sensor value (0.01 V units). */
    static uint16_t encodeVoltage(float volts) {
        return (uint16_t)(volts * 100.0f + 0.5f);
    }

    /** Encode temperature in °C to raw sensor value (0.1°C, offset 400). */
    static uint16_t encodeTemperature(float celsius) {
        return (uint16_t)((celsius + 40.0f) * 10.0f + 0.5f);
    }

    /** Encode RPM directly (uint16). */
    static uint16_t encodeRPM(int rpm) {
        return (uint16_t)constrain(rpm, 0, 65535);
    }

    // ── Channel reading ──────────────────────────────────────────────────

    /**
     * Read a channel value.
     * @param ch  Channel index 0–13 (CH1=0, CH2=1, …)
     * @return    Channel value (typically 1000–2000, center ~1500).
     *            Returns 0 if no data received yet.
     */
    uint16_t channel(uint8_t ch) const {
        return (ch < MAX_CHANNELS) ? _channels[ch] : 0;
    }

    /** True once at least one valid channel frame has been received. */
    bool available() const { return _channelFrameCount > 0; }

    /** Milliseconds since the last valid channel frame. */
    uint32_t timeSinceLastFrame() const { return millis() - _lastFrameMs; }

    /** Total valid channel frames received. */
    uint32_t frameCount() const { return _channelFrameCount; }

    // ── Core loop — call as often as possible ────────────────────────────

    /**
     * Process incoming serial data.  Call this from loop() — the more
     * frequently the better.  Non-blocking, returns immediately.
     */
    void loop();

private:

    // ── Channel parser state ─────────────────────────────────────────────

    HardwareSerial* _chSerial  = nullptr;
    uint8_t  _chBuf[32];
    uint8_t  _chIdx            = 0;
    uint16_t _channels[MAX_CHANNELS] = {};
    uint32_t _lastFrameMs      = 0;
    uint32_t _channelFrameCount = 0;

    void _processChannelByte(uint8_t b);
    bool _validateChannelFrame();

    // ── Sensor responder state ───────────────────────────────────────────

    HardwareSerial* _snSerial  = nullptr;
    bool     _sensorSamePort   = false;   // true if sensor port == channel port

    struct SensorSlot {
        uint16_t type     = 0;
        uint16_t value    = 0;
        SensorCallback cb = nullptr;
        bool     active   = false;
    };
    SensorSlot _sensors[MAX_SENSORS];
    uint8_t    _sensorCount = 0;

    uint8_t  _snBuf[4];
    uint8_t  _snIdx = 0;

    void _processSensorByte(uint8_t b);
    void _handleSensorRequest();
    void _sendSensorReply(const uint8_t* data, uint8_t len);

    // ── Checksum ─────────────────────────────────────────────────────────

    static uint16_t _checksum(const uint8_t* data, uint8_t len);
};
