/**
 * FlipskyFT85BD.h
 * ===============
 * Arduino / ESP32 library for the Flipsky FT85BD dual ESC and any ESC
 * using the Flipsky FTESC UART V1.4 protocol.
 *
 * Protocol
 * --------
 * Frame:  0xAA | DLEN | CMD | DATA... | CRC_hi | CRC_lo | 0xDD
 * CRC:    Modbus CRC-16 (poly=0xA001, init=0xFFFF)
 *
 * Key commands
 *   0x00  OBTAIN_DATA_ONCE  – request 28-byte telemetry
 *   0x10  CAN_OVER          – forward a command to a slave via CAN
 *   0x19  KEEP_LIVE         – heartbeat (call every ≤500 ms while running)
 *   0x27  SET_SPEED         – int32 eRPM, big-endian
 *
 * Hardware prerequisite
 * ---------------------
 * Flipsky ESC Tool → App Config → Input Signal Type must be set to OFF
 * on every channel you want to control via UART.
 *
 * Wiring (FT85BD UART header)
 * --------
 *   ESC  GND  → MCU GND
 *   ESC  TX   → MCU RX
 *   ESC  RX   → MCU TX
 *   Baud rate : 460800
 *
 * Quick start
 * -----------
 * @code
 * #include <FlipskyFT85BD.h>
 *
 * // ESP32: Serial2 on pins RX=16, TX=17
 * FlipskyFT85BD esc(Serial2, 460800);
 *
 * void setup() {
 *   esc.begin();
 * }
 *
 * void loop() {
 *   FlipskyFT85BD::Telemetry t;
 *   if (esc.getTelemetry(t)) {
 *     Serial.printf("V=%.2f  eRPM=%d\n", t.voltage_V, t.erpm);
 *   }
 *   esc.setSpeed(3000);       // master forward 3000 eRPM
 *   esc.setSpeed(-3000, 184); // slave reverse  (CAN id 184)
 *   esc.keepAlive();
 *   delay(50);
 * }
 * @endcode
 *
 * License: MIT
 */

#pragma once
#include <Arduino.h>

class FlipskyFT85BD {
public:

    // ── Telemetry ────────────────────────────────────────────────────────────

    struct Telemetry {
        uint8_t  mcu_id;           ///< ESC's own CAN ID
        uint8_t  error_code;       ///< 0 = no error
        float    voltage_V;        ///< Bus voltage in volts
        float    batt_current_A;   ///< Battery current in amperes
        float    motor_current_A;  ///< Motor phase current in amperes
        int32_t  erpm;             ///< Electrical RPM (positive = forward)
        float    duty;             ///< Duty cycle  0.0 – 1.0
        float    temp_fet_C;       ///< MOSFET temperature °C
        float    temp_motor_C;     ///< Motor temperature °C
        float    cpu_load_pct;     ///< MCU CPU load %

        /** Return a human-readable error name. */
        const char* errorName() const;

        /** True when error_code == 0. */
        bool ok() const { return error_code == 0; }
    };

    // ── Error codes ──────────────────────────────────────────────────────────

    static constexpr uint8_t ERR_NONE                 =  0;
    static constexpr uint8_t ERR_PHASE_A_OVER_CURRENT =  1;
    static constexpr uint8_t ERR_PHASE_B_OVER_CURRENT =  2;
    static constexpr uint8_t ERR_PHASE_C_OVER_CURRENT =  3;
    static constexpr uint8_t ERR_PHASE_A_SENSOR       =  4;
    static constexpr uint8_t ERR_PHASE_B_SENSOR       =  5;
    static constexpr uint8_t ERR_PHASE_C_SENSOR       =  6;
    static constexpr uint8_t ERR_PHASE_SUM_NOT_ZERO   =  7;
    static constexpr uint8_t ERR_BUS_UNDER_VOLTAGE    =  8;
    static constexpr uint8_t ERR_BUS_OVER_VOLTAGE     =  9;
    static constexpr uint8_t ERR_MOSFET_OVERHEAT      = 10;
    static constexpr uint8_t ERR_MOTOR_OVERHEAT       = 11;
    static constexpr uint8_t ERR_MOSFET_TEMP_SENSOR   = 12;
    static constexpr uint8_t ERR_WATCHDOG_RESET       = 13;
    static constexpr uint8_t ERR_FLASH_CORRUPT        = 14;
    static constexpr uint8_t ERR_MCU_UNDER_VOLTAGE    = 15;
    static constexpr uint8_t ERR_MOTOR_TEMP_SENSOR    = 16;
    static constexpr uint8_t ERR_MOTOR_BLOCKED        = 17;
    static constexpr uint8_t ERR_DRIVER_FAULT         = 18;
    static constexpr uint8_t ERR_PHASE_LOSS           = 19;
    static constexpr uint8_t ERR_BUS_OVER_CURRENT     = 20;

    // ── Construction ─────────────────────────────────────────────────────────

    /**
     * @param serial        HardwareSerial port (e.g. Serial1, Serial2)
     * @param baud          Baud rate (FT85BD default: 460800)
     * @param rxPin         RX pin override for ESP32 (-1 = use default)
     * @param txPin         TX pin override for ESP32 (-1 = use default)
     * @param readTimeoutMs Milliseconds to wait for a telemetry reply
     */
    explicit FlipskyFT85BD(HardwareSerial& serial,
                            uint32_t baud          = 460800,
                            int8_t   rxPin         = -1,
                            int8_t   txPin         = -1,
                            uint16_t readTimeoutMs = 150);

    /** Initialise the serial port.  Call from setup(). */
    void begin();

    // ── Control API ──────────────────────────────────────────────────────────

    /**
     * Request telemetry from master (can_id=-1) or a CAN slave.
     *
     * @param t      Output telemetry struct (filled on success).
     * @param can_id -1 = master; 0–254 = slave CAN id.
     * @return true on valid response, false on timeout or CRC error.
     */
    bool getTelemetry(Telemetry& t, int16_t can_id = -1);

    /**
     * Command a speed in electrical RPM.
     *
     * @param erpm   Target eRPM (positive = forward, negative = reverse).
     * @param can_id -1 = master; 0–254 = slave CAN id.
     */
    void setSpeed(int32_t erpm, int16_t can_id = -1);

    /** Command zero speed (stop). */
    void stop(int16_t can_id = -1) { setSpeed(0, can_id); }

    /**
     * Send heartbeat packet.
     * Must be called at least every 500 ms while motors are running,
     * otherwise the ESC will cut power.
     */
    void keepAlive();

private:
    HardwareSerial& _serial;
    uint32_t  _baud;
    int8_t    _rxPin;
    int8_t    _txPin;
    uint16_t  _readTimeoutMs;

    // frame buffer (max payload 8 bytes + 4 overhead = 12)
    static constexpr uint8_t FRAME_BUF = 16;
    uint8_t _buf[FRAME_BUF];

    static uint16_t _crc16(const uint8_t* data, uint8_t len);

    /**
     * Build a complete FTESC UART frame into _buf.
     * @param info     Info payload bytes (CMD + DATA)
     * @param info_len Length of info payload
     * @return         Total frame length in bytes
     */
    uint8_t _buildFrame(const uint8_t* info, uint8_t info_len);

    /**
     * Send request, accumulate bytes until a valid frame is found.
     * @param req      Request frame bytes
     * @param req_len  Length of request frame
     * @param t        Output Telemetry struct
     * @return true on success
     */
    bool _query(const uint8_t* req, uint8_t req_len, Telemetry& t);

    /** Parse a telemetry info payload (starting with CMD byte). */
    bool _parseTelemetry(const uint8_t* info, uint8_t info_len, Telemetry& t);
};
