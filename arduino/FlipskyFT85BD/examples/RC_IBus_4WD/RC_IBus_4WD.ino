/**
 * RC_IBus_4WD.ino
 * ================
 * RC receiver (FlySky iBus) → ESP32-S3 → 4× Flipsky FT85BD ESC
 *
 * Uses the IBusRC library (no timers, no third-party deps) to read
 * iBus channels + send sensor telemetry back to the transmitter.
 *
 * ────────────────────────────────────────────────────────────────────
 *  WIRING  (ESP32-S3)
 * ────────────────────────────────────────────────────────────────────
 *
 *   FS-iA6B Receiver
 *   ┌─────────────────┐
 *   │  iBus SERVO     │── Signal ──→ GPIO4   (Serial1 RX)   channels
 *   │  iBus SENSOR    │── Signal ──→ GPIO5   (Serial0 RX)   telemetry
 *   │                 │── Signal ←── GPIO6   (Serial0 TX)   telemetry
 *   │  VCC            │── 5V (from ESC UART header)
 *   │  GND            │── GND (common)
 *   └─────────────────┘
 *
 *   FT85BD #1 (UART)       (Master:164 + Slave:115)
 *   ┌─────────────────┐
 *   │  UART TX        │──→ GPIO16  (Serial2 RX)
 *   │  UART RX        │←── GPIO17  (Serial2 TX)
 *   │  5V             │──→ ESP32 VIN + iA6B VCC
 *   │  GND            │──→ common GND
 *   └────┬────────────┘
 *        │ CAN bus
 *   ┌────┴────────────┐
 *   │  FT85BD #2      │    (Slave:184 + Slave:251)  — CAN only
 *   └─────────────────┘
 *
 * ────────────────────────────────────────────────────────────────────
 *  TELEMETRY TO TRANSMITTER
 * ────────────────────────────────────────────────────────────────────
 *   Sensor 0: Battery voltage   (shows on TX as "IntV")
 *   Sensor 1: FET temperature   (shows on TX as "Temp")
 *   Sensor 2: Motor RPM         (shows on TX as "RPM")
 *
 *   On the transmitter: Setup → Sensors → Discover sensors
 *
 * ────────────────────────────────────────────────────────────────────
 *  RC CHANNEL MAPPING
 * ────────────────────────────────────────────────────────────────────
 *   CH1 (right stick H) → Steering
 *   CH2 (right stick V) → Throttle
 *   CH5 (switch)        → Kill switch (↓ = kill)
 *
 * ────────────────────────────────────────────────────────────────────
 *  PREREQUISITES
 * ────────────────────────────────────────────────────────────────────
 *   1. ESC Tool → App Config → Input Signal Type = OFF on all channels
 *   2. ESC Tool → Motor Settings → Max ERPM = 6000 on all channels
 *
 * License: MIT
 */

#include <IBusRC.h>
#include <FlipskyFT85BD.h>

// ── Motor CAN IDs ────────────────────────────────────────────────────────────
static constexpr int16_t LEFT_FRONT  = -1;   // master 164
static constexpr int16_t LEFT_REAR   = 115;
static constexpr int16_t RIGHT_FRONT = 184;
static constexpr int16_t RIGHT_REAR  = 251;

static constexpr int16_t LEFT_MOTORS[]  = { LEFT_FRONT,  LEFT_REAR  };
static constexpr int16_t RIGHT_MOTORS[] = { RIGHT_FRONT, RIGHT_REAR };

// Set to -1 or +1 to flip direction per side
static constexpr int LEFT_DIR  =  1;
static constexpr int RIGHT_DIR = -1;  // motors face opposite way

// ── Tuning ───────────────────────────────────────────────────────────────────
static constexpr int32_t  MAX_ERPM       = 6000;
static constexpr uint16_t DEADZONE       = 40;     // iBus center ±40
static constexpr uint32_t CTRL_PERIOD_MS = 50;     // 20 Hz command rate
static constexpr uint32_t ESC_BAUD       = 460800;
static constexpr uint32_t IBUS_TIMEOUT   = 500;    // ms failsafe

// ── iBus channel indices (0-based) ───────────────────────────────────────────
static constexpr uint8_t CH_STEER    = 0;  // CH1
static constexpr uint8_t CH_THROTTLE = 1;  // CH2
static constexpr uint8_t CH_KILL     = 4;  // CH5

// ── Pin assignments (ESP32-S3) ───────────────────────────────────────────────
static constexpr int8_t IBUS_CH_RX   = 4;   // Serial1 RX ← iA6B SERVO signal
static constexpr int8_t IBUS_SN_RX   = 5;   // Serial0 RX ← iA6B SENSOR signal
static constexpr int8_t IBUS_SN_TX   = 6;   // Serial0 TX → iA6B SENSOR signal
static constexpr int8_t ESC_RX       = 18;  // Serial2 RX ← FT85BD TX
static constexpr int8_t ESC_TX       = 17;  // Serial2 TX → FT85BD RX

// ── Objects ──────────────────────────────────────────────────────────────────
IBusRC rc;
FlipskyFT85BD esc(Serial2, ESC_BAUD, ESC_RX, ESC_TX);

uint32_t lastCtrl = 0;

// ── Telemetry state (updated each loop) ──────────────────────────────────────
static float     g_voltage = 0;
static float     g_tempFet = 0;
static int32_t   g_erpm    = 0;

// ── Sensor callbacks ─────────────────────────────────────────────────────────
uint16_t onVoltage(uint8_t) { return IBusRC::encodeVoltage(g_voltage); }
uint16_t onTemp(uint8_t)    { return IBusRC::encodeTemperature(g_tempFet); }
uint16_t onRPM(uint8_t)     { return IBusRC::encodeRPM(abs(g_erpm)); }

// ── Helpers ──────────────────────────────────────────────────────────────────

int32_t ibusToErpm(uint16_t raw) {
    int16_t centered = (int16_t)raw - 1500;
    if (abs(centered) < (int16_t)DEADZONE) return 0;
    int16_t sign  = (centered > 0) ? 1 : -1;
    int16_t mag   = abs(centered) - DEADZONE;
    int16_t range = 500 - DEADZONE;
    return (int32_t)sign * ((int32_t)mag * MAX_ERPM / range);
}

void stopAll() {
    for (auto id : LEFT_MOTORS)  esc.stop(id);
    for (auto id : RIGHT_MOTORS) esc.stop(id);
    esc.keepAlive();
}

// ── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);  // USB-CDC debug
    while (!Serial && millis() < 2000) {}

    // iBus channels on Serial1 (RX only from SERVO port)
    rc.beginChannels(Serial1, IBUS_CH_RX);

    // iBus sensor telemetry on Serial0 (TX+RX to SENSOR port)
    rc.beginSensor(Serial0, IBUS_SN_RX, IBUS_SN_TX);

    // Register telemetry sensors (visible on transmitter after discovery)
    rc.addSensor(IBusRC::SENSOR_INTV, onVoltage);  // Sensor 0: battery voltage
    rc.addSensor(IBusRC::SENSOR_TEMP, onTemp);      // Sensor 1: FET temperature
    rc.addSensor(IBusRC::SENSOR_RPM,  onRPM);       // Sensor 2: motor RPM

    esc.begin();

    Serial.println("RC iBus 4WD + Telemetry");
    Serial.println("Waiting for RC signal...");
}

// ── Main loop ────────────────────────────────────────────────────────────────

void loop() {
    // Process iBus data (channels + sensor responses) — call often!
    rc.loop();

    if (millis() - lastCtrl < CTRL_PERIOD_MS) return;
    lastCtrl = millis();

    // ── Read RC channels ─────────────────────────────────────────────────
    uint16_t steerRaw    = rc.channel(CH_STEER);
    uint16_t throttleRaw = rc.channel(CH_THROTTLE);
    uint16_t killRaw     = rc.channel(CH_KILL);

    bool hasSignal = rc.available() && rc.timeSinceLastFrame() < IBUS_TIMEOUT;

    // ── Kill switch / failsafe ───────────────────────────────────────────
    bool killed = !hasSignal || (killRaw > 0 && killRaw < 1500);

    if (killed) {
        stopAll();
        static uint32_t lastWarn = 0;
        if (millis() - lastWarn > 1000) {
            Serial.println(hasSignal ? "KILLED by switch" : "NO RC SIGNAL");
            lastWarn = millis();
        }
        return;
    }

    // ── Mix throttle + steering ──────────────────────────────────────────
    int32_t throttle = ibusToErpm(throttleRaw);
    int32_t steer    = ibusToErpm(steerRaw);

    int32_t leftErpm  = constrain(throttle + steer, -MAX_ERPM, MAX_ERPM) * LEFT_DIR;
    int32_t rightErpm = constrain(throttle - steer, -MAX_ERPM, MAX_ERPM) * RIGHT_DIR;

    // ── Send to all 4 ESCs ──────────────────────────────────────────────
    for (auto id : LEFT_MOTORS)  esc.setSpeed(leftErpm,  id);
    for (auto id : RIGHT_MOTORS) esc.setSpeed(rightErpm, id);
    esc.keepAlive();

    // ── Read telemetry from master (for TX sensor display) ──────────────
    static uint32_t telOk = 0, telFail = 0;
    FlipskyFT85BD::Telemetry t;
    if (esc.getTelemetry(t)) {
        g_voltage = t.voltage_V;
        g_tempFet = t.temp_fet_C;
        g_erpm    = t.erpm;
        telOk++;
    } else {
        telFail++;
    }

    // ── Debug output ────────────────────────────────────────────────────
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 250) {
        lastPrint = millis();
        Serial.printf("THR=%4d  STR=%4d  CH5=%4d  → L=%+6d  R=%+6d  V=%.1f  T=%.1f°C  ESC[ok=%lu fail=%lu rx=%d]\n",
                      throttleRaw, steerRaw, killRaw, leftErpm, rightErpm,
                      g_voltage, g_tempFet, telOk, telFail, Serial2.available());
    }
}
