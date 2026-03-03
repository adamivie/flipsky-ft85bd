/**
 * ESC_UartTest.ino
 * ================
 * Comprehensive UART debug for FT85BD ESC.
 * Tries multiple baud rates, passive listen, and active request.
 *
 * Wiring:
 *   ESP32 GPIO17 (RX) ← ESC TX
 *   ESP32 GPIO18 (TX) → ESC RX
 *   GND ↔ GND
 */

#include <FlipskyFT85BD.h>

static constexpr int8_t  RX_PIN = 17;
static constexpr int8_t  TX_PIN = 18;

static const uint32_t BAUDS[] = { 460800, 115200, 9600 };
static constexpr int NUM_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("=== ESC UART Comprehensive Test ===");
    Serial.printf("RX=GPIO%d  TX=GPIO%d\n\n", RX_PIN, TX_PIN);
}

void tryBaud(uint32_t baud) {
    Serial.printf("──── Trying %lu baud ────\n", baud);

    Serial2.end();
    delay(10);
    Serial2.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(50);

    // ── Passive listen: any bytes already coming? ────────────────────
    int stale = 0;
    while (Serial2.available()) { Serial2.read(); stale++; }
    if (stale > 0) Serial.printf("  Flushed %d stale bytes\n", stale);

    Serial.print("  Passive listen (500ms)... ");
    uint8_t buf[128];
    int got = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 500 && got < (int)sizeof(buf)) {
        if (Serial2.available()) buf[got++] = Serial2.read();
    }
    if (got > 0) {
        Serial.printf("got %d bytes: ", got);
        for (int i = 0; i < min(got, 32); i++) Serial.printf("%02X ", buf[i]);
        if (got > 32) Serial.print("...");
        Serial.println();
    } else {
        Serial.println("silence");
    }

    // ── Active: send OBTAIN_DATA via library ─────────────────────────
    FlipskyFT85BD esc(Serial2, baud, -1, -1, 300);  // pins already set
    // Don't call begin() — Serial2 is already open

    Serial.print("  getTelemetry()... ");
    FlipskyFT85BD::Telemetry t;
    bool ok = esc.getTelemetry(t);
    if (ok) {
        Serial.println("OK!");
        Serial.printf("    V=%.2f  eRPM=%ld  FET=%.1f°C  ID=%d  err=%s\n",
                      t.voltage_V, t.erpm, t.temp_fet_C, t.mcu_id, t.errorName());
    } else {
        Serial.println("FAIL");
    }

    // ── Raw send + dump ──────────────────────────────────────────────
    while (Serial2.available()) Serial2.read();  // flush

    uint8_t req[] = { 0xAA, 0x01, 0x00, 0x40, 0xBF, 0xDD };
    Serial2.write(req, sizeof(req));
    Serial2.flush();

    got = 0;
    t0 = millis();
    while (millis() - t0 < 300 && got < (int)sizeof(buf)) {
        if (Serial2.available()) buf[got++] = Serial2.read();
    }

    Serial.printf("  Raw probe: sent 6B, got %d bytes", got);
    if (got > 0) {
        Serial.print(": ");
        for (int i = 0; i < min(got, 40); i++) Serial.printf("%02X ", buf[i]);
        if (got > 40) Serial.print("...");
    }
    Serial.println();
    Serial.println();
}

void loop() {
    for (int i = 0; i < NUM_BAUDS; i++) {
        tryBaud(BAUDS[i]);
    }
    Serial.println("=== Cycle complete. Waiting 5s... ===\n");
    delay(5000);
}
