/**
 * LoopbackTest.ino
 * ================
 * UART loopback test for ESP32-S3 Serial2.
 * Wire GPIO17 (TX) → GPIO18 (RX) with a jumper.
 * Sends a known pattern and checks if it comes back.
 */

static constexpr int8_t RX_PIN = 18;
static constexpr int8_t TX_PIN = 17;
static constexpr uint32_t BAUD = 460800;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) {}

    Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
    delay(100);

    Serial.println("=== UART2 Loopback Test ===");
    Serial.printf("TX=GPIO%d  RX=GPIO%d  Baud=%lu\n", TX_PIN, RX_PIN, BAUD);
    Serial.println("Wire TX→RX with a jumper.\n");
}

void loop() {
    // Flush any stale bytes
    while (Serial2.available()) Serial2.read();

    // Send test pattern
    const char* pattern = "HELLO_ESC";
    uint8_t len = strlen(pattern);

    Serial2.write((const uint8_t*)pattern, len);
    Serial2.flush();  // wait for TX to complete

    // Read back
    delay(10);
    char buf[32] = {0};
    uint8_t got = 0;
    uint32_t t0 = millis();
    while (got < len && millis() - t0 < 100) {
        if (Serial2.available()) {
            buf[got++] = (char)Serial2.read();
        }
    }
    buf[got] = '\0';

    // Report
    if (got == len && memcmp(buf, pattern, len) == 0) {
        Serial.printf("PASS: sent \"%s\" got \"%s\" (%d bytes)\n", pattern, buf, got);
    } else if (got == 0) {
        Serial.printf("FAIL: sent \"%s\" got NOTHING (0 bytes) — check wiring\n", pattern);
    } else {
        Serial.printf("FAIL: sent \"%s\" got \"%s\" (%d/%d bytes)\n", pattern, buf, got, len);
        Serial.print("  Raw hex: ");
        for (uint8_t i = 0; i < got; i++) Serial.printf("%02X ", (uint8_t)buf[i]);
        Serial.println();
    }

    delay(1000);
}
