/**
 * SpinTest.ino
 * ============
 * Ramp both motors to a target eRPM, hold for a set duration, then stop.
 *
 * ⚠  Lift the robot off the ground before running.
 * ⚠  ESC Tool → Input Signal Type must be OFF on both channels.
 *
 * Wiring (ESP32):
 *   FT85BD GND → ESP32 GND
 *   FT85BD TX  → ESP32 GPIO16 (RX2)
 *   FT85BD RX  → ESP32 GPIO17 (TX2)
 */

#include <FlipskyFT85BD.h>

// ── Configuration ────────────────────────────────────────────────────────────
static constexpr int16_t  SLAVE_CAN_ID  = 184;
static constexpr int32_t  TARGET_ERPM   = 5000;   // eRPM to run at
static constexpr uint32_t RUN_DURATION  = 5000;   // milliseconds
static constexpr uint32_t BAUD_RATE     = 460800;

FlipskyFT85BD esc(Serial2, BAUD_RATE, /*rx=*/16, /*tx=*/17);

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    esc.begin();

    Serial.println("FlipskyFT85BD – Spin Test");
    Serial.printf("Target: %d eRPM  Duration: %u ms\n", TARGET_ERPM, RUN_DURATION);
    Serial.println("Starting in 3 seconds…");
    delay(3000);

    Serial.println("Running!");
    uint32_t t_end = millis() + RUN_DURATION;

    while (millis() < t_end) {
        esc.setSpeed( TARGET_ERPM);               // master forward
        esc.setSpeed(-TARGET_ERPM, SLAVE_CAN_ID); // slave reverse (inverted)
        esc.keepAlive();

        FlipskyFT85BD::Telemetry master, slave;
        if (esc.getTelemetry(master)) {
            Serial.printf("[MASTER] eRPM=%d  duty=%.1f%%  V=%.2fV  Imot=%.2fA\n",
                master.erpm, master.duty * 100.0f,
                master.voltage_V, master.motor_current_A);
        }
        if (esc.getTelemetry(slave, SLAVE_CAN_ID)) {
            Serial.printf("[SLAVE ] eRPM=%d  duty=%.1f%%  V=%.2fV  Imot=%.2fA\n",
                slave.erpm, slave.duty * 100.0f,
                slave.voltage_V, slave.motor_current_A);
        }
        delay(200);
    }

    // Stop
    esc.stop();
    esc.stop(SLAVE_CAN_ID);
    Serial.println("\nStopped.");
}

void loop() {
    // Nothing after the one-shot test
}
