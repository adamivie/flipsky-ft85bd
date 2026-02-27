/**
 * Telemetry.ino
 * =============
 * Print live telemetry from both ESC channels over USB Serial at 2 Hz.
 *
 * Wiring (ESP32 example):
 *   FT85BD GND → ESP32 GND
 *   FT85BD TX  → ESP32 GPIO16 (RX2)
 *   FT85BD RX  → ESP32 GPIO17 (TX2)
 *
 * For Arduino Mega use Serial1/Serial2/Serial3 — adjust port accordingly.
 */

#include <FlipskyFT85BD.h>

// ── Configuration ────────────────────────────────────────────────────────────
static constexpr int16_t  SLAVE_CAN_ID = 184;   // set to your slave CAN id
static constexpr uint32_t BAUD_RATE    = 460800;

// ESP32: Serial2, with explicit pin assignment
// Arduino Mega: swap Serial2 for Serial1/Serial2/Serial3, remove pin args
FlipskyFT85BD esc(Serial2, BAUD_RATE, /*rx=*/16, /*tx=*/17);

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    esc.begin();
    Serial.println("FlipskyFT85BD – Telemetry example");
}

void loop() {
    FlipskyFT85BD::Telemetry master, slave;

    Serial.print("[MASTER] ");
    if (esc.getTelemetry(master)) {
        Serial.printf("MCU=%d  err=%s  V=%.2fV  Imot=%.2fA  eRPM=%d  duty=%.1f%%  Tfet=%.1fC\n",
            master.mcu_id, master.errorName(),
            master.voltage_V, master.motor_current_A,
            master.erpm, master.duty * 100.0f,
            master.temp_fet_C);
    } else {
        Serial.println("no response");
    }

    Serial.printf("[SLAVE CAN=%d] ", SLAVE_CAN_ID);
    if (esc.getTelemetry(slave, SLAVE_CAN_ID)) {
        Serial.printf("MCU=%d  err=%s  V=%.2fV  Imot=%.2fA  eRPM=%d  duty=%.1f%%  Tfet=%.1fC\n",
            slave.mcu_id, slave.errorName(),
            slave.voltage_V, slave.motor_current_A,
            slave.erpm, slave.duty * 100.0f,
            slave.temp_fet_C);
    } else {
        Serial.println("no response");
    }

    Serial.println();
    delay(500);
}
