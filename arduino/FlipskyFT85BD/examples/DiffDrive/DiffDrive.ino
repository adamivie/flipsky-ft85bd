/**
 * DiffDrive.ino
 * =============
 * Serial-controlled differential drive example.
 *
 * Send commands from Serial Monitor (or any UART terminal) at 115200 baud:
 *   'f' = forward      'b' = back
 *   'l' = turn left    'r' = turn right
 *   's' = stop         '+' = increase speed   '-' = decrease speed
 *
 * ⚠  ESC Tool → Input Signal Type must be OFF on both channels.
 *
 * Wiring (ESP32):
 *   FT85BD GND → ESP32 GND
 *   FT85BD TX  → ESP32 GPIO16 (RX2)
 *   FT85BD RX  → ESP32 GPIO17 (TX2)
 */

#include <FlipskyFT85BD.h>

// ── Configuration ────────────────────────────────────────────────────────────
static constexpr int16_t  SLAVE_CAN_ID = 184;
static constexpr int32_t  ERPM_STEP    = 1000;   // eRPM increment per '+'/'-'
static constexpr int32_t  ERPM_MAX     = 10000;
static constexpr uint32_t CTRL_PERIOD  = 50;     // ms between speed commands
static constexpr uint32_t BAUD_RATE    = 460800;

FlipskyFT85BD esc(Serial2, BAUD_RATE, /*rx=*/16, /*tx=*/17);

int32_t  targetSpeed  = 2000;   // current speed magnitude (eRPM)
int32_t  leftErpm     = 0;
int32_t  rightErpm    = 0;
uint32_t lastCtrl     = 0;

void printHelp() {
    Serial.println("Commands: f=forward  b=back  l=left  r=right  s=stop  +=faster  -=slower");
    Serial.printf("Speed: %d eRPM\n", targetSpeed);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    esc.begin();
    Serial.println("FlipskyFT85BD – Diff Drive Example");
    printHelp();
}

void loop() {
    // ── Handle incoming commands ─────────────────────────────────────────────
    if (Serial.available()) {
        char ch = (char)Serial.read();
        switch (ch) {
            case 'f': leftErpm =  targetSpeed; rightErpm =  targetSpeed; break;
            case 'b': leftErpm = -targetSpeed; rightErpm = -targetSpeed; break;
            case 'l': leftErpm = -targetSpeed / 2; rightErpm =  targetSpeed / 2; break;
            case 'r': leftErpm =  targetSpeed / 2; rightErpm = -targetSpeed / 2; break;
            case 's': leftErpm = 0; rightErpm = 0; break;
            case '+': targetSpeed = min((int32_t)(targetSpeed + ERPM_STEP), ERPM_MAX); break;
            case '-': targetSpeed = max((int32_t)(targetSpeed - ERPM_STEP), ERPM_STEP); break;
            default: break;
        }
        Serial.printf("left=%+d  right=%+d  speed=%d eRPM\n",
                      leftErpm, rightErpm, targetSpeed);
    }

    // ── Send speed commands at CTRL_PERIOD rate ──────────────────────────────
    if (millis() - lastCtrl >= CTRL_PERIOD) {
        lastCtrl = millis();
        esc.setSpeed( leftErpm);               // master: left wheel forward = positive
        esc.setSpeed(-rightErpm, SLAVE_CAN_ID); // slave: right wheel is mechanically inverted
        esc.keepAlive();
    }
}
