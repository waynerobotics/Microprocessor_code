// 00_base — Warrior base controller sketch
// Target: Arduino Uno
//
// Hardware wiring assumed:
//   Pin 6  → NeoPixel data line (3 fans × 20 LEDs = 60 total)
//   Pin 9  → Fan 0 PWM speed signal (PWM-capable on Uno)
//   Pin 10 → Fan 1 PWM speed signal
//   Pin 11 → Fan 2 PWM speed signal
//
// Dependencies (install via Library Manager):
//   - Adafruit NeoPixel

#include <FanControl.h>

// --- Configuration -------------------------------------------------------
#define LED_PIN      6
#define NUM_FANS     3
#define LEDS_PER_FAN 20

#define FAN0_PWM_PIN  9
#define FAN1_PWM_PIN  10
#define FAN2_PWM_PIN  11

// -------------------------------------------------------------------------
FanControl fans(LED_PIN, NUM_FANS, LEDS_PER_FAN);

void setup() {
    Serial.begin(115200);

    fans.begin();
    fans.setBrightness(60);

    // Register PWM pins for each fan's motor controller.
    // Remove/comment any fan that does not have a speed controller wired up.
    fans.setSpeedPin(0, FAN0_PWM_PIN);
    fans.setSpeedPin(1, FAN1_PWM_PIN);
    fans.setSpeedPin(2, FAN2_PWM_PIN);

    // Start fans at 50% speed
    fans.setAllSpeeds(50);

    Serial.println("FanControl ready.");
}

void loop() {
    // Non-blocking moving-block LED animation (advances every 40 ms)
    fans.updateAnimation(40);

    // --- Example: change speed based on Serial input --------------------
    // Send a number 0-100 over Serial to set all fan speeds.
    if (Serial.available()) {
        int speed = Serial.parseInt();
        if (speed >= 0 && speed <= 100) {
            fans.setAllSpeeds((uint8_t)speed);
            Serial.print("Fan speed set to ");
            Serial.print(speed);
            Serial.println("%");
        }
    }
}
