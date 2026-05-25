#include <Arduino.h>
#include <FanControl.h>

// =====================================================================
// 02_fan — ARGB fan bring-up on Mega 2560.
// ARGB data line wired to digital pin 7.
// 3 fans x 20 LEDs each, daisy-chained on one data pin.
// Animation: alternating green/yellow fans with a moving block.
// =====================================================================

const uint8_t FAN_LED_PIN  = 7;
const uint8_t FAN_COUNT    = 3;
const uint8_t LEDS_PER_FAN = 20;

FanControl fans(FAN_LED_PIN, FAN_COUNT, LEDS_PER_FAN);

void setup() {
    fans.begin();
}

void loop() {
    fans.updateAnimation();
}
