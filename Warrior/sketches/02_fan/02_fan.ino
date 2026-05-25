#include <Arduino.h>
#include <FanControl.h>

// =====================================================================
// 02_fan — ARGB fan bring-up on Mega 2560.
// ARGB data line wired to digital pin 7.
// Fan PWM (4-pin fan control wire) wired to digital pin 4.
// 3 fans x 20 LEDs each, daisy-chained on one data pin.
//
// Animation: 5-color band chase (red/yellow/blue/purple/teal) that
// walks around the chain. Alternates between a fast strobe (rapid
// on/off) and a slower flash (longer on/off) every few seconds.
//
// Fan speed: continuously sweeps 0% -> 100% -> 0% via analogWrite.
// Note: pin 4 PWM is ~976 Hz, not the 25 kHz PC-fan spec. Most fans
// tolerate it; if yours whines, move to a Timer 1/3/4/5 pin and
// reconfigure the timer for ~25 kHz.
// =====================================================================

const uint8_t  FAN_LED_PIN  = 7;
const uint8_t  FAN_PWM_PIN  = 4;
const uint8_t  FAN_COUNT    = 3;
const uint8_t  LEDS_PER_FAN = 20;
const uint16_t TOTAL_LEDS   = FAN_COUNT * LEDS_PER_FAN;  // 60

FanControl fans(FAN_LED_PIN, FAN_COUNT, LEDS_PER_FAN);

struct Color { uint8_t r, g, b; };
const Color PALETTE[5] = {
    {255,   0,   0},  // red
    {255, 255,   0},  // yellow
    {  0,   0, 255},  // blue
    {128,   0, 128},  // purple
    {  0, 128, 128},  // teal
};
const uint16_t BAND_LEN = TOTAL_LEDS / 5;  // 12 LEDs per color band

// Timing
const uint16_t CHASE_FRAME_MS   = 60;    // band shift cadence
const uint16_t STROBE_MS        = 60;    // fast on/off half-period
const uint16_t FLASH_MS         = 400;   // slow on/off half-period
const uint32_t MODE_DURATION_MS = 3000;  // strobe<->flash swap interval
const uint16_t SWEEP_STEP_MS    = 30;    // 30ms/step * 255 steps ~= 7.6s 0->100%

void renderChase(uint16_t offset) {
    for (uint16_t i = 0; i < TOTAL_LEDS; i++) {
        uint16_t pos = (i + offset) % TOTAL_LEDS;
        const Color& c = PALETTE[(pos / BAND_LEN) % 5];
        fans.setLED(i, c.r, c.g, c.b);
    }
    fans.show();
}

void renderOff() {
    fans.clear();
    fans.show();
}

void updateFanSpeed() {
    static uint32_t lastStepMs = 0;
    static int16_t  duty = 0;
    static int8_t   dir  = 1;

    if (millis() - lastStepMs < SWEEP_STEP_MS) return;
    lastStepMs = millis();

    duty += dir;
    if (duty >= 255) { duty = 255; dir = -1; }
    else if (duty <= 0) { duty = 0; dir = 1; }

    analogWrite(FAN_PWM_PIN, (uint8_t)duty);
}

void setup() {
    fans.begin();
    pinMode(FAN_PWM_PIN, OUTPUT);
    analogWrite(FAN_PWM_PIN, 0);
}

void loop() {
    static uint32_t lastChaseMs = 0;
    static uint32_t lastBlinkMs = 0;
    static uint32_t lastModeMs  = 0;
    static uint16_t chaseOffset = 0;
    static bool     blinkOn     = true;
    static bool     strobeMode  = true;  // true = strobe, false = flash

    uint32_t now = millis();
    bool dirty = false;

    if (now - lastModeMs >= MODE_DURATION_MS) {
        lastModeMs = now;
        strobeMode = !strobeMode;
    }

    uint16_t blinkRate = strobeMode ? STROBE_MS : FLASH_MS;
    if (now - lastBlinkMs >= blinkRate) {
        lastBlinkMs = now;
        blinkOn = !blinkOn;
        dirty = true;
    }

    if (now - lastChaseMs >= CHASE_FRAME_MS) {
        lastChaseMs = now;
        chaseOffset = (chaseOffset + 1) % TOTAL_LEDS;
        if (blinkOn) dirty = true;
    }

    if (dirty) {
        if (blinkOn) renderChase(chaseOffset);
        else         renderOff();
    }

    updateFanSpeed();
}
