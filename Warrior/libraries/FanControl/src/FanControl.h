#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// FanControl — LED ring animations for fan-mounted NeoPixel rings.
// (Fan speed / PWM control intentionally NOT included — that's wired to ESCs
// elsewhere in the stack. This library only drives the lights.)
//
// One NeoPixel data pin drives a chain of `numFans * ledsPerFan` pixels.
// Default behaviour: alternating green/yellow background with a moving
// block that walks around the chain — ported from FAN_CODE.ino.
//
// Compatible with AVR (Uno, Mega) and ESP32. Non-blocking: call
// updateAnimation() every iteration of loop().

class FanControl {
public:
    // ledPin     — NeoPixel data pin (any digital pin)
    // numFans    — number of fans on the chain
    // ledsPerFan — LEDs in each fan ring
    FanControl(uint8_t ledPin, uint8_t numFans, uint8_t ledsPerFan);

    // Call once in setup(). Initializes the strip and turns it off.
    void begin();

    // Overall strip brightness, 0..255. Default: 60.
    void setBrightness(uint8_t brightness);

    // --- LED control ---------------------------------------------------
    // Solid colour for one fan ring.
    void setFanColor(uint8_t fanIndex, uint8_t r, uint8_t g, uint8_t b);

    // Default alternating green/yellow background across all fans.
    void setDefaultBackground();

    // Set a single LED by absolute strip index.
    void setLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

    // Turn off all LEDs (does not call show()).
    void clear();

    // Push buffered LED data to the strip.
    void show();

    // --- Animation -----------------------------------------------------
    // Non-blocking moving-block animation. Call every loop() iteration —
    // it advances one frame per frameDelayMs and returns immediately
    // otherwise.
    void updateAnimation(uint16_t frameDelayMs = 40);

private:
    Adafruit_NeoPixel _strip;
    uint8_t  _numFans;
    uint8_t  _ledsPerFan;
    uint16_t _totalLEDs;
    uint16_t _animHead;
    unsigned long _lastFrameMs;
};
