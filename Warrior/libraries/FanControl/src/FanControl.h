#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define FANCONTROL_MAX_FANS 8

// FanControl - LED ring and PWM fan speed control
// Compatible with Arduino Uno (AVR) and ESP32.
//
// LED rings: controlled via a single NeoPixel data pin (NEO_GRB, 800 KHz).
// Fan speed: controlled via PWM pins (analogWrite, 0-255 -> 0-100%).
class FanControl {
public:
    // ledPin:     NeoPixel data pin
    // numFans:    number of fans
    // ledsPerFan: number of LEDs on each fan ring
    FanControl(uint8_t ledPin, uint8_t numFans, uint8_t ledsPerFan);

    // Call once in setup(). Initialises strip and turns all LEDs off.
    void begin();

    // Overall strip brightness: 0 (off) to 255 (full). Default: 60.
    void setBrightness(uint8_t brightness);

    // --- Speed Control (PWM) -------------------------------------------
    // Register a PWM-capable pin for a fan's motor controller.
    // Must be called before setSpeed() for that fan.
    void setSpeedPin(uint8_t fanIndex, uint8_t pin);

    // Set one fan's speed. percent: 0 (off) to 100 (full).
    void setSpeed(uint8_t fanIndex, uint8_t percent);

    // Set all registered fans to the same speed.
    void setAllSpeeds(uint8_t percent);

    // --- LED Control ----------------------------------------------------
    // Set every LED on one fan ring to a solid colour.
    void setFanColor(uint8_t fanIndex, uint8_t r, uint8_t g, uint8_t b);

    // Default alternating green/yellow background across all fans.
    void setDefaultBackground();

    // Set a single LED by its absolute index in the strip.
    void setLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

    // Turn off all LEDs (does not call show()).
    void clear();

    // Push buffered LED data to the strip.
    void show();

    // --- Animation ------------------------------------------------------
    // Non-blocking moving-block animation. Call every iteration of loop().
    // Advances one frame per frameDelayMs milliseconds.
    void updateAnimation(uint16_t frameDelayMs = 40);

private:
    Adafruit_NeoPixel _strip;
    uint8_t  _numFans;
    uint8_t  _ledsPerFan;
    uint16_t _totalLEDs;
    int      _animHead;
    unsigned long _lastFrameMs;

    uint8_t _speedPins[FANCONTROL_MAX_FANS];
    bool    _hasSpeedPin[FANCONTROL_MAX_FANS];
};
