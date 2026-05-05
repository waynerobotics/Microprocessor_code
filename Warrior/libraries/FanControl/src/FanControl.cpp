#include "FanControl.h"

FanControl::FanControl(uint8_t ledPin, uint8_t numFans, uint8_t ledsPerFan)
    : _strip((uint16_t)(numFans * ledsPerFan), ledPin, NEO_GRB + NEO_KHZ800),
      _numFans(numFans),
      _ledsPerFan(ledsPerFan),
      _totalLEDs((uint16_t)(numFans * ledsPerFan)),
      _animHead(0),
      _lastFrameMs(0)
{}

void FanControl::begin() {
    _strip.begin();
    _strip.setBrightness(60);
    _strip.show();
}

void FanControl::setBrightness(uint8_t brightness) {
    _strip.setBrightness(brightness);
}

// --- LED Control ---------------------------------------------------------

void FanControl::setFanColor(uint8_t fanIndex, uint8_t r, uint8_t g, uint8_t b) {
    if (fanIndex >= _numFans) return;
    uint16_t start = (uint16_t)(fanIndex * _ledsPerFan);
    _strip.fill(_strip.Color(r, g, b), start, _ledsPerFan);
}

void FanControl::setDefaultBackground() {
    uint32_t green  = _strip.Color(0, 255, 0);
    uint32_t yellow = _strip.Color(255, 255, 0);
    for (uint8_t f = 0; f < _numFans; f++) {
        uint16_t start = (uint16_t)(f * _ledsPerFan);
        uint32_t c = (f % 2 == 0) ? green : yellow;
        _strip.fill(c, start, _ledsPerFan);
    }
}

void FanControl::setLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= _totalLEDs) return;
    _strip.setPixelColor(index, _strip.Color(r, g, b));
}

void FanControl::clear() {
    _strip.clear();
}

void FanControl::show() {
    _strip.show();
}

// --- Animation -----------------------------------------------------------

void FanControl::updateAnimation(uint16_t frameDelayMs) {
    unsigned long now = millis();
    if ((now - _lastFrameMs) < (unsigned long)frameDelayMs) return;
    _lastFrameMs = now;

    uint32_t green  = _strip.Color(0, 255, 0);
    uint32_t yellow = _strip.Color(255, 255, 0);

    // Background, then overlay the moving block (one fan-ring long).
    setDefaultBackground();

    const uint8_t blockLen = _ledsPerFan;
    for (uint8_t k = 0; k < blockLen; k++) {
        uint16_t idx    = (uint16_t)((_animHead + k) % _totalLEDs);
        uint8_t  fanIdx = (uint8_t)(idx / _ledsPerFan);
        // Block uses the opposite colour of the fan it's currently on.
        uint32_t blockColor = (fanIdx % 2 == 0) ? yellow : green;
        _strip.setPixelColor(idx, blockColor);
    }

    _strip.show();
    _animHead = (uint16_t)((_animHead + 1) % _totalLEDs);
}
