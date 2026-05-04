// RadioLink.h
#pragma once
#include <Arduino.h>

// Reads an SBUS frame stream from a HardwareSerial port and exposes the
// 8 main channels as a ControllerState with semantic names (aileron,
// elevator, throttle, rudder, switchA/B, buttonVRB, knobVRA).
//
// Stick channels are mapped to a normalized -100..100 with a deadband
// applied around center. Calibration is per-channel — measure the actual
// min/center/max your transmitter sends and call setChannelCalibration()
// in setup().
//
// Usage:
//   RadioLink radio(Serial1, SBUS_RX_PIN);
//   radio.begin();
//   radio.setChannelCalibration(1, 200, 822, 1598);  // elevator
//   ...
//   void loop() {
//     radio.update();                     // pulls bytes from UART
//     auto state = radio.getState();
//     if (state.valid) { ... }
//   }
//
// Designed for the Arduino Nano ESP32 (SBUS Serial requires SERIAL_8E2
// + inverted, which is ESP32 HardwareSerial syntax).

class RadioLink {
public:
  static const uint8_t MAX_CHANNELS = 8;

  struct ControllerState {
    // Stick channels mapped to -100..100 (with deadband around 0)
    int16_t aileron;
    int16_t elevator;
    int16_t throttle;
    int16_t rudder;
    int16_t switchA;
    int16_t switchB;
    int16_t knobVRA;
    bool    buttonVRB;

    // Raw SBUS channel values (0..2047)
    uint16_t rawAileron;
    uint16_t rawElevator;
    uint16_t rawThrottle;
    uint16_t rawRudder;
    uint16_t rawSwitchA;
    uint16_t rawButtonVRB;
    uint16_t rawSwitchB;
    uint16_t rawKnobVRA;

    bool valid;  // false if no recent frame, frame_lost, or failsafe active
  };

  RadioLink(HardwareSerial& serial, uint8_t rxPin);

  void begin();   // initializes the SBUS UART
  void update();  // call every loop() — drains the UART, parses frames
  ControllerState getState();

  // Override calibration for a specific channel. Without this, the standard
  // 11-bit SBUS range (172, 992, 1811) is used.
  void setChannelCalibration(uint8_t channel, uint16_t min, uint16_t center, uint16_t max);

  // Helpers
  void printControllerState(const ControllerState& state);
  bool deadmanActive(const ControllerState& state);

  // SBUS defaults
  static const uint16_t SBUS_DEFAULT_MIN    = 172;
  static const uint16_t SBUS_DEFAULT_CENTER = 992;
  static const uint16_t SBUS_DEFAULT_MAX    = 1811;
  static const uint16_t DEADBAND            = 5;     // SBUS counts
  static const unsigned long FRAME_TIMEOUT_MS = 100; // mark invalid if no frame in this window

private:
  struct ChannelCalibration {
    uint16_t min;
    uint16_t center;
    uint16_t max;
  };

  HardwareSerial& _serial;
  uint8_t _rxPin;

  static const uint8_t SBUS_FRAME_SIZE   = 25;
  static const uint8_t SBUS_HEADER_BYTE  = 0x0F;
  static const uint8_t SBUS_FOOTER_BYTE  = 0x00;
  static const uint8_t SBUS_FLAG_FRAME_LOST = 0x04;
  static const uint8_t SBUS_FLAG_FAILSAFE   = 0x08;

  uint8_t  _frame[SBUS_FRAME_SIZE];
  uint8_t  _framePos = 0;
  uint16_t _channels[MAX_CHANNELS];
  ChannelCalibration _calibrations[MAX_CHANNELS];

  unsigned long _lastFrameMs = 0;
  bool _failsafeActive = false;
  bool _frameLost      = false;

  // Apply deadband + per-channel calibration → -100..100
  int16_t mapToCommand(uint8_t channel) const;
};
