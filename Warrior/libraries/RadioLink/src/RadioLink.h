// RadioLink.h
#pragma once
#include <Arduino.h>

class RadioLink {
public:
  static const uint8_t MAX_CHANNELS = 8;

  struct Frame {
    uint16_t PPMReceived[MAX_CHANNELS];
    int16_t mapped[MAX_CHANNELS];
    uint8_t channelCount;
    bool valid;
  };

  struct ControllerState {
    int16_t aileron;
    int16_t elevator;
    int16_t throttle;
    int16_t rudder;

    int16_t switchA;
    int16_t switchB;
    bool buttonVRB;
    int16_t knobVRA;

    uint16_t rawAileron;
    uint16_t rawElevator;
    uint16_t rawThrottle;
    uint16_t rawRudder;
    uint16_t rawSwitchA;
    uint16_t rawButtonVRB;
    uint16_t rawSwitchB;
    uint16_t rawKnobVRA;

    bool valid;
  };

  // Constructor takes the pin number where the PPM signal is connected
  RadioLink(uint8_t pin);

  // Initializes the radio link and starts listening for signals
  void begin();

  // Reads the latest frame of PPM data and returns it as a Frame struct
  Frame readFrame();

  // Converts the raw PPM data into a more user-friendly ControllerState struct
  ControllerState readControllerState();

  // Prints the controller state to the serial monitor for debugging
  void printControllerState(const ControllerState& state);

  // getter function for deadman state
  bool deadmanActive(const ControllerState& state);
  // getter function for throttle
  uint16_t getThrottleMicroseconds(const ControllerState& state);
  // getter function for speed control (knobVRA)
  uint16_t getSpeedControlMicroseconds(const ControllerState& state);

private:
  uint8_t _pin;

  // Singleton instance for interrupt handling
  static RadioLink* _instance;
  // Interrupt handler needs to be static, so we use a singleton pattern
  static void handleInterruptStatic();

  // Minimal class to handle interrupts as fast as possible
  void handleInterrupt();

  volatile uint16_t _channels[MAX_CHANNELS];
  volatile uint8_t _currentChannel = 0;
  volatile bool _frameReady = false;
  volatile uint32_t _lastRiseMicros = 0;

  static const uint16_t SYNC_GAP_US = 3000;
  static const uint16_t MIN_PULSE_US = 800;
  static const uint16_t MAX_PULSE_US = 2200;

  int16_t mapPulse(uint16_t pulse);
};