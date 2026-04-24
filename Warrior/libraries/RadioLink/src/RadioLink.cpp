// RadioLink.cpp
#include "RadioLink.h"

// This is the single part of the class, so we need to define the static instance pointer here
RadioLink* RadioLink::_instance = nullptr;

// Constructor takes the pin number where the PPM signal is connected
RadioLink::RadioLink(uint8_t pin) : _pin(pin) {}

void RadioLink::begin() {
  _instance = this;

  pinMode(_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pin), handleInterruptStatic, RISING);
}

void RadioLink::handleInterruptStatic() {
  if (_instance != nullptr) {
    _instance->handleInterrupt();
  }
}

void RadioLink::handleInterrupt() {
  uint32_t now = micros();
  uint32_t gap = now - _lastRiseMicros;
  _lastRiseMicros = now;

  if (gap > SYNC_GAP_US) {
    _currentChannel = 0;
    _frameReady = true;
    return;
  }

  if (_currentChannel < MAX_CHANNELS) {
    _channels[_currentChannel] = gap;
    _currentChannel++;
  }
}

RadioLink::Frame RadioLink::readFrame() {
  Frame frame;
  frame.channelCount = MAX_CHANNELS;
  frame.valid = false;

  noInterrupts();

  bool ready = _frameReady;
  _frameReady = false;

  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    frame.PPMReceived[i] = _channels[i];
  }

  interrupts();

  if (!ready) {
    return frame;
  }

  frame.valid = true;

  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    if (frame.PPMReceived[i] < MIN_PULSE_US || frame.PPMReceived[i] > MAX_PULSE_US) {
      frame.valid = false;
    }

    frame.mapped[i] = mapPulse(frame.PPMReceived[i]);
  }

  return frame;
}

RadioLink::ControllerState RadioLink::readControllerState() {
  Frame frame = readFrame();

  ControllerState state = {};
  state.valid = frame.valid;

  if (!frame.valid) {
    return state;
  }

  // RadioLink R8EF / T8S channel mapping:
  // CH1 = Aileron
  // CH2 = Elevator
  // CH3 = Throttle
  // CH4 = Rudder
  // CH5 = SWA
  // CH6 = VRB push button / deadman
  // CH7 = SWB
  // CH8 = VRA knob / speed control

  state.aileron  = frame.mapped[0];
  state.elevator = frame.mapped[1];
  state.throttle = frame.mapped[2];
  state.rudder   = frame.mapped[3];

  state.switchA   = frame.mapped[4];
  state.buttonVRB = frame.mapped[5] > 50;
  state.switchB   = frame.mapped[6];
  state.knobVRA   = frame.mapped[7];

  state.rawAileron   = frame.PPMReceived[0];
  state.rawElevator  = frame.PPMReceived[1];
  state.rawThrottle  = frame.PPMReceived[2];
  state.rawRudder    = frame.PPMReceived[3];
  state.rawSwitchA   = frame.PPMReceived[4];
  state.rawButtonVRB = frame.PPMReceived[5];
  state.rawSwitchB   = frame.PPMReceived[6];
  state.rawKnobVRA   = frame.PPMReceived[7];

  return state;
}

void RadioLink::printControllerState(const ControllerState& state) {
  if (!state.valid) {
    Serial.println("Controller state invalid");
    return;
  }

  Serial.print("aileron=");
  Serial.print(state.aileron);
  Serial.print(" (");
  Serial.print(state.rawAileron);
  Serial.print("us)");

  Serial.print(", elevator=");
  Serial.print(state.elevator);
  Serial.print(" (");
  Serial.print(state.rawElevator);
  Serial.print("us)");

  Serial.print(", throttle=");
  Serial.print(state.throttle);
  Serial.print(" (");
  Serial.print(state.rawThrottle);
  Serial.print("us)");

  Serial.print(", rudder=");
  Serial.print(state.rudder);
  Serial.print(" (");
  Serial.print(state.rawRudder);
  Serial.print("us)");

  Serial.print(", switchA=");
  Serial.print(state.switchA);
  Serial.print(" (");
  Serial.print(state.rawSwitchA);
  Serial.print("us)");

  Serial.print(", buttonVRB=");
  Serial.print(state.buttonVRB);
  Serial.print(" (");
  Serial.print(state.rawButtonVRB);
  Serial.print("us)");

  Serial.print(", switchB=");
  Serial.print(state.switchB);
  Serial.print(" (");
  Serial.print(state.rawSwitchB);
  Serial.print("us)");

  Serial.print(", knobVRA=");
  Serial.print(state.knobVRA);
  Serial.print(" (");
  Serial.print(state.rawKnobVRA);
  Serial.println("us)");
}

bool RadioLink::deadmanActive(const ControllerState& state) {
  return state.rawButtonVRB > 1800;
}

uint16_t RadioLink::getThrottleMicroseconds(const ControllerState& state) {
  return state.rawThrottle;
}

uint16_t RadioLink::getSpeedControlMicroseconds(const ControllerState& state) {
  return state.rawKnobVRA;
}

int16_t RadioLink::mapPulse(uint16_t pulse) {
  pulse = constrain(pulse, 1000, 2000);
  return map(pulse, 1000, 2000, -100, 100);
}