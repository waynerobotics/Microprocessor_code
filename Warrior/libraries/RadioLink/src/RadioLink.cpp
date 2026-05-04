// RadioLink.cpp
#include "RadioLink.h"

RadioLink::RadioLink(HardwareSerial& serial, uint8_t rxPin)
  : _serial(serial), _rxPin(rxPin) {
  for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
    _channels[i] = SBUS_DEFAULT_CENTER;
    _calibrations[i] = { SBUS_DEFAULT_MIN, SBUS_DEFAULT_CENTER, SBUS_DEFAULT_MAX };
  }
}

void RadioLink::begin() {
  // SBUS protocol: 100000 baud, 8 data, even parity, 2 stop bits, inverted.
#if defined(ESP32) || defined(ESP_PLATFORM)
  _serial.begin(100000, SERIAL_8E2, _rxPin, -1, true);
#else
  // AVR / classic Arduino: HardwareSerial::begin can't pick a pin or invert
  // the line. The board needs a hardware inverter on its dedicated RX pin.
  (void)_rxPin;
  _serial.begin(100000, SERIAL_8E2);
#endif
}

void RadioLink::update() {
  // Drain backlog so we always parse the freshest frame
  while (_serial.available() > SBUS_FRAME_SIZE) {
    _serial.read();
  }

  while (_serial.available()) {
    uint8_t b = _serial.read();
    if (_framePos == 0 && b != SBUS_HEADER_BYTE) {
      continue;
    }
    _frame[_framePos++] = b;

    if (_framePos == SBUS_FRAME_SIZE) {
      if (_frame[24] == SBUS_FOOTER_BYTE) {
        _channels[0] = ((_frame[1]      | _frame[2] << 8)                       & 0x07FF);
        _channels[1] = ((_frame[2] >> 3 | _frame[3] << 5)                       & 0x07FF);
        _channels[2] = ((_frame[3] >> 6 | _frame[4] << 2 | _frame[5] << 10)     & 0x07FF);
        _channels[3] = ((_frame[5] >> 1 | _frame[6] << 7)                       & 0x07FF);
        _channels[4] = ((_frame[6] >> 4 | _frame[7] << 4)                       & 0x07FF);
        _channels[5] = ((_frame[7] >> 7 | _frame[8] << 1 | _frame[9] << 9)      & 0x07FF);
        _channels[6] = ((_frame[9] >> 2 | _frame[10] << 6)                      & 0x07FF);
        _channels[7] = ((_frame[10] >> 5 | _frame[11] << 3)                     & 0x07FF);

        _frameLost      = (_frame[23] & SBUS_FLAG_FRAME_LOST) != 0;
        _failsafeActive = (_frame[23] & SBUS_FLAG_FAILSAFE)   != 0;
        _lastFrameMs    = millis();
      }
      _framePos = 0;
    }
  }
}

void RadioLink::setChannelCalibration(uint8_t channel, uint16_t min, uint16_t center, uint16_t max) {
  if (channel < MAX_CHANNELS) {
    _calibrations[channel] = { min, center, max };
  }
}

int16_t RadioLink::mapToCommand(uint8_t channel) const {
  if (channel >= MAX_CHANNELS) return 0;

  uint16_t val = _channels[channel];
  const ChannelCalibration& cal = _calibrations[channel];

  // Deadband around center → snap to zero
  if (val + DEADBAND >= cal.center && val <= cal.center + DEADBAND) {
    return 0;
  }

  long mapped;
  if (val < cal.center) {
    mapped = map((long)val, (long)cal.min, (long)cal.center, -100, 0);
  } else {
    mapped = map((long)val, (long)cal.center, (long)cal.max, 0, 100);
  }
  return (int16_t)constrain(mapped, -100L, 100L);
}

RadioLink::ControllerState RadioLink::getState() {
  ControllerState state = {};

  // RadioLink R8EF / T8S channel mapping:
  // CH1 = Aileron, CH2 = Elevator, CH3 = Throttle, CH4 = Rudder
  // CH5 = SWA,     CH6 = VRB pushbutton/deadman,
  // CH7 = SWB,     CH8 = VRA knob

  state.aileron  = mapToCommand(0);
  state.elevator = mapToCommand(1);
  state.throttle = mapToCommand(2);
  state.rudder   = mapToCommand(3);
  state.switchA  = mapToCommand(4);
  state.switchB  = mapToCommand(6);
  state.knobVRA  = mapToCommand(7);

  state.rawAileron   = _channels[0];
  state.rawElevator  = _channels[1];
  state.rawThrottle  = _channels[2];
  state.rawRudder    = _channels[3];
  state.rawSwitchA   = _channels[4];
  state.rawButtonVRB = _channels[5];
  state.rawSwitchB   = _channels[6];
  state.rawKnobVRA   = _channels[7];

  state.buttonVRB = state.rawButtonVRB > (SBUS_DEFAULT_CENTER + 200);

  bool fresh = (millis() - _lastFrameMs) <= FRAME_TIMEOUT_MS;
  state.valid = fresh && !_failsafeActive && !_frameLost && _lastFrameMs != 0;

  return state;
}

bool RadioLink::deadmanActive(const ControllerState& state) {
  return state.buttonVRB;
}

void RadioLink::printControllerState(const ControllerState& state) {
  if (!state.valid) {
    Serial.println("Controller state invalid");
    return;
  }

  Serial.print("aileron=");   Serial.print(state.aileron);
  Serial.print(" (");         Serial.print(state.rawAileron);   Serial.print(")");
  Serial.print(", elevator="); Serial.print(state.elevator);
  Serial.print(" (");         Serial.print(state.rawElevator);  Serial.print(")");
  Serial.print(", throttle="); Serial.print(state.throttle);
  Serial.print(" (");         Serial.print(state.rawThrottle);  Serial.print(")");
  Serial.print(", rudder=");   Serial.print(state.rudder);
  Serial.print(" (");         Serial.print(state.rawRudder);    Serial.print(")");
  Serial.print(", swA=");      Serial.print(state.switchA);
  Serial.print(", swB=");      Serial.print(state.switchB);
  Serial.print(", btnVRB=");   Serial.print(state.buttonVRB);
  Serial.print(", knobVRA=");  Serial.println(state.knobVRA);
}
