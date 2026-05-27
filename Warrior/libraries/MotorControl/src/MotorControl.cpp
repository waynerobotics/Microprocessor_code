#include "MotorControl.h"

MotorControl::MotorControl(uint8_t flipskyPin)
  : _flipskyPin(flipskyPin) {}

void MotorControl::begin() {
  _flipsky.attach(_flipskyPin, REVERSE_US, FORWARD_US);
  stopAll();
}

void MotorControl::setFlipskyCommand(int command) {
  _flipskyCommand      = constrain(command, CMD_MIN, CMD_MAX);
  _flipskyMicroseconds = commandToMicroseconds(_flipskyCommand);
  _lastCommandMs = millis();
}

void MotorControl::stopAll() {
  _flipskyCommand      = 0;
  _flipskyMicroseconds = STOP_US;
  _flipsky.writeMicroseconds(STOP_US);
}

void MotorControl::update() {
  if (millis() - _lastCommandMs > WATCHDOG_TIMEOUT_MS) {
    _flipskyMicroseconds = STOP_US;
  }
  _flipsky.writeMicroseconds(_flipskyMicroseconds);
}

int MotorControl::getFlipskyCommand() const      { return _flipskyCommand; }
int MotorControl::getFlipskyMicroseconds() const { return _flipskyMicroseconds; }

int MotorControl::commandToMicroseconds(int command) {
  command = constrain(command, CMD_MIN, CMD_MAX);
  // -100 -> REVERSE_US (1000)
  //    0 -> STOP_US    (1500)
  //  100 -> FORWARD_US (2000)
  return map(command, CMD_MIN, CMD_MAX, REVERSE_US, FORWARD_US);
}
