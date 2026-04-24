#include "MotorControl.h"

MotorControl::MotorControl(uint8_t flipskyPin, uint8_t sparkMaxPin)
  : _flipskyPin(flipskyPin),
    _sparkMaxPin(sparkMaxPin) {}

void MotorControl::begin() {
  _flipsky.attach(_flipskyPin, REVERSE_US, FORWARD_US);
  _sparkMax.attach(_sparkMaxPin, REVERSE_US, FORWARD_US);

  stopAll();
}

void MotorControl::setDriveCommand(int command) {
  _driveCommand = constrain(command, -100, 100);
  _driveMicroseconds = commandToMicroseconds(_driveCommand);
}

void MotorControl::setSteeringCommand(int command) {
  _steeringCommand = constrain(command, -100, 100);
  _steeringMicroseconds = commandToMicroseconds(_steeringCommand);
}

void MotorControl::stopAll() {
  _driveCommand = 0;
  _steeringCommand = 0;

  _driveMicroseconds = STOP_US;
  _steeringMicroseconds = STOP_US;

  _flipsky.writeMicroseconds(STOP_US);
  _sparkMax.writeMicroseconds(STOP_US);
}

void MotorControl::writeOutputs() {
  _flipsky.writeMicroseconds(_driveMicroseconds);
  _sparkMax.writeMicroseconds(_steeringMicroseconds);
}

int MotorControl::getDriveMicroseconds() const {
  return _driveMicroseconds;
}

int MotorControl::getSteeringMicroseconds() const {
  return _steeringMicroseconds;
}

int MotorControl::commandToMicroseconds(int command) {
  command = constrain(command, -100, 100);

  // -100 -> 1000 us
  //    0 -> 1500 us
  //  100 -> 2000 us
  return map(command, -100, 100, REVERSE_US, FORWARD_US);
}