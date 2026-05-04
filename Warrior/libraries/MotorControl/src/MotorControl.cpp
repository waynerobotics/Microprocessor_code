#include "MotorControl.h"

MotorControl::MotorControl(uint8_t sparkPin, uint8_t flipskyPin)
  : _sparkPin(sparkPin),
    _flipskyPin(flipskyPin) {}

void MotorControl::begin() {
  _spark.attach(_sparkPin,     REVERSE_US, FORWARD_US);
  _flipsky.attach(_flipskyPin, REVERSE_US, FORWARD_US);
  stopAll();
}

void MotorControl::setSparkCommand(int command) {
  _sparkCommand = constrain(command, CMD_MIN, CMD_MAX);
  _sparkMicroseconds = commandToMicroseconds(_sparkCommand);
  _lastCommandMs = millis();
}

void MotorControl::setFlipskyCommand(int command) {
  _flipskyCommand = constrain(command, CMD_MIN, CMD_MAX);
  _flipskyMicroseconds = commandToMicroseconds(_flipskyCommand);
  _lastCommandMs = millis();
}

void MotorControl::setCommands(int sparkCommand, int flipskyCommand) {
  _sparkCommand        = constrain(sparkCommand,   CMD_MIN, CMD_MAX);
  _flipskyCommand      = constrain(flipskyCommand, CMD_MIN, CMD_MAX);
  _sparkMicroseconds   = commandToMicroseconds(_sparkCommand);
  _flipskyMicroseconds = commandToMicroseconds(_flipskyCommand);
  _lastCommandMs = millis();
}

void MotorControl::stopAll() {
  _sparkCommand        = 0;
  _flipskyCommand      = 0;
  _sparkMicroseconds   = STOP_US;
  _flipskyMicroseconds = STOP_US;
  _spark.writeMicroseconds(STOP_US);
  _flipsky.writeMicroseconds(STOP_US);
}

void MotorControl::update() {
  if (millis() - _lastCommandMs > WATCHDOG_TIMEOUT_MS) {
    _sparkMicroseconds   = STOP_US;
    _flipskyMicroseconds = STOP_US;
  }
  _spark.writeMicroseconds(_sparkMicroseconds);
  _flipsky.writeMicroseconds(_flipskyMicroseconds);
}

int MotorControl::getSparkCommand() const       { return _sparkCommand; }
int MotorControl::getFlipskyCommand() const     { return _flipskyCommand; }
int MotorControl::getSparkMicroseconds() const  { return _sparkMicroseconds; }
int MotorControl::getFlipskyMicroseconds() const { return _flipskyMicroseconds; }

int MotorControl::commandToMicroseconds(int command) {
  command = constrain(command, CMD_MIN, CMD_MAX);
  // -100 -> REVERSE_US (1000)
  //    0 -> STOP_US    (1500)
  //  100 -> FORWARD_US (2000)
  return map(command, CMD_MIN, CMD_MAX, REVERSE_US, FORWARD_US);
}
