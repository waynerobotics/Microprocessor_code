#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class MotorControl {
public:
  MotorControl(uint8_t flipskyPin, uint8_t sparkMaxPin);

  void begin();

  void setDriveCommand(int command);      // -100 to +100
  void setSteeringCommand(int command);   // -100 to +100 for now

  void stopAll();

  void writeOutputs();

  int getDriveMicroseconds() const;
  int getSteeringMicroseconds() const;

private:
  Servo _flipsky;
  Servo _sparkMax;

  uint8_t _flipskyPin;
  uint8_t _sparkMaxPin;

  int _driveCommand = 0;
  int _steeringCommand = 0;

  int _driveMicroseconds = 1500;
  int _steeringMicroseconds = 1500;

  static const int REVERSE_US = 1000;
  static const int STOP_US = 1500;
  static const int FORWARD_US = 2000;

  int commandToMicroseconds(int command);
};