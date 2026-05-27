#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

// Drives a FlipSky ESC via RC-PWM from a normalized -100..100 command.
// Includes a safety watchdog that returns the motor to neutral if no command
// is received within WATCHDOG_TIMEOUT_MS.
//
// Typical usage:
//   MotorControl motors(flipskyPin);
//   motors.begin();
//   ...
//   motors.setFlipskyCommand(drive);   // when a new command arrives
//   motors.update();                   // every loop()

class MotorControl
{
public:
  MotorControl(uint8_t flipskyPin);

  void begin();

  void setFlipskyCommand(int command); // -100..100

  void stopAll(); // force neutral now (also clears stored command)
  void update();  // call every loop(): applies watchdog + writes output

  int getFlipskyCommand() const;
  int getFlipskyMicroseconds() const;

  static const int CMD_MIN = -100;
  static const int CMD_MAX = 100;
  static const int REVERSE_US = 1000;
  static const int STOP_US = 1500;
  static const int FORWARD_US = 2000;
  static const unsigned long WATCHDOG_TIMEOUT_MS = 500;

private:
  Servo _flipsky;

  uint8_t _flipskyPin;

  int _flipskyCommand = 0;
  int _flipskyMicroseconds = STOP_US;

  unsigned long _lastCommandMs = 0;

  static int commandToMicroseconds(int command);
};
