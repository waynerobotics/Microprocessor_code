#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

// Drives two RC-PWM motor controllers (sparkMax + flipsky) from normalized
// -100..100 commands. Includes a safety watchdog that returns motors to
// neutral if no command is received within WATCHDOG_TIMEOUT_MS.
//
// Typical usage:
//   MotorControl motors(sparkPin, flipskyPin);
//   motors.begin();
//   ...
//   motors.setCommands(spark, flipsky);   // when a new command arrives
//   motors.update();                      // every loop()

class MotorControl {
public:
  MotorControl(uint8_t sparkPin, uint8_t flipskyPin);

  void begin();

  void setSparkCommand(int command);                      // -100..100
  void setFlipskyCommand(int command);                    // -100..100
  void setCommands(int sparkCommand, int flipskyCommand); // both at once

  void stopAll();    // force neutral now (also clears stored commands)
  void update();     // call every loop(): applies watchdog + writes outputs

  int getSparkCommand() const;
  int getFlipskyCommand() const;
  int getSparkMicroseconds() const;
  int getFlipskyMicroseconds() const;

  static const int CMD_MIN = -100;
  static const int CMD_MAX =  100;
  static const int REVERSE_US = 1000;
  static const int STOP_US    = 1500;
  static const int FORWARD_US = 2000;
  static const unsigned long WATCHDOG_TIMEOUT_MS = 500;

private:
  Servo _spark;
  Servo _flipsky;

  uint8_t _sparkPin;
  uint8_t _flipskyPin;

  int _sparkCommand   = 0;
  int _flipskyCommand = 0;

  int _sparkMicroseconds   = STOP_US;
  int _flipskyMicroseconds = STOP_US;

  unsigned long _lastCommandMs = 0;

  static int commandToMicroseconds(int command);
};
