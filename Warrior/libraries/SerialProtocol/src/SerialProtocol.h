// SerialProtocol.h
#pragma once
#include <Arduino.h>

class SerialProtocol {
public:
  static const uint8_t MAX_MESSAGE_LENGTH = 96;

  SerialProtocol(const char* deviceName);

  void begin(unsigned long baudRate = 115200);
  void update();

  bool hasMessage() const;
  const char* getMessage() const;
  void clearMessage();

  void sendDeviceName();
  void sendControllerState(
    int aileron,
    int elevator,
    int throttle,
    int rudder,
    int switchA,
    bool buttonVRB,
    int switchB,
    int knobVRA
  );

  void sendFeedback(
    int driveVelocity,
    int steeringVelocity,
    int steeringPosition
  );

  void sendAck(const char* messageType);
  void sendError(const char* errorMessage);

private:
  const char* _deviceName;

  char _buffer[MAX_MESSAGE_LENGTH];
  char _lastMessage[MAX_MESSAGE_LENGTH];

  uint8_t _bufferIndex = 0;
  bool _insideMessage = false;
  bool _messageReady = false;

  unsigned long _messageStartTime = 0;

  static const unsigned long MESSAGE_TIMEOUT_MS = 100;

  void resetBuffer();
  void handleCompletedMessage();
};