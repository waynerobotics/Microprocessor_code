// SerialProtocol.cpp
#include "SerialProtocol.h"

SerialProtocol::SerialProtocol(const char* deviceName)
  : _deviceName(deviceName) {
  resetBuffer();
  _lastMessage[0] = '\0';
}

void SerialProtocol::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
}

void SerialProtocol::update() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '<') {
      resetBuffer();
      _insideMessage = true;
      _messageStartTime = millis();
    }
    else if (c == '>') {
      if (_insideMessage) {
        handleCompletedMessage();
      }

      resetBuffer();
    }
    else {
      if (_insideMessage) {
        if (_bufferIndex < MAX_MESSAGE_LENGTH - 1) {
          _buffer[_bufferIndex++] = c;
          _buffer[_bufferIndex] = '\0';
        } else {
          sendError("buffer_overflow");
          resetBuffer();
        }
      }
    }
  }

  if (_insideMessage && millis() - _messageStartTime > MESSAGE_TIMEOUT_MS) {
    sendError("message_timeout");
    resetBuffer();
  }
}

bool SerialProtocol::hasMessage() const {
  return _messageReady;
}

const char* SerialProtocol::getMessage() const {
  return _lastMessage;
}

void SerialProtocol::clearMessage() {
  _messageReady = false;
  _lastMessage[0] = '\0';
}

void SerialProtocol::sendDeviceName() {
  Serial.print("<NAME,");
  Serial.print(_deviceName);
  Serial.println(">");
}

void SerialProtocol::sendControllerState(
  int aileron,
  int elevator,
  int throttle,
  int rudder,
  int switchA,
  bool buttonVRB,
  int switchB,
  int knobVRA
) {
  Serial.print("<CTRL,");
  Serial.print(aileron);
  Serial.print(",");
  Serial.print(elevator);
  Serial.print(",");
  Serial.print(throttle);
  Serial.print(",");
  Serial.print(rudder);
  Serial.print(",");
  Serial.print(switchA);
  Serial.print(",");
  Serial.print(buttonVRB ? 1 : 0);
  Serial.print(",");
  Serial.print(switchB);
  Serial.print(",");
  Serial.print(knobVRA);
  Serial.println(">");
}

void SerialProtocol::sendFeedback(
  int driveVelocity,
  int steeringVelocity,
  int steeringPosition
) {
  Serial.print("<FBK,");
  Serial.print(driveVelocity);
  Serial.print(",");
  Serial.print(steeringVelocity);
  Serial.print(",");
  Serial.print(steeringPosition);
  Serial.println(">");
}

void SerialProtocol::sendAck(const char* messageType) {
  Serial.print("<ACK,");
  Serial.print(messageType);
  Serial.println(">");
}

void SerialProtocol::sendError(const char* errorMessage) {
  Serial.print("<ERR,");
  Serial.print(errorMessage);
  Serial.println(">");
}

void SerialProtocol::resetBuffer() {
  _bufferIndex = 0;
  _buffer[0] = '\0';
  _insideMessage = false;
}

void SerialProtocol::handleCompletedMessage() {
  _buffer[_bufferIndex] = '\0';

  strncpy(_lastMessage, _buffer, MAX_MESSAGE_LENGTH - 1);
  _lastMessage[MAX_MESSAGE_LENGTH - 1] = '\0';

  _messageReady = true;
}