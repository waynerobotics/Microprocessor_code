#include <Arduino.h>
#include <SerialProtocol.h>
#include <MotorControl.h>

// Running on Arduino Nano ESP32
// Receives <MOT,target,spark,flipsky> from Python bridge. Acts on the message
// only if `target` matches DEVICE_NAME — other swerves' messages are ignored.
// Values are normalized -100..100, where 0 = neutral. Drives D6/D7 via the
// MotorControl library.
// IMPORTANT: use raw GPIO numbers, not D6/D7 aliases — ESP32Servo misbehaves with the aliases

const char* DEVICE_NAME = "02_swerve";

const int sparkGPIO   = 9;   // = D6
const int flipskyGPIO = 10;  // = D7

SerialProtocol serialProtocol(DEVICE_NAME);
MotorControl motors(sparkGPIO, flipskyGPIO);

void setup()
{
    serialProtocol.begin(115200);
    motors.begin();
}

void loop()
{
    serialProtocol.update();

    if (serialProtocol.hasMessage()) {
        handleSerialMessage(serialProtocol.getMessage());
        serialProtocol.clearMessage();
    }

    // Applies the watchdog (return to neutral after WATCHDOG_TIMEOUT_MS without commands)
    // and writes the current commanded microseconds to both ESCs.
    motors.update();
}

void handleSerialMessage(const char* message)
{
    if (strcmp(message, "WHO") == 0) {
        serialProtocol.sendDeviceName();
    } else if (strcmp(message, "PING") == 0) {
        serialProtocol.sendAck("PONG");
    } else if (strncmp(message, "MOT,", 4) == 0) {
        handleMotorMessage(message);
    } else {
        serialProtocol.sendError("unknown_message");
    }
}

void handleMotorMessage(const char* message)
{
    // Format: "MOT,target,spark,flipsky"  (values normalized -100..100)
    // Ignore commands not addressed to this device.
    char target[24] = {0};
    int spark = 0, flipsky = 0;
    if (sscanf(message, "MOT,%23[^,],%d,%d", target, &spark, &flipsky) != 3) {
        return;
    }
    if (strcmp(target, DEVICE_NAME) != 0) {
        return;
    }
    motors.setCommands(spark, flipsky);
    // No ACK sent — bridge does not read responses, sending would fill TX buffer and block
}
