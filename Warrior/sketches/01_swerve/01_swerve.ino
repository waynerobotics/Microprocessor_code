#include <Arduino.h>
#include <SerialProtocol.h>
#include <MotorControl.h>

// Running on Arduino Nano ESP32
// Receives <DRV,target,drive_percent> from Python bridge. Acts on the message
// only if `target` matches DEVICE_NAME — other swerves' messages are ignored.
// drive_percent is -100..100 (0 = neutral) and is routed to the FlipSky ESC.
// Steering (SPARK MAX) is no longer driven from the Arduino.
// IMPORTANT: use raw GPIO numbers, not Dx aliases — ESP32Servo misbehaves with the aliases

const char* DEVICE_NAME = "02_swerve";
// const char* DEVICE_NAME = "03_swerve";
// const char* DEVICE_NAME = "04_swerve";

const int sparkGPIO   = 5;   // = D2
const int flipskyGPIO = 6;   // = D3

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
    } else if (strncmp(message, "DRV,", 4) == 0) {
        handleDriveMessage(message);
    } else {
        serialProtocol.sendError("unknown_message");
    }
}

void handleDriveMessage(const char* message)
{
    // Format: "DRV,target,drive_percent"  (drive_percent -100..100, FlipSky only)
    char target[24] = {0};
    int drivePercent = 0;
    int parsed = sscanf(message, "DRV,%23[^,],%d", target, &drivePercent);

    // Target parsed but addressed to another swerve: silent ignore (shared bus).
    if (parsed >= 1 && strcmp(target, DEVICE_NAME) != 0) {
        return;
    }
    if (parsed != 2) {
        serialProtocol.sendError("bad_drv_format");
        return;
    }

    if (drivePercent < -100) drivePercent = -100;
    if (drivePercent >  100) drivePercent =  100;
    motors.setFlipskyCommand(drivePercent);

    char ackPayload[40];
    snprintf(ackPayload, sizeof(ackPayload), "DRV,%s", DEVICE_NAME);
    serialProtocol.sendAck(ackPayload);
}
