#include <Arduino.h>
#include <ESP32Servo.h>
#include <SerialProtocol.h>

// Running on Arduino Nano ESP32
// Receives <PWM,spark,flipsky> from Python bridge and drives D6/D7
// IMPORTANT: use raw GPIO numbers, not D6/D7 aliases — ESP32Servo misbehaves with the aliases

const char* DEVICE_NAME = "02_swerve";

const int sparkGPIO   = 9;   // = D6
const int flipskyGPIO = 10;  // = D7

SerialProtocol serialProtocol(DEVICE_NAME);

Servo motor1; // spark
Servo motor2; // flipsky

unsigned long lastPwmMs = 0;

void setup()
{
    serialProtocol.begin(115200);
    motor1.attach(sparkGPIO,   1000, 2000);
    motor2.attach(flipskyGPIO, 1000, 2000);
    motor1.writeMicroseconds(1500);
    motor2.writeMicroseconds(1500);
}

void loop()
{
    serialProtocol.update();

    if (serialProtocol.hasMessage()) {
        handleSerialMessage(serialProtocol.getMessage());
        serialProtocol.clearMessage();
    }

    // Safety: return to neutral if no PWM command received for 500 ms
    if (millis() - lastPwmMs > 500) {
        motor1.writeMicroseconds(1500);
        motor2.writeMicroseconds(1500);
    }
}

void handleSerialMessage(const char* message)
{
    if (strcmp(message, "WHO") == 0) {
        serialProtocol.sendDeviceName();
    } else if (strcmp(message, "PING") == 0) {
        serialProtocol.sendAck("PONG");
    } else if (strncmp(message, "PWM,", 4) == 0) {
        handlePwmMessage(message);
    } else {
        serialProtocol.sendError("unknown_message");
    }
}

void handlePwmMessage(const char* message)
{
    // Format: "PWM,spark,flipsky"  (values in microseconds, 1000–2000)
    int spark = 1500, flipsky = 1500;
    sscanf(message, "PWM,%d,%d", &spark, &flipsky);

    spark   = constrain(spark,   1000, 2000);
    flipsky = constrain(flipsky, 1000, 2000);

    motor1.writeMicroseconds(spark);
    motor2.writeMicroseconds(flipsky);

    lastPwmMs = millis();
    // No ACK sent — bridge does not read responses, sending would fill TX buffer and block
}
