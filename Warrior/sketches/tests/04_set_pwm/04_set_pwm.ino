#include <ESP32Servo.h>
#include <SerialProtocol.h>

const char* DEVICE_NAME = "02_swerve";

SerialProtocol serialProtocol(DEVICE_NAME);

Servo motor1;
Servo motor2;

unsigned long lastPwmMs = 0;

void setup()
{
    serialProtocol.begin(115200);
    // motor1.attach(D6, 1000, 2000);
    // motor2.attach(D7, 1000, 2000);
    motor1.attach(D4, 1000, 2000);
    motor2.attach(D7, 1000, 2000);
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
