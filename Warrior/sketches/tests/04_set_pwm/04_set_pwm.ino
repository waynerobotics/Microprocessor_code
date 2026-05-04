#include <Arduino.h>
#include <ESP32Servo.h>
#include <SerialProtocol.h>

// Running on Arduino Nano ESP32
// Receives <MOT,spark,flipsky> from Python bridge (values are -100..100,
// where -100 = 1000us, 0 = 1500us neutral, 100 = 2000us) and drives D6/D7.
// IMPORTANT: use raw GPIO numbers, not D6/D7 aliases — ESP32Servo misbehaves with the aliases

const char* DEVICE_NAME = "02_swerve";

const int sparkGPIO   = 9;   // = D6
const int flipskyGPIO = 10;  // = D7

const int CMD_MIN = -100;
const int CMD_MAX =  100;
const int PWM_MIN_US = 1000;
const int PWM_NEUTRAL_US = 1500;
const int PWM_MAX_US = 2000;

SerialProtocol serialProtocol(DEVICE_NAME);

Servo motor1; // spark
Servo motor2; // flipsky

unsigned long lastMotorMs = 0;

int commandToMicroseconds(int cmd)
{
    cmd = constrain(cmd, CMD_MIN, CMD_MAX);
    return map(cmd, CMD_MIN, CMD_MAX, PWM_MIN_US, PWM_MAX_US);
}

void setup()
{
    serialProtocol.begin(115200);
    motor1.attach(sparkGPIO,   PWM_MIN_US, PWM_MAX_US);
    motor2.attach(flipskyGPIO, PWM_MIN_US, PWM_MAX_US);
    motor1.writeMicroseconds(PWM_NEUTRAL_US);
    motor2.writeMicroseconds(PWM_NEUTRAL_US);
}

void loop()
{
    serialProtocol.update();

    if (serialProtocol.hasMessage()) {
        handleSerialMessage(serialProtocol.getMessage());
        serialProtocol.clearMessage();
    }

    // Safety: return to neutral if no motor command received for 500 ms
    if (millis() - lastMotorMs > 500) {
        motor1.writeMicroseconds(PWM_NEUTRAL_US);
        motor2.writeMicroseconds(PWM_NEUTRAL_US);
    }
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
    // Format: "MOT,spark,flipsky"  (values normalized -100..100)
    int spark = 0, flipsky = 0;
    sscanf(message, "MOT,%d,%d", &spark, &flipsky);

    motor1.writeMicroseconds(commandToMicroseconds(spark));
    motor2.writeMicroseconds(commandToMicroseconds(flipsky));

    lastMotorMs = millis();
    // No ACK sent — bridge does not read responses, sending would fill TX buffer and block
}
