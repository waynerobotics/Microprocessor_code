#include <ESP32Servo.h>

Servo motor1;
Servo motor2;

void setup()
{
    motor1.attach(D6, 1000, 2000);
    motor2.attach(D7, 1000, 2000);
}

void loop()
{
    motor1.writeMicroseconds(1500);
    motor2.writeMicroseconds(1500);
}
