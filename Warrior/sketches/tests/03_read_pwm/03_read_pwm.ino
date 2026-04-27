#include <ESP32Servo.h>

Servo motor;

const int OUT_PIN = D7;

void setup() {
  Serial.begin(115200);

  motor.attach(OUT_PIN, 1000, 2000);

  Serial.println("PWM output on D7");
}

void loop() {
  motor.writeMicroseconds(1500);  // neutral signal

  Serial.println("Sending 1500us");
  delay(1000);
}