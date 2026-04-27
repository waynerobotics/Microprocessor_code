#include <Arduino.h>

const int pwmInPin4 = 4;
const int pwmInPin7 = 7;

void setup()
{
    // Start serial at high speed for responsive logging
    Serial.begin(115200);

    pinMode(pwmInPin4, INPUT);
    pinMode(pwmInPin7, INPUT);

    Serial.println("Listening for PWM on D4 and D7...");
}

void loop()
{
    unsigned long duration4 = pulseIn(pwmInPin4, HIGH, 30000);
    unsigned long duration7 = pulseIn(pwmInPin7, HIGH, 30000);

    Serial.print("D4: ");
    Serial.print(duration4);
    Serial.print(" us | D7: ");
    Serial.print(duration7);
    Serial.println(" us");

    // Small delay to keep the Serial Monitor readable
    delay(100);
}