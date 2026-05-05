#include <Arduino.h>

// Running on Arduino Mega2560
// Reads RC PWM via interrupt on pins 2 and 3
// Wire: ESP32 D6 -> Mega pin 2, ESP32 D7 -> Mega pin 3, ESP32 GND -> Mega GND
//
// Reports 0 us if no signal seen for SIGNAL_TIMEOUT_MS (PWM period is 20 ms at 50 Hz)

const int pwmInPinA = 2;
const int pwmInPinB = 3;

const unsigned long SIGNAL_TIMEOUT_MS = 100;

volatile unsigned long riseA = 0;
volatile unsigned long riseB = 0;
volatile unsigned long widthA = 0;
volatile unsigned long widthB = 0;
volatile unsigned long lastEdgeMsA = 0;
volatile unsigned long lastEdgeMsB = 0;

void isrA()
{
    lastEdgeMsA = millis();
    if (digitalRead(pwmInPinA) == HIGH) {
        riseA = micros();
    } else {
        widthA = micros() - riseA;
    }
}

void isrB()
{
    lastEdgeMsB = millis();
    if (digitalRead(pwmInPinB) == HIGH) {
        riseB = micros();
    } else {
        widthB = micros() - riseB;
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(pwmInPinA, INPUT);
    pinMode(pwmInPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(pwmInPinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pwmInPinB), isrB, CHANGE);
    Serial.println("Mega: listening for PWM on pins 2 and 3");
}

void loop()
{
    noInterrupts();
    unsigned long a = widthA;
    unsigned long b = widthB;
    unsigned long edgeA = lastEdgeMsA;
    unsigned long edgeB = lastEdgeMsB;
    interrupts();

    unsigned long now = millis();
    if (now - edgeA > SIGNAL_TIMEOUT_MS) a = 0;
    if (now - edgeB > SIGNAL_TIMEOUT_MS) b = 0;

    Serial.print("P2: ");
    Serial.print(a);
    Serial.print(" us | P3: ");
    Serial.print(b);
    Serial.println(" us");

    delay(100);
}
