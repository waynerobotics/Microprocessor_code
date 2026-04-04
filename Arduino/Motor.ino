#include <ESP32Servo.h>
 
Servo motor;
 
#define channel2pin 8   // throttle
#define channel6pin 2   // deadman
#define channel8pin 7   // speed control
#define drivepin      // output to SPARK MAX
 
volatile unsigned long rise1 = 0, rise2 = 0, rise3 = 0;
volatile unsigned long pulse1 = 1500, pulse2 = 1500, pulse3 = 1000;
 
const int back = 1000;
const int stop = 1500;
const int forward = 2000;
 
int drive;
int speedcontrol;
 
void IRAM_ATTR isrCh2() {
  if (digitalRead(channel2pin)) {
    rise1 = micros();
  } else {
    pulse1 = micros() - rise1;
  }
}
 
void IRAM_ATTR isrCh8() {
  if (digitalRead(channel8pin)) {
    rise2 = micros();
  } else {
    pulse2 = micros() - rise2;
  }
}
 
void IRAM_ATTR isrCh6() {
  if (digitalRead(channel6pin)) {
    rise3 = micros();
  } else {
    pulse3 = micros() - rise3;
  }
}
 
void setup() {
  Serial.begin(9600);
 
  pinMode(channel2pin, INPUT);
  pinMode(channel6pin, INPUT);
  pinMode(channel8pin, INPUT);
 
  motor.attach(drivepin, back, forward);
  motor.writeMicroseconds(stop);
 
  attachInterrupt(digitalPinToInterrupt(channel2pin), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel8pin), isrCh8, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel6pin), isrCh6, CHANGE);
 
  delay(1000);
  Serial.println("working here");
}
 
void loop() {
  unsigned long p1, p2, p3;
 
  noInterrupts();
  p1 = pulse1;
  p2 = pulse2;
  p3 = pulse3;
  interrupts();
 
  p1 = constrain(p1, 1000, 2000);
  p2 = constrain(p2, 1000, 2000);
  p3 = constrain(p3, 1000, 2000);
 
  speedcontrol = map(p2, 1000, 2000, 0, 300);
  drive = map(p1, 1000, 2000, forward - speedcontrol, back + speedcontrol);
  drive = constrain(drive, back, forward);
 
  if (p3 > 1800) {
    motor.writeMicroseconds(drive);
  } else {
    motor.writeMicroseconds(stop);
  }
 
  Serial.print("p1: ");
  Serial.print(p1);
  Serial.print("  p2: ");
  Serial.print(p2);
  Serial.print("  p3: ");
  Serial.print(p3);
  Serial.print("  drive: ");
  Serial.println(drive);
 
  delay(20);
}