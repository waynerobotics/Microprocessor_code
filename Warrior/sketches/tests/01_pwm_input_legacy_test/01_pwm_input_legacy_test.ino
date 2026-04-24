#include <ESP32Servo.h>
 
Servo output1;   // original drive output
Servo output2;   // extra output controlled by extra channel
 
// ===== INPUT PINS =====
#define channel2pin 8   // throttle / drive input
#define channel6pin 2   // deadman switch
#define channel8pin 7   // speed control
#define channel4pin 4   // extra channel
 
// ===== OUTPUT PINS =====
#define output1Pin 11   // main output
#define output2Pin 12   // extra output
 
// ===== SHARED PULSE VARIABLES =====
volatile unsigned long rise1 = 0, rise2 = 0, rise3 = 0, rise4 = 0;
volatile unsigned long pulse1 = 1500;
volatile unsigned long pulse2 = 1500;
volatile unsigned long pulse3 = 1000;
volatile unsigned long pulse4 = 1500;
 
// ===== OUTPUT LIMITS =====
const int back = 1000;
const int stopVal = 1500;
const int forward = 2000;
 
int drive;
int speedcontrol;
int extraOut;
 
// ===== INTERRUPTS =====
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
 
void IRAM_ATTR isrCh4() {
  if (digitalRead(channel4pin)) {
    rise4 = micros();
  } else {
    pulse4 = micros() - rise4;
  }
}
 
void setup() {
  Serial.begin(115200);
 
  pinMode(channel2pin, INPUT);
  pinMode(channel6pin, INPUT);
  pinMode(channel8pin, INPUT);
  pinMode(channel4pin, INPUT);
 
  output1.attach(output1Pin, back, forward);
  output2.attach(output2Pin, back, forward);
 
  output1.writeMicroseconds(stopVal);
  output2.writeMicroseconds(stopVal);
 
  attachInterrupt(digitalPinToInterrupt(channel2pin), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel8pin), isrCh8, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel6pin), isrCh6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel4pin), isrCh4, CHANGE);
 
  Serial.println("Ready");
}
 
void loop() {
  unsigned long p1, p2, p3, p4;
 
  noInterrupts();
  p1 = pulse1;
  p2 = pulse2;
  p3 = pulse3;
  p4 = pulse4;
  interrupts();
 
  p1 = constrain(p1, 1000, 2000);
  p2 = constrain(p2, 1000, 2000);
  p3 = constrain(p3, 1000, 2000);
  p4 = constrain(p4, 1000, 2000);
 
  // ===== ORIGINAL OUTPUT 1 LOGIC =====
  speedcontrol = map(p2, 1000, 2000, 0, 300);
  drive = map(p1, 1000, 2000, forward - speedcontrol, back + speedcontrol);
  drive = constrain(drive, back, forward);
 
  // ===== EXTRA CHANNEL DIRECTLY CONTROLS OUTPUT 2 =====
  extraOut = p4;
 
  // ===== DEADMAN SWITCH =====
  if (p3 > 1800) {
    output1.writeMicroseconds(drive);
    output2.writeMicroseconds(extraOut);
  } else {
    output1.writeMicroseconds(stopVal);
    output2.writeMicroseconds(stopVal);
  }
 
  Serial.print("p1: ");
  Serial.print(p1);
  Serial.print(" p2: ");
  Serial.print(p2);
  Serial.print(" p3: ");
  Serial.print(p3);
  Serial.print(" p4: ");
  Serial.print(p4);
  Serial.print(" out1: ");
  Serial.print(drive);
  Serial.print(" out2: ");
  Serial.println(extraOut);
 
  delay(20);
}