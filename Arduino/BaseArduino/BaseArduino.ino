#include <Adafruit_NeoPixel.h>
#include <IRremote.hpp>
 
#define PIN 5
#define NUMPIXELS 16
#define SIGNAL_PIN 2
 
#define IR_PIN 6
#define lightPin 3
 
#define POWER_BUTTON 0xBF40FF00
 
Adafruit_NeoPixel ring(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
bool lightState = false;
 
void setup() {
  pinMode(SIGNAL_PIN, INPUT_PULLUP);
  pinMode(lightPin, OUTPUT);
 
  ring.begin();
  ring.show();
 
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
}
 
void loop() {
  handleNeoPixel();
  handleIR();
}
 
void handleNeoPixel() {
  bool signalState = digitalRead(SIGNAL_PIN);
 
  if (signalState == HIGH) {
    setColor(255, 255, 0);
  } else {
    flashRed();
  }
}
 
void handleIR() {
  if (IrReceiver.decode()) {
    unsigned long code = IrReceiver.decodedIRData.decodedRawData;
 
    if (code == POWER_BUTTON) {
      lightState = !lightState;
    }
 
    digitalWrite(lightPin, lightState ? HIGH : LOW);
 
    IrReceiver.resume();
  }
}
 
void setColor(int r, int g, int b) {
  for (int i = 0; i < NUMPIXELS; i++) {
    ring.setPixelColor(i, ring.Color(r, g, b));
  }
  ring.show();
}
 
void flashRed() {
  setColor(255, 0, 0);
  delay(300);
  setColor(0, 0, 0);
  delay(300);
}
