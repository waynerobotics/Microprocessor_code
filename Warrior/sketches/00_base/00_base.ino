#include <Adafruit_NeoPixel.h>
#include <IRremote.hpp>

// =====================================================================
// 00_base — Base Arduino code (NeoPixel ring + IR-toggled light) plus
// FAN_CODE animation refactored into fanSetup() / fanLoop() functions.
//
// Pin conflict resolved: FAN_CODE originally used pin 6, but Base
// Arduino uses pin 6 for IR. Fan LED data line moved to pin 7.
// =====================================================================

// === Base Arduino code (kept as-is) ==================================
#define PIN 5
#define NUMPIXELS 16
#define SIGNAL_PIN 2

#define IR_PIN 6
#define lightPin 3

#define POWER_BUTTON 0xBF40FF00

Adafruit_NeoPixel ring(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

bool lightState = false;

// === FAN code (moved into functions) =================================
#define LED_PIN 7           // moved from 6 to avoid IR_PIN conflict
#define FANS 3
#define LEDS_PER_FAN 20
#define TOTAL_LEDS (FANS * LEDS_PER_FAN)
#define BLOCK_LEN 20
#define FRAME_DELAY_MS 40

Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t GREEN;
uint32_t YELLOW;

void setFanBackground() {
  for (int f = 0; f < FANS; f++) {
    int start = f * LEDS_PER_FAN;
    uint32_t c = (f % 2 == 0) ? GREEN : YELLOW;
    strip.fill(c, start, LEDS_PER_FAN);
  }
}

void fanSetup() {
  strip.begin();
  strip.setBrightness(60);
  strip.show();

  GREEN  = strip.Color(0, 255, 0);
  YELLOW = strip.Color(255, 255, 0);
}

void fanLoop() {
  static int head = 0;

  setFanBackground();

  for (int k = 0; k < BLOCK_LEN; k++) {
    int idx = (head + k) % TOTAL_LEDS;
    int fanIndex = idx / LEDS_PER_FAN;
    uint32_t blockColor = (fanIndex % 2 == 0) ? YELLOW : GREEN;
    strip.setPixelColor(idx, blockColor);
  }

  strip.show();
  delay(FRAME_DELAY_MS);

  head = (head + 1) % TOTAL_LEDS;
}

// === Combined setup / loop ===========================================
void setup() {
  pinMode(SIGNAL_PIN, INPUT_PULLUP);
  pinMode(lightPin, OUTPUT);

  ring.begin();
  ring.show();

  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  fanSetup();
}

void loop() {
  handleNeoPixel();
  handleIR();
  fanLoop();
}

// === Base Arduino functions (kept as-is) =============================
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
