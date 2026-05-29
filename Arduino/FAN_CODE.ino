#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 7

#define FANS 3
#define LEDS_PER_FAN 20
#define TOTAL_LEDS (FANS * LEDS_PER_FAN)
#define BLOCK_LEN 20
#define FRAME_DELAY_MS 10

#define LOOPS_BEFORE_FLASH 7
#define FLASH_COUNT 3
#define FLASH_ON_MS 150
#define FLASH_OFF_MS 150

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

void doFlash() {
  for (int i = 0; i < FLASH_COUNT; i++) {
    // ON
    setFanBackground();
    strip.show();
    delay(FLASH_ON_MS);

    // OFF
    strip.clear();
    strip.show();
    delay(FLASH_OFF_MS);
  }
}

void setup() {
  strip.begin();
  strip.setBrightness(60);
  strip.show();

  GREEN  = strip.Color(0, 255, 0);
  YELLOW = strip.Color(255, 255, 0);
}

void loop() {
  static int head = 0;
  static int loopCount = 0;

  // Animation frame
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

  // Detect a completed loop (head wrapped back to 0)
  if (head == 0) {
    loopCount++;
    if (loopCount >= LOOPS_BEFORE_FLASH) {
      loopCount = 0;
      doFlash();
    }
  }
}