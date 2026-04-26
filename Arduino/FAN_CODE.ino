#include <Adafruit_NeoPixel.h>

// Data Pin
#define LED_PIN 6

// Setup
#define FANS 3
#define LEDS_PER_FAN 20
#define TOTAL_LEDS (FANS * LEDS_PER_FAN)   
#define BLOCK_LEN 20                       // moving segment length
#define FRAME_DELAY_MS 40


Adafruit_NeoPixel strip(TOTAL_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t GREEN;
uint32_t YELLOW;

void setFanBackground() {
  // Fan 0: green, Fan 1: yellow, Fan 2: green
  for (int f = 0; f < FANS; f++) {
    int start = f * LEDS_PER_FAN;
    uint32_t c = (f % 2 == 0) ? GREEN : YELLOW; // 0->G, 1->Y, 2->G
    strip.fill(c, start, LEDS_PER_FAN);
  }
}

void setup() {
  strip.begin();
  strip.setBrightness(60); // keep it low for safety on multi-fan setups
  strip.show();

  GREEN  = strip.Color(0, 255, 0);
  YELLOW = strip.Color(255, 255, 0);
}

void loop() {
  static int head = 0;

  // 1) Background: fan-by-fan colors
  setFanBackground();

  // 2) Moving block across ALL fans (overrides background)
  // We'll use the opposite color of whatever fan segment it's currently in.
  for (int k = 0; k < BLOCK_LEN; k++) {
    int idx = (head + k) % TOTAL_LEDS;

    int fanIndex = idx / LEDS_PER_FAN;
    uint32_t blockColor = (fanIndex % 2 == 0) ? YELLOW : GREEN; // opposite of background

    strip.setPixelColor(idx, blockColor);
  }

  strip.show();
  delay(FRAME_DELAY_MS);

  head = (head + 1) % TOTAL_LEDS;
}
